#!/usr/bin/env python3
"""Validate the actual ROS 2 EGO planner using local synthetic inputs only.

Start the planner separately in the same isolated ROS_DOMAIN_ID with
ROS_LOCALHOST_ONLY=1. This script publishes 50 Hz ideal odometry, a 2 Hz
12457-point fixed floor/box cloud, alternating goals, and optionally three
stationary peer trajectories at 50 Hz each. It follows the planner's actual
cubic B-spline and derivative. It never opens a camera, serial port, flight
controller, or MAVLink connection, and never arms a vehicle.

Example (after sourcing Humble and the native EGO workspace):
  python3 validate_ego_ros2.py --domain-id 66 --duration 30 --output new.json

The measurement begins after the first valid planner trajectory and --warmup.
This is an interface/planning validation with an ideal follower, not a dynamics,
localization, wireless-link, or flight test.
"""

import argparse
import array
import datetime as dt
import json
import math
import os
from pathlib import Path
import resource
import sys
import time


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return number


def de_boor(points, degree, knots, u):
    """Use the same span convention and endpoint clamping as UniformBspline."""
    u = min(max(u, knots[degree]), knots[len(points)])
    span = degree
    while span < len(points) - 1 and knots[span + 1] < u:
        span += 1
    values = [list(point) for point in points[span - degree:span + 1]]
    for level in range(1, degree + 1):
        for index in range(degree, level - 1, -1):
            left = knots[index + span - degree]
            denominator = knots[index + 1 + span - level] - left
            alpha = (u - left) / denominator if denominator else 0.0
            values[index] = [(1 - alpha) * a + alpha * b
                             for a, b in zip(values[index - 1], values[index])]
    return values[degree]


class Trajectory:
    def __init__(self, message):
        self.degree = int(message.order)
        self.points = [(point.x, point.y, point.z) for point in message.pos_pts]
        self.knots = list(message.knots)
        self.start = message.start_time.sec + message.start_time.nanosec * 1e-9
        if (self.degree != 3 or len(self.points) <= self.degree or
                len(self.knots) != len(self.points) + self.degree + 1 or
                not all(math.isfinite(value) for point in self.points for value in point) or
                not all(math.isfinite(value) for value in self.knots + [self.start]) or
                any(a > b for a, b in zip(self.knots, self.knots[1:]))):
            raise ValueError("malformed cubic trajectory")
        self.low = self.knots[self.degree]
        self.duration = self.knots[len(self.points)] - self.low
        if self.duration <= 0:
            raise ValueError("nonpositive trajectory duration")
        self.velocity_points = []
        for index, (a, b) in enumerate(zip(self.points, self.points[1:])):
            denominator = self.knots[index + self.degree + 1] - self.knots[index + 1]
            if denominator <= 0:
                raise ValueError("invalid derivative knot span")
            derivative = [self.degree * (y - x) / denominator for x, y in zip(a, b)]
            if not all(math.isfinite(value) for value in derivative):
                raise ValueError("non-finite trajectory derivative")
            self.velocity_points.append(derivative)

    def evaluate(self, stamp):
        elapsed = min(max(stamp - self.start, 0.0), self.duration)
        u = self.low + elapsed
        position = de_boor(self.points, self.degree, self.knots, u)
        velocity = (de_boor(self.velocity_points, self.degree - 1, self.knots[1:-1], u)
                    if 0 <= stamp - self.start < self.duration else [0.0] * 3)
        if not all(math.isfinite(value) for value in position + velocity):
            raise ValueError("non-finite evaluated trajectory")
        return position, velocity


def fixed_cloud_points():
    # Cell centers fill all XY voxels of the 10 x 10 m map without boundary gaps.
    floor = [((x + 0.5) / 10.0, (y + 0.5) / 10.0, 0.0)
             for x in range(-50, 50) for y in range(-50, 50)]
    # A filled box blocks the straight (-3,0,1) -> (3,0,1) route.
    obstacle = [(x / 10.0, y / 10.0, z / 10.0)
                for x in range(-4, 5) for y in range(-6, 7) for z in range(1, 22)]
    return floor + obstacle


def create_validation_node(args):
    # Import ROS only after main has selected the isolated, loopback-only domain.
    from geometry_msgs.msg import Point, PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2, PointField
    from traj_utils.msg import Bspline

    class ValidationNode(Node):
        def __init__(self):
            super().__init__("rk3566_ego_validation_" + str(os.getpid()))
            self.started = time.monotonic()
            self.status, self.reason = "running", ""
            self.trajectory = None
            self.first_trajectory = self.measurement_start = self.measurement_end = None
            self.received, self.invalid = 0, 0
            self.ids, self.measured_ids = set(), set()
            self.trajectory_events, self.peer_events = [], []
            self.max_delivery_age = 0.0
            self.position, self.velocity = [-3.0, 0.0, 1.0], [0.0] * 3
            self.initial = list(self.position)
            self.position_min, self.position_max = list(self.position), list(self.position)
            self.measured_position_min = self.measured_position_max = None
            self.distance = self.measured_distance = self.max_speed = self.max_position_step = 0.0
            self.goals = [(3.0, 0.0, 1.0), (-3.0, 0.0, 1.0)]
            self.goal_index, self.goal_at = 0, -float("inf")
            self.ready_at = None
            self.next_cloud = self.next_status = self.started
            self.counts = {key: 0 for key in ("odom", "cloud", "goal", "peer")}
            self.measured_counts = dict(self.counts)
            self.peer_counts = {str(index): 0 for index in range(1, args.peers + 1)}
            self.measured_peer_counts = dict(self.peer_counts)
            self.peer_connections_min = None
            self.tick_events = []
            self.odom_pub = self.create_publisher(Odometry, "/drone_0_bench_odom", 1)
            self.cloud_pub = self.create_publisher(PointCloud2, "/drone_0_bench_cloud", 1)
            self.goal_pub = self.create_publisher(PoseStamped, "/move_base_simple/goal", 1)
            self.peer_pub = (self.create_publisher(Bspline, "/broadcast_bspline", 10)
                             if args.peers else None)
            self.trajectory_sub = self.create_subscription(
                Bspline, "/drone_0_planning/bspline", self.on_trajectory, 10)
            points = fixed_cloud_points()
            cloud = PointCloud2()
            cloud.header.frame_id = "world"
            cloud.height, cloud.width = 1, len(points)
            cloud.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                            for name, offset in (("x", 0), ("y", 4), ("z", 8))]
            cloud.is_bigendian = sys.byteorder == "big"
            cloud.point_step, cloud.row_step = 12, 12 * len(points)
            cloud.is_dense = True
            cloud.data = array.array("B", array.array("f", (v for p in points for v in p)).tobytes())
            self.cloud = cloud
            self.timer = self.create_timer(0.02, self.tick)
            self.emit("started", domain_id=args.domain_id, peers=args.peers,
                      cloud_points=cloud.width, target_odom_hz=50.0)

        @staticmethod
        def emit(event, **values):
            print(json.dumps({"event": event, **values}, allow_nan=False), flush=True)

        def on_trajectory(self, message):
            try:
                trajectory = Trajectory(message)
            except (ValueError, IndexError, OverflowError) as error:
                self.invalid += 1
                self.get_logger().warning("Rejected planner trajectory: " + str(error))
                return
            now = time.monotonic()
            self.trajectory = trajectory
            self.received += 1
            self.ids.add(message.traj_id)
            self.trajectory_events.append({"monotonic_s": now, "traj_id": message.traj_id})
            if self.first_trajectory is None:
                self.first_trajectory = now
                self.measurement_start = now + args.warmup
                self.measurement_end = self.measurement_start + args.duration
                self.emit("ready", first_trajectory_monotonic_s=now,
                          measurement_start_monotonic_s=self.measurement_start,
                          measurement_end_monotonic_s=self.measurement_end)
            if self.measurement_start <= now < self.measurement_end:
                self.measured_ids.add(message.traj_id)
            ros_now = self.get_clock().now().nanoseconds * 1e-9
            self.max_delivery_age = max(self.max_delivery_age, abs(ros_now - trajectory.start))

        def tick(self):
            tick = time.monotonic()
            if self.measurement_end is not None and tick >= self.measurement_end:
                self.status = "complete"
                self.timer.cancel()
                return
            if self.first_trajectory is None and tick - self.started > args.startup_timeout:
                self.status, self.reason = "failed", "no valid local B-spline before startup timeout"
                self.timer.cancel()
                return
            stamp = self.get_clock().now().to_msg()
            seconds = stamp.sec + stamp.nanosec * 1e-9
            measuring = self.measurement_start is not None and self.measurement_start <= tick
            if self.trajectory is not None and seconds >= self.trajectory.start:
                position, self.velocity = self.trajectory.evaluate(seconds)
                step = math.dist(self.position, position)
                self.distance += step
                self.measured_distance += step if measuring else 0.0
                self.max_position_step = max(self.max_position_step, step)
                self.position = position
                self.max_speed = max(self.max_speed, math.sqrt(sum(v * v for v in self.velocity)))
            self.position_min = [min(a, b) for a, b in zip(self.position_min, self.position)]
            self.position_max = [max(a, b) for a, b in zip(self.position_max, self.position)]
            if measuring:
                self.tick_events.append(tick)
                self.measured_position_min = (list(self.position) if self.measured_position_min is None else
                                             [min(a, b) for a, b in zip(self.measured_position_min, self.position)])
                self.measured_position_max = (list(self.position) if self.measured_position_max is None else
                                             [max(a, b) for a, b in zip(self.measured_position_max, self.position)])
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, "world", "drone_0"
            odom.pose.pose.position = Point(x=self.position[0], y=self.position[1], z=self.position[2])
            odom.pose.pose.orientation.w = 1.0
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = self.velocity
            self.odom_pub.publish(odom)
            self.count("odom", measuring)
            if tick >= self.next_cloud:
                self.cloud.header.stamp = stamp
                self.cloud_pub.publish(self.cloud)
                self.count("cloud", measuring)
                self.next_cloud += 0.5
                if self.next_cloud <= tick:
                    self.next_cloud = tick + 0.5
            connected = (self.odom_pub.get_subscription_count() >= 1 and
                         self.cloud_pub.get_subscription_count() >= 1 and
                         self.goal_pub.get_subscription_count() >= 1 and
                         self.count_publishers("/drone_0_planning/bspline") >= 1)
            if connected and self.ready_at is None:
                self.ready_at = tick
            near_goal = math.dist(self.position, self.goals[self.goal_index]) < 0.8
            if self.ready_at is not None and tick - self.ready_at >= 1.0 and (
                    self.counts["goal"] == 0 or (self.first_trajectory is not None and
                                                near_goal and tick - self.goal_at > 2.0)):
                if self.counts["goal"]:
                    self.goal_index = 1 - self.goal_index
                goal = PoseStamped()
                goal.header.stamp, goal.header.frame_id = stamp, "world"
                xyz = self.goals[self.goal_index]
                goal.pose.position = Point(x=xyz[0], y=xyz[1], z=xyz[2])
                goal.pose.orientation.w = 1.0
                self.goal_pub.publish(goal)
                self.count("goal", measuring)
                self.goal_at = tick
            # Publish every actual 50 Hz timer tick, without a second jitter-prone clock.
            if self.peer_pub is not None and self.ready_at is not None:
                batch_ids = []
                for peer_id in range(1, args.peers + 1):
                    peer = Bspline()
                    peer.drone_id, peer.order, peer.traj_id = peer_id, 3, self.counts["peer"] + 1
                    peer.start_time = stamp
                    peer.pos_pts = [Point(x=(peer_id - 2) * 1.5, y=3.8, z=1.8) for _ in range(6)]
                    peer.knots = [(index - 3) * 0.5 for index in range(10)]
                    self.peer_pub.publish(peer)
                    self.count("peer", measuring)
                    self.peer_counts[str(peer_id)] += 1
                    self.measured_peer_counts[str(peer_id)] += int(measuring)
                    batch_ids.append(peer_id)
                self.peer_events.append({"monotonic_s": tick, "peer_ids": batch_ids})
                if measuring:
                    connected_peers = self.peer_pub.get_subscription_count()
                    self.peer_connections_min = (connected_peers if self.peer_connections_min is None else
                                                 min(self.peer_connections_min, connected_peers))
            if tick >= self.next_status:
                self.emit("progress", elapsed_s=tick - self.started, bsplines=self.received,
                          goals=self.counts["goal"], position=self.position,
                          peer_messages=self.counts["peer"])
                self.next_status = tick + 5.0

        def count(self, name, measuring):
            self.counts[name] += 1
            self.measured_counts[name] += int(measuring)

        def report(self):
            finished = time.monotonic()
            measured_events = [event for event in self.trajectory_events
                               if self.measurement_start is not None and
                               self.measurement_start <= event["monotonic_s"] < self.measurement_end]
            event_span = (measured_events[-1]["monotonic_s"] - measured_events[0]["monotonic_s"]
                          if len(measured_events) >= 2 else 0.0)
            last_age = (self.measurement_end - measured_events[-1]["monotonic_s"]
                        if measured_events else None)
            peer_valid = (not args.peers or (self.peer_connections_min is not None and
                          self.peer_connections_min >= 1 and
                          all(count >= args.duration * 40.0 for count in self.measured_peer_counts.values())))
            sustained = (len(self.measured_ids) >= 2 and event_span >= args.duration * 0.5 and
                         last_age is not None and last_age <= min(5.0, args.duration))
            valid = (self.status == "complete" and self.invalid == 0 and sustained and
                     self.measured_distance > 0.5 and peer_valid and
                     self.measured_counts["odom"] >= args.duration * 40.0)
            if self.status == "complete" and not valid:
                self.reason = "window completed without sustained valid planning, motion, or input rates"
            usage = resource.getrusage(resource.RUSAGE_SELF)
            return {
                "schema_version": 1, "workload_valid": valid, "status": self.status, "reason": self.reason,
                "scope": "actual_ROS2_EGO_planner_with_synthetic_inputs_and_ideal_Bspline_follower_no_flight_io",
                "finished_at": dt.datetime.now(dt.timezone(dt.timedelta(hours=8))).isoformat(),
                "ros_domain_id": args.domain_id, "ros_localhost_only": True,
                "started_monotonic_s": self.started, "finished_monotonic_s": finished,
                "elapsed_s": finished - self.started, "measurement_start_monotonic_s": self.measurement_start,
                "measurement_end_monotonic_s": self.measurement_end,
                "warmup_s": args.warmup, "measurement_duration_s": args.duration,
                "first_trajectory_latency_s": (self.first_trajectory - self.started
                                                if self.first_trajectory is not None else None),
                "local_bspline_messages": self.received, "local_unique_trajectory_ids": len(self.ids),
                "local_trajectory_ids": sorted(self.ids), "invalid_local_trajectories": self.invalid,
                "local_trajectory_events": self.trajectory_events,
                "measured_bspline_messages": len(measured_events),
                "measured_unique_trajectory_ids": len(self.measured_ids),
                "measured_trajectory_ids": sorted(self.measured_ids),
                "measured_trajectory_event_span_s": event_span,
                "last_measured_trajectory_age_at_window_end_s": last_age,
                "max_local_trajectory_delivery_age_s": self.max_delivery_age,
                "initial_position": self.initial, "final_position": self.position,
                "position_range_m": {"min": self.position_min, "max": self.position_max},
                "measured_position_range_m": {"min": self.measured_position_min, "max": self.measured_position_max},
                "path_length_m": self.distance, "measured_path_length_m": self.measured_distance,
                "max_speed_m_s": self.max_speed, "max_following_position_step_m": self.max_position_step,
                "goals_sent": self.counts["goal"], "odom_messages": self.counts["odom"],
                "cloud_messages": self.counts["cloud"], "cloud_points": self.cloud.width,
                "map_dimensions_m": [10, 10, 3], "input_target_hz": {"odom": 50.0, "cloud": 2.0},
                "measured_input_messages": self.measured_counts,
                "measured_input_hz": {key: count / args.duration for key, count in self.measured_counts.items()},
                "measured_input_tick_span_s": (self.tick_events[-1] - self.tick_events[0]
                                                if len(self.tick_events) >= 2 else 0.0),
                "peer_count": args.peers, "peer_messages": self.counts["peer"],
                "peer_messages_by_id": self.peer_counts, "measured_peer_messages_by_id": self.measured_peer_counts,
                "measured_peer_hz_by_id": {key: count / args.duration for key, count in self.measured_peer_counts.items()},
                "peer_target_hz_per_id": 50.0 if args.peers else 0.0, "peer_events": self.peer_events,
                "peer_publisher_min_connections": self.peer_connections_min,
                "peer_internal_buffer_acceptance_instrumented": False,
                "peer_scope": "stationary trajectories on planner broadcast input; no remote planners or wireless link",
                "input_process_cpu_seconds": usage.ru_utime + usage.ru_stime,
                "input_process_peak_rss_kib": usage.ru_maxrss,
            }

    return ValidationNode()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--domain-id", type=int, default=66)
    parser.add_argument("--duration", type=positive, default=30.0)
    parser.add_argument("--warmup", type=float, default=2.0)
    parser.add_argument("--startup-timeout", type=positive, default=60.0)
    parser.add_argument("--peers", type=int, choices=(0, 3), default=0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.domain_id <= 232:
        parser.error("--domain-id must be an isolated nondefault domain in 1..232")
    if not math.isfinite(args.warmup) or args.warmup < 0 or args.duration > 3600:
        parser.error("invalid warmup or duration")
    args.output = args.output.resolve()
    if args.output.exists():
        parser.error("--output must be a new file")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"

    import rclpy

    rclpy.init(args=[])
    node = None
    try:
        node = create_validation_node(args)
        while rclpy.ok() and node.status == "running":
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.status == "running":
            node.status, node.reason = "interrupted", "ROS shutdown before measurement completion"
    except KeyboardInterrupt:
        if node is not None:
            node.status, node.reason = "interrupted", "input process interrupted"
    except Exception as error:
        if node is None:
            raise
        node.status, node.reason = "failed", type(error).__name__ + ": " + str(error)
    finally:
        if node is not None:
            report = node.report()
            with args.output.open("x") as stream:
                json.dump(report, stream, indent=2, allow_nan=False)
                stream.write("\n")
            node.emit("result", workload_valid=report["workload_valid"], output=str(args.output),
                      bsplines=report["local_bspline_messages"],
                      measured_unique_trajectory_ids=report["measured_unique_trajectory_ids"],
                      reason=report["reason"])
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node is not None and report["workload_valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
