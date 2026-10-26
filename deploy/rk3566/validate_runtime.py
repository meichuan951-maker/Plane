#!/usr/bin/env python3
"""Validate a built Humble deployment inside /workspace without hardware.

Source Humble and the unified /workspace/install/setup.bash before running.
Creates a NEW output directory with report.json and per-process logs. Stages:
1. Real demo_nodes_cpp talker -> demo_nodes_py listener DDS delivery.
2. Actual EGO planner -> synthetic-input/ideal-follower validator for 30 s.
3. position_to_mavros, read-only parameter requests, and synthetic observers
   that reject/count any unexpected arming or mode requests. No MAVROS or FCU
   is started, and no flight command is sent by this validator.

Only this invocation's subprocesses are signalled during cleanup. Processes
run in separate sessions; Linux child-subreaper and /proc identity tracking
also cover descendants that detach from the original process group.
"""

import argparse
import ctypes
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import signal
import subprocess
import sys
import time


def emit(event, **values):
    print(json.dumps({"event": event, **values}, allow_nan=False), flush=True)


def inside(path, root):
    return path == root or root in path.parents


def read_processes():
    processes = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = (entry / "stat").read_text()
            fields = stat[stat.rfind(")") + 2:].split()
            processes[int(entry.name)] = {
                "start": int(fields[19]), "ppid": int(fields[1]),
                "pgrp": int(fields[2]), "state": fields[0],
            }
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    return processes


class Processes:
    def __init__(self, directory, workspace, domain):
        self.directory, self.workspace, self.domain = directory, workspace, domain
        self.children, self.owned = [], {}
        self.env = os.environ.copy()
        self.env.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1", ROS_IPV6="off",
                        PYTHONUNBUFFERED="1", ROS_LOG_DIR=str(directory / "ros_logs"),
                        ROS_HOME=str(directory / "ros_home"))
        self.env.pop("ROS_MASTER_URI", None)
        self.env.pop("ROS_IP", None)
        self.env.pop("ROS_HOSTNAME", None)
        self.env.pop("ROS_NAMESPACE", None)

    def refresh(self):
        processes = read_processes()
        parents = {os.getpid()}
        while True:
            discovered = {pid for pid, info in processes.items() if info["ppid"] in parents}
            if discovered <= parents:
                break
            parents.update(discovered)
        for pid in parents - {os.getpid()}:
            self.owned[pid] = processes[pid]["start"]
        return {pid: info for pid, info in processes.items()
                if self.owned.get(pid) == info["start"]}

    def start(self, name, command):
        stdout = (self.directory / (name + ".stdout.log")).open("wb")
        stderr = (self.directory / (name + ".stderr.log")).open("wb")
        try:
            process = subprocess.Popen(command, cwd=self.workspace, env=self.env,
                                       stdin=subprocess.DEVNULL, stdout=stdout, stderr=stderr,
                                       start_new_session=True)
        except BaseException:
            stdout.close()
            stderr.close()
            raise
        record = {"name": name, "command": command, "pid": process.pid,
                  "stdout": stdout.name, "stderr": stderr.name,
                  "process": process, "streams": (stdout, stderr)}
        self.children.append(record)
        self.refresh()
        emit("process_started", stage=self.directory.name, name=name, pid=process.pid)
        return process

    def log_text(self, name):
        return "\n".join((self.directory / (name + suffix)).read_text(errors="replace")
                          for suffix in (".stdout.log", ".stderr.log"))

    def pause(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.refresh()
            time.sleep(min(0.1, max(0.0, end - time.monotonic())))

    def signal_owned(self, sig):
        active = self.refresh()
        groups = {info["pgrp"] for info in active.values() if info["state"] != "Z"}
        groups.discard(os.getpgrp())
        for group in groups:
            try:
                os.killpg(group, sig)
            except ProcessLookupError:
                pass
        for pid, info in active.items():
            if info["pgrp"] not in groups and info["state"] != "Z":
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass

    def reap(self):
        # Popen must reap its direct children first to preserve their exit codes.
        for record in self.children:
            record["process"].poll()
        while True:
            try:
                pid, _ = os.waitpid(-1, os.WNOHANG)
            except ChildProcessError:
                break
            if pid == 0:
                break

    def cleanup(self):
        errors = []
        for sig, grace in ((signal.SIGINT, 2.0), (signal.SIGTERM, 1.5), (signal.SIGKILL, 1.0)):
            try:
                self.signal_owned(sig)
            except OSError as error:
                errors.append(str(error))
            deadline = time.monotonic() + grace
            while time.monotonic() < deadline:
                self.reap()
                if not self.refresh():
                    break
                time.sleep(0.05)
            if not self.refresh():
                break
        self.reap()
        remaining = self.refresh()
        children = []
        for record in self.children:
            for stream in record["streams"]:
                stream.close()
            children.append({key: value for key, value in record.items()
                             if key not in ("process", "streams")})
            children[-1]["returncode"] = record["process"].poll()
        return {"success": not remaining and not errors,
                "remaining_processes": remaining, "errors": errors, "processes": children}


def check_alive(process, label):
    if process.poll() is not None:
        raise RuntimeError(label + " exited with code " + str(process.returncode))


def dds_stage(pool, args):
    listener = pool.start("listener", ["ros2", "run", "demo_nodes_py", "listener"])
    talker = pool.start("talker", ["ros2", "run", "demo_nodes_cpp", "talker"])
    deadline = time.monotonic() + 20.0
    received = []
    while time.monotonic() < deadline:
        check_alive(listener, "DDS listener")
        check_alive(talker, "DDS talker")
        received = sorted(set(re.findall(r"I heard:.*Hello World:\s*(\d+)", pool.log_text("listener"))), key=int)
        if len(received) >= 3:
            return {"success": True, "received_distinct_messages": len(received),
                    "received_sequence_numbers": [int(value) for value in received],
                    "evidence": "actual official listener log contains at least three distinct talker messages"}
        pool.pause(0.2)
    raise TimeoutError("DDS listener did not receive three distinct messages within 20 s")


def planner_stage(pool, args):
    validator = Path(__file__).resolve().with_name("validate_ego_ros2.py")
    if not validator.is_file():
        raise FileNotFoundError(str(validator))
    planner = pool.start("planner", [
        "ros2", "launch", "ego_planner", "advanced_param.launch.py",
        "map_size_x_:=10.0", "map_size_y_:=10.0", "map_size_z_:=3.0",
        "odometry_topic:=bench_odom", "cloud_topic:=bench_cloud", "drone_id:=0",
        "flight_type:=1", "obj_num_set:=0", "max_vel:=1.5", "max_acc:=2.0",
        "planning_horizon:=7.5",
    ])
    report_path = pool.directory / "planner_validation.json"
    inputs = pool.start("synthetic_inputs", [
        sys.executable, str(validator), "--domain-id", str(pool.domain),
        "--duration", str(args.duration), "--warmup", "2", "--startup-timeout", "60",
        "--peers", "3", "--output", str(report_path),
    ])
    deadline = time.monotonic() + args.duration + 90.0
    next_status = time.monotonic() + 10.0
    while inputs.poll() is None and time.monotonic() < deadline:
        check_alive(planner, "EGO planner launch")
        if time.monotonic() >= next_status:
            emit("stage_progress", stage="planner", remaining_timeout_s=deadline - time.monotonic())
            next_status = time.monotonic() + 10.0
        pool.pause(0.1)
    if inputs.poll() is None:
        raise TimeoutError("EGO synthetic-input validation exceeded its bounded startup/measurement timeout")
    check_alive(planner, "EGO planner launch")
    if not report_path.is_file():
        raise RuntimeError("EGO validator exited without a JSON report; code=" + str(inputs.returncode))
    report = json.loads(report_path.read_text())
    result = {"success": inputs.returncode == 0 and report.get("workload_valid") is True,
              "validator_exit_code": inputs.returncode, "validator_report": str(report_path),
              "measured_unique_trajectory_ids": report.get("measured_unique_trajectory_ids"),
              "measured_trajectory_event_span_s": report.get("measured_trajectory_event_span_s"),
              "measured_path_length_m": report.get("measured_path_length_m"),
              "measured_peer_hz_by_id": report.get("measured_peer_hz_by_id"),
              "reason": report.get("reason", "")}
    return result


def control_stage(pool, args):
    # This manager initializes ROS only here; earlier stages used subprocesses.
    os.environ["ROS_DOMAIN_ID"] = str(pool.domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_LOG_DIR"] = str(pool.directory / "probe_ros_logs")
    os.environ["ROS_HOME"] = str(pool.directory / "probe_ros_home")
    import rclpy
    from mavros_msgs.srv import CommandBool, SetMode
    from rcl_interfaces.msg import ParameterType
    from rcl_interfaces.srv import GetParameters
    from rclpy.executors import SingleThreadedExecutor

    context = rclpy.context.Context()
    rclpy.init(args=[], context=context)
    node = executor = None
    try:
        node = rclpy.create_node("rk3566_control_readonly_probe", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        counts = {"arming": 0, "set_mode": 0}

        def reject_arm(request, response):
            counts["arming"] += 1
            response.success = False
            return response

        def reject_mode(request, response):
            counts["set_mode"] += 1
            response.mode_sent = False
            return response

        # These are local rejecting observers, not MAVROS/FCU connections.
        arm_observer = node.create_service(CommandBool, "/mavros/cmd/arming", reject_arm)
        mode_observer = node.create_service(SetMode, "/mavros/set_mode", reject_mode)
        process = pool.start("position_to_mavros", [
            "ros2", "run", "px4_control", "position_to_mavros_node", "--ros-args",
            "-r", "__node:=rk3566_control_smoke", "-p", "target_frame_id:=map",
            "-p", "source_frame_id:=base_link", "-p", "output_rate:=20.0",
            "-p", "max_path_length:=3",
        ])
        client = node.create_client(GetParameters, "/rk3566_control_smoke/get_parameters")
        deadline = time.monotonic() + 25.0
        while not client.wait_for_service(timeout_sec=0.1):
            check_alive(process, "position_to_mavros")
            pool.refresh()
            executor.spin_once(timeout_sec=0.05)
            if time.monotonic() >= deadline:
                raise TimeoutError("control node parameter services were not ready within 25 s")
        readings = []
        for index in range(2):
            future = client.call_async(GetParameters.Request(names=["output_rate"]))
            while not future.done():
                check_alive(process, "position_to_mavros")
                pool.refresh()
                executor.spin_once(timeout_sec=0.05)
                if time.monotonic() >= deadline:
                    raise TimeoutError("control node parameter read timed out")
            response = future.result()
            if response is None or len(response.values) != 1:
                raise RuntimeError("control node returned an invalid parameter response")
            value = response.values[0]
            if value.type != ParameterType.PARAMETER_DOUBLE or value.double_value != 20.0:
                raise RuntimeError("control output_rate is not double 20.0")
            readings.append({"monotonic_s": time.monotonic(), "output_rate": value.double_value})
            if index == 0:
                observe_until = time.monotonic() + 2.0
                while time.monotonic() < observe_until:
                    check_alive(process, "position_to_mavros")
                    pool.refresh()
                    executor.spin_once(timeout_sec=0.05)
        clients = node.get_client_names_and_types_by_node("rk3566_control_smoke", "/")
        command_clients = [name for name, _ in clients
                           if name in ("/mavros/cmd/arming", "/mavros/set_mode")]
        check_alive(process, "position_to_mavros")
        result = {"success": not any(counts.values()) and not command_clients,
                  "parameter_readings": readings,
                  "parameter_request_count": 2, "validator_flight_commands_sent": 0,
                  "observed_arming_requests": counts["arming"],
                  "observed_set_mode_requests": counts["set_mode"],
                  "control_node_arming_or_mode_client_endpoints": command_clients,
                  "control_node_alive_after_reads": True,
                  "observation_span_s": readings[-1]["monotonic_s"] - readings[0]["monotonic_s"],
                  "mavros_started": False, "real_flight_controller_connected": False,
                  "tf_published": False,
                  "observer_scope": "loopback-only rejecting synthetic services; observation limited to this stage"}
        (pool.directory / "control_probe.json").write_text(json.dumps(result, indent=2) + "\n")
        # Keep service handles referenced until after the observations complete.
        del arm_observer, mode_observer
        return result
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            rclpy.shutdown(context=context)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--workspace", type=Path, default=Path("/workspace"))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=30.0, help="EGO measurement seconds (default: 30)")
    args = parser.parse_args()
    workspace_root = Path("/workspace").resolve()
    args.workspace = args.workspace.resolve()
    args.output_dir = args.output_dir.resolve()
    if not inside(args.workspace, workspace_root) or not args.workspace.is_dir():
        parser.error("--workspace must be an existing directory under /workspace")
    if not inside(args.output_dir, args.workspace) or args.output_dir.exists():
        parser.error("--output-dir must be a NEW directory inside --workspace")
    if not 1.0 <= args.duration <= 3600.0:
        parser.error("--duration must be finite and in 1..3600 seconds")
    args.output_dir.mkdir(parents=True)
    os.chdir(args.workspace)
    started = time.monotonic()
    report = {"schema_version": 1, "success": False,
              "scope": "isolated_Humble_DDS_planner_and_readonly_control_validation_no_hardware",
              "started_at": dt.datetime.now(dt.timezone(dt.timedelta(hours=8))).isoformat(),
              "started_monotonic_s": started, "workspace": str(args.workspace),
              "architecture": platform.machine(), "kernel": platform.release(),
              "ros_distro": os.environ.get("ROS_DISTRO"), "python_version": platform.python_version(),
              "domains": {"dds": 65, "planner": 66, "control": 67}, "stages": []}
    interrupted = False

    def terminate(signum, frame):
        raise KeyboardInterrupt("validation received signal " + str(signum))

    old_term_handler = signal.signal(signal.SIGTERM, terminate)
    try:
        for name in ("validate_runtime.py", "validate_ego_ros2.py"):
            path = Path(__file__).resolve().with_name(name)
            report.setdefault("validator_sha256", {})[name] = hashlib.sha256(path.read_bytes()).hexdigest()
        if shutil.which("ros2") is None:
            raise RuntimeError("ros2 is unavailable; source Humble and /workspace/install/setup.bash first")
        libc = ctypes.CDLL(None, use_errno=True)
        if libc.prctl(36, 1, 0, 0, 0) != 0:
            raise OSError(ctypes.get_errno(), "cannot enable Linux child-subreaper for bounded cleanup")
        for name, domain, function in (("dds", 65, dds_stage), ("planner", 66, planner_stage),
                                       ("control", 67, control_stage)):
            directory = args.output_dir / name
            directory.mkdir()
            pool = Processes(directory, args.workspace, domain)
            phase = {"name": name, "domain_id": domain, "started_monotonic_s": time.monotonic()}
            emit("stage_started", stage=name, domain_id=domain)
            try:
                phase.update(function(pool, args))
            except KeyboardInterrupt:
                phase.update(success=False, error="KeyboardInterrupt: validation interrupted")
                interrupted = True
            except Exception as error:
                phase.update(success=False, error=type(error).__name__ + ": " + str(error))
            finally:
                phase["cleanup"] = pool.cleanup()
                phase["success"] = bool(phase.get("success")) and phase["cleanup"]["success"]
                phase["finished_monotonic_s"] = time.monotonic()
                report["stages"].append(phase)
                (directory / "stage.json").write_text(json.dumps(phase, indent=2, allow_nan=False) + "\n")
                emit("stage_result", stage=name, success=phase["success"],
                     error=phase.get("error", phase.get("reason", "")))
            if interrupted or not phase["cleanup"]["success"]:
                break
        report["success"] = len(report["stages"]) == 3 and all(stage["success"] for stage in report["stages"])
    except KeyboardInterrupt:
        interrupted = True
        report["error"] = "KeyboardInterrupt: validation interrupted"
    except Exception as error:
        report["error"] = type(error).__name__ + ": " + str(error)
    finally:
        signal.signal(signal.SIGTERM, old_term_handler)
        report["finished_monotonic_s"] = time.monotonic()
        report["elapsed_s"] = report["finished_monotonic_s"] - started
        report["interrupted"] = interrupted
        (args.output_dir / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        emit("result", success=report["success"], output_dir=str(args.output_dir))
    return 0 if report["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
