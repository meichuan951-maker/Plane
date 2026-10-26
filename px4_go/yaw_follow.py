"""Headless, single-serial-owner visual yaw task. Never imports gogogo.py.

Run ``python3 -m px4_go.yaw_follow --config FILE --check-config`` first.
Checking configuration does not open hardware or start child processes.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, fields
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import stat
import subprocess
import sys
import termios
import time

from .yaw_mission import FlightState, MissionConfig, YawMission
from .yaw_policy import parse_visual_report, yaw_rate_for_sample


def log(event: str, **data) -> None:
    try:
        print(json.dumps({"event": event, "monotonic_s": time.monotonic(), **data},
                         ensure_ascii=False, allow_nan=False), flush=True)
    except OSError:
        pass  # Losing a log consumer must not bypass the flight shutdown path.


def number(value) -> bool:
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def checked_path(value, name: str, *, absolute: bool = False) -> Path:
    if not isinstance(value, str) or not value or any(c in value for c in "\n\r\x00"):
        raise ValueError(f"{name}: expected a nonempty path")
    if "REPLACE_ME" in value or "?" in value:
        raise ValueError(f"{name}: replace the placeholder before deployment")
    path = Path(value)
    if absolute and not path.is_absolute():
        raise ValueError(f"{name}: expected an absolute path")
    return path


@dataclass(frozen=True)
class RuntimeConfig:
    project_dir: str
    vision_binary: str
    vision_config: str
    camera_device: str
    width: int
    height: int
    connection: str
    baud: int
    flight_height_m: float
    target_loss_timeout_s: float
    mission_duration_s: float
    target_loss_action: str
    auto_arm: bool
    camera_forward_unmirrored_confirmed: bool
    target_system: int = 1
    target_component: int = 1
    source_system: int = 245
    source_component: int = 191
    max_yaw_rate_rad_s: float = 0.2
    yaw_gain_rad_s: float = 0.3
    deadband_normalized: float = 0.05
    yaw_direction: int = 1
    takeoff_speed_m_s: float = 0.2
    ready_timeout_s: float = 60.0
    landing_timeout_s: float = 45.0
    telemetry_timeout_s: float = 0.5
    heartbeat_timeout_s: float = 2.0
    prestream_s: float = 2.0
    require_estimator_status: bool = True

    @classmethod
    def load(cls, path: Path) -> RuntimeConfig:
        def no_duplicates(pairs):
            result = {}
            for key, value in pairs:
                if key in result:
                    raise ValueError(f"duplicate configuration key: {key}")
                result[key] = value
            return result

        with path.open(encoding="utf-8") as stream:
            data = json.load(stream, object_pairs_hook=no_duplicates)
        if not isinstance(data, dict):
            raise ValueError("runtime configuration must be a JSON object")
        unknown = data.keys() - {item.name for item in fields(cls)}
        if unknown:
            raise ValueError(f"unknown configuration keys: {sorted(unknown)}")
        try:
            config = cls(**data)
        except TypeError as exc:
            raise ValueError(str(exc)) from exc
        config.validate()
        return config

    def validate(self) -> None:
        checked_path(self.project_dir, "project_dir", absolute=True)
        checked_path(self.vision_binary, "vision_binary")
        checked_path(self.vision_config, "vision_config")
        for name in ("camera_device", "connection"):
            path = checked_path(getattr(self, name), name, absolute=True)
            if not str(path).startswith("/dev/"):
                raise ValueError(f"{name}: this deployment expects a /dev/ serial/camera path")
        if self.camera_device == self.connection:
            raise ValueError("camera and flight controller cannot use the same device")
        for name in ("auto_arm", "camera_forward_unmirrored_confirmed",
                     "require_estimator_status"):
            if getattr(self, name) is not True:
                raise ValueError(f"{name}: must be true for this automatic yaw task")
        if self.target_loss_action not in ("land", "hold"):
            raise ValueError("target_loss_action: explicitly select 'land' or 'hold'")
        for name in ("target_system", "target_component", "source_system", "source_component"):
            value = getattr(self, name)
            if type(value) is not int or not 1 <= value <= 255:
                raise ValueError(f"{name}: expected integer 1..255")
        if self.source_system == self.target_system:
            raise ValueError("source_system must differ from target_system")
        for name in ("width", "height"):
            if type(getattr(self, name)) is not int or not 16 <= getattr(self, name) <= 4096:
                raise ValueError(f"{name}: expected integer 16..4096")
        if type(self.baud) is not int or self.baud not in (57600, 115200, 230400, 460800, 921600):
            raise ValueError("baud: use 57600/115200/230400/460800/921600, matching PX4")
        bounds = {
            "flight_height_m": (0.1, 10), "target_loss_timeout_s": (0.25, 300),
            "mission_duration_s": (1, 3600), "max_yaw_rate_rad_s": (0.01, 0.5),
            "yaw_gain_rad_s": (0.01, 2), "takeoff_speed_m_s": (0.05, 1),
            "ready_timeout_s": (5, 300), "landing_timeout_s": (10, 50),
            "telemetry_timeout_s": (0.1, 1), "heartbeat_timeout_s": (1, 3),
            "prestream_s": (2, 10), "deadband_normalized": (0, 0.5),
        }
        for name, (low, high) in bounds.items():
            value = getattr(self, name)
            if not number(value) or not low <= value <= high:
                raise ValueError(f"{name}: fill a finite number in [{low}, {high}]")
        if type(self.yaw_direction) is not int or self.yaw_direction not in (-1, 1):
            raise ValueError("yaw_direction: expected 1 or -1")
        if self.mission_duration_s <= self.flight_height_m / self.takeoff_speed_m_s + 5:
            raise ValueError("mission_duration_s must exceed the planned climb time plus 5 seconds")

    def project_path(self, value: str) -> Path:
        path = Path(value)
        return path if path.is_absolute() else Path(self.project_dir) / path

    def check_files(self) -> None:
        import yaml
        project = Path(self.project_dir)
        if not project.is_dir():
            raise ValueError(f"project_dir does not exist: {project}")
        binary = self.project_path(self.vision_binary)
        if not binary.is_file() or not os.access(binary, os.X_OK):
            raise ValueError(f"build armor_auto on the RK3588 first: {binary}")
        config_path = self.project_path(self.vision_config)
        with config_path.open(encoding="utf-8") as stream:
            vision = yaml.safe_load(stream)
        if not isinstance(vision, dict) or not isinstance(vision.get("camera"), dict):
            raise ValueError("vision_config: missing camera mapping")
        camera = vision["camera"]
        for name, expected in (("device", self.camera_device), ("width", self.width), ("height", self.height)):
            if type(camera.get(name)) is not type(expected) or camera[name] != expected:
                raise ValueError(f"vision_config camera.{name} must match runtime configuration")
        if camera.get("format") not in ("YUYV", "MJPG"):
            raise ValueError("vision_config camera.format must be YUYV or MJPG")
        if type(camera.get("fps")) is not int or not 1 <= camera["fps"] <= 120:
            raise ValueError("vision_config camera.fps must be an integer 1..120")
        detector_path = vision.get("detector_config")
        if not isinstance(detector_path, str) or not (config_path.parent / detector_path).is_file():
            raise ValueError("vision_config detector_config must point to an existing detector file")

    def mission_config(self) -> MissionConfig:
        return MissionConfig(**{name: getattr(self, name) for name in (
            "flight_height_m", "target_loss_timeout_s", "mission_duration_s",
            "takeoff_speed_m_s", "ready_timeout_s", "landing_timeout_s", "prestream_s",
            "target_loss_action")})


class VisionPipe:
    """Bounded nonblocking JSONL reader; no log files are tailed for control."""
    def __init__(self, config: RuntimeConfig):
        self.config = config
        self.process = None
        self.buffer = b""
        self.latest = None
        self.last_frame_at = -math.inf
        self.last_key = None
        self.failed = False

    def start(self) -> None:
        command = [str(self.config.project_path(self.config.vision_binary)), "--config",
                   str(self.config.project_path(self.config.vision_config)), "--no-show", "--hz", "10"]
        self.process = subprocess.Popen(command, cwd=self.config.project_dir,
                                        stdout=subprocess.PIPE, start_new_session=True)
        os.set_blocking(self.process.stdout.fileno(), False)
        log("vision_started", pid=self.process.pid)

    @property
    def alive(self) -> bool:
        return self.process is not None and self.process.poll() is None and not self.failed

    def accept(self, report, now: float) -> None:
        if not isinstance(report, dict):
            self.latest = None
            return
        sample = report.get("sample_time_monotonic_s")
        frame_age = report.get("frame_age_ms")
        frame, generation = report.get("frame"), report.get("generation")
        current = (report.get("source") == "automatic_armor_rgb_pnp"
                   and report.get("coordinate_frame") == "rgb_optical"
                   and type(frame) is int and frame > 0
                   and type(generation) is int and generation >= 0
                   and number(sample) and 0 <= now - sample <= 0.25
                   and number(frame_age) and 0 <= frame_age <= 250
                   and report.get("reason") not in ("camera_failed", "stopped"))
        key = (generation, frame)
        if current and (self.last_key is None or key >= self.last_key):
            self.last_frame_at = sample
            self.last_key = key
            self.latest = parse_visual_report(report, now=now, width=self.config.width,
                                               height=self.config.height)
        else:
            self.latest = None

    def poll(self, now: float) -> None:
        if not self.process:
            return
        # A broken producer cannot monopolize the 20 Hz control loop or grow memory.
        for _ in range(8):
            try:
                data = os.read(self.process.stdout.fileno(), 16384)
            except BlockingIOError:
                break
            if not data:
                self.failed = True
                self.latest = None
                break
            self.buffer += data
            while b"\n" in self.buffer:
                line, self.buffer = self.buffer.split(b"\n", 1)
                if len(line) > 65536:
                    self.failed = True
                    self.latest = None
                    return
                try:
                    self.accept(json.loads(line), now)
                except (ValueError, TypeError, UnicodeError):
                    self.latest = None
            if len(self.buffer) > 65536:
                self.failed = True
                self.latest = None
                return

    def close(self) -> None:
        if self.process is None:
            return
        try:
            os.killpg(self.process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid, signal.SIGKILL)
            self.process.wait(timeout=2)
        self.process.stdout.close()


class Px4Link:
    """One MAVLink reader for both local-position supervision and control."""
    def __init__(self, config: RuntimeConfig, master, mavlink):
        self.config, self.master, self.m = config, master, mavlink
        self.heartbeat = None
        self.heartbeat_at = self.position_at = self.estimator_at = self.landed_at = -math.inf
        self.position = None
        self.velocity = (0.0, 0.0, 0.0)
        self.boot_ms = None
        self.estimator_flags = 0
        self.landed = 0
        self.reset_counter = None
        self.fault = None
        self.streams_requested = False
        self.next_heartbeat = 0.0
        self.last_status_at = -math.inf

    def command(self, command: int, *parameters) -> None:
        values = list(parameters) + [0.0] * (7 - len(parameters))
        self.master.mav.command_long_send(self.config.target_system, self.config.target_component,
                                          command, 0, *values)

    def poll(self, now: float) -> None:
        deadline = time.monotonic() + 0.005
        for _ in range(200):
            if time.monotonic() >= deadline:
                break
            message = self.master.recv_match(blocking=False)
            if message is None:
                break
            if ((message.get_srcSystem(), message.get_srcComponent()) !=
                    (self.config.target_system, self.config.target_component)):
                continue
            self.accept(message, now)
        if self.heartbeat is not None and not self.streams_requested:
            for message_id, interval_us in ((32, 50000), (230, 100000), (245, 500000), (331, 200000)):
                self.command(self.m.MAV_CMD_SET_MESSAGE_INTERVAL, message_id, interval_us)
            self.streams_requested = True
        if now >= self.next_heartbeat:
            self.master.mav.heartbeat_send(self.m.MAV_TYPE_ONBOARD_CONTROLLER,
                                           self.m.MAV_AUTOPILOT_INVALID, 0, 0,
                                           self.m.MAV_STATE_ACTIVE)
            self.next_heartbeat = now + 1.0

    def accept(self, message, now: float) -> None:
        kind = message.get_type()
        if kind == "HEARTBEAT":
            multirotors = (self.m.MAV_TYPE_QUADROTOR, self.m.MAV_TYPE_HEXAROTOR,
                           self.m.MAV_TYPE_OCTOROTOR, self.m.MAV_TYPE_TRICOPTER)
            if message.autopilot == self.m.MAV_AUTOPILOT_PX4 and message.type in multirotors:
                self.heartbeat, self.heartbeat_at = message, now
        elif kind == "LOCAL_POSITION_NED":
            position = (message.x, message.y, message.z)
            velocity = (message.vx, message.vy, message.vz)
            if not all(number(value) for value in (*position, *velocity)):
                self.fault = "nonfinite_local_position"
                return
            boot_ms = message.time_boot_ms
            if self.boot_ms is not None:
                delta = (boot_ms - self.boot_ms) & 0xFFFFFFFF
                if delta == 0:
                    return
                if delta > 0x7FFFFFFF:
                    self.fault = "flight_controller_clock_reset"
                    return
                # Multiple telemetry messages drained in one poll share a host
                # timestamp. The FC boot interval still advances for each one.
                dt = delta / 1000.0
                if dt <= 1:
                    expected = tuple(p + v * dt for p, v in zip(self.position, self.velocity))
                    if math.dist(position, expected) > 0.5:
                        self.fault = "local_position_discontinuity"
                        return
            self.position, self.velocity = position, velocity
            self.position_at, self.boot_ms = now, boot_ms
        elif kind == "ESTIMATOR_STATUS":
            self.estimator_flags, self.estimator_at = message.flags, now
        elif kind == "EXTENDED_SYS_STATE":
            self.landed, self.landed_at = message.landed_state, now
        elif kind == "ODOMETRY":
            if self.reset_counter is not None and message.reset_counter != self.reset_counter:
                self.fault = "estimator_origin_reset"
            self.reset_counter = message.reset_counter
        elif kind == "COMMAND_ACK":
            log("command_ack", command=message.command, result=message.result)
            if (message.command == self.m.MAV_CMD_COMPONENT_ARM_DISARM
                    and message.result not in (self.m.MAV_RESULT_ACCEPTED, self.m.MAV_RESULT_IN_PROGRESS)):
                self.fault = "arm_rejected"
        elif kind == "STATUSTEXT" and now - self.last_status_at >= 0.2:
            text = message.text
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="replace")
            log("px4_status", severity=message.severity, text=str(text)[:256])
            self.last_status_at = now

    def state(self, now: float, vision: VisionPipe) -> FlightState:
        heartbeat_fresh = now - self.heartbeat_at <= self.config.heartbeat_timeout_s
        heartbeat = self.heartbeat
        armed = bool(heartbeat is not None and heartbeat.base_mode & self.m.MAV_MODE_FLAG_SAFETY_ARMED)
        custom_mode = heartbeat.custom_mode if heartbeat else 0
        main_mode, sub_mode = (custom_mode >> 16) & 0xFF, (custom_mode >> 24) & 0xFF
        flags = self.estimator_flags
        required = (self.m.ESTIMATOR_ATTITUDE | self.m.ESTIMATOR_VELOCITY_HORIZ |
                    self.m.ESTIMATOR_VELOCITY_VERT | self.m.ESTIMATOR_POS_VERT_ABS)
        horizontal = self.m.ESTIMATOR_POS_HORIZ_REL | self.m.ESTIMATOR_POS_HORIZ_ABS
        invalid = self.m.ESTIMATOR_CONST_POS_MODE | self.m.ESTIMATOR_ACCEL_ERROR
        estimator_ready = (flags & required == required and bool(flags & horizontal)
                           and not flags & invalid
                           and now - self.estimator_at <= self.config.telemetry_timeout_s)
        telemetry_ready = (self.fault is None and self.position is not None
                           and now - self.position_at <= self.config.telemetry_timeout_s
                           and estimator_ready)
        on_ground = (self.landed == self.m.MAV_LANDED_STATE_ON_GROUND
                     and now - self.landed_at <= 2.0)
        return FlightState(heartbeat_fresh=heartbeat_fresh, telemetry_ready=telemetry_ready,
                           armed=armed, offboard=main_mode == 6, on_ground=on_ground,
                           vision_alive=vision.alive, frame_fresh=now - vision.last_frame_at <= 0.25,
                           position=self.position, land_mode=main_mode == 4 and sub_mode == 6)

    def send_action(self, action, now: float) -> None:
        if action.position is not None:
            # Enable local XYZ position + yaw rate. Ignore velocity, acceleration and yaw angle.
            mask = (self.m.POSITION_TARGET_TYPEMASK_VX_IGNORE | self.m.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                    self.m.POSITION_TARGET_TYPEMASK_VZ_IGNORE | self.m.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                    self.m.POSITION_TARGET_TYPEMASK_AY_IGNORE | self.m.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                    self.m.POSITION_TARGET_TYPEMASK_YAW_IGNORE)
            self.master.mav.set_position_target_local_ned_send(
                int(now * 1000) & 0xFFFFFFFF, self.config.target_system, self.config.target_component,
                self.m.MAV_FRAME_LOCAL_NED, mask, *action.position, 0, 0, 0, 0, 0, 0, 0, action.yaw_rate)
        if action.request_mode:
            self.master.mav.set_mode_send(self.config.target_system,
                                          self.m.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6 << 16)
            log("request_offboard")
        if action.request_arm:
            self.command(self.m.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            log("request_arm")
        if action.request_land:
            self.command(self.m.MAV_CMD_NAV_LAND, 0, 0, 0, math.nan, math.nan, math.nan, math.nan)
            log("request_land", reason=action.reason)


def wait_devices(config: RuntimeConfig, stopped) -> bool:
    deadline = time.monotonic() + config.ready_timeout_s
    while time.monotonic() < deadline and not stopped():
        try:
            if all(os.access(path, os.R_OK | os.W_OK) and stat.S_ISCHR(Path(path).stat().st_mode)
                   for path in (config.connection, config.camera_device)):
                return True
        except OSError:
            pass  # A USB device can disappear between access() and stat().
        time.sleep(0.2)
    return False


def run(config: RuntimeConfig) -> int:
    # Lazy import: --help / --check-config never initialize a transport.
    from pymavlink import mavutil
    stop_requested = False

    def stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    if not wait_devices(config, lambda: stop_requested):
        log("devices_not_ready_or_stopped")
        return 1
    vision = VisionPipe(config)
    master = None
    link = None
    mission = None
    may_have_armed = False
    state = None
    try:
        master = mavutil.mavlink_connection(config.connection, baud=config.baud, dialect="common",
                                            source_system=config.source_system,
                                            source_component=config.source_component,
                                            autoreconnect=False, force_connected=False)
        # Linux exclusivity also prevents an accidental second serial reader.
        master.port.exclusive = True
        fcntl.ioctl(master.port.fileno(), termios.TIOCEXCL)
        master.port.write_timeout = 0.02
        link = Px4Link(config, master, mavutil.mavlink)
        vision.start()
        mission = YawMission(config.mission_config(), time.monotonic())
        next_tick = time.monotonic()
        next_log = 0.0
        old_phase = None
        while True:
            now = time.monotonic()
            vision.poll(now)
            link.poll(now)
            state = link.state(now, vision)
            sample = vision.latest
            target_fresh = sample is not None and 0 <= now - sample.sample_time_s <= 0.25
            rate = yaw_rate_for_sample(sample, now=now, gain=config.yaw_gain_rad_s,
                                       max_rate=config.max_yaw_rate_rad_s,
                                       deadband=config.deadband_normalized, direction=config.yaw_direction)
            action = mission.tick(now, state, yaw_rate=rate, target_fresh=target_fresh,
                                  stop=stop_requested)
            may_have_armed = may_have_armed or action.request_arm
            link.send_action(action, now)
            if old_phase != mission.phase or now >= next_log or action.done:
                log("task_status", phase=mission.phase, reason=action.reason,
                    telemetry_ready=state.telemetry_ready, fault=link.fault,
                    armed=state.armed, target_fresh=target_fresh,
                    yaw_rate_rad_s=action.yaw_rate, position_ned_m=state.position)
                old_phase, next_log = mission.phase, now + 1.0
            if action.done:
                return 0 if action.success else 1
            next_tick += 0.05
            delay = next_tick - time.monotonic()
            if delay <= 0:
                next_tick = time.monotonic()
            else:
                time.sleep(delay)
    except (OSError, RuntimeError, ValueError) as exc:
        log("runtime_error", error=str(exc))
        # On transport loss no landing success can be asserted. Never reconnect/re-arm.
        # poll() may have received a takeover heartbeat before its later read
        # failed. Read the cache itself, never the preceding loop's snapshot.
        heartbeat = link.heartbeat if link is not None else None
        controlled = False
        if heartbeat is not None and time.monotonic() - link.heartbeat_at <= config.heartbeat_timeout_s:
            main_mode = (heartbeat.custom_mode >> 16) & 0xFF
            sub_mode = (heartbeat.custom_mode >> 24) & 0xFF
            controlled = main_mode == 6 or (main_mode == 4 and sub_mode == 6)
        if link is not None and may_have_armed and controlled:
            try:
                link.command(link.m.MAV_CMD_NAV_LAND, 0, 0, 0, math.nan, math.nan, math.nan, math.nan)
                log("emergency_land_requested")
            except (OSError, RuntimeError, ValueError):
                log("link_unavailable_for_landing")
        return 1
    finally:
        vision.close()
        if master is not None:
            master.close()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--check-config", action="store_true", help="validate files without opening hardware")
    args = parser.parse_args(argv)
    try:
        config = RuntimeConfig.load(args.config)
        config.check_files()
        if args.check_config:
            # Check required packages without opening devices.
            import importlib.util
            if importlib.util.find_spec("pymavlink") is None:
                raise ValueError("install pymavlink in the selected Python environment")
            log("configuration_ok", project_dir=config.project_dir, hardware_opened=False)
            return 0
        return run(config)
    except (OSError, ValueError, ImportError) as exc:
        print(f"configuration/startup error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
