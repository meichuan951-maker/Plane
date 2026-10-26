"""Runtime wiring regression using files and in-memory MAVLink stubs only."""
from collections import deque
from dataclasses import asdict, replace
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from px4_go import yaw_follow
from px4_go.yaw_follow import Px4Link, RuntimeConfig, VisionPipe
from px4_go.yaw_mission import Action


def runtime_config(project_dir="/tmp/yaw-follow-fixture"):
    return RuntimeConfig(
        project_dir=str(project_dir), vision_binary="build/armor_auto",
        vision_config="config/armor_auto.yaml", camera_device="/dev/video-test",
        width=640, height=480, connection="/dev/tty-flight-test", baud=921600,
        flight_height_m=1.0, target_loss_timeout_s=3.0, mission_duration_s=60.0,
        target_loss_action="land", auto_arm=True,
        camera_forward_unmirrored_confirmed=True)


class RuntimeConfigTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="yaw-follow-config-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.config = runtime_config(self.root)
        self.json_path = self.root / "runtime.json"

    def load(self, data):
        self.json_path.write_text(json.dumps(data), encoding="utf-8")
        return RuntimeConfig.load(self.json_path)

    def make_vision_files(self):
        (self.root / "build").mkdir()
        (self.root / "config").mkdir()
        binary = self.root / "build/armor_auto"
        binary.write_text("fixture, never executed\n", encoding="utf-8")
        binary.chmod(0o755)
        (self.root / "config/detector.yaml").write_text("brightness: 100\n", encoding="utf-8")
        self.write_camera()

    def write_camera(self, **overrides):
        camera = dict(device=self.config.camera_device, width=640, height=480,
                      fps=30, format="YUYV")
        camera.update(overrides)
        # JSON is a valid YAML document and avoids a test-time YAML writer.
        (self.root / "config/armor_auto.yaml").write_text(json.dumps({
            "camera": camera, "detector_config": "detector.yaml"}), encoding="utf-8")

    def test_valid_configuration_roundtrip_and_mission_parameters(self):
        loaded = self.load(asdict(self.config))
        self.assertEqual(loaded, self.config)
        mission = loaded.mission_config()
        for name in mission.__dataclass_fields__:
            self.assertEqual(getattr(mission, name), getattr(loaded, name))
        self.assertEqual(loaded.project_path("config/armor_auto.yaml"),
                         self.root / "config/armor_auto.yaml")
        self.assertEqual(loaded.project_path("/tmp/absolute"), Path("/tmp/absolute"))

    def test_null_for_every_field_is_rejected(self):
        data = asdict(self.config)
        for name in data:
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.load({**data, name: None})

    def test_strict_boolean_integer_and_numeric_types(self):
        data = asdict(self.config)
        bad = {
            "auto_arm": [False, 1, "true"],
            "camera_forward_unmirrored_confirmed": [False, 1, "true"],
            "require_estimator_status": [False, 1, "true"],
            "target_system": [True, 1.0, "1", 0, 256],
            "target_component": [True, 1.0, 0, 256],
            "source_system": [True, 245.0, 0, 256, self.config.target_system],
            "source_component": [True, 191.0, 0, 256],
            "width": [True, 640.0, "640", 15, 4097],
            "height": [True, 480.0, "480", 15, 4097],
            "baud": [True, 921600.0, "921600", 9600],
            "yaw_direction": [True, 1.0, "1", 0, 2],
            "target_loss_action": [False, "return", "", {}],
        }
        for name, values in bad.items():
            for value in values:
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    self.load({**data, name: value})
        for name in ("flight_height_m", "target_loss_timeout_s", "mission_duration_s",
                     "max_yaw_rate_rad_s", "yaw_gain_rad_s", "deadband_normalized",
                     "takeoff_speed_m_s", "ready_timeout_s", "landing_timeout_s",
                     "telemetry_timeout_s", "heartbeat_timeout_s", "prestream_s"):
            for value in (True, "1", [], float("nan"), float("inf"), -1, 10**1000):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    self.load({**data, name: value})

    def test_missing_unknown_duplicate_and_nonobject_json(self):
        data = asdict(self.config)
        for invalid in ([1], None, "config", {**data, "typo_width": 640}):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                self.load(invalid)
        del data["flight_height_m"]
        with self.assertRaises(ValueError):
            self.load(data)
        encoded = json.dumps(asdict(self.config))
        for field, value in (("width", 320), ("auto_arm", False)):
            self.json_path.write_text(encoded[:-1] + f', "{field}": {json.dumps(value)}}}',
                                      encoding="utf-8")
            with self.subTest(duplicate=field), self.assertRaisesRegex(ValueError, "duplicate"):
                RuntimeConfig.load(self.json_path)

    def test_paths_must_be_concrete_and_devices_distinct(self):
        data = asdict(self.config)
        for name in ("project_dir", "vision_binary", "vision_config", "camera_device", "connection"):
            for value in ("", "REPLACE_ME", "/tmp/REPLACE_ME/path", "/dev/ttyS?", "a\nb", 1):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    self.load({**data, name: value})
        for name, value in (("project_dir", "relative"), ("camera_device", "/tmp/camera"),
                            ("connection", "ttyACM0"), ("connection", self.config.camera_device)):
            with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                self.load({**data, name: value})

    def test_duration_and_prestream_are_accepted_by_mission_too(self):
        data = asdict(self.config)
        with self.assertRaises(ValueError):
            self.load({**data, "mission_duration_s": 10})  # 5 s climb + 5 s margin.
        for prestream in (1.1, 1.9, 1.999):
            with self.subTest(prestream=prestream), self.assertRaises(ValueError):
                self.load({**data, "prestream_s": prestream})
        loaded = self.load({**data, "prestream_s": 2.0})
        self.assertEqual(loaded.mission_config().prestream_s, 2.0)

    def test_target_loss_hold_is_accepted_and_forwarded_to_mission(self):
        loaded = self.load({**asdict(self.config), "target_loss_action": "hold"})
        self.assertEqual(loaded.target_loss_action, "hold")
        self.assertEqual(loaded.mission_config().target_loss_action, "hold")

    @unittest.skipUnless(importlib.util.find_spec("yaml"), "PyYAML is not installed")
    def test_json_camera_yaml_must_match_values_and_types(self):
        self.make_vision_files()
        self.config.check_files()
        for changes in ({"device": "/dev/other-camera"}, {"width": 320}, {"height": 240},
                        {"width": 640.0}, {"height": "480"}, {"format": "RGB"},
                        {"fps": True}, {"fps": 30.0}, {"fps": 0}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.write_camera(**changes)
                self.config.check_files()
        self.write_camera()
        (self.root / "config/detector.yaml").unlink()
        with self.assertRaisesRegex(ValueError, "detector_config"):
            self.config.check_files()

    @unittest.skipUnless(importlib.util.find_spec("yaml"), "PyYAML is not installed")
    def test_check_files_requires_project_binary_and_camera_mapping(self):
        self.make_vision_files()
        binary = self.root / "build/armor_auto"
        binary.chmod(0o644)
        with self.assertRaises(ValueError):
            self.config.check_files()
        binary.chmod(0o755)
        path = self.root / "config/armor_auto.yaml"
        for content in ("null", "[]", "{}", '{"camera":null}'):
            path.write_text(content, encoding="utf-8")
            with self.subTest(content=content), self.assertRaises(ValueError):
                self.config.check_files()
        with self.assertRaises(ValueError):
            replace(self.config, project_dir=str(self.root / "missing")).check_files()

    @unittest.skipUnless(importlib.util.find_spec("yaml"), "PyYAML is not installed")
    def test_check_config_main_never_opens_transport_or_starts_vision(self):
        self.make_vision_files()
        self.load(asdict(self.config))
        forbidden = Mock(side_effect=AssertionError("check-config must not start hardware"))
        with patch.object(yaw_follow, "run", forbidden), \
                patch.object(yaw_follow.subprocess, "Popen", forbidden), \
                patch.object(VisionPipe, "start", forbidden), \
                patch.object(yaw_follow.os, "open", forbidden), \
                patch("socket.socket", forbidden), \
                patch("importlib.util.find_spec", return_value=object()), \
                patch.object(yaw_follow, "log") as logged:
            self.assertEqual(yaw_follow.main(["--config", str(self.json_path), "--check-config"]), 0)
        forbidden.assert_not_called()
        logged.assert_called_once_with("configuration_ok", project_dir=self.config.project_dir,
                                       hardware_opened=False)


class VisionPipeTests(unittest.TestCase):
    def setUp(self):
        self.pipe = VisionPipe(runtime_config())

    def report(self):
        return dict(source="automatic_armor_rgb_pnp", coordinate_frame="rgb_optical",
                    detection_valid=True, tracking_confirmed=True, generation=1, frame=3,
                    track_id=1, sample_time_monotonic_s=10.0, frame_age_ms=100,
                    armor_corners_px=[[400, 100], [560, 100], [560, 200], [400, 200]],
                    valid=False, reason="rgb_calibration_missing")

    def test_no_pnp_still_supplies_confirmed_fresh_horizontal_sample(self):
        self.pipe.accept(self.report(), 10.1)
        self.assertIsNotNone(self.pipe.latest)
        self.assertAlmostEqual(self.pipe.latest.error_x, 0.5)
        self.assertEqual(self.pipe.last_frame_at, 10.0)

    def test_invalid_or_stale_report_immediately_clears_cached_target(self):
        changes = ({"detection_valid": False}, {"tracking_confirmed": False},
                   {"sample_time_monotonic_s": 9.0}, {"sample_time_monotonic_s": 11.0},
                   {"frame_age_ms": 251}, {"frame_age_ms": float("nan")},
                   {"reason": "stopped"}, {"reason": "camera_failed"},
                   {"source": "other"}, {"coordinate_frame": "other"},
                   {"armor_corners_px": [[-1, 0]] * 4}, {"frame": 2},
                   {"generation": 0}, {"frame": True}, {"track_id": 0})
        for change in changes:
            with self.subTest(change=change):
                self.pipe = VisionPipe(runtime_config())
                self.pipe.accept(self.report(), 10.1)
                self.assertIsNotNone(self.pipe.latest)
                self.pipe.accept({**self.report(), **change}, 10.1)
                self.assertIsNone(self.pipe.latest)
        self.pipe.accept(self.report(), 10.1)
        self.pipe.accept([], 10.1)
        self.assertIsNone(self.pipe.latest)

    def test_stale_report_does_not_refresh_frame_watchdog(self):
        self.pipe.accept(self.report(), 10.1)
        self.pipe.accept({**self.report(), "frame": 4}, 11.0)
        self.assertIsNone(self.pipe.latest)
        self.assertEqual(self.pipe.last_frame_at, 10.0)


class RuntimeFailureTests(unittest.TestCase):
    def test_log_sink_oserror_does_not_escape(self):
        for error in (BrokenPipeError("log consumer closed"), OSError("journal unavailable")):
            with self.subTest(error=error), patch("builtins.print", side_effect=error) as output:
                yaw_follow.log("fixture_event", value=1)
            output.assert_called_once()

    def test_wait_devices_retries_transient_stat_errors_without_opening(self):
        for error in (FileNotFoundError("device removed"), OSError("device not ready")):
            forbidden = Mock(side_effect=AssertionError("device checks must not open hardware"))
            character = SimpleNamespace(st_mode=yaw_follow.stat.S_IFCHR)
            with self.subTest(error=error), \
                    patch.object(yaw_follow.time, "monotonic", side_effect=[0.0, 0.0, 0.2]), \
                    patch.object(yaw_follow.time, "sleep") as slept, \
                    patch.object(yaw_follow.os, "access", return_value=True), \
                    patch.object(Path, "stat", side_effect=[error, character, character]) as checked, \
                    patch.object(yaw_follow.os, "open", forbidden), patch("socket.socket", forbidden):
                self.assertTrue(yaw_follow.wait_devices(runtime_config(), lambda: False))
            self.assertEqual(checked.call_count, 3)
            slept.assert_called_once_with(0.2)
            forbidden.assert_not_called()

    def test_wait_devices_stat_error_still_obeys_timeout(self):
        with patch.object(yaw_follow.time, "monotonic", side_effect=[0.0, 0.0, 61.0]), \
                patch.object(yaw_follow.time, "sleep") as slept, \
                patch.object(yaw_follow.os, "access", return_value=True), \
                patch.object(Path, "stat", side_effect=FileNotFoundError("not enumerated")):
            self.assertFalse(yaw_follow.wait_devices(runtime_config(), lambda: False))
        slept.assert_called_once_with(0.2)

    def test_error_fallback_uses_latest_heartbeat_and_respects_pilot_takeover(self):
        # An ARM action was sent in the preceding iteration. The next poll
        # updates the heartbeat before a later read fails. Cached state still
        # says Offboard, so only consulting the newest heartbeat avoids LAND
        # overriding a pilot's newly selected mode.
        for latest_mode, should_land in ((6 << 16, True),
                                         ((4 << 16) | (6 << 24), True),
                                         (3 << 16, False),
                                         ((4 << 16) | (4 << 24), False)):
            with self.subTest(latest_mode=latest_mode):
                master = Mock()
                master.port.fileno.return_value = 42
                mavutil = SimpleNamespace(mavlink=SimpleNamespace(MAV_CMD_NAV_LAND=21),
                                          mavlink_connection=Mock(return_value=master))
                package = ModuleType("pymavlink")
                package.mavutil = mavutil
                vision = Mock(latest=None)
                link = Mock(m=mavutil.mavlink, fault=None, heartbeat_at=10.0)
                link.heartbeat = SimpleNamespace(custom_mode=6 << 16)
                link.state.return_value = SimpleNamespace(
                    telemetry_ready=True, armed=False, offboard=True,
                    position=(1.0, 2.0, -1.0))
                polls = []

                def poll(now):
                    polls.append(now)
                    if len(polls) == 2:
                        link.heartbeat = SimpleNamespace(custom_mode=latest_mode)
                        link.heartbeat_at = now
                        raise OSError("read failed after heartbeat update")

                link.poll.side_effect = poll
                mission = Mock(phase="WAIT_ARM")
                mission.tick.return_value = Action(request_arm=True, reason="arm_requested")
                with patch.dict(sys.modules, {"pymavlink": package}), \
                        patch.object(yaw_follow, "wait_devices", return_value=True), \
                        patch.object(yaw_follow, "VisionPipe", return_value=vision), \
                        patch.object(yaw_follow, "Px4Link", return_value=link), \
                        patch.object(yaw_follow, "YawMission", return_value=mission), \
                        patch.object(yaw_follow.signal, "signal"), \
                        patch.object(yaw_follow.fcntl, "ioctl"), \
                        patch.object(yaw_follow.time, "monotonic", return_value=10.0), \
                        patch.object(yaw_follow.time, "sleep"), \
                        patch.object(yaw_follow, "log"), \
                        patch.object(yaw_follow.os, "open", side_effect=AssertionError("hardware IO")), \
                        patch("socket.socket", side_effect=AssertionError("socket IO")), \
                        patch.object(yaw_follow.subprocess, "Popen", side_effect=AssertionError("spawn")):
                    self.assertEqual(yaw_follow.run(runtime_config()), 1)
                self.assertEqual(len(polls), 2)
                mission.tick.assert_called_once()
                link.send_action.assert_called_once()
                self.assertTrue(link.send_action.call_args.args[0].request_arm)
                if should_land:
                    arguments = link.command.call_args.args
                    self.assertEqual(arguments[:4], (21, 0, 0, 0))
                    self.assertTrue(all(math.isnan(value) for value in arguments[4:]))
                else:
                    link.command.assert_not_called()
                vision.close.assert_called_once()
                master.close.assert_called_once()


class FakeMessage:
    def __init__(self, kind, *, source_system=1, source_component=1, **values):
        self.kind = kind
        self.source_system = source_system
        self.source_component = source_component
        self.__dict__.update(values)

    def get_type(self):
        return self.kind

    def get_srcSystem(self):
        return self.source_system

    def get_srcComponent(self):
        return self.source_component


class FakeMaster:
    def __init__(self):
        self.mav = Mock()
        self.messages = deque()

    def recv_match(self, *, blocking):
        if blocking:
            raise AssertionError("runtime poll must not block")
        return self.messages.popleft() if self.messages else None


@unittest.skipUnless(importlib.util.find_spec("pymavlink"), "pymavlink is not installed")
class Px4LinkTests(unittest.TestCase):
    def setUp(self):
        # Only generated protocol constants are imported. No mavutil/transport factory.
        from pymavlink.dialects.v20 import common
        self.m = common
        self.master = FakeMaster()
        self.link = Px4Link(runtime_config(), self.master, self.m)
        self.vision = SimpleNamespace(alive=True, last_frame_at=10.0)

    def position(self, boot_ms=100, **overrides):
        values = dict(time_boot_ms=boot_ms, x=1.0, y=2.0, z=-1.0, vx=0.0, vy=0.0, vz=0.0)
        values.update(overrides)
        return FakeMessage("LOCAL_POSITION_NED", **values)

    def healthy_flags(self):
        return (self.m.ESTIMATOR_ATTITUDE | self.m.ESTIMATOR_VELOCITY_HORIZ |
                self.m.ESTIMATOR_VELOCITY_VERT | self.m.ESTIMATOR_POS_VERT_ABS |
                self.m.ESTIMATOR_POS_HORIZ_REL)

    def healthy(self):
        self.link.accept(FakeMessage("HEARTBEAT", autopilot=self.m.MAV_AUTOPILOT_PX4,
                                     type=self.m.MAV_TYPE_QUADROTOR, base_mode=0,
                                     custom_mode=0), 10.0)
        self.link.accept(self.position(), 10.0)
        self.link.accept(FakeMessage("ESTIMATOR_STATUS", flags=self.healthy_flags()), 10.0)
        self.link.accept(FakeMessage("EXTENDED_SYS_STATE",
                                     landed_state=self.m.MAV_LANDED_STATE_ON_GROUND), 10.0)

    def test_finite_local_position_and_velocity_only(self):
        for field in ("x", "y", "z", "vx", "vy", "vz"):
            for bad in (float("nan"), float("inf"), True, "1", None):
                with self.subTest(field=field, bad=bad):
                    link = Px4Link(runtime_config(), FakeMaster(), self.m)
                    link.accept(self.position(), 10.0)
                    link.accept(self.position(200, **{field: bad}), 10.1)
                    self.assertEqual(link.fault, "nonfinite_local_position")
                    self.assertEqual(link.position_at, 10.0)
                    self.assertEqual(link.position, (1.0, 2.0, -1.0))

    def test_duplicate_and_clock_rollback_do_not_refresh_position(self):
        self.link.accept(self.position(1000), 10.0)
        self.link.accept(self.position(1000, x=5), 10.1)
        self.assertEqual(self.link.position_at, 10.0)
        self.assertEqual(self.link.position, (1.0, 2.0, -1.0))
        self.assertIsNone(self.link.fault)
        self.link.accept(self.position(900), 10.2)
        self.assertEqual(self.link.position_at, 10.0)
        self.assertEqual(self.link.boot_ms, 1000)
        self.assertEqual(self.link.fault, "flight_controller_clock_reset")

    def test_boot_counter_wraparound_and_position_discontinuity(self):
        self.link.accept(self.position(0xFFFFFFF0), 10.0)
        self.link.accept(self.position(34), 10.05)
        self.assertEqual(self.link.position_at, 10.05)
        self.assertIsNone(self.link.fault)
        self.link.accept(self.position(84, x=2.0), 10.1)
        self.assertEqual(self.link.fault, "local_position_discontinuity")
        self.assertEqual(self.link.position_at, 10.05)

    def test_multiple_positions_in_same_poll_detect_discontinuity(self):
        self.master.messages.extend((self.position(100, x=0.01),
                                     self.position(150, x=10.0)))
        with patch.object(yaw_follow.time, "monotonic", return_value=0.0):
            self.link.poll(10.0)
        self.assertEqual(self.link.fault, "local_position_discontinuity")
        self.assertEqual(self.link.position, (0.01, 2.0, -1.0))
        self.assertEqual(self.link.boot_ms, 100)

    def test_estimator_flags_and_freshness_gate_position_readiness(self):
        self.healthy()
        state = self.link.state(10.1, self.vision)
        self.assertTrue(state.telemetry_ready)
        self.assertTrue(state.heartbeat_fresh)
        self.assertTrue(state.on_ground)
        flags = self.healthy_flags()
        for flag in (self.m.ESTIMATOR_ATTITUDE, self.m.ESTIMATOR_VELOCITY_HORIZ,
                     self.m.ESTIMATOR_VELOCITY_VERT, self.m.ESTIMATOR_POS_VERT_ABS,
                     self.m.ESTIMATOR_POS_HORIZ_REL):
            self.link.estimator_flags = flags & ~flag
            with self.subTest(missing_flag=flag):
                self.assertFalse(self.link.state(10.1, self.vision).telemetry_ready)
        for flag in (self.m.ESTIMATOR_CONST_POS_MODE, self.m.ESTIMATOR_ACCEL_ERROR):
            self.link.estimator_flags = flags | flag
            with self.subTest(invalid_flag=flag):
                self.assertFalse(self.link.state(10.1, self.vision).telemetry_ready)
        self.link.estimator_flags = flags
        self.assertFalse(self.link.state(10.6, self.vision).telemetry_ready)
        self.link.position_at = 10.6
        self.assertFalse(self.link.state(10.6, self.vision).telemetry_ready)
        self.link.estimator_at = 10.6
        self.assertTrue(self.link.state(10.6, self.vision).telemetry_ready)
        self.link.fault = "estimator_origin_reset"
        self.assertFalse(self.link.state(10.6, self.vision).telemetry_ready)

    def test_horizontal_absolute_estimate_also_satisfies_gate(self):
        self.healthy()
        self.link.estimator_flags = (self.healthy_flags() & ~self.m.ESTIMATOR_POS_HORIZ_REL |
                                     self.m.ESTIMATOR_POS_HORIZ_ABS)
        self.assertTrue(self.link.state(10.1, self.vision).telemetry_ready)

    def test_wrong_source_messages_are_ignored_by_poll(self):
        for identity in ((2, 1), (1, 2)):
            position = self.position()
            position.source_system, position.source_component = identity
            self.master.messages.append(position)
        with patch.object(yaw_follow.time, "monotonic", return_value=0.0):
            self.link.poll(10.0)
        self.assertIsNone(self.link.position)
        self.assertEqual(self.link.position_at, -math.inf)
        self.master.messages.append(self.position())
        with patch.object(yaw_follow.time, "monotonic", return_value=0.0):
            self.link.poll(10.1)
        self.assertEqual(self.link.position, (1.0, 2.0, -1.0))
        self.assertEqual(self.link.position_at, 10.1)

    def test_send_action_keeps_xyz_and_enables_only_yaw_rate(self):
        action = Action(position=(1.0, 2.0, -3.0), yaw_rate=0.2)
        self.link.send_action(action, 10.0)
        arguments = self.master.mav.set_position_target_local_ned_send.call_args.args
        self.assertEqual(arguments[:4], (10000, 1, 1, self.m.MAV_FRAME_LOCAL_NED))
        mask = arguments[4]
        self.assertEqual(arguments[5:8], (1.0, 2.0, -3.0))
        self.assertEqual(arguments[8:15], (0, 0, 0, 0, 0, 0, 0))
        self.assertEqual(arguments[15], 0.2)
        for flag in (self.m.POSITION_TARGET_TYPEMASK_X_IGNORE, self.m.POSITION_TARGET_TYPEMASK_Y_IGNORE,
                     self.m.POSITION_TARGET_TYPEMASK_Z_IGNORE, self.m.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE):
            self.assertFalse(mask & flag)
        for flag in (self.m.POSITION_TARGET_TYPEMASK_VX_IGNORE, self.m.POSITION_TARGET_TYPEMASK_VY_IGNORE,
                     self.m.POSITION_TARGET_TYPEMASK_VZ_IGNORE, self.m.POSITION_TARGET_TYPEMASK_AX_IGNORE,
                     self.m.POSITION_TARGET_TYPEMASK_AY_IGNORE, self.m.POSITION_TARGET_TYPEMASK_AZ_IGNORE,
                     self.m.POSITION_TARGET_TYPEMASK_YAW_IGNORE):
            self.assertTrue(mask & flag)

    def test_mode_arm_and_land_command_arguments(self):
        with patch.object(yaw_follow, "log"):
            self.link.send_action(Action(request_mode=True, request_arm=True,
                                         request_land=True, reason="test"), 10.0)
        self.master.mav.set_mode_send.assert_called_once_with(
            1, self.m.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6 << 16)
        calls = self.master.mav.command_long_send.call_args_list
        self.assertEqual(len(calls), 2)
        self.assertEqual(calls[0].args, (1, 1, self.m.MAV_CMD_COMPONENT_ARM_DISARM,
                                       0, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertEqual(calls[1].args[:7], (1, 1, self.m.MAV_CMD_NAV_LAND, 0, 0, 0, 0))
        self.assertTrue(all(math.isnan(value) for value in calls[1].args[7:]))
        self.master.mav.set_position_target_local_ned_send.assert_not_called()


if __name__ == "__main__":
    unittest.main()
