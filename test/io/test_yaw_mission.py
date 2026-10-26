"""Mission decision tests: no serial device, MAVLink sender or flight."""
from dataclasses import replace
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from px4_go.yaw_mission import FlightState, MissionConfig, YawMission


class YawMissionTests(unittest.TestCase):
    def setUp(self):
        self.config = MissionConfig(2.0, 3.0, 120.0)
        self.state = FlightState(True, True, False, False, True, True, True,
                                 (13.0, -4.0, 8.0))
        self.mission = YawMission(self.config, 0.0)

    def tick(self, now, **overrides):
        state_changes = overrides.pop("state", {})
        return self.mission.tick(now, replace(self.state, **state_changes),
                                 yaw_rate=overrides.pop("yaw_rate", 0.25),
                                 target_fresh=overrides.pop("target_fresh", True),
                                 **overrides)

    def arm(self):
        self.tick(0.0)
        self.tick(2.0)
        self.state = replace(self.state, offboard=True)
        action = self.tick(2.1)
        self.assertTrue(action.request_arm)
        self.state = replace(self.state, armed=True)
        self.tick(2.2)
        self.assertEqual(self.mission.phase, "TAKEOFF")

    def follow(self):
        self.arm()
        self.state = replace(self.state, position=(13.0, -4.0, 6.0), on_ground=False)
        self.tick(12.2)
        self.assertEqual(self.mission.phase, "FOLLOW")

    def test_origin_is_captured_and_ground_prefill_has_zero_yaw(self):
        action = self.tick(0.0)
        self.assertEqual(action.position, (13.0, -4.0, 8.0))
        self.assertEqual(action.yaw_rate, 0.0)
        self.assertFalse(self.tick(1.99).request_mode)
        action = self.tick(2.0, state={"position": (13.1, -4.1, 8.1)})
        self.assertEqual(action.position, (13.0, -4.0, 8.0))
        self.assertTrue(action.request_mode)

    def test_mode_retries_once_per_second_and_never_arms_on_timeout(self):
        self.tick(0.0)
        self.tick(2.0)
        self.assertFalse(self.tick(2.99).request_mode)
        self.assertTrue(self.tick(3.0).request_mode)
        action = self.tick(7.0)
        self.assertTrue(action.done)
        self.assertFalse(action.request_arm)
        self.assertFalse(action.request_land)
        self.assertEqual(action.reason, "offboard_timeout")

    def test_ready_requires_all_inputs_including_ground_and_finite_position(self):
        for changes in ({"heartbeat_fresh": False}, {"telemetry_ready": False},
                        {"on_ground": False}, {"vision_alive": False},
                        {"frame_fresh": False}, {"position": None},
                        {"position": (0, 0, float("nan"))}):
            with self.subTest(changes=changes):
                mission = YawMission(self.config, 0)
                action = mission.tick(0, replace(self.state, **changes),
                                      yaw_rate=0, target_fresh=True)
                self.assertIsNone(action.position)
                self.assertEqual(mission.phase, "WAIT_READY")
        self.assertEqual(self.tick(60).reason, "ready_timeout")

    def test_already_armed_start_is_terminal_and_never_takes_over(self):
        action = self.tick(0, state={"armed": True, "offboard": True})
        self.assertTrue(action.done)
        self.assertFalse(action.success)
        self.assertIsNone(action.position)
        self.assertFalse(action.request_land)
        self.assertIs(self.tick(1), action)

    def test_external_arming_before_request_is_not_taken_over(self):
        self.tick(0)
        action = self.tick(1, state={"armed": True})
        self.assertEqual(action.reason, "armed_without_mission_request")
        self.assertFalse(action.request_land)

    def test_ramp_is_relative_to_nonzero_origin_and_waits_for_actual_altitude(self):
        self.arm()
        action = self.tick(7.2)
        self.assertEqual(action.position, (13.0, -4.0, 7.0))
        self.assertEqual(action.yaw_rate, 0)
        action = self.tick(12.2)
        self.assertEqual(action.position, (13.0, -4.0, 6.0))
        self.assertEqual(self.mission.phase, "TAKEOFF")
        self.tick(12.3, state={"position": (13.0, -4.0, 6.14)})
        self.assertEqual(self.mission.phase, "FOLLOW")

    def test_only_follow_uses_yaw_and_never_moves_xy(self):
        self.follow()
        action = self.tick(12.3, yaw_rate=-0.15)
        self.assertEqual(action.position, (13.0, -4.0, 6.0))
        self.assertEqual(action.yaw_rate, -0.15)
        self.assertEqual(self.tick(12.4, yaw_rate=float("nan")).yaw_rate, 0)

    def test_target_loss_stops_yaw_immediately_then_lands(self):
        self.follow()
        action = self.tick(13, target_fresh=False)
        self.assertEqual(action.yaw_rate, 0)
        self.assertFalse(action.request_land)
        self.assertFalse(self.tick(15.99, target_fresh=False).request_land)
        action = self.tick(16, target_fresh=False)
        self.assertTrue(action.request_land)
        self.assertEqual(self.mission.phase, "LANDING")

    def test_reacquired_target_resets_loss_timeout(self):
        self.follow()
        self.tick(13, target_fresh=False)
        self.tick(15.5, target_fresh=True)
        self.assertEqual(self.tick(16, target_fresh=False).yaw_rate, 0)
        self.assertFalse(self.tick(18.9, target_fresh=False).request_land)
        self.assertTrue(self.tick(19, target_fresh=False).request_land)

    def test_hold_target_loss_keeps_position_until_mission_duration(self):
        self.mission = YawMission(replace(self.config, target_loss_action="hold"), 0)
        self.follow()
        self.assertEqual(self.tick(13, target_fresh=False).yaw_rate, 0)
        action = self.tick(30, target_fresh=False)
        self.assertEqual(self.mission.phase, "FOLLOW")
        self.assertFalse(action.request_land)
        self.assertEqual(action.yaw_rate, 0)
        self.assertEqual(action.position, (13.0, -4.0, 6.0))
        # The finite mission duration still applies while waiting for a target.
        action = self.tick(122.2, target_fresh=False)
        self.assertTrue(action.request_land)
        self.assertEqual(action.reason, "mission_duration_reached")

    def test_hold_target_loss_can_resume_yaw_after_reacquisition(self):
        self.mission = YawMission(replace(self.config, target_loss_action="hold"), 0)
        self.follow()
        self.tick(13, target_fresh=False)
        self.tick(30, target_fresh=False)
        action = self.tick(31, target_fresh=True, yaw_rate=-0.1)
        self.assertEqual(action.yaw_rate, -0.1)
        self.assertFalse(action.request_land)

    def test_offboard_takeover_has_no_position_mode_arm_or_land_command(self):
        self.follow()
        action = self.tick(13, state={"offboard": False, "land_mode": True})
        self.assertTrue(action.done)
        self.assertEqual(action.reason, "offboard_taken_over")
        self.assertIsNone(action.position)
        self.assertFalse(action.request_mode or action.request_arm or action.request_land)

    def test_stop_before_arm_has_no_land_or_arm_command(self):
        self.tick(0)
        action = self.tick(1, stop=True)
        self.assertTrue(action.done)
        self.assertFalse(action.request_arm or action.request_land)

    def test_stop_with_arm_in_transit_stays_alive_and_covers_late_confirmation(self):
        self.tick(0)
        self.tick(2)
        self.state = replace(self.state, offboard=True)
        self.tick(2.1)
        action = self.tick(2.2, stop=True)
        self.assertTrue(action.request_land)
        self.assertFalse(action.done)
        self.assertFalse(self.tick(2.3).done)
        self.assertFalse(self.tick(2.4, state={"armed": True}).done)
        action = self.tick(3, state={"offboard": False, "land_mode": True})
        self.assertTrue(action.done)
        self.assertFalse(action.success)
        self.assertIn("stop_requested", action.reason)

    def test_arm_timeout_attempts_land_instead_of_exiting(self):
        self.tick(0)
        self.tick(2)
        self.state = replace(self.state, offboard=True)
        self.tick(2.1)
        action = self.tick(12.1)
        self.assertTrue(action.request_land)
        self.assertFalse(action.done)
        self.assertEqual(self.mission.phase, "LANDING")

    def test_telemetry_or_vision_failure_after_arm_requests_land(self):
        self.arm()
        action = self.tick(3, state={"frame_fresh": False})
        self.assertTrue(action.request_land)
        self.assertEqual(action.yaw_rate, 0)

    def test_heartbeat_loss_attempts_land_but_never_claims_disarm_success(self):
        self.arm()
        action = self.tick(3, state={"heartbeat_fresh": False})
        self.assertTrue(action.request_land)
        self.assertIsNone(action.position)
        self.assertFalse(self.tick(4, state={"heartbeat_fresh": False, "armed": False}).done)
        action = self.tick(48, state={"heartbeat_fresh": False, "armed": False})
        self.assertTrue(action.done)
        self.assertFalse(action.success)
        self.assertIn("landing_timeout", action.reason)

    def test_landing_holds_current_measurement_then_stops_position_in_land_mode(self):
        self.follow()
        action = self.tick(13, state={"position": (14.0, -5.0, 7.0)}, stop=True)
        self.assertEqual(action.position, (14.0, -5.0, 7.0))
        self.assertTrue(action.request_land)
        self.assertFalse(self.tick(13.5).request_land)
        action = self.tick(14, state={"offboard": False, "land_mode": True})
        self.assertIsNone(action.position)
        self.assertTrue(action.request_land)
        action = self.tick(14.1, state={"offboard": False, "land_mode": False})
        self.assertTrue(action.done)
        self.assertFalse(action.request_land)

    def test_mission_duration_begins_at_arm_confirmation_including_takeoff(self):
        self.mission = YawMission(replace(self.config, mission_duration_s=3), 0)
        self.arm()
        self.assertFalse(self.tick(5.19).request_land)
        action = self.tick(5.2)
        self.assertTrue(action.request_land)
        self.assertEqual(action.reason, "mission_duration_reached")
        action = self.tick(5.3, state={"armed": False, "offboard": False,
                                       "land_mode": True})
        self.assertTrue(action.done)
        self.assertTrue(action.success)

    def test_takeoff_timeout_lands_once_without_rearming(self):
        self.arm()
        action = self.tick(32.2)
        self.assertTrue(action.request_land)
        self.assertEqual(action.reason, "takeoff_timeout")
        action = self.tick(33.2)
        self.assertFalse(action.request_arm)

    def test_invalid_config_rejected(self):
        for field, value in (("flight_height_m", 0), ("flight_height_m", float("nan")),
                             ("target_loss_timeout_s", -1),
                             ("mission_duration_s", True), ("prestream_s", 1.9),
                             ("target_loss_action", "LAND"),
                             ("target_loss_action", "hover"),
                             ("target_loss_action", None),
                             ("target_loss_action", True)):
            with self.subTest(field=field, value=value):
                with self.assertRaises(ValueError):
                    replace(self.config, **{field: value})


if __name__ == "__main__":
    unittest.main()
