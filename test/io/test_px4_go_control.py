"""Pure in-memory protocol regression; never execute the supplied flight scripts."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from px4_go.control import (ControlConfig, MemoryCodec, OFFBOARD_CUSTOM_MODE,
                            Phase, PositionController, SimulatedPeer,
                            exchange, position_mask)


@unittest.skipUnless(importlib.util.find_spec("pymavlink"), "pymavlink is not installed")
class ControlTests(unittest.TestCase):
    def setup_pair(self):
        config = ControlConfig(prefill_cycles=3, origin_ned=(2., 3., -1.))
        return PositionController(config), SimulatedPeer(config)

    def test_import_has_no_connection_side_effects(self):
        code = """
import socket, os
from unittest.mock import patch
with patch.object(socket, 'socket', side_effect=AssertionError('socket IO')), \\
     patch.object(os, 'open', side_effect=AssertionError('os.open IO')):
    import px4_go.control
    import px4_go.offline_control
assert 'pymavlink.mavutil' not in __import__('sys').modules
"""
        result = subprocess.run([sys.executable, "-c", code], cwd=ROOT,
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, "")

    def test_handshake_setpoint_and_land_are_real_mavlink_bytes(self):
        controller, peer = self.setup_pair()
        for i in range(8):
            exchange(controller, peer, i * .05)
        self.assertEqual(controller.phase, Phase.HOVER)
        self.assertTrue(controller.handshake_completed)
        self.assertEqual(peer.arm_commands, 1)
        self.assertEqual(peer.last_target, (2., 3., -3.))
        self.assertGreater(controller.codec.decoded, 0)
        controller.finish(.5)
        for frame in controller.drain():
            self.assertEqual(frame[0], 0xFD)  # MAVLink v2 framing.
            for response in peer.receive(frame, .5):
                controller.receive(response, .5)
        controller.step(.5)
        self.assertEqual(controller.phase, Phase.COMPLETE)
        self.assertEqual(peer.land_commands, 1)

    def test_setpoint_serialization_mask_and_origin(self):
        controller, peer = self.setup_pair()
        controller.receive(peer.heartbeat(), 10.)
        controller.step(10.)
        frames = controller.drain()
        decoded = peer.codec.decode(frames[0])[0]
        self.assertEqual(decoded.get_type(), "SET_POSITION_TARGET_LOCAL_NED")
        self.assertEqual(decoded.coordinate_frame, peer.m.MAV_FRAME_LOCAL_NED)
        self.assertEqual(decoded.type_mask, position_mask(peer.m))
        self.assertEqual((decoded.x, decoded.y, decoded.z), (2., 3., -3.))
        self.assertEqual((decoded.vx, decoded.vy, decoded.vz), (0, 0, 0))

    def test_non_heartbeat_messages_do_not_change_heartbeat(self):
        controller, peer = self.setup_pair()
        controller.receive(peer.heartbeat(), 0.)
        heartbeat = controller.last_heartbeat
        frame = peer.codec.encode(peer.codec.encoder.attitude_encode(0, 0, 0, 0, 0, 0, 0))
        controller.receive(frame, .1)
        self.assertIs(controller.last_heartbeat, heartbeat)
        self.assertEqual(controller.last_heartbeat_s, 0.)
        foreign = MemoryCodec(2, 1)
        frame = foreign.encode(foreign.encoder.heartbeat_encode(
            2, foreign.m.MAV_AUTOPILOT_PX4, 128, OFFBOARD_CUSTOM_MODE, 4, 3))
        controller.receive(frame, .2)
        self.assertEqual(controller.foreign_messages, 1)
        self.assertEqual(controller.last_heartbeat_s, 0.)

    def test_heartbeat_loss_stops_setpoints(self):
        controller, peer = self.setup_pair()
        for i in range(8):
            exchange(controller, peer, i * .05)
        before = controller.setpoints_sent
        controller.step(3.)
        self.assertEqual(controller.phase, Phase.FAILED)
        self.assertEqual(controller.reason, "heartbeat_timeout")
        self.assertEqual(controller.setpoints_sent, before)

    def test_status_cache_is_bounded_and_time_cannot_regress(self):
        controller, peer = self.setup_pair()
        for i in range(100):
            frame = peer.codec.encode(peer.codec.encoder.statustext_encode(6, str(i).encode()))
            controller.receive(frame, i * .001)
        self.assertEqual(len(controller.status_history), 20)
        with self.assertRaises(ValueError):
            controller.step(.01)

    def test_core_does_not_use_hardware_factories(self):
        import socket
        from unittest.mock import patch
        with patch.object(socket, "socket", side_effect=AssertionError("socket IO")):
            controller, peer = self.setup_pair()
            for i in range(8):
                exchange(controller, peer, i * .05)
        self.assertEqual(controller.phase, Phase.HOVER)
        self.assertNotIn("pymavlink.mavutil", sys.modules)


if __name__ == "__main__":
    unittest.main()
