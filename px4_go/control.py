"""Bounded, transport-free NED setpoint state machine and MAVLink test peer.

The peer is a protocol fixture, not PX4, an EKF, or a flight dynamics simulator.
Only pymavlink's generated common dialect is used; mavutil/device APIs are not.
"""
from collections import deque
from dataclasses import dataclass
from enum import Enum
import math


OFFBOARD_CUSTOM_MODE = 6 << 16


def _dialect():
    # Lazy import keeps importing this module free of third-party initialization.
    from pymavlink.dialects.v20 import common
    return common


@dataclass(frozen=True)
class ControlConfig:
    rate_hz: float = 20.0
    prefill_cycles: int = 100
    origin_ned: tuple = (0.0, 0.0, 0.0)
    height_above_origin_m: float = 2.0
    heartbeat_timeout_s: float = 2.5
    handshake_timeout_s: float = 5.0
    own_system: int = 42
    own_component: int = 191
    target_system: int = 1
    target_component: int = 1

    def __post_init__(self):
        positive = (self.rate_hz, self.height_above_origin_m,
                    self.heartbeat_timeout_s, self.handshake_timeout_s)
        if any(not math.isfinite(v) or v <= 0 for v in positive):
            raise ValueError("Control rates, heights and timeouts must be finite and positive")
        if not isinstance(self.prefill_cycles, int) or not 1 <= self.prefill_cycles <= 100000:
            raise ValueError("prefill_cycles must be 1..100000")
        if len(self.origin_ned) != 3 or not all(math.isfinite(v) for v in self.origin_ned):
            raise ValueError("origin_ned must contain three finite NED coordinates")
        ids = (self.own_system, self.own_component,
               self.target_system, self.target_component)
        if any(not isinstance(v, int) or not 1 <= v <= 255 for v in ids):
            raise ValueError("MAVLink identities must be 1..255")
        if ids[:2] == ids[2:]:
            raise ValueError("Controller and peer identities must differ")

    @property
    def target_ned(self):
        x, y, z = self.origin_ned
        return (x, y, z - self.height_above_origin_m)


def position_mask(m):
    """The position-only mask used by the original gogogo.py."""
    return (m.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            m.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            m.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            m.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            m.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            m.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            m.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
            m.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)


class MemoryCodec:
    """MAVLink v2 byte encoding/decoding without a file, socket, or serial port."""
    def __init__(self, system, component):
        self.m = _dialect()
        self.encoder = self.m.MAVLink(None, srcSystem=system, srcComponent=component)
        self.decoder = self.m.MAVLink(None)
        self.encoded = 0
        self.decoded = 0
        self.encoded_bytes = 0

    def encode(self, message):
        frame = message.pack(self.encoder)
        self.encoder.seq = (self.encoder.seq + 1) & 255
        self.encoded += 1
        self.encoded_bytes += len(frame)
        return frame

    def decode(self, data):
        if not isinstance(data, bytes) or len(data) > 4096:
            raise ValueError("An inbound chunk must be bytes of at most 4096 bytes")
        messages = self.decoder.parse_buffer(data) or []
        self.decoded += len(messages)
        return messages


class Phase(str, Enum):
    PREFILL = "prefill"
    WAIT_OFFBOARD = "wait_offboard"
    WAIT_ARM = "wait_arm"
    HOVER = "hover"
    LANDING = "landing"
    COMPLETE = "complete"
    FAILED = "failed"


class PositionController:
    """A single-reader controller with explicit time and an in-memory byte outbox."""
    def __init__(self, config=None):
        self.config = config or ControlConfig()
        c = self.config
        self.codec = MemoryCodec(c.own_system, c.own_component)
        self.m = self.codec.m
        self.phase = Phase.PREFILL
        self.reason = "prefilling"
        self.entered_s = None
        self.last_now = None
        self.started_s = None
        self.last_heartbeat_s = None
        self.last_heartbeat = None
        self.last_position = None
        self.last_ack = None
        self.status_history = deque(maxlen=20)
        self.outbox = deque(maxlen=64)
        self.setpoints_sent = 0
        self.foreign_messages = 0
        self.accepted_messages = 0
        self.handshake_completed = False

    def _time(self, now):
        if not math.isfinite(now) or now < 0 or (self.last_now is not None and now < self.last_now):
            raise ValueError("Controller time must be finite and monotonic")
        self.last_now = now
        if self.started_s is None:
            self.started_s = now
            self.entered_s = now

    def _queue(self, message):
        if len(self.outbox) == self.outbox.maxlen:
            raise RuntimeError("Controller outbox must be drained once per cycle")
        self.outbox.append(self.codec.encode(message))

    def drain(self):
        frames = list(self.outbox)
        self.outbox.clear()
        return frames

    def _enter(self, phase, now, reason):
        self.phase, self.entered_s, self.reason = phase, now, reason

    def _armed(self):
        return self.last_heartbeat is not None and bool(
            self.last_heartbeat.base_mode & self.m.MAV_MODE_FLAG_SAFETY_ARMED)

    def receive(self, data, now):
        self._time(now)
        c = self.config
        for message in self.codec.decode(data):
            if (message.get_srcSystem(), message.get_srcComponent()) != (
                    c.target_system, c.target_component):
                self.foreign_messages += 1
                continue
            self.accepted_messages += 1
            kind = message.get_type()
            if kind == "HEARTBEAT":
                if message.autopilot != self.m.MAV_AUTOPILOT_PX4:
                    continue
                self.last_heartbeat, self.last_heartbeat_s = message, now
            elif kind == "LOCAL_POSITION_NED":
                self.last_position = message
            elif kind == "COMMAND_ACK":
                # Keep one ACK; never let an unrelated command replace it.
                if message.command in (self.m.MAV_CMD_COMPONENT_ARM_DISARM,
                                       self.m.MAV_CMD_NAV_LAND):
                    self.last_ack = (message.command, message.result)
            elif kind == "STATUSTEXT":
                text = message.text
                if isinstance(text, bytes):
                    text = text.decode("utf-8", errors="replace")
                self.status_history.append(str(text).rstrip("\x00")[:256])

    def _send_position(self, now):
        c = self.config
        stamp = int((now - self.started_s) * 1000) & 0xFFFFFFFF
        self._queue(self.codec.encoder.set_position_target_local_ned_encode(
            stamp, c.target_system, c.target_component,
            self.m.MAV_FRAME_LOCAL_NED, position_mask(self.m),
            *c.target_ned, 0, 0, 0, 0, 0, 0, 0, 0))
        self.setpoints_sent += 1

    def _command(self, command, first=0):
        c = self.config
        self._queue(self.codec.encoder.command_long_encode(
            c.target_system, c.target_component, command, 0,
            first, 0, 0, 0, 0, 0, 0))

    def step(self, now):
        """Send at most one position target; the caller owns periodic scheduling."""
        self._time(now)
        if self.phase in (Phase.COMPLETE, Phase.FAILED):
            return
        c = self.config
        heartbeat_age = now - (self.last_heartbeat_s if self.last_heartbeat_s is not None
                               else self.started_s)
        if heartbeat_age > c.heartbeat_timeout_s:
            self._enter(Phase.FAILED, now, "heartbeat_timeout")
            return
        if self.phase not in (Phase.PREFILL, Phase.HOVER) and now - self.entered_s > c.handshake_timeout_s:
            self._enter(Phase.FAILED, now, "handshake_timeout")
            return
        if self.phase == Phase.WAIT_OFFBOARD and self.last_heartbeat is not None:
            if self.last_heartbeat.custom_mode == OFFBOARD_CUSTOM_MODE:
                self.last_ack = None
                self._command(self.m.MAV_CMD_COMPONENT_ARM_DISARM, 1)
                self._enter(Phase.WAIT_ARM, now, "awaiting_arm")
        elif self.phase == Phase.WAIT_ARM:
            if self.last_ack is not None and self.last_ack[0] == self.m.MAV_CMD_COMPONENT_ARM_DISARM:
                if self.last_ack[1] != self.m.MAV_RESULT_ACCEPTED:
                    self._enter(Phase.FAILED, now, "arm_rejected")
                    return
            if self._armed() and self.last_heartbeat.custom_mode == OFFBOARD_CUSTOM_MODE:
                self.handshake_completed = True
                self._enter(Phase.HOVER, now, "holding_fixed_ned_target")
        elif self.phase == Phase.HOVER:
            if not self._armed() or self.last_heartbeat.custom_mode != OFFBOARD_CUSTOM_MODE:
                self._enter(Phase.FAILED, now, "offboard_or_arm_lost")
                return
        elif self.phase == Phase.LANDING:
            if self.last_ack is not None and self.last_ack[0] == self.m.MAV_CMD_NAV_LAND:
                if self.last_ack[1] != self.m.MAV_RESULT_ACCEPTED:
                    self._enter(Phase.FAILED, now, "land_rejected")
                    return
            if not self._armed():
                self._enter(Phase.COMPLETE, now, "simulated_land_disarmed")
                return
        self._send_position(now)
        if self.phase == Phase.PREFILL and self.setpoints_sent >= c.prefill_cycles:
            self._queue(self.codec.encoder.set_mode_encode(
                c.target_system, self.m.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                OFFBOARD_CUSTOM_MODE))
            self._enter(Phase.WAIT_OFFBOARD, now, "awaiting_offboard")

    def finish(self, now):
        """Request LAND from the protocol peer, without claiming physical landing."""
        self._time(now)
        if self.phase in (Phase.COMPLETE, Phase.FAILED, Phase.LANDING):
            return
        self.last_ack = None
        self._command(self.m.MAV_CMD_NAV_LAND)
        self._enter(Phase.LANDING, now, "awaiting_simulated_disarm")


class SimulatedPeer:
    """Deterministic MAVLink handshake peer with static origin, no flight physics."""
    def __init__(self, config=None):
        self.config = config or ControlConfig()
        c = self.config
        self.codec = MemoryCodec(c.target_system, c.target_component)
        self.m = self.codec.m
        self.mode = 0
        self.armed = False
        self.setpoints_received = 0
        self.arm_commands = 0
        self.land_commands = 0
        self.last_target = None
        self.last_heartbeat_s = None
        self.last_telemetry_s = None

    def heartbeat(self):
        flags = self.m.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if self.armed:
            flags |= self.m.MAV_MODE_FLAG_SAFETY_ARMED
        return self.codec.encode(self.codec.encoder.heartbeat_encode(
            self.m.MAV_TYPE_QUADROTOR, self.m.MAV_AUTOPILOT_PX4,
            flags, self.mode, self.m.MAV_STATE_ACTIVE, 3))

    def tick(self, now):
        frames = []
        if self.last_heartbeat_s is None or now - self.last_heartbeat_s >= 1.0:
            frames.append(self.heartbeat())
            self.last_heartbeat_s = now
        if self.last_telemetry_s is None or now - self.last_telemetry_s >= .1:
            # Static coordinates intentionally do not follow the commanded target.
            frames.append(self.codec.encode(self.codec.encoder.local_position_ned_encode(
                int(now * 1000) & 0xFFFFFFFF, *self.config.origin_ned, 0, 0, 0)))
            frames.append(self.codec.encode(self.codec.encoder.attitude_encode(
                int(now * 1000) & 0xFFFFFFFF, 0, 0, 0, 0, 0, 0)))
            self.last_telemetry_s = now
        return frames

    def receive(self, data, now):
        frames = []
        c = self.config
        for message in self.codec.decode(data):
            if (message.get_srcSystem(), message.get_srcComponent()) != (
                    c.own_system, c.own_component):
                continue
            if getattr(message, "target_system", None) != c.target_system:
                continue
            if getattr(message, "target_component", c.target_component) != c.target_component:
                continue
            kind = message.get_type()
            if kind == "SET_POSITION_TARGET_LOCAL_NED":
                if message.coordinate_frame != self.m.MAV_FRAME_LOCAL_NED or message.type_mask != position_mask(self.m):
                    raise ValueError("Unexpected frame or setpoint mask")
                target = (message.x, message.y, message.z)
                if not all(math.isfinite(v) for v in target):
                    raise ValueError("Non-finite target")
                self.last_target = target
                self.setpoints_received += 1
            elif kind == "SET_MODE":
                if self.setpoints_received >= c.prefill_cycles and message.custom_mode == OFFBOARD_CUSTOM_MODE:
                    self.mode = OFFBOARD_CUSTOM_MODE
                    frames.append(self.heartbeat())
            elif kind == "COMMAND_LONG":
                result = self.m.MAV_RESULT_UNSUPPORTED
                if message.command == self.m.MAV_CMD_COMPONENT_ARM_DISARM:
                    self.arm_commands += 1
                    if self.mode == OFFBOARD_CUSTOM_MODE and message.param1 == 1:
                        self.armed = True
                        result = self.m.MAV_RESULT_ACCEPTED
                    else:
                        result = self.m.MAV_RESULT_DENIED
                elif message.command == self.m.MAV_CMD_NAV_LAND:
                    self.land_commands += 1
                    self.armed = False
                    self.mode = 0
                    result = self.m.MAV_RESULT_ACCEPTED
                frames.append(self.codec.encode(self.codec.encoder.command_ack_encode(
                    message.command, result)))
                frames.append(self.heartbeat())
        return frames


def exchange(controller, peer, now):
    """One bounded scheduler cycle; all communication remains ordinary bytes."""
    for frame in peer.tick(now):
        controller.receive(frame, now)
    controller.step(now)
    for frame in controller.drain():
        for response in peer.receive(frame, now):
            controller.receive(response, now)
