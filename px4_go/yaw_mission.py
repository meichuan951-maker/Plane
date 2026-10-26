"""Single-use takeoff, image yaw tracking and landing state machine.

This module has no device access.  The owner supplies fresh flight/vision state
and translates each returned Action into MAVLink messages.  Coordinates are
local NED: climbing decreases z, relative to the actual captured origin.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class MissionConfig:
    flight_height_m: float
    target_loss_timeout_s: float
    mission_duration_s: float
    takeoff_speed_m_s: float = 0.2
    ready_timeout_s: float = 60.0
    landing_timeout_s: float = 45.0
    prestream_s: float = 2.0
    target_loss_action: str = "land"

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            if name == "target_loss_action":
                continue
            value = getattr(self, name)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or value <= 0):
                raise ValueError(f"{name} must be finite and greater than zero")
        if self.prestream_s < 2.0:
            raise ValueError("prestream_s must be at least 2 seconds")
        if (not isinstance(self.target_loss_action, str)
                or self.target_loss_action not in ("land", "hold")):
            raise ValueError("target_loss_action must be 'land' or 'hold'")


@dataclass(frozen=True)
class FlightState:
    heartbeat_fresh: bool
    telemetry_ready: bool
    armed: bool
    offboard: bool
    on_ground: bool
    vision_alive: bool
    frame_fresh: bool
    position: tuple[float, float, float] | None
    # The owner must explicitly identify PX4 AUTO.LAND, not just !offboard.
    land_mode: bool = False


@dataclass(frozen=True)
class Action:
    position: tuple[float, float, float] | None = None
    yaw_rate: float = 0.0
    request_mode: bool = False
    request_arm: bool = False
    request_land: bool = False
    done: bool = False
    success: bool = False
    reason: str = ""


class YawMission:
    """One mission only: a new instance never takes over an armed aircraft.

    A terminal result is persistent.  No path returns an ARM request after the
    first ARM request has either been confirmed or abandoned for landing.
    """

    MODE_TIMEOUT_S = 5.0
    ARM_TIMEOUT_S = 10.0
    RETRY_S = 1.0

    def __init__(self, config: MissionConfig, now: float):
        if not math.isfinite(now):
            raise ValueError("now must be finite")
        self.config = config
        self.phase = "WAIT_READY"
        self._phase_since = now
        self._last_tick = now
        self._origin = None
        self._first_tick = True
        self._arm_request_at = None
        self._armed_at = None
        self._takeoff_at = None
        self._lost_since = None
        self._last_mode_request = None
        self._last_land_request = None
        self._landing_reason = ""
        self._landing_success = False
        self._terminal = None

    @staticmethod
    def _position_valid(position):
        return (isinstance(position, (tuple, list)) and len(position) == 3
                and all(isinstance(v, (int, float)) and not isinstance(v, bool)
                        and math.isfinite(v) for v in position))

    @classmethod
    def _telemetry_healthy(cls, state):
        return (state.heartbeat_fresh and state.telemetry_ready
                and cls._position_valid(state.position))

    @classmethod
    def _ready(cls, state):
        return (cls._telemetry_healthy(state) and state.on_ground
                and state.vision_alive and state.frame_fresh)

    @classmethod
    def _unhealthy_reason(cls, state):
        if not state.heartbeat_fresh:
            return "heartbeat_stale"
        if not state.telemetry_ready or not cls._position_valid(state.position):
            return "telemetry_not_ready"
        if not state.vision_alive:
            return "vision_not_alive"
        if not state.frame_fresh:
            return "vision_frame_stale"
        return ""

    def _transition(self, phase, now):
        self.phase = phase
        self._phase_since = now

    def _finish(self, reason, success=False):
        self.phase = "DONE" if success else "FAILED"
        self._terminal = Action(done=True, success=success, reason=reason)
        return self._terminal

    def _land(self, now, state, reason, success=False):
        self._transition("LANDING", now)
        self._landing_reason = reason
        self._landing_success = success
        # Do not mistake the pre-ARM disarmed heartbeat for a landing result.
        return self._landing_tick(now, state, first=True)

    def _landing_tick(self, now, state, *, first=False):
        # After ARM was sent, an old disarmed state does not establish that the
        # command has finished.  Observe armed->disarmed, or wait out the ARM
        # confirmation window before accepting a new disarmed heartbeat.
        arm_settled = (self._armed_at is not None
                       or (self._arm_request_at is not None
                           and now - self._arm_request_at >= self.ARM_TIMEOUT_S))
        if (not first and arm_settled and state.heartbeat_fresh
                and not state.armed):
            return self._finish(self._landing_reason + "; disarmed",
                                self._landing_success)
        if now - self._phase_since >= self.config.landing_timeout_s:
            return self._finish(self._landing_reason + "; landing_timeout")
        if (not first and state.heartbeat_fresh
                and not state.offboard and not state.land_mode):
            return self._finish(self._landing_reason + "; landing_mode_taken_over")

        position = None
        if state.offboard and self._telemetry_healthy(state):
            # Hold the latest measured position while LAND is being accepted;
            # never continue streaming the previous climb/high-altitude target.
            position = tuple(state.position)
        retry = (first or self._last_land_request is None
                 or now - self._last_land_request >= self.RETRY_S)
        if retry:
            self._last_land_request = now
        return Action(position=position, request_land=retry,
                      reason=self._landing_reason)

    def tick(self, now: float, state: FlightState, *, yaw_rate: float,
             target_fresh: bool, stop: bool = False) -> Action:
        if not math.isfinite(now) or now < self._last_tick:
            raise ValueError("now must be finite and monotonic")
        self._last_tick = now
        if self._terminal is not None:
            return self._terminal

        if self._first_tick:
            self._first_tick = False
            if state.armed:
                return self._finish("already_armed_at_start")
        if self._arm_request_at is None and state.armed:
            return self._finish("armed_without_mission_request")
        if (self._arm_request_at is not None and self._armed_at is None
                and state.heartbeat_fresh and state.armed):
            self._armed_at = now

        if self.phase == "LANDING":
            return self._landing_tick(now, state)

        # A fresh mode change after arming belongs to the pilot/PX4.  Never
        # reassert OFFBOARD, ARM or LAND over that takeover.
        if (self._armed_at is not None and state.heartbeat_fresh
                and not state.offboard):
            return self._finish("offboard_taken_over")
        if (self._armed_at is not None and state.heartbeat_fresh
                and not state.armed):
            return self._finish("unexpected_disarm")

        reason = self._unhealthy_reason(state)
        if stop or (self.phase != "WAIT_READY" and reason):
            reason = "stop_requested" if stop else reason
            if self._arm_request_at is not None:
                return self._land(now, state, reason)
            return self._finish(reason)

        if (self._armed_at is not None
                and now - self._armed_at >= self.config.mission_duration_s):
            return self._land(now, state, "mission_duration_reached", success=True)

        if self.phase == "WAIT_READY":
            if now - self._phase_since >= self.config.ready_timeout_s:
                return self._finish("ready_timeout")
            if not self._ready(state):
                return Action(reason=reason or "waiting_for_on_ground")
            self._origin = tuple(state.position)
            self._transition("PREFILL", now)
            return Action(position=self._origin)

        if self.phase == "PREFILL":
            if not state.on_ground:
                return self._finish("left_ground_before_arm")
            if now - self._phase_since >= self.config.prestream_s:
                self._transition("WAIT_OFFBOARD", now)
                self._last_mode_request = now
                return Action(position=self._origin, request_mode=True)
            return Action(position=self._origin)

        if self.phase == "WAIT_OFFBOARD":
            if not state.on_ground:
                return self._finish("left_ground_before_arm")
            if now - self._phase_since >= self.MODE_TIMEOUT_S:
                return self._finish("offboard_timeout")
            if state.offboard:
                self._transition("WAIT_ARM", now)
                self._arm_request_at = now
                return Action(position=self._origin, request_arm=True)
            retry = now - self._last_mode_request >= self.RETRY_S
            if retry:
                self._last_mode_request = now
            return Action(position=self._origin, request_mode=retry)

        if self.phase == "WAIT_ARM":
            if not state.offboard:
                return self._land(now, state, "offboard_lost_while_arming")
            if now - self._phase_since >= self.ARM_TIMEOUT_S:
                return self._land(now, state, "arm_timeout")
            if self._armed_at is not None:
                self._transition("TAKEOFF", now)
                self._takeoff_at = now
            return Action(position=self._origin)

        target = (self._origin[0], self._origin[1],
                  self._origin[2] - self.config.flight_height_m)
        if self.phase == "TAKEOFF":
            limit = max(30.0, self.config.flight_height_m
                        / self.config.takeoff_speed_m_s + 10.0)
            if now - self._takeoff_at >= limit:
                return self._land(now, state, "takeoff_timeout")
            climb = min(self.config.flight_height_m,
                        self.config.takeoff_speed_m_s * (now - self._takeoff_at))
            position = (self._origin[0], self._origin[1], self._origin[2] - climb)
            # Require the ramp to finish too, even for heights < 0.15 m.
            if (climb >= self.config.flight_height_m
                    and abs(state.position[2] - target[2]) <= 0.15):
                self._transition("FOLLOW", now)
                self._lost_since = None if target_fresh else now
            return Action(position=position)

        if self.phase == "FOLLOW":
            if target_fresh:
                self._lost_since = None
            elif self._lost_since is None:
                self._lost_since = now
            if (self.config.target_loss_action == "land"
                    and self._lost_since is not None
                    and now - self._lost_since >= self.config.target_loss_timeout_s):
                return self._land(now, state, "target_loss_timeout", success=True)
            rate = yaw_rate if target_fresh and math.isfinite(yaw_rate) else 0.0
            return Action(position=target, yaw_rate=rate)

        return self._finish("invalid_phase")
