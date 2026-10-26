"""Offline resource-test worker; never opens a serial port, UDP port, or socket."""
import argparse
from collections import deque
import json
import math
import resource
import sys
import time

from .control import ControlConfig, Phase, PositionController, SimulatedPeer, exchange


def emit(packet):
    print(json.dumps(packet, allow_nan=False, separators=(",", ":")), flush=True)


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, math.ceil(len(ordered) * fraction) - 1)] if ordered else None


def wait_until(deadline):
    while True:
        delay = deadline - time.monotonic()
        if delay <= 0:
            return
        time.sleep(min(delay, .02))


def read_request():
    line = sys.stdin.readline(4097)
    if not line or len(line) > 4096:
        raise ValueError("Expected one JSON start request of at most 4096 characters")
    request = json.loads(line)
    if not isinstance(request, dict):
        raise ValueError("Start request must be an object")
    keys = ("start_monotonic_s", "warmup_s", "measure_s")
    for key in keys:
        value = request.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{key} must be a finite number")
    if request["start_monotonic_s"] < 0 or not 0 <= request["warmup_s"] <= 3600 or not 0 < request["measure_s"] <= 3600:
        raise ValueError("Invalid monotonic start or durations (maximum 3600 seconds each)")
    return request


def run(controller, peer, request):
    start = request["start_monotonic_s"]
    warmup = request["warmup_s"]
    duration = request["measure_s"]
    measurement_start, end = start + warmup, start + warmup + duration
    if time.monotonic() >= measurement_start:
        # Never return a nominal-length result after missing its beginning.
        raise ValueError("Start request arrived too late; choose a future common start")
    wait_until(start)
    first_tick = time.monotonic()
    next_tick = start
    dt = 1 / controller.config.rate_hz
    periods = deque(maxlen=8192)
    lateness = deque(maxlen=8192)
    period_max = lateness_max = 0.0
    measured = measured_hover = misses = period_count = 0
    previous = None
    while True:
        wait_until(next_tick)
        now = time.monotonic()
        if now >= end:
            break
        late_ms = max(0.0, now - next_tick) * 1000
        skipped = int(max(0.0, now - next_tick) / dt)
        next_tick += (skipped + 1) * dt
        before = controller.setpoints_sent
        exchange(controller, peer, now)
        if now >= measurement_start:
            misses += skipped
            lateness.append(late_ms)
            lateness_max = max(lateness_max, late_ms)
            sent = controller.setpoints_sent - before
            measured += sent
            if controller.phase == Phase.HOVER:
                measured_hover += sent
            if sent:
                if previous is not None:
                    period_ms = (now - previous) * 1000
                    periods.append(period_ms)
                    period_max = max(period_max, period_ms)
                    period_count += 1
                previous = now
        if controller.phase == Phase.FAILED:
            break
    measurement_ended = time.monotonic()
    measurement_elapsed = max(0.0, min(measurement_ended, end) - measurement_start)
    # Closing handshake is outside the common resource measurement window.
    now = time.monotonic()
    controller.finish(now)
    for frame in controller.drain():
        for response in peer.receive(frame, now):
            controller.receive(response, now)
    controller.step(now)
    for frame in controller.drain():
        peer.receive(frame, now)
    success = (controller.phase == Phase.COMPLETE and controller.handshake_completed
               and measured_hover > 0 and measurement_ended >= end)
    return {
        "event": "result", "role": "control", "success": success,
        "backend": "pymavlink_v2_in_memory_protocol_peer",
        "real_px4": False, "flight_dynamics": False, "hardware_io": False,
        "telemetry_model": "static_local_origin_no_dynamics",
        "state": controller.phase.value, "reason": controller.reason,
        "handshake_completed": controller.handshake_completed,
        "start_monotonic_s": start, "actual_start_monotonic_s": first_tick,
        "measurement_start_monotonic_s": measurement_start,
        "measurement_end_monotonic_s": end,
        "warmup_s": warmup, "measure_s": duration,
        "actual_measure_s": measurement_elapsed,
        "configured_hz": controller.config.rate_hz,
        "prefill_cycles": controller.config.prefill_cycles,
        "origin_ned_m": controller.config.origin_ned,
        "target_ned_m": controller.config.target_ned,
        "setpoints_sent": controller.setpoints_sent,
        "measured_setpoints": measured,
        "measured_hover_setpoints": measured_hover,
        "actual_hz": measured / duration,
        "period_p95_ms": percentile(periods, .95),
        "period_max_ms": period_max if period_count else None,
        "period_count": period_count,
        "period_samples_retained": len(periods),
        "period_quantile_scope": "last_8192_measurement_periods",
        "deadline_lateness_p95_ms": percentile(lateness, .95),
        "deadline_lateness_max_ms": lateness_max,
        "missed_deadlines": misses,
        "peer_setpoints_received": peer.setpoints_received,
        "peer_arm_commands": peer.arm_commands,
        "peer_land_commands": peer.land_commands,
        "mavlink_frames_encoded": controller.codec.encoded + peer.codec.encoded,
        "mavlink_frames_decoded": controller.codec.decoded + peer.codec.decoded,
        "mavlink_bytes_encoded": controller.codec.encoded_bytes + peer.codec.encoded_bytes,
        "peak_rss_kib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        "accepted_messages": controller.accepted_messages,
        "foreign_messages": controller.foreign_messages,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--benchmark-worker", action="store_true")
    parser.add_argument("--seconds", type=float, default=10,
                        help="standalone measurement duration after 6-second warmup")
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 3600:
        parser.error("--seconds must be finite and in (0,3600]")
    try:
        controller = PositionController()
        peer = SimulatedPeer(controller.config)
        if args.benchmark_worker:
            emit({"event": "ready", "role": "control",
                  "backend": "pymavlink_v2_in_memory_protocol_peer",
                  "recommended_warmup_s": 6.0, "hardware_io": False})
            request = read_request()
        else:
            request = {"start_monotonic_s": time.monotonic() + .1,
                       "warmup_s": 6.0, "measure_s": args.seconds}
        result = run(controller, peer, request)
        emit(result)
        if args.benchmark_worker:
            # Parent samples the completed process before releasing it.
            sys.stdin.readline(4097)
        return 0 if result["success"] else 1
    except (ImportError, ValueError, RuntimeError, OSError) as error:
        emit({"event": "result", "role": "control", "success": False,
              "error": f"{type(error).__name__}: {error}", "hardware_io": False})
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
