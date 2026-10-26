"""Transport-free horizontal image centering; no camera or flight device access.

Two-dimensional observations remain useful when RGB PnP calibration is missing.
Positive yaw rate turns right in NED for an upright, forward-facing camera;
the deployment must explicitly choose the installation's direction sign.
"""
from dataclasses import dataclass
import math


def _number(value):
    """Accept JSON numbers, excluding booleans and non-finite values."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        result = float(value)
    except (OverflowError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _parameter(value, name, *, minimum=0.0, strictly_positive=False):
    result = _number(value)
    if result is None or result < minimum or (strictly_positive and result <= 0):
        raise ValueError(f"{name} must be finite and "
                         f"{'positive' if strictly_positive else f'>= {minimum}'}")
    return result


def _identity(value, minimum):
    return isinstance(value, int) and not isinstance(value, bool) and value >= minimum


@dataclass(frozen=True)
class VisionSample:
    sample_time_s: float
    generation: int
    frame: int
    track_id: int
    error_x: float

    def __post_init__(self):
        _parameter(self.sample_time_s, "sample_time_s")
        if not (_identity(self.generation, 0) and _identity(self.frame, 1) and
                _identity(self.track_id, 1)):
            raise ValueError("generation must be >= 0; frame and track_id must be > 0 integers")
        error = _number(self.error_x)
        if error is None or not -1 <= error <= 1:
            raise ValueError("error_x must be finite and within [-1, 1]")


def parse_visual_report(report: dict, *, now: float, width: int, height: int,
                        max_age_s: float = 0.25) -> VisionSample | None:
    """Return a fresh confirmed 2D sample, or None for an unusable report.

    Invalid caller parameters raise ValueError. Report data is untrusted and
    rejected by returning None. Times must share the host monotonic clock.
    The PnP ``valid`` flag is deliberately independent of this 2D policy.
    """
    now = _parameter(now, "now")
    max_age_s = _parameter(max_age_s, "max_age_s", strictly_positive=True)
    if not (_identity(width, 1) and _identity(height, 1)):
        raise ValueError("width and height must be positive integers")
    # A finite float representation is also needed for normalized coordinates.
    width_value = _number(width)
    height_value = _number(height)
    if width_value is None or height_value is None:
        raise ValueError("width and height must be finite")

    if not isinstance(report, dict):
        return None
    if (report.get("source") != "automatic_armor_rgb_pnp" or
            report.get("coordinate_frame") != "rgb_optical" or
            report.get("detection_valid") is not True or
            report.get("tracking_confirmed") is not True):
        return None
    generation = report.get("generation")
    frame = report.get("frame")
    track_id = report.get("track_id")
    if not (_identity(generation, 0) and _identity(frame, 1) and _identity(track_id, 1)):
        return None
    sample_time = _number(report.get("sample_time_monotonic_s"))
    frame_age_ms = _number(report.get("frame_age_ms"))
    if (sample_time is None or sample_time < 0 or
            not 0 <= now - sample_time <= max_age_s or
            frame_age_ms is None or not 0 <= frame_age_ms <= max_age_s * 1000):
        return None

    corners = report.get("armor_corners_px")
    if not isinstance(corners, (list, tuple)) or len(corners) != 4:
        return None
    normalized_x = []
    for point in corners:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            return None
        x, y = (_number(value) for value in point)
        if (x is None or y is None or not 0 <= x < width_value or
                not 0 <= y < height_value):
            return None
        normalized_x.append(x / width_value)
    # Normalize before summing, so even large finite coordinates cannot overflow.
    error_x = 2 * (sum(normalized_x) / 4) - 1
    return VisionSample(sample_time, generation, frame, track_id, error_x)


def yaw_rate_for_sample(sample: VisionSample | None, *, now: float, gain: float,
                        max_rate: float, deadband: float, direction: float = 1.0,
                        max_age_s: float = 0.25) -> float:
    """Map horizontal image error to a bounded yaw rate in radians/second.

    The deadband uses normalized half-image-width units. At or inside the
    deadband, with no sample, or after expiry, return zero yaw rate. This pure
    policy does not hold aircraft position or altitude or command flight modes.
    """
    now = _parameter(now, "now")
    gain = _parameter(gain, "gain")
    max_rate = _parameter(max_rate, "max_rate", strictly_positive=True)
    deadband = _parameter(deadband, "deadband")
    max_age_s = _parameter(max_age_s, "max_age_s", strictly_positive=True)
    if deadband > 1:
        raise ValueError("deadband must be within [0, 1]")
    direction = _number(direction)
    if direction not in (-1.0, 1.0):
        raise ValueError("direction must be +1 or -1")
    if sample is not None and not isinstance(sample, VisionSample):
        raise ValueError("sample must be a VisionSample or None")
    if sample is None or not 0 <= now - sample.sample_time_s <= max_age_s:
        return 0.0
    if abs(sample.error_x) <= deadband:
        return 0.0
    return float(max(-max_rate, min(max_rate, direction * gain * sample.error_x)))
