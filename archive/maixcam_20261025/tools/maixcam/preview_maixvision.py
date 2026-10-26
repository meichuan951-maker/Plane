"""Run in MaixVision for on-camera traditional armor detection and an FPS overlay."""
import math
import os
import time

WIDTH = 640
HEIGHT = 360
# GC4653 supports its high-frame-rate sensor mode at resolutions <= 1280x720.
# This is a request, not a guarantee of the PC image panel's display rate.
REQUESTED_FPS = 60
# Prefer throughput while testing detection; raise quality after profiling.
JPEG_QUALITY = 80
ENABLE_ARMOR_DETECTION = True
# Current bench target is red. Use "blue" or "both" for other targets.
TARGET_COLOR = "red"
# Normalized x/y/width/height. At 640x360 this is the central 128x72 window.
# Keep the entire armor inside the white box. None restores full-frame search.
DETECTION_WINDOW = (.40, .40, .20, .20)
# Microseconds; None restores auto exposure. Do not confuse this with gain.
EXPOSURE_US = 1500
# MaixPy LAB, not OpenCV HSV. Order is red then blue; tune under real lighting.
#                    Lmin Lmax Amin Amax Bmin Bmax
LAB_THRESHOLDS = [(20, 100, 20, 127, -20, 127),
                  (10, 100, -20, 127, -128, -25)]
BLOB_PIXELS_THRESHOLD = 6
BLOB_AREA_THRESHOLD = 8
BLOB_X_STRIDE = 2
MIN_BLOB_FILL = .25
MAX_BLOBS = 64

# Geometry uses the same units/names as config/light_bars.yaml. These are
# candidate filters, not armor identity or calibrated pose measurements.
ARMOR_GEOMETRY = {
    "min_length": 6.0,
    "min_aspect": 2.0,
    "max_aspect": 30.0,
    "max_tilt_deg": 40.0,
    "pair_min_ratio": 0.8,
    "pair_max_ratio": 5.0,
    "pair_max_length_ratio": 1.5,
    "pair_max_y_offset": 0.5,
    "pair_max_angle_deg": 15.0,
    "pair_max_rectangular_error_deg": 25.0,
    "pair_min_score": 0.25,
    # Only soften scoring for small candidates: integer corner locations have
    # roughly pixel-scale uncertainty. Hard pair rejection limits stay intact.
    "small_target_max_length": 16.0,
    "small_target_tolerance_px": 1.0,
    "armor_width_m": 0.135,
    "lightbar_length_m": 0.056,
}
MAX_BARS = 16
PAIR_EXACT_LIMIT = 10


def _geometry_config(config):
    values = ARMOR_GEOMETRY.copy()
    if config is not None:
        unknown = set(config) - set(values)
        if unknown:
            raise ValueError("Unknown armor geometry keys: " + ", ".join(sorted(unknown)))
        values.update(config)
    if not all(isinstance(value, (int, float)) and math.isfinite(value)
               for value in values.values()):
        raise ValueError("Armor geometry values must be finite numbers")
    if (values["min_length"] <= 0 or values["min_aspect"] <= 1 or
            values["max_aspect"] < values["min_aspect"] or
            not 0 <= values["max_tilt_deg"] <= 90 or
            values["pair_min_ratio"] <= 0 or
            values["pair_max_ratio"] < values["pair_min_ratio"] or
            values["pair_max_length_ratio"] <= 1 or
            values["pair_max_y_offset"] <= 0 or
            not 0 < values["pair_max_angle_deg"] <= 90 or
            not 0 < values["pair_max_rectangular_error_deg"] <= 90 or
            not 0 < values["pair_min_score"] <= 1 or
            values["small_target_max_length"] < 0 or
            values["small_target_tolerance_px"] < 0 or
            values["armor_width_m"] <= 0 or values["lightbar_length_m"] <= 0):
        raise ValueError("Invalid armor geometry thresholds")
    return values


def _point(value):
    """Return a finite float pair, or None for an unusable SDK candidate."""
    try:
        point = (float(value[0]), float(value[1]))
    except (TypeError, ValueError, IndexError, OverflowError):
        return None
    return point if all(math.isfinite(v) for v in point) else None


def light_bar_from_corners(color, corners, config=None):
    """Convert Blob.mini_corners() to a vertical light bar, or reject it.

    The four rectangle corners may arrive in any order. Short-edge midpoints
    define the long-axis endpoints, avoiding Blob.rotation() axis conventions.
    """
    c = _geometry_config(config)
    if color not in (0, 1):
        return None
    try:
        points = [_point(p) for p in corners]
    except TypeError:
        return None
    if len(points) != 4 or any(p is None for p in points):
        return None
    center = (sum(p[0] for p in points) / 4, sum(p[1] for p in points) / 4)
    points.sort(key=lambda p: math.atan2(p[1] - center[1], p[0] - center[0]))
    edges = [(points[(i + 1) % 4][0] - points[i][0],
              points[(i + 1) % 4][1] - points[i][1]) for i in range(4)]
    crosses = [edges[i][0] * edges[(i + 1) % 4][1] -
               edges[i][1] * edges[(i + 1) % 4][0] for i in range(4)]
    if not (all(v > 1e-8 for v in crosses) or all(v < -1e-8 for v in crosses)):
        return None
    sizes = [math.hypot(*edge) for edge in edges]
    even = (sizes[0] + sizes[2]) / 2
    odd = (sizes[1] + sizes[3]) / 2
    start = 0 if even <= odd else 1
    endpoints = [((points[i][0] + points[(i + 1) % 4][0]) / 2,
                  (points[i][1] + points[(i + 1) % 4][1]) / 2)
                 for i in (start, start + 2)]
    top, bottom = sorted(endpoints, key=lambda p: (p[1], p[0]))
    dx, dy = bottom[0] - top[0], bottom[1] - top[1]
    length, width = math.hypot(dx, dy), min(even, odd)
    if width <= 0 or length < c["min_length"]:
        return None
    aspect = length / width
    tilt = math.degrees(math.atan2(abs(dx), abs(dy)))
    if not c["min_aspect"] <= aspect <= c["max_aspect"] or tilt > c["max_tilt_deg"]:
        return None
    return {"color": color, "center": center, "top": top, "bottom": bottom,
            "length": length, "width": width, "tilt_deg": tilt,
            "axis": (dx / length, dy / length), "corners": tuple(points)}


def merge_light_bar_fragments(bars, config=None, max_side_ratio=0.2):
    """Merge nearby colored side strips around one overexposed white core.

    Candidates must have the same color, similar lengths, nearly parallel axes,
    and at least 70% longitudinal overlap. Every member must be close to every
    other member, preventing a chain of noise from bridging separate lights.
    """
    c = _geometry_config(config)
    if not math.isfinite(max_side_ratio) or not 0 < max_side_ratio < c["pair_min_ratio"]:
        raise ValueError("Fragment spacing must be positive and smaller than armor spacing")
    valid = []
    for bar in bars:
        try:
            normal = light_bar_from_corners(bar["color"], bar["corners"], c)
        except (TypeError, KeyError):
            continue
        if normal is not None:
            valid.append(normal)
    valid.sort(key=lambda b: (-b["length"], b["center"], b["color"]))
    valid = sorted(valid[:MAX_BARS * 2], key=lambda b: (b["center"], b["color"]))

    def nearby(a, b):
        if a["color"] != b["color"] or max(a["length"], b["length"]) > 1.3 * min(a["length"], b["length"]):
            return False
        ax, ay = a["axis"]
        bx, by = b["axis"]
        angle = math.degrees(math.acos(max(-1, min(1, ax * bx + ay * by))))
        if angle > 10:
            return False
        scale = math.hypot(ax + bx, ay + by)
        axis = ((ax + bx) / scale, (ay + by) / scale)
        side = (axis[1], -axis[0])
        delta = (b["center"][0] - a["center"][0], b["center"][1] - a["center"][1])
        if abs(delta[0] * side[0] + delta[1] * side[1]) > max_side_ratio * (a["length"] + b["length"]) / 2:
            return False
        project = lambda point: point[0] * axis[0] + point[1] * axis[1]
        a0, a1 = sorted((project(a["top"]), project(a["bottom"])))
        b0, b1 = sorted((project(b["top"]), project(b["bottom"])))
        return min(a1, b1) - max(a0, b0) >= .7 * min(a1 - a0, b1 - b0)

    groups = []
    for bar in valid:
        for group in groups:
            if all(nearby(bar, other) for other in group):
                group.append(bar)
                break
        else:
            groups.append([bar])
    result = []
    for group in groups:
        if len(group) == 1:
            result.append(group[0])
            continue
        ax = sum(bar["axis"][0] for bar in group)
        ay = sum(bar["axis"][1] for bar in group)
        scale = math.hypot(ax, ay)
        axis, side = (ax / scale, ay / scale), (ay / scale, -ax / scale)
        points = [point for bar in group for point in bar["corners"]]
        along = [point[0] * axis[0] + point[1] * axis[1] for point in points]
        across = [point[0] * side[0] + point[1] * side[1] for point in points]
        corners = [(s * side[0] + t * axis[0], s * side[1] + t * axis[1])
                   for s, t in ((min(across), min(along)), (max(across), min(along)),
                                (max(across), max(along)), (min(across), max(along)))]
        merged = light_bar_from_corners(group[0]["color"], corners, c)
        result.extend([merged] if merged is not None else group)
    return sorted(result, key=lambda b: (b["center"], b["color"]))


def _select_armor_pairs(pairs, count):
    """Maximum-weight matching on small conflict groups; bounded greedy fallback."""
    edges = [[] for _ in range(count)]
    for index, (left, right, _) in enumerate(pairs):
        edges[left].append(index)
        edges[right].append(index)
    seen, selected = set(), []
    for root in range(count):
        if root in seen or not edges[root]:
            continue
        group = [root]
        seen.add(root)
        for vertex in group:
            for edge in edges[vertex]:
                left, right, _ = pairs[edge]
                other = right if left == vertex else left
                if other not in seen:
                    seen.add(other)
                    group.append(other)
        group.sort()
        if len(group) > PAIR_EXACT_LIMIT:
            ranked = sorted({e for vertex in group for e in edges[vertex]},
                            key=lambda e: (-pairs[e][2], pairs[e][0], pairs[e][1]))
            used = set()
            for edge in ranked:
                left, right, _ = pairs[edge]
                if left not in used and right not in used:
                    selected.append(edge)
                    used.update((left, right))
            continue
        local = {vertex: i for i, vertex in enumerate(group)}
        memo, choice = {0: 0.0}, {}

        def solve(mask):
            if mask in memo:
                return memo[mask]
            first = (mask & -mask).bit_length() - 1
            vertex = group[first]
            rest = mask & ~(1 << first)
            best = solve(rest)
            for edge in edges[vertex]:
                left, right, score = pairs[edge]
                other = right if left == vertex else left
                bit = 1 << local[other]
                if rest & bit:
                    value = score + solve(rest & ~bit)
                    if value > best + 1e-12:
                        best, choice[mask] = value, edge
            memo[mask] = best
            return best

        mask = (1 << len(group)) - 1
        solve(mask)
        while mask:
            first = (mask & -mask).bit_length() - 1
            edge = choice.get(mask)
            mask &= ~(1 << first)
            if edge is not None:
                selected.append(edge)
                left, right, _ = pairs[edge]
                other = right if left == group[first] else left
                mask &= ~(1 << local[other])
    return selected


def pair_light_bars(bars, config=None):
    """Return same-color armor quadrilaterals without reusing any light bar.

    Input bar dicts contain color (0=red/1=blue), center, top and bottom. Result
    corners are left-top/right-top/right-bottom/left-bottom. Only the longest
    MAX_BARS candidates enter matching, limiting work in cluttered scenes.
    """
    c = _geometry_config(config)
    geometry = []
    for source_index, bar in enumerate(bars):
        try:
            color = bar["color"]
            top, bottom, center = (_point(bar[name]) for name in ("top", "bottom", "center"))
        except (TypeError, KeyError):
            continue
        if color not in (0, 1) or any(p is None for p in (top, bottom, center)):
            continue
        dx, dy = bottom[0] - top[0], bottom[1] - top[1]
        length = math.hypot(dx, dy)
        if length < 1 or dy <= 0:
            continue
        geometry.append({"color": color, "top": top, "bottom": bottom, "center": center,
                         "length": length, "axis": (dx / length, dy / length),
                         "source_index": source_index})
    geometry.sort(key=lambda b: (-b["length"], b["center"], b["color"], b["top"], b["bottom"]))
    geometry = geometry[:MAX_BARS]
    geometry.sort(key=lambda b: (b["center"], b["color"], b["top"], b["bottom"]))
    pairs = []
    expected_ratio = c["armor_width_m"] / c["lightbar_length_m"]
    min_axis_dot = math.cos(math.radians(c["pair_max_angle_deg"]))
    for i, left in enumerate(geometry):
        max_dx = c["pair_max_ratio"] * left["length"] * (1 + c["pair_max_length_ratio"]) / 2
        for j in range(i + 1, len(geometry)):
            right = geometry[j]
            dx = right["center"][0] - left["center"][0]
            dy = right["center"][1] - left["center"][1]
            if dx > max_dx:
                break
            if dx <= 0 or left["color"] != right["color"]:
                continue
            height = (left["length"] + right["length"]) / 2
            side_ratio = max(left["length"], right["length"]) / min(left["length"], right["length"])
            separation = math.hypot(dx, dy)
            ratio, y_offset = separation / height, abs(dy) / height
            la, ra = left["axis"], right["axis"]
            angle = math.degrees(math.acos(max(-1, min(1, la[0] * ra[0] + la[1] * ra[1]))))
            rectangular = max(abs((la[0] * dx + la[1] * dy) / separation),
                              abs((ra[0] * dx + ra[1] * dy) / separation))
            error = math.degrees(math.asin(min(1, rectangular)))
            if (side_ratio > c["pair_max_length_ratio"] or y_offset > c["pair_max_y_offset"] or
                    angle > c["pair_max_angle_deg"] or error > c["pair_max_rectangular_error_deg"] or
                    not c["pair_min_ratio"] <= ratio <= c["pair_max_ratio"]):
                continue
            middle_y = (left["center"][1] + right["center"][1]) / 2
            if any(middle["color"] == left["color"] and
                   left["center"][0] < middle["center"][0] < right["center"][0] and
                   max(middle["length"], height) / min(middle["length"], height) <= c["pair_max_length_ratio"] and
                   middle["axis"][0] * la[0] + middle["axis"][1] * la[1] >= min_axis_dot and
                   middle_y - height / 2 <= middle["center"][1] < middle_y + height / 2
                   for middle in geometry[i + 1:j]):
                continue
            score_ratio, score_y, score_angle, score_error = side_ratio, y_offset, angle, error
            if height <= c["small_target_max_length"] and c["small_target_tolerance_px"] > 0:
                tolerance = c["small_target_tolerance_px"]
                shorter = min(left["length"], right["length"])
                # Each light can differ by one quantized pixel in length; the
                # center offset has a one-pixel budget. Angular uncertainty is
                # the angle subtended by that pixel at the shorter light.
                length_difference = abs(left["length"] - right["length"])
                score_ratio = 1 + max(0, length_difference - 2 * tolerance) / shorter
                score_y = max(0, abs(dy) - tolerance) / height
                angle_tolerance = math.degrees(math.atan2(tolerance, shorter))
                score_angle = max(0, angle - angle_tolerance)
                score_error = max(0, error - angle_tolerance)
            cost = (math.log(score_ratio) / math.log(c["pair_max_length_ratio"]) +
                    score_y / c["pair_max_y_offset"] + score_angle / c["pair_max_angle_deg"] +
                    score_error / c["pair_max_rectangular_error_deg"] +
                    .25 * abs(math.log(ratio / expected_ratio)))
            score = math.exp(-cost)
            if score >= c["pair_min_score"]:
                pairs.append((i, j, score))
    result = []
    for index in _select_armor_pairs(pairs, len(geometry)):
        i, j, score = pairs[index]
        left, right = geometry[i], geometry[j]
        result.append({"color": left["color"],
                       "corners": (left["top"], right["top"], right["bottom"], left["bottom"]),
                       "center": ((left["center"][0] + right["center"][0]) / 2,
                                  (left["center"][1] + right["center"][1]) / 2),
                       "score": score,
                       "bar_indices": (left["source_index"], right["source_index"])})
    return sorted(result, key=lambda armor: armor["center"])


def detection_settings():
    """Resolve an explicit native ROI and threshold-code to target-color mapping."""
    colors = {"red": (0,), "blue": (1,), "both": (0, 1)}.get(TARGET_COLOR)
    if colors is None:
        raise ValueError('TARGET_COLOR must be "red", "blue", or "both"')
    if DETECTION_WINDOW is None:
        roi = [0, 0, WIDTH, HEIGHT]
    else:
        if len(DETECTION_WINDOW) != 4 or not all(math.isfinite(v) for v in DETECTION_WINDOW):
            raise ValueError("DETECTION_WINDOW requires four finite normalized values")
        x, y, w, h = DETECTION_WINDOW
        if (min(x, y) < 0 or min(w, h) <= 0 or x + w > 1 + 1e-9 or y + h > 1 + 1e-9):
            raise ValueError("DETECTION_WINDOW must fit inside normalized image bounds")
        x, y = int(round(x * WIDTH)), int(round(y * HEIGHT))
        w, h = min(int(round(w * WIDTH)), WIDTH - x), min(int(round(h * HEIGHT)), HEIGHT - y)
        if min(w, h) < 1:
            raise ValueError("DETECTION_WINDOW is empty at this image resolution")
        roi = [x, y, w, h]
    return roi, [LAB_THRESHOLDS[index] for index in colors], {
        1 << position: color for position, color in enumerate(colors)}


def configure_exposure(cam, camera):
    if EXPOSURE_US is None:
        cam.exp_mode(camera.AeMode.Auto)
        print("Exposure mode=auto", flush=True)
    else:
        if not isinstance(EXPOSURE_US, int) or EXPOSURE_US <= 0:
            raise ValueError("EXPOSURE_US must be positive microseconds, or None")
        cam.exp_mode(camera.AeMode.Manual)
        cam.exposure(EXPOSURE_US)
        # Setter echoes the request on this SDK; query separately after warmup.
        print("Exposure requested_us=%d mode=manual; awaiting readback" % EXPOSURE_US,
              flush=True)


def detect_armors(frame):
    """Use native LAB blobs, then bounded Python geometry. Does not draw."""
    roi, thresholds, color_codes = detection_settings()
    started = time.monotonic()
    blobs = frame.find_blobs(
        thresholds, roi=roi, x_stride=BLOB_X_STRIDE, y_stride=1,
        area_threshold=BLOB_AREA_THRESHOLD, pixels_threshold=BLOB_PIXELS_THRESHOLD,
        merge=False)
    found = time.monotonic()
    truncated = len(blobs) > MAX_BLOBS
    bars = []
    # White cores can leave very thin colored side strips. Apply the normal
    # aspect limit after merging; otherwise those strips never reach the merge.
    fragment_config = ARMOR_GEOMETRY.copy()
    fragment_config["max_aspect"] = max(120.0, fragment_config["max_aspect"])
    for blob in sorted(blobs, key=lambda b: b.pixels(), reverse=True)[:MAX_BLOBS]:
        code = blob.code()
        if code not in color_codes:
            continue  # Never treat unsupported or mixed threshold codes as red.
        color = color_codes[code]
        bar = light_bar_from_corners(color, blob.mini_corners(), fragment_config)
        if bar is None:
            # A one-pixel-wide vertical component has degenerate min-area
            # corners. Use its narrow pixel bounds instead of losing the light.
            x, y, w, h = blob.rect()
            if 0 < w <= 2 and h >= ARMOR_GEOMETRY["min_length"]:
                bar = light_bar_from_corners(color, [(x, y), (x + w, y),
                    (x + w, y + h), (x, y + h)], fragment_config)
        if bar is None:
            continue
        pixels = blob.pixels()
        if (pixels < BLOB_PIXELS_THRESHOLD or
                pixels / (bar["length"] * bar["width"]) < MIN_BLOB_FILL):
            continue
        bars.append(bar)
    truncated = truncated or len(bars) > MAX_BARS * 2
    merged = merge_light_bar_fragments(bars, fragment_config)
    bars = [light_bar_from_corners(bar["color"], bar["corners"]) for bar in merged]
    bars = [bar for bar in bars if bar is not None]
    truncated = truncated or len(bars) > MAX_BARS
    bars = sorted(bars, key=lambda b: (-b["length"], b["center"], b["color"]))[:MAX_BARS]
    bars.sort(key=lambda b: (b["center"], b["color"]))
    armors = pair_light_bars(bars)
    return {"bars": bars, "armors": armors, "truncated": truncated, "roi": tuple(roi),
            "blob_count": len(blobs), "blob_ms": (found - started) * 1000,
            "geometry_ms": (time.monotonic() - found) * 1000}


def draw_armor_overlay(frame, result, image):
    """Mark light axes and close the actual armor quad, including tilted edges."""
    roi = result.get("roi")
    if roi is not None and DETECTION_WINDOW is not None:
        frame.draw_rect(*roi, image.COLOR_WHITE, thickness=1)
    for bar in result["bars"]:
        color = image.COLOR_RED if bar["color"] == 0 else image.COLOR_BLUE
        top, bottom = bar["top"], bar["bottom"]
        frame.draw_line(int(round(top[0])), int(round(top[1])),
                        int(round(bottom[0])), int(round(bottom[1])), color, thickness=2)
    for armor in result["armors"]:
        corners = armor["corners"]
        for index in range(4):
            start, end = corners[index], corners[(index + 1) % 4]
            frame.draw_line(int(round(start[0])), int(round(start[1])),
                            int(round(end[0])), int(round(end[1])),
                            image.COLOR_GREEN, thickness=2)
        label = "RED ARMOR" if armor["color"] == 0 else "BLUE ARMOR"
        label_y = int(min(p[1] for p in corners)) - 18
        if label_y < 64:
            label_y = int(max(p[1] for p in corners)) + 4
        frame.draw_string(max(0, int(corners[0][0])),
                          max(0, min(HEIGHT - 18, label_y)), label,
                          color=image.COLOR_GREEN, scale=1, wrap=False)


def main():
    if os.environ.get("MAIXVISION") != "1":
        raise RuntimeError("Open preview_maixvision.py in MaixVision and click Run. "
                           "Do not run this preview in the Ubuntu/SSH terminal.")

    # ImageTrans reads this once at the first send_to_maixvision() call.
    os.environ["MAIXVISION_IMG_QUALITY"] = str(JPEG_QUALITY)
    from maix import app, camera, display, image, log

    log.set_log_level(log.LogLevel.LEVEL_WARN, False)
    roi = detection_settings()[0] if ENABLE_ARMOR_DETECTION else None
    print("Opening camera for MaixVision preview: %dx%d, requested %d FPS, "
          "JPEG quality=%d..." % (WIDTH, HEIGHT, REQUESTED_FPS, JPEG_QUALITY),
          flush=True)
    cam = camera.Camera(WIDTH, HEIGHT, fps=REQUESTED_FPS)
    if ENABLE_ARMOR_DETECTION:
        print("Traditional armor target=%s roi=%s. Keep the entire armor "
              "inside the detection window." % (TARGET_COLOR, roi), flush=True)
    else:
        print("Pure MaixVision preview; keep this script running.", flush=True)
    frames = 0
    last = time.monotonic()
    fps_started = last
    fps_frames = 0
    fps = None
    stage_totals = [0.0] * 4
    detail_totals = [0.0, 0.0]
    produced = 0
    try:
        configure_exposure(cam, camera)
        while not app.need_exit():
            started = time.monotonic()
            frame = cam.read()
            if frame is None:
                raise RuntimeError("Camera returned no image; check device logs.")
            captured = time.monotonic()
            # Detect before drawing, so text/colored overlays cannot become blobs.
            result = (detect_armors(frame) if ENABLE_ARMOR_DETECTION else
                      {"bars": [], "armors": [], "truncated": False, "roi": None,
                       "blob_count": 0, "blob_ms": 0.0, "geometry_ms": 0.0})
            detected = time.monotonic()
            # Measure intervals between captured frames. After the first window,
            # each interval includes previous detection/draw/encode/send and the
            # current camera read. Refresh twice per second to reduce jitter.
            fps_frames += 1
            elapsed = captured - fps_started
            if elapsed >= .5:
                fps = fps_frames / elapsed
                fps_frames = 0
                fps_started = captured
            label = "FPS: --" if fps is None else "FPS: %.1f" % fps
            if ENABLE_ARMOR_DETECTION:
                label += "  " + TARGET_COLOR.upper() + (" ROI" if DETECTION_WINDOW is not None else " FULL")
            frame.draw_rect(4, 4, min(WIDTH - 8, 360) if ENABLE_ARMOR_DETECTION else 180,
                            58 if ENABLE_ARMOR_DETECTION else 36,
                            image.COLOR_BLACK, thickness=-1)
            frame.draw_string(10, 10, label, color=image.COLOR_GREEN, scale=1.5)
            if ENABLE_ARMOR_DETECTION:
                red = sum(bar["color"] == 0 for bar in result["bars"])
                blue = len(result["bars"]) - red
                status = "RED: %d  BLUE: %d  ARMOR: %d%s" % (
                    red, blue, len(result["armors"]), "  LIMIT" if result["truncated"] else "")
                frame.draw_string(10, 40, status, color=image.COLOR_GREEN, scale=1, wrap=False)
                # Draw target edges last so the HUD cannot erase a top-of-frame plate.
                draw_armor_overlay(frame, result, image)
            drawn = time.monotonic()
            # The SDK initializes the MaixVision transport without a physical
            # Display. Encode/send once; no UVC or RGB byte copies.
            display.send_to_maixvision(frame)
            frames += 1
            produced += 1
            now = time.monotonic()
            for index, duration in enumerate((captured - started, detected - captured,
                                              drawn - detected, now - drawn)):
                stage_totals[index] += duration
            detail_totals[0] += result["blob_ms"]
            detail_totals[1] += result["geometry_ms"]
            if produced == 5 and EXPOSURE_US is not None:
                try:
                    print("Exposure readback_us=%s requested_us=%d" %
                          (cam.exposure(), EXPOSURE_US), flush=True)
                except (AttributeError, TypeError, RuntimeError, ValueError) as error:
                    print("Exposure readback unavailable: " + str(error), flush=True)
            if now - last >= 2:
                print("MaixVision preview submitted: %.1f FPS "
                      "read_ms=%.1f detect_ms=%.1f draw_ms=%.1f encode_send_ms=%.1f "
                      "blob_ms=%.1f geometry_ms=%.1f" %
                      (frames / (now - last),
                       *(duration * 1000 / frames for duration in stage_totals),
                       *(duration / frames for duration in detail_totals)),
                      flush=True)
                if ENABLE_ARMOR_DETECTION and result["bars"] and not result["armors"]:
                    print("Unpaired bars: " + "; ".join(
                        "%s center=(%.1f,%.1f) length=%.1f width=%.1f axis_deg=%.1f" % (
                            "R" if bar["color"] == 0 else "B", *bar["center"],
                            bar["length"], bar["width"],
                            math.degrees(math.atan2(*bar["axis"]))) for bar in result["bars"][:4]),
                        flush=True)
                frames, last = 0, now
                stage_totals = [0.0] * 4
                detail_totals = [0.0, 0.0]
    finally:
        cam.close()


if __name__ == "__main__":
    main()
