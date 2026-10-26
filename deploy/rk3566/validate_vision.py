#!/usr/bin/env python3
"""Check a native vision build with synthetic inputs and no hardware I/O.

Run after sourcing the build's runtime environment. Every invocation requires a
new output directory; report.json and process logs retain the actual results.
Only PyYAML is required beyond Python's standard library.
"""

import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path
import platform
import struct
import subprocess
import sys
import time
import zlib


PROJECT = Path(__file__).resolve().parents[2]
BINARIES = ("demo_aim_cpp", "armor_auto", "camera_light_test", "vision_video_worker")
ELF_MACHINES = {"aarch64": 183, "x86_64": 62}
POSE_FIELDS = (
    "position_camera_m", "range_m", "z_m", "normal_yaw_deg", "normal_pitch_deg",
    "bearing_yaw_deg", "bearing_pitch_deg", "reprojection_error_px",
)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def elf_architecture(path):
    with path.open("rb") as stream:
        header = stream.read(64)
    require(len(header) == 64 and header[:4] == b"\x7fELF", f"Not an ELF binary: {path}")
    require(header[4] == 2 and header[5] in (1, 2), f"Expected ELF64: {path}")
    machine = struct.unpack_from("<H" if header[5] == 1 else ">H", header, 18)[0]
    return {"class": "ELF64", "machine": machine,
            "architecture": next((name for name, value in ELF_MACHINES.items()
                                  if value == machine), "unknown")}


def run(name, command, output, report):
    command = [str(part) for part in command]
    check = {"name": name, "command": command, "cwd": str(PROJECT), "timeout_s": 20,
             "stdout": name + ".stdout.log", "stderr": name + ".stderr.log"}
    report["checks"].append(check)
    started = time.monotonic()
    try:
        with (output / check["stdout"]).open("wb") as stdout, \
             (output / check["stderr"]).open("wb") as stderr:
            result = subprocess.run(command, cwd=PROJECT, stdin=subprocess.DEVNULL,
                                    stdout=stdout, stderr=stderr, timeout=20, check=False)
        check["exit_code"] = result.returncode
        require(result.returncode == 0, f"{name} returned {result.returncode}; see process logs")
    except Exception as error:
        check["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        check["elapsed_s"] = round(time.monotonic() - started, 3)
    return (output / check["stdout"]).read_text(errors="replace")


def json_lines(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def write_bars_png(path, width, height, color):
    """Write two colored vertical bars as an RGB PNG without Python OpenCV."""
    length = max(10, min(width, height) // 5)
    thickness = max(2, length // 8)
    centers = (width // 2 - length, width // 2 + length)
    y0, y1 = (height - length) // 2, (height + length) // 2
    rgb = b"\x00\x00\xff" if color == 1 else b"\xff\x00\x00"
    blank = bytes(width * 3)
    bars = bytearray(blank)
    for center in centers:
        x0, x1 = max(0, center - thickness // 2), min(width, center + thickness // 2)
        bars[x0 * 3:x1 * 3] = rgb * (x1 - x0)
    compressor = zlib.compressobj()
    compressed = bytearray()
    for y in range(height):
        compressed.extend(compressor.compress(b"\x00" + (bytes(bars) if y0 <= y < y1 else blank)))
    compressed.extend(compressor.flush())

    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))

    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) +
                     chunk(b"IDAT", compressed) + chunk(b"IEND", b""))


def check_retirement(build, report):
    manifest_path = PROJECT / "archive/astra_pro_20261023/manifest.json"
    manifest = json.loads(manifest_path.read_text())
    moved = [entry["source_path"] for entry in manifest["files"]
             if entry.get("action") == "moved" and
             Path(entry["source_path"]).parts[:1] != ("install",)]
    require(len(moved) == len(set(moved)) == 16,
            "Archive manifest must identify the 16 distinct retired active source paths")
    remaining = []
    for relative in moved:
        path = PROJECT / relative
        require(not Path(relative).is_absolute() and PROJECT in path.resolve().parents,
                f"Archive source path is outside the project: {relative}")
        if path.exists() or path.is_symlink():
            remaining.append(relative)
    binaries = [str(build / name) for name in ("astra_live_measure", "astra_depth_check")
                if (build / name).exists() or (build / name).is_symlink()]
    report["astra_retirement"] = {
        "manifest": str(manifest_path), "manifest_sha256": sha256(manifest_path),
        "checked_source_paths": moved, "remaining_source_paths": remaining,
        "remaining_active_binaries": binaries,
    }
    require(not remaining and not binaries, "Retired Astra source paths or active binaries remain")


def validate(args, output, report):
    import yaml  # --help remains available even before runtime dependencies are installed.

    config_path = PROJECT / "config/armor_auto.yaml"
    config = yaml.safe_load(config_path.read_text())
    camera = config["camera"]
    width, height = camera["width"], camera["height"]
    color = config["target_color"]
    require(type(width) is int and type(height) is int and
            16 <= width <= 4096 and 16 <= height <= 4096, "Invalid configured image dimensions")
    require(type(color) is int and color in (1, 2), "target_color must be 1 (blue) or 2 (red)")
    require(type(camera["fps"]) is int and
            all(isinstance(camera[key], str) for key in ("device", "format")) and
            all(isinstance(config[key], str) for key in ("detector_config", "rgb_calibration")),
            "Invalid configured camera parameters or paths")
    calibration = (config_path.parent / config["rgb_calibration"]).resolve()
    report["default_config"] = {
        "path": str(config_path), "sha256": sha256(config_path), "target_color": color,
        "requested_camera": {key: camera[key] for key in ("device", "width", "height", "fps", "format")},
        "detector_config": config["detector_config"],
        "rgb_calibration": {"path": str(calibration), "exists": calibration.is_file()},
        "camera_parameters_are_requested_not_negotiated": True,
    }
    if calibration.is_file():
        report["default_config"]["rgb_calibration"]["sha256"] = sha256(calibration)
    check_retirement(args.build_dir, report)

    for name in BINARIES:
        binary = args.build_dir / name
        observed = elf_architecture(binary)
        report["binaries"][name] = {"path": str(binary), "sha256": sha256(binary), **observed}
        require(observed["machine"] == ELF_MACHINES[args.expect_arch],
                f"{name}: expected {args.expect_arch}, observed {observed['architecture']}")
        help_text = run(name + "_help", [binary, "--help"], output, report)
        require(bool(help_text.strip()), f"{name}: empty --help output")
        if name == "demo_aim_cpp":
            compiled = "ROS 2: compiled" in help_text
            report["ros2_support"] = {"compiled": compiled, "required": args.expect_arch == "aarch64"}
            require(compiled or args.expect_arch == "x86_64", "demo_aim_cpp lacks compiled ROS 2 support")

    synthetic = run("demo_synthetic", [args.build_dir / "demo_aim_cpp", "--source", "synthetic",
                    "--config", PROJECT / "config/aim.yaml", "--frames", "3",
                    "--output", output / "synthetic.jsonl"], output, report)
    summary = json.loads(synthetic.strip().splitlines()[-1])
    packets = json_lines(output / "synthetic.jsonl")
    reasons = [packet.get("reason") for packet in packets]
    report["synthetic_pipeline"] = {"summary": summary, "reason_sequence": reasons,
                                    "valid_sequence": [packet.get("valid") for packet in packets]}
    require(summary.get("frames") == 3 and summary.get("detections_valid") == 1 and
            reasons == ["acquiring", "acquiring", "ok"] and packets[-1].get("valid") is True,
            "Synthetic pipeline did not confirm on frame three")

    image_path = output / ("synthetic_blue_bars.png" if color == 1 else "synthetic_red_bars.png")
    write_bars_png(image_path, width, height, color)
    run("armor_synthetic", [args.build_dir / "armor_auto", "--config", config_path,
        "--image", image_path, "--no-show", "--frames", "1", "--output", output / "armor.jsonl"],
        output, report)
    packets = json_lines(output / "armor.jsonl")
    detected = [packet for packet in packets if packet.get("detection_valid") is True]
    report["traditional_detector"] = {
        "input": str(image_path), "size": [width, height], "target_color": color,
        "detected_frames": len(detected), "packets": packets,
        "pose_accuracy_validated": False, "tracking_continuity_validated": False,
    }
    require(bool(detected), "Traditional light-bar detector did not detect the synthetic pair")
    if not calibration.is_file():
        require(all(packet.get("valid") is False and
                    all(field in packet and packet[field] is None for field in POSE_FIELDS)
                    for packet in packets), "Missing calibration produced a valid or numeric pose")
        report["traditional_detector"]["missing_calibration_pose_fields_null"] = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True, help="New directory for report and logs")
    parser.add_argument("--build-dir", type=Path, default=PROJECT / "build/rk3566-humble",
                        help="Native build directory (default: project/build/rk3566-humble)")
    parser.add_argument("--expect-arch", choices=ELF_MACHINES, default="aarch64",
                        help="Expected ELF architecture; x86_64 permits a build without ROS 2")
    args = parser.parse_args()
    output, args.build_dir = args.output_dir.absolute(), args.build_dir.resolve()
    try:
        output.mkdir(parents=True, exist_ok=False)
    except OSError as error:
        parser.error(f"Cannot create new --output-dir: {error}")
    report = {
        "schema_version": 1, "success": False,
        "started_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "project": str(PROJECT), "build_dir": str(args.build_dir),
        "host_architecture": platform.machine(), "expected_architecture": args.expect_arch,
        "scope": "offline_vision_loading_synthetic_confirmation_and_detection",
        "checks": [], "binaries": {},
        "real_camera_opened": False, "real_flight_controller_connected": False,
        "ros_nodes_started": False,
        "limits": [
            "Camera device, resolution, FPS and format are configuration requests; no driver negotiation was tested.",
            "Three injected demo frames test confirmation logic, not YLR4K video or model accuracy.",
            "One generated light-bar image tests detection; continuous tracking and calibrated PnP accuracy were not validated.",
            "Missing calibration must leave metric pose fields null; this check does not create calibration data.",
            "No real camera, serial port, ROS publisher/control node, ADB or SSH connection is started.",
        ],
    }
    try:
        validate(args, output, report)
        report["success"] = True
    except Exception as error:
        report["error"] = f"{type(error).__name__}: {error}"
    finally:
        report["finished_at_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
        report_path = output / "report.json"
        report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    print(json.dumps({"success": report["success"], "error": report.get("error"),
                      "report": str(report_path)}))
    return 0 if report["success"] else 1


if __name__ == "__main__":
    sys.exit(main())
