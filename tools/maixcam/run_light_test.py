#!/usr/bin/env python3
"""Run ON the Ubuntu PC: find the MaixCAM Lite UVC stream and start our detector.

The USB descriptors identify the MaixCAM family, not the Lite physical variant.
The connected board must be the project's selected MaixCAM Lite.
"""
import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


def belongs_to_maixcam(path):
    for parent in path.resolve().parents:
        vendor, product = parent / "idVendor", parent / "product"
        if vendor.is_file():
            name = product.read_text().strip().lower() if product.is_file() else ""
            return vendor.read_text().strip().lower() == "359f" or "maixcam" in name
    return False


def discover():
    if not shutil.which("v4l2-ctl"):
        raise RuntimeError("v4l2-ctl is required (Ubuntu package: v4l-utils).")
    candidates = []
    for entry in sorted(Path("/sys/class/video4linux").glob("video*"),
                        key=lambda p: int(p.name.removeprefix("video"))):
        if not belongs_to_maixcam(entry):
            continue
        node = "/dev/" + entry.name
        result = subprocess.run(["v4l2-ctl", "-d", node, "--list-formats-ext"],
                                capture_output=True, text=True, timeout=5,
                                env={**os.environ, "LC_ALL": "C"})
        if result.returncode == 0 and "'MJPG'" in result.stdout:
            candidates.append((node, result.stdout))
    if not candidates:
        raise RuntimeError("No MaixCAM MJPG capture node found. Enable UVC on the "
                           "MaixCAM, reconnect power, then run stream_uvc.py in MaixVision. "
                           "The built-in ASUS camera will not be selected.")
    if len(candidates) > 1:
        raise RuntimeError("Multiple MaixCAM capture nodes found: " +
                           ", ".join(node for node, _ in candidates) +
                           ". Inspect their formats before choosing a node.")
    return candidates[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="query nodes and formats only")
    args = parser.parse_args()
    node, formats = discover()
    print("MaixCAM Lite UVC capture node (MaixCAM USB family):", node, flush=True)
    print(formats, flush=True)
    if args.check:
        return 0
    # Confirm the requested size belongs to the MJPG section, not another format.
    sections = re.split(r"\[\d+\]:", formats)
    mjpg = next((section for section in sections if "'MJPG'" in section), "")
    if not re.search(r"Size:\s+Discrete\s+640x360\b", mjpg):
        raise RuntimeError("This UVC configuration does not advertise discrete MJPG "
                           "640x360. Send the formats printed above before choosing a mode.")
    binary = ROOT / "build/camera_light_test"
    if not os.access(binary, os.X_OK):
        raise RuntimeError("Build first with ./scripts/build.sh")
    env = os.environ.copy()
    if Path("/usr/share/glib-2.0/schemas/gschemas.compiled").is_file():
        env["GSETTINGS_SCHEMA_DIR"] = "/usr/share/glib-2.0/schemas"
    command = [str(binary), "--device", node, "--width", "640", "--height", "360",
               "--fps", "30", "--format", "MJPG", "--config",
               str(ROOT / "config/light_bars.yaml"), "--mask", "--no-controls"]
    print("Starting 640x360 / requested 30 FPS; see negotiated values in capture logs.",
          flush=True)
    os.chdir(ROOT)
    os.execve(binary, command, env)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
        print("maixcam_light_test:", error, file=sys.stderr)
        raise SystemExit(1)
