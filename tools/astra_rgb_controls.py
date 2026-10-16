"""Apply explicitly configured V4L2 controls and verify driver readback."""

import argparse
from pathlib import Path
import re
import subprocess
import sys

import yaml


def run(device, *arguments):
    result = subprocess.run(
        ["v4l2-ctl", "-d", device, *arguments],
        capture_output=True, text=True, timeout=5, check=True,
    )
    return result.stdout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path,
        default=Path(__file__).resolve().parents[1] / "config/astra_rgb_controls.yaml",
    )
    parser.add_argument("--apply", action="store_true",
                        help="write configured values; otherwise only query")
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    if not isinstance(config, dict) or not isinstance(config.get("device"), str):
        raise ValueError("Configuration requires a device path")
    controls = config.get("controls")
    if not isinstance(controls, dict) or not controls:
        raise ValueError("Configuration requires a nonempty controls mapping")
    allowed = {"backlight_compensation", "brightness", "contrast", "saturation",
               "gamma", "sharpness"}
    device = config["device"]
    listing = run(device, "--list-ctrls")
    # Validate every requested control before applying any change.
    for name, value in controls.items():
        if name not in allowed or type(value) is not int:
            raise ValueError(f"Unsupported control or noninteger value: {name}")
        match = re.search(
            rf"^\s*{re.escape(name)}\s+0x\w+\s+\(int\).*?"
            r"min=(-?\d+) max=(-?\d+) step=(\d+)", listing, re.MULTILINE,
        )
        if not match:
            raise ValueError(f"Camera does not expose control: {name}")
        minimum, maximum, step = map(int, match.groups())
        if not minimum <= value <= maximum or (value - minimum) % max(1, step):
            raise ValueError(f"{name}={value} outside [{minimum}, {maximum}], step {step}")
    for name, value in controls.items():
        if args.apply:
            run(device, f"--set-ctrl={name}={value}")
        readback = run(device, f"--get-ctrl={name}").strip()
        actual = int(readback.rsplit(":", 1)[1].strip())
        if args.apply and actual != value:
            raise RuntimeError(f"{name}: requested {value}, read back {actual}")
        print(f"{name}: requested={value}, current={actual}, "
              f"{'verified' if args.apply else 'query only'}")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, RuntimeError, OSError, subprocess.SubprocessError,
            yaml.YAMLError) as error:
        print(f"astra_rgb_controls: {error}", file=sys.stderr)
        sys.exit(1)
