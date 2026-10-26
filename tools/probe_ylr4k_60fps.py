#!/usr/bin/env python3
"""Bounded, transient UVC PROBE diagnostic for the connected YLR4K.

The default is a read-only plan. --apply requires root and a closed preview.
It does not COMMIT a stream, capture frames, reset USB, change configuration,
write vendor extension controls, or flash firmware. A successful 60 fps PROBE
response does not establish a real 60 fps frame rate.
"""

import argparse
import json
import os
from pathlib import Path
import stat
import struct
import sys
import time


VID = 0x27C4
PID = 0x0531
UVC_VERSION = 0x0110
CONTROL_INTERFACE = 0
STREAM_INTERFACE = 1
PROBE_LENGTH = 34
TIMEOUT_MS = 1000
TARGET_INTERVAL = 166666  # floor(10000000 / 60), matching Linux's integer conversion.
MODES = ((1, 640, 360), (3, 1280, 720), (4, 1920, 1080))


def inspect_descriptors(data):
    """Fail closed unless interface and MJPG indices match this exact layout."""
    interface = None
    mjpg_index = None
    uvc_version = None
    frames = {}
    offset = 0
    while offset < len(data):
        length = data[offset]
        if length < 2 or offset + length > len(data):
            raise RuntimeError("Malformed USB descriptors")
        descriptor = data[offset:offset + length]
        dtype = descriptor[1]
        if dtype == 4 and length >= 9:
            interface = tuple(descriptor[2:9])
        elif dtype == 0x24 and interface is not None and length >= 3:
            ifnum, alternate, _, iclass, subclass, _, _ = interface
            if iclass == 14 and subclass == 1 and descriptor[2] == 1:
                if ifnum != CONTROL_INTERFACE or length < 5:
                    raise RuntimeError("Unexpected UVC control interface")
                uvc_version = struct.unpack_from("<H", descriptor, 3)[0]
            if ifnum == STREAM_INTERFACE and alternate == 0 and iclass == 14 and subclass == 2:
                if descriptor[2] == 6 and length >= 11:
                    mjpg_index = descriptor[3]
                elif descriptor[2] in (4, 0x10):
                    mjpg_index = None
                elif descriptor[2] == 7 and mjpg_index == 1 and length >= 26:
                    frames[descriptor[3]] = struct.unpack_from("<HH", descriptor, 5)
        offset += length
    if uvc_version != UVC_VERSION:
        raise RuntimeError("Expected UVC 1.10; refusing a different control layout")
    for index, width, height in MODES:
        if frames.get(index) != (width, height):
            raise RuntimeError("Expected MJPG format index 1 with the known frame indices")
    return {"uvc_version": "1.10", "mjpg_format_index": 1,
            "frames": [{"index": i, "width": w, "height": h} for i, w, h in MODES]}


def discover_device():
    matches = []
    for path in Path("/sys/bus/usb/devices").iterdir():
        try:
            if (int((path / "idVendor").read_text(), 16) == VID
                    and int((path / "idProduct").read_text(), 16) == PID):
                matches.append(path)
        except (FileNotFoundError, ValueError, PermissionError):
            continue
    if len(matches) != 1:
        raise RuntimeError("Expected exactly one 27c4:0531 camera; found " + str(len(matches)))
    path = matches[0]
    layout = inspect_descriptors((path / "descriptors").read_bytes())
    drivers = {}
    for interface in (CONTROL_INTERFACE, STREAM_INTERFACE):
        intf = path.parent / (path.name + ":1." + str(interface))
        driver = intf / "driver"
        drivers[str(interface)] = driver.resolve().name if driver.exists() else None
    if drivers != {"0": "uvcvideo", "1": "uvcvideo"}:
        raise RuntimeError("Expected uvcvideo bound to interfaces 0 and 1: " + str(drivers))
    video_nodes = []
    for video in Path("/sys/class/video4linux").glob("video*"):
        if path.resolve() in (video / "device").resolve().parents:
            video_nodes.append("/dev/" + video.name)
    if not video_nodes:
        raise RuntimeError("No video nodes found for this camera; refusing to skip the busy check")
    return {"vid": "27c4", "pid": "0531", "sysfs_name": path.name,
            "bus": int((path / "busnum").read_text()),
            "address": int((path / "devnum").read_text()),
            "device_version": (path / "bcdDevice").read_text().strip(),
            "drivers": drivers, "video_nodes": sorted(video_nodes), **layout}


def camera_users(video_nodes):
    """Check device numbers so aliases and deleted fd names cannot hide users."""
    numbers = {os.stat(node).st_rdev for node in video_nodes}
    users = []
    for process in Path("/proc").iterdir():
        if not process.name.isdecimal():
            continue
        try:
            for fd in (process / "fd").iterdir():
                try:
                    info = fd.stat()
                except (FileNotFoundError, PermissionError):
                    continue
                if stat.S_ISCHR(info.st_mode) and info.st_rdev in numbers:
                    users.append({"pid": int(process.name),
                                  "command": (process / "comm").read_text().strip()})
                    break
        except FileNotFoundError:
            continue
        except PermissionError as error:
            raise RuntimeError("Cannot inspect camera users; run --apply as root") from error
    return users


def decode_probe(data):
    if len(data) != PROBE_LENGTH:
        raise RuntimeError("Expected a 34-byte UVC 1.10 probe response")
    interval = struct.unpack_from("<I", data, 4)[0]
    return {"hex": bytes(data).hex(), "format_index": data[2], "frame_index": data[3],
            "interval_100ns": interval,
            "reported_fps": 10000000 / interval if interval else None,
            "max_frame_bytes": struct.unpack_from("<I", data, 18)[0],
            "max_payload_bytes": struct.unpack_from("<I", data, 22)[0]}


def perform_probe(device, usb_util):
    """Keep all device mutations and their recovery in a single finally scope."""
    report = {"operation": "standard_UVC_PROBE_only", "status": "failed",
              "results": [], "recovery": {"errors": []},
              "note": "PROBE readback is not a measured capture frame rate"}
    original = None
    claimed = False
    detach_attempted = False
    originally_bound = {}

    def get_probe():
        return bytes(device.ctrl_transfer(0xA1, 0x81, 0x0100, STREAM_INTERFACE,
                                          PROBE_LENGTH, timeout=TIMEOUT_MS))

    def set_probe(data):
        if len(data) != PROBE_LENGTH:
            raise RuntimeError("Refusing a PROBE write that is not exactly 34 bytes")
        count = device.ctrl_transfer(0x21, 0x01, 0x0100, STREAM_INTERFACE,
                                     data, timeout=TIMEOUT_MS)
        if count != PROBE_LENGTH:
            raise RuntimeError("Short UVC PROBE write: " + str(count))

    try:
        originally_bound = {i: device.is_kernel_driver_active(i) for i in (0, 1)}
        if not all(originally_bound.values()):
            raise RuntimeError("Both camera interfaces must initially be bound")
        # Detaching VC0 unregisters the video nodes and releases associated VS1.
        # Stop every camera client first; this helper never kills processes.
        detach_attempted = True
        device.detach_kernel_driver(CONTROL_INTERFACE)
        if device.is_kernel_driver_active(STREAM_INTERFACE):
            device.detach_kernel_driver(STREAM_INTERFACE)
        usb_util.claim_interface(device, STREAM_INTERFACE)
        claimed = True
        candidate = get_probe()
        decoded = decode_probe(candidate)
        original = candidate  # A malformed/short response must never be written back.
        report["original_probe"] = decoded
        for index, width, height in MODES:
            request = bytearray(original)
            struct.pack_into("<H", request, 0, 1)  # Hint: frame interval is requested.
            request[2] = 1  # MJPG, verified from the device descriptor.
            request[3] = index
            struct.pack_into("<I", request, 4, TARGET_INTERVAL)
            # Retain other fields returned by the device, including its read-only
            # frame/payload sizes; do not invent a USB bandwidth value.
            entry = {"requested_width": width, "requested_height": height,
                     "requested_interval_100ns": TARGET_INTERVAL}
            report["results"].append(entry)
            set_probe(request)
            entry["response"] = decode_probe(get_probe())
            entry["accepted_60fps_probe"] = (
                entry["response"]["format_index"] == 1
                and entry["response"]["frame_index"] == index
                and entry["response"]["interval_100ns"] in (166666, 166667))
        report["status"] = "probe_complete"
    except BaseException as error:
        report["error"] = type(error).__name__ + ": " + str(error)
    finally:
        recovery = report["recovery"]
        if original is not None and claimed:
            try:
                set_probe(original)
                restored = get_probe()
                recovery["restored_probe"] = decode_probe(restored)
                recovery["probe_restored"] = restored[2:8] == original[2:8]
                if not recovery["probe_restored"]:
                    recovery["errors"].append("Original format/frame/interval was not restored")
            except BaseException as error:
                recovery["errors"].append("Restore probe: " + str(error))
        if claimed:
            try:
                usb_util.release_interface(device, STREAM_INTERFACE)
            except BaseException as error:
                recovery["errors"].append("Release interface: " + str(error))
        try:
            # Closing the usbfs handle also releases any remaining interface claim.
            usb_util.dispose_resources(device)
        except BaseException as error:
            recovery["errors"].append("Close USB handle: " + str(error))
        if detach_attempted:
            try:
                # Bind VC0 first: its probe automatically claims and rebuilds VS1.
                # Attaching VS1 alone cannot reconstruct this UVC function.
                if originally_bound.get(0) and not device.is_kernel_driver_active(0):
                    device.attach_kernel_driver(0)
                deadline = time.monotonic() + 5
                while time.monotonic() < deadline:
                    bound = {str(i): device.is_kernel_driver_active(i) for i in (0, 1)}
                    if all(bound.values()):
                        break
                    time.sleep(0.1)
                recovery["drivers_bound"] = bound
                if not all(bound.values()):
                    recovery["errors"].append("uvcvideo did not rebind both interfaces; reconnect USB")
            except BaseException as error:
                recovery["errors"].append("Rebind driver: " + str(error))
        try:
            usb_util.dispose_resources(device)
        except BaseException as error:
            recovery["errors"].append("Final close: " + str(error))
        recovery["ok"] = not recovery["errors"]
        if recovery["errors"]:
            report["status"] = "recovery_failed"
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--apply", action="store_true",
                        help="temporarily detach this camera and send standard PROBE requests")
    args = parser.parse_args(argv)
    report = {"apply": args.apply}
    try:
        if args.apply and os.geteuid() != 0:
            raise RuntimeError("--apply needs root USB access; close the preview, then run with sudo")
        identity = discover_device()
        report["device"] = identity
        if not args.apply:
            report.update({"status": "dry_run_no_changes", "requires_root_for_apply": True,
                           "requires_all_camera_clients_closed": True,
                           "requests": {"format": "MJPG", "format_index": 1,
                                        "frame_indices": [i for i, _, _ in MODES],
                                        "interval_100ns": TARGET_INTERVAL,
                                        "probe_bytes": PROBE_LENGTH,
                                        "commits_stream": False}})
        else:
            users = camera_users(identity["video_nodes"])
            if users:
                report["camera_users"] = users
                raise RuntimeError("Camera is in use; close every preview/capture application first")
            import usb.core
            import usb.util
            device = usb.core.find(idVendor=VID, idProduct=PID,
                                   bus=identity["bus"], address=identity["address"])
            if device is None:
                raise RuntimeError("Camera identity changed before USB open")
            report.update(perform_probe(device, usb.util))
        print(json.dumps(report, indent=2))
        return 0 if report["status"] in ("dry_run_no_changes", "probe_complete") else 1
    except Exception as error:
        report.update({"status": "refused", "error": type(error).__name__ + ": " + str(error)})
        print(json.dumps(report, indent=2))
        return 1


if __name__ == "__main__":
    sys.exit(main())
