"""Run ON MaixCAM via MaixVision; keep running while the PC reads its UVC stream.

Encodes each real camera frame once for 640x360 MJPEG. MaixVision preview is
optional and reuses that JPEG without initializing a physical display.
This script contains no detection; the existing PC C++ detector processes frames.
"""
import os
import time

# Leave preview disabled while checking USB throughput. Sending RGB to both
# destinations makes the SDK encode it twice, with different JPEG qualities.
JPEG_QUALITY = 80
MAIXVISION_PREVIEW = False
# Full RGB -> Python bytes copies can perturb throughput on the device. Enable
# only when checking whether a black picture already originates at the sensor.
RGB_DIAGNOSTICS = False


def pixel_sample(frame):
    """A small sample of RGB bytes for diagnosis, not calibrated luminance."""
    data = frame.to_bytes()  # Default copy=True keeps buffer ownership safe.
    if not data:
        return "sample_rgb=empty"
    sample = data[::max(1, len(data) // 1024)]
    return "sample_rgb_mean=%.1f min=%d max=%d" % (
        sum(sample) / len(sample), min(sample), max(sample))


def main():
    from maix import app, camera, display, log, uvc, sys as device_sys

    # UvcStreamer.show() emits an INFO line for every frame. Avoid synchronous
    # console traffic/forwarding backpressure while preserving warnings/errors.
    log.set_log_level(log.LogLevel.LEVEL_WARN, False)

    if device_sys.device_id() != "maixcam":
        raise RuntimeError("This stream setup targets legacy MaixCAM; check the device model.")
    if not os.path.exists("/boot/usb.uvc"):
        raise RuntimeError("Run enable_uvc.py on MaixCAM first, then reconnect power.")
    # The official constructor opens an existing SHM segment; it does not create
    # the USB gadget or start its server. Fail clearly instead of calling it early.
    if not os.path.exists("/dev/shm/uvc_shared_mem_yuyv"):
        raise RuntimeError("UVC background server is not ready. Wait for boot; "
                           "if this persists, report the MaixCAM system/runtime version.")
    print("MaixCAM init: opening camera 640x360; waiting for the sensor...", flush=True)
    cam = camera.Camera(640, 360, fps=30)
    streamer = uvc.UvcStreamer()
    streamer.use_mjpg(1)
    preview = getattr(display, "send_to_maixvision", None) if MAIXVISION_PREVIEW else None
    if MAIXVISION_PREVIEW and not callable(preview):
        print("MaixVision preview API unavailable in this runtime; UVC will "
              "continue. Report the MaixPy version if preview is needed.", flush=True)
    print("Streaming real camera frames: 640x360 MJPEG quality=%d preview=%s "
          "rgb_diagnostics=%s sdk_log=WARN. Keep this script running." %
          (JPEG_QUALITY, bool(preview), RGB_DIAGNOSTICS), flush=True)
    frames = 0
    produced = 0
    next_preview = 0.0
    sample_available = RGB_DIAGNOSTICS
    last = time.monotonic()
    stages = [0.0] * 4
    while not app.need_exit():
        started = time.monotonic()
        frame = cam.read()
        captured = time.monotonic()
        if frame is None:
            raise RuntimeError("Camera returned no image; check MaixVision device logs.")
        # RGB -> JPEG is synchronous in this SDK. Reuse the encoded image so
        # UVC and optional preview do not compete for a second encode.
        jpeg = frame.to_jpeg(quality=JPEG_QUALITY)
        encoded = time.monotonic()
        if jpeg is None:
            raise RuntimeError("JPEG encoder returned no image; check device logs.")
        streamer.show(jpeg)
        sent = time.monotonic()
        frames += 1
        produced += 1
        if callable(preview) and sent >= next_preview:
            preview(jpeg)
            next_preview = sent + .2
        now = time.monotonic()
        for index, duration in enumerate((captured - started, encoded - captured,
                                          sent - encoded, now - sent)):
            stages[index] += duration
        if now - last >= 1:
            diagnostic = ""
            if sample_available:
                try:
                    diagnostic = pixel_sample(frame)
                except (AttributeError, TypeError, ValueError, RuntimeError) as error:
                    sample_available = False
                    diagnostic = "RGB diagnostic unavailable: " + str(error)
            print("MaixCAM stream: %.1f FPS produced=%d "
                  "read_ms=%.1f jpeg_ms=%.1f uvc_ms=%.1f preview_ms=%.1f %s" %
                  (frames / (now - last), produced,
                   *(duration * 1000 / frames for duration in stages), diagnostic),
                  flush=True)
            frames, last = 0, now
            stages = [0.0] * 4


if __name__ == "__main__":
    main()
