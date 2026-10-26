"""Exercise sender routing without importing the device SDK or opening hardware."""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "maixcam_stream", ROOT / "tools/maixcam/stream_uvc.py")
stream = importlib.util.module_from_spec(spec)
spec.loader.exec_module(stream)


class StreamTest(unittest.TestCase):
    def run_sender(self, frames, preview=False, preview_api=True, rgb_diagnostics=False):
        sdk = types.ModuleType("maix")
        sdk.log = types.SimpleNamespace(
            LogLevel=types.SimpleNamespace(LEVEL_WARN=2), set_log_level=Mock())
        sdk.sys = types.SimpleNamespace(device_id=lambda: "maixcam")
        sdk.app = types.SimpleNamespace(need_exit=Mock(side_effect=[False] * len(frames) + [True]))
        camera = Mock()
        camera.read.side_effect = frames
        sdk.camera = types.SimpleNamespace(Camera=Mock(return_value=camera))
        uvc = Mock()
        sdk.uvc = types.SimpleNamespace(UvcStreamer=Mock(return_value=uvc))
        sdk.display = types.SimpleNamespace()
        if preview_api:
            sdk.display.send_to_maixvision = Mock()
        output = io.StringIO()
        with patch.dict(sys.modules, {"maix": sdk}), \
             patch.object(stream.os.path, "exists", return_value=True), \
             patch.object(stream, "MAIXVISION_PREVIEW", preview), \
             patch.object(stream, "RGB_DIAGNOSTICS", rgb_diagnostics), \
             patch.object(stream.time, "monotonic", side_effect=[i * .2 for i in range(100)]), \
             contextlib.redirect_stdout(output):
            stream.main()
        return sdk, uvc, output.getvalue()

    def test_single_encode_reused_by_both_destinations(self):
        frames = [Mock(), Mock()]
        for frame in frames:
            frame.to_bytes.return_value = bytes([16, 32, 48]) * 128
        sdk, uvc, output = self.run_sender(frames, preview=True, rgb_diagnostics=True)
        jpegs = [frame.to_jpeg.return_value for frame in frames]
        for frame in frames:
            frame.to_jpeg.assert_called_once_with(quality=80)
        self.assertEqual([c.args[0] for c in uvc.show.call_args_list], jpegs)
        self.assertEqual([c.args[0] for c in sdk.display.send_to_maixvision.call_args_list], jpegs)
        self.assertIn("produced=2", output)
        self.assertIn("sample_rgb_mean=32.0", output)
        self.assertIn("jpeg_ms=", output)
        sdk.camera.Camera.assert_called_once_with(640, 360, fps=30)
        sdk.log.set_log_level.assert_called_once_with(sdk.log.LogLevel.LEVEL_WARN, False)

    def test_default_sender_does_not_send_maixvision_preview(self):
        frame = Mock()
        frame.to_bytes.return_value = b"\x10" * 1024
        sdk, uvc, _ = self.run_sender([frame])
        sdk.display.send_to_maixvision.assert_not_called()
        frame.to_bytes.assert_not_called()
        uvc.show.assert_called_once_with(frame.to_jpeg.return_value)

    def test_missing_preview_api_does_not_stop_uvc(self):
        frame = Mock()
        frame.to_bytes.return_value = b"\x10" * 1024
        _, uvc, output = self.run_sender([frame], preview=True, preview_api=False)
        uvc.show.assert_called_once_with(frame.to_jpeg.return_value)
        self.assertIn("preview API unavailable", output)

    def test_missing_camera_frame_is_explicit_failure(self):
        with self.assertRaisesRegex(RuntimeError, "Camera returned no image"):
            self.run_sender([None])

    def test_missing_encoded_image_is_explicit_failure(self):
        frame = Mock()
        frame.to_jpeg.return_value = None
        with self.assertRaisesRegex(RuntimeError, "JPEG encoder returned no image"):
            self.run_sender([frame])

    def test_empty_rgb_data_is_not_reported_as_a_valid_mean(self):
        frame = Mock()
        frame.to_bytes.return_value = b""
        self.assertEqual(stream.pixel_sample(frame), "sample_rgb=empty")


if __name__ == "__main__":
    unittest.main()
