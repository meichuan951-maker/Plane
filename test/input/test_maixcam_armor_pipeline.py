"""Device API routing checks; no camera, MaixPy SDK, or network is opened."""
import contextlib
import importlib.util
import io
import itertools
import os
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, call, patch

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "maixcam_armor_preview", ROOT / "tools/maixcam/preview_maixvision.py")
preview = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preview)


def blob(x, code=1, y=100, width=6, length=50, pixels=None):
    return types.SimpleNamespace(
        code=lambda: code,
        pixels=lambda: width * length if pixels is None else pixels,
        rect=lambda: (x, y, width, length),
        mini_corners=lambda: [(x, y), (x + width, y),
                              (x + width, y + length), (x, y + length)])


class ArmorPipelineTest(unittest.TestCase):
    def setUp(self):
        self.patches = [patch.object(preview, "TARGET_COLOR", "both"),
                        patch.object(preview, "DETECTION_WINDOW", None)]
        for item in self.patches:
            item.start()
            self.addCleanup(item.stop)

    def test_red_and_blue_armors_are_detected_separately(self):
        frame = Mock()
        frame.find_blobs.return_value = [blob(20), blob(140),
                                        blob(350, 2), blob(470, 2)]
        result = preview.detect_armors(frame)
        self.assertEqual(len(result["bars"]), 4)
        self.assertEqual([a["color"] for a in result["armors"]], [0, 1])
        kwargs = frame.find_blobs.call_args.kwargs
        self.assertFalse(kwargs["merge"])
        self.assertEqual(frame.find_blobs.call_args.args[0], preview.LAB_THRESHOLDS)

    def test_mixed_color_and_speckled_candidates_are_rejected(self):
        frame = Mock()
        frame.find_blobs.return_value = [blob(20, 3), blob(140, 0),
                                        blob(250, pixels=1)]
        result = preview.detect_armors(frame)
        self.assertEqual(result["bars"], [])
        self.assertEqual(result["armors"], [])

    def test_empty_scene_does_not_retain_previous_armor(self):
        frame = Mock()
        frame.find_blobs.side_effect = [[blob(20), blob(140)], []]
        self.assertEqual(len(preview.detect_armors(frame)["armors"]), 1)
        self.assertEqual(preview.detect_armors(frame)["armors"], [])

    def test_thin_white_core_edges_are_merged_before_normal_aspect_filter(self):
        frame = Mock()
        frame.find_blobs.return_value = [blob(x, width=1, length=56)
                                        for x in (100, 108, 235, 243)]
        result = preview.detect_armors(frame)
        self.assertEqual(len(result["bars"]), 2)
        self.assertEqual(len(result["armors"]), 1)
        frame.find_blobs.return_value = [blob(100, width=1, length=56)]
        self.assertEqual(preview.detect_armors(frame)["bars"], [])

    def test_native_roi_uses_global_coordinates_without_offset(self):
        frame = Mock()
        frame.find_blobs.return_value = [blob(210), blob(330)]
        with patch.object(preview, "DETECTION_WINDOW", (.25, .25, .5, .5)):
            result = preview.detect_armors(frame)
        self.assertEqual(frame.find_blobs.call_args.kwargs["roi"], [160, 90, 320, 180])
        self.assertEqual(result["armors"][0]["center"], (273.0, 125.0))

    def test_blue_only_threshold_code_one_is_still_blue(self):
        frame = Mock()
        frame.find_blobs.return_value = [blob(210, 1), blob(330, 1)]
        with patch.object(preview, "TARGET_COLOR", "blue"):
            result = preview.detect_armors(frame)
        self.assertEqual(frame.find_blobs.call_args.args[0], [preview.LAB_THRESHOLDS[1]])
        self.assertEqual(result["armors"][0]["color"], 1)

    def test_degenerate_narrow_native_corners_preserve_small_lights(self):
        left, right = blob(280, width=1, length=10), blob(305, width=1, length=10)
        left.mini_corners = lambda: [(280, 100), (280, 100), (280, 110), (280, 110)]
        right.mini_corners = lambda: [(305, 100), (305, 100), (305, 110), (305, 110)]
        frame = Mock()
        frame.find_blobs.return_value = [left, right]
        result = preview.detect_armors(frame)
        self.assertEqual(len(result["bars"]), 2)
        self.assertEqual(len(result["armors"]), 1)

    def test_bad_roi_is_explicit_and_never_falls_back_to_full_scan(self):
        frame = Mock()
        with patch.object(preview, "DETECTION_WINDOW", (.9, .4, .2, .2)):
            with self.assertRaisesRegex(ValueError, "bounds"):
                preview.detect_armors(frame)
        frame.find_blobs.assert_not_called()

    def run_camera(self, frame, enabled=True, frame_count=1):
        sdk = types.ModuleType("maix")
        sdk.app = types.SimpleNamespace(need_exit=Mock(side_effect=[False] * frame_count + [True]))
        sdk.log = types.SimpleNamespace(
            LogLevel=types.SimpleNamespace(LEVEL_WARN=2), set_log_level=Mock())
        sdk.image = types.SimpleNamespace(COLOR_BLACK=0, COLOR_GREEN=1,
                                         COLOR_RED=2, COLOR_BLUE=3, COLOR_WHITE=4)
        cam = Mock()
        cam.read.return_value = frame
        cam.exposure.side_effect = lambda *args: args[0] if args else 1490
        sdk.camera = types.SimpleNamespace(Camera=Mock(return_value=cam),
                                          AeMode=types.SimpleNamespace(Manual=1, Auto=0))
        sdk.display = types.SimpleNamespace(send_to_maixvision=Mock())
        clock = itertools.count(0, .01)
        output = io.StringIO()
        with patch.dict(sys.modules, {"maix": sdk}), \
             patch.dict(os.environ, {"MAIXVISION": "1"}), \
             patch.object(preview, "ENABLE_ARMOR_DETECTION", enabled), \
             patch.object(preview.time, "monotonic", side_effect=lambda: next(clock)), \
             contextlib.redirect_stdout(output):
            preview.main()
        sdk.test_output = output.getvalue()
        return sdk, cam

    def test_exposure_is_read_back_separately_after_five_frames(self):
        frame = Mock()
        frame.find_blobs.return_value = []
        sdk, cam = self.run_camera(frame, frame_count=5)
        self.assertEqual(cam.exposure.call_args_list, [call(preview.EXPOSURE_US), call()])
        self.assertIn("Exposure readback_us=1490 requested_us=1500", sdk.test_output)

    def test_none_exposure_restores_auto(self):
        frame = Mock()
        with patch.object(preview, "EXPOSURE_US", None):
            sdk, cam = self.run_camera(frame, enabled=False)
        cam.exp_mode.assert_called_once_with(sdk.camera.AeMode.Auto)
        cam.exposure.assert_not_called()

    def test_detection_happens_before_drawing_and_frame_is_sent_once(self):
        frame = Mock()
        frame.find_blobs.return_value = [blob(20), blob(140)]
        sdk, cam = self.run_camera(frame)
        methods = [call[0] for call in frame.mock_calls]
        self.assertEqual(methods[0], "find_blobs")
        self.assertIn("draw_line", methods)
        self.assertLess(methods.index("draw_rect"), methods.index("draw_line"))
        self.assertTrue(any("ARMOR" in call.args[2]
                            for call in frame.draw_string.call_args_list))
        sdk.display.send_to_maixvision.assert_called_once_with(frame)
        cam.close.assert_called_once()
        cam.exp_mode.assert_called_once_with(sdk.camera.AeMode.Manual)
        cam.exposure.assert_called_once_with(preview.EXPOSURE_US)

    def test_pure_preview_switch_skips_detection(self):
        frame = Mock()
        sdk, cam = self.run_camera(frame, enabled=False)
        frame.find_blobs.assert_not_called()
        sdk.display.send_to_maixvision.assert_called_once_with(frame)
        cam.close.assert_called_once()

    def test_camera_closes_on_detector_failure(self):
        frame = Mock()
        frame.find_blobs.side_effect = RuntimeError("detector failed")
        camera = Mock()
        camera.read.return_value = frame
        sdk = types.ModuleType("maix")
        sdk.app = types.SimpleNamespace(need_exit=lambda: False)
        sdk.camera = types.SimpleNamespace(Camera=Mock(return_value=camera),
                                          AeMode=types.SimpleNamespace(Manual=1, Auto=0))
        sdk.display = types.SimpleNamespace()
        sdk.image = types.SimpleNamespace()
        sdk.log = types.SimpleNamespace(
            LogLevel=types.SimpleNamespace(LEVEL_WARN=2), set_log_level=Mock())
        with patch.dict(sys.modules, {"maix": sdk}), \
             patch.dict(os.environ, {"MAIXVISION": "1"}), \
             patch.object(preview, "ENABLE_ARMOR_DETECTION", True), \
             contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError, "detector failed"):
                preview.main()
        camera.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
