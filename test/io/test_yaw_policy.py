"""Pure image-centering policy regression; no device or flight simulation."""
import copy
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from px4_go.yaw_policy import VisionSample, parse_visual_report, yaw_rate_for_sample


class YawPolicyTests(unittest.TestCase):
    def report(self):
        return {
            "source": "automatic_armor_rgb_pnp",
            "coordinate_frame": "rgb_optical",
            "detection_valid": True,
            "tracking_confirmed": True,
            "generation": 0,
            "frame": 3,
            "track_id": 1,
            "valid": False,
            "reason": "rgb_calibration_missing",
            "sample_time_monotonic_s": 10.0,
            "frame_age_ms": 100.0,
            "armor_corners_px": [[400, 100], [560, 100], [560, 200], [400, 200]],
        }

    def parse(self, report=None, **parameters):
        arguments = dict(now=10.1, width=640, height=480)
        arguments.update(parameters)
        return parse_visual_report(self.report() if report is None else report, **arguments)

    def rate(self, sample, **parameters):
        arguments = dict(now=10.1, gain=0.8, max_rate=0.3, deadband=0.05)
        arguments.update(parameters)
        return yaw_rate_for_sample(sample, **arguments)

    def test_uncalibrated_confirmed_2d_report_is_usable(self):
        sample = self.parse()
        self.assertEqual(sample, VisionSample(10.0, 0, 3, 1, 0.5))
        report = self.report()
        report.pop("valid")
        self.assertEqual(self.parse(report), sample)

    def test_required_report_contract_and_strict_identity_types(self):
        invalid = {
            "source": [None, "manual_depth_roi"],
            "coordinate_frame": [None, "local_ned"],
            "detection_valid": [None, False, 1, "true"],
            "tracking_confirmed": [None, False, 1, "true"],
            "generation": [None, -1, True, 1.0, "1"],
            "frame": [None, 0, -1, True, 3.0, "3"],
            "track_id": [None, 0, -1, True, 1.0, "1"],
        }
        for field, values in invalid.items():
            for value in values:
                with self.subTest(field=field, value=value):
                    report = self.report()
                    report[field] = value
                    self.assertIsNone(self.parse(report))
        for value in (None, [], "report", 1, True):
            with self.subTest(report=value):
                self.assertIsNone(parse_visual_report(value, now=10.1, width=640, height=480))

    def test_reject_missing_required_fields(self):
        for field in self.report():
            if field in ("valid", "reason"):
                continue
            with self.subTest(field=field):
                report = self.report()
                del report[field]
                self.assertIsNone(self.parse(report))

    def test_report_age_and_future_sample(self):
        self.assertIsNotNone(self.parse(now=10.25))
        self.assertIsNone(self.parse(now=10.25001))
        self.assertIsNone(self.parse(now=9.999))
        for field in ("sample_time_monotonic_s", "frame_age_ms"):
            for value in (None, -1, float("nan"), float("inf"), -float("inf"),
                          True, "10", 10**1000):
                with self.subTest(field=field, value=value):
                    report = self.report()
                    report[field] = value
                    self.assertIsNone(self.parse(report))
        report = self.report()
        report["frame_age_ms"] = 250
        self.assertIsNotNone(self.parse(report))
        report["frame_age_ms"] = 250.0001
        self.assertIsNone(self.parse(report))

    def test_validate_four_in_image_numeric_corners(self):
        for corners in (None, [], [[0, 0]] * 3, [[0, 0]] * 5,
                        "not corners", {"x": 0, "y": 0}):
            with self.subTest(corners=corners):
                report = self.report()
                report["armor_corners_px"] = corners
                self.assertIsNone(self.parse(report))
        for point in ([640, 0], [-0.01, 0], [0, 480], [0, -0.01],
                      [float("nan"), 0], [0, float("inf")], [True, 0],
                      ["1", 0], [None, 0], [10**1000, 0],
                      [0], [0, 0, 0], {"x": 0, "y": 0}, None):
            with self.subTest(point=point):
                report = self.report()
                report["armor_corners_px"][0] = point
                self.assertIsNone(self.parse(report))
        report = self.report()
        report["armor_corners_px"] = [[0, 0], [639.9, 0], [639.9, 479.9], [0, 479.9]]
        self.assertIsNotNone(self.parse(report))

    def test_right_positive_left_negative_and_direction(self):
        sample = self.parse()
        self.assertAlmostEqual(self.rate(sample, max_rate=1), 0.4)
        left = copy.deepcopy(self.report())
        left["armor_corners_px"] = [[80, 100], [240, 100], [240, 200], [80, 200]]
        self.assertAlmostEqual(self.rate(self.parse(left), max_rate=1), -0.4)
        self.assertAlmostEqual(self.rate(sample, direction=-1, max_rate=1), -0.4)

    def test_deadband_max_rate_and_gain_zero(self):
        sample = self.parse()
        self.assertEqual(self.rate(sample), 0.3)
        self.assertEqual(self.rate(sample, direction=-1), -0.3)
        self.assertEqual(self.rate(sample, gain=0), 0.0)
        self.assertEqual(self.rate(VisionSample(10, 0, 3, 1, 0.05)), 0.0)
        self.assertEqual(self.rate(VisionSample(10, 0, 3, 1, -0.05)), 0.0)
        self.assertGreater(self.rate(VisionSample(10, 0, 3, 1, 0.05001)), 0.0)
        self.assertEqual(self.rate(VisionSample(10, 0, 3, 1, 0.0), deadband=0), 0.0)

    def test_no_sample_expired_or_future_sample_yields_zero(self):
        self.assertEqual(self.rate(None), 0.0)
        sample = self.parse()
        self.assertNotEqual(self.rate(sample, now=10.25), 0.0)
        self.assertEqual(self.rate(sample, now=10.25001), 0.0)
        self.assertEqual(self.rate(sample, now=9.99), 0.0)

    def test_invalid_parse_parameters_raise_value_error(self):
        invalid = {
            "now": [-1, float("nan"), float("inf"), True, "10", None],
            "max_age_s": [0, -1, float("nan"), True, "0.25", None],
            "width": [0, -1, True, 640.0, "640", None, 10**1000],
            "height": [0, -1, True, 480.0, "480", None, 10**1000],
        }
        for field, values in invalid.items():
            for value in values:
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    self.parse(**{field: value})

    def test_invalid_yaw_parameters_raise_value_error_even_without_sample(self):
        invalid = {
            "now": [-1, float("nan"), float("inf"), True, "10", None],
            "gain": [-1, float("nan"), float("inf"), True, "1", None],
            "max_rate": [0, -1, float("nan"), float("inf"), True, "1", None],
            "deadband": [-1, 1.001, float("nan"), float("inf"), True, "0", None],
            "direction": [0, 2, -2, float("nan"), float("inf"), True, "1", None],
            "max_age_s": [0, -1, float("nan"), float("inf"), True, "1", None],
        }
        for field, values in invalid.items():
            for value in values:
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    self.rate(None, **{field: value})
        with self.assertRaises(ValueError):
            self.rate({"error_x": 0.5})

    def test_invalid_direct_sample_construction_is_rejected(self):
        for values in ((-1, 0, 1, 1, 0), (10, -1, 1, 1, 0),
                       (10, 0, True, 1, 0), (10, 0, 1, 0, 0),
                       (10, 0, 1, 1, float("nan")), (10, 0, 1, 1, 1.1)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                VisionSample(*values)


if __name__ == "__main__":
    unittest.main()
