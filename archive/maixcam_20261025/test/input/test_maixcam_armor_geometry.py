"""Offline checks for MaixCAM armor geometry; no camera SDK or model required."""
import importlib.util
import itertools
import math
from pathlib import Path
import random
import unittest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "maixcam_preview", ROOT / "tools/maixcam/preview_maixvision.py")
preview = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(preview)


def corners(cx, cy=100, length=56, width=6, angle=0):
    radians = math.radians(angle)
    cs, sn = math.cos(radians), math.sin(radians)
    return tuple((cx + u * cs + v * sn, cy - u * sn + v * cs)
                 for u, v in ((-width / 2, -length / 2), (width / 2, -length / 2),
                              (width / 2, length / 2), (-width / 2, length / 2)))


def bar(cx, cy=100, length=56, width=6, color=0, angle=0):
    return preview.light_bar_from_corners(color, corners(cx, cy, length, width, angle))


class LightBarGeometryTest(unittest.TestCase):
    def test_corner_order_does_not_change_endpoints(self):
        rectangle = corners(100, 100, 56, 6, 15)
        for permutation in itertools.permutations(rectangle):
            result = preview.light_bar_from_corners(0, permutation)
            self.assertIsNotNone(result)
            self.assertAlmostEqual(result["length"], 56)
            self.assertAlmostEqual(result["width"], 6)
            self.assertAlmostEqual(result["tilt_deg"], 15)
            self.assertLess(result["top"][1], result["bottom"][1])

    def test_non_light_shapes_are_rejected(self):
        self.assertIsNone(bar(100, width=40))
        self.assertIsNone(bar(100, length=5, width=1))
        self.assertIsNone(bar(100, angle=70))
        self.assertIsNone(preview.light_bar_from_corners(2, corners(100)))
        self.assertIsNone(preview.light_bar_from_corners(0, [(0, 0)] * 4))
        self.assertIsNone(preview.light_bar_from_corners(0, [(0, 0), (1, 1), (2, 2), (3, 3)]))
        self.assertIsNone(preview.light_bar_from_corners(0, [(0, 0), (1, 1), (2, 2), (math.nan, 3)]))

    def test_white_core_side_fragments_merge_into_one_light(self):
        fragments = [bar(x, width=2) for x in (100, 108, 235, 243)]
        merged = preview.merge_light_bar_fragments(fragments)
        self.assertEqual(len(merged), 2)
        self.assertAlmostEqual(merged[0]["width"], 10)
        self.assertAlmostEqual(merged[0]["center"][0], 104)
        self.assertEqual(len(preview.pair_light_bars(merged)), 1)

    def test_fragment_merge_preserves_color_and_vertical_separation(self):
        self.assertEqual(len(preview.merge_light_bar_fragments(
            [bar(100), bar(108, color=1)])), 2)
        self.assertEqual(len(preview.merge_light_bar_fragments(
            [bar(100), bar(108, cy=155)])), 2)
        self.assertEqual(len(preview.merge_light_bar_fragments(
            [bar(100), bar(235)])), 2)

    def test_fragment_merge_does_not_chain_across_many_lights(self):
        merged = preview.merge_light_bar_fragments([bar(x) for x in (100, 110, 120)])
        self.assertEqual(len(merged), 2)
        self.assertLess(max(b["width"] for b in merged), 20)


class ArmorPairingTest(unittest.TestCase):
    def test_same_color_pair_has_ordered_quad(self):
        result = preview.pair_light_bars([bar(20), bar(155)])
        self.assertEqual(len(result), 1)
        armor = result[0]
        self.assertEqual(armor["color"], 0)
        self.assertEqual(armor["corners"], ((20.0, 72.0), (155.0, 72.0),
                                            (155.0, 128.0), (20.0, 128.0)))
        self.assertGreaterEqual(armor["score"], .25)
        self.assertLessEqual(armor["score"], 1)

    def test_incompatible_pair_geometry_is_rejected(self):
        scenarios = (
            [bar(20), bar(155, color=1)],
            [bar(20), bar(155, length=100)],
            [bar(20), bar(155, cy=140)],
            [bar(20, angle=15), bar(155, angle=-15)],
            [bar(20, angle=30), bar(155, cy=120, angle=30)],
            [bar(20), bar(28)],
            [bar(20), bar(500)],
        )
        for bars in scenarios:
            with self.subTest(bars=bars):
                self.assertEqual(preview.pair_light_bars(bars), [])

    def test_intervening_same_color_light_blocks_wide_pair(self):
        result = preview.pair_light_bars([bar(20), bar(90), bar(155)])
        self.assertEqual(len(result), 1)
        self.assertNotEqual(result[0]["bar_indices"], (0, 2))
        result = preview.pair_light_bars([bar(20), bar(90, color=1), bar(155)])
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["bar_indices"], (0, 2))

    def test_matching_preserves_two_adjacent_armors(self):
        bars = [bar(x) for x in (20, 155, 255, 390)]
        result = preview.pair_light_bars(bars)
        self.assertEqual([a["bar_indices"] for a in result], [(0, 1), (2, 3)])
        original = [(a["corners"], a["score"]) for a in result]
        random.Random(42).shuffle(bars)
        shuffled = preview.pair_light_bars(bars)
        self.assertEqual([(a["corners"], a["score"]) for a in shuffled], original)

    def test_small_or_misaligned_middle_noise_does_not_hide_a_plate(self):
        for noise in (bar(90, length=8, width=2), bar(90, angle=35)):
            with self.subTest(noise=noise):
                result = preview.pair_light_bars([bar(20), noise, bar(155)])
                self.assertEqual([a["bar_indices"] for a in result], [(0, 2)])

    def test_small_pair_accepts_pixel_scale_quantization_without_lowering_min_score(self):
        bars = [bar(100, length=8, width=2, angle=-3),
                bar(120, cy=102, length=10, width=2, angle=3)]
        result = preview.pair_light_bars(bars)
        self.assertEqual(len(result), 1)
        self.assertGreaterEqual(result[0]["score"], preview.ARMOR_GEOMETRY["pair_min_score"])
        self.assertEqual(result[0]["corners"],
                         (bars[0]["top"], bars[1]["top"], bars[1]["bottom"], bars[0]["bottom"]))
        self.assertEqual(preview.pair_light_bars(bars, {"small_target_tolerance_px": 0}), [])
        self.assertEqual(preview.pair_light_bars(bars, {"small_target_max_length": 0}), [])

    def test_small_pair_still_obeys_original_hard_limits(self):
        scenarios = (
            [bar(100, length=8, width=2), bar(120, length=10, width=2, color=1)],
            [bar(100, length=8, width=2), bar(120, length=13, width=2)],
            [bar(100, length=8, width=2), bar(120, cy=105, length=10, width=2)],
            [bar(100, length=8, width=2, angle=-10), bar(120, length=10, width=2, angle=10)],
            [bar(100, length=8, width=2, angle=30), bar(120, cy=102, length=10, width=2, angle=30)],
            [bar(100, length=8, width=2), bar(105, length=10, width=2)],
            [bar(100, length=8, width=2), bar(160, length=10, width=2)],
        )
        for bars in scenarios:
            with self.subTest(bars=bars):
                self.assertEqual(preview.pair_light_bars(bars), [])

    def test_large_pair_scores_are_unchanged_by_small_target_tolerance(self):
        bars = [bar(100, length=56, angle=-2), bar(235, cy=106, length=62, angle=2)]
        normal = preview.pair_light_bars(bars)
        disabled = preview.pair_light_bars(bars, {"small_target_tolerance_px": 0})
        self.assertEqual(len(normal), 1)
        self.assertEqual(normal, disabled)

    def test_vertically_separated_plates_do_not_block_each_other(self):
        bars = [bar(x, cy=y) for y in (100, 230) for x in (20, 155)]
        self.assertEqual(len(preview.pair_light_bars(bars)), 2)

    def test_crowded_scene_is_bounded_and_never_reuses_lights(self):
        bars = [bar(20 + i * 50) for i in range(40)]
        result = preview.pair_light_bars(bars)
        self.assertEqual(len(result), preview.MAX_BARS // 2)
        used = [index for armor in result for index in armor["bar_indices"]]
        self.assertEqual(len(used), len(set(used)))
        self.assertLessEqual(len(used), preview.MAX_BARS)

    def test_invalid_measurements_are_skipped_without_stale_candidates(self):
        invalid = {"color": 0, "top": (0, 0), "bottom": (0, 0), "center": (0, 0)}
        nonfinite = {"color": 0, "top": (math.inf, 0), "bottom": (20, 56), "center": (20, 28)}
        self.assertEqual(preview.pair_light_bars([None, {}, invalid, nonfinite]), [])
        self.assertEqual(preview.pair_light_bars([]), [])

    def test_invalid_config_is_explicit(self):
        for config in ({"pair_max_angle_deg": 0}, {"pair_max_length_ratio": 1},
                       {"pair_min_score": math.nan}, {"unknown": 1},
                       {"small_target_max_length": -1}, {"small_target_tolerance_px": -1},
                       {"small_target_max_length": math.inf},
                       {"small_target_tolerance_px": math.nan}):
            with self.subTest(config=config), self.assertRaises(ValueError):
                preview.pair_light_bars([], config)


if __name__ == "__main__":
    unittest.main()
