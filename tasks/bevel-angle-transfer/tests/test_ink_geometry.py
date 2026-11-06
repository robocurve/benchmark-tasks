# SPDX-License-Identifier: MIT
"""Adversarial and invariance checks for final visible line geometry."""
import json
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bevel_angle_transfer.scoring import score_strokes


def line(start=(-0.06, 0), end=(0.06, 0), n=2):
    return np.linspace(start, end, n)


class ScoringTests(unittest.TestCase):
    def test_full_line_and_reversal(self):
        for points in (line(), line()[::-1]):
            result = score_strokes([points])
            self.assertEqual(result['score'], 1)
            self.assertAlmostEqual(result['coverage'], 1)
            self.assertAlmostEqual(result['stray_length'], 0)
            json.dumps(result, allow_nan=False)

    def test_subdivision_dwell_and_retracing_invariant(self):
        original = score_strokes([line()])
        dense = line(n=1001)
        for points in (dense, np.repeat(dense, 3, axis=0),
                       np.concatenate((dense, dense[::-1], dense))):
            result = score_strokes([points])
            for key in ('score', 'coverage', 'stray_length',
                        'unique_ink_length', 'angle_error_deg', 'line_rms_width'):
                self.assertAlmostEqual(original[key], result[key], places=8, msg=key)

    def test_collinear_overlap_and_adjoining_pen_lifts(self):
        self.assertEqual(score_strokes([line((-0.06, 0), (0, 0)),
                                      line((0, 0), (0.06, 0))])['score'], 1)
        result = score_strokes([line(), line((-0.02, 0), (0.04, 0), 20)])
        self.assertEqual(result['score'], 1)
        self.assertAlmostEqual(result['unique_ink_length'], 0.12, delta=.00025)

    def test_microjitter_and_overlapping_retraces_remain_one_visible_line(self):
        rng = np.random.default_rng(2026)
        base = line(n=1000)
        jitter = base.copy()
        jitter[:, 1] = rng.uniform(-.0002, .0002, len(jitter))
        result = score_strokes([jitter, jitter[::-1], base])
        self.assertEqual(result['score'], 1)
        self.assertLess(result['unique_ink_length'], .15)
        self.assertLess(result['line_rms_width'], .0005)
        self.assertLessEqual(result['raster_cells'], 2000000)
        self.assertEqual(result['raster_resolution'], .00025)
        # Thousands of tiny movements during one stationary contact remain a dot.
        dwell = rng.uniform(-.00015, .00015, (2000, 2))
        with_dwell = score_strokes([base, dwell])
        self.assertEqual(with_dwell['score'], 1)
        self.assertAlmostEqual(with_dwell['unique_ink_length'],
                               score_strokes([base])['unique_ink_length'], delta=.00025)

    def test_overlapping_contact_stamps_form_a_valid_visible_line(self):
        dense_stamps = [np.array([[x, 0]]) for x in np.linspace(-.06, .06, 121)]
        result = score_strokes(dense_stamps)
        self.assertEqual(result['score'], 1)
        self.assertAlmostEqual(result['coverage'], 1)
        self.assertAlmostEqual(result['full_max_gap'], 0)
        self.assertAlmostEqual(result['full_continuous_coverage'], 1)
        sparse_stamps = [np.array([[x, 0]]) for x in np.linspace(-.06, .06, 10)]
        self.assertEqual(score_strokes(sparse_stamps)['score'], 0)

    def test_short_lifted_dashes_with_overlapping_caps_match_visible_span(self):
        dashes = [line((x, 0), (x+.001, 0)) for x in np.arange(-.06, .06, .002)]
        result = score_strokes(dashes)
        self.assertEqual(result['score'], 1)
        self.assertAlmostEqual(result['coverage'], score_strokes([line()])['coverage'])
        self.assertAlmostEqual(result['full_max_gap'], 0)
        # Geometry is unioned; repeating stamps or dashes adds no ink.
        repeated = score_strokes(dashes*3)
        for key in ('score', 'coverage', 'unique_ink_length', 'stray_length'):
            self.assertAlmostEqual(result[key], repeated[key])

    def test_visible_lateral_retraces_still_count_as_extra_ink(self):
        strokes = [line((-.06, y), (.06, y)) for y in np.linspace(-.003, .003, 20)]
        result = score_strokes(strokes)
        self.assertEqual(result['score'], 0)
        self.assertGreater(result['unique_ink_length'], .18)

    def test_finite_grading_region_is_explicit_and_bounded(self):
        result = score_strokes([line(), line((.3, 0), (.4, 0))])
        self.assertEqual(result['score'], 0)
        self.assertTrue(result['ink_outside_grading_region'])
        self.assertLessEqual(result['raster_cells'], 2000000)

    def test_empty_spots_and_short_lines(self):
        for strokes in ([], [np.empty((0, 2))], [[[0, 0]]],
                        [np.zeros((10000, 2))], [line((-0.01, 0), (0.01, 0))],
                        [np.array([[x, 0]]) for x in np.linspace(-0.06, 0.06, 5)]):
            self.assertEqual(score_strokes(strokes)['score'], 0)

    def test_partial_and_boundaries(self):
        self.assertEqual(score_strokes([line((-0.03, 0), (0.03, 0))])['score'], .5)
        self.assertEqual(score_strokes([line((-0.054, 0), (0.054, 0))])['score'], 1)
        self.assertEqual(score_strokes([line((-0.06, .004), (0.06, .004))])['score'], .5)
        self.assertEqual(score_strokes([line((-0.06, .006), (0.06, .006))])['score'], 0)

    def test_lift_and_nonfinite_never_bridge(self):
        left = line((-0.06, 0), (-0.02, 0))
        right = line((0.02, 0), (0.06, 0))
        result = score_strokes([left, right])
        broken = score_strokes([np.vstack((left, [np.nan, 0], right))])
        self.assertEqual(result['score'], 0)
        self.assertAlmostEqual(result['coverage'], .082/.12)
        self.assertAlmostEqual(result['coverage'], broken['coverage'])
        self.assertAlmostEqual(result['full_max_gap'], .038)
        self.assertEqual(broken['invalid_points'], 1)
        self.assertEqual(score_strokes([[[np.inf, 0], [np.nan, 1]]])['score'], 0)

    def test_fragmented_dashes_with_large_gaps_fail(self):
        strokes = [line((x, 0), (x+.008, 0)) for x in np.arange(-.06, .06, .012)]
        # A 6 mm centreline break leaves 4 mm of visible unpainted board.
        high_coverage = [line((-.06, 0), (-.003, 0)), line((.003, 0), (.06, 0))]
        self.assertGreater(score_strokes(high_coverage)['coverage'], .9)
        self.assertEqual(score_strokes(high_coverage)['score'], .5)
        # 4 mm internal gaps exceed full credit's 3 mm criterion.
        self.assertEqual(score_strokes(strokes)['score'], .5)

    def test_off_target_stroke_spots_and_scribbles_fail(self):
        self.assertEqual(score_strokes([line(), line((-.06, .03), (.06, .03))])['score'], 0)
        dots = [np.array([[x, .03]]) for x in np.linspace(-.06, .06, 30)]
        self.assertEqual(score_strokes([line(), *dots])['score'], 0)
        painted = [line((-.06, y), (.06, y)) for y in np.linspace(-.02, .02, 50)]
        self.assertEqual(score_strokes(painted)['score'], 0)
        multiple_lines = [line((-.06, y), (.06, y)) for y in (-.002, 0, .002)]
        self.assertEqual(score_strokes(multiple_lines)['score'], 0)
        zigzag = np.column_stack((np.linspace(-.06, .06, 100),
                                  np.tile([-.002, .002], 50)))
        self.assertEqual(score_strokes([zigzag])['score'], 0)

    def test_angle_and_overlong_line(self):
        def rotated(degrees):
            theta = np.radians(degrees)
            return line() @ np.array([[np.cos(theta), np.sin(theta)],
                                      [-np.sin(theta), np.cos(theta)]])
        self.assertEqual(score_strokes([rotated(1)])['score'], 1)
        self.assertEqual(score_strokes([rotated(4)])['score'], .5)
        self.assertEqual(score_strokes([rotated(6)])['score'], 0)
        self.assertEqual(score_strokes([line((-.10, 0), (.10, 0))])['score'], 0)
        self.assertAlmostEqual(score_strokes([rotated(4)])['angle_error_deg'], 4, delta=.05)

    def test_arbitrary_target_frame(self):
        start, end = np.array([.4, -.3]), np.array([.4, -.18])
        result = score_strokes([line(start, end, 31)], start, end)
        self.assertEqual(result['score'], 1)
        self.assertAlmostEqual(result['target_length'], .12)

    def test_actual_140mm_reference_with_130mm_demonstration(self):
        result = score_strokes([line((-.065, 0), (.065, 0), 100)],
                               target_start=(-.07, 0), target_end=(.07, 0))
        self.assertEqual(result['score'], 1)
        self.assertAlmostEqual(result['target_length'], .14)
        self.assertAlmostEqual(result['coverage'], .132/.14)
        self.assertEqual(result['raster_resolution'], .00025)
        self.assertLessEqual(result['raster_cells'], 2000000)

    def test_small_far_stray_dot_respects_stray_allowance(self):
        result = score_strokes([line(), np.array([[0, .12]])])
        self.assertEqual(result['score'], 1)
        self.assertFalse(result['ink_outside_grading_region'])
        self.assertGreater(result['stray_length'], .001)
        self.assertLess(result['stray_length'], .005)

    def test_malformed_shapes_and_targets(self):
        for strokes in ([np.ones((2, 3))], [[1, 2]], [[[1]]]):
            with self.assertRaises(ValueError):
                score_strokes(strokes)
        for end in ((0, 0), (np.nan, 0)):
            with self.assertRaises(ValueError):
                score_strokes([], (0, 0), end)


if __name__ == '__main__':
    unittest.main()
