"""Tests score predictions across all modes using offline replay."""

import os
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

import predict_scores


class TestPredictions(unittest.TestCase):
    def test_demo_reaches_full_score(self):
        res = predict_scores.replay("demo")
        self.assertEqual(res.score, 1.00)
        self.assertTrue(res.success)
        self.assertTrue(res.details["locked_in_band_left"])
        self.assertTrue(res.details["locked_in_band_right"])
        self.assertTrue(res.details["has_reversed_right"])

    def test_naive_ablation_scores_0_50(self):
        res = predict_scores.replay("naive")
        self.assertEqual(res.score, 0.50)
        self.assertFalse(res.success)
        self.assertTrue(res.details["locked_in_band_left"])
        self.assertFalse(res.details["locked_in_band_right"])
        # Right dial ended at 46 mm (outside band 40 +- 2.5 mm)
        self.assertAlmostEqual(res.details["displayed_right"], 46.0)

    def test_no_lock_ablation_scores_0_25(self):
        res = predict_scores.replay("no_lock")
        self.assertEqual(res.score, 0.25)
        self.assertFalse(res.success)
        self.assertFalse(res.details["locked_left"])
        self.assertFalse(res.details["locked_right"])

    def test_settle_ablation_scores_0_00(self):
        res = predict_scores.replay("settle")
        self.assertEqual(res.score, 0.00)
        self.assertFalse(res.success)
        self.assertFalse(res.details["knob_moved"])


if __name__ == "__main__":
    unittest.main()
