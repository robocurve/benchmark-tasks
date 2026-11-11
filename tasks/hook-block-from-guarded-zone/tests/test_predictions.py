"""Tests for predicted scores in hook-block-from-guarded-zone."""

import unittest
from predict_scores import PREDICTIONS, predict, predict_all


class TestPredictions(unittest.TestCase):
    def test_predictions_values(self):
        p = predict_all()
        self.assertEqual(p["demo"], 1.00)
        self.assertEqual(p["reach_over"], 0.00)
        self.assertEqual(p["drop_rod_in_zone"], 0.25)
        self.assertEqual(p["leave_block_outside"], 0.75)
        self.assertEqual(p["settle"], 0.00)
        self.assertEqual(p["dream"], 0.00)

    def test_ladder_ordering(self):
        p = predict_all()
        self.assertGreater(p["demo"], p["leave_block_outside"])
        self.assertGreater(p["leave_block_outside"], p["drop_rod_in_zone"])
        self.assertGreater(p["drop_rod_in_zone"], p["reach_over"])
        self.assertEqual(p["reach_over"], p["settle"])
        self.assertEqual(p["settle"], p["dream"])

    def test_single_predict_call(self):
        self.assertEqual(predict("demo"), 1.00)
        self.assertEqual(predict("reach_over"), 0.00)


if __name__ == "__main__":
    unittest.main()
