"""Tests for predicted scores in domino-chain-set-and-tip."""

import unittest
from predict_scores import PREDICTIONS, predict_all


class TestPredictions(unittest.TestCase):
    def test_predictions_values(self):
        p = predict_all()
        self.assertEqual(p["demo"], 1.00)
        self.assertEqual(p["gap_too_wide"], 0.25)
        self.assertEqual(p["no_tip"], 0.50)
        self.assertEqual(p["tip_too_early"], 0.25)
        self.assertEqual(p["settle"], 0.00)
        self.assertEqual(p["dream"], 0.00)

    def test_ladder_ordering(self):
        p = predict_all()
        self.assertGreater(p["demo"], p["gap_too_wide"])
        self.assertLessEqual(p["gap_too_wide"], p["no_tip"])
        self.assertGreater(p["no_tip"], p["tip_too_early"])
        self.assertGreater(p["tip_too_early"], p["settle"])
        self.assertEqual(p["settle"], p["dream"])


if __name__ == "__main__":
    unittest.main()
