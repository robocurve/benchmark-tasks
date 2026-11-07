"""Tests for predicted scores in stack-crates-largest-first."""

import unittest
from predict_scores import PREDICTIONS, predict_all


class TestPredictions(unittest.TestCase):
    def test_predictions_values(self):
        p = predict_all()
        self.assertEqual(p["demo"], 1.00)
        self.assertEqual(p["wrong_order"], 0.00)
        self.assertEqual(p["offset"], 0.50)
        self.assertEqual(p["leave_small"], 0.50)
        self.assertEqual(p["knock"], 0.75)
        self.assertEqual(p["settle"], 0.00)

    def test_ladder_ordering(self):
        p = predict_all()
        self.assertLess(p["knock"], p["demo"])
        self.assertLess(p["wrong_order"], p["leave_small"])
        self.assertLessEqual(p["offset"], p["leave_small"])
        self.assertLess(p["offset"], p["demo"])


if __name__ == "__main__":
    unittest.main()

