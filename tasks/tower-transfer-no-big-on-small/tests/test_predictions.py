"""Tests for predicted scores in tower-transfer-no-big-on-small."""

import unittest
from predict_scores import PREDICTIONS, predict_all


class TestPredictions(unittest.TestCase):
    def test_predictions_values(self):
        p = predict_all()
        self.assertEqual(p["demo"], 1.00)
        self.assertEqual(p["big_on_small"], 0.00)
        self.assertEqual(p["leave_one"], 0.50)
        self.assertEqual(p["wrong_target"], 0.50)
        self.assertEqual(p["settle"], 0.00)
        self.assertEqual(p["dream"], 0.00)

    def test_ladder_ordering(self):
        p = predict_all()
        self.assertGreater(p["demo"], p["leave_one"])
        self.assertGreaterEqual(p["leave_one"], p["wrong_target"])
        self.assertGreater(p["wrong_target"], p["big_on_small"])
        self.assertEqual(p["big_on_small"], p["settle"])


if __name__ == "__main__":
    unittest.main()

