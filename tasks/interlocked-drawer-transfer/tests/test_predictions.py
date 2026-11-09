"""Tests for predicted scores in interlocked-drawer-transfer."""

import unittest
from predict_scores import PREDICTIONS, predict_all


class TestPredictions(unittest.TestCase):
    def test_predictions_values(self):
        p = predict_all()
        self.assertEqual(p["demo"], 1.00)
        self.assertEqual(p["park"], 1.00)
        self.assertEqual(p["both_open"], 0.25)
        self.assertEqual(p["leave_open"], 0.75)
        self.assertEqual(p["wrong_drawer"], 0.25)
        self.assertEqual(p["block_dropped"], 0.25)
        self.assertEqual(p["settle"], 0.00)
        self.assertEqual(p["dream"], 0.00)

    def test_ladder_ordering(self):
        p = predict_all()
        self.assertEqual(p["settle"], 0.00)
        self.assertEqual(p["block_dropped"], 0.25)
        self.assertLessEqual(p["both_open"], p["wrong_drawer"])
        self.assertLess(p["wrong_drawer"], p["leave_open"])
        self.assertLess(p["leave_open"], p["demo"])
        self.assertEqual(p["demo"], p["park"])


if __name__ == "__main__":
    unittest.main()
