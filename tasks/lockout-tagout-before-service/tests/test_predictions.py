"""Tests for predicted scores in lockout-tagout-before-service."""

import unittest
from predict_scores import PREDICTIONS, predict_all


class TestPredictions(unittest.TestCase):
    def test_predictions_values(self):
        p = predict_all()
        self.assertEqual(p["demo"], 1.00)
        self.assertEqual(p["cover_first"], 0.00)
        self.assertEqual(p["no_pin"], 0.25)
        self.assertEqual(p["no_test"], 0.50)
        self.assertEqual(p["no_hold"], 0.75)
        self.assertEqual(p["order_swapped"], 0.00)
        self.assertEqual(p["settle"], 0.00)

    def test_ladder_ordering(self):
        p = predict_all()
        self.assertLess(p["cover_first"], p["demo"])
        self.assertLess(p["no_pin"], p["no_test"])
        self.assertLess(p["no_test"], p["no_hold"])
        self.assertLess(p["no_hold"], p["demo"])


if __name__ == "__main__":
    unittest.main()

