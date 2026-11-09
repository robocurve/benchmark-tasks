"""Tests for predicted scores in compress-and-lock-spring-cartridge."""

import unittest
from predict_scores import predict_all


class TestPredictions(unittest.TestCase):
    def test_prediction_values(self):
        p = predict_all()
        self.assertEqual(p["demo"], 1.00)
        self.assertEqual(p["order_bar_first_arm"], 1.00)
        self.assertEqual(p["one_arm"], 0.25)
        self.assertEqual(p["release_early"], 0.25)
        self.assertEqual(p["bar_first"], 0.00)
        self.assertEqual(p["no_release"], 0.50)
        self.assertEqual(p["settle"], 0.00)
        self.assertEqual(p["dream"], 0.00)

    def test_ladder_ordering(self):
        p = predict_all()
        self.assertLess(p["bar_first"], p["one_arm"])
        self.assertLess(p["one_arm"], p["no_release"])
        self.assertLess(p["no_release"], p["demo"])


if __name__ == "__main__":
    unittest.main()
