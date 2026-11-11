"""Tests for predicted scores in bridge-plate-across-trench."""

import unittest
from predict_scores import PREDICTIONS, predict_all


class TestPredictions(unittest.TestCase):
    def test_predictions_values(self):
        p = predict_all()
        self.assertEqual(p["demo"], 1.00)
        self.assertEqual(p["no_bridge_push"], 0.00)
        self.assertEqual(p["bridge_short_overlap"], 0.00)
        self.assertEqual(p["skip_target"], 0.50)
        self.assertEqual(p["settle"], 0.00)
        self.assertEqual(p["dream"], 0.00)

    def test_ladder_ordering(self):
        p = predict_all()
        self.assertGreater(p["demo"], p["skip_target"])
        self.assertGreater(p["skip_target"], p["no_bridge_push"])
        self.assertEqual(p["no_bridge_push"], p["bridge_short_overlap"])
        self.assertEqual(p["bridge_short_overlap"], p["settle"])
        self.assertEqual(p["settle"], p["dream"])


if __name__ == "__main__":
    unittest.main()
