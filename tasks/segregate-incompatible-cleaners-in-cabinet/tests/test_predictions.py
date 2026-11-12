"""Unit tests verifying predicted scores dictionary in segregate-incompatible-cleaners-in-cabinet."""

from __future__ import annotations

import unittest
from predict_scores import PREDICTIONS


class TestPredictions(unittest.TestCase):
    def test_expected_modes_present(self):
        expected_modes = {"demo", "same_bay_violation", "one_bottle_only", "settle"}
        self.assertEqual(set(PREDICTIONS.keys()), expected_modes)

    def test_scores_valid(self):
        for mode, data in PREDICTIONS.items():
            sc = data["predicted_score"]
            self.assertGreaterEqual(sc, 0.0)
            self.assertLessEqual(sc, 1.0)
            self.assertTrue(len(data["reason"]) > 10)

    def test_demo_is_full_score(self):
        self.assertEqual(PREDICTIONS["demo"]["predicted_score"], 1.00)

    def test_same_bay_violation_score(self):
        self.assertEqual(PREDICTIONS["same_bay_violation"]["predicted_score"], 0.50)

    def test_one_bottle_only_score(self):
        self.assertEqual(PREDICTIONS["one_bottle_only"]["predicted_score"], 0.25)

    def test_settle_score(self):
        self.assertEqual(PREDICTIONS["settle"]["predicted_score"], 0.00)


if __name__ == "__main__":
    unittest.main()
