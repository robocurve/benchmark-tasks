"""Unit tests for predicted scores across all modes in pack-cold-chain-shipper-with-thermal-buffer."""

from __future__ import annotations

import unittest
import predict_scores


class TestPredictions(unittest.TestCase):
    def test_all_modes_predicted(self):
        expected_modes = {
            "demo",
            "no_buffer_violation",
            "brick_only",
            "buffer_without_carton",
            "lid_unlatched",
            "no_lid",
            "settle",
            "dream",
        }
        preds = predict_scores.predict_all()
        self.assertEqual(set(preds.keys()), expected_modes)

    def test_demo_predicted_full_score(self):
        self.assertEqual(predict_scores.predict("demo"), 1.00)

    def test_no_buffer_violation_predicted_capped(self):
        self.assertEqual(predict_scores.predict("no_buffer_violation"), 0.25)

    def test_brick_only_predicted_quarter(self):
        self.assertEqual(predict_scores.predict("brick_only"), 0.25)

    def test_buffer_without_carton_predicted_half(self):
        self.assertEqual(predict_scores.predict("buffer_without_carton"), 0.50)

    def test_lid_ablations_predicted_below_full_score(self):
        self.assertLess(predict_scores.predict("lid_unlatched"), 1.0)
        self.assertEqual(predict_scores.predict("lid_unlatched"), 0.75)
        self.assertEqual(predict_scores.predict("no_lid"), 0.75)

    def test_idle_modes_predicted_zero(self):
        self.assertEqual(predict_scores.predict("settle"), 0.00)
        self.assertEqual(predict_scores.predict("dream"), 0.00)


if __name__ == "__main__":
    unittest.main()
