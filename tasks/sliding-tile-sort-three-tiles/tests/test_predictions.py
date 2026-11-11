"""Unit tests verifying predicted score mappings."""

import unittest

import predict_scores


class TestPredictions(unittest.TestCase):
    def test_predictions_complete(self):
        expected_modes = {"demo", "wrong_order", "lift_tile", "settle", "dream"}
        preds = predict_scores.predict_all()
        self.assertEqual(set(preds.keys()), expected_modes)

    def test_predicted_scores(self):
        self.assertEqual(predict_scores.predict("demo"), 1.00)
        self.assertEqual(predict_scores.predict("wrong_order"), 0.50)
        self.assertEqual(predict_scores.predict("lift_tile"), 0.00)
        self.assertEqual(predict_scores.predict("settle"), 0.00)
        self.assertEqual(predict_scores.predict("dream"), 0.00)


if __name__ == "__main__":
    unittest.main()

