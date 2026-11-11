"""Unit tests verifying predicted scores."""

import unittest

import predict_scores


class TestPredictions(unittest.TestCase):
    def test_predictions_complete(self):
        expected_modes = {"demo", "knock_neighbour", "stop_over_gap", "settle", "dream"}
        preds = predict_scores.predict_all()
        self.assertEqual(set(preds.keys()), expected_modes)

    def test_predicted_scores(self):
        self.assertEqual(predict_scores.predict("demo"), 1.00)
        self.assertEqual(predict_scores.predict("knock_neighbour"), 0.0)
        self.assertEqual(predict_scores.predict("stop_over_gap"), 0.00)
        self.assertEqual(predict_scores.predict("settle"), 0.00)
        self.assertEqual(predict_scores.predict("dream"), 0.00)


if __name__ == "__main__":
    unittest.main()

