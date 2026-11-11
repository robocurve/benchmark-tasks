"""Unit tests for score prediction model in clear-lane-then-push-block-to-dock."""

import unittest
import predict_scores


class TestPredictions(unittest.TestCase):
    def test_prediction_values(self):
        self.assertEqual(predict_scores.predict("demo"), 1.00)
        self.assertEqual(predict_scores.predict("push_first"), 0.00)
        self.assertEqual(predict_scores.predict("hit_post"), 0.25)
        self.assertEqual(predict_scores.predict("stop_halfway"), 0.50)
        self.assertEqual(predict_scores.predict("settle"), 0.00)
        self.assertEqual(predict_scores.predict("dream"), 0.00)


if __name__ == "__main__":
    unittest.main()

