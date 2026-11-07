"""Unit tests for score predictions in estop-before-clearing-jam."""

import sys
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import predict_scores as P


class TestPredictions(unittest.TestCase):
    def test_predictions_match_expected(self):
        predictions = P.predict_all()
        self.assertAlmostEqual(predictions["demo"], 1.00)
        self.assertAlmostEqual(predictions["no_estop"], 0.00)
        self.assertAlmostEqual(predictions["start_early"], 0.50)
        self.assertAlmostEqual(predictions["skip_reset"], 0.50)
        self.assertAlmostEqual(predictions["leave_stopped"], 0.50)
        self.assertAlmostEqual(predictions["settle"], 0.00)
        self.assertAlmostEqual(predictions["dream"], 0.00)


if __name__ == "__main__":
    unittest.main()
