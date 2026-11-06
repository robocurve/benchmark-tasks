"""Unit tests for score predictions in spring-drawer-hold-and-retrieve."""
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
        self.assertAlmostEqual(predictions["no_hold"], 0.25)
        self.assertAlmostEqual(predictions["pinch"], 0.00)
        self.assertAlmostEqual(predictions["leave_open"], 0.75)
        self.assertAlmostEqual(predictions["settle"], 0.00)


if __name__ == "__main__":
    unittest.main()

