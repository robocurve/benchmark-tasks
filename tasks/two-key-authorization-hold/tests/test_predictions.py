"""Unit tests verifying that simulated ablation predictions match expected scores."""

from __future__ import annotations

import sys
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import predict_scores


class TestTwoKeyPredictions(unittest.TestCase):
    def test_predicted_scores_match_expectations(self):
        scores = predict_scores.predict_all()
        self.assertEqual(scores["demo"], 1.00)
        self.assertEqual(scores["sequential"], 0.25)
        self.assertEqual(scores["one_key"], 0.25)
        self.assertEqual(scores["release_early"], 0.25)
        self.assertEqual(scores["no_confirm"], 0.25)
        self.assertEqual(scores["settle"], 0.00)


if __name__ == "__main__":
    unittest.main()
