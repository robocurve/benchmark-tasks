"""Test that predict_scores accurately matches the required scoring ladder for every mode.

- demo: 1.00, success=True
- no_commit: 0.75, success=False
- no_buffer: 0.50, success=False
- settle: 0.00, success=False
- dream: 0.00, success=False
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import predict_scores


class TestPredictions(unittest.TestCase):
    def test_demo_predicted_score(self):
        res = predict_scores.simulate_mode("demo")
        self.assertEqual(res["score"], 1.00)
        self.assertTrue(res["success"])
        self.assertFalse(res["details"].get("failed", False))
        self.assertTrue(res["details"]["commit_valid"])
        self.assertEqual(res["details"]["lamp_state"], "green")
        self.assertGreaterEqual(res["details"]["hold_steps"], 20)

    def test_no_commit_predicted_score(self):
        res = predict_scores.simulate_mode("no_commit")
        self.assertEqual(res["score"], 0.75)
        self.assertFalse(res["success"])
        self.assertFalse(res["details"]["commit_valid"])
        self.assertEqual(res["details"]["lamp_state"], "green")

    def test_no_buffer_predicted_score(self):
        res = predict_scores.simulate_mode("no_buffer")
        self.assertEqual(res["score"], 0.50)
        self.assertFalse(res["success"])
        self.assertEqual(res["details"]["lamp_state"], "red")

    def test_settle_predicted_score(self):
        res = predict_scores.simulate_mode("settle")
        self.assertEqual(res["score"], 0.00)
        self.assertFalse(res["success"])
        self.assertFalse(res["details"]["has_pulled"])

    def test_dream_predicted_score(self):
        res = predict_scores.simulate_mode("dream")
        self.assertEqual(res["score"], 0.00)
        self.assertFalse(res["success"])


if __name__ == "__main__":
    unittest.main()
