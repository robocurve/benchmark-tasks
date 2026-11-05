"""Test that predict_scores accurately matches the required scoring ladder for every mode.

- demo: 1.00, success=True
- naive: 0.75, success=False
- overshoot: 0.00, success=False
- no_sample1: 0.75, success=False
- settle: 0.00, success=False
"""

from __future__ import annotations

import os
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
        self.assertFalse(res["details"]["fire_tripped"])
        self.assertTrue(res["details"]["s1_complete"])
        self.assertGreaterEqual(res["details"]["hold_steps"], 20)
        self.assertTrue(60.0 <= res["details"]["s2_core_temp"] <= 70.0)

    def test_naive_predicted_score(self):
        res = predict_scores.simulate_mode("naive")
        self.assertEqual(res["score"], 0.75)
        self.assertFalse(res["success"])
        self.assertFalse(res["details"]["fire_tripped"])

    def test_overshoot_predicted_score(self):
        res = predict_scores.simulate_mode("overshoot")
        self.assertEqual(res["score"], 0.00)
        self.assertFalse(res["success"])
        self.assertTrue(res["details"]["fire_tripped"])

    def test_no_sample1_predicted_score(self):
        res = predict_scores.simulate_mode("no_sample1")
        self.assertEqual(res["score"], 0.75)
        self.assertFalse(res["success"])
        self.assertFalse(res["details"]["fire_tripped"])
        self.assertFalse(res["details"]["s1_complete"])

    def test_settle_predicted_score(self):
        res = predict_scores.simulate_mode("settle")
        self.assertEqual(res["score"], 0.00)
        self.assertFalse(res["success"])
