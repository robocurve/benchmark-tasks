"""Unit tests for scoring.py in segregate-incompatible-cleaners-in-cabinet."""

from __future__ import annotations

import unittest
import geometry as g
from scoring import Scorer


class TestScoring(unittest.TestCase):
    def test_evaluate_success(self):
        scorer = Scorer(variant="A", hold_s=2.0)
        res = None
        for _ in range(45):
            res = scorer.update(
                bleach_pos=g.BAY_LEFT_POS,
                bleach_quat=(0.0, 0.0, 0.0, 1.0),
                bleach_vel=(0.0, 0.0, 0.0),
                ammonia_pos=g.BAY_RIGHT_POS,
                ammonia_quat=(0.0, 0.0, 0.0, 1.0),
                ammonia_vel=(0.0, 0.0, 0.0),
            )
        self.assertIsNotNone(res)
        self.assertEqual(res.score, 1.00)
        self.assertTrue(res.success)
        self.assertIn("safely segregated", res.details["dbg_reason"])

    def test_evaluate_violation(self):
        scorer = Scorer(variant="A")
        # Place both bottles close together in Left Bay
        res = scorer.update(
            bleach_pos=g.BAY_LEFT_POS,
            bleach_quat=(0.0, 0.0, 0.0, 1.0),
            bleach_vel=(0.0, 0.0, 0.0),
            ammonia_pos=(g.CABINET_X, g.BAY_LEFT_Y + 0.010, g.BOTTLE_CENTER_Z),
            ammonia_quat=(0.0, 0.0, 0.0, 1.0),
            ammonia_vel=(0.0, 0.0, 0.0),
        )
        self.assertEqual(res.score, 0.25)
        self.assertFalse(res.success)
        self.assertTrue(res.details["dbg_incompatible_mix_violation"])
        self.assertIn("chloramine", res.details["dbg_reason"].lower())

    def test_evaluate_partial_hold(self):
        scorer = Scorer(variant="A", hold_s=2.0)
        # Only 5 steps (0.25s < 2.0s hold)
        res = None
        for _ in range(5):
            res = scorer.update(
                bleach_pos=g.BAY_LEFT_POS,
                bleach_quat=(0.0, 0.0, 0.0, 1.0),
                bleach_vel=(0.0, 0.0, 0.0),
                ammonia_pos=g.BAY_RIGHT_POS,
                ammonia_quat=(0.0, 0.0, 0.0, 1.0),
                ammonia_vel=(0.0, 0.0, 0.0),
            )
        self.assertIsNotNone(res)
        self.assertEqual(res.score, 0.75)
        self.assertFalse(res.success)
        self.assertIn("hold pending", res.details["dbg_reason"])


if __name__ == "__main__":
    unittest.main()
