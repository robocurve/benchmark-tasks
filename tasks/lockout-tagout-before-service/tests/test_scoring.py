"""Tests for scoring ladder and zero-gates in lockout-tagout-before-service."""

import unittest
import numpy as np

import geometry as g
from scoring import Scorer


class TestScoring(unittest.TestCase):
    def setUp(self):
        self.scorer = Scorer(hold_s=1.0, hz=20.0)

    def test_ladder_progression(self):
        pin_hole = (g.HOLE_POS[0], g.HOLE_POS[1], g.PIN_SEATED_Z)

        # 0.0 at start
        res = self.scorer.update(0.0, g.PIN_STAND_POS, 0.0, 0.0, 0.0)
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)

        # 0.25: Breaker OFF
        res = self.scorer.update(0.035, g.PIN_STAND_POS, 0.0, 0.0, 0.0)
        self.assertEqual(res.score, 0.25)
        self.assertFalse(res.success)

        # 0.50: Pin seated
        res = self.scorer.update(0.035, pin_hole, 0.0, 0.0, 0.0)
        self.assertEqual(res.score, 0.50)
        self.assertFalse(res.success)

        # 0.75: TEST pressed dead
        res = self.scorer.update(0.035, pin_hole, -0.007, 0.0, 0.0)
        self.assertEqual(res.score, 0.75)
        self.assertFalse(res.success)

        # Cover opened >= 70 mm and SERVICE button pressed
        # Requires 20 steps (1.0 s) for service pressed, then 20 steps hold
        for _ in range(40):
            res = self.scorer.update(0.035, pin_hole, 0.0, 0.075, -0.007)

        self.assertEqual(res.score, 1.00)
        self.assertTrue(res.success)

    def test_fault_stops_success(self):
        # Breaker OFF -> 0.25
        res = self.scorer.update(0.035, g.PIN_STAND_POS, 0.0, 0.0, 0.0)
        self.assertEqual(res.score, 0.25)

        # Skip pin, open cover -> FAULT!
        res = self.scorer.update(0.035, g.PIN_STAND_POS, 0.0, 0.025, 0.0)
        self.assertEqual(res.score, 0.25)  # cannot advance
        self.assertFalse(res.success)

        # Even if service button pressed later, score cannot reach 1.0
        for _ in range(40):
            res = self.scorer.update(0.035, (g.HOLE_POS[0], g.HOLE_POS[1], g.PIN_SEATED_Z), 0.0, 0.075, -0.007)
        self.assertEqual(res.score, 0.25)
        self.assertFalse(res.success)

    def test_cover_first_fault(self):
        # Immediately open cover -> FAULT -> 0.0
        res = self.scorer.update(0.0, g.PIN_STAND_POS, 0.0, 0.025, 0.0)
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)


if __name__ == "__main__":
    unittest.main()

