"""Tests for scoring ladder and ablation predictions in compress-and-lock-spring-cartridge."""

import unittest
import numpy as np

import geometry as g
from scoring import Scorer


class TestScoring(unittest.TestCase):
    def setUp(self):
        self.scorer = Scorer(hold_s=1.0, hz=20.0)

    def test_ladder_progression(self):
        arm_clear_tcps = {
            "left": (0.200, +0.305, g.HOVER_Z),
            "right": (0.200, -0.305, g.HOVER_Z),
        }

        # Step 0: Idle start -> score 0.00
        res = self.scorer.update(plunger_disp=0.0, bar_disp=0.0, forced_plunger_released=True)
        self.assertEqual(res.score, 0.00)
        self.assertFalse(res.success)

        # Step 1: Plunger compressed >= 38 mm for 1.0 s (20 steps) -> score 0.25
        for _ in range(20):
            res = self.scorer.update(plunger_disp=0.045, bar_disp=0.0, forced_plunger_released=False)
        self.assertEqual(res.score, 0.25)
        self.assertFalse(res.success)

        # Step 2: Lock bar across >= 55 mm while compressed -> score 0.50
        res = self.scorer.update(plunger_disp=0.045, bar_disp=0.060, forced_plunger_released=False)
        self.assertEqual(res.score, 0.50)
        self.assertFalse(res.success)

        # Step 3: Plunger released and stays compressed >= 38 mm for 2.0 s (40 steps) -> score 0.75
        for _ in range(40):
            res = self.scorer.update(plunger_disp=0.0, bar_disp=0.060, forced_plunger_released=True)
        self.assertEqual(res.score, 0.75)
        self.assertFalse(res.success)

        # Step 4: Both arms clear and lamp green held 1.0 s (20 steps) -> score 1.00
        for _ in range(20):
            res = self.scorer.update(
                plunger_disp=0.0,
                bar_disp=0.060,
                grippers_open=True,
                arm_tcps=arm_clear_tcps,
                forced_plunger_released=True,
            )
        self.assertEqual(res.score, 1.00)
        self.assertTrue(res.success)

    def test_ablation_one_arm(self):
        """one_arm ablation: compress plunger, cannot reach lock bar -> 0.25."""
        scorer = Scorer()
        for _ in range(20):
            res = scorer.update(plunger_disp=0.045, bar_disp=0.0, forced_plunger_released=False)
        self.assertEqual(res.score, 0.25)
        # Continue holding, bar never touched
        for _ in range(50):
            res = scorer.update(plunger_disp=0.045, bar_disp=0.0, forced_plunger_released=False)
        self.assertEqual(res.score, 0.25)

    def test_ablation_release_early(self):
        """release_early ablation: let go before bar is across -> 0.25."""
        scorer = Scorer()
        for _ in range(20):
            res = scorer.update(plunger_disp=0.045, bar_disp=0.0, forced_plunger_released=False)
        self.assertEqual(res.score, 0.25)
        # Release early: plunger returns to 0
        for _ in range(40):
            res = scorer.update(plunger_disp=0.0, bar_disp=0.0, forced_plunger_released=True)
        self.assertEqual(res.score, 0.25)

    def test_ablation_bar_first(self):
        """bar_first ablation: push bar before compressing -> bar stops at 15 mm -> 0.00."""
        scorer = Scorer()
        for _ in range(30):
            res = scorer.update(plunger_disp=0.0, bar_disp=0.050, forced_plunger_released=True)
        self.assertEqual(res.score, 0.00)
        self.assertEqual(res.details["bar_disp"], 0.015)
        self.assertTrue(res.details["bar_blocked"])

    def test_ablation_no_release(self):
        """no_release ablation: keep holding plunger until end -> 0.50."""
        scorer = Scorer()
        # Compress
        for _ in range(20):
            scorer.update(plunger_disp=0.045, bar_disp=0.0, forced_plunger_released=False)
        # Lock bar
        for _ in range(10):
            res = scorer.update(plunger_disp=0.045, bar_disp=0.060, forced_plunger_released=False)
        self.assertEqual(res.score, 0.50)
        # Continue holding without releasing
        for _ in range(60):
            res = scorer.update(plunger_disp=0.045, bar_disp=0.060, forced_plunger_released=False)
        self.assertEqual(res.score, 0.50)


if __name__ == "__main__":
    unittest.main()
