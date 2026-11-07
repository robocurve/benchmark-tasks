"""Tests for scoring ladder and zero-gates in stack-crates-largest-first."""

import unittest
import numpy as np

import geometry as g
from scoring import Scorer


class TestScoring(unittest.TestCase):
    def setUp(self):
        self.scorer = Scorer(hold_s=2.0, hz=20.0)

    def test_ladder_progression(self):
        large_on_pallet = (g.PALLET_POS[0], g.PALLET_POS[1], g.LEVEL_1_TARGET_Z)
        med_on_large = (g.PALLET_POS[0], g.PALLET_POS[1], g.LEVEL_2_TARGET_Z)
        small_on_med = (g.PALLET_POS[0], g.PALLET_POS[1], g.LEVEL_3_TARGET_Z)

        arm_tcps = {
            "left": (0.200, 0.280, 0.900),
            "right": (0.200, -0.280, 0.900),
        }

        # Step 0: start
        res = self.scorer.update(g.LARGE_START_POS, medium_pos=g.MEDIUM_START_POS, small_pos=g.SMALL_START_POS)
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)

        # Step 1: Large on pallet -> 0.25
        res = self.scorer.update(large_on_pallet, medium_pos=g.MEDIUM_START_POS, small_pos=g.SMALL_START_POS)
        self.assertEqual(res.score, 0.25)
        self.assertFalse(res.success)

        # Step 2: Medium on large -> 0.50
        res = self.scorer.update(large_on_pallet, medium_pos=med_on_large, small_pos=g.SMALL_START_POS)
        self.assertEqual(res.score, 0.50)
        self.assertFalse(res.success)

        # Step 3: Small on medium -> 0.75
        res = self.scorer.update(large_on_pallet, medium_pos=med_on_large, small_pos=small_on_med)
        self.assertEqual(res.score, 0.75)
        self.assertFalse(res.success)

        # Step 4: Hold stable for 40 steps (2.0 s at 20 Hz)
        for _ in range(40):
            res = self.scorer.update(large_on_pallet, medium_pos=med_on_large, small_pos=small_on_med, arm_tcps=arm_tcps)

        self.assertEqual(res.score, 1.00)
        self.assertTrue(res.success)

    def test_floor_fault_zeroes_score(self):
        large_on_pallet = (g.PALLET_POS[0], g.PALLET_POS[1], g.LEVEL_1_TARGET_Z)
        res = self.scorer.update(large_on_pallet, medium_pos=g.MEDIUM_START_POS, small_pos=g.SMALL_START_POS)
        self.assertEqual(res.score, 0.25)

        # A crate falls off the table -> floor fault!
        small_fallen = (g.PALLET_POS[0], g.PALLET_POS[1], g.TOP - 0.100)
        res = self.scorer.update(large_on_pallet, medium_pos=g.MEDIUM_START_POS, small_pos=small_fallen)
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)


if __name__ == "__main__":
    unittest.main()

