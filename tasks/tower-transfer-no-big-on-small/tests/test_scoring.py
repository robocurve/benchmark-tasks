"""Tests for scoring ladder and zero-gates in tower-transfer-no-big-on-small."""

import unittest
import numpy as np

import geometry as g
from scoring import Scorer


class TestScoring(unittest.TestCase):
    def setUp(self):
        self.scorer = Scorer(hold_s=2.0, hz=20.0)

    def test_ladder_progression(self):
        arm_tcps = {
            "left": (0.200, 0.280, 0.900),
            "right": (0.200, -0.280, 0.900),
        }

        # Step 0: start
        res = self.scorer.update(
            g.LARGE_START_POS,
            medium_pos=g.MEDIUM_START_POS,
            small_pos=g.SMALL_START_POS,
            arm_tcps=arm_tcps,
        )
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)

        # Step 1: Small on RIGHT mark -> 0.25 (need 7 steps to settle resting)
        s_on_right = (g.RIGHT_MARK_POS[0], g.RIGHT_MARK_POS[1], g.LEVEL_1_TARGET_Z)
        for _ in range(7):
            res = self.scorer.update(
                g.LARGE_START_POS,
                medium_pos=g.MEDIUM_START_POS,
                small_pos=s_on_right,
                arm_tcps=arm_tcps,
            )
        self.assertEqual(res.score, 0.25)
        self.assertFalse(res.success)

        # Step 2: Large at MID mark on table -> 0.50
        l_on_mid = (g.MID_MARK_POS[0], g.MID_MARK_POS[1], g.LEVEL_1_TARGET_Z)
        for _ in range(7):
            res = self.scorer.update(
                l_on_mid,
                medium_pos=g.MEDIUM_START_POS,
                small_pos=s_on_right,
                arm_tcps=arm_tcps,
            )
        self.assertEqual(res.score, 0.50)
        self.assertFalse(res.success)

        # Step 3: Large on RIGHT mark, Medium on Large at RIGHT -> 0.75
        l_on_right = (g.RIGHT_MARK_POS[0], g.RIGHT_MARK_POS[1], g.LEVEL_1_TARGET_Z)
        m_on_large_right = (g.RIGHT_MARK_POS[0], g.RIGHT_MARK_POS[1], g.LEVEL_2_TARGET_Z)
        s_on_left = (g.LEFT_MARK_POS[0], g.LEFT_MARK_POS[1], g.LEVEL_1_TARGET_Z)
        for _ in range(7):
            res = self.scorer.update(
                l_on_right,
                medium_pos=m_on_large_right,
                small_pos=s_on_left,
                arm_tcps=arm_tcps,
            )
        self.assertEqual(res.score, 0.75)
        self.assertFalse(res.success)

        # Step 4: Small on Medium on Large at RIGHT mark (complete tower)
        s_on_med_right = (g.RIGHT_MARK_POS[0], g.RIGHT_MARK_POS[1], g.LEVEL_3_TARGET_Z)
        for _ in range(40):
            res = self.scorer.update(
                l_on_right,
                medium_pos=m_on_large_right,
                small_pos=s_on_med_right,
                arm_tcps=arm_tcps,
            )
        self.assertEqual(res.score, 1.00)
        self.assertTrue(res.success)

    def test_zero_gate_rule_broken(self):
        arm_tcps = {"left": (0.200, 0.280, 0.900), "right": (0.200, -0.280, 0.900)}
        s_on_right = (g.RIGHT_MARK_POS[0], g.RIGHT_MARK_POS[1], g.LEVEL_1_TARGET_Z)
        for _ in range(7):
            res = self.scorer.update(
                g.LARGE_START_POS,
                medium_pos=g.MEDIUM_START_POS,
                small_pos=s_on_right,
                arm_tcps=arm_tcps,
            )
        self.assertEqual(res.score, 0.25)

        # Large placed directly onto Small (illegal!)
        l_on_small = (g.RIGHT_MARK_POS[0], g.RIGHT_MARK_POS[1], g.LEVEL_2_TARGET_Z)
        for _ in range(7):
            res = self.scorer.update(
                l_on_small,
                medium_pos=g.MEDIUM_START_POS,
                small_pos=s_on_right,
                arm_tcps=arm_tcps,
            )
        self.assertEqual(res.score, 0.00)
        self.assertFalse(res.success)


if __name__ == "__main__":
    unittest.main()

