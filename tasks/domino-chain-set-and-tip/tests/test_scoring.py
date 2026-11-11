"""Tests for scoring ladder and zero-gates in domino-chain-set-and-tip."""

import math
import unittest
import numpy as np

import geometry as g
from scoring import Scorer


def pitch_quat(deg: float) -> tuple[float, float, float, float]:
    rad = math.radians(deg)
    return (0.0, math.sin(rad / 2.0), 0.0, math.cos(rad / 2.0))


UPRIGHT_QUAT = (0.0, 0.0, 0.0, 1.0)
FALLEN_PLUS_X_QUAT = pitch_quat(70.0)


class TestScoring(unittest.TestCase):
    def setUp(self):
        self.scorer = Scorer(rest_s=0.3, max_tip_time_s=5.0, hz=20.0)
        self.arm_tcps_clear = {
            "left": (0.200, +0.280, g.HOVER_Z),
            "right": (0.200, -0.280, g.HOVER_Z),
        }

    def test_ladder_progression(self):
        # Step 0: start
        res = self.scorer.update(
            list(g.DOMINO_START_POSITIONS),
            [UPRIGHT_QUAT] * 4,
            [(0.0, 0.0, 0.0)] * 4,
            self.arm_tcps_clear,
        )
        self.assertEqual(res.score, 0.00)
        self.assertFalse(res.success)

        # Step 1: Two dominoes standing on marks 0 & 1 -> 0.25 (need 7 steps at 20 Hz to exceed 0.3 s resting)
        pos_0 = (g.MARK_0_POS[0], g.MARK_0_POS[1], g.STANDING_Z)
        pos_1 = (g.MARK_1_POS[0], g.MARK_1_POS[1], g.STANDING_Z)
        poses_2 = [pos_0, pos_1, g.DOMINO_2_START_POS, g.DOMINO_3_START_POS]
        for _ in range(7):
            res = self.scorer.update(poses_2, [UPRIGHT_QUAT] * 4, [(0.0, 0.0, 0.0)] * 4, self.arm_tcps_clear)

        self.assertEqual(res.score, 0.25)
        self.assertFalse(res.success)

        # Step 2: All four dominoes standing on marks -> 0.50
        pos_2 = (g.MARK_2_POS[0], g.MARK_2_POS[1], g.STANDING_Z)
        pos_3 = (g.MARK_3_POS[0], g.MARK_3_POS[1], g.STANDING_Z)
        poses_4 = [pos_0, pos_1, pos_2, pos_3]
        for _ in range(7):
            res = self.scorer.update(poses_4, [UPRIGHT_QUAT] * 4, [(0.0, 0.0, 0.0)] * 4, self.arm_tcps_clear)

        self.assertEqual(res.score, 0.50)
        self.assertFalse(res.success)

        # Step 3: First domino tipped (tilt = 25 deg > 20 deg) while others standing -> 0.75
        quats_tipped = [pitch_quat(25.0), UPRIGHT_QUAT, UPRIGHT_QUAT, UPRIGHT_QUAT]
        res = self.scorer.update(poses_4, quats_tipped, [(0.0, 0.0, 0.0)] * 4, self.arm_tcps_clear)
        self.assertEqual(res.score, 0.75)
        self.assertFalse(res.success)

        # Step 4: All four fallen toward +x within 5 s, arms clear -> 1.00
        quats_all_fallen = [FALLEN_PLUS_X_QUAT] * 4
        res = self.scorer.update(poses_4, quats_all_fallen, [(0.0, 0.0, 0.0)] * 4, self.arm_tcps_clear)
        self.assertEqual(res.score, 1.00)
        self.assertTrue(res.success)

    def test_zero_gate_premature_fall(self):
        # Two standing -> 0.25
        pos_0 = (g.MARK_0_POS[0], g.MARK_0_POS[1], g.STANDING_Z)
        pos_1 = (g.MARK_1_POS[0], g.MARK_1_POS[1], g.STANDING_Z)
        poses_2 = [pos_0, pos_1, g.DOMINO_2_START_POS, g.DOMINO_3_START_POS]
        for _ in range(7):
            res = self.scorer.update(poses_2, [UPRIGHT_QUAT] * 4, [(0.0, 0.0, 0.0)] * 4, self.arm_tcps_clear)
        self.assertEqual(res.score, 0.25)

        # Domino 0 falls prematurely (tilt = 55 deg > 45 deg)
        quats_fallen = [pitch_quat(55.0), UPRIGHT_QUAT, UPRIGHT_QUAT, UPRIGHT_QUAT]
        res = self.scorer.update(poses_2, quats_fallen, [(0.0, 0.0, 0.0)] * 4, self.arm_tcps_clear)
        self.assertEqual(res.score, 0.25)
        self.assertTrue(res.details["dbg_premature"])

        # Even if all four now fall forward, score remains capped at 0.25 max
        pos_2 = (g.MARK_2_POS[0], g.MARK_2_POS[1], g.STANDING_Z)
        pos_3 = (g.MARK_3_POS[0], g.MARK_3_POS[1], g.STANDING_Z)
        poses_4 = [pos_0, pos_1, pos_2, pos_3]
        quats_all_fallen = [FALLEN_PLUS_X_QUAT] * 4
        res = self.scorer.update(poses_4, quats_all_fallen, [(0.0, 0.0, 0.0)] * 4, self.arm_tcps_clear)
        self.assertEqual(res.score, 0.25)
        self.assertFalse(res.success)

    def test_floor_fault(self):
        # A domino dropped below table top
        dropped_pos = [(0.260, 0.000, g.TOP - 0.100)] + list(g.DOMINO_START_POSITIONS[1:])
        res = self.scorer.update(dropped_pos, [UPRIGHT_QUAT] * 4, [(0.0, 0.0, 0.0)] * 4, self.arm_tcps_clear)
        self.assertEqual(res.score, 0.00)
        self.assertTrue(res.details["dbg_floor_fault"])


if __name__ == "__main__":
    unittest.main()
