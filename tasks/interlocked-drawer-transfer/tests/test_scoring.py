"""Unit tests for Scorer in interlocked-drawer-transfer."""

import unittest
import numpy as np

import geometry as g
from scoring import Scorer


class TestScoring(unittest.TestCase):
    def setUp(self):
        self.scorer = Scorer(hold_s=1.0, hz=20.0)

    def test_initial_score_zero(self):
        res = self.scorer.update(
            disp_left=0.0,
            disp_right=0.0,
            block_pos=g.BLOCK_START_POS,
            grippers_open=True,
        )
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)

    def test_milestone_0_25(self):
        res = self.scorer.update(
            disp_left=0.080,
            disp_right=0.0,
            block_pos=(0.350, g.DRAWER_LEFT_Y, g.DRAWER_RIM_Z + 0.030),
            grippers_open=False,
        )
        self.assertEqual(res.score, 0.25)
        self.assertFalse(res.success)

    def test_milestone_0_50_carry_solution(self):
        # 0.25
        self.scorer.update(0.080, 0.0, (0.350, g.DRAWER_LEFT_Y, g.DRAWER_RIM_Z + 0.030))
        # Close left while holding block in air
        res = self.scorer.update(0.002, 0.0, (0.280, 0.050, g.HOVER_Z))
        self.assertEqual(res.score, 0.50)
        self.assertFalse(res.success)

    def test_milestone_0_50_park_solution(self):
        # 0.25
        self.scorer.update(0.080, 0.0, (0.350, g.DRAWER_LEFT_Y, g.DRAWER_RIM_Z + 0.030))
        # Close left while block is on staging pad
        res = self.scorer.update(0.002, 0.0, g.STAGING_SPOT)
        self.assertEqual(res.score, 0.50)
        self.assertFalse(res.success)

    def test_milestone_0_75(self):
        self.scorer.update(0.080, 0.0, (0.350, g.DRAWER_LEFT_Y, g.DRAWER_RIM_Z + 0.030))
        self.scorer.update(0.002, 0.0, g.STAGING_SPOT)
        # Open right drawer >= 70 mm
        res = self.scorer.update(0.002, 0.085, g.STAGING_SPOT)
        self.assertEqual(res.score, 0.75)
        self.assertFalse(res.success)

    def test_milestone_1_00_success_hold(self):
        # Step through milestones
        self.scorer.update(0.080, 0.0, (0.350, g.DRAWER_LEFT_Y, g.DRAWER_RIM_Z + 0.030))
        self.scorer.update(0.002, 0.0, (0.280, 0.050, g.HOVER_Z))
        self.scorer.update(0.002, 0.085, (0.280, 0.050, g.HOVER_Z))

        block_in_right = (0.350, g.DRAWER_RIGHT_Y, g.BLOCK_REST_Z)
        arm_tcps = {"left": (0.200, 0.280, g.HOVER_Z), "right": (0.200, -0.280, g.HOVER_Z)}
        arm_vels = {"left": (np.zeros(3), np.zeros(3)), "right": (np.zeros(3), np.zeros(3))}

        # Hold for 19 steps -> not yet 1.00
        for _ in range(19):
            res = self.scorer.update(
                disp_left=0.002,
                disp_right=0.002,
                block_pos=block_in_right,
                grippers_open=True,
                arm_tcps=arm_tcps,
                arm_velocities=arm_vels,
            )
            self.assertEqual(res.score, 0.75)
            self.assertFalse(res.success)

        # 20th step -> 1.00 and success!
        res = self.scorer.update(
            disp_left=0.002,
            disp_right=0.002,
            block_pos=block_in_right,
            grippers_open=True,
            arm_tcps=arm_tcps,
            arm_velocities=arm_vels,
        )
        self.assertEqual(res.score, 1.00)
        self.assertTrue(res.success)

    def test_tip_fault_caps_score_at_0_25(self):
        # 1. Achieve 0.25
        self.scorer.update(0.080, 0.0, (0.350, g.DRAWER_LEFT_Y, g.DRAWER_RIM_Z + 0.030))
        # 2. Open right drawer while left drawer remains open for >= 0.2 s (4 steps)
        for _ in range(4):
            res = self.scorer.update(0.080, 0.050, (0.280, 0.050, g.HOVER_Z))
        self.assertEqual(res.score, 0.25)
        self.assertFalse(res.success)
        self.assertTrue(res.details.get("tip_fault"))

        # Subsequent attempts to complete task do not exceed 0.25
        block_in_right = (0.350, g.DRAWER_RIGHT_Y, g.BLOCK_REST_Z)
        res = self.scorer.update(0.0, 0.0, block_in_right, grippers_open=True)
        self.assertEqual(res.score, 0.25)
        self.assertFalse(res.success)

    def test_leave_open_scores_0_75(self):
        self.scorer.update(0.080, 0.0, (0.350, g.DRAWER_LEFT_Y, g.DRAWER_RIM_Z + 0.030))
        self.scorer.update(0.002, 0.0, (0.280, 0.050, g.HOVER_Z))
        self.scorer.update(0.002, 0.085, (0.280, 0.050, g.HOVER_Z))

        # Block placed in right drawer but right drawer kept open 85 mm
        block_in_right = (0.350 - 0.085, g.DRAWER_RIGHT_Y, g.BLOCK_REST_Z)
        for _ in range(30):
            res = self.scorer.update(0.002, 0.085, block_in_right, grippers_open=True)
        self.assertEqual(res.score, 0.75)
        self.assertFalse(res.success)

    def test_wrong_drawer_scores_0_50(self):
        self.scorer.update(0.080, 0.0, (0.350, g.DRAWER_LEFT_Y, g.DRAWER_RIM_Z + 0.030))
        self.scorer.update(0.002, 0.0, (0.280, 0.050, g.HOVER_Z))
        # Returned to left drawer, right drawer never opened
        for _ in range(30):
            res = self.scorer.update(0.002, 0.000, g.BLOCK_START_POS, grippers_open=True)
        self.assertEqual(res.score, 0.50)
        self.assertFalse(res.success)

    def test_block_dropped_zeros_score(self):
        self.scorer.update(0.080, 0.0, (0.350, g.DRAWER_LEFT_Y, g.DRAWER_RIM_Z + 0.030))
        # Drop block off table
        res = self.scorer.update(0.002, 0.0, (0.200, 0.0, g.TOP - 0.100))
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)


if __name__ == "__main__":
    unittest.main()
