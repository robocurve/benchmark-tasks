"""Unit tests for evaluation logic and zero-gate handling in clear-lane-then-push-block-to-dock."""

import unittest
import numpy as np

import geometry as g
from logic import TaskLogic


class TestLogic(unittest.TestCase):
    def setUp(self):
        self.logic = TaskLogic(hold_s=g.HOLD_TIME_S, hz=20.0)
        self.neutral_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.post_start = np.array(g.POST_START_POS)
        self.blue_start = np.array(g.BLUE_START_POS)
        self.yellow_start = np.array(g.YELLOW_START_POS)
        self.cargo_start = np.array(g.CARGO_START_POS)

    def test_initial_state(self):
        res = self.logic.update(
            cargo_pos=self.cargo_start,
            cargo_quat=self.neutral_quat,
            blue_pos=self.blue_start,
            yellow_pos=self.yellow_start,
            post_pos=self.post_start,
            post_quat=self.neutral_quat,
        )
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)
        self.assertFalse(res.details["post_hit"])
        self.assertFalse(res.details["both_cleared"])

    def test_stage_1_both_cleared(self):
        # Move both loose blocks outside lane (|y| > 0.075)
        blue_clear = np.array([0.220, +0.160, g.TABLE_Z + 0.020])
        yellow_clear = np.array([0.280, -0.160, g.TABLE_Z + 0.020])

        res = self.logic.update(
            cargo_pos=self.cargo_start,
            cargo_quat=self.neutral_quat,
            blue_pos=blue_clear,
            yellow_pos=yellow_clear,
            post_pos=self.post_start,
            post_quat=self.neutral_quat,
        )
        self.assertEqual(res.score, 0.25)
        self.assertTrue(res.details["both_cleared"])

    def test_stage_2_cargo_moved_80mm(self):
        blue_clear = np.array([0.220, +0.160, g.TABLE_Z + 0.020])
        yellow_clear = np.array([0.280, -0.160, g.TABLE_Z + 0.020])
        cargo_moved = np.array([0.240, 0.000, g.TABLE_Z + 0.020])  # moved 90 mm from 0.150

        res = self.logic.update(
            cargo_pos=cargo_moved,
            cargo_quat=self.neutral_quat,
            blue_pos=blue_clear,
            yellow_pos=yellow_clear,
            post_pos=self.post_start,
            post_quat=self.neutral_quat,
        )
        self.assertEqual(res.score, 0.50)

    def test_stage_3_cargo_at_dock(self):
        blue_clear = np.array([0.220, +0.160, g.TABLE_Z + 0.020])
        yellow_clear = np.array([0.280, -0.160, g.TABLE_Z + 0.020])
        cargo_near_dock = np.array([0.355, 0.000, g.TABLE_Z + 0.020])  # 15 mm from 0.340 mark

        res = self.logic.update(
            cargo_pos=cargo_near_dock,
            cargo_quat=self.neutral_quat,
            blue_pos=blue_clear,
            yellow_pos=yellow_clear,
            post_pos=self.post_start,
            post_quat=self.neutral_quat,
        )
        self.assertEqual(res.score, 0.75)

    def test_stage_4_success_hold(self):
        blue_clear = np.array([0.220, +0.160, g.TABLE_Z + 0.020])
        yellow_clear = np.array([0.280, -0.160, g.TABLE_Z + 0.020])
        cargo_on_mark = np.array([0.340, 0.000, g.TABLE_Z + 0.020])

        tcps_clear = {
            "left": [0.200, +0.280, 0.860],
            "right": [0.200, -0.280, 0.860],
        }

        # Step 39 times: score remains 0.75
        for _ in range(39):
            res = self.logic.update(
                cargo_pos=cargo_on_mark,
                cargo_quat=self.neutral_quat,
                blue_pos=blue_clear,
                yellow_pos=yellow_clear,
                post_pos=self.post_start,
                post_quat=self.neutral_quat,
                arm_tcps=tcps_clear,
            )
            self.assertEqual(res.score, 0.75)
            self.assertFalse(res.success)

        # Step 40: score achieves 1.00
        res = self.logic.update(
            cargo_pos=cargo_on_mark,
            cargo_quat=self.neutral_quat,
            blue_pos=blue_clear,
            yellow_pos=yellow_clear,
            post_pos=self.post_start,
            post_quat=self.neutral_quat,
            arm_tcps=tcps_clear,
        )
        self.assertEqual(res.score, 1.00)
        self.assertTrue(res.success)

    def test_post_hit_zero_gate_cap(self):
        blue_clear = np.array([0.220, +0.160, g.TABLE_Z + 0.020])
        yellow_clear = np.array([0.280, -0.160, g.TABLE_Z + 0.020])
        cargo_on_mark = np.array([0.340, 0.000, g.TABLE_Z + 0.020])

        # Post tilted by 15 degrees about x
        rad = np.radians(15.0)
        tilted_quat = np.array([np.sin(rad / 2), 0.0, 0.0, np.cos(rad / 2)])

        res = self.logic.update(
            cargo_pos=cargo_on_mark,
            cargo_quat=self.neutral_quat,
            blue_pos=blue_clear,
            yellow_pos=yellow_clear,
            post_pos=self.post_start,
            post_quat=tilted_quat,
        )
        self.assertTrue(res.details["post_hit"])
        self.assertLessEqual(res.score, 0.25)


if __name__ == "__main__":
    unittest.main()

