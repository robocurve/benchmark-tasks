"""Unit tests for sliding-tile-sort-three-tiles task logic and zero-gates."""

import unittest
import numpy as np

import geometry as g
import logic


class TestTaskLogic(unittest.TestCase):
    def setUp(self):
        self.tl = logic.TaskLogic(hold_s=2.0, hz=20.0)
        self.pos_b = np.array(g.TILE_BLUE_START)
        self.pos_o = np.array(g.TILE_ORANGE_START)
        self.pos_y = np.array(g.TILE_YELLOW_START)
        self.quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.vel = np.zeros(3)
        self.arm_tcps = {
            "left": [0.200, +0.280, 0.850],
            "right": [0.200, -0.280, 0.850],
        }

    def test_initial_state_zero_score(self):
        # Initial: Y in 0, O in 2, B in 3. Goal is B in 0, O in 1, Y in 2.
        # None of the 3 tiles are in their goal cell!
        res = self.tl.update(
            self.pos_b, self.quat, self.vel,
            self.pos_o, self.quat, self.vel,
            self.pos_y, self.quat, self.vel,
            self.arm_tcps,
        )
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)

    def test_escaped_zero_gate(self):
        # Lift Yellow tile > 12 mm above table
        high_pos_y = np.array([g.CELL_0[0], g.CELL_0[1], g.TILE_MAX_Z + 0.005])
        res = self.tl.update(
            self.pos_b, self.quat, self.vel,
            self.pos_o, self.quat, self.vel,
            high_pos_y, self.quat, self.vel,
            self.arm_tcps,
        )
        self.assertTrue(self.tl.ever_escaped)
        self.assertEqual(res.score, 0.0)

    def test_milestone_progression_and_hold(self):
        # 1. Place Yellow in cell 2 (1 in goal -> 0.25)
        pos_y_goal = np.array([g.CELL_2[0], g.CELL_2[1], g.TILE_REST_Z])
        res1 = self.tl.update(
            self.pos_b, self.quat, self.vel,
            self.pos_o, self.quat, self.vel,
            pos_y_goal, self.quat, self.vel,
            self.arm_tcps,
        )
        self.assertEqual(res1.score, 0.25)

        # 2. Place Blue in cell 0 (2 in goal -> 0.50)
        pos_b_goal = np.array([g.CELL_0[0], g.CELL_0[1], g.TILE_REST_Z])
        res2 = self.tl.update(
            pos_b_goal, self.quat, self.vel,
            self.pos_o, self.quat, self.vel,
            pos_y_goal, self.quat, self.vel,
            self.arm_tcps,
        )
        self.assertEqual(res2.score, 0.50)

        # 3. Place Orange in cell 1 (all 3 in goal -> 0.75)
        pos_o_goal = np.array([g.CELL_1[0], g.CELL_1[1], g.TILE_REST_Z])
        res3 = self.tl.update(
            pos_b_goal, self.quat, self.vel,
            pos_o_goal, self.quat, self.vel,
            pos_y_goal, self.quat, self.vel,
            self.arm_tcps,
        )
        self.assertEqual(res3.score, 0.75)
        self.assertFalse(res3.success)

        # 4. Hold for 38 more steps (total 39 steps < 40 req) -> still 0.75
        for _ in range(38):
            res = self.tl.update(
                pos_b_goal, self.quat, self.vel,
                pos_o_goal, self.quat, self.vel,
                pos_y_goal, self.quat, self.vel,
                self.arm_tcps,
            )
            self.assertEqual(res.score, 0.75)
            self.assertFalse(res.success)

        # 40th step completes 2.0 s hold -> 1.00
        res_final = self.tl.update(
            pos_b_goal, self.quat, self.vel,
            pos_o_goal, self.quat, self.vel,
            pos_y_goal, self.quat, self.vel,
            self.arm_tcps,
        )
        self.assertEqual(res_final.score, 1.00)
        self.assertTrue(res_final.success)


if __name__ == "__main__":
    unittest.main()

