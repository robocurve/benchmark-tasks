"""Unit tests for task logic, milestone progression, and knocked zero-gate in squeeze-in-the-last-slab."""

import unittest
import numpy as np

import geometry as g
import logic


class TestTaskLogic(unittest.TestCase):
    def setUp(self):
        self.tl = logic.TaskLogic(hold_s=2.0, hz=20.0)
        self.p1 = np.array(g.SLAB_1_POS)
        self.p2 = np.array(g.SLAB_2_POS)
        self.p3 = np.array(g.SLAB_3_POS)
        self.p4 = np.array(g.SLAB_4_POS)
        self.p5 = np.array(g.SLAB_5_START)
        self.q = np.array([0.0, 0.0, 0.0, 1.0])
        self.arm_tcps = {
            "left": [0.200, +0.280, 0.880],
            "right": [0.200, -0.280, 0.880],
        }

    def test_initial_state_zero_score(self):
        res = self.tl.update(
            self.p1, self.q,
            self.p2, self.q,
            self.p3, self.q,
            self.p4, self.q,
            self.p5, self.q,
            self.arm_tcps,
        )
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)

    def test_knocked_zero_gate_cap(self):
        # Tilt neighbour slab 3 by 15 degrees (> 10 deg limit)
        # 15 deg around Y: qy = sin(7.5 deg) = 0.1305, qw = cos(7.5 deg) = 0.9908
        q_tilt15 = np.array([0.0, 0.1305, 0.0, 0.9908])
        # Place slab 5 in gap perfectly
        p5_in_gap = np.array(g.SLOT_TARGET_POS)
        res = self.tl.update(
            self.p1, self.q,
            self.p2, self.q,
            self.p3, q_tilt15,
            self.p4, self.q,
            p5_in_gap, self.q,
            self.arm_tcps,
        )
        self.assertTrue(self.tl.ever_knocked)
        self.assertLessEqual(res.score, 0.50)

    def test_milestone_progression_and_hold(self):
        # 1. Standing anywhere on strip (0.25)
        p5_on_strip = np.array([0.250, 0.000, g.SLAB_STANDING_Z])
        res1 = self.tl.update(
            self.p1, self.q,
            self.p2, self.q,
            self.p3, self.q,
            self.p4, self.q,
            p5_on_strip, self.q,
            self.arm_tcps,
        )
        self.assertEqual(res1.score, 0.25)

        # 2. In gap but leaning 10 deg (0.50)
        q_lean10 = np.array([0.0, 0.0872, 0.0, 0.9962])
        p5_in_gap = np.array(g.SLOT_TARGET_POS)
        res2 = self.tl.update(
            self.p1, self.q,
            self.p2, self.q,
            self.p3, self.q,
            self.p4, self.q,
            p5_in_gap, q_lean10,
            self.arm_tcps,
        )
        self.assertEqual(res2.score, 0.50)

        # 3. In gap with low tilt (0.75)
        res3 = self.tl.update(
            self.p1, self.q,
            self.p2, self.q,
            self.p3, self.q,
            self.p4, self.q,
            p5_in_gap, self.q,
            self.arm_tcps,
        )
        self.assertEqual(res3.score, 0.75)
        self.assertFalse(res3.success)

        # 4. Hold for 38 more steps (total 39 steps < 40 req) -> still 0.75
        for _ in range(38):
            res = self.tl.update(
                self.p1, self.q,
                self.p2, self.q,
                self.p3, self.q,
                self.p4, self.q,
                p5_in_gap, self.q,
                self.arm_tcps,
            )
            self.assertEqual(res.score, 0.75)
            self.assertFalse(res.success)

        # 40th step completes 2.0 s hold -> 1.00
        res_final = self.tl.update(
            self.p1, self.q,
            self.p2, self.q,
            self.p3, self.q,
            self.p4, self.q,
            p5_in_gap, self.q,
            self.arm_tcps,
        )
        self.assertEqual(res_final.score, 1.00)
        self.assertTrue(res_final.success)


if __name__ == "__main__":
    unittest.main()

