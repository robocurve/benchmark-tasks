"""Tests for state machine and rule verification in tower-transfer-no-big-on-small."""

import unittest
import numpy as np

import geometry as g
import logic


class TestLogic(unittest.TestCase):
    def setUp(self):
        self.lg = logic.TowerLogic(hold_s=2.0)

    def test_initial_state(self):
        st = self.lg.step(
            dt=0.05,
            large_pos=g.LARGE_START_POS,
            large_quat=(0.0, 0.0, 0.0, 1.0),
            large_vel=(0.0, 0.0, 0.0),
            medium_pos=g.MEDIUM_START_POS,
            medium_quat=(0.0, 0.0, 0.0, 1.0),
            medium_vel=(0.0, 0.0, 0.0),
            small_pos=g.SMALL_START_POS,
            small_quat=(0.0, 0.0, 0.0, 1.0),
            small_vel=(0.0, 0.0, 0.0),
        )
        self.assertFalse(st.rule_broken)
        self.assertFalse(st.floor_fault)
        self.assertEqual(st.large_mark, "LEFT")
        self.assertEqual(st.med_mark, "LEFT")
        self.assertEqual(st.small_mark, "LEFT")
        self.assertFalse(st.ever_25)
        self.assertFalse(st.ever_50)
        self.assertFalse(st.ever_75)
        self.assertFalse(st.ever_100)
        self.assertFalse(st.solved)

    def test_zero_gate_big_on_small(self):
        # Medium on Small resting at MID mark
        mid_s = (g.MID_MARK_POS[0], g.MID_MARK_POS[1], g.LEVEL_1_TARGET_Z)
        mid_m_illegal = (g.MID_MARK_POS[0], g.MID_MARK_POS[1], g.LEVEL_2_TARGET_Z)

        # First step moving: should not trigger until resting 0.3 s
        st = self.lg.step(
            dt=0.05,
            large_pos=g.LARGE_START_POS,
            large_quat=(0.0, 0.0, 0.0, 1.0),
            large_vel=(0.0, 0.0, 0.0),
            medium_pos=mid_m_illegal,
            medium_quat=(0.0, 0.0, 0.0, 1.0),
            medium_vel=(0.02, 0.0, 0.0),  # moving
            small_pos=mid_s,
            small_quat=(0.0, 0.0, 0.0, 1.0),
            small_vel=(0.0, 0.0, 0.0),
        )
        self.assertFalse(st.rule_broken)

        # Resting for >= 0.3 s (6 steps at 0.05 s)
        for _ in range(7):
            st = self.lg.step(
                dt=0.05,
                large_pos=g.LARGE_START_POS,
                large_quat=(0.0, 0.0, 0.0, 1.0),
                large_vel=(0.0, 0.0, 0.0),
                medium_pos=mid_m_illegal,
                medium_quat=(0.0, 0.0, 0.0, 1.0),
                medium_vel=(0.0, 0.0, 0.0),  # still
                small_pos=mid_s,
                small_quat=(0.0, 0.0, 0.0, 1.0),
                small_vel=(0.0, 0.0, 0.0),
            )
        self.assertTrue(st.rule_broken)

    def test_floor_fault(self):
        st = self.lg.step(
            dt=0.05,
            large_pos=g.LARGE_START_POS,
            large_quat=(0.0, 0.0, 0.0, 1.0),
            large_vel=(0.0, 0.0, 0.0),
            medium_pos=g.MEDIUM_START_POS,
            medium_quat=(0.0, 0.0, 0.0, 1.0),
            medium_vel=(0.0, 0.0, 0.0),
            small_pos=(0.30, 0.0, g.TOP - 0.100),  # fell off table
            small_quat=(0.0, 0.0, 0.0, 1.0),
            small_vel=(0.0, 0.0, 0.0),
        )
        self.assertTrue(st.floor_fault)

    def test_7_move_optimal_solution(self):
        """Simulate the 7-move optimal solution step by step."""
        lg = logic.TowerLogic(hold_s=2.0)
        arm_tcps = {"left": (0.20, 0.28, 0.88), "right": (0.20, -0.28, 0.88)}

        def settle(l_pos, m_pos, s_pos, n=8):
            st = None
            for _ in range(n):
                st = lg.step(
                    dt=0.05,
                    large_pos=l_pos, large_quat=(0.0, 0.0, 0.0, 1.0), large_vel=(0.0, 0.0, 0.0),
                    medium_pos=m_pos, medium_quat=(0.0, 0.0, 0.0, 1.0), medium_vel=(0.0, 0.0, 0.0),
                    small_pos=s_pos, small_quat=(0.0, 0.0, 0.0, 1.0), small_vel=(0.0, 0.0, 0.0),
                    arm_tcps=arm_tcps,
                )
            return st

        xy_l = (g.LEFT_MARK_POS[0], g.LEFT_MARK_POS[1])
        xy_m = (g.MID_MARK_POS[0], g.MID_MARK_POS[1])
        xy_r = (g.RIGHT_MARK_POS[0], g.RIGHT_MARK_POS[1])

        z_l1, z_l2, z_l3 = g.LEVEL_1_TARGET_Z, g.LEVEL_2_TARGET_Z, g.LEVEL_3_TARGET_Z

        # Move 1: S -> R
        pos_l = (xy_l[0], xy_l[1], z_l1)
        pos_m = (xy_l[0], xy_l[1], z_l2)
        pos_s = (xy_r[0], xy_r[1], z_l1)
        st = settle(pos_l, pos_m, pos_s)
        self.assertFalse(st.rule_broken)
        self.assertTrue(st.ever_25)

        # Move 2: M -> MID
        pos_m = (xy_m[0], xy_m[1], z_l1)
        st = settle(pos_l, pos_m, pos_s)
        self.assertFalse(st.rule_broken)

        # Move 3: S -> MID (on M)
        pos_s = (xy_m[0], xy_m[1], z_l2)
        st = settle(pos_l, pos_m, pos_s)
        self.assertFalse(st.rule_broken)

        # Move 4: L -> R
        pos_l = (xy_r[0], xy_r[1], z_l1)
        st = settle(pos_l, pos_m, pos_s)
        self.assertFalse(st.rule_broken)
        self.assertTrue(st.ever_50)
        self.assertTrue(st.ever_75)

        # Move 5: S -> LEFT
        pos_s = (xy_l[0], xy_l[1], z_l1)
        st = settle(pos_l, pos_m, pos_s)
        self.assertFalse(st.rule_broken)

        # Move 6: M -> R (on L)
        pos_m = (xy_r[0], xy_r[1], z_l2)
        st = settle(pos_l, pos_m, pos_s)
        self.assertFalse(st.rule_broken)

        # Move 7: S -> R (on M)
        pos_s = (xy_r[0], xy_r[1], z_l3)
        st = settle(pos_l, pos_m, pos_s)
        self.assertFalse(st.rule_broken)

        # Hold for 2.0 s (40 steps)
        for _ in range(40):
            st = lg.step(
                dt=0.05,
                large_pos=pos_l, large_quat=(0.0, 0.0, 0.0, 1.0), large_vel=(0.0, 0.0, 0.0),
                medium_pos=pos_m, medium_quat=(0.0, 0.0, 0.0, 1.0), medium_vel=(0.0, 0.0, 0.0),
                small_pos=pos_s, small_quat=(0.0, 0.0, 0.0, 1.0), small_vel=(0.0, 0.0, 0.0),
                arm_tcps=arm_tcps,
            )
        self.assertTrue(st.ever_100)
        self.assertTrue(st.solved)

    def test_9_move_valid_solution(self):
        """Simulate an alternate valid longer legal solution (9 moves)."""
        lg = logic.TowerLogic(hold_s=2.0)
        arm_tcps = {"left": (0.20, 0.28, 0.88), "right": (0.20, -0.28, 0.88)}

        def settle(l_pos, m_pos, s_pos, n=8):
            st = None
            for _ in range(n):
                st = lg.step(
                    dt=0.05,
                    large_pos=l_pos, large_quat=(0.0, 0.0, 0.0, 1.0), large_vel=(0.0, 0.0, 0.0),
                    medium_pos=m_pos, medium_quat=(0.0, 0.0, 0.0, 1.0), medium_vel=(0.0, 0.0, 0.0),
                    small_pos=s_pos, small_quat=(0.0, 0.0, 0.0, 1.0), small_vel=(0.0, 0.0, 0.0),
                    arm_tcps=arm_tcps,
                )
            return st

        xy_l = (g.LEFT_MARK_POS[0], g.LEFT_MARK_POS[1])
        xy_m = (g.MID_MARK_POS[0], g.MID_MARK_POS[1])
        xy_r = (g.RIGHT_MARK_POS[0], g.RIGHT_MARK_POS[1])

        z_l1, z_l2, z_l3 = g.LEVEL_1_TARGET_Z, g.LEVEL_2_TARGET_Z, g.LEVEL_3_TARGET_Z

        # 1. S -> MID
        pos_l = (xy_l[0], xy_l[1], z_l1)
        pos_m = (xy_l[0], xy_l[1], z_l2)
        pos_s = (xy_m[0], xy_m[1], z_l1)
        st = settle(pos_l, pos_m, pos_s)
        self.assertFalse(st.rule_broken)
        self.assertTrue(st.ever_25)

        # 2. S -> R
        pos_s = (xy_r[0], xy_r[1], z_l1)
        st = settle(pos_l, pos_m, pos_s)
        self.assertFalse(st.rule_broken)

        # 3. M -> MID
        pos_m = (xy_m[0], xy_m[1], z_l1)
        st = settle(pos_l, pos_m, pos_s)
        self.assertFalse(st.rule_broken)

        # 4. S -> MID (on M)
        pos_s = (xy_m[0], xy_m[1], z_l2)
        st = settle(pos_l, pos_m, pos_s)
        self.assertFalse(st.rule_broken)

        # 5. L -> R
        pos_l = (xy_r[0], xy_r[1], z_l1)
        st = settle(pos_l, pos_m, pos_s)
        self.assertFalse(st.rule_broken)
        self.assertTrue(st.ever_50)

        # 6. S -> LEFT
        pos_s = (xy_l[0], xy_l[1], z_l1)
        st = settle(pos_l, pos_m, pos_s)
        self.assertFalse(st.rule_broken)

        # 7. M -> R (on L)
        pos_m = (xy_r[0], xy_r[1], z_l2)
        st = settle(pos_l, pos_m, pos_s)
        self.assertFalse(st.rule_broken)

        # 8. S -> MID
        pos_s = (xy_m[0], xy_m[1], z_l1)
        st = settle(pos_l, pos_m, pos_s)
        self.assertFalse(st.rule_broken)

        # 9. S -> R (on M)
        pos_s = (xy_r[0], xy_r[1], z_l3)
        st = settle(pos_l, pos_m, pos_s)
        self.assertFalse(st.rule_broken)

        for _ in range(40):
            st = lg.step(
                dt=0.05,
                large_pos=pos_l, large_quat=(0.0, 0.0, 0.0, 1.0), large_vel=(0.0, 0.0, 0.0),
                medium_pos=pos_m, medium_quat=(0.0, 0.0, 0.0, 1.0), medium_vel=(0.0, 0.0, 0.0),
                small_pos=pos_s, small_quat=(0.0, 0.0, 0.0, 1.0), small_vel=(0.0, 0.0, 0.0),
                arm_tcps=arm_tcps,
            )
        self.assertTrue(st.ever_100)
        self.assertTrue(st.solved)


if __name__ == "__main__":
    unittest.main()

