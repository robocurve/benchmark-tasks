"""Tests for state machine and stack verification logic in stack-crates-largest-first."""

import unittest
import numpy as np

import geometry as g
import logic


class TestLogic(unittest.TestCase):
    def setUp(self):
        self.lg = logic.StackLogic()

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
        self.assertFalse(st.large_on_pallet)
        self.assertFalse(st.medium_on_large)
        self.assertFalse(st.small_on_medium)
        self.assertFalse(st.solved)
        self.assertFalse(st.floor_fault)

    def test_large_on_pallet(self):
        large_on_pallet_pos = (g.PALLET_POS[0], g.PALLET_POS[1], g.LEVEL_1_TARGET_Z)
        st = self.lg.step(
            dt=0.05,
            large_pos=large_on_pallet_pos,
            large_quat=(0.0, 0.0, 0.0, 1.0),
            large_vel=(0.0, 0.0, 0.0),
            medium_pos=g.MEDIUM_START_POS,
            medium_quat=(0.0, 0.0, 0.0, 1.0),
            medium_vel=(0.0, 0.0, 0.0),
            small_pos=g.SMALL_START_POS,
            small_quat=(0.0, 0.0, 0.0, 1.0),
            small_vel=(0.0, 0.0, 0.0),
        )
        self.assertTrue(st.large_on_pallet)
        self.assertFalse(st.medium_on_large)

    def test_full_correct_stack(self):
        large_pos = (g.PALLET_POS[0], g.PALLET_POS[1], g.LEVEL_1_TARGET_Z)
        med_pos = (g.PALLET_POS[0], g.PALLET_POS[1], g.LEVEL_2_TARGET_Z)
        small_pos = (g.PALLET_POS[0], g.PALLET_POS[1], g.LEVEL_3_TARGET_Z)

        arm_tcps = {
            "left": (0.200, 0.280, 0.900),
            "right": (0.200, -0.280, 0.900),
        }

        st = self.lg.step(
            dt=0.05,
            large_pos=large_pos,
            large_quat=(0.0, 0.0, 0.0, 1.0),
            large_vel=(0.0, 0.0, 0.0),
            medium_pos=med_pos,
            medium_quat=(0.0, 0.0, 0.0, 1.0),
            medium_vel=(0.0, 0.0, 0.0),
            small_pos=small_pos,
            small_quat=(0.0, 0.0, 0.0, 1.0),
            small_vel=(0.0, 0.0, 0.0),
            arm_tcps=arm_tcps,
        )
        self.assertTrue(st.large_on_pallet)
        self.assertTrue(st.medium_on_large)
        self.assertTrue(st.small_on_medium)
        self.assertTrue(st.all_offsets_valid)
        self.assertTrue(st.all_tilts_valid)
        self.assertTrue(st.all_crates_still)
        self.assertTrue(st.arms_clear)
        self.assertTrue(st.solved)

    def test_wrong_order_rejection(self):
        """Rule: A crate cannot count if supported by a smaller crate."""
        # Medium on pallet first
        med_on_pallet = (g.PALLET_POS[0], g.PALLET_POS[1], g.LEVEL_1_TARGET_Z)
        # Large placed on top of Medium
        large_on_med = (g.PALLET_POS[0], g.PALLET_POS[1], g.LEVEL_2_TARGET_Z)

        st = self.lg.step(
            dt=0.05,
            large_pos=large_on_med,
            large_quat=(0.0, 0.0, 0.0, 1.0),
            large_vel=(0.0, 0.0, 0.0),
            medium_pos=med_on_pallet,
            medium_quat=(0.0, 0.0, 0.0, 1.0),
            medium_vel=(0.0, 0.0, 0.0),
            small_pos=g.SMALL_START_POS,
            small_quat=(0.0, 0.0, 0.0, 1.0),
            small_vel=(0.0, 0.0, 0.0),
        )
        # Large is on top of smaller crate -> not counted as on pallet
        self.assertFalse(st.large_on_pallet)
        self.assertFalse(st.medium_on_large)
        self.assertFalse(st.solved)

    def test_floor_fault(self):
        # Large crate fell off table
        large_fallen = (g.PALLET_POS[0], g.PALLET_POS[1], g.TOP - 0.100)
        st = self.lg.step(
            dt=0.05,
            large_pos=large_fallen,
            large_quat=(0.0, 0.0, 0.0, 1.0),
            large_vel=(0.0, 0.0, 0.0),
            medium_pos=g.MEDIUM_START_POS,
            medium_quat=(0.0, 0.0, 0.0, 1.0),
            medium_vel=(0.0, 0.0, 0.0),
            small_pos=g.SMALL_START_POS,
            small_quat=(0.0, 0.0, 0.0, 1.0),
            small_vel=(0.0, 0.0, 0.0),
        )
        self.assertTrue(st.floor_fault)
        self.assertFalse(st.solved)


if __name__ == "__main__":
    unittest.main()

