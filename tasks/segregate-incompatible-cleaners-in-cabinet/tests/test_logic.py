"""Unit tests for logic.py in segregate-incompatible-cleaners-in-cabinet."""

from __future__ import annotations

import unittest
import numpy as np

import geometry as g
from logic import CleanersCabinetLogic


class TestLogic(unittest.TestCase):
    def test_initial_state(self):
        logic = CleanersCabinetLogic(variant="A")
        st = logic.step(
            dt=0.05,
            bleach_pos=g.STAGING_LEFT_POS,
            bleach_quat=(0.0, 0.0, 0.0, 1.0),
            bleach_vel=(0.0, 0.0, 0.0),
            ammonia_pos=g.STAGING_RIGHT_POS,
            ammonia_quat=(0.0, 0.0, 0.0, 1.0),
            ammonia_vel=(0.0, 0.0, 0.0),
        )
        self.assertEqual(st.score, 0.0)
        self.assertFalse(st.solved)
        self.assertFalse(st.incompatible_mix_violation)

    def test_variant_a_success_bleach_left_ammonia_right(self):
        logic = CleanersCabinetLogic(variant="A", hold_s=2.0)
        # Bleach in Left Bay, Ammonia in Right Bay
        for _ in range(45):  # 45 * 0.05s = 2.25s > 2.0s
            st = logic.step(
                dt=0.05,
                bleach_pos=g.BAY_LEFT_POS,
                bleach_quat=(0.0, 0.0, 0.0, 1.0),
                bleach_vel=(0.0, 0.0, 0.0),
                ammonia_pos=g.BAY_RIGHT_POS,
                ammonia_quat=(0.0, 0.0, 0.0, 1.0),
                ammonia_vel=(0.0, 0.0, 0.0),
            )
        self.assertTrue(st.both_placed_separate)
        self.assertTrue(st.solved)
        self.assertEqual(st.score, 1.00)
        self.assertFalse(st.incompatible_mix_violation)

    def test_variant_b_success_ammonia_left_bleach_right(self):
        logic = CleanersCabinetLogic(variant="B", hold_s=2.0)
        # Ammonia in Left Bay, Bleach in Right Bay (both assignments legal!)
        for _ in range(45):
            st = logic.step(
                dt=0.05,
                bleach_pos=g.BAY_RIGHT_POS,
                bleach_quat=(0.0, 0.0, 0.0, 1.0),
                bleach_vel=(0.0, 0.0, 0.0),
                ammonia_pos=g.BAY_LEFT_POS,
                ammonia_quat=(0.0, 0.0, 0.0, 1.0),
                ammonia_vel=(0.0, 0.0, 0.0),
            )
        self.assertTrue(st.both_placed_separate)
        self.assertTrue(st.solved)
        self.assertEqual(st.score, 1.00)
        self.assertFalse(st.incompatible_mix_violation)

    def test_same_bay_latched_violation(self):
        logic = CleanersCabinetLogic(variant="A")
        # Both bottles placed together in Left Bay
        st = logic.step(
            dt=0.05,
            bleach_pos=g.BAY_LEFT_POS,
            bleach_quat=(0.0, 0.0, 0.0, 1.0),
            bleach_vel=(0.0, 0.0, 0.0),
            ammonia_pos=(g.CABINET_X, g.BAY_LEFT_Y + 0.010, g.BOTTLE_CENTER_Z),
            ammonia_quat=(0.0, 0.0, 0.0, 1.0),
            ammonia_vel=(0.0, 0.0, 0.0),
        )
        self.assertTrue(st.incompatible_mix_violation)
        self.assertEqual(st.score, 0.25)
        self.assertFalse(st.solved)

    def test_proximity_latched_violation(self):
        logic = CleanersCabinetLogic(variant="A")
        # Bottles placed within 0.080 m (< 0.120 m threshold)
        st = logic.step(
            dt=0.05,
            bleach_pos=(0.250, 0.040, g.BOTTLE_CENTER_Z),
            bleach_quat=(0.0, 0.0, 0.0, 1.0),
            bleach_vel=(0.0, 0.0, 0.0),
            ammonia_pos=(0.250, -0.040, g.BOTTLE_CENTER_Z),
            ammonia_quat=(0.0, 0.0, 0.0, 1.0),
            ammonia_vel=(0.0, 0.0, 0.0),
        )
        self.assertTrue(st.incompatible_mix_violation)
        self.assertEqual(st.score, 0.25)
        self.assertFalse(st.solved)

    def test_one_bottle_placed_score(self):
        logic = CleanersCabinetLogic(variant="A")
        # Bleach placed in Left Bay, Ammonia untouched at right staging
        st = logic.step(
            dt=0.05,
            bleach_pos=g.BAY_LEFT_POS,
            bleach_quat=(0.0, 0.0, 0.0, 1.0),
            bleach_vel=(0.0, 0.0, 0.0),
            ammonia_pos=g.STAGING_RIGHT_POS,
            ammonia_quat=(0.0, 0.0, 0.0, 1.0),
            ammonia_vel=(0.0, 0.0, 0.0),
        )
        self.assertTrue(st.one_placed)
        self.assertFalse(st.both_moved)
        self.assertEqual(st.score, 0.25)

    def test_both_moved_score(self):
        logic = CleanersCabinetLogic(variant="A")
        # Both bottles lifted into transit (neither in bay yet)
        st = logic.step(
            dt=0.05,
            bleach_pos=(0.220, 0.290, g.HOVER_Z),
            bleach_quat=(0.0, 0.0, 0.0, 1.0),
            bleach_vel=(0.0, 0.0, 0.0),
            ammonia_pos=(0.220, -0.290, g.HOVER_Z),
            ammonia_quat=(0.0, 0.0, 0.0, 1.0),
            ammonia_vel=(0.0, 0.0, 0.0),
        )
        self.assertTrue(st.both_moved)
        self.assertFalse(st.one_placed)
        self.assertEqual(st.score, 0.50)

    def test_tilt_threshold_for_success(self):
        logic = CleanersCabinetLogic(variant="A")
        # Tilt = 5.5 deg (between 5.0 and 6.0 deg)
        # Quaternion for 5.5 deg rotation about X
        theta = np.radians(5.5)
        q_tilt = (np.sin(theta / 2), 0.0, 0.0, np.cos(theta / 2))

        for _ in range(45):
            st = logic.step(
                dt=0.05,
                bleach_pos=g.BAY_LEFT_POS,
                bleach_quat=q_tilt,
                bleach_vel=(0.0, 0.0, 0.0),
                ammonia_pos=g.BAY_RIGHT_POS,
                ammonia_quat=(0.0, 0.0, 0.0, 1.0),
                ammonia_vel=(0.0, 0.0, 0.0),
            )
        # Tilt >= 5.0 prevents 1.00 full success, but tilt < 6.0 allows 0.75
        self.assertTrue(st.both_placed_separate)
        self.assertEqual(st.score, 0.75)
        self.assertFalse(st.solved)


if __name__ == "__main__":
    unittest.main()
