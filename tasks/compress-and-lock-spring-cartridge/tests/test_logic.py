"""Tests for state machine and verification logic in compress-and-lock-spring-cartridge."""

import unittest
import numpy as np

import geometry as g
import logic


class TestLogic(unittest.TestCase):
    def setUp(self):
        self.lg = logic.CartridgeLogic()

    def test_initial_state(self):
        st = self.lg.step(
            dt=0.05,
            raw_plunger_disp=0.0,
            raw_bar_disp=0.0,
            forced_plunger_released=True,
        )
        self.assertFalse(st.plunger_compressed_1s)
        self.assertFalse(st.bar_across)
        self.assertFalse(st.ever_locked)
        self.assertFalse(st.released_and_held_2s)
        self.assertFalse(st.lamp_green)
        self.assertFalse(st.solved)
        self.assertFalse(st.bar_blocked)

    def test_physical_stop_box_blocks_early_bar_push(self):
        """When plunger is not compressed (< 38 mm), the stop box limits bar travel to 15 mm."""
        # Case 1: Plunger at rest (0 mm), bar commanded to 40 mm
        st = self.lg.step(dt=0.05, raw_plunger_disp=0.000, raw_bar_disp=0.040, forced_plunger_released=True)
        self.assertEqual(st.bar_disp, 0.015)
        self.assertTrue(st.bar_blocked)
        self.assertFalse(st.ever_locked)

        # Case 2: Plunger partially pushed (25 mm), bar commanded to 60 mm
        st = self.lg.step(dt=0.05, raw_plunger_disp=0.025, raw_bar_disp=0.060, forced_plunger_released=True)
        self.assertEqual(st.bar_disp, 0.015)
        self.assertTrue(st.bar_blocked)
        self.assertFalse(st.ever_locked)

        # Case 3: Plunger compressed to threshold (38 mm) -> stop box cleared, notch aligned!
        st = self.lg.step(dt=0.05, raw_plunger_disp=0.038, raw_bar_disp=0.060, forced_plunger_released=False)
        self.assertEqual(st.bar_disp, 0.060)
        self.assertFalse(st.bar_blocked)
        self.assertTrue(st.ever_locked)

    def test_plunger_compressed_hold_duration(self):
        """Plunger must be held >= 38 mm for >= 1.0 s (20 steps at 20 Hz)."""
        for _ in range(19):
            st = self.lg.step(dt=0.05, raw_plunger_disp=0.040, raw_bar_disp=0.0, forced_plunger_released=False)
            self.assertFalse(st.plunger_compressed_1s)

        # Step 20 reaches 1.0 s
        st = self.lg.step(dt=0.05, raw_plunger_disp=0.040, raw_bar_disp=0.0, forced_plunger_released=False)
        self.assertTrue(st.plunger_compressed_1s)

    def test_latched_ever_locked(self):
        """Lesson 8: Latch ever_locked once bar across >= 55 mm while compressed."""
        # Compress plunger and slide bar across
        st = self.lg.step(dt=0.05, raw_plunger_disp=0.042, raw_bar_disp=0.060, forced_plunger_released=False)
        self.assertTrue(st.ever_locked)

        # Even if raw plunger measurement drops slightly, latched ever_locked persists
        st = self.lg.step(dt=0.05, raw_plunger_disp=0.035, raw_bar_disp=0.060, forced_plunger_released=False)
        self.assertTrue(st.ever_locked)

    def test_mechanical_retention_when_locked(self):
        """When locked, releasing arm contact retains plunger at >= 38 mm."""
        # 1. Lock the mechanism
        self.lg.step(dt=0.05, raw_plunger_disp=0.045, raw_bar_disp=0.060, forced_plunger_released=False)

        # 2. Arm releases (raw plunger spring tries to return to 0)
        st = self.lg.step(dt=0.05, raw_plunger_disp=0.000, raw_bar_disp=0.060, forced_plunger_released=True)
        # Plunger is mechanically retained by the bar at >= 38 mm!
        self.assertGreaterEqual(st.plunger_disp, 0.038)
        self.assertTrue(st.plunger_released)
        self.assertTrue(st.lamp_green)

    def test_full_correct_cycle_and_clearance(self):
        """Complete successful demonstration sequence."""
        # Step 1: Compress plunger for 1.0 s
        for _ in range(20):
            st = self.lg.step(dt=0.05, raw_plunger_disp=0.045, raw_bar_disp=0.0, forced_plunger_released=False)
        self.assertTrue(st.plunger_compressed_1s)

        # Step 2: Slide lock bar across to 60 mm (>= 55 mm)
        for _ in range(10):
            st = self.lg.step(dt=0.05, raw_plunger_disp=0.045, raw_bar_disp=0.060, forced_plunger_released=False)
        self.assertTrue(st.ever_locked)
        self.assertTrue(st.lamp_green)

        # Step 3: Release plunger arm; retained for 2.0 s (40 steps)
        for _ in range(40):
            st = self.lg.step(dt=0.05, raw_plunger_disp=0.000, raw_bar_disp=0.060, forced_plunger_released=True)
        self.assertTrue(st.released_and_held_2s)

        # Step 4: Arms clear for 1.0 s (20 steps)
        arm_clear_tcps = {
            "left": (0.200, +0.305, g.HOVER_Z),
            "right": (0.200, -0.305, g.HOVER_Z),
        }
        for _ in range(20):
            st = self.lg.step(
                dt=0.05,
                raw_plunger_disp=0.000,
                raw_bar_disp=0.060,
                arm_tcps=arm_clear_tcps,
                forced_plunger_released=True,
            )
        self.assertTrue(st.arms_clear)
        self.assertTrue(st.solved)

    def test_two_valid_orders(self):
        """Rule 4: Every valid order of actions reaches 1.0.

        Order 1: Plunger arm compresses first, then bar arm slides bar.
        Order 2: Bar arm reaches standby first (does not push past 15 mm), then plunger arm compresses, then bar locks.
        """
        # Order 1 tested in test_full_correct_cycle_and_clearance

        # Order 2:
        lg2 = logic.CartridgeLogic()
        arm_standby_tcps = {
            "left": (0.200, +0.305, g.HOVER_Z),
            "right": (g.LOCK_BAR_REST_X, g.LOCK_BAR_REST_Y - 0.030, g.LOCK_BAR_REST_Z),
        }
        # Bar arm in standby, bar at 0
        st = lg2.step(dt=0.05, raw_plunger_disp=0.0, raw_bar_disp=0.0, arm_tcps=arm_standby_tcps, forced_plunger_released=True)
        self.assertFalse(st.ever_locked)

        # Plunger compressed for 1.0 s
        for _ in range(20):
            st = lg2.step(dt=0.05, raw_plunger_disp=0.045, raw_bar_disp=0.0, forced_plunger_released=False)
        self.assertTrue(st.plunger_compressed_1s)

        # Bar slid across to 60 mm
        for _ in range(10):
            st = lg2.step(dt=0.05, raw_plunger_disp=0.045, raw_bar_disp=0.060, forced_plunger_released=False)
        self.assertTrue(st.ever_locked)

        # Plunger released and held 2.0 s
        for _ in range(40):
            st = lg2.step(dt=0.05, raw_plunger_disp=0.0, raw_bar_disp=0.060, forced_plunger_released=True)
        self.assertTrue(st.released_and_held_2s)

        # Arms clear for 1.0 s
        arm_clear_tcps = {
            "left": (0.200, +0.305, g.HOVER_Z),
            "right": (0.200, -0.305, g.HOVER_Z),
        }
        for _ in range(20):
            st = lg2.step(dt=0.05, raw_plunger_disp=0.0, raw_bar_disp=0.060, arm_tcps=arm_clear_tcps, forced_plunger_released=True)
        self.assertTrue(st.solved)


if __name__ == "__main__":
    unittest.main()
