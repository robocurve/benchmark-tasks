"""Unit tests for state logic in two-key-authorization-hold."""

from __future__ import annotations

import sys
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import geometry as g
import logic


class TestTwoKeyLogic(unittest.TestCase):
    def test_initial_state(self):
        l = logic.TwoKeyLogic()
        st = l.state
        self.assertFalse(st.both_guards_open)
        self.assertFalse(st.both_keys_held)
        self.assertFalse(st.armed)
        self.assertFalse(st.locked)
        self.assertFalse(st.solved)

    def test_guard_opening_threshold(self):
        l = logic.TwoKeyLogic()
        st = l.step(0.05, {}, {"guard_left": 0.050, "guard_right": 0.0})
        self.assertFalse(st.guard_left_open)
        self.assertFalse(st.both_guards_open)

        st = l.step(0.05, {}, {"guard_left": 0.065, "guard_right": 0.070})
        self.assertTrue(st.guard_left_open)
        self.assertTrue(st.guard_right_open)
        self.assertTrue(st.both_guards_open)

    def test_simultaneous_hold_and_arming(self):
        l = logic.TwoKeyLogic()
        guards = {"guard_left": 0.070, "guard_right": 0.070}
        btns = {"key_left": -0.007, "key_right": -0.007, "confirm": 0.0}

        for _ in range(40):
            st = l.step(0.05, btns, guards)

        self.assertEqual(st.progress_count, 2)
        self.assertFalse(st.armed)
        self.assertAlmostEqual(st.simultaneous_hold_time, 2.0, places=2)

        for _ in range(22):
            st = l.step(0.05, btns, guards)

        self.assertTrue(st.armed)
        self.assertEqual(st.progress_count, 4)
        self.assertFalse(st.locked)

    def test_strike_on_early_release(self):
        l = logic.TwoKeyLogic()
        guards = {"guard_left": 0.070, "guard_right": 0.070}
        btns_pressed = {"key_left": -0.007, "key_right": -0.007, "confirm": 0.0}
        btns_released = {"key_left": 0.0, "key_right": 0.0, "confirm": 0.0}

        for _ in range(20):
            st = l.step(0.05, btns_pressed, guards)
        self.assertAlmostEqual(st.simultaneous_hold_time, 1.0, places=2)

        st = l.step(0.05, btns_released, guards)
        self.assertEqual(st.strikes, 1)
        self.assertEqual(st.simultaneous_hold_time, 0.0)
        self.assertFalse(st.locked)

        for _ in range(20):
            st = l.step(0.05, btns_pressed, guards)
        st = l.step(0.05, btns_released, guards)
        self.assertEqual(st.strikes, 2)
        self.assertTrue(st.locked)
        self.assertTrue(st.fault_lamp)

    def test_timeout_after_armed(self):
        l = logic.TwoKeyLogic()
        guards = {"guard_left": 0.070, "guard_right": 0.070}
        btns = {"key_left": -0.007, "key_right": -0.007, "confirm": 0.0}
        for _ in range(65):
            st = l.step(0.05, btns, guards)
        self.assertTrue(st.armed)

        btns_rel = {"key_left": 0.0, "key_right": 0.0, "confirm": 0.0}
        for _ in range(205):
            st = l.step(0.05, btns_rel, guards)

        self.assertFalse(st.armed)
        self.assertTrue(st.locked)
        self.assertTrue(st.fault_lamp)

    def test_confirm_press_solves(self):
        l = logic.TwoKeyLogic()
        guards = {"guard_left": 0.070, "guard_right": 0.070}
        btns = {"key_left": -0.007, "key_right": -0.007, "confirm": 0.0}
        for _ in range(65):
            st = l.step(0.05, btns, guards)
        self.assertTrue(st.armed)

        st = l.step(0.05, {"key_left": 0.0, "key_right": 0.0, "confirm": -0.007}, guards)
        self.assertTrue(st.confirm_pressed)
        self.assertTrue(st.solved)
        self.assertFalse(st.locked)


if __name__ == "__main__":
    unittest.main()
