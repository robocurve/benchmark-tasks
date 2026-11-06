"""Unit tests for logic.py: backlash hysteresis, direction reversal, target bands and LOCK states."""

import os
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

import logic


class TestLogic(unittest.TestCase):
    def test_backlash_forward_motion(self):
        dial = logic.BacklashDial("left")
        dial.reset(30.0)
        self.assertEqual(dial.displayed, 30.0)

        # Forward motion: display tracks true position
        y = dial.update_position(35.0)
        self.assertAlmostEqual(y, 35.0)
        y = dial.update_position(65.0)
        self.assertAlmostEqual(y, 65.0)

    def test_backlash_direction_reversal_deadband_and_lag(self):
        dial = logic.BacklashDial("right")
        dial.reset(70.0)
        self.assertEqual(dial.displayed, 70.0)

        # Reversing direction: moving downward from 70 mm
        # Deadband is 6 mm (70 down to 64 mm): display holds at 70 mm
        y = dial.update_position(68.0)
        self.assertAlmostEqual(y, 70.0)
        y = dial.update_position(66.0)
        self.assertAlmostEqual(y, 70.0)
        y = dial.update_position(64.0)
        self.assertAlmostEqual(y, 70.0)

        # Moving past 6 mm deadband: display follows with 6 mm lag (y = x + 6)
        y = dial.update_position(50.0)
        self.assertAlmostEqual(y, 56.0)
        y = dial.update_position(28.0)
        self.assertAlmostEqual(y, 34.0)

        # Reversing direction again: moving upward from 28 mm
        # Deadband is 6 mm (28 up to 34 mm): display holds at 34 mm
        y = dial.update_position(30.0)
        self.assertAlmostEqual(y, 34.0)
        y = dial.update_position(34.0)
        self.assertAlmostEqual(y, 34.0)

        # Moving upward past 34 mm: display follows true position (y = x)
        y = dial.update_position(38.0)
        self.assertAlmostEqual(y, 38.0)
        y = dial.update_position(40.0)
        self.assertAlmostEqual(y, 40.0)

    def test_reversal_detection(self):
        dial = logic.BacklashDial("right")
        dial.reset(70.0)
        self.assertFalse(dial.has_reversed)

        # Only downward motion -> no reversal
        dial.update_position(28.0)
        self.assertFalse(dial.has_reversed)

        # Followed by upward motion >= 6 mm -> reversal detected
        dial.update_position(40.0)
        self.assertTrue(dial.has_reversed)

    def test_target_band(self):
        self.assertTrue(logic.is_in_band("left", 65.0))
        self.assertTrue(logic.is_in_band("left", 62.5))
        self.assertTrue(logic.is_in_band("left", 67.5))
        self.assertFalse(logic.is_in_band("left", 62.4))
        self.assertFalse(logic.is_in_band("left", 67.6))

        self.assertTrue(logic.is_in_band("right", 40.0))
        self.assertTrue(logic.is_in_band("right", 37.5))
        self.assertTrue(logic.is_in_band("right", 42.5))
        self.assertFalse(logic.is_in_band("right", 37.4))
        self.assertFalse(logic.is_in_band("right", 42.6))

    def test_lock_button_toggle_and_latching(self):
        dial = logic.BacklashDial("left")
        dial.reset(65.0)

        # Button press (<= -6.4 mm)
        pressed, edge = dial.update_button(-0.007)
        self.assertTrue(pressed)
        self.assertTrue(edge)
        self.assertTrue(dial.locked)
        self.assertAlmostEqual(dial.locked_val, 65.0)
        self.assertTrue(dial.locked_in_band)
        self.assertEqual(dial.lock_attempts, 1)

        # Button release (>= -3.5 mm)
        pressed, edge = dial.update_button(0.0)
        self.assertFalse(pressed)
        self.assertFalse(edge)
        self.assertTrue(dial.locked)

        # Press again unlocks
        pressed, edge = dial.update_button(-0.007)
        self.assertTrue(edge)
        self.assertFalse(dial.locked)
        self.assertFalse(dial.locked_in_band)
        self.assertIsNone(dial.locked_val)

    def test_zero_gate_on_excessive_lock_attempts(self):
        log = logic.BacklashDialLogic()
        # Attempt 1 outside band
        log.step(30.0, 70.0, -0.007, 0.0)
        log.step(30.0, 70.0, 0.0, 0.0)
        self.assertFalse(log.zero_gate_latched)

        # Unlock
        log.step(30.0, 70.0, -0.007, 0.0)
        log.step(30.0, 70.0, 0.0, 0.0)
        self.assertFalse(log.zero_gate_latched)

        # Attempt 2 outside band -> latches zero gate (second outside lock)
        state = log.step(30.0, 70.0, -0.007, 0.0)
        self.assertTrue(state.zero_gate)


if __name__ == "__main__":
    unittest.main()
