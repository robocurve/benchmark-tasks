"""Unit tests for Task 4 hidden-state logic: blink code timing, healthy module removal, and green verification."""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))
sys.path.insert(0, str(HERE))

from logic import BlinkCodeLogic, FAILED_SOCKET, FAILED_MODULE, SPARE_MODULE


class TestTask4Logic(unittest.TestCase):
    def setUp(self) -> None:
        self.logic = BlinkCodeLogic()
        self.normal_seated = {
            "module_1": 0,
            "module_2": 1,
            "module_3": 2,
            "module_4": 3,
            "spare": None,
        }

    def test_initial_unpressed_state(self) -> None:
        st = self.logic.update(self.normal_seated, 0.0)
        self.assertFalse(st.diag_held)
        self.assertEqual(st.held_steps, 0)
        self.assertEqual(st.lamp_state, "off")
        self.assertFalse(st.swap_completed)
        self.assertTrue(st.healthy_seated)
        self.assertFalse(st.diag_held_ge_3s)
        self.assertFalse(st.green_verified)

    def test_diag_button_hysteresis(self) -> None:
        # Not pressed at rest
        st = self.logic.update(self.normal_seated, -0.002)
        self.assertFalse(st.diag_held)

        # In intermediate zone (-3.5 mm to -6.4 mm) before reaching press threshold -> still unpressed
        st = self.logic.update(self.normal_seated, -0.005)
        self.assertFalse(st.diag_held)

        # Reaches press threshold <= -0.0064 m (-6.4 mm) -> becomes pressed
        st = self.logic.update(self.normal_seated, -0.0065)
        self.assertTrue(st.diag_held)

        # In intermediate zone while releasing -> remains pressed due to hysteresis
        st = self.logic.update(self.normal_seated, -0.005)
        self.assertTrue(st.diag_held)

        # Crosses release threshold >= -0.0035 m -> releases
        st = self.logic.update(self.normal_seated, -0.0034)
        self.assertFalse(st.diag_held)

    def test_blink_code_timing_3_pulses_and_gap(self) -> None:
        """3 pulses (6 steps ON = 0.3s, 6 steps OFF = 0.3s), then 30 steps OFF = 1.5s gap."""
        # Hold DIAG down continuously
        history = []
        for _ in range(66 * 2):  # Two full cycles
            st = self.logic.update(self.normal_seated, -0.007)
            history.append(st.lamp_state)

        # Check Pulse 1 (steps 0..5 ON, 6..11 OFF)
        self.assertTrue(all(h == "blink" for h in history[0:6]), f"Pulse 1 ON failed: {history[0:6]}")
        self.assertTrue(all(h == "off" for h in history[6:12]), f"Pulse 1 OFF failed: {history[6:12]}")

        # Check Pulse 2 (steps 12..17 ON, 18..23 OFF)
        self.assertTrue(all(h == "blink" for h in history[12:18]), f"Pulse 2 ON failed: {history[12:18]}")
        self.assertTrue(all(h == "off" for h in history[18:24]), f"Pulse 2 OFF failed: {history[18:24]}")

        # Check Pulse 3 (steps 24..29 ON, 30..35 OFF)
        self.assertTrue(all(h == "blink" for h in history[24:30]), f"Pulse 3 ON failed: {history[24:30]}")
        self.assertTrue(all(h == "off" for h in history[30:36]), f"Pulse 3 OFF failed: {history[30:36]}")

        # Check Gap (steps 36..65 OFF, exactly 30 steps)
        self.assertTrue(all(h == "off" for h in history[36:66]), f"Gap OFF failed: {history[36:66]}")

        # Check repeat starts exactly at step 66
        self.assertEqual(history[66], "blink")
        self.assertTrue(all(h == "blink" for h in history[66:72]))

    def test_release_resets_blink_pattern(self) -> None:
        # Step through 4 steps of pulse 1
        for _ in range(4):
            self.logic.update(self.normal_seated, -0.007)

        # Release button
        st_rel = self.logic.update(self.normal_seated, 0.0)
        self.assertEqual(st_rel.lamp_state, "off")
        self.assertEqual(st_rel.held_steps, 0)

        # Press again: must restart at pulse 1 step 0 (lamp = 'blink')
        st_next = self.logic.update(self.normal_seated, -0.007)
        self.assertEqual(st_next.lamp_state, "blink")

    def test_healthy_module_removed_shows_steady_red(self) -> None:
        # Remove module_2 from socket 1
        tampered_seated = dict(self.normal_seated)
        tampered_seated["module_2"] = None

        # Hold DIAG down -> lamp must show steady red
        for _ in range(10):
            st = self.logic.update(tampered_seated, -0.007)
            self.assertFalse(st.healthy_seated)
            self.assertEqual(st.lamp_state, "red")

    def test_swap_completed_shows_steady_green_and_verifies(self) -> None:
        # Spare in socket 2, module_3 unseated, healthy seated
        swapped_seated = {
            "module_1": 0,
            "module_2": 1,
            "module_3": None,
            "module_4": 3,
            "spare": 2,
        }

        # Press and hold DIAG for 59 steps: green, but not yet verified (needs >= 60 steps = 3 s)
        for _ in range(59):
            st = self.logic.update(swapped_seated, -0.007)
            self.assertTrue(st.swap_completed)
            self.assertEqual(st.lamp_state, "green")
            self.assertFalse(st.green_verified)

        # Step 60: green_verified becomes True
        st60 = self.logic.update(swapped_seated, -0.007)
        self.assertTrue(st60.green_verified)
        self.assertEqual(st60.lamp_state, "green")


if __name__ == "__main__":
    unittest.main()
