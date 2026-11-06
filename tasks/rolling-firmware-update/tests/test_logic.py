"""Unit tests for rolling firmware update hidden state machine (logic.py)."""

import unittest
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import logic
import geometry as g


class TestLogic(unittest.TestCase):
    def setUp(self):
        self.logic = logic.RollingFirmwareLogic()

    def test_initial_state(self):
        state = self.logic.step(0.0, 0.0, 0.0, 0.0, 0.0)
        self.assertEqual(state["active_unit"], "A")
        self.assertFalse(state["outage"])
        self.assertEqual(state["unit_a_version"], 0)
        self.assertEqual(state["unit_b_version"], 0)
        self.assertFalse(state["unit_a_updating"])
        self.assertFalse(state["unit_b_updating"])
        self.assertEqual(state["load_lamp"], "green")
        self.assertEqual(state["version_lamp_a"], "grey")
        self.assertEqual(state["version_lamp_b"], "grey")
        self.assertEqual(state["progress_lamps_a"], 0)
        self.assertEqual(state["progress_lamps_b"], 0)

    def test_update_without_door_open_does_nothing(self):
        # Press UPDATE B without door open
        state = self.logic.step(
            door_a_deflection=0.0,
            door_b_deflection=0.010,  # 10 mm < 18 mm
            btn_update_a_z=0.0,
            btn_update_b_z=-0.007,   # pressed <= -6.4 mm
            btn_switch_z=0.0,
        )
        self.assertFalse(state["unit_b_updating"])
        self.assertEqual(state["unit_b_version"], 0)
        self.assertEqual(self.logic.counted_update_b_presses, 0)
        self.assertEqual(self.logic.raw_button_press_count, 1)

    def test_update_with_door_open_starts_update(self):
        # Press UPDATE B with door open 20 mm >= 18 mm
        state = self.logic.step(
            door_a_deflection=0.0,
            door_b_deflection=0.020,
            btn_update_a_z=0.0,
            btn_update_b_z=-0.007,
            btn_switch_z=0.0,
        )
        self.assertTrue(state["unit_b_updating"])
        self.assertEqual(state["unit_b_version"], 0)
        self.assertFalse(state["outage"])  # B is standby, so updating B causes no outage
        self.assertEqual(state["version_lamp_b"], "amber")
        self.assertEqual(self.logic.counted_update_b_presses, 1)

    def test_door_released_early_cancels_update(self):
        # Start update on B (step 1)
        self.logic.step(0.0, 0.020, 0.0, -0.007, 0.0)
        # Advance 39 more steps (total 40 steps = 2.0 s)
        for _ in range(39):
            state = self.logic.step(0.0, 0.020, 0.0, 0.0, 0.0)
        self.assertTrue(state["unit_b_updating"])
        self.assertEqual(state["unit_b_progress_steps"], 40)
        self.assertEqual(state["progress_lamps_b"], 2)

        # Release door to 10 mm (< 18 mm)
        state = self.logic.step(0.0, 0.010, 0.0, 0.0, 0.0)
        self.assertFalse(state["unit_b_updating"])
        self.assertEqual(state["unit_b_progress_steps"], 0)
        self.assertEqual(state["unit_b_version"], 0)
        self.assertEqual(state["version_lamp_b"], "grey")
        self.assertEqual(state["progress_lamps_b"], 0)

    def test_continuous_hold_completes_update_to_v1(self):
        # Start update on B (step 1)
        self.logic.step(0.0, 0.020, 0.0, -0.007, 0.0)
        # Hold door open for 79 more steps (total 80 steps = 4.0 s)
        for _ in range(79):
            state = self.logic.step(0.0, 0.020, 0.0, 0.0, 0.0)
        self.assertFalse(state["unit_b_updating"])
        self.assertEqual(state["unit_b_version"], 1)
        self.assertEqual(state["version_lamp_b"], "white")
        self.assertEqual(state["progress_lamps_b"], 4)

    def test_updating_active_unit_latches_outage(self):
        # Unit A is active. Try to update A.
        # Open door A and press UPDATE A
        state = self.logic.step(
            door_a_deflection=0.020,
            door_b_deflection=0.0,
            btn_update_a_z=-0.007,
            btn_update_b_z=0.0,
            btn_switch_z=0.0,
        )
        self.assertTrue(state["unit_a_updating"])
        self.assertTrue(state["outage"])  # Active unit went offline!
        self.assertEqual(state["load_lamp"], "red")

        # OUTAGE latches permanently even after update ends or door released
        state = self.logic.step(0.0, 0.0, 0.0, 0.0, 0.0)
        self.assertTrue(state["outage"])
        self.assertEqual(state["load_lamp"], "red")

    def test_switch_button_moves_load(self):
        self.assertEqual(self.logic.active_unit, "A")
        # Press SWITCH
        state = self.logic.step(0.0, 0.0, 0.0, 0.0, -0.007)
        self.assertEqual(state["active_unit"], "B")
        self.assertFalse(state["outage"])

        # Release and press SWITCH again
        self.logic.step(0.0, 0.0, 0.0, 0.0, 0.0)
        state = self.logic.step(0.0, 0.0, 0.0, 0.0, -0.007)
        self.assertEqual(state["active_unit"], "A")

    def test_switching_to_offline_unit_latches_outage(self):
        # Start update on B (B is offline)
        self.logic.step(0.0, 0.020, 0.0, -0.007, 0.0)
        self.assertTrue(self.logic.unit_b.updating)
        self.assertFalse(self.logic.outage)

        # Re-arm UPDATE button
        self.logic.step(0.0, 0.020, 0.0, 0.0, 0.0)

        # Press SWITCH while B is offline/updating -> OUTAGE!
        state = self.logic.step(0.0, 0.020, 0.0, 0.0, -0.007)
        self.assertTrue(state["outage"])
        self.assertEqual(state["load_lamp"], "red")

    def test_full_successful_rolling_update(self):
        # 1. Update standby unit B
        self.logic.step(0.0, 0.020, 0.0, -0.007, 0.0)
        for _ in range(80):
            self.logic.step(0.0, 0.020, 0.0, 0.0, 0.0)
        self.assertEqual(self.logic.unit_b.version, 1)
        self.assertFalse(self.logic.outage)

        # Release door B
        self.logic.step(0.0, 0.0, 0.0, 0.0, 0.0)

        # 2. Press SWITCH to make B active
        self.logic.step(0.0, 0.0, 0.0, 0.0, -0.007)
        self.assertEqual(self.logic.active_unit, "B")
        self.assertFalse(self.logic.outage)

        # Re-arm SWITCH
        self.logic.step(0.0, 0.0, 0.0, 0.0, 0.0)

        # 3. Update standby unit A
        self.logic.step(0.020, 0.0, -0.007, 0.0, 0.0)
        for _ in range(80):
            self.logic.step(0.020, 0.0, 0.0, 0.0, 0.0)
        self.assertEqual(self.logic.unit_a.version, 1)
        self.assertFalse(self.logic.outage)

        # Release door A
        state = self.logic.step(0.0, 0.0, 0.0, 0.0, 0.0)
        self.assertEqual(state["unit_a_version"], 1)
        self.assertEqual(state["unit_b_version"], 1)
        self.assertFalse(state["outage"])
        self.assertEqual(state["load_lamp"], "green")
        self.assertEqual(state["version_lamp_a"], "white")
        self.assertEqual(state["version_lamp_b"], "white")


if __name__ == "__main__":
    unittest.main()
