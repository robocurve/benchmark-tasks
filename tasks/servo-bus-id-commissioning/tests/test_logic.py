"""Unit tests for the servo bus commissioning hidden-state logic."""

from __future__ import annotations

import unittest

from logic import (
    PRESS_THRESHOLD,
    RELEASE_THRESHOLD,
    SERVOS,
    CommissioningLogic,
)


class TestCommissioningLogic(unittest.TestCase):
    def setUp(self) -> None:
        self.logic = CommissioningLogic()

    def test_initial_state_with_three_seated(self) -> None:
        state = self.logic.update({"red": True, "green": True, "blue": True}, 0.0, 0.0)
        self.assertEqual(state.ids, {"red": None, "green": None, "blue": None})
        self.assertEqual(state.counter, 0)
        self.assertEqual(state.bus_lamp, "red")
        self.assertEqual(state.readout, {"red": None, "green": None, "blue": None})
        self.assertFalse(state.assign_event)
        self.assertFalse(state.reset_event)

    def test_assign_blocked_with_multiple_unassigned_seated(self) -> None:
        # Step 1: 3 unassigned seated, press ASSIGN
        state = self.logic.update({"red": True, "green": True, "blue": True}, PRESS_THRESHOLD, 0.0)
        self.assertTrue(state.assign_event)
        self.assertEqual(state.ids, {"red": None, "green": None, "blue": None})
        self.assertEqual(state.counter, 0)
        self.assertEqual(state.bus_lamp, "red")

        # Step 2: 2 unassigned seated, release ASSIGN
        state = self.logic.update({"red": True, "green": True, "blue": False}, RELEASE_THRESHOLD, 0.0)
        self.assertEqual(state.bus_lamp, "red")

        # Step 3: press ASSIGN with 2 unassigned seated -> still blocked
        state = self.logic.update({"red": True, "green": True, "blue": False}, PRESS_THRESHOLD, 0.0)
        self.assertTrue(state.assign_event)
        self.assertEqual(state.ids, {"red": None, "green": None, "blue": None})
        self.assertEqual(state.counter, 0)

    def test_single_unassigned_seated_turns_lamp_green(self) -> None:
        # Only red seated
        state = self.logic.update({"red": True, "green": False, "blue": False}, 0.0, 0.0)
        self.assertEqual(state.bus_lamp, "green")
        self.assertEqual(state.ids["red"], None)

    def test_sequential_commissioning_happy_path(self) -> None:
        # Step 1: only red seated
        self.logic.update({"red": True, "green": False, "blue": False}, 0.0, 0.0)

        # Step 2: press ASSIGN -> red gets 1
        state = self.logic.update({"red": True, "green": False, "blue": False}, PRESS_THRESHOLD, 0.0)
        self.assertEqual(state.ids["red"], 1)
        self.assertEqual(state.counter, 1)
        self.assertEqual(state.bus_lamp, "green")

        # Step 3: release ASSIGN
        self.logic.update({"red": True, "green": False, "blue": False}, RELEASE_THRESHOLD, 0.0)

        # Step 4: plug green in -> red is numbered, green is unassigned (1 unassigned on bus)
        state = self.logic.update({"red": True, "green": True, "blue": False}, 0.0, 0.0)
        self.assertEqual(state.bus_lamp, "green")

        # Step 5: press ASSIGN -> green gets 2
        state = self.logic.update({"red": True, "green": True, "blue": False}, PRESS_THRESHOLD, 0.0)
        self.assertEqual(state.ids["green"], 2)
        self.assertEqual(state.counter, 2)
        self.assertEqual(state.bus_lamp, "green")

        # Step 6: release ASSIGN
        self.logic.update({"red": True, "green": True, "blue": False}, RELEASE_THRESHOLD, 0.0)

        # Step 7: plug blue in -> red=1, green=2, blue unassigned
        state = self.logic.update({"red": True, "green": True, "blue": True}, 0.0, 0.0)
        self.assertEqual(state.bus_lamp, "green")

        # Step 8: press ASSIGN -> blue gets 3
        state = self.logic.update({"red": True, "green": True, "blue": True}, PRESS_THRESHOLD, 0.0)
        self.assertEqual(state.ids, {"red": 1, "green": 2, "blue": 3})
        self.assertEqual(state.counter, 3)
        self.assertEqual(state.bus_lamp, "green")

    def test_assign_button_hysteresis(self) -> None:
        # Only red seated
        self.logic.update({"red": True, "green": False, "blue": False}, 0.0, 0.0)

        # Step 1: Press
        state1 = self.logic.update({"red": True, "green": False, "blue": False}, PRESS_THRESHOLD - 0.001, 0.0)
        self.assertTrue(state1.assign_event)
        self.assertEqual(state1.ids["red"], 1)

        # Step 2: Still held down -> no second event
        state2 = self.logic.update({"red": True, "green": False, "blue": False}, PRESS_THRESHOLD - 0.001, 0.0)
        self.assertFalse(state2.assign_event)
        self.assertEqual(state2.counter, 1)

        # Step 3: Partially released but not above RELEASE_THRESHOLD
        state3 = self.logic.update({"red": True, "green": False, "blue": False}, -0.004, 0.0)
        self.assertFalse(state3.assign_event)

        # Step 4: Fully released
        state4 = self.logic.update({"red": True, "green": False, "blue": False}, 0.0, 0.0)
        self.assertFalse(state4.assign_event)

        # Step 5: Press again -> now re-armed, but no unassigned servos, so counter stays 1
        state5 = self.logic.update({"red": True, "green": False, "blue": False}, PRESS_THRESHOLD, 0.0)
        self.assertTrue(state5.assign_event)
        self.assertEqual(state5.counter, 1)

    def test_reset_clears_all_and_sets_lamp_red_if_plugged(self) -> None:
        # Commission red=1, green=2
        self.logic.update({"red": True, "green": False, "blue": False}, PRESS_THRESHOLD, 0.0)
        self.logic.update({"red": True, "green": False, "blue": False}, RELEASE_THRESHOLD, 0.0)
        self.logic.update({"red": True, "green": True, "blue": False}, PRESS_THRESHOLD, 0.0)
        self.logic.update({"red": True, "green": True, "blue": True}, RELEASE_THRESHOLD, 0.0)

        # Press RESET while all three are plugged in
        state = self.logic.update({"red": True, "green": True, "blue": True}, 0.0, PRESS_THRESHOLD)
        self.assertTrue(state.reset_event)
        self.assertEqual(state.ids, {"red": None, "green": None, "blue": None})
        self.assertEqual(state.counter, 0)
        self.assertEqual(state.bus_lamp, "red")  # 3 unassigned now seated!


if __name__ == "__main__":
    unittest.main()
