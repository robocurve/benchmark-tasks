"""Unit tests for logic.py in buffer-socket-module-permutation."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

from logic import (
    PRESS_THRESHOLD,
    RELEASE_THRESHOLD,
    PermutationLogic,
)


class TestPermutationLogic(unittest.TestCase):
    def setUp(self):
        self.logic = PermutationLogic()

    def test_initial_misracked_state(self):
        # Initial: green in 0, red in 1, blue in 2
        initial_seated = {
            "module_green": 0,
            "module_red": 1,
            "module_blue": 2,
        }
        state = self.logic.update(initial_seated, commit_slider_q=0.0)
        self.assertFalse(state.commit_held)
        self.assertEqual(state.lamp_state, "red")
        self.assertFalse(state.all_three_matching)
        self.assertEqual(state.matching_swapped_count, 0)
        self.assertEqual(state.matching_total_count, 1)  # blue is matching
        self.assertFalse(state.commit_pressed_valid)

    def test_move_1_green_to_buffer(self):
        # Green in buffer (socket 3), red in 1, blue in 2
        seated = {
            "module_green": 3,
            "module_red": 1,
            "module_blue": 2,
        }
        state = self.logic.update(seated, commit_slider_q=0.0)
        self.assertEqual(state.lamp_state, "red")
        self.assertFalse(state.all_three_matching)
        self.assertEqual(state.matching_swapped_count, 0)
        self.assertEqual(state.matching_total_count, 1)

    def test_move_2_red_to_socket_1(self):
        # Red in socket 0 (matching!), green in 3, blue in 2
        seated = {
            "module_green": 3,
            "module_red": 0,
            "module_blue": 2,
        }
        state = self.logic.update(seated, commit_slider_q=0.0)
        self.assertEqual(state.lamp_state, "red")
        self.assertFalse(state.all_three_matching)
        self.assertEqual(state.matching_swapped_count, 1)
        self.assertEqual(state.matching_total_count, 2)

    def test_move_3_green_to_socket_2(self):
        # All three matching: red in 0, green in 1, blue in 2
        seated = {
            "module_green": 1,
            "module_red": 0,
            "module_blue": 2,
        }
        state = self.logic.update(seated, commit_slider_q=0.0)
        self.assertEqual(state.lamp_state, "green")
        self.assertTrue(state.all_three_matching)
        self.assertEqual(state.matching_swapped_count, 2)
        self.assertEqual(state.matching_total_count, 3)

    def test_lamp_red_while_any_displaced_or_held(self):
        seated = {
            "module_green": 1,
            "module_red": 0,
            "module_blue": 2,
        }
        # Even if seated, if any_displaced is True (moving/held), lamp stays red
        state = self.logic.update(seated, commit_slider_q=0.0, any_displaced=True)
        self.assertEqual(state.lamp_state, "red")
        self.assertFalse(state.all_three_matching)

    def test_commit_button_premature_press_rejected(self):
        # Initial mis-racked state: press commit button
        initial_seated = {
            "module_green": 0,
            "module_red": 1,
            "module_blue": 2,
        }
        state = self.logic.update(initial_seated, commit_slider_q=-0.007)
        self.assertTrue(state.commit_held)
        self.assertFalse(state.commit_pressed_valid, "Premature commit press while lamp is red must not validate")

    def test_commit_button_valid_press_latches(self):
        all_matching = {
            "module_green": 1,
            "module_red": 0,
            "module_blue": 2,
        }
        # First step: lamp turns green
        state = self.logic.update(all_matching, commit_slider_q=0.0)
        self.assertEqual(state.lamp_state, "green")
        self.assertFalse(state.commit_pressed_valid)

        # Second step: press commit while green
        state = self.logic.update(all_matching, commit_slider_q=-0.007)
        self.assertTrue(state.commit_held)
        self.assertTrue(state.commit_pressed_valid)

        # Third step: release commit button; valid status remains latched
        state = self.logic.update(all_matching, commit_slider_q=0.0)
        self.assertFalse(state.commit_held)
        self.assertTrue(state.commit_pressed_valid)

    def test_button_hysteresis(self):
        seated = {"module_green": 0, "module_red": 1, "module_blue": 2}
        # Press past -6.4 mm
        s1 = self.logic.update(seated, commit_slider_q=-0.0065)
        self.assertTrue(s1.commit_held)

        # In-between -6.4 mm and -3.5 mm -> stays held
        s2 = self.logic.update(seated, commit_slider_q=-0.0050)
        self.assertTrue(s2.commit_held)

        # Released past -3.5 mm
        s3 = self.logic.update(seated, commit_slider_q=-0.0030)
        self.assertFalse(s3.commit_held)


if __name__ == "__main__":
    unittest.main()
