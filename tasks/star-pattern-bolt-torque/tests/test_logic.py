"""Unit tests for Task 1 hidden-state logic: star pattern rules, level constraints, hysteresis, and over-torque."""

from __future__ import annotations

import unittest
from logic import StarPatternLogic, PRESS_THRESHOLD, RELEASE_THRESHOLD, CORNERS, DIAGONAL_PARTNER


class TestStarPatternLogic(unittest.TestCase):
    def setUp(self) -> None:
        self.logic = StarPatternLogic()

    def test_initial_state(self) -> None:
        self.assertEqual(self.logic.stages, {c: 0 for c in CORNERS})
        self.assertFalse(self.logic.warp)
        self.assertIsNone(self.logic.warp_reason)
        self.assertEqual(self.logic.status_lamp, "off")
        self.assertEqual(self.logic.completed_passes, 0)
        self.assertEqual(self.logic.current_pass, [])

    def test_single_pass_star_order(self) -> None:
        # Pass 1: L_near -> R_far -> R_near -> L_far
        self.logic.register_press("L_near")
        self.assertEqual(self.logic.stages["L_near"], 1)
        self.assertFalse(self.logic.warp)
        self.assertEqual(self.logic.current_pass, ["L_near"])
        self.assertEqual(self.logic.status_lamp, "off")

        self.logic.register_press("R_far")
        self.assertEqual(self.logic.stages["R_far"], 1)
        self.assertFalse(self.logic.warp)
        self.assertEqual(self.logic.current_pass, ["L_near", "R_far"])

        self.logic.register_press("R_near")
        self.assertEqual(self.logic.stages["R_near"], 1)
        self.assertFalse(self.logic.warp)
        self.assertEqual(self.logic.current_pass, ["L_near", "R_far", "R_near"])

        self.logic.register_press("L_far")
        self.assertEqual(self.logic.stages["L_far"], 1)
        self.assertFalse(self.logic.warp)
        self.assertEqual(self.logic.completed_passes, 1)
        self.assertEqual(self.logic.current_pass, [])
        self.assertEqual(self.logic.status_lamp, "off")

    def test_full_three_passes_green_lamp(self) -> None:
        seq = ["L_near", "R_far", "R_near", "L_far"] * 3
        for idx, corner in enumerate(seq):
            self.logic.register_press(corner)
            self.assertFalse(self.logic.warp, f"Failed at press {idx+1}: {corner}")

        self.assertEqual(self.logic.completed_passes, 3)
        self.assertEqual(self.logic.stages, {c: 3 for c in CORNERS})
        self.assertFalse(self.logic.warp)
        self.assertEqual(self.logic.status_lamp, "green")

    def test_alternative_valid_star_pass(self) -> None:
        # Starting with L_far -> R_near -> L_near -> R_far
        seq = ["L_far", "R_near", "L_near", "R_far"]
        for c in seq:
            self.logic.register_press(c)
        self.assertFalse(self.logic.warp)
        self.assertEqual(self.logic.completed_passes, 1)

    def test_warp_on_adjacent_press(self) -> None:
        # p1: L_near, then p2: L_far (adjacent, not diagonal)
        self.logic.register_press("L_near")
        self.logic.register_press("L_far")
        self.assertTrue(self.logic.warp)
        self.assertIn("STAR_ORDER", self.logic.warp_reason)
        self.assertEqual(self.logic.status_lamp, "red")

    def test_warp_on_front_adjacent_press(self) -> None:
        # p1: L_near, then p2: R_near (adjacent across y, not diagonal)
        self.logic.register_press("L_near")
        self.logic.register_press("R_near")
        self.assertTrue(self.logic.warp)
        self.assertIn("STAR_ORDER", self.logic.warp_reason)
        self.assertEqual(self.logic.status_lamp, "red")

    def test_warp_on_level_violation(self) -> None:
        # If someone presses L_near, R_far (diagonal pair, stages 1, 1),
        # then in next pass presses L_near again before R_near / L_far get stage 1:
        # stage L_near = 2 while R_near = 0 -> max - min = 2 - 0 = 2 > 1
        # (Note: p3 must be new corner in pass, but if 2 presses occurred and pass was aborted or if we test register_press directly)
        self.logic.stages["L_near"] = 1
        self.logic.stages["R_far"] = 1
        self.logic.stages["R_near"] = 0
        self.logic.stages["L_far"] = 0
        self.logic.current_pass = []
        # Press L_near -> stage 2, min is 0 -> diff 2 > 1 -> WARP by LEVEL
        self.logic.register_press("L_near")
        self.assertTrue(self.logic.warp)
        self.assertIn("LEVEL", self.logic.warp_reason)
        self.assertEqual(self.logic.status_lamp, "red")

    def test_warp_on_duplicate_corner_in_pass(self) -> None:
        self.logic.register_press("L_near")
        self.logic.register_press("R_far")
        # Try to press L_near again as p3
        self.logic.register_press("L_near")
        self.assertTrue(self.logic.warp)
        self.assertIn("already pressed in pass", self.logic.warp_reason)
        self.assertEqual(self.logic.status_lamp, "red")

    def test_warp_on_p4_not_diagonal(self) -> None:
        self.logic.register_press("L_near")
        self.logic.register_press("R_far")
        self.logic.register_press("R_near")
        # p4 must be L_far. Try pressing R_far (or another)
        self.logic.register_press("R_far")
        self.assertTrue(self.logic.warp)
        self.assertEqual(self.logic.status_lamp, "red")

    def test_warp_on_over_torque(self) -> None:
        # Complete all 3 passes
        seq = ["L_near", "R_far", "R_near", "L_far"] * 3
        for c in seq:
            self.logic.register_press(c)
        self.assertEqual(self.logic.status_lamp, "green")
        self.assertFalse(self.logic.warp)

        # 13th press on any bolt -> OVER-TORQUE
        self.logic.register_press("L_near")
        self.assertTrue(self.logic.warp)
        self.assertIn("OVER_TORQUE", self.logic.warp_reason)
        self.assertEqual(self.logic.status_lamp, "red")

    def test_hysteresis_and_update(self) -> None:
        # Displacement <= -0.0064 triggers rising edge
        sliders = {f"btn_{c}": 0.0 for c in CORNERS}
        sliders["btn_L_near"] = -0.0065
        state = self.logic.update(sliders)
        self.assertEqual(state.stages["L_near"], 1)
        self.assertEqual(state.last_press, "L_near")

        # Holding at -0.0065 does NOT trigger again
        state = self.logic.update(sliders)
        self.assertEqual(state.stages["L_near"], 1)
        self.assertIsNone(state.last_press)

        # Releasing partially to -0.0040 (still below -0.0035) does NOT re-arm
        sliders["btn_L_near"] = -0.0040
        self.logic.update(sliders)
        sliders["btn_L_near"] = -0.0065
        state = self.logic.update(sliders)
        self.assertEqual(state.stages["L_near"], 1)

        # Releasing to -0.0020 (>= -0.0035) re-arms
        sliders["btn_L_near"] = -0.0020
        self.logic.update(sliders)
        # Next press on diagonal partner R_far
        sliders["btn_L_near"] = 0.0
        sliders["btn_R_far"] = -0.0070
        state = self.logic.update(sliders)
        self.assertEqual(state.stages["R_far"], 1)
        self.assertEqual(state.last_press, "R_far")
        self.assertFalse(state.warp)

    def test_reset(self) -> None:
        self.logic.register_press("L_near")
        self.logic.register_press("L_far")  # WARP
        self.assertTrue(self.logic.warp)
        self.logic.reset()
        self.assertFalse(self.logic.warp)
        self.assertEqual(self.logic.stages, {c: 0 for c in CORNERS})
        self.assertEqual(self.logic.status_lamp, "off")


if __name__ == "__main__":
    unittest.main()
