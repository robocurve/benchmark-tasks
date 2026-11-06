"""Boundary and property tests for rolling-firmware-update Scorer."""

import unittest
import sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import geometry as g
from scoring import Scorer

ZERO_POS = g.BASE_POS
ZERO_QUAT = (0.0, 0.0, 0.0, 1.0)
ZERO_VEL = (0.0, 0.0, 0.0)
NO_SLIDERS = {"door_a": 0.0, "door_b": 0.0, "btn_update_a": 0.0, "btn_update_b": 0.0, "btn_switch": 0.0}


class TestScoring(unittest.TestCase):
    def setUp(self):
        self.scorer = Scorer()

    def step(
        self,
        sliders=None,
        grippers_open=True,
        fixture_pos=ZERO_POS,
        fixture_quat=ZERO_QUAT,
        fixture_lin_vel=ZERO_VEL,
        fixture_ang_vel=ZERO_VEL,
    ):
        s = dict(NO_SLIDERS)
        if sliders:
            s.update(sliders)
        return self.scorer.update(
            fixture_pos=fixture_pos,
            fixture_quat=fixture_quat,
            fixture_lin_vel=fixture_lin_vel,
            fixture_ang_vel=fixture_ang_vel,
            fixture_sliders=s,
            grippers_open=grippers_open,
        )

    def test_start_scores_zero(self):
        res = self.step()
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)

    def test_level_025_door_open(self):
        # Deflection below 18 mm does not trigger 0.25 (unless button pressed)
        res = self.step(sliders={"door_a": -0.015})
        self.assertEqual(res.score, 0.0)

        # Deflection >= 18 mm triggers Level 0.25
        res = self.step(sliders={"door_a": -0.019})
        self.assertEqual(res.score, 0.25)
        self.assertFalse(res.success)

        # Releasing door maintains latched partial score
        res = self.step(sliders={"door_a": 0.0})
        self.assertEqual(res.score, 0.25)

    def test_level_025_button_press(self):
        # Press SWITCH button (-0.007 m)
        res = self.step(sliders={"btn_switch": -0.007})
        self.assertEqual(res.score, 0.25)

    def test_level_050_standby_unit_b_updated(self):
        # Open door B (22 mm) and press UPDATE B (step 1)
        self.step(sliders={"door_b": -0.022, "btn_update_b": -0.007})
        # Hold door open for steps 2..79
        for _ in range(78):
            res = self.step(sliders={"door_b": -0.022})
            self.assertEqual(res.score, 0.25)
        # 80th step completes update to v1!
        res = self.step(sliders={"door_b": -0.022})
        self.assertEqual(res.score, 0.50)

        # Release door B: score 0.50 remains latched
        res = self.step(sliders={"door_b": 0.0})
        self.assertEqual(res.score, 0.50)

    def test_level_075_switch_to_b(self):
        # Update B (80 steps)
        self.step(sliders={"door_b": -0.022, "btn_update_b": -0.007})
        for _ in range(79):
            self.step(sliders={"door_b": -0.022})
        self.step(sliders={"door_b": 0.0})
        self.assertEqual(self.scorer.highest_partial_tier, 0.50)

        # Press SWITCH
        res = self.step(sliders={"btn_switch": -0.007})
        self.assertEqual(res.score, 0.75)
        # Release switch: 0.75 remains latched
        res = self.step(sliders={"btn_switch": 0.0})
        self.assertEqual(res.score, 0.75)

    def test_order_agnostic_ladder_a_first(self):
        # Switch first so Unit B carries load and Unit A is standby
        self.step(sliders={"btn_switch": -0.007})
        self.step(sliders={"btn_switch": 0.0})
        self.assertEqual(self.scorer.highest_partial_tier, 0.25)

        # Update Unit A (standby)
        self.step(sliders={"door_a": -0.022, "btn_update_a": -0.007})
        for _ in range(79):
            self.step(sliders={"door_a": -0.022})
        self.step(sliders={"door_a": 0.0})
        # Either unit v1 with no outage -> 0.50
        self.assertEqual(self.scorer.highest_partial_tier, 0.50)

        # Switch load back to Unit A (Unit A is v1 AND active) -> 0.75
        self.step(sliders={"btn_switch": -0.007})
        self.step(sliders={"btn_switch": 0.0})
        self.assertEqual(self.scorer.highest_partial_tier, 0.75)

    def test_level_100_full_sequence_with_hold(self):
        # 1. Update B
        self.step(sliders={"door_b": -0.022, "btn_update_b": -0.007})
        for _ in range(79):
            self.step(sliders={"door_b": -0.022})
        self.step(sliders={"door_b": 0.0})

        # 2. Switch
        self.step(sliders={"btn_switch": -0.007})
        self.step(sliders={"btn_switch": 0.0})

        # 3. Update A
        self.step(sliders={"door_a": -0.022, "btn_update_a": -0.007})
        for _ in range(79):
            self.step(sliders={"door_a": -0.022})
        # Release door A with grippers closed so hold timer starts from 0
        self.step(sliders={"door_a": 0.0}, grippers_open=False)
        self.assertEqual(self.scorer.highest_partial_tier, 0.75)
        self.assertEqual(self.scorer.consecutive_success_steps, 0)

        # Both v1 now. Doors closed (< 2 mm). Still.
        # Now open grippers: hold for 19 steps -> score remains 0.75
        for _ in range(19):
            res = self.step(grippers_open=True)
            self.assertEqual(res.score, 0.75)
            self.assertFalse(res.success)

        # 20th step achieves 1.00!
        res = self.step(grippers_open=True)
        self.assertEqual(res.score, 1.00)
        self.assertTrue(res.success)

    def test_level_100_fails_if_door_not_released(self):
        # Perform full update but leave door A open 5 mm (>= 2 mm)
        self.step(sliders={"door_b": -0.022, "btn_update_b": -0.007})
        for _ in range(80):
            self.step(sliders={"door_b": -0.022})
        self.step(sliders={"door_b": 0.0})
        self.step(sliders={"btn_switch": -0.007})
        self.step(sliders={"btn_switch": 0.0})
        self.step(sliders={"door_a": -0.022, "btn_update_a": -0.007})
        for _ in range(80):
            self.step(sliders={"door_a": -0.022})

        # Door A held at 5 mm deflection
        for _ in range(30):
            res = self.step(sliders={"door_a": -0.005}, grippers_open=True)
        self.assertEqual(res.score, 0.75)
        self.assertFalse(res.success)

    def test_level_100_fails_if_gripper_closed(self):
        # Full update but gripper stays closed
        self.step(sliders={"door_b": -0.022, "btn_update_b": -0.007})
        for _ in range(80):
            self.step(sliders={"door_b": -0.022})
        self.step(sliders={"door_b": 0.0})
        self.step(sliders={"btn_switch": -0.007})
        self.step(sliders={"btn_switch": 0.0})
        self.step(sliders={"door_a": -0.022, "btn_update_a": -0.007})
        for _ in range(80):
            self.step(sliders={"door_a": -0.022})
        self.step(sliders={"door_a": 0.0})

        for _ in range(30):
            res = self.step(grippers_open=False)
        self.assertEqual(res.score, 0.75)
        self.assertFalse(res.success)

    def test_zero_gate_outage(self):
        # Update A while active -> OUTAGE (door open >= 18 mm)
        res = self.step(sliders={"door_a": -0.022, "btn_update_a": -0.007})
        self.assertEqual(res.score, 0.0)
        self.assertTrue(res.details.get("zero_gate"))
        self.assertIn("OUTAGE", res.details.get("reason", ""))

        # Score remains 0.0 forever
        res = self.step()
        self.assertEqual(res.score, 0.0)

    def test_zero_gate_fixture_displacement(self):
        # Shift fixture by 6 mm > 5 mm
        shifted_pos = (g.BASE_CX + 0.006, g.BASE_CY, g.BASE_CZ)
        res = self.step(fixture_pos=shifted_pos)
        self.assertEqual(res.score, 0.0)
        self.assertTrue(res.details.get("zero_gate"))
        self.assertIn("Fixture moved", res.details.get("reason", ""))

    def test_zero_gate_fixture_tilt(self):
        # Tilt fixture by 6 deg > 5 deg (rotation around x)
        angle = np.radians(6.0)
        quat = (np.sin(angle / 2), 0.0, 0.0, np.cos(angle / 2))
        res = self.step(fixture_quat=quat)
        self.assertEqual(res.score, 0.0)
        self.assertTrue(res.details.get("zero_gate"))
        self.assertIn("Fixture tilted", res.details.get("reason", ""))


if __name__ == "__main__":
    unittest.main()
