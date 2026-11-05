"""Unit tests for Task 1 scorer: partial credit ladder, hold timing, zero-gates, and fuzz robustness."""

from __future__ import annotations

import math
import unittest
import numpy as np

import geometry as g
from scoring import (
    Scorer,
    FIXTURE_MOVE_LIMIT,
    FIXTURE_TILT_LIMIT,
    SUCCESS_HOLD_STEPS,
    SCORE_NONE,
    SCORE_TOUCHED,
    SCORE_PASS1,
    SCORE_PASS2,
    SCORE_SUCCESS,
)


def rot_z_quat(deg: float) -> list[float]:
    rad = math.radians(deg)
    return [0.0, 0.0, math.sin(rad / 2), math.cos(rad / 2)]


class TestTask1Scoring(unittest.TestCase):
    def setUp(self) -> None:
        self.scorer = Scorer()
        self.base_pos = np.array(g.BASE_POS, dtype=float)
        self.zero_sliders = {f"btn_{c}": 0.0 for c in g.CORNERS}

    def test_initial_zero_gate_not_tripped(self) -> None:
        res = self.scorer.update(
            fixture_pos=self.base_pos,
            fixture_quat=[0.0, 0.0, 0.0, 1.0],
            fixture_sliders=self.zero_sliders,
            fixture_lin_vel=[0.0, 0.0, 0.0],
            fixture_ang_vel=[0.0, 0.0, 0.0],
            grippers_open=True,
        )
        self.assertEqual(res.score, SCORE_NONE)
        self.assertFalse(res.success)
        self.assertFalse(res.details.get("failed", False))

    def test_button_touched_reaches_quarter_score(self) -> None:
        # Slight button press to -2.5 mm (< -2.0 mm)
        sliders = dict(self.zero_sliders)
        sliders["btn_L_near"] = -0.0025
        res = self.scorer.update(
            fixture_pos=self.base_pos,
            fixture_quat=[0.0, 0.0, 0.0, 1.0],
            fixture_sliders=sliders,
            grippers_open=True,
        )
        self.assertEqual(res.score, SCORE_TOUCHED)
        self.assertFalse(res.success)

        # Release button: score latches at 0.25
        res2 = self.scorer.update(
            fixture_pos=self.base_pos,
            fixture_quat=[0.0, 0.0, 0.0, 1.0],
            fixture_sliders=self.zero_sliders,
            grippers_open=True,
        )
        self.assertEqual(res2.score, SCORE_TOUCHED)

    def _execute_counted_press(self, corner: str) -> None:
        """Simulate a full press and release on corner."""
        press_sliders = dict(self.zero_sliders)
        press_sliders[f"btn_{corner}"] = -0.0070
        self.scorer.update(
            fixture_pos=self.base_pos,
            fixture_quat=[0.0, 0.0, 0.0, 1.0],
            fixture_sliders=press_sliders,
            grippers_open=False,
        )
        self.scorer.update(
            fixture_pos=self.base_pos,
            fixture_quat=[0.0, 0.0, 0.0, 1.0],
            fixture_sliders=self.zero_sliders,
            grippers_open=False,
        )

    def test_pass_1_credit(self) -> None:
        # Execute pass 1: L_near -> R_far -> R_near -> L_far
        for c in ["L_near", "R_far", "R_near", "L_far"]:
            self._execute_counted_press(c)

        res = self.scorer.update(
            fixture_pos=self.base_pos,
            fixture_quat=[0.0, 0.0, 0.0, 1.0],
            fixture_sliders=self.zero_sliders,
            grippers_open=False,
        )
        self.assertEqual(res.score, SCORE_PASS1)
        self.assertFalse(res.success)

    def test_pass_2_credit(self) -> None:
        pass1 = ["L_near", "R_far", "R_near", "L_far"]
        for c in pass1:
            self._execute_counted_press(c)
        for c in pass1:
            self._execute_counted_press(c)

        res = self.scorer.update(
            fixture_pos=self.base_pos,
            fixture_quat=[0.0, 0.0, 0.0, 1.0],
            fixture_sliders=self.zero_sliders,
            grippers_open=False,
        )
        self.assertEqual(res.score, SCORE_PASS2)
        self.assertFalse(res.success)

    def test_full_success_hold_timing(self) -> None:
        pass_seq = ["L_near", "R_far", "R_near", "L_far"]
        for _ in range(3):
            for c in pass_seq:
                self._execute_counted_press(c)

        # All 3 passes complete, but grippers closed: score is 0.75
        res = self.scorer.update(
            fixture_pos=self.base_pos,
            fixture_quat=[0.0, 0.0, 0.0, 1.0],
            fixture_sliders=self.zero_sliders,
            grippers_open=False,
        )
        self.assertEqual(res.score, SCORE_PASS2)
        self.assertFalse(res.success)

        # Now open grippers and keep parts still for 19 steps
        for step in range(19):
            res = self.scorer.update(
                fixture_pos=self.base_pos,
                fixture_quat=[0.0, 0.0, 0.0, 1.0],
                fixture_sliders=self.zero_sliders,
                fixture_lin_vel=[0.0, 0.0, 0.0],
                fixture_ang_vel=[0.0, 0.0, 0.0],
                grippers_open=True,
            )
            self.assertEqual(res.score, SCORE_PASS2, f"Premature 1.0 at step {step+1}")
            self.assertFalse(res.success)

        # 20th step (1.0 s held) -> reaches 1.0 and success
        res = self.scorer.update(
            fixture_pos=self.base_pos,
            fixture_quat=[0.0, 0.0, 0.0, 1.0],
            fixture_sliders=self.zero_sliders,
            fixture_lin_vel=[0.0, 0.0, 0.0],
            fixture_ang_vel=[0.0, 0.0, 0.0],
            grippers_open=True,
        )
        self.assertEqual(res.score, SCORE_SUCCESS)
        self.assertTrue(res.success)

    def test_zero_gate_warp_drops_score_to_zero(self) -> None:
        # Reach 0.50 first
        for c in ["L_near", "R_far", "R_near", "L_far"]:
            self._execute_counted_press(c)
        res = self.scorer.update(
            fixture_pos=self.base_pos,
            fixture_quat=[0.0, 0.0, 0.0, 1.0],
            fixture_sliders=self.zero_sliders,
            grippers_open=False,
        )
        self.assertEqual(res.score, SCORE_PASS1)

        # Trigger WARP by pressing non-diagonal adjacent corner in pass 2 (e.g. L_near then L_far)
        self._execute_counted_press("L_near")
        self._execute_counted_press("L_far")  # WARP!

        res = self.scorer.update(
            fixture_pos=self.base_pos,
            fixture_quat=[0.0, 0.0, 0.0, 1.0],
            fixture_sliders=self.zero_sliders,
            grippers_open=True,
        )
        self.assertEqual(res.score, SCORE_NONE)
        self.assertFalse(res.success)
        self.assertTrue(res.details.get("failed", False))

    def test_zero_gate_fixture_displacement(self) -> None:
        self.scorer.update(self.base_pos, [0, 0, 0, 1], self.zero_sliders)

        # Boundary: 4.9 mm shift (ok)
        res_ok = self.scorer.update(
            self.base_pos + [0.0049, 0.0, 0.0],
            [0, 0, 0, 1],
            self.zero_sliders,
        )
        self.assertFalse(res_ok.details.get("failed", False))

        # Boundary: 5.1 mm shift (trips zero-gate)
        res_fail = self.scorer.update(
            self.base_pos + [0.0051, 0.0, 0.0],
            [0, 0, 0, 1],
            self.zero_sliders,
        )
        self.assertEqual(res_fail.score, SCORE_NONE)
        self.assertTrue(res_fail.details.get("failed", False))

    def test_zero_gate_fixture_tilt(self) -> None:
        self.scorer.update(self.base_pos, [0, 0, 0, 1], self.zero_sliders)

        # 4.5 deg tilt (ok)
        res_ok = self.scorer.update(
            self.base_pos,
            rot_z_quat(4.5),
            self.zero_sliders,
        )
        self.assertFalse(res_ok.details.get("failed", False))

        # 5.5 deg tilt (fails)
        res_fail = self.scorer.update(
            self.base_pos,
            rot_z_quat(5.5),
            self.zero_sliders,
        )
        self.assertEqual(res_fail.score, SCORE_NONE)
        self.assertTrue(res_fail.details.get("failed", False))

    def test_fuzz_inputs_no_unhandled_crashes(self) -> None:
        rng = np.random.default_rng(42)
        scorer = Scorer()
        for _ in range(100):
            pos = self.base_pos + rng.uniform(-0.01, 0.01, 3)
            quat = rng.uniform(-1, 1, 4)
            quat /= np.linalg.norm(quat)
            sliders = {f"btn_{c}": float(rng.uniform(-0.009, 0.0)) for c in g.CORNERS}
            lin_vel = rng.uniform(-0.05, 0.05, 3)
            ang_vel = rng.uniform(-0.5, 0.5, 3)
            g_open = bool(rng.choice([True, False]))

            res = scorer.update(pos, quat, sliders, lin_vel, ang_vel, g_open)
            self.assertIn(res.score, (0.0, 0.25, 0.50, 0.75, 1.0))
            self.assertIsInstance(res.success, bool)


if __name__ == "__main__":
    unittest.main()
