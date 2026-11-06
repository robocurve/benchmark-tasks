"""Unit tests for Task G scoring: exact ladder threshold boundaries, hold timing, zero-gates, and fuzz testing."""

from __future__ import annotations

import unittest
from pathlib import Path
import sys
import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import geometry as g
import logic
from scoring import Scorer


class TestTaskGScoring(unittest.TestCase):
    def setUp(self):
        self.scorer = Scorer(hold_s=1.0, hz=20.0)
        self.fix_pos = np.array(g.BASE_POS)
        self.btn_rest = {"read_btn": 0.0}
        self.quat = (0.0, 0.0, 0.0, 1.0)
        self.still_vel = (0.0, 0.0, 0.0)
        self.moving_vel = (0.010, 0.0, 0.0)  # 10 mm/s > 5 mm/s

    def _knob_tuple(self, x_mm: float, track: str, lin_vel=(0.0, 0.0, 0.0), dy: float = 0.0, dz: float = 0.0):
        cx = g.knob_cx_from_setting_mm(x_mm)
        cy = g.TRACK_Y[track] + dy
        cz = g.KNOB_CZ + dz
        return ((cx, cy, cz), self.quat, lin_vel, (0.0, 0.0, 0.0))

    def test_start_state_scores_zero(self):
        kl = self._knob_tuple(30.0, "left")
        kr = self._knob_tuple(30.0, "right")
        res = self.scorer.update(self.fix_pos, self.btn_rest, kl, kr, grippers_open=True)
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)
        self.assertFalse(res.details["zero_gate"])

    def test_threshold_0_25_knob_movement(self):
        # 1. Moved 2.9 mm (< 3.0 mm) -> still 0.0
        s = Scorer()
        kl = self._knob_tuple(30.0 + 2.9, "left")
        kr = self._knob_tuple(30.0, "right")
        res = s.update(self.fix_pos, self.btn_rest, kl, kr, grippers_open=True)
        self.assertEqual(res.score, 0.0)

        # 2. Moved exactly 3.0 mm (== 3.0 mm) -> 0.25
        s = Scorer()
        kl = self._knob_tuple(30.0 + 3.0, "left")
        res = s.update(self.fix_pos, self.btn_rest, kl, kr, grippers_open=True)
        self.assertEqual(res.score, 0.25)

        # 3. Moved 3.1 mm (> 3.0 mm) -> 0.25
        s = Scorer()
        kl = self._knob_tuple(30.0 + 3.1, "left")
        res = s.update(self.fix_pos, self.btn_rest, kl, kr, grippers_open=True)
        self.assertEqual(res.score, 0.25)

        # 4. Right knob moved 3.0 mm -> 0.25
        s = Scorer()
        kl = self._knob_tuple(30.0, "left")
        kr = self._knob_tuple(30.0 + 3.0, "right")
        res = s.update(self.fix_pos, self.btn_rest, kl, kr, grippers_open=True)
        self.assertEqual(res.score, 0.25)

        # 5. Knobs untouched at 30 mm, but READ button pressed -> 0.25
        s = Scorer()
        kl = self._knob_tuple(30.0, "left")
        kr = self._knob_tuple(30.0, "right")
        res = s.update(self.fix_pos, {"read_btn": -0.007}, kl, kr, grippers_open=True)
        self.assertEqual(res.score, 0.25)

    def test_threshold_0_50_boundary(self):
        s = Scorer()

        # Direct evaluation of settings giving r = 0.499 vs 0.501
        def make_pair(target_r):
            for test_x in np.linspace(25, 80, 5501):
                if logic.reading(test_x, 25.0) >= target_r:
                    return test_x, 25.0
            return 50.0, 25.0

        x_just_below, y_val = make_pair(0.499)
        self.assertLess(logic.reading(x_just_below, y_val), 0.50)
        res = s.update(self.fix_pos, self.btn_rest, self._knob_tuple(x_just_below, "left"), self._knob_tuple(y_val, "right"), grippers_open=True)
        self.assertEqual(res.score, 0.25)

        x_at_or_above, y_val = make_pair(0.500)
        self.assertGreaterEqual(logic.reading(x_at_or_above, y_val), 0.50)
        res = s.update(self.fix_pos, self.btn_rest, self._knob_tuple(x_at_or_above, "left"), self._knob_tuple(y_val, "right"), grippers_open=True)
        self.assertEqual(res.score, 0.50)

    def test_no_read_ablation_caps_at_0_50(self):
        """Construct validity test: blind guess to peak without readings cannot score 0.75 or 1.0."""
        s = Scorer()
        kl = self._knob_tuple(80.0, "left")
        kr = self._knob_tuple(50.0, "right")
        # Knobs at exact peak (r = 1.0), but 0 READ presses
        res = s.update(self.fix_pos, self.btn_rest, kl, kr, grippers_open=True)
        self.assertEqual(res.score, 0.50)
        self.assertFalse(res.success)
        self.assertFalse(res.details["has_distinct_readings"])

    def test_threshold_0_75_and_budget_cap(self):
        kl_probe1 = self._knob_tuple(30.0, "left")
        kr_probe1 = self._knob_tuple(30.0, "right")
        kl_peak = self._knob_tuple(65.0, "left")
        kr_peak = self._knob_tuple(35.0, "right")

        # Case A: 8 presses within budget at distinct settings -> scores 0.75
        s = Scorer()
        # Reading 1 at (30, 30)
        s.update(self.fix_pos, {"read_btn": -0.007}, kl_probe1, kr_probe1, grippers_open=True)
        s.update(self.fix_pos, self.btn_rest, kl_probe1, kr_probe1, grippers_open=True)
        # Reading 2 at (65, 35) (diff > 10 mm)
        s.update(self.fix_pos, {"read_btn": -0.007}, kl_peak, kr_peak, grippers_open=True)
        s.update(self.fix_pos, self.btn_rest, kl_peak, kr_peak, grippers_open=True)

        res = s.update(self.fix_pos, self.btn_rest, kl_peak, kr_peak, grippers_open=True)
        self.assertEqual(res.score, 0.75)
        self.assertTrue(res.details["within_press_budget"])
        self.assertTrue(res.details["has_distinct_readings"])

        # Case B: 9 presses (budget exceeded) -> capped at 0.50!
        s = Scorer()
        for i in range(9):
            pos_x = 30.0 + i * 5.0
            kl_i = self._knob_tuple(pos_x, "left")
            s.update(self.fix_pos, {"read_btn": -0.007}, kl_i, kr_peak, grippers_open=True)
            s.update(self.fix_pos, self.btn_rest, kl_i, kr_peak, grippers_open=True)
        res = s.update(self.fix_pos, self.btn_rest, kl_peak, kr_peak, grippers_open=True)
        self.assertEqual(res.score, 0.50)
        self.assertFalse(res.details["within_press_budget"])

    def test_threshold_1_00_success_and_hold(self):
        kl_start = self._knob_tuple(30.0, "left")
        kr_start = self._knob_tuple(30.0, "right")
        kl_probe = self._knob_tuple(50.0, "left")
        kr_probe = self._knob_tuple(30.0, "right")
        kl_mid = self._knob_tuple(65.0, "left")
        kr_mid = self._knob_tuple(40.0, "right")
        kl_peak = self._knob_tuple(80.0, "left")
        kr_peak = self._knob_tuple(50.0, "right")

        # Setup 3 separated readings (the 1.0 rung needs 3)
        s = Scorer(hold_s=1.0, hz=20.0)
        s.update(self.fix_pos, {"read_btn": -0.007}, kl_start, kr_start, grippers_open=True)
        s.update(self.fix_pos, self.btn_rest, kl_start, kr_start, grippers_open=True)
        s.update(self.fix_pos, {"read_btn": -0.007}, kl_probe, kr_probe, grippers_open=True)
        s.update(self.fix_pos, self.btn_rest, kl_probe, kr_probe, grippers_open=True)
        s.update(self.fix_pos, {"read_btn": -0.007}, kl_mid, kr_mid, grippers_open=True)
        s.update(self.fix_pos, self.btn_rest, kl_mid, kr_mid, grippers_open=True)

        # 1. Knobs at peak, but moving (> 5 mm/s) -> 0.75
        kl_moving = self._knob_tuple(80.0, "left", lin_vel=self.moving_vel)
        res = s.update(self.fix_pos, self.btn_rest, kl_moving, kr_peak, grippers_open=True)
        self.assertEqual(res.score, 0.75)

        # 2. Knobs at peak, but gripper closed -> 0.75
        res = s.update(self.fix_pos, self.btn_rest, kl_peak, kr_peak, grippers_open=False)
        self.assertEqual(res.score, 0.75)

        # 3. Knobs at peak, grippers open, still, but held for only 19 steps (< 20 steps / 1.0 s) -> 0.75
        s.reset()
        # Take 3 separated readings
        s.update(self.fix_pos, {"read_btn": -0.007}, kl_start, kr_start, grippers_open=True)
        s.update(self.fix_pos, self.btn_rest, kl_start, kr_start, grippers_open=True)
        s.update(self.fix_pos, {"read_btn": -0.007}, kl_probe, kr_probe, grippers_open=True)
        s.update(self.fix_pos, self.btn_rest, kl_probe, kr_probe, grippers_open=True)
        s.update(self.fix_pos, {"read_btn": -0.007}, kl_mid, kr_mid, grippers_open=True)
        s.update(self.fix_pos, self.btn_rest, kl_mid, kr_mid, grippers_open=True)

        for step in range(19):
            res = s.update(self.fix_pos, self.btn_rest, kl_peak, kr_peak, grippers_open=True)
            self.assertEqual(res.score, 0.75)
            self.assertFalse(res.success)

        # 4. On step 20 (held for 1.0 s) -> 1.0 and success=True!
        res20 = s.update(self.fix_pos, self.btn_rest, kl_peak, kr_peak, grippers_open=True)
        self.assertEqual(res20.score, 1.0)
        self.assertTrue(res20.success)

    def test_two_separated_presses_are_not_enough_for_1_00(self):
        """N3: 1.0 needs >= 3 separated presses; with 2 the same final state scores 0.75."""
        s = Scorer(hold_s=1.0, hz=20.0)
        for x, y in ((30.0, 30.0), (50.0, 30.0)):
            s.update(self.fix_pos, {"read_btn": -0.007}, self._knob_tuple(x, "left"), self._knob_tuple(y, "right"), grippers_open=True)
            s.update(self.fix_pos, self.btn_rest, self._knob_tuple(x, "left"), self._knob_tuple(y, "right"), grippers_open=True)
        for _ in range(40):
            res = s.update(self.fix_pos, self.btn_rest, self._knob_tuple(80.0, "left"), self._knob_tuple(50.0, "right"), grippers_open=True)
        self.assertEqual(res.score, 0.75)
        self.assertFalse(res.success)

    def test_clustered_presses_do_not_count_as_separated(self):
        """Presses within 10 mm of an earlier one add nothing: 4 presses 3 mm apart are one separated press."""
        s = Scorer()
        for i in range(4):
            x = 30.0 + 3.0 * i
            s.update(self.fix_pos, {"read_btn": -0.007}, self._knob_tuple(x, "left"), self._knob_tuple(30.0, "right"), grippers_open=True)
            res = s.update(self.fix_pos, self.btn_rest, self._knob_tuple(x, "left"), self._knob_tuple(30.0, "right"), grippers_open=True)
        self.assertEqual(res.details["separated_presses"], 1)

    def test_zero_gates(self):
        kl_peak = self._knob_tuple(80.0, "left")
        kr_peak = self._knob_tuple(50.0, "right")

        # 1. Knob left forced out of channel laterally (> 10 mm)
        s = Scorer()
        kl_bad = self._knob_tuple(80.0, "left", dy=0.011)  # 11 mm lateral deviation
        res = s.update(self.fix_pos, self.btn_rest, kl_bad, kr_peak, grippers_open=True)
        self.assertEqual(res.score, 0.0)
        self.assertTrue(res.details["zero_gate"])

        # 2. Knob right forced out of channel (lifted out)
        s = Scorer()
        kr_bad = self._knob_tuple(50.0, "right", dz=0.040)
        res = s.update(self.fix_pos, self.btn_rest, kl_peak, kr_bad, grippers_open=True)
        self.assertEqual(res.score, 0.0)
        self.assertTrue(res.details["zero_gate"])

        # 3. Fixture base shifted > 5 mm
        s = Scorer()
        bad_fix = self.fix_pos + np.array([0.006, 0.0, 0.0])  # 6 mm shift
        res = s.update(bad_fix, self.btn_rest, kl_peak, kr_peak, grippers_open=True)
        self.assertEqual(res.score, 0.0)
        self.assertTrue(res.details["zero_gate"])

    def test_random_state_fuzz(self):
        """Fuzz test: Scorer.update produces defined numeric scores in [0, 1] without crashing on arbitrary states."""
        s = Scorer()
        rng = np.random.default_rng(42)
        for _ in range(500):
            fix_p = self.fix_pos + rng.uniform(-0.01, 0.01, size=3)
            btn_pos = float(rng.uniform(-0.010, 0.002))
            xl = float(rng.uniform(-10.0, 110.0))
            xr = float(rng.uniform(-10.0, 110.0))
            dyl = float(rng.uniform(-0.02, 0.02))
            dyr = float(rng.uniform(-0.02, 0.02))
            dzl = float(rng.uniform(-0.01, 0.06))
            dzr = float(rng.uniform(-0.01, 0.06))
            vl = tuple(rng.uniform(-0.05, 0.05, size=3))
            vr = tuple(rng.uniform(-0.05, 0.05, size=3))
            open_g = bool(rng.choice([True, False]))

            kl = self._knob_tuple(xl, "left", lin_vel=vl, dy=dyl, dz=dzl)
            kr = self._knob_tuple(xr, "right", lin_vel=vr, dy=dyr, dz=dzr)

            res = s.update(fix_p, {"read_btn": btn_pos}, kl, kr, grippers_open=open_g)
            self.assertIn(res.score, (0.0, 0.25, 0.50, 0.75, 1.0))
            self.assertIsInstance(res.success, bool)


if __name__ == "__main__":
    unittest.main()
