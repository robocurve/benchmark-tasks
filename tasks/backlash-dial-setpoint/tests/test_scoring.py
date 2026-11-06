"""Unit tests for scoring.py: ladder, zero-gates and hold conditions."""

import os
import sys
import unittest
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

import geometry as g
import scoring


class TestScoring(unittest.TestCase):
    def setUp(self):
        self.scorer = scoring.Scorer()
        self.quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.vel0 = np.zeros(3)

    def _knob(self, cx, y):
        return (np.array([cx, y, g.KNOB_CZ]), self.quat, self.vel0, self.vel0)

    def test_start_score_is_zero(self):
        kl = self._knob(g.knob_cx_from_setting_mm(30.0), g.TRACK_Y["left"])
        kr = self._knob(g.knob_cx_from_setting_mm(70.0), g.TRACK_Y["right"])
        res = self.scorer.update(g.BASE_POS, {}, kl, kr, True)
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)

    def test_score_0_25_knob_moved_5mm(self):
        # Move left knob by 5 mm (30 -> 35 mm)
        kl = self._knob(g.knob_cx_from_setting_mm(35.0), g.TRACK_Y["left"])
        kr = self._knob(g.knob_cx_from_setting_mm(70.0), g.TRACK_Y["right"])
        res = self.scorer.update(g.BASE_POS, {}, kl, kr, True)
        self.assertEqual(res.score, 0.25)

    def test_score_0_50_one_dial_locked_in_band(self):
        # Move left knob to 65 mm (setpoint)
        kl = self._knob(g.knob_cx_from_setting_mm(65.0), g.TRACK_Y["left"])
        kr = self._knob(g.knob_cx_from_setting_mm(70.0), g.TRACK_Y["right"])
        # Press left lock
        res = self.scorer.update(g.BASE_POS, {"lock_btn_left": -0.007}, kl, kr, True)
        self.assertEqual(res.score, 0.50)

    def test_score_stays_0_50_when_second_dial_locked_outside_band(self):
        # Left dial at 65 mm (in band)
        kl = self._knob(g.knob_cx_from_setting_mm(65.0), g.TRACK_Y["left"])
        # Right dial pushed directly to 40 mm from 70 mm without reversal (display lags at 46 mm -> outside band [37.5, 42.5])
        kr = self._knob(g.knob_cx_from_setting_mm(40.0), g.TRACK_Y["right"])
        self.scorer.update(g.BASE_POS, {}, kl, kr, False)
        # Lock both
        res = self.scorer.update(g.BASE_POS, {"lock_btn_left": -0.007, "lock_btn_right": -0.007}, kl, kr, False)
        self.assertEqual(res.score, 0.50)
        self.assertTrue(res.details["locked_in_band_left"])
        self.assertFalse(res.details["locked_in_band_right"])

    def test_score_0_75_both_dials_locked_without_reversal(self):
        # Left dial at 65 mm (in band)
        kl = self._knob(g.knob_cx_from_setting_mm(65.0), g.TRACK_Y["left"])
        # Right dial pushed directly down from 70 mm to 34 mm (no reversal, display = 34 + 6 = 40 mm in band)
        kr = self._knob(g.knob_cx_from_setting_mm(34.0), g.TRACK_Y["right"])
        self.scorer.update(g.BASE_POS, {}, kl, kr, False)
        # Lock both
        res = self.scorer.update(g.BASE_POS, {"lock_btn_left": -0.007, "lock_btn_right": -0.007}, kl, kr, False)
        self.assertEqual(res.score, 0.75)
        self.assertFalse(res.details["has_reversed_left"])
        self.assertFalse(res.details["has_reversed_right"])

    def test_score_0_75_both_dials_locked_with_reversal(self):
        # Left dial at 65 mm (in band)
        kl = self._knob(g.knob_cx_from_setting_mm(65.0), g.TRACK_Y["left"])
        # Right dial overshoots down to 28 mm, then up to 40 mm (in band with reversal)
        kr1 = self._knob(g.knob_cx_from_setting_mm(28.0), g.TRACK_Y["right"])
        self.scorer.update(g.BASE_POS, {}, kl, kr1, False)

        kr2 = self._knob(g.knob_cx_from_setting_mm(40.0), g.TRACK_Y["right"])
        self.scorer.update(g.BASE_POS, {}, kl, kr2, False)

        # Press both locks
        res = self.scorer.update(g.BASE_POS, {"lock_btn_left": -0.007, "lock_btn_right": -0.007}, kl, kr2, False)
        self.assertEqual(res.score, 0.75)

    def test_score_1_00_hold_condition_without_reversal(self):
        # Direct motion to in-band displayed values: Left at 65 mm, Right at true 34 mm (display 40 mm)
        kl = self._knob(g.knob_cx_from_setting_mm(65.0), g.TRACK_Y["left"])
        kr = self._knob(g.knob_cx_from_setting_mm(34.0), g.TRACK_Y["right"])
        self.scorer.update(g.BASE_POS, {}, kl, kr, False)

        # Lock both
        self.scorer.update(g.BASE_POS, {"lock_btn_left": -0.007, "lock_btn_right": -0.007}, kl, kr, False)
        self.scorer.update(g.BASE_POS, {"lock_btn_left": 0.0, "lock_btn_right": 0.0}, kl, kr, False)

        # Hold still with open grippers for 20 steps
        for _ in range(19):
            res = self.scorer.update(g.BASE_POS, {}, kl, kr, True)
            self.assertEqual(res.score, 0.75)
            self.assertFalse(res.success)

        # 20th step
        res = self.scorer.update(g.BASE_POS, {}, kl, kr, True)
        self.assertEqual(res.score, 1.00)
        self.assertTrue(res.success)

    def test_score_1_00_hold_condition(self):
        # Set both dials in band with reversal
        kl = self._knob(g.knob_cx_from_setting_mm(65.0), g.TRACK_Y["left"])
        kr_down = self._knob(g.knob_cx_from_setting_mm(28.0), g.TRACK_Y["right"])
        self.scorer.update(g.BASE_POS, {}, kl, kr_down, False)

        kr = self._knob(g.knob_cx_from_setting_mm(40.0), g.TRACK_Y["right"])
        self.scorer.update(g.BASE_POS, {}, kl, kr, False)

        # Lock both
        self.scorer.update(g.BASE_POS, {"lock_btn_left": -0.007, "lock_btn_right": -0.007}, kl, kr, False)
        self.scorer.update(g.BASE_POS, {"lock_btn_left": 0.0, "lock_btn_right": 0.0}, kl, kr, False)

        # Hold still with open grippers for 20 steps
        for _ in range(19):
            res = self.scorer.update(g.BASE_POS, {}, kl, kr, True)
            self.assertEqual(res.score, 0.75)
            self.assertFalse(res.success)

        # 20th step
        res = self.scorer.update(g.BASE_POS, {}, kl, kr, True)
        self.assertEqual(res.score, 1.00)
        self.assertTrue(res.success)

    def test_zero_gate_fixture_shift(self):
        kl = self._knob(g.knob_cx_from_setting_mm(65.0), g.TRACK_Y["left"])
        kr = self._knob(g.knob_cx_from_setting_mm(40.0), g.TRACK_Y["right"])
        shifted_pos = (g.BASE_POS[0] + 0.006, g.BASE_POS[1], g.BASE_POS[2])
        res = self.scorer.update(shifted_pos, {}, kl, kr, True)
        self.assertEqual(res.score, 0.0)
        self.assertTrue(res.details["zero_gate_triggered"])


if __name__ == "__main__":
    unittest.main()
