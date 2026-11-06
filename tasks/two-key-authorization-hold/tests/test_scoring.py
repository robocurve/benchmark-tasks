"""Unit tests for scoring ladder, zero-gates, and valid solution paths in two-key-authorization-hold."""

from __future__ import annotations

import sys
from pathlib import Path
import unittest
import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import geometry as g
import scoring


class TestTwoKeyScoring(unittest.TestCase):
    def test_scorer_initial(self):
        s = scoring.Scorer()
        res = s.update(
            fixture_pos=g.BASE_POS,
            fixture_quat=(0.0, 0.0, 0.0, 1.0),
            fixture_sliders={},
            grippers_open=True,
        )
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)

    def test_ladder_0_25_guards(self):
        s = scoring.Scorer()
        res = s.update(
            fixture_pos=g.BASE_POS,
            fixture_quat=(0.0, 0.0, 0.0, 1.0),
            fixture_sliders={"slider_guard_left": 0.070, "slider_guard_right": 0.070},
            grippers_open=True,
        )
        self.assertEqual(res.score, 0.25)

    def test_ladder_0_50_keys_held(self):
        s = scoring.Scorer()
        for _ in range(32):
            res = s.update(
                fixture_pos=g.BASE_POS,
                fixture_quat=(0.0, 0.0, 0.0, 1.0),
                fixture_sliders={
                    "slider_guard_left": 0.070,
                    "slider_guard_right": 0.070,
                    "btn_key_left": -0.007,
                    "btn_key_right": -0.007,
                },
                grippers_open=False,
            )
        self.assertEqual(res.score, 0.50)

    def test_ladder_0_75_armed(self):
        s = scoring.Scorer()
        for _ in range(62):
            res = s.update(
                fixture_pos=g.BASE_POS,
                fixture_quat=(0.0, 0.0, 0.0, 1.0),
                fixture_sliders={
                    "slider_guard_left": 0.070,
                    "slider_guard_right": 0.070,
                    "btn_key_left": -0.007,
                    "btn_key_right": -0.007,
                },
                grippers_open=False,
            )
        self.assertEqual(res.score, 0.75)

    def test_two_valid_solution_orders_both_reach_1_0(self):
        """Lesson 3: Assert both valid action orders reach 1.0."""
        zero_vel = {"left": (np.zeros(3), np.zeros(3)), "right": (np.zeros(3), np.zeros(3))}

        # Order 1: Left guard opened first, then Right guard opened
        s1 = scoring.Scorer()
        s1.update(g.BASE_POS, (0, 0, 0, 1), {"slider_guard_left": 0.070, "slider_guard_right": 0.0}, False)
        s1.update(g.BASE_POS, (0, 0, 0, 1), {"slider_guard_left": 0.070, "slider_guard_right": 0.070}, False)
        for _ in range(70):
            s1.update(
                g.BASE_POS, (0, 0, 0, 1),
                {"slider_guard_left": 0.070, "slider_guard_right": 0.070, "btn_key_left": -0.007, "btn_key_right": -0.007},
                False,
            )
        s1.update(g.BASE_POS, (0, 0, 0, 1), {"slider_guard_left": 0.070, "slider_guard_right": 0.070}, False)
        s1.update(g.BASE_POS, (0, 0, 0, 1), {"slider_guard_left": 0.070, "slider_guard_right": 0.070, "btn_confirm": -0.007}, False)
        for _ in range(21):
            res1 = s1.update(g.BASE_POS, (0, 0, 0, 1), {"slider_guard_left": 0.070, "slider_guard_right": 0.070}, True, zero_vel)
        self.assertEqual(res1.score, 1.00)
        self.assertTrue(res1.success)

        # Order 2: Right guard opened first, then Left guard opened
        s2 = scoring.Scorer()
        s2.update(g.BASE_POS, (0, 0, 0, 1), {"slider_guard_left": 0.0, "slider_guard_right": 0.070}, False)
        s2.update(g.BASE_POS, (0, 0, 0, 1), {"slider_guard_left": 0.070, "slider_guard_right": 0.070}, False)
        for _ in range(70):
            s2.update(
                g.BASE_POS, (0, 0, 0, 1),
                {"slider_guard_left": 0.070, "slider_guard_right": 0.070, "btn_key_left": -0.007, "btn_key_right": -0.007},
                False,
            )
        s2.update(g.BASE_POS, (0, 0, 0, 1), {"slider_guard_left": 0.070, "slider_guard_right": 0.070}, False)
        s2.update(g.BASE_POS, (0, 0, 0, 1), {"slider_guard_left": 0.070, "slider_guard_right": 0.070, "btn_confirm": -0.007}, False)
        for _ in range(21):
            res2 = s2.update(g.BASE_POS, (0, 0, 0, 1), {"slider_guard_left": 0.070, "slider_guard_right": 0.070}, True, zero_vel)
        self.assertEqual(res2.score, 1.00)
        self.assertTrue(res2.success)

    def test_zero_gate_lockout_caps_score(self):
        s = scoring.Scorer()
        guards = {"slider_guard_left": 0.070, "slider_guard_right": 0.070}
        keys_down = {**guards, "btn_key_left": -0.007, "btn_key_right": -0.007}

        for _ in range(32):
            s.update(g.BASE_POS, (0, 0, 0, 1), keys_down, False)
        s.update(g.BASE_POS, (0, 0, 0, 1), guards, False)

        for _ in range(20):
            s.update(g.BASE_POS, (0, 0, 0, 1), keys_down, False)
        res = s.update(g.BASE_POS, (0, 0, 0, 1), guards, False)

        self.assertEqual(res.score, 0.25)
        self.assertFalse(res.success)

    def test_zero_gate_fixture_shift(self):
        s = scoring.Scorer()
        shifted_pos = (g.BASE_POS[0] + 0.015, g.BASE_POS[1], g.BASE_POS[2])
        res = s.update(shifted_pos, (0, 0, 0, 1), {"slider_guard_left": 0.070, "slider_guard_right": 0.070}, True)
        self.assertEqual(res.score, 0.0)
        self.assertEqual(res.details["reason"], "fixture_shifted")


if __name__ == "__main__":
    unittest.main()
