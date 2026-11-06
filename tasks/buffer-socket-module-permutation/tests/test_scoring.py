"""Unit tests for scoring.py in buffer-socket-module-permutation."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import geometry as g
from scoring import (
    SCORE_NONE,
    SCORE_PULLED,
    SCORE_ONE_MATCHING,
    SCORE_TWO_MATCHING,
    SCORE_SUCCESS,
    SUCCESS_HOLD_STEPS,
    Scorer,
    find_seated_socket,
    is_module_seated_in_socket,
)

Q_SEATED = np.array([0.0, 0.0, np.sin(g.PLUG_YAW_SEATED / 2), np.cos(g.PLUG_YAW_SEATED / 2)])


def make_module(pos, quat=Q_SEATED, lin_vel=(0, 0, 0), ang_vel=(0, 0, 0)):
    return (np.array(pos, float), np.array(quat, float), np.array(lin_vel, float), np.array(ang_vel, float))


class TestScoring(unittest.TestCase):
    def setUp(self):
        self.scorer = Scorer()
        self.fpos = np.array([0.0, 0.0, 0.0])
        self.fquat = np.array([0.0, 0.0, 0.0, 1.0])
        self.sliders = {"commit_btn": 0.0}

    def start_modules(self):
        return {
            "module_green": make_module(g.START_POS["module_green"]),
            "module_red": make_module(g.START_POS["module_red"]),
            "module_blue": make_module(g.START_POS["module_blue"]),
        }

    def test_initial_state_score_zero(self):
        res = self.scorer.update(self.fpos, self.fquat, self.sliders, self.start_modules(), True)
        self.assertEqual(res.score, SCORE_NONE)
        self.assertFalse(res.success)
        self.assertFalse(res.details["has_pulled"])
        self.assertEqual(res.details["lamp_state"], "red")

    def test_tier_025_any_module_pulled(self):
        mods = self.start_modules()
        # Initial step
        self.scorer.update(self.fpos, self.fquat, self.sliders, mods, True)

        # Pull green module 20 mm back along -x (depth < 16 mm)
        pulled_pos = (g.START_POS["module_green"][0] - 0.020, g.START_POS["module_green"][1], g.START_POS["module_green"][2])
        mods["module_green"] = make_module(pulled_pos)

        res = self.scorer.update(self.fpos, self.fquat, self.sliders, mods, False)
        self.assertEqual(res.score, SCORE_PULLED)
        self.assertTrue(res.details["has_pulled"])
        self.assertFalse(res.success)

    def test_tier_050_one_swapped_module_matching(self):
        mods = self.start_modules()
        # Pull green to buffer socket (socket 3)
        mods["module_green"] = make_module((g.MOUTH_X, g.SOCKET_Y[3], g.SOCKET_Z))
        # Place red into socket 1 (socket index 0, matching!)
        mods["module_red"] = make_module((g.MOUTH_X, g.SOCKET_Y[0], g.SOCKET_Z))

        res = self.scorer.update(self.fpos, self.fquat, self.sliders, mods, False)
        self.assertEqual(res.score, SCORE_ONE_MATCHING)
        self.assertEqual(res.details["matching_swapped"], 1)
        self.assertEqual(res.details["lamp_state"], "red")

    def test_tier_075_both_swapped_modules_matching(self):
        mods = self.start_modules()
        # Red in socket 0 (matching)
        mods["module_red"] = make_module((g.MOUTH_X, g.SOCKET_Y[0], g.SOCKET_Z))
        # Green in socket 1 (matching)
        mods["module_green"] = make_module((g.MOUTH_X, g.SOCKET_Y[1], g.SOCKET_Z))
        # Blue in socket 2 (matching)
        mods["module_blue"] = make_module((g.START_POS["module_blue"]))

        res = self.scorer.update(self.fpos, self.fquat, self.sliders, mods, True)
        self.assertEqual(res.score, SCORE_TWO_MATCHING)
        self.assertEqual(res.details["matching_swapped"], 2)
        self.assertEqual(res.details["lamp_state"], "green")
        self.assertFalse(res.success, "Cannot succeed without COMMIT press")

    def test_tier_100_full_success_hold(self):
        mods = self.start_modules()
        # All three matching
        mods["module_red"] = make_module((g.MOUTH_X, g.SOCKET_Y[0], g.SOCKET_Z))
        mods["module_green"] = make_module((g.MOUTH_X, g.SOCKET_Y[1], g.SOCKET_Z))
        mods["module_blue"] = make_module((g.MOUTH_X, g.SOCKET_Y[2], g.SOCKET_Z))

        # Press commit while green
        self.sliders["commit_btn"] = -0.007
        res = self.scorer.update(self.fpos, self.fquat, self.sliders, mods, False)
        self.assertTrue(res.details["commit_valid"])

        # Release commit button and hold still with open grippers for 20 steps
        self.sliders["commit_btn"] = 0.0
        for step in range(SUCCESS_HOLD_STEPS):
            res = self.scorer.update(self.fpos, self.fquat, self.sliders, mods, True)
            if step < SUCCESS_HOLD_STEPS - 1:
                self.assertFalse(res.success)
                self.assertEqual(res.score, SCORE_TWO_MATCHING)

        self.assertTrue(res.success)
        self.assertEqual(res.score, SCORE_SUCCESS)

    def test_zero_gate_fixture_shift(self):
        mods = self.start_modules()
        self.scorer.update(self.fpos, self.fquat, self.sliders, mods, True)

        # Move fixture > 5 mm
        shifted_fpos = np.array([0.006, 0.0, 0.0])
        res = self.scorer.update(shifted_fpos, self.fquat, self.sliders, mods, True)
        self.assertTrue(res.details["failed"])
        self.assertEqual(res.score, SCORE_NONE)

    def test_zero_gate_module_fall(self):
        mods = self.start_modules()
        self.scorer.update(self.fpos, self.fquat, self.sliders, mods, True)

        # Drop module > 20 cm below table
        fallen_pos = (g.START_POS["module_green"][0], g.START_POS["module_green"][1], g.TOP - 0.25)
        mods["module_green"] = make_module(fallen_pos)
        res = self.scorer.update(self.fpos, self.fquat, self.sliders, mods, True)
        self.assertTrue(res.details["failed"])
        self.assertEqual(res.score, SCORE_NONE)

    def test_seating_tolerance_boundaries(self):
        # Nominal seated at socket 0 (y = -0.260)
        sock_y = g.SOCKET_Y[0]
        pos_seated = (g.MOUTH_X, sock_y, g.SOCKET_Z)
        self.assertTrue(is_module_seated_in_socket(pos_seated, Q_SEATED, 0))

        # 1. Depth test: nose past mouth
        # nominal depth = NOSE_L = 18 mm.
        # At x = MOUTH_X - 0.002, depth = 16.0 mm -> seated
        self.assertTrue(is_module_seated_in_socket((g.MOUTH_X - 0.0019, sock_y, g.SOCKET_Z), Q_SEATED, 0))
        # At x = MOUTH_X - 0.003, depth = 15.0 mm -> unseated
        self.assertFalse(is_module_seated_in_socket((g.MOUTH_X - 0.0030, sock_y, g.SOCKET_Z), Q_SEATED, 0))

        # 2. Lateral offset test (limit 1.5 mm)
        self.assertTrue(is_module_seated_in_socket((g.MOUTH_X, sock_y + 0.0014, g.SOCKET_Z), Q_SEATED, 0))
        self.assertFalse(is_module_seated_in_socket((g.MOUTH_X, sock_y + 0.0016, g.SOCKET_Z), Q_SEATED, 0))

        # 3. Tilt test (limit 10 degrees)
        yaw_good = g.PLUG_YAW_SEATED + math.radians(8.0)
        q_good = np.array([0.0, 0.0, math.sin(yaw_good / 2), math.cos(yaw_good / 2)])
        self.assertTrue(is_module_seated_in_socket(pos_seated, q_good, 0))

        yaw_bad = g.PLUG_YAW_SEATED + math.radians(12.0)
        q_bad = np.array([0.0, 0.0, math.sin(yaw_bad / 2), math.cos(yaw_bad / 2)])
        self.assertFalse(is_module_seated_in_socket(pos_seated, q_bad, 0))


if __name__ == "__main__":
    unittest.main()
