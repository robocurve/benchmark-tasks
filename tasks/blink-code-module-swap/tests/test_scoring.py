"""Unit tests for Task 4 scoring: threshold boundaries, partial credit ladder, zero-gates, and success hold."""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent.parent
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))
sys.path.insert(0, str(HERE))

import geometry as g
from scoring import (
    Scorer,
    is_module_seated_in_socket,
    get_seated_socket,
    SCORE_NONE,
    SCORE_TOUCHED,
    SCORE_PULLED,
    SCORE_SWAPPED,
    SCORE_SUCCESS,
)

Q_SEATED = np.array([0.0, 0.0, np.sin(g.PLUG_YAW_SEATED / 2), np.cos(g.PLUG_YAW_SEATED / 2)])


def make_module(pos, quat=Q_SEATED, lin_vel=(0, 0, 0), ang_vel=(0, 0, 0)):
    return (np.array(pos, float), np.array(quat, float), np.array(lin_vel, float), np.array(ang_vel, float))


class TestTask4Scoring(unittest.TestCase):
    def setUp(self) -> None:
        self.scorer = Scorer()
        self.fixture_pos = np.array([0.0, 0.0, 0.0])
        self.fixture_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.zero_sliders = {"diag_btn": 0.0, "plunger_0": 0.0, "plunger_1": 0.0, "plunger_2": 0.0, "plunger_3": 0.0}

    def start_modules(self):
        return {
            "module_1": make_module(g.START_POS["module_1"]),
            "module_2": make_module(g.START_POS["module_2"]),
            "module_3": make_module(g.START_POS["module_3"]),
            "module_4": make_module(g.START_POS["module_4"]),
            "spare": make_module(g.SPARE_STAGE_POS),
        }

    def test_initial_state_score_zero(self) -> None:
        res = self.scorer.update(self.fixture_pos, self.fixture_quat, self.zero_sliders, self.start_modules(), True)
        self.assertEqual(res.score, SCORE_NONE)
        self.assertFalse(res.success)

    def test_seated_boundary_depth(self) -> None:
        # Seated depth threshold is 0.016 m past mouth
        mouth_x = g.MOUTH_X
        sy = g.SOCKET_Y[2]
        sz = g.SOCKET_Z

        # At exactly 16 mm nose depth: origin x = mouth_x - NOSE_L + 0.016 = mouth_x - 0.018 + 0.016 = mouth_x - 0.002
        x_pass = mouth_x - g.NOSE_L + 0.0160
        x_fail = mouth_x - g.NOSE_L + 0.0159

        self.assertTrue(is_module_seated_in_socket((x_pass, sy, sz), Q_SEATED, 2))
        self.assertFalse(is_module_seated_in_socket((x_fail, sy, sz), Q_SEATED, 2))

    def test_seated_boundary_lateral(self) -> None:
        mouth_x = g.MOUTH_X
        sy = g.SOCKET_Y[2]
        sz = g.SOCKET_Z
        x_seated = mouth_x

        # Lateral limit 0.0015 m
        self.assertTrue(is_module_seated_in_socket((x_seated, sy + 0.0015, sz), Q_SEATED, 2))
        self.assertFalse(is_module_seated_in_socket((x_seated, sy + 0.0016, sz), Q_SEATED, 2))

    def test_seated_boundary_angle(self) -> None:
        mouth_x = g.MOUTH_X
        sy = g.SOCKET_Y[2]
        sz = g.SOCKET_Z

        # Yaw error around z: 10 deg limit
        th_pass = g.PLUG_YAW_SEATED + np.radians(9.9)
        q_pass = np.array([0.0, 0.0, np.sin(th_pass / 2), np.cos(th_pass / 2)])
        th_fail = g.PLUG_YAW_SEATED + np.radians(10.1)
        q_fail = np.array([0.0, 0.0, np.sin(th_fail / 2), np.cos(th_fail / 2)])

        self.assertTrue(is_module_seated_in_socket((mouth_x, sy, sz), q_pass, 2))
        self.assertFalse(is_module_seated_in_socket((mouth_x, sy, sz), q_fail, 2))

    def test_tier_025_button_touch_or_module_moved(self) -> None:
        mods = self.start_modules()
        # Button pressed slightly (<= -0.002 m)
        res_btn = self.scorer.update(self.fixture_pos, self.fixture_quat, {"diag_btn": -0.0025}, mods, False)
        self.assertEqual(res_btn.score, SCORE_TOUCHED)

    def test_tier_050_failed_module_pulled(self) -> None:
        mods = self.start_modules()
        # Pull module 3 to staging spot
        mods["module_3"] = make_module(g.STAGE_POS["module_3"])
        res = self.scorer.update(self.fixture_pos, self.fixture_quat, self.zero_sliders, mods, False)
        self.assertEqual(res.score, SCORE_PULLED)

    def test_tier_075_spare_plugged_in_socket_3(self) -> None:
        mods = self.start_modules()
        mods["module_3"] = make_module(g.STAGE_POS["module_3"])
        # Plug spare into socket 3
        mods["spare"] = make_module(g.START_POS["module_3"])
        res = self.scorer.update(self.fixture_pos, self.fixture_quat, self.zero_sliders, mods, False)
        self.assertEqual(res.score, SCORE_SWAPPED)

    def test_zero_gate_healthy_module_unseated_for_more_than_1s(self) -> None:
        mods = self.start_modules()
        # Unseat module 2 (healthy module)
        mods["module_2"] = make_module(g.STAGE_POS["module_2"])

        # Steps 1..20: unseated tolerance
        for _ in range(20):
            res = self.scorer.update(self.fixture_pos, self.fixture_quat, self.zero_sliders, mods, False)
            self.assertFalse(res.details.get("failed", False))

        # Step 21 (> 20 steps = 1.0 s) -> zero gate fires permanently
        res21 = self.scorer.update(self.fixture_pos, self.fixture_quat, self.zero_sliders, mods, False)
        self.assertEqual(res21.score, SCORE_NONE)
        self.assertTrue(res21.details.get("failed", False))

    def test_zero_gate_module_dropped_off_table(self) -> None:
        mods = self.start_modules()
        mods["module_3"] = make_module((0.3, -0.2, g.TOP - 0.25))
        res = self.scorer.update(self.fixture_pos, self.fixture_quat, self.zero_sliders, mods, False)
        self.assertEqual(res.score, SCORE_NONE)
        self.assertTrue(res.details.get("failed", False))

    def test_tier_100_full_success_hold(self) -> None:
        mods = self.start_modules()
        # 1. Unplug module 3 and place on table
        mods["module_3"] = make_module(g.STAGE_POS["module_3"])
        # 2. Plug spare into socket 3
        mods["spare"] = make_module(g.START_POS["module_3"])

        # 3. Left arm presses DIAG button for 60 steps (3.0 s) to verify steady green
        for _ in range(60):
            self.scorer.update(self.fixture_pos, self.fixture_quat, {"diag_btn": -0.007}, mods, False)

        # 4. Release button and open grippers, modules still
        # Need 20 consecutive steps for success hold
        for step in range(19):
            res = self.scorer.update(self.fixture_pos, self.fixture_quat, self.zero_sliders, mods, True)
            self.assertEqual(res.score, SCORE_SWAPPED)
            self.assertFalse(res.success)

        # 20th step reaches 1.00 and success True
        res20 = self.scorer.update(self.fixture_pos, self.fixture_quat, self.zero_sliders, mods, True)
        self.assertEqual(res20.score, SCORE_SUCCESS)
        self.assertTrue(res20.success)


if __name__ == "__main__":
    unittest.main()
