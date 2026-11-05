"""Unit tests for all Task 4 run modes, plan durations, and end-state scores."""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent.parent
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))
else:
    cand = HERE.parent.parent / "refs" / "yam-isaac-lab-arena"
    if cand.exists():
        sys.path.insert(0, str(cand))
sys.path.insert(0, str(HERE))

import demo_plan
import geometry as g
from scoring import (
    Scorer,
    SCORE_NONE,
    SCORE_TOUCHED,
    SCORE_PULLED,
    SCORE_SWAPPED,
    SCORE_SUCCESS,
)

Q_SEATED = np.array([0.0, 0.0, np.sin(g.PLUG_YAW_SEATED / 2), np.cos(g.PLUG_YAW_SEATED / 2)])


def make_module(pos, quat=Q_SEATED, lin_vel=(0, 0, 0), ang_vel=(0, 0, 0)):
    return (np.array(pos, float), np.array(quat, float), np.array(lin_vel, float), np.array(ang_vel, float))


class TestTask4Modes(unittest.TestCase):
    def setUp(self) -> None:
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

    def test_mode_settle_plan_and_score(self) -> None:
        """settle mode: empty demo plan, score remains 0.0."""
        os.environ["DEMO_SETTLE_ONLY"] = "1"
        try:
            plans = demo_plan.plans()
            self.assertEqual(plans, {})
        finally:
            os.environ.pop("DEMO_SETTLE_ONLY", None)

        scorer = Scorer()
        res = scorer.update(self.fixture_pos, self.fixture_quat, self.zero_sliders, self.start_modules(), True)
        self.assertEqual(res.score, SCORE_NONE)
        self.assertFalse(res.success)

    def test_mode_demo_duration_and_steps(self) -> None:
        """demo mode: plan duration must be ~900-1,300 steps at 20 Hz, leaving > 25% margin under 3000 steps."""
        plans = demo_plan.plans()
        self.assertIn("left", plans)
        self.assertIn("right", plans)

        dur_l = sum(w.duration for w in plans["left"])
        dur_r = sum(w.duration for w in plans["right"])
        max_dur = max(dur_l, dur_r)
        steps = max_dur * 20

        self.assertGreaterEqual(steps, 900)
        self.assertLessEqual(steps, 1300)
        self.assertLess(steps, 3000 * 0.75, "demo steps must leave >= 25% margin under 3000 steps")

    def test_mode_wrong_socket_plan_and_score(self) -> None:
        """wrong_socket mode: swaps module 2 instead of 3 -> healthy module removed > 20 steps -> zero-gate 0.0."""
        os.environ["DEMO_WRONG_SOCKET"] = "1"
        try:
            plans = demo_plan.plans()
            self.assertIn("left", plans)
            self.assertIn("right", plans)
            self.assertTrue(any("socket_2" in w.label for w in plans["right"]))
        finally:
            os.environ.pop("DEMO_WRONG_SOCKET", None)

        scorer = Scorer()
        mods = self.start_modules()
        # Initial step
        scorer.update(self.fixture_pos, self.fixture_quat, self.zero_sliders, mods, True)
        # Pull module 2 (healthy module) and leave it unseated for 25 steps
        mods["module_2"] = make_module(g.STAGE_POS["module_2"])
        for _ in range(25):
            res = scorer.update(self.fixture_pos, self.fixture_quat, self.zero_sliders, mods, False)
        self.assertEqual(res.score, SCORE_NONE)
        self.assertTrue(res.details.get("failed", False))

    def test_mode_no_diag_score(self) -> None:
        """no_diag mode: swap completed without ever pressing DIAG -> score 0.75."""
        os.environ["DEMO_NO_DIAG"] = "1"
        try:
            plans = demo_plan.plans()
            self.assertIn("left", plans)
            self.assertIn("right", plans)
            # Left arm does not have DIAG press waypoints
            self.assertFalse(any("press DIAG" in w.label for w in plans["left"]))
        finally:
            os.environ.pop("DEMO_NO_DIAG", None)

        scorer = Scorer()
        mods = self.start_modules()
        mods["module_3"] = make_module(g.STAGE_POS["module_3"])
        mods["spare"] = make_module(g.START_POS["module_3"])
        # No DIAG button press occurs: diag_btn stays 0.0
        res = scorer.update(self.fixture_pos, self.fixture_quat, self.zero_sliders, mods, True)
        self.assertEqual(res.score, SCORE_SWAPPED)
        self.assertFalse(res.success)

    def test_mode_no_verify_score(self) -> None:
        """no_verify mode: initial DIAG press and swap, but no verify press -> score 0.75."""
        os.environ["DEMO_NO_VERIFY"] = "1"
        try:
            plans = demo_plan.plans()
            self.assertIn("left", plans)
            self.assertIn("right", plans)
            self.assertTrue(any("hold DIAG blink" in w.label for w in plans["left"]))
            self.assertFalse(any("verify" in w.label for w in plans["left"]))
        finally:
            os.environ.pop("DEMO_NO_VERIFY", None)

        scorer = Scorer()
        mods = self.start_modules()
        # 1. Initial DIAG press
        for _ in range(60):
            scorer.update(self.fixture_pos, self.fixture_quat, {"diag_btn": -0.007}, mods, False)
        # 2. Swap modules
        mods["module_3"] = make_module(g.STAGE_POS["module_3"])
        mods["spare"] = make_module(g.START_POS["module_3"])
        # 3. Settle without verify press
        res = scorer.update(self.fixture_pos, self.fixture_quat, self.zero_sliders, mods, True)
        self.assertEqual(res.score, SCORE_SWAPPED)
        self.assertFalse(res.success)

    def test_mode_no_swap_score(self) -> None:
        """no_swap mode: presses DIAG only, no module swapped -> score 0.25."""
        os.environ["DEMO_NO_SWAP"] = "1"
        try:
            plans = demo_plan.plans()
            self.assertIn("left", plans)
            self.assertIn("right", plans)
            self.assertFalse(any("pull" in w.label for w in plans["right"]))
        finally:
            os.environ.pop("DEMO_NO_SWAP", None)

        scorer = Scorer()
        mods = self.start_modules()
        for _ in range(60):
            scorer.update(self.fixture_pos, self.fixture_quat, {"diag_btn": -0.007}, mods, False)
        res = scorer.update(self.fixture_pos, self.fixture_quat, self.zero_sliders, mods, True)
        self.assertEqual(res.score, SCORE_TOUCHED)
        self.assertFalse(res.success)


if __name__ == "__main__":
    unittest.main()
