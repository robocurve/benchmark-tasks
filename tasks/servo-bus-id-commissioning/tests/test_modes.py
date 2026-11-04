"""Unit tests for all run.sh modes, end-state scoring, plan durations, and success hold (C3, C4)."""

from __future__ import annotations

import os
import re
import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))
sys.path.insert(0, str(HERE))

import demo_plan
import geometry as g
from scoring import Scorer

Q_SEATED = np.array([0.0, 0.0, np.sin(-np.pi / 4), np.cos(-np.pi / 4)])


def make_servo(pos, quat=Q_SEATED, lin_vel=(0, 0, 0), ang_vel=(0, 0, 0)):
    return (np.array(pos, float), np.array(quat, float), np.array(lin_vel, float), np.array(ang_vel, float))


class TestTaskFModes(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture_pos = np.array([0.0, 0.0, 0.0])
        self.zero_sliders = {"assign_btn": 0.0, "reset_btn": 0.0, "plunger_0": 0.0, "plunger_1": 0.0, "plunger_2": 0.0}

    def start_servos(self):
        return {
            "blue": make_servo(g.START_POS["blue"]),
            "red": make_servo(g.START_POS["red"]),
            "green": make_servo(g.START_POS["green"]),
        }

    def test_mode_settle_plan_and_score(self) -> None:
        """Settle mode: empty demo plan, score remains 0.0."""
        os.environ["DEMO_SETTLE_ONLY"] = "1"
        try:
            plans = demo_plan.plans()
            self.assertEqual(plans, {})
        finally:
            os.environ.pop("DEMO_SETTLE_ONLY", None)

        scorer = Scorer()
        res = scorer.update(self.fixture_pos, self.zero_sliders, self.start_servos(), False)
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)

    def test_mode_all_plugged_plan_and_score(self) -> None:
        """all_plugged mode: ASSIGN pressed while 3 unassigned servos on bus -> score 0.25, IDs None."""
        os.environ["DEMO_ALL_PLUGGED"] = "1"
        try:
            plans = demo_plan.plans()
            self.assertIn("left", plans)
            self.assertIn("right", plans)
            dur = sum(w.duration for w in plans["left"])
            self.assertGreater(dur, 0.0)
        finally:
            os.environ.pop("DEMO_ALL_PLUGGED", None)

        scorer = Scorer()
        # Initial step
        scorer.update(self.fixture_pos, self.zero_sliders, self.start_servos(), False)
        # ASSIGN pressed
        res_press = scorer.update(self.fixture_pos, {"assign_btn": -0.007, "reset_btn": 0.0}, self.start_servos(), False)
        self.assertEqual(res_press.score, 0.25)
        # Bus clash: lamp stays red, no ID assigned
        self.assertEqual(res_press.details["bus_lamp"], "red")
        self.assertIsNone(res_press.details["ids"]["red"])
        self.assertIsNone(res_press.details["ids"]["green"])
        self.assertIsNone(res_press.details["ids"]["blue"])
        # Release ASSIGN and let go
        res_end = scorer.update(self.fixture_pos, self.zero_sliders, self.start_servos(), True)
        self.assertEqual(res_end.score, 0.25)
        self.assertFalse(res_end.success)

    def test_mode_wrong_order_plan_and_score(self) -> None:
        """wrong_order mode: Green commissioned first, then Red, then Blue -> score 0.25."""
        os.environ["DEMO_WRONG_ORDER"] = "1"
        try:
            plans = demo_plan.plans()
            self.assertIn("left", plans)
            self.assertIn("right", plans)
        finally:
            os.environ.pop("DEMO_WRONG_ORDER", None)

        scorer = Scorer()
        servos = self.start_servos()
        # Unplug red and blue (green remains seated)
        servos["red"] = make_servo(g.STAGE_POS["red"])
        servos["blue"] = make_servo(g.STAGE_POS["blue"])
        scorer.update(self.fixture_pos, self.zero_sliders, servos, False)

        # ASSIGN green -> green gets 1
        scorer.update(self.fixture_pos, {"assign_btn": -0.007, "reset_btn": 0.0}, servos, False)
        scorer.update(self.fixture_pos, self.zero_sliders, servos, False)

        # Plug red and ASSIGN -> red gets 2
        servos["red"] = make_servo(g.START_POS["red"])
        scorer.update(self.fixture_pos, {"assign_btn": -0.007, "reset_btn": 0.0}, servos, False)
        scorer.update(self.fixture_pos, self.zero_sliders, servos, False)

        # Plug blue and ASSIGN -> blue gets 3
        servos["blue"] = make_servo(g.START_POS["blue"])
        scorer.update(self.fixture_pos, {"assign_btn": -0.007, "reset_btn": 0.0}, servos, False)
        res_end = scorer.update(self.fixture_pos, self.zero_sliders, servos, True)

        self.assertEqual(res_end.details["ids"]["green"], 1)
        self.assertEqual(res_end.details["ids"]["red"], 2)
        self.assertEqual(res_end.details["ids"]["blue"], 3)
        # Red is not 1, so tier 0.50 condition fails; score is 0.25
        self.assertEqual(res_end.score, 0.25)
        self.assertFalse(res_end.success)

    def test_mode_recover_plan_and_score(self) -> None:
        """recover mode: Wrong order / error, RESET pressed, then correct commissioning -> score 1.0."""
        os.environ["DEMO_RECOVER"] = "1"
        try:
            plans = demo_plan.plans()
            self.assertIn("left", plans)
            self.assertIn("right", plans)
            dur = max(sum(w.duration for w in plans[arm]) for arm in ("left", "right"))
            self.assertLess(dur * 20, 3000)  # must fit run limit
        finally:
            os.environ.pop("DEMO_RECOVER", None)

        scorer = Scorer()
        servos = self.start_servos()
        # Erroneous ASSIGN press
        scorer.update(self.fixture_pos, {"assign_btn": -0.007, "reset_btn": 0.0}, servos, False)
        scorer.update(self.fixture_pos, self.zero_sliders, servos, False)

        # RESET press clears all
        scorer.update(self.fixture_pos, {"assign_btn": 0.0, "reset_btn": -0.007}, servos, False)
        scorer.update(self.fixture_pos, self.zero_sliders, servos, False)

        # Correct procedure: unplug green and blue
        servos["green"] = make_servo(g.STAGE_POS["green"])
        servos["blue"] = make_servo(g.STAGE_POS["blue"])
        scorer.update(self.fixture_pos, self.zero_sliders, servos, False)

        # ASSIGN red -> red=1
        scorer.update(self.fixture_pos, {"assign_btn": -0.007, "reset_btn": 0.0}, servos, False)
        scorer.update(self.fixture_pos, self.zero_sliders, servos, False)

        # Plug green, ASSIGN -> green=2
        servos["green"] = make_servo(g.START_POS["green"])
        scorer.update(self.fixture_pos, {"assign_btn": -0.007, "reset_btn": 0.0}, servos, False)
        scorer.update(self.fixture_pos, self.zero_sliders, servos, False)

        # Plug blue, ASSIGN -> blue=3
        servos["blue"] = make_servo(g.START_POS["blue"])
        scorer.update(self.fixture_pos, {"assign_btn": -0.007, "reset_btn": 0.0}, servos, False)
        res_end = scorer.update(self.fixture_pos, self.zero_sliders, servos, True)

        self.assertEqual(res_end.score, 1.0)
        self.assertTrue(res_end.success)
        self.assertEqual(res_end.details["ids"]["red"], 1)
        self.assertEqual(res_end.details["ids"]["green"], 2)
        self.assertEqual(res_end.details["ids"]["blue"], 3)

    def test_run_sh_step_limits_exceed_demo_lengths(self) -> None:
        """Verify STEPS in run.sh is at least 1.25x the planned duration for each mode (C3)."""
        run_sh_path = HERE / "run.sh"
        self.assertTrue(run_sh_path.exists())
        text = run_sh_path.read_text()

        # Parse STEPS per mode from run.sh
        # e.g.: demo) ... STEPS=800 ;;
        mode_steps = {}
        for m in re.finditer(r"([a-z_]+)\)\s*.*?\bSTEPS=(\d+)", text):
            mode_name, steps_val = m.group(1), int(m.group(2))
            mode_steps[mode_name] = steps_val

        self.assertIn("demo", mode_steps)
        self.assertIn("all_plugged", mode_steps)
        self.assertIn("wrong_order", mode_steps)
        self.assertIn("recover", mode_steps)

        # Check demo mode duration
        for mode_name, env_key in [
            ("demo", None),
            ("all_plugged", "DEMO_ALL_PLUGGED"),
            ("wrong_order", "DEMO_WRONG_ORDER"),
            ("recover", "DEMO_RECOVER"),
        ]:
            if env_key:
                os.environ[env_key] = "1"
            try:
                plans = demo_plan.plans()
                dur = max(sum(w.duration for w in plans[arm]) for arm in ("left", "right"))
                plan_steps = int(round(dur * 20))
                min_required = int(plan_steps * 1.25)
                configured = mode_steps[mode_name]
                self.assertGreaterEqual(
                    configured,
                    min_required,
                    f"Mode {mode_name}: STEPS={configured} is less than 1.25x plan length ({min_required} steps)",
                )
            finally:
                if env_key:
                    os.environ.pop(env_key, None)

    def test_hold_steps_duration(self) -> None:
        """Verify success hold requirement is 1.0 s = 20 steps at 20 Hz control rate (C4)."""
        from yam_arena import task_file
        task = task_file.load(HERE / "task.py")
        self.assertEqual(task.success_hold_s, 1.0)
        # 1.0 s * 20 Hz = 20 steps
        self.assertEqual(round(task.success_hold_s * 20), 20)


if __name__ == "__main__":
    unittest.main()
