"""Glue tests between run.sh and demo_plan.py: every mode's environment variables are read by demo_plan,
every mode builds the plan it claims to, and STEPS leaves >= 25 percent spare margin over replayed run length.
"""

from __future__ import annotations

import math
import os
import re
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

from yam_arena.constants import FINGER_OPEN
import demo_plan
import geometry as g
from scoring import Scorer

RUN_SH = (HERE / "run.sh").read_text()
ENV_FLAGS = ("DEMO_SETTLE_ONLY", "DEMO_ADJACENT", "DEMO_ONE_ARM", "DEMO_NO_STAR")


def parse_run_sh() -> dict[str, dict]:
    """mode -> {'env': [exported names], 'steps': int} parsed from case block of run.sh."""
    modes = {}
    for line in RUN_SH.splitlines():
        m = re.match(r"^\s*([a-z_]+)\)\s*(.*?)\s*;;\s*$", line)
        if not m or m.group(1) == "*":
            continue
        body = m.group(2)
        env_vars = re.findall(r"export\s+([A-Z_]+)=", body)
        steps_m = re.search(r"STEPS=(\d+)", body)
        steps = int(steps_m.group(1)) if steps_m else 0
        modes[m.group(1)] = dict(env=env_vars, steps=steps)
    return modes


def labels(mode: str) -> list[str]:
    seqs = demo_plan.demo_plans(mode)
    return [w.label for arm in seqs.values() for w in arm]


class TestRunShVersusDemoPlan(unittest.TestCase):
    def setUp(self) -> None:
        for k in ENV_FLAGS:
            os.environ.pop(k, None)

    def tearDown(self) -> None:
        for k in ENV_FLAGS:
            os.environ.pop(k, None)

    def test_every_mode_is_present(self) -> None:
        modes = parse_run_sh()
        self.assertEqual(set(modes), {"settle", "demo", "adjacent", "one_arm", "no_star", "dream"})

    def test_every_exported_variable_is_read_by_demo_plan(self) -> None:
        src = (HERE / "demo_plan.py").read_text()
        for mode, info in parse_run_sh().items():
            for var in info["env"]:
                self.assertIn(f'"{var}"', src, f"mode {mode}: demo_plan.py never reads {var}")

    def test_each_flag_selects_its_mode_and_changes_the_plan(self) -> None:
        default = labels("demo")
        for flag, mode in (
            ("DEMO_SETTLE_ONLY", "settle"),
            ("DEMO_ADJACENT", "adjacent"),
            ("DEMO_ONE_ARM", "one_arm"),
            ("DEMO_NO_STAR", "no_star"),
        ):
            os.environ[flag] = "1"
            self.assertEqual(demo_plan.mode_from_env(), mode)
            env_plan = [w.label for arm in demo_plan.plans().values() for w in arm]
            self.assertEqual(env_plan, labels(mode))
            if mode != "demo":
                self.assertNotEqual(env_plan, default, f"{flag} does not change the plan")
            os.environ.pop(flag)

    def test_steps_leaves_twenty_five_percent_spare_margin(self) -> None:
        modes = parse_run_sh()
        plans = demo_plan.demo_plans("demo")
        duration = max(sum(w.duration for w in plans["left"]), sum(w.duration for w in plans["right"]))
        demo_steps = int(math.ceil(duration * 20))

        # Check demo mode STEPS >= 1.25x demo length
        self.assertGreaterEqual(modes["demo"]["steps"], int(demo_steps * 1.25))

        # Check dream mode STEPS is within run limit
        self.assertLessEqual(modes["dream"]["steps"], 2400)
        self.assertGreaterEqual(modes["dream"]["steps"], modes["demo"]["steps"])

    def test_demo_and_ablations_predicted_scores(self) -> None:
        """Verify logic and scoring for every mode."""
        # 1. settle -> 0.0
        scorer_settle = Scorer()
        res_settle = scorer_settle.update(g.BASE_POS, [0, 0, 0, 1], {f"btn_{c}": 0.0 for c in g.CORNERS})
        self.assertEqual(res_settle.score, 0.0)

        # 2. demo -> 1.0 (with 12 presses in star order)
        scorer_demo = Scorer()
        for c in demo_plan.bolt_press_sequence("demo"):
            scorer_demo.logic.register_press(c)
        self.assertEqual(scorer_demo.logic.completed_passes, 3)
        self.assertFalse(scorer_demo.logic.warp)
        self.assertEqual(scorer_demo.logic.status_lamp, "green")

        # 3. adjacent -> WARP 0.0
        scorer_adj = Scorer()
        for c in demo_plan.bolt_press_sequence("adjacent"):
            scorer_adj.logic.register_press(c)
        self.assertTrue(scorer_adj.logic.warp)
        res_adj = scorer_adj.update(g.BASE_POS, [0, 0, 0, 1], {f"btn_{c}": 0.0 for c in g.CORNERS})
        self.assertEqual(res_adj.score, 0.0)

        # 4. one_arm -> WARP 0.0
        scorer_one = Scorer()
        for c in demo_plan.bolt_press_sequence("one_arm"):
            scorer_one.logic.register_press(c)
        self.assertTrue(scorer_one.logic.warp)
        res_one = scorer_one.update(g.BASE_POS, [0, 0, 0, 1], {f"btn_{c}": 0.0 for c in g.CORNERS})
        self.assertEqual(res_one.score, 0.0)

        # 5. no_star -> WARP 0.0
        scorer_no_star = Scorer()
        for c in demo_plan.bolt_press_sequence("no_star"):
            scorer_no_star.logic.register_press(c)
        self.assertTrue(scorer_no_star.logic.warp)
        res_no_star = scorer_no_star.update(g.BASE_POS, [0, 0, 0, 1], {f"btn_{c}": 0.0 for c in g.CORNERS})
        self.assertEqual(res_no_star.score, 0.0)

    def test_all_action_plans_end_with_open_fingers(self) -> None:
        for mode in ("demo", "adjacent", "one_arm", "no_star"):
            p = demo_plan.demo_plans(mode)
            self.assertLessEqual(p["left"][-1].finger, FINGER_OPEN * 0.9, f"{mode}: left arm must finish open")
            self.assertLessEqual(p["right"][-1].finger, FINGER_OPEN * 0.9, f"{mode}: right arm must finish open")


if __name__ == "__main__":
    import math
    unittest.main()
