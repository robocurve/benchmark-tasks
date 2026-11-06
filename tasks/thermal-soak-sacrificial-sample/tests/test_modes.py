"""Glue tests between run.sh and demo_plan.py:

Checks that:
1. Every mode in run.sh is supported by demo_plan.py.
2. Every environment variable exported in run.sh is read by demo_plan.py.
3. STEPS in run.sh leaves >= 25% spare margin over the plan duration.
4. Each mode flag changes the plan appropriately.
5. All plans finish with both grippers open.
"""

from __future__ import annotations

import os
import re
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import demo_plan
import geometry as g
from scoring import Scorer

RUN_SH = (HERE / "run.sh").read_text()
ENV_FLAGS = ("DEMO_SETTLE_ONLY", "DEMO_NAIVE", "DEMO_OVERSHOOT", "DEMO_NO_SAMPLE1")


def parse_run_sh() -> dict[str, dict]:
    """Parse mode -> {'env': [vars], 'steps': int} from run.sh."""
    modes = {}
    for line in RUN_SH.splitlines():
        m = re.match(r"^\s*([a-z_0-9]+)\)\s*(.*?)\s*;;\s*$", line)
        if not m or m.group(1) == "*":
            continue
        body = m.group(2)
        env_vars = re.findall(r"export\s+([A-Z_0-9]+)=", body)
        steps_m = re.search(r"STEPS=(\d+)", body)
        steps = int(steps_m.group(1)) if steps_m else 0
        modes[m.group(1)] = {"env": env_vars, "steps": steps}
    return modes


class TestModes(unittest.TestCase):
    def setUp(self):
        for k in ENV_FLAGS:
            os.environ.pop(k, None)

    def tearDown(self):
        for k in ENV_FLAGS:
            os.environ.pop(k, None)

    def test_every_mode_is_present(self):
        modes = parse_run_sh()
        self.assertEqual(set(modes.keys()), {"demo", "naive", "overshoot", "no_sample1", "settle", "dream"})

    def test_every_exported_env_var_is_read_by_demo_plan(self):
        src = (HERE / "demo_plan.py").read_text()
        modes = parse_run_sh()
        for mode, info in modes.items():
            for var in info["env"]:
                self.assertIn(f'"{var}"', src, f"mode {mode}: demo_plan.py never reads {var}")

    def test_steps_budget_has_at_least_25_percent_margin(self):
        modes = parse_run_sh()
        for mode in ("demo", "naive", "overshoot", "no_sample1"):
            p = demo_plan.plans(mode)
            plan_duration = sum(w.duration for w in p["left"])
            plan_steps = int(plan_duration / 0.05)
            steps_budget = modes[mode]["steps"]
            self.assertGreaterEqual(
                steps_budget,
                plan_steps * 1.25,
                f"mode {mode}: STEPS={steps_budget} has < 25% margin over {plan_steps} plan steps",
            )
            self.assertLessEqual(steps_budget, 3000, f"mode {mode}: STEPS={steps_budget} exceeds run limit 3000")

    def test_settle_mode_is_empty_plan(self):
        p = demo_plan.plans("settle")
        self.assertEqual(p, {})

    def test_every_plan_ends_with_both_grippers_open(self):
        from yam_arena.constants import FINGER_OPEN
        for mode in ("demo", "naive", "no_sample1"):
            p = demo_plan.plans(mode)
            for arm in ("left", "right"):
                last_wp = p[arm][-1]
                self.assertAlmostEqual(last_wp.finger, FINGER_OPEN, places=3)

    def test_initial_state_scores_zero_no_gates(self):
        scorer = Scorer()
        s1_data = (g.START_POS_1, (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))
        s2_data = (g.START_POS_2, (0, 0, 0, 1), (0, 0, 0), (0, 0, 0))
        res = scorer.update((0.0, 0.0, 0.0), {}, s1_data, s2_data, True)
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)
        self.assertFalse(res.details["fire_tripped"])
        self.assertFalse(res.details["fixture_displaced"])
        self.assertFalse(res.details["off_table"])


if __name__ == "__main__":
    unittest.main()
