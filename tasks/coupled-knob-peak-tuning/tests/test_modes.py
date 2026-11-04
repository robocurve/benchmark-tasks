"""Glue between run.sh and demo_plan.py (cross-cutting C3): every mode's environment variables are read by demo_plan, every
mode builds the plan it claims to, STEPS leaves >= 25 percent spare over the replayed run length, and the initial state
scores 0 with no gate."""

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
import logic
import predict_scores

try:
    import yam_arena  # noqa: F401
    from yam_arena import task_file
    from parts import PartState
except ImportError:
    PartState = None
    task_file = None

RUN_SH = (HERE / "run.sh").read_text()
ENV_FLAGS = ("DEMO_SETTLE_ONLY", "DEMO_NO_READ", "DEMO_AXIS_SEARCH", "DEMO_MEMORIZE", "DEMO_RECOVER")


def parse_run_sh() -> dict[str, dict]:
    """mode -> {'env': [exported names], 'steps': int} parsed from the case block of run.sh."""
    modes = {}
    for line in RUN_SH.splitlines():
        m = re.match(r"^\s*([a-z_]+)\)\s*(.*?)\s*;;\s*$", line)
        if not m or m.group(1) == "*":
            continue
        body = m.group(2)
        modes[m.group(1)] = dict(env=re.findall(r"export\s+([A-Z_]+)=", body), steps=int(re.search(r"STEPS=(\d+)", body).group(1)))
    return modes


def labels(mode, variant=0):
    seqs = demo_plan.plans(mode, variant)
    return [w.label for arm in seqs.values() for w in arm]


class TestRunShVersusDemoPlan(unittest.TestCase):
    def setUp(self):
        for k in ENV_FLAGS:
            os.environ.pop(k, None)

    def tearDown(self):
        for k in ENV_FLAGS:
            os.environ.pop(k, None)

    def test_every_mode_is_present(self):
        modes = parse_run_sh()
        self.assertEqual(set(modes), {"settle", "demo", "recover", "axis_search", "no_read", "memorize", "dream"})

    def test_every_exported_variable_is_read_by_demo_plan(self):
        src = (HERE / "demo_plan.py").read_text()
        for mode, info in parse_run_sh().items():
            for var in info["env"]:
                self.assertIn(f'"{var}"', src, f"mode {mode}: demo_plan.py never reads {var}")
        self.assertIn("TASK_VARIANT", RUN_SH)
        self.assertIn("TASK_VARIANT", (HERE / "logic.py").read_text())

    def test_each_flag_selects_its_mode_and_changes_the_plan(self):
        default = labels("demo")
        for flag, mode in (("DEMO_SETTLE_ONLY", "settle"), ("DEMO_NO_READ", "no_read"), ("DEMO_AXIS_SEARCH", "axis_search"),
                           ("DEMO_MEMORIZE", "memorize"), ("DEMO_RECOVER", "recover")):
            os.environ[flag] = "1"
            self.assertEqual(demo_plan.mode_from_env(), mode)
            env_plan = [w.label for arm in demo_plan.plans().values() for w in arm]
            self.assertEqual(env_plan, labels(mode))
            self.assertNotEqual(env_plan, default, f"{flag} does not change the plan")
            os.environ.pop(flag)

    def test_mode_contents_match_their_claims(self):
        n_reads = lambda m, v=0: sum(1 for lb in labels(m, v) if "press btn" in lb)
        self.assertEqual(n_reads("demo"), 5 * 1)  # one 'press btn' waypoint per READ
        self.assertEqual(n_reads("settle"), 0)
        self.assertEqual(n_reads("no_read"), 0)
        self.assertEqual(n_reads("memorize"), 3)
        self.assertEqual(n_reads("axis_search"), 8)
        self.assertEqual(n_reads("recover"), 6)
        # recover contains a backward push
        ops = demo_plan.build_ops("recover")
        self.assertTrue(any(o[0] == "push" and o[3] < o[2] for o in ops))
        self.assertFalse(any(o[0] == "push" and o[3] < o[2] for o in demo_plan.build_ops("demo")))
        # every plan ends with both grippers open
        for m in ("demo", "recover", "axis_search", "no_read", "memorize", "settle"):
            for arm in demo_plan.plans(m).values():
                self.assertEqual(arm[-1].finger, demo_plan.FINGER_OPEN)

    def test_steps_leave_25_percent_over_the_replayed_run(self):
        modes = parse_run_sh()
        self.assertEqual(modes["dream"]["steps"], 2400)
        for mode in ("demo", "recover", "axis_search", "no_read", "memorize", "settle"):
            r = predict_scores.replay(mode, 0)
            need = max(r["run_end"], r["plan_steps"])
            self.assertGreaterEqual(modes[mode]["steps"], 1.25 * need, f"{mode}: STEPS {modes[mode]['steps']} vs run {need}")
        logic.set_variant(0)

    def test_demo_ends_inside_its_plan_and_inside_the_run_limit(self):
        r = predict_scores.replay("demo", 0)
        self.assertEqual(r["final"], 1.0)
        self.assertLessEqual(r["run_end"], r["plan_steps"], "the harness hold must fit inside the final hold")
        self.assertLess(r["plan_steps"], 0.75 * 2400)

    def test_scorer_runs_once_per_step_hold_lasts_exactly_20_steps(self):
        r = predict_scores.replay("demo", 0)
        # first 1.0 is exactly hold_steps after the scorer first sees the full success condition
        self.assertIsNotNone(r["first_success"])
        self.assertEqual(r["run_end"] - r["first_success"], 20)


@unittest.skipIf(task_file is None, "yam_arena not importable")
class TestGlueOnRealFixturePose(unittest.TestCase):
    def test_initial_state_scores_zero_without_gate(self):
        task = task_file.load(HERE / "task.py")
        mod = task.module
        mod._reset_scorer()
        mk = lambda pos, sl=None: PartState(pos=np.array(pos, float), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3),
                                            ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3), sliders=dict(sl or {}))
        objects = {"fixture": mk(g.BASE_POS, {"read_btn": 0.0}), "knob_left": mk(g.START_POS["left"]),
                   "knob_right": mk(g.START_POS["right"]), "needle": mk(g.needle_pos_from_r(0.0))}
        res = mod._update(objects)
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.details["zero_gate"])


if __name__ == "__main__":
    unittest.main()
