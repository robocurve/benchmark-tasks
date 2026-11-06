"""Tests for demo_plan modes, env var bindings, and duration limits."""

import os
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

import demo_plan

MODE_ENV_VARS = ["DEMO_SETTLE_ONLY", "DEMO_NAIVE", "DEMO_NO_LOCK"]
RUN_LIMIT_STEPS = 2400


class TestModes(unittest.TestCase):
    def setUp(self):
        self._orig = {k: os.environ.get(k) for k in MODE_ENV_VARS}
        for k in MODE_ENV_VARS:
            os.environ.pop(k, None)

    def tearDown(self):
        for k, v in self._orig.items():
            if v is not None:
                os.environ[k] = v
            else:
                os.environ.pop(k, None)

    def test_mode_from_env_default(self):
        self.assertEqual(demo_plan.mode_from_env(), "demo")

    def test_mode_from_env_flags(self):
        for var, mode in (("DEMO_SETTLE_ONLY", "settle"), ("DEMO_NAIVE", "naive"), ("DEMO_NO_LOCK", "no_lock")):
            os.environ[var] = "1"
            self.assertEqual(demo_plan.mode_from_env(), mode)
            os.environ.pop(var, None)

    def test_plans_generation_all_modes(self):
        for mode in ("demo", "naive", "no_lock", "settle"):
            seqs = demo_plan.plans(mode)
            self.assertIn("left", seqs)
            self.assertIn("right", seqs)
            steps = demo_plan.duration_steps(seqs)
            self.assertGreater(steps, 0)
            self.assertLess(steps, RUN_LIMIT_STEPS)

    def test_demo_length_in_target_range(self):
        seqs = demo_plan.plans("demo")
        steps = demo_plan.duration_steps(seqs)
        # Target range: 600 - 1200 steps
        self.assertGreaterEqual(steps, 600)
        self.assertLessEqual(steps, 1200)

    def test_demo_push_overshoot_env_exposed(self):
        self.assertEqual(demo_plan.GPU_PUSH_OVERSHOOT_MM, 6.5)  # right dial (GPU run 4)
        self.assertEqual(demo_plan.GPU_LEFT_OVERSHOOT_MM, 6.0)
        self.assertEqual(demo_plan.GPU_LEFT_SPEED_MM_S, 20.0)
        os.environ["DEMO_PUSH_OVERSHOOT_MM"] = "6.0"
        self.assertAlmostEqual(demo_plan._f("DEMO_PUSH_OVERSHOOT_MM", 8.0), 6.0)
        os.environ.pop("DEMO_PUSH_OVERSHOOT_MM", None)


if __name__ == "__main__":
    unittest.main()
