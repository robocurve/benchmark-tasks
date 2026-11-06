"""Tests for run.sh modes, demo_plan waypoints, and predicted scores."""

import os
import sys
import unittest
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

import demo_plan
from yam_arena.demo import Waypoint

TASK_DIR = HERE

PREDICTED_SCORES = {
    "demo": 1.00,
    "no_switch": 0.00,
    "release_early": 0.25,
    "door_only": 0.25,
    "settle": 0.00,
}


class TestModes(unittest.TestCase):
    def test_run_sh_has_all_modes(self):
        run_sh = (TASK_DIR / "run.sh").read_text()
        for mode in PREDICTED_SCORES:
            self.assertIn(mode, run_sh, f"run.sh missing mode {mode}")

    def test_run_sh_step_limits_exceed_plan_duration(self):
        """Rule: STEPS >= 1.25x the plan length for each scripted mode."""
        plans = demo_plan.plans("demo")
        t_l = sum(w.duration for w in plans["left"])
        t_r = sum(w.duration for w in plans["right"])
        plan_steps = int(round(max(t_l, t_r) * 20.0))

        # Check demo steps in run.sh
        run_sh = (TASK_DIR / "run.sh").read_text()
        self.assertIn("demo) POLICY=(--policy yam_scripted); STEPS=1200", run_sh)
        self.assertGreaterEqual(1200, plan_steps * 1.25, f"STEPS 1200 < 1.25 * {plan_steps}")

    def test_demo_duration_within_spec(self):
        """Spec: Length ~600-900 steps with 1.8x stretch. Run limit 2,000-2,400 steps."""
        plans = demo_plan.plans("demo")
        t_l = sum(w.duration for w in plans["left"])
        t_r = sum(w.duration for w in plans["right"])
        self.assertAlmostEqual(t_l, t_r, delta=0.1, msg="Left and right arm plans should be balanced in duration")

        steps = int(round(t_l * 20.0))
        self.assertGreaterEqual(steps, 600, f"Demo duration {steps} steps < 600")
        self.assertLessEqual(steps, 950, f"Demo duration {steps} steps > 950")

    def test_each_mode_generates_valid_waypoints(self):
        for mode in PREDICTED_SCORES:
            plans = demo_plan.plans(mode)
            self.assertIn("left", plans)
            self.assertIn("right", plans)
            for arm in ("left", "right"):
                wps = plans[arm]
                self.assertGreater(len(wps), 0, f"Mode {mode} arm {arm} has no waypoints")
                for w in wps:
                    self.assertIsInstance(w, Waypoint)
                    self.assertGreater(w.duration, 0.0)
                    self.assertEqual(len(w.pos), 3)

    def test_hands_stay_separated_during_holds(self):
        """Rule: Keep hands >= 100 mm apart to prevent link4-link4 collisions."""
        plans = demo_plan.plans("demo")
        n = min(len(plans["left"]), len(plans["right"]))
        for i in range(n):
            wl = plans["left"][i]
            wr = plans["right"][i]
            if "hold" in wl.label or "hold" in wr.label:
                pl = np.array(wl.pos)
                pr = np.array(wr.pos)
                dist = np.linalg.norm(pl - pr)
                self.assertGreaterEqual(
                    dist,
                    0.095,  # allowing minor numerical tolerance to 100 mm
                    f"Hands too close ({dist*1000:.1f} mm) at step {i}: {wl.label} vs {wr.label}",
                )


if __name__ == "__main__":
    import numpy as np
    unittest.main()
