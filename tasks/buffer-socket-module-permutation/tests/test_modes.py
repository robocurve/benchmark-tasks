"""Unit tests for demo_plan.py run modes, plan durations, and env knobs."""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))
else:
    cand = HERE.parent.parent / "refs" / "yam-isaac-lab-arena"
    if cand.exists():
        sys.path.insert(0, str(cand))

import demo_plan
import geometry as g
from yam_arena.constants import FINGER_OPEN


class TestModes(unittest.TestCase):
    def tearDown(self):
        for k in (
            "DEMO_SETTLE_ONLY",
            "DEMO_NO_COMMIT",
            "DEMO_NO_BUFFER",
            "DEMO_TIME_SCALE",
            "DEMO_OVERSHOOT",
            "DEMO_INSERT_DZ",
            "DEMO_PLUG_DY",
        ):
            os.environ.pop(k, None)

    def test_settle_mode_explicit_hold(self):
        os.environ["DEMO_SETTLE_ONLY"] = "1"
        plans = demo_plan.plans()
        self.assertIn("left", plans)
        self.assertIn("right", plans)
        self.assertEqual(len(plans["left"]), 1)
        self.assertEqual(len(plans["right"]), 1)

        wl = plans["left"][0]
        wr = plans["right"][0]
        self.assertEqual(wl.duration, 8.0)
        self.assertEqual(wr.duration, 8.0)
        self.assertEqual(wl.finger, FINGER_OPEN)
        self.assertEqual(wr.finger, FINGER_OPEN)

    def test_demo_mode_duration_and_steps(self):
        plans = demo_plan.plans("demo")
        self.assertIn("left", plans)
        self.assertIn("right", plans)

        dur_l = sum(w.duration for w in plans["left"])
        dur_r = sum(w.duration for w in plans["right"])
        max_dur = max(dur_l, dur_r)
        steps = max_dur * 20

        # Must be in 900-1400 steps range
        self.assertGreaterEqual(steps, 900)
        self.assertLessEqual(steps, 1400)
        # Margin under 1800-step max execution cap and 2500 run limit
        self.assertLess(steps, 1800)
        self.assertLess(steps, 2500 * 0.75, "demo must leave >= 25% margin under run limit")

    def test_no_commit_mode(self):
        os.environ["DEMO_NO_COMMIT"] = "1"
        plans = demo_plan.plans()
        left_labels = [w.label for w in plans["left"]]
        self.assertFalse(any("press COMMIT" in lbl for lbl in left_labels))

    def test_no_buffer_mode(self):
        os.environ["DEMO_NO_BUFFER"] = "1"
        plans = demo_plan.plans()
        right_labels = [w.label for w in plans["right"]]
        self.assertTrue(any("place green on table" in lbl for lbl in right_labels))

    def test_time_scale_knob(self):
        p_base = demo_plan.plans("demo")
        dur_base = sum(w.duration for w in p_base["left"])

        os.environ["DEMO_TIME_SCALE"] = "1.0"
        p_1x = demo_plan.plans("demo")
        dur_1x = sum(w.duration for w in p_1x["left"])

        self.assertAlmostEqual(dur_base / dur_1x, 1.8, places=2)


if __name__ == "__main__":
    unittest.main()
