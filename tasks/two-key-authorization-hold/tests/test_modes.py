"""Unit tests for demo plans and mode selection via environment flags."""

from __future__ import annotations

import os
import sys
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import demo_plan
from yam_arena.demo import Waypoint


class TestTwoKeyModes(unittest.TestCase):
    def test_mode_from_env_defaults(self):
        for k in ("DEMO_SETTLE_ONLY", "DEMO_SEQUENTIAL", "DEMO_ONE_KEY", "DEMO_RELEASE_EARLY", "DEMO_NO_CONFIRM"):
            os.environ.pop(k, None)
        self.assertEqual(demo_plan.mode_from_env(), "demo")

    def test_mode_from_env_flags(self):
        os.environ["DEMO_SETTLE_ONLY"] = "1"
        self.assertEqual(demo_plan.mode_from_env(), "settle")
        os.environ.pop("DEMO_SETTLE_ONLY")

        os.environ["DEMO_SEQUENTIAL"] = "1"
        self.assertEqual(demo_plan.mode_from_env(), "sequential")
        os.environ.pop("DEMO_SEQUENTIAL")

        os.environ["DEMO_ONE_KEY"] = "1"
        self.assertEqual(demo_plan.mode_from_env(), "one_key")
        os.environ.pop("DEMO_ONE_KEY")

        os.environ["DEMO_RELEASE_EARLY"] = "1"
        self.assertEqual(demo_plan.mode_from_env(), "release_early")
        os.environ.pop("DEMO_RELEASE_EARLY")

        os.environ["DEMO_NO_CONFIRM"] = "1"
        self.assertEqual(demo_plan.mode_from_env(), "no_confirm")
        os.environ.pop("DEMO_NO_CONFIRM")

    def test_plans_no_arg_reads_env(self):
        """Lesson 4: plans() with no argument reads all mode flags from environment variables."""
        os.environ["DEMO_SETTLE_ONLY"] = "1"
        p = demo_plan.plans()
        self.assertEqual(len(p["left"]), 1)
        self.assertIn("settle", p["left"][0].label)
        os.environ.pop("DEMO_SETTLE_ONLY")

    def test_all_modes_return_plans(self):
        for m in ("demo", "sequential", "one_key", "release_early", "no_confirm", "settle", "dream"):
            p = demo_plan.plans(m)
            self.assertIn("left", p)
            self.assertIn("right", p)
            self.assertGreater(len(p["left"]), 0)
            self.assertGreater(len(p["right"]), 0)
            self.assertTrue(all(isinstance(w, Waypoint) for w in p["left"]))
            self.assertTrue(all(isinstance(w, Waypoint) for w in p["right"]))


if __name__ == "__main__":
    unittest.main()
