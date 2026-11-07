"""Unit tests for demo modes and plan generation in estop-before-clearing-jam."""

import os
import sys
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import demo_plan as D


class TestModes(unittest.TestCase):
    def test_mode_from_env_defaults(self):
        for k in ("DEMO_SETTLE_ONLY", "DEMO_NO_ESTOP", "DEMO_START_EARLY", "DEMO_SKIP_RESET", "DEMO_LEAVE_STOPPED", "DEMO_DREAM"):
            os.environ.pop(k, None)
        self.assertEqual(D.mode_from_env(), "demo")

    def test_mode_from_env_flags(self):
        os.environ["DEMO_SETTLE_ONLY"] = "1"
        self.assertEqual(D.mode_from_env(), "settle")
        os.environ.pop("DEMO_SETTLE_ONLY")

        os.environ["DEMO_NO_ESTOP"] = "1"
        self.assertEqual(D.mode_from_env(), "no_estop")
        os.environ.pop("DEMO_NO_ESTOP")

        os.environ["DEMO_START_EARLY"] = "1"
        self.assertEqual(D.mode_from_env(), "start_early")
        os.environ.pop("DEMO_START_EARLY")

        os.environ["DEMO_SKIP_RESET"] = "1"
        self.assertEqual(D.mode_from_env(), "skip_reset")
        os.environ.pop("DEMO_SKIP_RESET")

        os.environ["DEMO_LEAVE_STOPPED"] = "1"
        self.assertEqual(D.mode_from_env(), "leave_stopped")
        os.environ.pop("DEMO_LEAVE_STOPPED")

        os.environ["DEMO_DREAM"] = "1"
        self.assertEqual(D.mode_from_env(), "dream")
        os.environ.pop("DEMO_DREAM")

    def test_plans_for_modes(self):
        for mode in ("demo", "no_estop", "start_early", "skip_reset", "leave_stopped", "settle", "dream"):
            p = D.plans(mode)
            self.assertIn("left", p)
            self.assertIn("right", p)
            self.assertGreater(len(p["left"]), 0, f"Mode {mode} left empty")
            self.assertGreater(len(p["right"]), 0, f"Mode {mode} right empty")


if __name__ == "__main__":
    unittest.main()
