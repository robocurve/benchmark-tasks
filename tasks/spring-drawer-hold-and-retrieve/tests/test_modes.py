"""Unit tests for demo modes in spring-drawer-hold-and-retrieve."""
import os
import sys
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import demo_plan as D


class TestModes(unittest.TestCase):
    def test_mode_from_env_defaults(self):
        for k in ("DEMO_SETTLE_ONLY", "DEMO_NO_HOLD", "DEMO_PINCH", "DEMO_LEAVE_OPEN"):
            os.environ.pop(k, None)
        self.assertEqual(D.mode_from_env(), "demo")

    def test_mode_from_env_flags(self):
        os.environ["DEMO_SETTLE_ONLY"] = "1"
        self.assertEqual(D.mode_from_env(), "settle")
        os.environ.pop("DEMO_SETTLE_ONLY")

        os.environ["DEMO_NO_HOLD"] = "1"
        self.assertEqual(D.mode_from_env(), "no_hold")
        os.environ.pop("DEMO_NO_HOLD")

        os.environ["DEMO_PINCH"] = "1"
        self.assertEqual(D.mode_from_env(), "pinch")
        os.environ.pop("DEMO_PINCH")

        os.environ["DEMO_LEAVE_OPEN"] = "1"
        self.assertEqual(D.mode_from_env(), "leave_open")
        os.environ.pop("DEMO_LEAVE_OPEN")

    def test_plans_for_modes(self):
        for mode in ("demo", "no_hold", "pinch", "leave_open", "settle"):
            p = D.plans(mode)
            self.assertIn("left", p)
            self.assertIn("right", p)
            self.assertGreater(len(p["left"]), 0, f"Mode {mode} left empty")
            self.assertGreater(len(p["right"]), 0, f"Mode {mode} right empty")


if __name__ == "__main__":
    unittest.main()

