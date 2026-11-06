"""Unit tests for demo modes in weigh-and-reject-heavy-block."""
import os
import sys
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import demo_plan as D


class TestModes(unittest.TestCase):
    def test_mode_from_env_defaults(self):
        for k in ("DEMO_SETTLE_ONLY", "DEMO_NO_WEIGH", "DEMO_WRONG_PICK", "DEMO_WEIGH_TOO_SHORT"):
            os.environ.pop(k, None)
        self.assertEqual(D.mode_from_env(), "demo")

    def test_mode_from_env_flags(self):
        os.environ["DEMO_SETTLE_ONLY"] = "1"
        self.assertEqual(D.mode_from_env(), "settle")
        os.environ.pop("DEMO_SETTLE_ONLY")

        os.environ["DEMO_NO_WEIGH"] = "1"
        self.assertEqual(D.mode_from_env(), "no_weigh")
        os.environ.pop("DEMO_NO_WEIGH")

        os.environ["DEMO_WRONG_PICK"] = "1"
        self.assertEqual(D.mode_from_env(), "wrong_pick")
        os.environ.pop("DEMO_WRONG_PICK")

        os.environ["DEMO_WEIGH_TOO_SHORT"] = "1"
        self.assertEqual(D.mode_from_env(), "weigh_too_short")
        os.environ.pop("DEMO_WEIGH_TOO_SHORT")

    def test_plans_for_modes(self):
        for mode in ("demo", "wrong_pick", "no_weigh", "weigh_too_short", "settle"):
            p = D.plans(mode)
            self.assertIn("left", p)
            self.assertIn("right", p)
            self.assertGreater(len(p["left"]), 0, f"Mode {mode} left empty")
            self.assertGreater(len(p["right"]), 0, f"Mode {mode} right empty")


if __name__ == "__main__":
    unittest.main()
