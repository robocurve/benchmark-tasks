"""Unit tests for demo modes in hot-swap-failed-drive-sled."""
import os
import sys
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

import demo_plan as D


class TestModes(unittest.TestCase):
    def test_mode_from_env_defaults(self):
        for k in ("DEMO_SETTLE_ONLY", "DEMO_NO_LATCH", "DEMO_WRONG_BAY", "DEMO_WRONG_SPARE", "DEMO_RELEASE_EARLY"):
            os.environ.pop(k, None)
        self.assertEqual(D.mode_from_env(), "demo")

    def test_mode_from_env_flags(self):
        os.environ["DEMO_SETTLE_ONLY"] = "1"
        self.assertEqual(D.mode_from_env(), "settle")
        os.environ.pop("DEMO_SETTLE_ONLY")

        os.environ["DEMO_NO_LATCH"] = "1"
        self.assertEqual(D.mode_from_env(), "no_latch")
        os.environ.pop("DEMO_NO_LATCH")

        os.environ["DEMO_WRONG_BAY"] = "1"
        self.assertEqual(D.mode_from_env(), "wrong_bay")
        os.environ.pop("DEMO_WRONG_BAY")

        os.environ["DEMO_WRONG_SPARE"] = "1"
        self.assertEqual(D.mode_from_env(), "wrong_spare")
        os.environ.pop("DEMO_WRONG_SPARE")

        os.environ["DEMO_RELEASE_EARLY"] = "1"
        self.assertEqual(D.mode_from_env(), "release_early")
        os.environ.pop("DEMO_RELEASE_EARLY")

    def test_plans_for_modes(self):
        for mode in ("demo", "no_latch", "wrong_bay", "wrong_spare", "release_early", "settle"):
            p = D.plans(mode)
            self.assertIn("left", p)
            self.assertIn("right", p)
            self.assertGreater(len(p["left"]), 0, f"Mode {mode} left empty")
            self.assertGreater(len(p["right"]), 0, f"Mode {mode} right empty")


if __name__ == "__main__":
    unittest.main()

