"""Unit tests for demonstration modes and waypoint plans in interlocked-drawer-transfer."""

import os
import unittest

import demo_plan as D
from yam_arena.demo import Waypoint


class TestModes(unittest.TestCase):
    MODES = ("demo", "park", "both_open", "leave_open", "wrong_drawer", "block_dropped", "settle", "dream")

    def test_mode_from_env_defaults(self):
        for k in ("DEMO_SETTLE_ONLY", "DEMO_BOTH_OPEN", "DEMO_LEAVE_OPEN", "DEMO_WRONG_DRAWER", "DEMO_BLOCK_DROPPED", "DEMO_PARK"):
            os.environ.pop(k, None)
        self.assertEqual(D.mode_from_env(), "demo")

    def test_mode_from_env_flags(self):
        env_map = {
            "DEMO_SETTLE_ONLY": "settle",
            "DEMO_BOTH_OPEN": "both_open",
            "DEMO_LEAVE_OPEN": "leave_open",
            "DEMO_WRONG_DRAWER": "wrong_drawer",
            "DEMO_BLOCK_DROPPED": "block_dropped",
            "DEMO_PARK": "park",
        }
        for var, expected_mode in env_map.items():
            os.environ[var] = "1"
            self.assertEqual(D.mode_from_env(), expected_mode)
            os.environ.pop(var)

    def test_all_modes_generate_valid_plans(self):
        for mode in self.MODES:
            p = D.plans(mode)
            self.assertIn("left", p, f"mode {mode} missing left")
            self.assertIn("right", p, f"mode {mode} missing right")
            self.assertGreater(len(p["left"]), 0, f"mode {mode} left empty")
            self.assertGreater(len(p["right"]), 0, f"mode {mode} right empty")

            t_left = sum(w.duration for w in p["left"])
            t_right = sum(w.duration for w in p["right"])
            self.assertAlmostEqual(t_left, t_right, delta=0.1, msg=f"mode {mode} desynchronized: {t_left} vs {t_right}")

    def test_waypoint_properties(self):
        p = D.plans("demo")
        for arm in ("left", "right"):
            for w in p[arm]:
                self.assertIsInstance(w, Waypoint)
                self.assertGreater(w.duration, 0.0)
                self.assertEqual(len(w.pos), 3)
                self.assertTrue(-0.05 <= w.finger <= 0.0)


if __name__ == "__main__":
    unittest.main()
