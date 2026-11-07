"""Tests for bimanual demonstration modes and waypoint plans."""

import unittest
import demo_plan
from yam_arena.demo import Waypoint


class TestModes(unittest.TestCase):
    MODES = ("demo", "wrong_order", "offset", "leave_small", "knock", "settle")

    def test_all_modes_generate_valid_plans(self):
        for mode in self.MODES:
            p = demo_plan.plans(mode)
            self.assertIn("left", p)
            self.assertIn("right", p)
            self.assertGreater(len(p["left"]), 0, f"mode {mode} left empty")
            self.assertGreater(len(p["right"]), 0, f"mode {mode} right empty")

            t_left = sum(w.duration for w in p["left"])
            t_right = sum(w.duration for w in p["right"])
            self.assertAlmostEqual(t_left, t_right, delta=0.1, msg=f"mode {mode} desynchronized: {t_left} vs {t_right}")

    def test_waypoint_properties(self):
        p = demo_plan.plans("demo")
        for arm in ("left", "right"):
            for w in p[arm]:
                self.assertIsInstance(w, Waypoint)
                self.assertGreater(w.duration, 0.0)
                self.assertEqual(len(w.pos), 3)
                self.assertTrue(-0.05 <= w.finger <= 0.0)


if __name__ == "__main__":
    unittest.main()

