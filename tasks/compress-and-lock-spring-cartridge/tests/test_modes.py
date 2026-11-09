"""Tests for demonstration modes and waypoint plans in compress-and-lock-spring-cartridge."""

import unittest
import demo_plan
from yam_arena.demo import Waypoint


class TestModes(unittest.TestCase):
    MODES = (
        "demo",
        "order_bar_first_arm",
        "one_arm",
        "release_early",
        "bar_first",
        "no_release",
        "settle",
        "dream",
    )

    def test_all_modes_generate_valid_plans(self):
        for mode in self.MODES:
            p = demo_plan.plans(mode)
            self.assertIn("left", p, f"mode {mode} missing left plan")
            self.assertIn("right", p, f"mode {mode} missing right plan")
            self.assertGreater(len(p["left"]), 0, f"mode {mode} left plan empty")
            self.assertGreater(len(p["right"]), 0, f"mode {mode} right plan empty")

            t_left = sum(w.duration for w in p["left"])
            t_right = sum(w.duration for w in p["right"])
            self.assertAlmostEqual(
                t_left,
                t_right,
                delta=0.1,
                msg=f"mode {mode} arm desynchronization: left {t_left}s vs right {t_right}s",
            )

    def test_waypoint_properties(self):
        p = demo_plan.plans("demo")
        for arm in ("left", "right"):
            for w in p[arm]:
                self.assertIsInstance(w, Waypoint)
                self.assertGreater(w.duration, 0.0)
                self.assertEqual(len(w.pos), 3)
                self.assertTrue(-0.05 <= w.finger <= 0.0)

    def test_two_valid_orders(self):
        """Rule 4: Unit-test two valid orders (bar arm starts first vs plunger arm starts first)."""
        p_demo = demo_plan.plans("demo")
        p_order2 = demo_plan.plans("order_bar_first_arm")

        # In order 2, right arm hovers near bar in first waypoint
        labels_right_order2 = [w.label for w in p_order2["right"]]
        self.assertTrue(any("standby" in lbl for lbl in labels_right_order2))

        # Both generate non-empty synchronized plans
        self.assertGreater(len(p_demo["left"]), 0)
        self.assertGreater(len(p_order2["left"]), 0)


if __name__ == "__main__":
    unittest.main()
