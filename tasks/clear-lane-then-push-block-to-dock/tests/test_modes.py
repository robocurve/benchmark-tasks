"""Unit tests for demo waypoint plans and operating modes in clear-lane-then-push-block-to-dock."""

import unittest
import numpy as np

import demo_plan


class TestModes(unittest.TestCase):
    def test_required_modes_present(self):
        req = {"demo", "push_first", "hit_post", "stop_halfway", "settle", "dream"}
        self.assertTrue(req.issubset(set(demo_plan.MODES)))

    def test_plans_generation(self):
        for mode in demo_plan.MODES:
            plans = demo_plan.plans(mode)
            self.assertIn("left", plans)
            self.assertIn("right", plans)
            self.assertGreater(len(plans["left"]), 0)
            self.assertGreater(len(plans["right"]), 0)

            for arm in ("left", "right"):
                for wp in plans[arm]:
                    self.assertGreater(wp.duration, 0.0)
                    self.assertTrue(np.all(np.isfinite(wp.pos)))
                    self.assertTrue(np.isfinite(wp.finger))
                    self.assertTrue(np.isfinite(wp.yaw))


if __name__ == "__main__":
    unittest.main()

