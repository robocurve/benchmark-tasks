"""Unit tests for demonstration plan modes in sliding-tile-sort-three-tiles."""

import unittest

import demo_plan

MODES = ["demo", "wrong_order", "lift_tile", "settle", "dream"]


class TestModes(unittest.TestCase):
    def test_plan_generation(self):
        for mode in MODES:
            plans = demo_plan.plans(mode=mode)
            self.assertIn("left", plans)
            self.assertIn("right", plans)
            self.assertGreater(len(plans["left"]), 0, f"left empty for {mode}")
            self.assertGreater(len(plans["right"]), 0, f"right empty for {mode}")

            for arm in ("left", "right"):
                for wp in plans[arm]:
                    self.assertGreater(wp.duration, 0.0, f"zero duration in {mode} {arm} {wp.label}")
                    self.assertEqual(len(wp.pos), 3)


if __name__ == "__main__":
    unittest.main()

