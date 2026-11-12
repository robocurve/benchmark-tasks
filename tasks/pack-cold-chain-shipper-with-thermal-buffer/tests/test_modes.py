"""Unit tests for demonstration plans across all modes in pack-cold-chain-shipper-with-thermal-buffer."""

from __future__ import annotations

import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
TASK_DIR = os.path.dirname(HERE)
if TASK_DIR not in sys.path:
    sys.path.insert(0, TASK_DIR)

if "YAM_ARENA_SRC" in os.environ:
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))
elif os.path.exists(os.path.expanduser("~/Projects/Robocurve/work/refs/yam-isaac-lab-arena")):
    sys.path.insert(0, os.path.expanduser("~/Projects/Robocurve/work/refs/yam-isaac-lab-arena"))

import demo_plan
import geometry as g


class TestModes(unittest.TestCase):
    def test_all_modes_generate_valid_plans_both_variants(self):
        modes = (
            "demo",
            "no_buffer_violation",
            "brick_only",
            "buffer_without_carton",
            "settle",
            "dream",
        )
        for variant in ("A", "B"):
            for mode in modes:
                plans = demo_plan.plans(mode, variant=variant)
                self.assertIn("left", plans, f"left plan missing for mode {mode} (variant {variant})")
                self.assertIn("right", plans, f"right plan missing for mode {mode} (variant {variant})")
                self.assertGreater(len(plans["left"]), 0)
                self.assertGreater(len(plans["right"]), 0)

                # Total durations must be positive and equal across arms
                dur_l = sum(wp.duration for wp in plans["left"])
                dur_r = sum(wp.duration for wp in plans["right"])
                self.assertGreater(dur_l, 0.0)
                self.assertGreater(dur_r, 0.0)
                self.assertAlmostEqual(dur_l, dur_r, places=4)

                # Check position format
                for arm in ("left", "right"):
                    for wp in plans[arm]:
                        self.assertEqual(len(wp.pos), 3)

    def test_demo_sequential_coordination(self):
        plans = demo_plan.plans("demo", variant="A")
        left_labels = [wp.label for wp in plans["left"]]
        right_labels = [wp.label for wp in plans["right"]]

        # In Variant A: Right arm handles brick, left arm handles buffer and carton
        self.assertTrue(any("brick" in lbl.lower() for lbl in right_labels))
        self.assertTrue(any("buffer" in lbl.lower() for lbl in left_labels))
        self.assertTrue(any("carton" in lbl.lower() for lbl in left_labels))

        # Both arms finish at home standby
        self.assertTrue(any("standby" in lbl.lower() or "home" in lbl.lower() for lbl in left_labels))
        self.assertTrue(any("standby" in lbl.lower() or "home" in lbl.lower() for lbl in right_labels))


if __name__ == "__main__":
    unittest.main()
