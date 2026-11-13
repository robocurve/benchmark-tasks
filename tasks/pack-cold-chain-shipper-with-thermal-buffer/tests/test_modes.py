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
            "lid_unlatched",
            "no_lid",
            "settle",
            "dream",
        )
        self.assertEqual(set(modes), set(demo_plan.MODES))
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

    def _labels(self, mode, variant="A"):
        plans = demo_plan.plans(mode, variant=variant)
        return {arm: [wp.label for wp in wps] for arm, wps in plans.items()}

    def test_waypoints_for_every_mode(self):
        """Which parts each mode touches, for both variants (the demo plan is what the scores rest on)."""
        touched = {
            "demo": {"brick", "buffer", "carton", "lid"},
            "no_buffer_violation": {"brick", "carton"},
            "brick_only": {"brick"},
            "buffer_without_carton": {"brick", "buffer"},
            "lid_unlatched": {"brick", "buffer", "carton", "lid"},
            "no_lid": {"brick", "buffer", "carton"},
            "settle": set(),
            "dream": set(),
        }
        for variant in ("A", "B"):
            lay = demo_plan.layout(variant)
            for mode, parts in touched.items():
                labels = self._labels(mode, variant)
                for name in demo_plan.ALL_PART_NAMES:
                    arm = lay["arms"][name]
                    got = {demo_plan.part_of(lbl) for lbl in labels[arm]} - {None}
                    other = {demo_plan.part_of(lbl) for lbl in labels["right" if arm == "left" else "left"]} - {None}
                    self.assertEqual(name in got, name in parts, f"{variant}/{mode}: {name}")
                    if name in parts:
                        for stage in ("hover_pick", "descend", "grasp", "lift", "approach_shipper", "place", "release", "lift_clear", "return_home"):
                            self.assertIn(f"{stage}_{name}", labels[arm], f"{variant}/{mode}: {stage}_{name}")
                    # the other arm never works on this part
                    self.assertTrue(name not in other or name in got and arm_is_only(labels, arm, name))

    def test_lid_press_is_only_in_the_demo(self):
        for variant in ("A", "B"):
            arm = demo_plan.layout(variant)["arms"]["lid"]
            demo = demo_plan.plans("demo", variant=variant)[arm]
            unlatched = demo_plan.plans("lid_unlatched", variant=variant)[arm]
            demo_place = next(w for w in demo if w.label == "place_lid")
            unlatched_place = next(w for w in unlatched if w.label == "place_lid")
            # the demo presses below the seat (the soft arm gives, the rim stops the lid); the ablation stops at the heads
            self.assertAlmostEqual(demo_place.pos[2], g.LID_PRESS_TCP_Z, places=6)
            self.assertAlmostEqual(unlatched_place.pos[2], g.LID_ON_LATCH_TCP_Z, places=6)
            self.assertLess(demo_place.pos[2], g.LID_SEAT_TCP_Z)
            self.assertGreater(unlatched_place.pos[2], g.LID_SEAT_TCP_Z)
            self.assertIn("unload_lid", [w.label for w in demo])
            self.assertNotIn("unload_lid", [w.label for w in unlatched])

    def test_lid_is_carried_over_the_shipper_and_pressed_in_the_middle(self):
        for variant in ("A", "B"):
            arm = demo_plan.layout(variant)["arms"]["lid"]
            for w in demo_plan.plans("demo", variant=variant)[arm]:
                if w.label in ("lower_lid", "place_lid", "pause_place_lid", "release_lid", "unload_lid", "approach_shipper_lid"):
                    self.assertAlmostEqual(w.pos[0], g.SHIPPER_X, places=6)
                    self.assertAlmostEqual(w.pos[1], g.SHIPPER_Y, places=6)
                if w.label in ("lift_lid", "approach_shipper_lid"):
                    self.assertAlmostEqual(w.pos[2], g.LID_TRANSIT_Z, places=6)

    def test_lid_is_last_and_arms_end_home(self):
        for variant in ("A", "B"):
            for mode in ("demo", "lid_unlatched"):
                plans = demo_plan.plans(mode, variant=variant)
                for arm, home in (("left", g.HOME_L), ("right", g.HOME_R)):
                    self.assertEqual(plans[arm][-1].label, "hold_settle")
                    for a, b in zip(plans[arm][-1].pos, home):
                        self.assertAlmostEqual(a, b, places=6)
                    self.assertGreaterEqual(plans[arm][-1].duration, 5.0)
                arm = demo_plan.layout(variant)["arms"]["lid"]
                labels = [w.label for w in plans[arm]]
                self.assertGreater(labels.index("hover_pick_lid"), labels.index("release_carton"))

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


def arm_is_only(labels, arm, name):
    """True when only ``arm`` has waypoints that work on ``name``."""
    other = "right" if arm == "left" else "left"
    return not any(demo_plan.part_of(lbl) == name for lbl in labels[other])


if __name__ == "__main__":
    unittest.main()
