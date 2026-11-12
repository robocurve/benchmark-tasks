"""Unit tests that every pick and place is physically graspable by the YAM gripper (pad tips, pinch width, free space)."""

from __future__ import annotations

import itertools
import math
import os
import sys
import unittest

import numpy as np

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
import task
from yam_arena import constants as yc
from yam_arena.demo import grip_rotation

SIZES = {
    "brick": (g.BRICK_WIDTH_X, g.BRICK_LENGTH_Y, g.BRICK_HEIGHT_Z),
    "buffer": (g.BUFFER_WIDTH_X, g.BUFFER_LENGTH_Y, g.BUFFER_HEIGHT_Z),
    "carton": (g.CARTON_WIDTH_X, g.CARTON_LENGTH_Y, g.CARTON_HEIGHT_Z),
}
# the surface each part rests on when it is placed (top of the layer below), by mode
SEAT_BOTTOM = {
    ("demo", "brick"): g.CAVITY_FLOOR_Z,
    ("demo", "buffer"): g.BRICK_TOP_SEATED_Z,
    ("demo", "carton"): g.BUFFER_TOP_SEATED_Z,
    ("no_buffer_violation", "brick"): g.CAVITY_FLOOR_Z,
    ("no_buffer_violation", "carton"): g.BRICK_TOP_SEATED_Z,
}
PICK_MODES = ("demo", "no_buffer_violation", "brick_only", "buffer_without_carton")


def _pinched_width(name: str, yaw: float) -> float:
    """Part extent along the closing axis of a grip turned ``yaw`` (yaw 0 closes along world Y)."""
    sx, sy, _ = SIZES[name]
    return abs(math.sin(yaw)) * sx + abs(math.cos(yaw)) * sy


class TestGrasp(unittest.TestCase):
    def test_constants_match_the_gripper(self):
        self.assertAlmostEqual(g.PAD_TIP_PAST_TCP, yc.PAD_TIP_PAST_TCP, places=6)
        self.assertAlmostEqual(g.FINGER_OPEN, yc.FINGER_OPEN, places=6)
        self.assertAlmostEqual(-2.0 * g.FINGER_OPEN, g.GRIPPER_MAX_OPEN, places=3)

    def test_closing_axis_is_world_x(self):
        for yaw in (g.GRIP_YAW_LEFT, g.GRIP_YAW_RIGHT):
            axis = grip_rotation(yaw)[:, 0]
            self.assertAlmostEqual(abs(axis[0]), 1.0, places=4)
            self.assertAlmostEqual(axis[1], 0.0, places=4)

    def test_every_pick_waypoint_pinches_the_part(self):
        checked = 0
        for variant, mode in itertools.product(("A", "B"), PICK_MODES):
            plans = demo_plan.plans(mode, variant=variant)
            lay = demo_plan.layout(variant)
            for arm, wps in plans.items():
                for w in wps:
                    if not w.label.startswith("grasp_"):
                        continue
                    name = demo_plan.part_of(w.label)
                    self.assertEqual(lay["arms"][name], arm)
                    x, y, tcp_z = w.pos
                    # the TCP is over the part centre on its staging pad
                    self.assertAlmostEqual(x, lay["staging"][name][0], places=4)
                    self.assertAlmostEqual(y, lay["staging"][name][1], places=4)
                    # pad tips are 13 mm below the TCP and must lie within the part's side height
                    tip = tcp_z - yc.PAD_TIP_PAST_TCP
                    bottom = g.PAD_SURFACE_Z + g.SPAWN_GAP
                    height = SIZES[name][2]
                    self.assertGreater(tip, bottom, f"{variant}/{mode}/{w.label}: pad tip below the part bottom")
                    self.assertLess(tip, bottom + height, f"{variant}/{mode}/{w.label}: pad tip above the part top")
                    self.assertGreaterEqual(tip - bottom, 0.003, "pad tips must ride at least 3 mm above the pad plate")
                    self.assertLessEqual(tip - bottom, 0.0045 + 1e-9)
                    self.assertGreaterEqual(bottom + height - tip, g.MIN_ENGAGE, "pad tips engage less than 8 mm of the side")
                    # pinched width fits the open gripper with margin on each side and is <= 80 mm
                    width = _pinched_width(name, w.yaw)
                    self.assertLessEqual(width, 0.080 + 1e-9)
                    self.assertGreaterEqual((g.GRIPPER_MAX_OPEN - width) / 2.0, 0.007 - 1e-9)
                    # the commanded pad gap squeezes the part
                    self.assertLess(-2.0 * w.finger, width)
                    checked += 1
        self.assertEqual(checked, 2 * (3 + 2 + 1 + 2))  # per variant: demo 3, no_buffer 2, brick_only 1, buffer_without_carton 2

    def test_every_place_waypoint_keeps_the_pinch_and_frees_the_fingers(self):
        checked = 0
        for variant, mode in itertools.product(("A", "B"), ("demo", "no_buffer_violation")):
            plans = demo_plan.plans(mode, variant=variant)
            for wps in plans.values():
                for w in wps:
                    if not w.label.startswith("release_"):
                        continue
                    name = demo_plan.part_of(w.label)
                    seat = SEAT_BOTTOM[(mode, name)]
                    height = SIZES[name][2]
                    # released 2 mm above its seat, pad tips inside the part's side height and clear of the layer below
                    tip = w.pos[2] - yc.PAD_TIP_PAST_TCP
                    self.assertAlmostEqual(tip - (seat + g.PLACE_DROP), g.PAD_GAP, places=4)
                    self.assertGreater(tip, seat + g.PLACE_DROP)
                    self.assertLess(tip, seat + g.PLACE_DROP + height)
                    self.assertGreaterEqual(seat + g.PLACE_DROP + height - tip, g.MIN_ENGAGE)
                    # opening on release leaves >= 7 mm of daylight per side around the part
                    width = _pinched_width(name, w.yaw)
                    self.assertGreaterEqual((-2.0 * w.finger - width) / 2.0, 0.007 - 1e-9)
                    self.assertLessEqual(-2.0 * w.finger, g.GRIPPER_MAX_OPEN + 1e-9)
                    checked += 1
        self.assertEqual(checked, 2 * (3 + 2))

    def test_place_is_over_the_cavity_centre_with_free_space_each_side(self):
        for name, (sx, sy, _) in SIZES.items():
            free_x = (g.CAVITY_WIDTH_X - sx) / 2.0
            free_y = (g.CAVITY_LENGTH_Y - sy) / 2.0
            self.assertGreaterEqual(free_x, 0.015)  # fingers go down beside the pinched part
            self.assertGreaterEqual(free_y, 0.004)  # the other axis keeps >= 4 mm per side
        for variant in ("A", "B"):
            for arm, wps in demo_plan.plans("demo", variant=variant).items():
                for w in wps:
                    if w.label.startswith("place_"):
                        self.assertAlmostEqual(w.pos[0], g.SHIPPER_X, places=4)
                        self.assertAlmostEqual(w.pos[1], g.SHIPPER_Y, places=4)

    def test_arm_pauses_at_hover_before_descending(self):
        for variant in ("A", "B"):
            for arm, wps in demo_plan.plans("demo", variant=variant).items():
                labels = [w.label for w in wps]
                for name in demo_plan.PART_NAMES:
                    if f"descend_{name}" not in labels:
                        continue
                    i = labels.index(f"descend_{name}")
                    self.assertEqual(labels[i - 1], f"pause_hover_pick_{name}")
                    self.assertGreaterEqual(wps[i - 1].duration, 0.6)
                    j = labels.index(f"place_{name}")
                    self.assertEqual(labels[j - 1], f"pause_above_shipper_{name}")
                    self.assertGreaterEqual(wps[j - 1].duration, 0.6)

    def test_carried_part_clears_the_rim(self):
        # a carried part rides with its bottom at TRANSIT_Z - GRIP_DZ; it must clear the shipper rim by 15 mm or more
        self.assertGreaterEqual(g.TRANSIT_Z - g.GRIP_DZ - g.CAVITY_RIM_Z, 0.015)

    def test_spawned_parts_overlap_nothing(self):
        for variant in ("A", "B"):
            brick, buffer_pad, carton = task._build_objects_for_variant(variant)
            objs = [task.SHIPPER_BASE, task.PAD_LEFT, task.PAD_RIGHT, task.PAD_FRONT, brick, buffer_pad, carton]
            shapes = []
            for o in objs:
                for centre, size, _col in o.boxes:
                    c = np.asarray(o.pos) + np.asarray(centre)
                    shapes.append((o.name, c - np.asarray(size) / 2.0, c + np.asarray(size) / 2.0))
            for (na, lo_a, hi_a), (nb, lo_b, hi_b) in itertools.combinations(shapes, 2):
                if na == nb:
                    continue
                overlap = np.all(np.minimum(hi_a, hi_b) - np.maximum(lo_a, lo_b) > 1e-9)
                self.assertFalse(overlap, f"variant {variant}: {na} overlaps {nb} at spawn")

    def test_spawned_part_sizes_match_geometry(self):
        brick, buffer_pad, carton = task._build_objects_for_variant("A")
        for obj, name in ((brick, "brick"), (buffer_pad, "buffer"), (carton, "carton")):
            body = obj.boxes[0][1]
            for got, want in zip(body, SIZES[name]):
                self.assertAlmostEqual(got, want, places=6)
            # no decal sticks out sideways: the pinched width is the body width
            for centre, size, _col in obj.boxes:
                self.assertLessEqual(size[0], body[0] + 1e-9)
                self.assertLessEqual(size[1], body[1] + 1e-9)

    def test_variants_differ_in_what_the_robot_sees(self):
        a = task._build_objects_for_variant("A")
        b = task._build_objects_for_variant("B")
        lay_a, lay_b = demo_plan.layout("A"), demo_plan.layout("B")
        self.assertNotEqual(lay_a["staging"]["brick"], lay_b["staging"]["brick"])
        self.assertNotEqual(a[2].boxes, b[2].boxes)  # carton label colour and icon
        for name, obj_a, obj_b in zip(demo_plan.PART_NAMES, a, b):
            self.assertAlmostEqual(obj_a.pos[0], lay_a["staging"][name][0], places=6)
            self.assertAlmostEqual(obj_a.pos[1], lay_a["staging"][name][1], places=6)
            self.assertAlmostEqual(obj_b.pos[0], lay_b["staging"][name][0], places=6)
            self.assertAlmostEqual(obj_b.pos[1], lay_b["staging"][name][1], places=6)


if __name__ == "__main__":
    unittest.main()
