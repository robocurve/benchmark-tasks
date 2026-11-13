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
    ("lid_unlatched", "brick"): g.CAVITY_FLOOR_Z,
    ("lid_unlatched", "buffer"): g.BRICK_TOP_SEATED_Z,
    ("lid_unlatched", "carton"): g.BUFFER_TOP_SEATED_Z,
    ("no_lid", "brick"): g.CAVITY_FLOOR_Z,
    ("no_lid", "buffer"): g.BRICK_TOP_SEATED_Z,
    ("no_lid", "carton"): g.BUFFER_TOP_SEATED_Z,
}
PICK_MODES = ("demo", "no_buffer_violation", "brick_only", "buffer_without_carton", "lid_unlatched", "no_lid")
PLACE_MODES = ("demo", "no_buffer_violation", "lid_unlatched", "no_lid")
LID_SIZE = (g.LID_WIDTH_X, g.LID_LENGTH_Y, g.LID_THICK_Z)
TAB_SIZE = (g.LID_TAB_X, g.LID_TAB_Y, g.LID_TAB_Z)


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
                    if name == "lid":
                        # the lid is pinched by its grip tab: pad tips 5 mm above the slab's top face, 17 mm up the tab
                        self.assertAlmostEqual(tip, bottom + g.LID_THICK_Z + g.LID_PAD_GAP, places=6)
                        self.assertGreaterEqual(tip - (bottom + g.LID_THICK_Z), 0.004, "pad tips must clear the lid slab by 4 mm")
                        self.assertGreaterEqual(bottom + g.LID_THICK_Z + g.LID_TAB_Z - tip, g.MIN_ENGAGE)
                        width = g.LID_TAB_X
                        self.assertLessEqual(width, 0.080 + 1e-9)
                        self.assertGreaterEqual((g.GRIPPER_MAX_OPEN - width) / 2.0, 0.007 - 1e-9)
                        self.assertLess(-2.0 * w.finger, width)
                        # fingers close along world X, across the tab's 40 mm
                        self.assertAlmostEqual(abs(grip_rotation(w.yaw)[0, 0]), 1.0, places=4)
                        checked += 1
                        continue
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
        # per variant: demo 4, no_buffer 2, brick_only 1, buffer_without_carton 2, lid_unlatched 4, no_lid 3
        self.assertEqual(checked, 2 * (4 + 2 + 1 + 2 + 4 + 3))

    def test_every_place_waypoint_keeps_the_pinch_and_frees_the_fingers(self):
        checked = 0
        for variant, mode in itertools.product(("A", "B"), PLACE_MODES):
            plans = demo_plan.plans(mode, variant=variant)
            for wps in plans.values():
                for w in wps:
                    if not w.label.startswith("release_"):
                        continue
                    name = demo_plan.part_of(w.label)
                    if name == "lid":
                        # released with the pad tips 5 mm over the lid's top face, wherever the lid rests
                        rest_bottom = g.LID_SEAT_BOTTOM_Z if mode == "demo" else g.LID_ON_LATCH_BOTTOM_Z
                        tip = w.pos[2] - yc.PAD_TIP_PAST_TCP
                        self.assertAlmostEqual(tip, rest_bottom + g.LID_THICK_Z + g.LID_PAD_GAP, places=4)
                        self.assertGreaterEqual(rest_bottom + g.LID_THICK_Z + g.LID_TAB_Z - tip, g.MIN_ENGAGE)
                        # opening on release leaves >= 7 mm of daylight per side around the 40 mm tab
                        self.assertGreaterEqual((-2.0 * w.finger - g.LID_TAB_X) / 2.0, 0.007 - 1e-9)
                        self.assertLessEqual(-2.0 * w.finger, g.GRIPPER_MAX_OPEN + 1e-9)
                        checked += 1
                        continue
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
        # per variant: demo 4, no_buffer 2, lid_unlatched 4, no_lid 3
        self.assertEqual(checked, 2 * (4 + 2 + 4 + 3))

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

    def test_carried_part_clears_the_latch_heads(self):
        # the latch heads stand 23.5 mm above the rim; a carried layer must clear them by 15 mm or more (a loaded arm
        # tracks about 10 mm below a commanded height), and so must the empty fingers lifting away after a release
        self.assertGreaterEqual(g.TRANSIT_Z - g.GRIP_DZ - g.LATCH_HEAD_TOP_Z, 0.015)
        self.assertGreaterEqual(g.RETURN_Z - g.PAD_TIP_PAST_TCP - g.LATCH_HEAD_TOP_Z, 0.015)

    def test_spawned_parts_overlap_nothing(self):
        for variant in ("A", "B"):
            brick, buffer_pad, carton = task._build_objects_for_variant(variant)
            lid, pad_lid = task._build_lid(variant), task._build_lid_pad(variant)
            objs = [task.SHIPPER_BASE, task.PAD_LEFT, task.PAD_RIGHT, task.PAD_FRONT, pad_lid, brick, buffer_pad, carton, lid]
            shapes = []
            for o in objs:
                for centre, size, _col in o.boxes:
                    c = np.asarray(o.pos) + np.asarray(centre)
                    shapes.append((o.name, c - np.asarray(size) / 2.0, c + np.asarray(size) / 2.0))
                for s_ in getattr(o, "sliders", ()):
                    for centre, size, _col in s_.boxes:
                        c = np.asarray(o.pos) + np.asarray(centre)
                        shapes.append((o.name + "/" + s_.name, c - np.asarray(size) / 2.0, c + np.asarray(size) / 2.0))
                    for centre, radius, height, _col, axis in s_.cylinders:
                        half = np.array([height / 2.0, radius, radius])  # a rod along X
                        c = np.asarray(o.pos) + np.asarray(centre)
                        shapes.append((o.name + "/" + s_.name, c - half, c + half))
            for (na, lo_a, hi_a), (nb, lo_b, hi_b) in itertools.combinations(shapes, 2):
                if na.split("/")[0] == nb.split("/")[0]:
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
        self.assertNotEqual(lay_a["staging"]["lid"], lay_b["staging"]["lid"])  # the lid is on the other side
        self.assertEqual(lay_a["staging"]["lid"][1], -lay_b["staging"]["lid"][1])
        self.assertNotEqual(a[2].boxes, b[2].boxes)  # carton label colour and icon
        for name, obj_a, obj_b in zip(demo_plan.PART_NAMES, a, b):
            self.assertAlmostEqual(obj_a.pos[0], lay_a["staging"][name][0], places=6)
            self.assertAlmostEqual(obj_a.pos[1], lay_a["staging"][name][1], places=6)
            self.assertAlmostEqual(obj_b.pos[0], lay_b["staging"][name][0], places=6)
            self.assertAlmostEqual(obj_b.pos[1], lay_b["staging"][name][1], places=6)

    def test_lid_tab_grasp_rules(self):
        # the tab is the only thing pinched: 40 mm across (<= 80 mm), 22 mm tall, wider along Y than the 24 mm finger pad
        self.assertAlmostEqual(g.LID_TAB_X, 0.040, places=6)
        self.assertGreaterEqual(g.LID_TAB_Z, 0.020)
        self.assertLessEqual(g.LID_TAB_Z, 0.025)
        self.assertGreaterEqual(g.LID_TAB_Y, g.FINGER_WIDTH)
        # the commanded pad gap squeezes the tab; the released gap leaves >= 7 mm per side (25 mm here)
        self.assertLess(-2.0 * g.FINGER_GRIP_TAB, g.LID_TAB_X)
        self.assertGreaterEqual((-2.0 * g.FINGER_RELEASE - g.LID_TAB_X) / 2.0, 0.007)
        # pad tips ride 5 mm over the slab's top face (>= 4 mm clear) and engage 17 mm of the 22 mm tab
        self.assertAlmostEqual(g.LID_PICK_TCP_Z - yc.PAD_TIP_PAST_TCP - g.LID_TOP_STAGED_Z, 0.005, places=6)
        self.assertAlmostEqual(g.LID_TOP_STAGED_Z + g.LID_TAB_Z - (g.LID_PICK_TCP_Z - yc.PAD_TIP_PAST_TCP), 0.017, places=6)
        # finger bodies (12 mm thick, centred half the pad gap + 6 mm out) stay 15 mm or more clear of the cavity-free
        # lid slab edge along X: the tab sits mid-lid, so the outer finger face is far inside the slab's 85 mm half width
        outer = g.LID_TAB_X / 2.0 + g.FINGER_THICK
        self.assertLess(outer, g.LID_WIDTH_X / 2.0)
        # staged lid and its plate clear the lateral pads by >= 4 mm and the table's far edge
        for y_lid in (g.LID_PAD_Y_A, g.LID_PAD_Y_B):
            lateral_edge = abs(g.PAD_LEFT_Y) + g.PAD_SIZE_Y / 2.0
            self.assertGreaterEqual(abs(y_lid) - g.LID_PAD_SIZE_Y / 2.0 - lateral_edge, 0.004)
            self.assertLess(abs(y_lid) + g.LID_PAD_SIZE_Y / 2.0, 0.65)
        # the lid is carried 15 mm or more above the latch heads: lid underside = TCP - 18 mm - 8 mm
        carried_bottom = g.LID_TRANSIT_Z - g.LID_GRIP_DZ - g.LID_THICK_Z
        self.assertGreaterEqual(carried_bottom - g.LATCH_HEAD_TOP_Z, 0.015)

    def test_lid_waypoints_follow_the_pattern(self):
        for variant in ("A", "B"):
            lay = demo_plan.layout(variant)
            arm = lay["arms"]["lid"]
            for mode in ("demo", "lid_unlatched"):
                wps = demo_plan.plans(mode, variant=variant)[arm]
                labels = [w.label for w in wps]
                i = labels.index("descend_lid")
                self.assertEqual(labels[i - 1], "pause_hover_pick_lid")
                self.assertGreaterEqual(wps[i - 1].duration, 0.6)
                j = labels.index("lower_lid")
                self.assertEqual(labels[j - 1], "pause_above_shipper_lid")
                self.assertGreaterEqual(wps[j - 1].duration, 0.6)
                # the arm never swaps yaw: left -90 deg, right +90 deg
                for w in wps:
                    self.assertAlmostEqual(w.yaw, g.GRIP_YAW_LEFT if arm == "left" else g.GRIP_YAW_RIGHT)


if __name__ == "__main__":
    unittest.main()
