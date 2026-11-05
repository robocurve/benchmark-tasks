"""On the station table, in front of the two arms: a flat, closed, handle-less cardboard box lies on the centre line
near the robot. Beyond it stands a fixed wooden riser with a narrow storage slot along its left side, between the
riser and a low wall. The box has to end up standing on edge in the slot, like a file in a rack. Lying flat, the box
is wider and deeper than a gripper can open, so no single gripper can pick it up: it has to be squeezed between both
grippers and lifted onto the riser. From there it can be tipped over the riser's left edge: the riser is exactly as
tall as half the box's width, so the box swings down about that edge and lands standing in the slot.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- box: closed white corrugated cardboard box, 12 cm deep (x) x 20 cm wide (y) x 9 cm thick (z), 350 g, lying flat,
  centred at (0.15, 0).
- riser: fixed dark wooden block, 12 cm deep x 12 cm wide x 10 cm tall, centred at (0.30, -0.04): its left face is at
  y 0.02.
- slot: the 14.5 cm gap between the riser's left face and a fixed wall (dark wood, 14 cm long x 1.5 cm thick x 9 cm
  tall) whose inner face is at y 0.165, from x 0.23 to x 0.37. A box standing on edge in it is 20 cm tall.

Full score: the box stands on edge in the slot (on one of its 12 x 9 cm faces: its 20 cm side within 20 degrees of
vertical, its underside within 1.5 cm of the table, its centre between the riser and the wall and within the slot's
length), still for 2 s.
"""

import os

import numpy as np

from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 2.0

CARDBOARD = (0.92, 0.91, 0.88)
WOOD = (0.36, 0.20, 0.10)

BOX_SIZE = (0.12, 0.20, 0.09)  # x (front to back), y (side to side), z (thickness), lying flat
BOX_KG = 0.35
BOX_X = 0.15
RISER_SIZE = (0.12, 0.12, 0.10)  # as tall as half the box's width
RISER_XY = (0.30, -0.04)
RISER_EDGE_Y = RISER_XY[1] + RISER_SIZE[1] / 2  # 0.02: the edge the box is tipped over
WALL_IN_Y, WALL_T, WALL_H, WALL_LEN = 0.165, 0.015, 0.09, 0.14

OBJECTS = [
    Box("riser", pos=(*RISER_XY, TOP + RISER_SIZE[2] / 2), size=RISER_SIZE, color=WOOD),
    Box("slot_wall", pos=(RISER_XY[0], WALL_IN_Y + WALL_T / 2, TOP + WALL_H / 2), size=(WALL_LEN, WALL_T, WALL_H),
        color=WOOD),
    Box("box", pos=(BOX_X, 0.0, TOP + 0.001 + BOX_SIZE[2] / 2), size=BOX_SIZE, mass=BOX_KG, color=CARDBOARD),
]

# Thresholds, as a reviewer would check them with a tape measure and a protractor.
EDGE_TILT_MAX = np.radians(20)  # on edge: the box's 20 cm side within this of vertical
FLAT_TILT_MAX = np.radians(10)  # half score: lying flat on the riser
ON_TABLE = 0.015
RISER_TOP = TOP + RISER_SIZE[2]

_DEBUG = os.environ.get("BE_DEBUG") == "1"
_state = {"step": 0}


def _axes(quat_xyzw):
    x, y, z, w = quat_xyzw
    # columns of the rotation matrix: the box's own x, y, z axes in the world
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _on_edge_in_slot(box) -> bool:
    r = _axes(box.quat_xyzw)
    width_vertical = float(np.arccos(np.clip(abs(r[2, 1]), 0.0, 1.0)))  # the box's own y (20 cm) side vs vertical
    in_slot = (RISER_EDGE_Y < box.pos[1] < WALL_IN_Y
               and abs(box.pos[0] - RISER_XY[0]) <= WALL_LEN / 2)
    return bool(width_vertical <= EDGE_TILT_MAX and box.low[2] - TOP <= ON_TABLE and in_slot)


def _flat_on_riser(box) -> bool:
    r = _axes(box.quat_xyzw)
    flat = float(np.arccos(np.clip(abs(r[2, 2]), 0.0, 1.0))) <= FLAT_TILT_MAX
    over = (abs(box.pos[0] - RISER_XY[0]) < RISER_SIZE[0] / 2 + 0.02
            and RISER_XY[1] - RISER_SIZE[1] / 2 - 0.02 < box.pos[1] < RISER_EDGE_Y)
    return bool(flat and over and abs(box.low[2] - RISER_TOP) < 0.01)


def success(objects):
    box = objects["box"]
    ok = _on_edge_in_slot(box)
    if _DEBUG:
        _state["step"] += 1
        if _state["step"] % 10 == 1:
            r = _axes(box.quat_xyzw)
            print(f"DBG step={_state['step']} pos=({box.pos[0]:.3f},{box.pos[1]:.3f}) low={box.low[2] - TOP:.3f} "
                  f"y_axis_up={abs(r[2, 1]):.2f} z_axis_up={abs(r[2, 2]):.2f} on_edge_in_slot={ok} "
                  f"flat_on_riser={_flat_on_riser(box)}", flush=True)
    return ok and box.resting()


def distance(objects):
    box = objects["box"]
    goal = np.array([RISER_XY[0], (RISER_EDGE_Y + WALL_IN_Y) / 2, TOP + BOX_SIZE[1] / 2])
    return float(np.linalg.norm(np.asarray(box.pos) - goal))


def score(objects):
    """Partial credit (success scores 1.0): 0.5 when the box lies flat on top of the riser (lifted there, but not
    tipped into the slot); 0 for anything else."""
    return 0.5 if _flat_on_riser(objects["box"]) else 0.0


# -- scripted demo -------------------------------------------------------------------------------------------------
# The lift: both grippers stay closed and press the box's left and right faces with the outer faces of their fingers
# (turned 90 degrees, in opposite directions for the two arms); each grip point is commanded inside the box so the
# arms' joint stiffness turns the shortfall into squeeze. Both lift the box, carry it over the riser and set it down
# flat on the riser's top, centred on it, then let go sideways and rise. The tip: the right gripper, closed, comes
# down beside the box's right face and pushes it left across the riser until it tips over the left edge into the slot.
FINGER_REACH = 0.034
SQUEEZE = float(os.environ.get("BE_SQUEEZE", "0.05"))
SQUEEZE_FAR = float(os.environ.get("BE_SQUEEZE_FAR", "0.06"))
CLEAR = 0.04
GRIP_Z = 0.03  # grip point above the box's underside: fingertips 1.7 cm up, the gripper body below the box's top
LIFT = float(os.environ.get("BE_LIFT", "0.13"))  # the box's underside while carrying (the riser's top is at 10 cm)
LET_GO = 0.003
PUSH_Z = 0.03  # push point above the box's underside, on the riser
PUSH_TO = float(os.environ.get("BE_PUSH_TO", "0.07"))  # box centre y the push aims for (past the tipping point)
PUSH_S = float(os.environ.get("BE_PUSH_S", "3.0"))


def demo():
    half = BOX_SIZE[1] / 2
    rx, ry = RISER_XY
    plans = {}
    for arm, side, yaw in (("left", 1.0, -np.pi / 2), ("right", -1.0, np.pi / 2)):
        y_touch = side * (half + FINGER_REACH)
        y_out = y_touch + side * CLEAR
        y_in, y_in_far = y_touch - side * SQUEEZE, y_touch - side * SQUEEZE_FAR
        z_grip, z_lift = TOP + GRIP_Z, TOP + LIFT + GRIP_Z
        z_place = RISER_TOP + LET_GO + GRIP_Z
        plans[arm] = [
            Waypoint(2.0, (BOX_X, y_out, z_lift), FINGER_CLOSED, "beside the box", yaw),
            Waypoint(1.5, (BOX_X, y_out, z_grip), FINGER_CLOSED, "down", yaw),
            Waypoint(1.5, (BOX_X, y_in, z_grip), FINGER_CLOSED, "squeeze", yaw),
            Waypoint(0.5, (BOX_X, y_in, z_grip), FINGER_CLOSED, "settle", yaw),
            Waypoint(2.0, (BOX_X, y_in, z_lift), FINGER_CLOSED, "lift", yaw),
            Waypoint(3.0, (rx, ry + y_in_far, z_lift), FINGER_CLOSED, "carry over the riser", yaw),
            Waypoint(1.5, (rx, ry + y_in_far, z_place), FINGER_CLOSED, "lower onto the riser", yaw),
            Waypoint(1.0, (rx, ry + y_out, z_place), FINGER_CLOSED, "let go", yaw),
            Waypoint(1.2, (rx, ry + y_out, z_lift + 0.03), FINGER_CLOSED, "rise", yaw),
        ]
    # left arm: out of the way of the slot
    plans["left"].append(Waypoint(1.5, (0.18, 0.28, TOP + LIFT + 0.05), FINGER_CLOSED, "out of the way", -np.pi / 2))
    # right arm: push the box left across the riser until it tips into the slot
    y_start = ry - (half + FINGER_REACH) - 0.01  # just right of the box's right face (box centred on the riser)
    z_push = RISER_TOP + PUSH_Z
    plans["right"] += [
        Waypoint(1.5, (rx, y_start, z_push + 0.06), FINGER_CLOSED, "above, right of the box", np.pi / 2),
        Waypoint(1.0, (rx, y_start, z_push), FINGER_CLOSED, "down beside it", np.pi / 2),
        Waypoint(PUSH_S, (rx, PUSH_TO - (half + FINGER_REACH), z_push), FINGER_CLOSED,
                 "push it left until it tips", np.pi / 2),
        Waypoint(1.0, (rx, PUSH_TO - (half + FINGER_REACH) - 0.04, z_push), FINGER_CLOSED, "back off", np.pi / 2),
        Waypoint(1.2, (rx, PUSH_TO - (half + FINGER_REACH) - 0.04, z_push + 0.05), FINGER_CLOSED, "rise",
                 np.pi / 2),
    ]
    end = max(sum(w.duration for w in p) for p in plans.values())
    for p in plans.values():
        if sum(w.duration for w in p) < end:
            p.append(Waypoint(end - sum(w.duration for w in p), p[-1].pos, p[-1].finger, "wait", p[-1].yaw))
    return plans
