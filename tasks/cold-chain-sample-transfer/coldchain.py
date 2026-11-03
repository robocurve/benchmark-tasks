"""On the station table, a frozen sample moves between two cold boxes and must spend as little time out as possible.

In front of the left arm: a blue cold box (open-top, 14 x 14 cm inside, 5 cm tall walls) holding a magenta sample
vial (a cylinder 4 cm wide and 6 cm tall, 30 g) standing in its middle. In front of the right arm: an identical, empty
cold box. Between the arms: a grey transfer pad (10 x 10 cm). Neither arm reaches the other arm's box, so the vial is
set on the transfer pad by the left arm and taken from there by the right arm.

The rule: the vial may spend at most 20 seconds in total outside the cold boxes. It is outside whenever it is not
down inside a box (centre over the box's inside and its bottom below the rim), including while lifted above a box.
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
CONTROL_HZ = 20  # the run's -E control_hz: the scorer measures time by counting control steps

# Plain primitives. Library alternatives to check in the catalog render: storage_box_hot3d_robolab or
# bin_b03_vomp_robolab for the boxes (update COOLER_IN and WALL_H to the model), frozen_vegetable_block for the sample.
COOLER_IN, WALL_T, WALL_H, FLOOR_T = 0.14, 0.01, 0.05, 0.004
LEFT_BOX_XY, RIGHT_BOX_XY = (0.27, 0.30), (0.27, -0.30)
VIAL_R, VIAL_H, VIAL_KG = 0.02, 0.06, 0.03
BOX_BLUE, FLOOR_BLUE = (0.25, 0.50, 0.85), (0.70, 0.85, 0.95)
FLOOR_TOP = TOP + FLOOR_T
RIM = TOP + WALL_H


def _cold_box(prefix, xy):
    """Four walls and a pale floor plate, all fixed."""
    cx, cy = xy
    z, a, outer = TOP + WALL_H / 2, COOLER_IN / 2 + WALL_T / 2, COOLER_IN + 2 * WALL_T
    return [
        Box(f"{prefix}_floor", pos=(cx, cy, TOP + FLOOR_T / 2), size=(COOLER_IN, COOLER_IN, FLOOR_T), color=FLOOR_BLUE),
        Box(f"{prefix}_wall_far", pos=(cx + a, cy, z), size=(WALL_T, outer, WALL_H), color=BOX_BLUE),
        Box(f"{prefix}_wall_near", pos=(cx - a, cy, z), size=(WALL_T, outer, WALL_H), color=BOX_BLUE),
        Box(f"{prefix}_wall_left", pos=(cx, cy + a, z), size=(COOLER_IN, WALL_T, WALL_H), color=BOX_BLUE),
        Box(f"{prefix}_wall_right", pos=(cx, cy - a, z), size=(COOLER_IN, WALL_T, WALL_H), color=BOX_BLUE),
    ]


VIAL = Cylinder("sample_vial", pos=(*LEFT_BOX_XY, FLOOR_TOP + VIAL_H / 2), radius=VIAL_R, height=VIAL_H, mass=VIAL_KG,
                color=(0.90, 0.20, 0.55))
PAD = Box("transfer_pad", pos=(0.25, 0.0, TOP + 0.002), size=(0.10, 0.10, 0.004), color=(0.60, 0.60, 0.62))
PAD_TOP = TOP + 0.004

OBJECTS = [VIAL, PAD] + _cold_box("left_box", LEFT_BOX_XY) + _cold_box("right_box", RIGHT_BOX_XY)

# Seconds outside -> score, for a vial delivered into the right box: the limit, then two looser bands.
TIME_BANDS = ((20.0, 1.0), (30.0, 0.5), (45.0, 0.25))
OFF_TABLE_M = 0.05
RESET_NEAR_M = 0.035


def _fresh_state():
    return {"prev": None, "out_steps": 0}


# Time outside is a history, so the scorer keeps it between control steps. score() is called once per step by the
# embodiment; a new episode is recognised when the vial jumps back to its start.
_state = _fresh_state()


def inside(vial, box_xy) -> bool:
    """Down in the box: centre over its inside and bottom below the rim."""
    return (
        abs(vial.pos[0] - box_xy[0]) <= COOLER_IN / 2
        and abs(vial.pos[1] - box_xy[1]) <= COOLER_IN / 2
        and vial.low[2] < RIM
    )


def _off_table(obj) -> bool:
    return obj.pos[2] < TOP - OFF_TABLE_M


def _update(objects) -> None:
    v = objects["sample_vial"]
    now, prev = np.array(v.pos, dtype=float), _state["prev"]
    jumped = prev is not None and np.linalg.norm(now - prev) > 0.01 + 0.2 * np.linalg.norm(v.lin_vel)
    if prev is None or (jumped and np.linalg.norm(now[:2] - np.array(LEFT_BOX_XY)) < RESET_NEAR_M):
        _state.update(_fresh_state())
    _state["prev"] = now
    if not (inside(v, LEFT_BOX_XY) or inside(v, RIGHT_BOX_XY)):
        _state["out_steps"] += 1


def seconds_outside() -> float:
    return _state["out_steps"] / CONTROL_HZ


def delivered(objects) -> bool:
    v = objects["sample_vial"]
    return inside(v, RIGHT_BOX_XY) and v.resting()


def score(objects) -> float:
    """For a vial resting in the right box: 1.0 if it spent at most 20 s outside the boxes, 0.5 at most 30 s,
    0.25 at most 45 s. Otherwise 0, including a vial not delivered or off the table."""
    _update(objects)
    v = objects["sample_vial"]
    if _off_table(v) or not delivered(objects):
        return 0.0
    for limit, s in TIME_BANDS:
        if seconds_outside() <= limit:
            return s
    return 0.0


# -- scripted demo --------------------------------------------------------------------------------------------------

SPEED = 1.25
TRAVEL = TOP + 0.14
LEFT_PARK = (0.18, 0.20, TRAVEL)
RIGHT_WAIT = (0.25, -0.16, TRAVEL)  # beside the pad, clear of the left gripper while it lets go


def _dur(waypoints) -> float:
    return sum(w.duration for w in waypoints)


def demo():
    """Left arm: vial from its box to the pad, then park. Right arm: wait beside the pad, then take the vial straight
    into its box. About 12 s outside."""
    vial_in_left = (*LEFT_BOX_XY, FLOOR_TOP + VIAL_H / 2)
    on_pad = (PAD.pos[0], PAD.pos[1], PAD_TOP + VIAL_H / 2)
    in_right = (*RIGHT_BOX_XY, FLOOR_TOP + VIAL_H / 2 + 0.005)
    left = pick_place_waypoints(vial_in_left, (on_pad[0], on_pad[1], on_pad[2] + 0.005), travel_z=TRAVEL, speed=SPEED)
    left += [Waypoint(1.0 / SPEED, LEFT_PARK, FINGER_OPEN, "park")]
    right = [Waypoint(_dur(left) + 0.2, RIGHT_WAIT, FINGER_OPEN, "wait beside the pad")]
    right += pick_place_waypoints(on_pad, in_right, travel_z=TRAVEL, speed=SPEED)
    return {"left": left, "right": right}
