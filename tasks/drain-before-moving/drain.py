"""On the station table, a part soaking in a parts-washer bath must drain over the bath before it is carried away.

In front of the left arm: a teal parts-washer bath (open-top, 14 x 14 cm inside, 5 cm tall walls) with a grey metal
part (a 4 cm cube, 150 g) soaking in its middle. In front of the right arm: a green drying rack (a 14 x 14 cm mat).
Between the arms: a white transfer pad (10 x 10 cm). Neither arm reaches both the bath and the rack, so the left arm
sets the part on the pad and the right arm takes it from there to the rack.

The rule: after lifting the part out of the bath, hold it still above the bath for at least 3 seconds, so it stops
dripping, before moving it away from over the bath.
"""

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
CONTROL_HZ = 20  # the run's -E control_hz: the scorer measures time by counting control steps

# Plain primitives. Library alternatives to check in the catalog render: plasticpail_a02_vomp_robolab for the bath,
# rack_l04_vomp_robolab for the rack, medium_gear or clamp_ycb_robolab for the part (update the sizes to the model).
BATH_IN, WALL_T, WALL_H, FLOOR_T = 0.14, 0.01, 0.05, 0.004
BATH_XY = (0.26, 0.30)
PART_EDGE, PART_KG = 0.04, 0.15
TEAL, LIQUID = (0.20, 0.55, 0.60), (0.30, 0.45, 0.50)
FLOOR_TOP = TOP + FLOOR_T
RIM = TOP + WALL_H


def _bath(xy):
    cx, cy = xy
    z, a, outer = TOP + WALL_H / 2, BATH_IN / 2 + WALL_T / 2, BATH_IN + 2 * WALL_T
    return [
        Box("bath_floor", pos=(cx, cy, TOP + FLOOR_T / 2), size=(BATH_IN, BATH_IN, FLOOR_T), color=LIQUID),
        Box("bath_wall_far", pos=(cx + a, cy, z), size=(WALL_T, outer, WALL_H), color=TEAL),
        Box("bath_wall_near", pos=(cx - a, cy, z), size=(WALL_T, outer, WALL_H), color=TEAL),
        Box("bath_wall_left", pos=(cx, cy + a, z), size=(BATH_IN, WALL_T, WALL_H), color=TEAL),
        Box("bath_wall_right", pos=(cx, cy - a, z), size=(BATH_IN, WALL_T, WALL_H), color=TEAL),
    ]


PART = Box("metal_part", pos=(*BATH_XY, FLOOR_TOP + PART_EDGE / 2), size=(PART_EDGE,) * 3, mass=PART_KG,
           color=(0.55, 0.57, 0.62))
PAD = Box("transfer_pad", pos=(0.25, 0.0, TOP + 0.002), size=(0.10, 0.10, 0.004), color=(0.88, 0.88, 0.88))
RACK = Box("drying_rack", pos=(0.30, -0.30, TOP + 0.002), size=(0.14, 0.14, 0.004), color=(0.25, 0.60, 0.35))
MAT_TOP = TOP + 0.004

OBJECTS = [PART, PAD, RACK] + _bath(BATH_XY)

DRAIN_S = 3.0  # seconds held still above the bath
STILL_MPS = 0.03  # "still": under 3 cm/s
ON_SURFACE_M = 0.01
OFF_TABLE_M = 0.05
RESET_NEAR_M = 0.035


def _fresh_state():
    # wet: has been in the bath since it last drained; drain_steps: consecutive still steps above the bath.
    return {"prev": None, "wet": True, "drain_steps": 0, "drained_once": False, "violated": False}


# The rule is about the order and timing of events, so the scorer keeps history between control steps. score() is
# called once per step by the embodiment; a new episode is recognised when the part jumps back into the bath.
_state = _fresh_state()


def over_bath(part) -> bool:
    return abs(part.pos[0] - BATH_XY[0]) <= BATH_IN / 2 and abs(part.pos[1] - BATH_XY[1]) <= BATH_IN / 2


def _over(obj, box) -> bool:
    return abs(obj.pos[0] - box.pos[0]) <= box.size[0] / 2 and abs(obj.pos[1] - box.pos[1]) <= box.size[1] / 2


def _off_table(obj) -> bool:
    return obj.pos[2] < TOP - OFF_TABLE_M


def _update(objects) -> None:
    p = objects["metal_part"]
    now, prev = np.array(p.pos, dtype=float), _state["prev"]
    jumped = prev is not None and np.linalg.norm(now - prev) > 0.01 + 0.2 * np.linalg.norm(p.lin_vel)
    if prev is None or (jumped and np.linalg.norm(now[:2] - np.array(BATH_XY)) < RESET_NEAR_M):
        _state.update(_fresh_state())
    # Speed from how far the part moved since the last step, not the simulator's velocity: a part squeezed in the
    # gripper reports 4-5 cm/s of contact jitter while its position does not change.
    speed = 0.0 if prev is None else float(np.linalg.norm(now - prev)) * CONTROL_HZ
    _state["prev"] = now
    if over_bath(p) and p.low[2] < RIM:  # in the bath: wet again
        _state.update(wet=True, drain_steps=0)
    elif over_bath(p):  # above the bath: draining while still
        still = speed < STILL_MPS
        _state["drain_steps"] = _state["drain_steps"] + 1 if still else 0
        if _state["drain_steps"] >= round(DRAIN_S * CONTROL_HZ):
            _state.update(wet=False, drained_once=True)
    elif _state["wet"]:  # moved away from over the bath while still wet
        _state["violated"] = True


def violated() -> bool:
    return bool(_state["violated"])


def score(objects) -> float:
    """1.0: part resting on the drying rack, never moved away from the bath before draining. 0.5: no violation, part
    resting on the table or pad, away from the bath. 0.25: no violation, part drained over the bath. Otherwise 0."""
    _update(objects)
    p = objects["metal_part"]
    if violated() or _off_table(p):
        return 0.0
    on_mat = abs(p.low[2] - MAT_TOP) < ON_SURFACE_M or abs(p.low[2] - TOP) < ON_SURFACE_M
    if _over(p, RACK) and on_mat and p.resting():
        return 1.0
    if not over_bath(p) and on_mat and p.resting():
        return 0.5
    if _state["drained_once"]:
        return 0.25
    return 0.0


# -- scripted demo --------------------------------------------------------------------------------------------------

TRAVEL = TOP + 0.14
LEFT_PARK = (0.18, 0.20, TRAVEL)
RIGHT_WAIT = (0.25, -0.16, TRAVEL)  # beside the pad, clear of the left gripper while it lets go


def _dur(waypoints) -> float:
    return sum(w.duration for w in waypoints)


def demo():
    """Left arm: lift the part straight up out of the bath, hold it still above the bath for 3.5 s, set it on the pad,
    park. Right arm: wait beside the pad, then take the part to the rack."""
    bx, by = BATH_XY
    grip = FLOOR_TOP + PART_EDGE / 2
    px, py = PAD.pos[0], PAD.pos[1]
    on_pad = MAT_TOP + PART_EDGE / 2
    left = [
        Waypoint(1.5, (bx, by, TRAVEL), FINGER_OPEN, "to the part"),
        Waypoint(1.5, (bx, by, grip), FINGER_OPEN, "descend"),
        Waypoint(0.8, (bx, by, grip), FINGER_CLOSED, "close"),
        Waypoint(1.5, (bx, by, TRAVEL), FINGER_CLOSED, "lift out"),
        Waypoint(DRAIN_S + 0.5, (bx, by, TRAVEL), FINGER_CLOSED, "drain over the bath"),
        Waypoint(1.8, (px, py, TRAVEL), FINGER_CLOSED, "carry"),
        Waypoint(1.2, (px, py, on_pad + 0.005), FINGER_CLOSED, "lower"),
        Waypoint(0.6, (px, py, on_pad + 0.005), FINGER_OPEN, "release"),
        Waypoint(1.0, (px, py, TRAVEL), FINGER_OPEN, "retreat"),
        Waypoint(1.0, LEFT_PARK, FINGER_OPEN, "park"),
    ]
    right = [Waypoint(_dur(left) + 0.2, RIGHT_WAIT, FINGER_OPEN, "wait beside the pad")]
    right += pick_place_waypoints((px, py, on_pad), (RACK.pos[0], RACK.pos[1], on_pad + 0.005), travel_z=TRAVEL)
    return {"left": left, "right": right}
