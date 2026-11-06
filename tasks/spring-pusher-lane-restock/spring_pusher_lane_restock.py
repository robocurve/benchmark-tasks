"""On the station table: a shelf lane (a white floor 8 cm wide between two low dark rails, a dark front wall at the
near end, open at the far end) lies along x in the middle of the table. Two red cartons (5 x 5 x 8 cm) stand in it
against the front wall. Behind them a grey spring pusher rests on its stop about 1 mm from the second one; its plate and side-arm handle (at y -15 cm) are lower than
the cartons. Two blue cartons stand on the table to the left of the lane.

Requires -E object_jitter=0: the cartons and the pusher's rest mark are measured from their set positions.

Run it:  inspect-robots run --embodiment yam_arena -E task=spring_pusher_lane_restock.py -E object_jitter=0 \\
           --instruction "Pull the grey pusher back and hold it. Set both blue cartons in the lane behind the red
           ones, without passing either over the red ones, then let the pusher forward so it closes up the row." --scorer task_score ...
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from belimits import ResetOnJumpTracker, Rect, bottom, footprint, inside, is_airborne, overlaps, tilt_deg
from mechkit import Joint, Mech, box, on_reset
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
SUCCESS_HOLD_S = 4.0  # full score must hold, with every carton still, this long
TOP = TABLE_TOP[SCENE]

RED, BLUE_C = (0.85, 0.12, 0.12), (0.15, 0.30, 0.85)
WHITE, DARK, GREY, LIGHT = (0.92, 0.92, 0.88), (0.18, 0.18, 0.22), (0.50, 0.50, 0.52), (0.68, 0.68, 0.70)

# -- the lane: fixed, along x at y 0 -------------------------------------------------------------------------------
LANE_X0, LANE_X1 = 0.065, 0.41  # floor, from the front wall to the open far end
INNER = 0.04  # half the inner width: rails are 8 cm apart (1.5 cm to spare each side of a 5 cm carton)
FLOOR_H, RAIL_TOP, WALL_H = 0.008, 0.028, 0.07
LANE_CX = (LANE_X0 + LANE_X1) / 2
FLOOR_Z = TOP + FLOOR_H

# -- the pusher: a slide along x, sprung toward the front wall ------------------------------------------------------
PLATE_X0, PLATE_T = 0.167, 0.01  # the plate's front face at rest, and its thickness
J_LOW, J_HIGH, SPRING, DAMPING = 0.0, 0.20, 15.0, 10.0


def _lx(x):
    return x - LANE_CX


LANE = Mech(
    "lane",
    pos=(LANE_CX, 0.0, TOP),
    mass=None,
    friction=0.4,
    shapes=[
        box((0, 0, FLOOR_H / 2), (LANE_X1 - LANE_X0, 2 * (INNER + 0.006), FLOOR_H), WHITE),
        box((0, INNER + 0.003, (FLOOR_H + RAIL_TOP) / 2), (LANE_X1 - LANE_X0, 0.006, RAIL_TOP - FLOOR_H), DARK),
        box((0, -INNER - 0.003, (FLOOR_H + RAIL_TOP) / 2), (LANE_X1 - LANE_X0, 0.006, RAIL_TOP - FLOOR_H), DARK),
        box((_lx(0.06), 0, WALL_H / 2), (0.01, 2 * (INNER + 0.006), WALL_H), DARK),
    ],
    joints=[
        Joint(
            "pusher",
            "slide",
            axis="x",
            at=(_lx(PLATE_X0 + PLATE_T / 2), 0.0, 0.0),
            low=J_LOW,
            high=J_HIGH,
            spring=SPRING,
            damping=DAMPING,
            mass=0.05,
            shapes=[
                box((0, 0, 0.0325), (PLATE_T, 0.078, 0.045), GREY),  # plate x 0.167..0.177, y +-0.039, z 0.01..0.055
                # side arm: x 0.177..0.189, y -0.039..-0.17, z 0.03..0.045; it crosses above the right rail (top 0.028)
                # and stays below the 0.088 carton tops
                box((0.011, -0.1045, 0.0375), (0.012, 0.131, 0.015), GREY),
                # handle, offset to y -0.15 (x 0.182..0.212, y -0.17..-0.13, z 0.03..0.075), clear of the left arm's work
                box((0.025, -0.15, 0.0525), (0.03, 0.04, 0.045), LIGHT),
            ],
        )
    ],
)

# -- the cartons: 5 x 5 x 8 cm, 40 g ----------------------------------------------------------------------------------
CARTON = (0.05, 0.05, 0.08)
RED_1 = Box("red_carton_1", pos=(0.09, 0.0, FLOOR_Z + CARTON[2] / 2), size=CARTON, mass=0.04, color=RED)
RED_2 = Box("red_carton_2", pos=(0.141, 0.0, FLOOR_Z + CARTON[2] / 2), size=CARTON, mass=0.04, color=RED)
BLUE_1 = Box("blue_carton_1", pos=(0.15, 0.20, TOP + CARTON[2] / 2), size=CARTON, mass=0.04, color=BLUE_C)
BLUE_2 = Box("blue_carton_2", pos=(0.25, 0.20, TOP + CARTON[2] / 2), size=CARTON, mass=0.04, color=BLUE_C)
REDS, BLUES = (RED_1, RED_2), (BLUE_1, BLUE_2)

OBJECTS = [LANE, RED_1, RED_2, BLUE_1, BLUE_2]

# -- scoring numbers ----------------------------------------------------------------------------------------------------
# "Inside the rails and the front wall" allows 2 mm: a carton touching the wall or a rail, or sunk up to 2 mm past its face
# by contact (the closed row is pressed into the wall by about 3 N), is inside.
LANE_AREA = Rect((LANE_X0, LANE_X1), (-INNER, INNER)).grown(0.002)
# The red stock's place: from the front wall to the red cartons' back face, rails included. A blue carton over it at any
# height breaks the order rule.
RED_STOCK = Rect((0.055, PLATE_X0 - 0.001), (-INNER - 0.006, INNER + 0.006))
STOCK_SLACK = 0.003  # overlap under this is contact, not "over"
LANE_BOTTOM_TOL = 0.01  # a carton whose bottom is more than 1 cm above the floor is not placed
MAX_TILT = 10.0
BAND = (0.092, 0.110)  # pusher joint (distance from its rest mark) for a closed-up row of four
PULLED = 0.06
PULLED_HOLD = 20  # steps (1 s at 20 Hz) the pusher must stay at PULLED or more for the 0.25 tier
PULLED_RED_TOL = 0.01  # "pulled" counts only while both reds are within 1 cm of their start positions
PLATE_SLACK = 0.004  # a carton counts as in front of the plate if its back face is at most 4 mm past the plate face

_HOME = {s.name: s.pos for s in (*REDS, *BLUES)}
# Latched over the whole run; cleared by the embodiment's reset() (the on_reset callback below) and, as a fallback, by a
# real trial reset seen from inside score() (a carton teleported from more than 3 cm away back to its mark).
_TRACK = ResetOnJumpTracker(_HOME)
_LAST = [None]  # the states dict last counted into _RUN (success() and score() share one per step)
_RUN = [0]  # consecutive steps with the pusher pulled, both reds in the lane and within 1 cm of start


@on_reset
def _clear_latch(env):
    _TRACK.clear()
    _RUN[0] = 0
    _LAST[0] = None


_SPEC = {s.name: s for s in (*REDS, *BLUES)}


def _joint(objects) -> float:
    return float(objects["lane"].joints["pusher"])


def _in_lane(s, spec, j) -> bool:
    """Inside the rails and the front wall, bottom within 1 cm of the floor, tilted at most 10 degrees, and in front
    of the pusher plate. A carton held up (bottom more than 1 cm above the floor) is not placed."""
    if is_airborne(s, spec, [FLOOR_Z], LANE_BOTTOM_TOL) or bottom(s, spec) < FLOOR_Z - 0.005:
        return False
    if tilt_deg(s) > MAX_TILT or not inside(s, spec, LANE_AREA):
        return False
    return float(footprint(s, spec)[:, 0].max()) <= PLATE_X0 + j + PLATE_SLACK


def _over_stock(s, spec) -> bool:
    """Any part of the carton's outline over the red stock's place, at any height."""
    return overlaps(s, spec, RED_STOCK.grown(-STOCK_SLACK))


def _state(objects):
    """Update the run's flags (kept over the whole run) and return (flags, joint, reds in, blues in)."""
    j = _joint(objects)
    reds_in = [_in_lane(objects[r.name], r, j) for r in REDS]
    blues_in = [_in_lane(objects[b.name], b, j) for b in BLUES]
    over = any(_over_stock(objects[b.name], b) for b in BLUES)
    reds_home = all(
        float(np.linalg.norm(np.asarray(objects[r.name].pos)[:2] - np.asarray(r.pos)[:2])) <= PULLED_RED_TOL
        for r in REDS
    )
    if objects is not _LAST[0]:  # advance the run at most once per control step
        _RUN[0] = _RUN[0] + 1 if (j >= PULLED and all(reds_in) and reds_home) else 0
        _LAST[0] = objects
    flags = _TRACK.update(objects, {"over_stock": float(over), "pulled_run": float(_RUN[0])})
    return flags, j, reds_in, blues_in


def score(objects):
    """0 if a blue carton was ever over the red stock's place (any height) or a blue carton in the lane ended nearer the front wall than a red one, or a red
    carton is out of the lane. Otherwise 1.0 for all four in the lane with the pusher 9.2 to 11.0 cm from its rest
    mark and every carton resting (under 1 cm/s), 0.75 for all four in the lane otherwise, 0.5 for one blue in the lane, 0.25 if the pusher was pulled
    6 cm or more for 1 s while both reds were in the lane and within 1 cm of their start positions. The order rule applies to
    blue cartons in the lane only (a blue on the table is not in the lane)."""
    flags, j, reds_in, blues_in = _state(objects)
    if flags.get("over_stock", 0.0) > 0 or not all(reds_in):
        return 0.0
    red_x = [objects[r.name].pos[0] for r in REDS]
    blue_x = [objects[b.name].pos[0] for b, ok in zip(BLUES, blues_in) if ok]
    if any(bx < rx for bx in blue_x for rx in red_x):
        return 0.0
    n = len(blue_x)
    if n == 2:
        still = all(objects[nm].resting() for nm in _HOME)
        return 1.0 if BAND[0] <= j <= BAND[1] and still else 0.75
    if n == 1:
        return 0.5
    return 0.25 if flags.get("pulled_run", 0.0) >= PULLED_HOLD else 0.0


def success(objects):
    """Full score with every carton still."""
    return score(objects) == 1.0 and all(objects[n].resting() for n in _HOME)


# -- the robot itself keeps clear of the red stock too (checked offline with the URDF) ------------------------------------
# The sweep checks link centrelines, so the region is the red stock and wall footprint (x 0.015..0.166, y +-0.066, rails
# included) grown by 2.5 cm on the +x and +-y sides, a little over the arm links' half-width (about 2 cm by the URDF collision
# meshes, whose own sweep of this demo leaves 1.9 cm at the left forearm (1.6 cm by the URDF visual mesh) and 5.3 cm at the right arm).
ARM_KEEP_OUT = [
    {"name": "red stock and front wall", "x": (0.015, 0.191), "y": (-0.091, 0.091), "z_top": 2.0},
]

# -- the demo ----------------------------------------------------------------------------------------------------------------
V_MAX = 0.22  # m/s: peak tool speed of every leg stays under the gate's 0.25
HANDLE_X = 0.197
HANDLE_Y = -0.15  # the handle sits on a side arm, outside the left arm's work area
PULL_X = HANDLE_X + 0.19  # handle at 0.387: pusher 19 cm back (2.85 N), gap behind the reds 19 cm for two 5 cm cartons
EASE_X = HANDLE_X + 0.114  # pusher 11.4 cm back: just outside the 9.2 to 11.0 band; the spring closes the row
GRIP_Z = TOP + 0.06  # mid-handle, above the side arm (top 4.5 cm)
CARRY_Z = TOP + 0.15
SET_Z = TOP + 0.07  # carton bottom 1.0 cm above the table, 0.2 cm above the lane floor, 1.8 cm below the rail tops
READY_L = (0.323, 0.305, 1.008)
READY_R = (0.323, -0.305, 1.008)


class _Path:
    def __init__(self, start):
        self.pos, self.legs = np.array(start, float), []

    def go(self, pos, finger, label, min_t=0.6, yaw=0.0):
        pos = np.array(pos, float)
        t = max(min_t, 1.5 * float(np.linalg.norm(pos - self.pos)) / V_MAX)
        self.legs.append(Waypoint(round(t, 2), tuple(pos), finger, label, yaw))
        self.pos = pos
        return t

    def wait(self, t, finger, label):
        self.legs.append(Waypoint(round(t, 2), tuple(self.pos), finger, label, 0.0))

    @property
    def total(self):
        return sum(w.duration for w in self.legs)


def _carry(path, pick, place_x, label):
    px, py = pick
    path.go((px, py, CARRY_Z), FINGER_OPEN, f"over {label}", 1.5)
    path.go((px, py, GRIP_Z), FINGER_OPEN, f"down to {label}", 1.5)
    path.wait(0.8, FINGER_CLOSED, f"grip {label}")
    path.go((px, py, CARRY_Z), FINGER_CLOSED, f"lift {label}", 1.2)
    path.go((place_x, 0.12, CARRY_Z), FINGER_CLOSED, f"carry {label} to beside the lane", 1.8)
    path.go((place_x, 0.0, CARRY_Z), FINGER_CLOSED, f"carry {label} over the lane", 1.2)
    path.go((place_x, 0.0, SET_Z), FINGER_CLOSED, f"lower {label} to the lane floor", 1.2)
    path.wait(0.6, FINGER_OPEN, f"release {label}")
    path.go((place_x, 0.0, CARRY_Z), FINGER_OPEN, "rise", 1.0)
    path.go((place_x, 0.12, CARRY_Z), FINGER_OPEN, "back out to the left, clear of the red stock", 1.0)


def demo():
    """RIGHT: pull the handle back to 19 cm and hold it. LEFT: carry the near blue carton (x 0.25) to the gap just
    behind the red cartons, then the other to the gap's far end, and clear. RIGHT: bring the pusher forward to 11.4 cm,
    open, rise and draw back; the spring closes the row. Then wait with the arm clear."""
    right = _Path(READY_R)
    right.go((HANDLE_X, HANDLE_Y, TOP + 0.14), FINGER_OPEN, "over the handle", 1.5)
    right.go((HANDLE_X, HANDLE_Y, GRIP_Z), FINGER_OPEN, "down on the handle", 1.5)
    right.wait(0.8, FINGER_CLOSED, "grip the handle")
    right.go((PULL_X, HANDLE_Y, GRIP_Z), FINGER_CLOSED, "pull the pusher back", 4.0)
    t_pulled = right.total

    left = _Path(READY_L)
    left.go((0.25, 0.20, CARRY_Z), FINGER_OPEN, "wait over the first blue carton", t_pulled)
    _carry(left, (0.25, 0.20), 0.26, "first blue carton")
    _carry(left, (0.15, 0.20), 0.315, "second blue carton")
    left.go((0.15, 0.24, CARRY_Z), FINGER_OPEN, "clear", 1.5)

    right.wait(left.total - t_pulled, FINGER_CLOSED, "hold the pusher back")
    right.go((EASE_X, HANDLE_Y, GRIP_Z), FINGER_CLOSED, "bring the pusher forward", 3.0)
    right.wait(1.0, FINGER_CLOSED, "steady")
    right.wait(0.8, FINGER_OPEN, "open")
    right.go((EASE_X, HANDLE_Y, TOP + 0.10), FINGER_OPEN, "rise off the handle", 1.2)
    right.go((0.30, -0.22, TOP + 0.12), FINGER_OPEN, "draw back to the right, clear", 1.5)
    right.wait(1.5, FINGER_OPEN, "arm clear while the spring closes the row")
    return {"left": left.legs, "right": right.legs}
