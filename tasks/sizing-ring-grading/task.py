"""On the station table, from left to right:

- In front of the left arm, a shallow dark grey picking tray (37 x 17 cm, 2.8 cm tall) holds five loose
  apricots in one row running away from the arms: orange spheres of one colour, three 44 mm across and two
  56 mm across.
- Between the arms, a closed wooden box: the small-grade bin (17 x 17 cm, 11.2 cm tall with its lid). The lid
  is a sizing board, 12 mm thick, with one 50 x 50 mm square opening in the middle. A fruit that passes
  through the opening drops into the bin; a fruit too big for it stays sitting in the opening.
- In front of the right arm, an open blue box: the large-grade bin (17 x 17 cm, 5.5 cm tall).

The tray, the bins and the board are fixed to the table; only the apricots move.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Sphere

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

# -- sizes (m) ---------------------------------------------------------------------------------------------
GRIPPER_MAX_OPENING = 0.094  # pad gap at FINGER_OPEN (yam_arena.constants; tests/test_constants.py)
R_SMALL = 0.022  # 44 mm apricot: passes the opening
R_LARGE = 0.028  # 56 mm apricot: does not
OPENING = 0.050  # square opening side of the sizing board
WALL = 0.010
SPAWN_GAP = 0.001  # loose fruit start this far above their support and settle onto it

# -- picking tray (in front of the left arm) ---------------------------------------------------------------
TRAY_X, TRAY_Y = 0.28, 0.20
TRAY_IN = (0.35, 0.15)  # inner length (x) and width (y)
TRAY_FLOOR_H, TRAY_RIM_H = 0.003, 0.025
TRAY_FLOOR_TOP = TOP + TRAY_FLOOR_H

# -- small-grade bin with the sizing board as its lid (between the arms) -----------------------------------
SB_X, SB_Y = 0.25, 0.0
SB_IN = 0.15  # square interior
SB_FLOOR_H = 0.005
SB_WALL_H = 0.095  # room for two small fruit stacked under the opening
SB_FLOOR_TOP = TOP + SB_FLOOR_H
BOARD_BOTTOM = SB_FLOOR_TOP + SB_WALL_H
BOARD_T = 0.012
BOARD_TOP = BOARD_BOTTOM + BOARD_T
SB_OUT = SB_IN + 2 * WALL

# -- large-grade bin (in front of the right arm) -----------------------------------------------------------
LB_X, LB_Y = 0.30, -0.21
LB_IN = 0.15
LB_FLOOR_H = 0.005
LB_WALL_H = 0.050
LB_FLOOR_TOP = TOP + LB_FLOOR_H
LB_RIM = LB_FLOOR_TOP + LB_WALL_H
LB_OUT = LB_IN + 2 * WALL

WOOD = (0.72, 0.56, 0.38)
BIN_WOOD = (0.55, 0.40, 0.26)
BLUE = (0.18, 0.36, 0.68)
TRAY_GREY = (0.22, 0.22, 0.24)
APRICOT = (0.96, 0.60, 0.20)


def _bin(prefix, cx, cy, inner, floor_h, wall_h, color):
    """An open box from fixed boxes: a floor slab and four walls standing on it."""
    out = inner + 2 * WALL
    z_wall = TOP + floor_h + wall_h / 2
    return [
        Box(f"{prefix}_floor", pos=(cx, cy, TOP + floor_h / 2), size=(out, out, floor_h), color=color),
        Box(f"{prefix}_wall_near", pos=(cx - (inner + WALL) / 2, cy, z_wall), size=(WALL, out, wall_h), color=color),
        Box(f"{prefix}_wall_far", pos=(cx + (inner + WALL) / 2, cy, z_wall), size=(WALL, out, wall_h), color=color),
        Box(f"{prefix}_wall_left", pos=(cx, cy + (inner + WALL) / 2, z_wall), size=(inner, WALL, wall_h), color=color),
        Box(f"{prefix}_wall_right", pos=(cx, cy - (inner + WALL) / 2, z_wall), size=(inner, WALL, wall_h), color=color),
    ]


def _tray():
    lx, ly = TRAY_IN
    ox, oy = lx + 2 * WALL, ly + 2 * WALL
    z_rim = TRAY_FLOOR_TOP + TRAY_RIM_H / 2
    rim, x, y = TRAY_RIM_H, TRAY_X, TRAY_Y
    return [
        Box("tray_floor", pos=(TRAY_X, TRAY_Y, TOP + TRAY_FLOOR_H / 2), size=(ox, oy, TRAY_FLOOR_H), color=TRAY_GREY),
        Box("tray_rim_near", pos=(x - (lx + WALL) / 2, y, z_rim), size=(WALL, oy, rim), color=TRAY_GREY),
        Box("tray_rim_far", pos=(x + (lx + WALL) / 2, y, z_rim), size=(WALL, oy, rim), color=TRAY_GREY),
        Box("tray_rim_left", pos=(x, y + (ly + WALL) / 2, z_rim), size=(lx, WALL, rim), color=TRAY_GREY),
        Box("tray_rim_right", pos=(x, y - (ly + WALL) / 2, z_rim), size=(lx, WALL, rim), color=TRAY_GREY),
    ]


def _board():
    """The sizing board: four fixed slabs covering the whole top of the small-grade bin except the opening."""
    z = BOARD_BOTTOM + BOARD_T / 2
    strip = (SB_OUT - OPENING) / 2  # 60 mm of board on each side of the opening
    off = (OPENING + strip) / 2
    return [
        Box("board_near", pos=(SB_X - off, SB_Y, z), size=(strip, SB_OUT, BOARD_T), color=WOOD),
        Box("board_far", pos=(SB_X + off, SB_Y, z), size=(strip, SB_OUT, BOARD_T), color=WOOD),
        Box("board_left", pos=(SB_X, SB_Y + off, z), size=(OPENING, strip, BOARD_T), color=WOOD),
        Box("board_right", pos=(SB_X, SB_Y - off, z), size=(OPENING, strip, BOARD_T), color=WOOD),
    ]


# The fruit, nearest the arms first. Names do not reveal the grade; GRADE does (for scoring and the demo).
FRUIT_X = (0.15, 0.215, 0.28, 0.345, 0.41)
FRUIT_R = (R_SMALL, R_LARGE, R_SMALL, R_LARGE, R_SMALL)
FRUIT_NAMES = ("apricot_a", "apricot_b", "apricot_c", "apricot_d", "apricot_e")
GRADE = {n: ("small" if r < OPENING / 2 else "large") for n, r in zip(FRUIT_NAMES, FRUIT_R, strict=True)}
RADIUS = dict(zip(FRUIT_NAMES, FRUIT_R, strict=True))
DENSITY = 1000.0  # kg/m^3, about that of a fresh apricot


def _mass(r):
    return round(DENSITY * 4 / 3 * math.pi * r**3, 3)


FRUIT = [
    Sphere(n, pos=(x, TRAY_Y, TRAY_FLOOR_TOP + r + SPAWN_GAP), radius=r, mass=_mass(r), color=APRICOT)
    for n, x, r in zip(FRUIT_NAMES, FRUIT_X, FRUIT_R, strict=True)
]

OBJECTS = [
    *_tray(),
    *_bin("small_bin", SB_X, SB_Y, SB_IN, SB_FLOOR_H, SB_WALL_H, BIN_WOOD),
    *_board(),
    *_bin("large_bin", LB_X, LB_Y, LB_IN, LB_FLOOR_H, LB_WALL_H, BLUE),
    *FRUIT,
]

# -- scoring -----------------------------------------------------------------------------------------------
# Each bin's inside as an axis-aligned region for a fruit's centre: within the inner walls, above the floor and
# below the rim (the small-grade bin's rim is the underside of the sizing board).
REGION = {
    "small": (np.array([SB_X - SB_IN / 2, SB_Y - SB_IN / 2, SB_FLOOR_TOP]),
              np.array([SB_X + SB_IN / 2, SB_Y + SB_IN / 2, BOARD_BOTTOM])),
    "large": (np.array([LB_X - LB_IN / 2, LB_Y - LB_IN / 2, LB_FLOOR_TOP]),
              np.array([LB_X + LB_IN / 2, LB_Y + LB_IN / 2, LB_RIM])),
}
OTHER = {"small": "large", "large": "small"}
OFF_TABLE_Z = TOP  # a fruit whose centre is below the table top is on the floor or falling off
OFF_TABLE_CAP = 0.6
PER_FRUIT = 0.2


def _inside(pos, grade):
    lo, hi = REGION[grade]
    return bool(np.all(pos >= lo) and np.all(pos <= hi))


def _tally(objects):
    """(fruit resting in their correct bin, fruit in the wrong bin, fruit off the table)."""
    correct = wrong = dropped = 0
    for name in FRUIT_NAMES:
        s = objects[name]
        grade = GRADE[name]
        if s.pos[2] < OFF_TABLE_Z:
            dropped += 1
        elif _inside(s.pos, grade):
            correct += int(s.resting())
        elif _inside(s.pos, OTHER[grade]):
            wrong += 1
    return correct, wrong, dropped


def success(objects):
    """All five apricots resting inside their correct bins."""
    correct, _, _ = _tally(objects)
    return correct == len(FRUIT_NAMES)


def distance(objects):
    """Sum over the apricots of the metres from each centre to the inside of its correct bin (0 when inside)."""
    total = 0.0
    for name in FRUIT_NAMES:
        lo, hi = REGION[GRADE[name]]
        p = objects[name].pos
        total += float(np.linalg.norm(np.maximum(lo - p, 0) + np.maximum(p - hi, 0)))
    return total


def score(objects):
    """0.2 per apricot resting in its correct bin, minus 0.2 per apricot in the wrong bin (floored at 0); any
    apricot below the table top caps the total at 0.6."""
    correct, wrong, dropped = _tally(objects)
    total = max(0.0, PER_FRUIT * (correct - wrong))
    if dropped:
        total = min(total, OFF_TABLE_CAP)
    return round(total, 6)


# -- scripted demo -----------------------------------------------------------------------------------------
GRIP_ABOVE_CENTRE = 0.006  # TCP this far above a fruit's centre: the pads still span its equator
SEAT_Z = BOARD_TOP + math.sqrt(R_LARGE**2 - (OPENING / 2) ** 2)  # a large fruit's centre sitting in the opening
DROP = (SB_X, SB_Y, BOARD_TOP + R_LARGE + 0.010 + GRIP_ABOVE_CENTRE)  # a large fruit's bottom 10 mm over the board
TRAVEL_Z = DROP[2]  # left arm: carrying height
LEFT_CLEAR = (SB_X, 0.17, TRAVEL_Z)  # left arm waits here, off the board, after each drop
RIGHT_Z = TOP + 0.160
RIGHT_PARK = (0.25, -0.20, RIGHT_Z)
LB_SPOTS = [(LB_X - 0.035, LB_Y), (LB_X + 0.035, LB_Y)]  # where the right arm sets down the two large fruit
LB_RELEASE_Z = LB_FLOOR_TOP + R_LARGE + 0.010 + GRIP_ABOVE_CENTRE
LIFT_AFTER_DROP = 0.03  # left arm rises this far before sliding off, so the open pads do not drag a fruit out
SETTLE_S = 1.0  # after a drop, before the right arm reaches for a fruit sitting in the opening


class _Track:
    """A waypoint list with its running end time."""

    def __init__(self, start_pos, finger=FINGER_OPEN):
        self.wps, self.t, self.pos, self.finger = [], 0.0, start_pos, finger

    def go(self, duration, pos=None, finger=None, label=""):
        if duration <= 0:
            return
        self.pos = self.pos if pos is None else pos
        self.finger = self.finger if finger is None else finger
        self.wps.append(Waypoint(duration, self.pos, self.finger, label))
        self.t += duration

    def wait_until(self, t, label="wait"):
        self.go(t - self.t, label=label)


def demo():
    """The left arm takes each apricot from the tray and lets go of it 10 mm above the sizing board's opening:
    the small ones drop through into the small-grade bin, the large ones stay sitting in the opening. The right
    arm lifts each one that stays out of the opening and sets it in the large-grade bin, while the left arm
    waits beside the board. Grips are straight down, fingers closing along y."""
    left = _Track(None)
    right = _Track(None)
    right.go(1.5, RIGHT_PARK, FINGER_OPEN, "to the park spot")
    board_free = 0.0
    spots = iter(LB_SPOTS)
    for name, x, r in zip(FRUIT_NAMES, FRUIT_X, FRUIT_R, strict=True):
        grip = (x, TRAY_Y, TRAY_FLOOR_TOP + r + GRIP_ABOVE_CENTRE)
        left.go(1.4, (x, TRAY_Y, TRAVEL_Z), FINGER_OPEN, f"over {name}")
        left.go(1.2, grip, label="descend")
        left.go(0.6, finger=FINGER_CLOSED, label="close")
        left.go(1.0, (x, TRAY_Y, TRAVEL_Z), label="lift")
        left.wait_until(board_free, "wait for the board")
        left.go(1.5, DROP, label="carry over the opening")
        left.go(0.5, finger=FINGER_OPEN, label="let go over the opening")
        dropped_at = left.t
        left.go(0.6, (SB_X, SB_Y, TRAVEL_Z + LIFT_AFTER_DROP), label="lift off the fruit")
        left.go(0.8, LEFT_CLEAR, label="clear the board")
        board_free = left.t
        if GRADE[name] == "large":
            seat = (SB_X, SB_Y, SEAT_Z + GRIP_ABOVE_CENTRE)
            spot = next(spots)
            right.wait_until(max(dropped_at + SETTLE_S, left.t), "wait for the left arm")
            right.go(1.2, (SB_X, SB_Y, RIGHT_Z), FINGER_OPEN, f"over {name} in the opening")
            right.go(1.0, seat, label="descend")
            right.go(0.6, finger=FINGER_CLOSED, label="close")
            right.go(1.0, (SB_X, SB_Y, RIGHT_Z), label="lift out of the opening")
            board_free = right.t + 0.5
            right.go(1.5, (*spot, RIGHT_Z), label="carry to the large-grade bin")
            right.go(1.0, (*spot, LB_RELEASE_Z), label="lower")
            right.go(0.5, finger=FINGER_OPEN, label="let go")
            right.go(0.8, (*spot, RIGHT_Z), label="up")
            right.go(1.0, RIGHT_PARK, label="back to the park spot")
    left.go(1.5, (0.20, TRAY_Y, TRAVEL_Z), FINGER_OPEN, "done")
    end = max(left.t, right.t) + 1.5  # hold still past the last release so the 1 s success hold can complete
    left.wait_until(end, "hold")
    right.wait_until(end, "hold")
    return {"left": left.wps, "right": right.wps}
