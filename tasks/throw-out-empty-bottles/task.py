"""On the station table: four identical opaque white plastic bottles (7 cm across, 15 cm tall, 0.5 l, caps on) stand in a
row 32 cm in front of the arms. Two of them, drawn at random, are full (0.55 kg); the other two are empty (40 g).
They look the same; only lifting them tells. In front of the row, between the arms, stands a green recycling bin
(24 x 16 cm inside, room for all four bottles, 4 cm walls).

The robot must put the empty bottles into the recycling bin and leave the full ones standing on the table.

Simulation: FULL=0,2 picks the full bottles (0 = leftmost); unset, two are drawn at random when the file loads.
"""

from __future__ import annotations

import math
import os
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from jointkit import part  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Put the empty bottles into the recycling bin. Leave the full ones on the table."

WHITE, CAP = (0.95, 0.95, 0.93), (0.92, 0.92, 0.90)
GREY, GREEN = (0.45, 0.47, 0.50), (0.15, 0.55, 0.25)
SIDES = 16
R, HB = 0.035, 0.15
BX = 0.32
BY = (0.15, 0.05, -0.05, -0.15)
NAMES = ("bottle_1", "bottle_2", "bottle_3", "bottle_4")
FULL_KG, EMPTY_KG = 0.55, 0.04

_pick = os.environ.get("FULL")
FULL = tuple(int(i) for i in _pick.split(",")) if _pick else tuple(sorted(random.SystemRandom().sample(range(4), 2)))


def ring(a_in, a_out, z0, z1, color, sides=SIDES):
    side = 2 * a_out * math.tan(math.pi / sides) * 1.02
    return [(((a_in + a_out) / 2 * math.cos(t), (a_in + a_out) / 2 * math.sin(t), (z0 + z1) / 2),
             (a_out - a_in, side, z1 - z0), color, math.degrees(t))
            for t in (2 * math.pi * k / sides for k in range(sides))]


BOTTLE = ring(0, R, 0, HB - 0.03, WHITE) + ring(0, 0.026, HB - 0.03, HB - 0.012, WHITE) + ring(0, 0.018, HB - 0.012, HB, CAP)
BOTTLES = [part(n, pos=(BX, y, TOP), boxes=BOTTLE, mass=FULL_KG if i in FULL else EMPTY_KG, color=WHITE, friction=0.8)
           for i, (n, y) in enumerate(zip(NAMES, BY))]

# -- the recycling bin (fixed, centred between the arms) --------------------------------------------------------------
CX, CYB = 0.18, 0.0
IN_X, IN_Y, WALL, WH, FLOOR = 0.16, 0.24, 0.006, 0.04, 0.004


def container(name, y, color):
    ox, oy = IN_X / 2 + WALL / 2, IN_Y / 2 + WALL / 2
    return [Box(f"{name}_floor", pos=(CX, y, TOP + FLOOR / 2), size=(IN_X + 2 * WALL, IN_Y + 2 * WALL, FLOOR), color=color),
            Box(f"{name}_back", pos=(CX + ox, y, TOP + WH / 2), size=(WALL, IN_Y + 2 * WALL, WH), color=color),
            Box(f"{name}_front", pos=(CX - ox, y, TOP + WH / 2), size=(WALL, IN_Y + 2 * WALL, WH), color=color),
            Box(f"{name}_left", pos=(CX, y + oy, TOP + WH / 2), size=(IN_X, WALL, WH), color=color),
            Box(f"{name}_right", pos=(CX, y - oy, TOP + WH / 2), size=(IN_X, WALL, WH), color=color)]


OBJECTS = BOTTLES + container("bin", CYB, GREEN)


# -- scoring --------------------------------------------------------------------------------------------------------
def in_bin(b) -> bool:
    x, y, z = b.pos
    return bool(abs(x - CX) < IN_X / 2 and abs(y - CYB) < IN_Y / 2 and z < TOP + FLOOR + 0.02)


def score(objects) -> float:
    """0 if a full bottle is in the bin, or any bottle tipped over (more than 30 degrees) or fell off the table.
    Otherwise 0.5 for each empty bottle in the bin: 1.0 with both, at the end of the episode."""
    total = 0.0
    for i, n in enumerate(NAMES):
        b = objects[n]
        if b.pos[2] < TOP - 0.05 or b.tilt() > math.radians(30):
            return 0.0
        if i in FULL:
            if in_bin(b):
                return 0.0
        else:
            total += 0.5 * in_bin(b)
    return total


# No success(): the trial does not end early once both empty bottles are in the bin, so every bottle gets tested;
# the score at the end of the episode counts (SCORE_OVER = "end").


# -- scripted demo: test-lift every bottle; full ones go straight back down, empty ones into the bin -----------------
OPEN, PINCH = -0.04695, -(R - 0.006)
GRASP = TOP + 0.10              # pads on the body 10 cm up, above the centre of mass: it hangs
TEST = TOP + 0.14               # lifted 4 cm to feel the weight
UP = TOP + 0.175                # bottoms clear the 4 cm walls by 1.5 cm
HOP = TOP + 0.175               # open fingertips pass 1 cm over the 15 cm bottle tops
YAW = math.pi / 2               # fingers close along x (neighbours stand along y)
SLOTS = [0.045, -0.045]         # places in the bin, filled in turn


def _test(i, yaw):
    x, y = BX, BY[i]
    return [(1.5, (x, y, HOP), OPEN, "over the bottle", yaw), (1.2, (x, y, GRASP), OPEN, "down", yaw),
            (0.8, (x, y, GRASP), PINCH, "grip", yaw), (1.2, (x, y, TEST), PINCH, "lift it: how heavy?", yaw),
            (0.8, (x, y, TEST), PINCH, "lift it: how heavy?", yaw)]


def _keep(i, yaw):
    x, y = BX, BY[i]
    return [(1.2, (x, y, GRASP + 0.002), PINCH, "full: set it back", yaw), (0.6, (x, y, GRASP + 0.002), OPEN, "let go", yaw),
            (1.0, (x, y, HOP), OPEN, "up", yaw)]


def _throw(i, slot, yaw):
    place = TOP + FLOOR + 0.004 + 0.10
    return [(1.5, (CX, slot, UP), PINCH, "empty: into the bin", yaw), (1.0, (CX, slot, place), PINCH, "lower", yaw),
            (0.6, (CX, slot, place), OPEN, "let go", yaw), (1.0, (CX, slot, HOP), OPEN, "up", yaw)]


def plan(full=FULL) -> dict[str, list]:
    """The left arm tests its two bottles, then moves away; only then does the right arm start on its two, so the
    hands never share the bin."""
    slots = list(SLOTS)
    park_l, park_r = (0.20, 0.30, UP), (0.20, -0.30, UP)
    left, right = [], []
    for i in (0, 1):
        left += _test(i, -YAW) + (_keep(i, -YAW) if i in full else _throw(i, slots.pop(0), -YAW))
    left += [(1.5, park_l, OPEN, "away", -YAW)]
    t_left = sum(w[0] for w in left)
    right += [(t_left, park_r, OPEN, "wait for the left arm", YAW)]
    for i in (3, 2):
        right += _test(i, YAW) + (_keep(i, YAW) if i in full else _throw(i, slots.pop(0), YAW))
    right += [(1.5, park_r, OPEN, "away", YAW)]
    return {"left": left, "right": right}


def demo():
    """Every bottle is lifted 4 cm to feel its weight: a full one is set straight back down, an empty one goes into
    the bin. The plan follows this trial's draw (FULL), as an operator would after lifting each bottle."""
    from yam_arena.demo import Waypoint

    return {arm: [Waypoint(d, tuple(p), f, label, yaw) for d, p, f, label, yaw in wps] for arm, wps in plan().items()}
