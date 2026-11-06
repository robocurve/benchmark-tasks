"""On the station table: a long steel prep counter runs straight out from between the arms and ends at a dark line,
the mouth of an open-topped pizza oven. Beyond the line the oven's pale stone deck continues at the counter's height,
with dark walls on its left, right and far sides. A wooden pizza peel lies flat on the counter, its handle toward the
robot and its thin square blade toward the oven, the blade's front edge just short of the dark line. A small pizza
(pale crust, red topping) sits on the blade.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import parts  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

STEEL, DARK, STONE, WOOD = (0.62, 0.64, 0.67), (0.12, 0.12, 0.13), (0.86, 0.83, 0.76), (0.80, 0.64, 0.42)
CRUST, RED = (0.93, 0.82, 0.58), (0.80, 0.12, 0.08)

YC = 0.050  # y of the counter's and oven's centre line (toward the left arm)
CT = TOP + 0.040  # counter top = stone top

# Counter: steel, x 0.15-0.73, then a 10 mm dark mouth line to x 0.74 (the oven mouth), 300 mm wide.
MOUTH = 0.740
COUNTER = parts.Part(
    "counter",
    pos=(0.0, YC, TOP),
    shapes=[
        parts.box((0.58, 0.30, 0.040), at=(0.44, 0.0, 0.020), color=STEEL),
        parts.box((0.01, 0.30, 0.040), at=(0.735, 0.0, 0.020), color=DARK),
    ],
    friction=(0.4, 0.4),  # brushed steel
)

# Oven: dark plinth, pale stone deck x 0.74-0.89 and 200 mm wide (top flush with the counter), dark walls 50 mm high.
D0, D1, STONE_HALF_Y = MOUTH, 0.890, 0.100
WALL_H = 0.050
OVEN = parts.Part(
    "oven",
    pos=(0.0, YC, TOP),
    shapes=[
        parts.box((D1 - D0, 2 * STONE_HALF_Y, 0.010), at=((D0 + D1) / 2, 0.0, 0.035), color=STONE),
        parts.box((0.170, 0.250, 0.030), at=(0.825, 0.0, 0.015), color=DARK),  # plinth x 0.74-0.91
        parts.box((0.170, 0.015, 0.010 + WALL_H), at=(0.825, 0.1175, 0.030 + (0.010 + WALL_H) / 2), color=DARK),
        parts.box((0.170, 0.015, 0.010 + WALL_H), at=(0.825, -0.1175, 0.030 + (0.010 + WALL_H) / 2), color=DARK),
        parts.box((0.015, 0.220, 0.010 + WALL_H), at=(0.9025, 0.0, 0.030 + (0.010 + WALL_H) / 2), color=DARK),
    ],
    friction=(1.0, 1.0),  # baking stone
)

# Peel: origin at the blade's rear bottom edge, x along the peel toward the blade tip.
BLADE = (0.110, 0.110, 0.004)
HANDLE = (0.420, 0.024, 0.024)
PEEL_X0 = 0.610  # blade rear at the start; tip at 0.720, 20 mm short of the mouth
PEEL = parts.Part(
    "peel",
    pos=(PEEL_X0, YC, CT + 0.0005),
    shapes=[
        parts.box(BLADE, at=(BLADE[0] / 2, 0.0, BLADE[2] / 2), color=WOOD),
        parts.box(HANDLE, at=(-HANDLE[0] / 2, 0.0, HANDLE[2] / 2), color=WOOD),
    ],
    mass=0.20,
    friction=(0.4, 0.4),  # dry wood
)

# Pizza: origin at the crust's bottom centre. 90 mm crust, 8 mm thick, 76 mm red topping disc 3 mm thick on top.
R = 0.045
PIZZA = parts.Part(
    "pizza",
    pos=(PEEL_X0 + BLADE[0] / 2, YC, CT + BLADE[2] + 0.0015),
    shapes=[
        parts.cylinder(R, 0.008, at=(0.0, 0.0, 0.004), color=CRUST, axis="z"),
        parts.cylinder(0.038, 0.003, at=(0.0, 0.0, 0.0095), color=RED, axis="z"),
    ],
    mass=0.15,
    friction=(0.1, 0.1),  # floured dough base; PhysX averages: 0.25 on the peel, 0.55 on the stone, 0.25 on steel
)

OBJECTS = [COUNTER, OVEN, PEEL, PIZZA]

FLAT_DEG = 5.0  # pizza tilt allowed
ON_STONE = 0.002  # pizza's lowest point within 2 mm of the stone top (on the blade it is 4 mm up)
TOUCH = 0.0015  # pizza within 1.5 mm of the counter top while over the counter counts as touching it
FALLEN = 0.005  # pizza bottom more than 5 mm below the counter top: it fell off (table, plinth)
EPS = 1e-6

# Whole-trial record: has the pizza touched the counter or fallen? Cleared by a scene reset (on_reset), and offline
# when every loose object is back within 0.5 mm of its start and at rest after a jump of more than 20 mm since the
# previous call (only a reset moves things that far in one step).
_HIST = {"touched": False, "last": None}
RESET_MM, RESET_JUMP = 0.0005, 0.020


@parts.on_reset
def _clear(sim):
    _HIST["touched"] = False
    _HIST["last"] = None
    _HIST["key"] = None


def tilt_deg(quat_xyzw) -> float:
    x, y, z, w = np.asarray(quat_xyzw, float) / np.linalg.norm(quat_xyzw)
    return math.degrees(math.acos(float(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _reset_rule(objects):
    now = {n: np.asarray(objects[n].pos, float) for n in ("peel", "pizza")}
    last = _HIST["last"]
    jumped = last is None or any(np.linalg.norm(now[n] - last[n]) > RESET_JUMP for n in now)
    home = all(np.linalg.norm(now[n] - np.asarray(o.pos)) <= RESET_MM and objects[n].resting()
               for n, o in (("peel", PEEL), ("pizza", PIZZA)))
    if jumped and home:
        _HIST["touched"] = False
    _HIST["last"] = now


def _record(objects):
    p = objects["pizza"]
    over_counter = p.pos[0] - R < MOUTH - EPS
    if (over_counter and p.low[2] < CT + TOUCH - EPS) or p.low[2] < CT - FALLEN - EPS:
        _HIST["touched"] = True


def _pizza_on_stone(p) -> bool:
    x, y = p.pos[0], p.pos[1]
    return bool(
        tilt_deg(p.quat_xyzw) <= FLAT_DEG + EPS
        and x - R >= D0 - EPS and x + R <= D1 + EPS and abs(y - YC) + R <= STONE_HALF_Y + EPS
        and abs(p.low[2] - CT) <= ON_STONE + EPS
        and p.resting()
    )


PEEL_LOW = 0.005  # the peel's lowest point no more than 5 mm below the counter top: on the counter or held


def _peel_out(peel) -> bool:
    """Wholly on the robot's side of the mouth line, and on the counter or held (not knocked onto the table)."""
    return bool(peel.high[0] <= MOUTH + EPS and peel.low[2] >= CT - PEEL_LOW - EPS)


def _update(objects):
    """Update the whole-trial record once per control step (success() and score() both see the same step)."""
    key = tuple(float(v) for n in ("peel", "pizza") for v in (*objects[n].pos, *objects[n].lin_vel))
    if _HIST.get("key") == key:
        return
    _HIST["key"] = key
    _reset_rule(objects)
    _record(objects)


def success(objects):
    _update(objects)
    return bool(not _HIST["touched"] and _pizza_on_stone(objects["pizza"]) and _peel_out(objects["peel"]))


def score(objects):
    """Pass/fail: 1.0 when success holds, otherwise 0."""
    return 1.0 if success(objects) else 0.0


# Demo (left arm): grip the handle 50 mm from its end, slide the peel 170 mm into the oven at an ordinary pace, then
# pull it back 200 mm in 0.1 s, so the blade leaves faster than friction can carry the pizza along.
GRIP_Z = CT + 0.022  # TCP; the pad tips 9 mm above the counter
HOVER_Z = CT + 0.100
GRIP_X0 = PEEL_X0 - HANDLE[0] + 0.050
PUSH = 0.170
PULL = 0.200


def demo():
    y = YC
    x_in = GRIP_X0 + PUSH
    x_out = x_in - PULL
    return {
        "left": [
            Waypoint(1.5, (GRIP_X0, y, HOVER_Z), FINGER_OPEN, "over the handle"),
            Waypoint(1.5, (GRIP_X0, y, GRIP_Z), FINGER_OPEN, "down to the handle"),
            Waypoint(0.8, (GRIP_X0, y, GRIP_Z), FINGER_CLOSED, "grip"),
            Waypoint(3.0, (x_in, y, GRIP_Z), FINGER_CLOSED, "slide the peel in"),
            Waypoint(1.0, (x_in, y, GRIP_Z), FINGER_CLOSED, "settle"),
            Waypoint(0.10, (x_out, y, GRIP_Z), FINGER_CLOSED, "pull back sharply"),
            Waypoint(1.0, (x_out, y, GRIP_Z), FINGER_CLOSED, "hold"),
            Waypoint(0.6, (x_out, y, GRIP_Z), FINGER_OPEN, "let go"),
            Waypoint(1.0, (x_out, y, HOVER_Z), FINGER_OPEN, "up"),
        ]
    }
