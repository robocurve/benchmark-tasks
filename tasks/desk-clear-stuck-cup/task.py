"""On the station table (an office desk top): three identical white glazed-ceramic pen cups (8 cm across, 10 cm tall,
280 g) stand in a row 32 cm in front of the arms, at y +14, 0 and -14 cm. One of them, drawn at random for each
trial, is stuck to the desk by two adhesive mounting pads on its base; it looks exactly like the other two. Between
the cups and the arms stands an empty grey polypropylene desk tray (33 x 12 cm inside, 4 cm walls). The robot must
put every cup that is not stuck down into the tray, without tearing the stuck one off the desk.

Real trials: the adhesive pads are replaced by a hidden steel disc under the stuck cup's base and a magnet under the
desk top with a 60 N pull-off force (check it with a luggage scale before each trial); a die picks the stuck cup
(1-2 left, 3-4 middle, 5-6 right). A cup lying off its magnet at the end means it was torn off.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Simulation: GLUED=0/1/2 picks the stuck cup (left, middle, right); unset, it is drawn at random at each reset.
"""

from __future__ import annotations

import dataclasses
import math
import os
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from jointkit import on_reset, on_step, part, set_glued  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Clear the desk for cleaning: put everything that isn't stuck down into the tray."

WHITE, RIM = (0.93, 0.93, 0.90), (0.35, 0.35, 0.38)
GREY = (0.55, 0.57, 0.60)
SIDES = 16


def ring(a_in, a_out, z0, z1, color, sides=SIDES):
    """A 16-sided ring (a disc when a_in = 0) between apothems a_in and a_out (m), from z0 to z1; faces at 0, 22.5,
    ... degrees, so two flats face +-y (where a straight-down grip's pads close)."""
    side = 2 * a_out * math.tan(math.pi / sides) * 1.02
    return [(((a_in + a_out) / 2 * math.cos(t), (a_in + a_out) / 2 * math.sin(t), (z0 + z1) / 2),
             (a_out - a_in, side, z1 - z0), color, math.degrees(t))
            for t in (2 * math.pi * k / sides for k in range(sides))]


# -- the pen cups (origin: centre of the base) ----------------------------------------------------------------------
R_OUT, WALL, FLOOR, H = 0.040, 0.004, 0.006, 0.100
CUP_MASS = 0.28
GLUE_N = float(os.environ.get("GLUE_N") or 60.0)  # pull-off force of the adhesive pads (N)
CUP_X = 0.32
CUP_Y = (0.14, 0.0, -0.14)                     # left, middle, right
NAMES = ("cup_left", "cup_middle", "cup_right")
CUP_BOXES = (ring(0, R_OUT, 0, FLOOR, WHITE) + ring(R_OUT - WALL, R_OUT, FLOOR, H - 0.004, WHITE)
             + ring(R_OUT - WALL, R_OUT, H - 0.004, H, RIM))
CUPS = [dataclasses.replace(part(n, pos=(CUP_X, y, TOP), boxes=CUP_BOXES, mass=CUP_MASS, color=WHITE, friction=0.6),
                            glue=GLUE_N) for n, y in zip(NAMES, CUP_Y)]

# -- the desk tray (fixed: it stands on the desk; inside 12 x 33 cm, 4 cm walls) -----------------------------------
TX, TY = 0.19, 0.0
T_IN_X, T_IN_Y, T_WALL, T_FLOOR, T_H = 0.12, 0.33, 0.006, 0.004, 0.04
TRAY = [Box("tray_floor", pos=(TX, TY, TOP + T_FLOOR / 2), size=(T_IN_X + 2 * T_WALL, T_IN_Y + 2 * T_WALL, T_FLOOR),
            color=GREY),
        Box("tray_wall_back", pos=(TX - (T_IN_X + T_WALL) / 2, TY, TOP + T_H / 2), size=(T_WALL, T_IN_Y + 2 * T_WALL, T_H),
            color=GREY),
        Box("tray_wall_front", pos=(TX + (T_IN_X + T_WALL) / 2, TY, TOP + T_H / 2), size=(T_WALL, T_IN_Y + 2 * T_WALL, T_H),
            color=GREY),
        Box("tray_wall_left", pos=(TX, TY + (T_IN_Y + T_WALL) / 2, TOP + T_H / 2), size=(T_IN_X, T_WALL, T_H), color=GREY),
        Box("tray_wall_right", pos=(TX, TY - (T_IN_Y + T_WALL) / 2, TOP + T_H / 2), size=(T_IN_X, T_WALL, T_H), color=GREY)]
OBJECTS = CUPS + TRAY

# -- which cup is stuck (drawn at each reset) ---------------------------------------------------------------------
_STATE: dict = {}


def _reset_state(environment=None) -> None:
    pick = os.environ.get("GLUED")
    glued = int(pick) if pick not in (None, "") else random.SystemRandom().randrange(3)
    _STATE.update(glued=glued, torn=False, knocked=False)
    if environment is not None:
        for i, n in enumerate(NAMES):
            set_glued(environment, n, i == glued)
        print(f"[desk] stuck cup: {NAMES[glued]}", flush=True)


_reset_state()
on_reset(_reset_state)

if os.environ.get("DESK_DEBUG"):
    _N = [0]

    @on_step
    def _debug(environment) -> None:
        from yam_arena.scene import object_states

        _N[0] += 1
        if _N[0] % 5 == 0 or _N[0] < 4:
            n = NAMES[_STATE["glued"]]
            c = object_states(environment, [n])[n]
            print(f"[desk] step {_N[0]} stuck {n} pos {np.round(c.pos * 1000, 1)} mm", flush=True)

# -- scoring --------------------------------------------------------------------------------------------------------
TORN = 0.003                   # the stuck cup moved more than 3 mm from its spot: torn off the desk
KNOCKED = math.radians(45)     # a cup tipped past 45 degrees (or off the desk)
UPRIGHT = math.radians(15)


def in_tray(c) -> bool:
    """Upright, base on the tray floor (within 1 cm), centre inside the tray's walls."""
    x, y, z = c.pos
    return bool(abs(x - TX) < T_IN_X / 2 - 0.01 and abs(y - TY) < T_IN_Y / 2 - 0.01
                and abs(z - (TOP + T_FLOOR)) < 0.01 and c.tilt() < UPRIGHT)


def _update(objects) -> float:
    g = _STATE["glued"]
    stuck = objects[NAMES[g]]
    _STATE["torn"] |= bool(np.linalg.norm(stuck.pos[:2] - (CUP_X, CUP_Y[g])) > TORN or stuck.pos[2] > TOP + TORN)
    _STATE["knocked"] |= any(objects[n].tilt() > KNOCKED or objects[n].pos[2] < TOP - 0.05 for n in NAMES)
    if _STATE["torn"] or _STATE["knocked"]:
        return 0.0
    loose = [objects[n] for i, n in enumerate(NAMES) if i != g]
    return 0.5 * sum(in_tray(c) and c.resting() for c in loose)


def success(objects) -> bool:
    """Both loose cups upright in the tray, the stuck cup never torn off, no cup knocked over; held 1 s."""
    return _update(objects) == 1.0


def score(objects) -> float:
    """0 if the stuck cup was ever torn off the desk (moved more than 3 mm) or any cup tipped past 45 degrees or fell
    off the desk. Otherwise 0.5 for each loose cup standing upright in the tray: 0, 0.5 or 1.0."""
    return _update(objects)


# -- scripted demo: the same plan whichever cup is stuck ------------------------------------------------------------
# Each cup is pinched lightly (pads 0.5 mm inside its flats) and lifted slowly. A loose cup (280 g) comes up with the
# hand; on the stuck one the pads slip up its sides below the pads' 60 N pull-off force, so it stays. DEMO_FORCE=1
# pinches hard (8 mm inside) and tears the stuck cup off.
OPEN = -0.04695
LIGHT = -(R_OUT - 0.0005)
FIRM = -(R_OUT - 0.008)
GRASP_Z = 0.065                # TCP above the cup's base when gripping it (pad tips 13 mm lower)
UP = 0.12                      # carrying height: the cup's base clears the tray walls by 1.5 cm
SLOTS = (0.11, -0.11, 0.0)     # where each cup goes in the tray (y), left, right, middle


def _move(cx, cy, ty, pinch):
    place_z = TOP + T_FLOOR + 0.003 + GRASP_Z
    return [(2.0, (cx, cy, TOP + UP), OPEN, "over the cup"),
            (1.5, (cx, cy, TOP + GRASP_Z), OPEN, "down"),
            (0.8, (cx, cy, TOP + GRASP_Z), pinch, "pinch"),
            (2.5, (cx, cy, TOP + UP), pinch, "lift slowly"),
            (2.0, (TX, ty, TOP + UP), pinch, "over the tray"),
            (1.5, (TX, ty, place_z), pinch, "lower"),
            (0.6, (TX, ty, place_z), OPEN, "let go"),
            (1.0, (TX, ty, TOP + UP), OPEN, "up")]


def plan(pinch=LIGHT) -> dict[str, list]:
    right = (_move(CUP_X, CUP_Y[2], SLOTS[1], pinch) + [(2.0, (TX, SLOTS[1], TOP + UP), OPEN, "wait for the left arm")]
             + _move(CUP_X, CUP_Y[1], SLOTS[2], pinch))
    right += [(1.5, (0.20, -0.28, TOP + 0.16), OPEN, "clear")]
    left = ([(1.0, (0.22, 0.25, TOP + 0.16), OPEN, "wait")] + _move(CUP_X, CUP_Y[0], SLOTS[0], pinch)
            + [(1.5, (0.20, 0.30, TOP + 0.16), OPEN, "clear")])
    return {"left": left, "right": right}


def demo():
    """Right arm: right cup into the tray's right slot, then the middle cup into the middle. Left arm: left cup into
    the left slot. Every cup is tried the same way; the stuck one slips out of the light pinch and stays."""
    from yam_arena.demo import Waypoint

    pinch = FIRM if os.environ.get("DEMO_FORCE") else LIGHT
    return {arm: [Waypoint(d, tuple(p), f, label, 0.0) for d, p, f, label in wps] for arm, wps in plan(pinch).items()}
