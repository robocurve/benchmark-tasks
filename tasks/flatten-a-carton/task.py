"""On the station table: an empty corrugated shipping carton lying on its side with both ends open, to be knocked
down flat for recycling.

The carton is a single-wall regular slotted carton, 200 x 150 x 150 mm inside (length x width x height), of 3 mm
corrugated board, 116 g. It lies on one of its long panels with its tube running left to right, so the arms look at
its near wall (200 x 150 mm, standing up) with its open ends to the left and right. All eight flaps are folded out,
in line with their panels: the top end was opened to unpack it and the tape on the bottom end has been slit.

The creases hold it square: each of the four corner creases is a spring of 0.12 N m/rad (0.19 N m at 90 degrees of
fold), so the carton springs back square from any shear it is pushed to and let go. Pushed level at its side, the
light carton slides on the table before its creases give: sliding it takes 0.5 N, shearing it 10 degrees about as
much. It folds over only when it is pressed down hard enough as well as over, and it has to be pressed all the way:
once fully flat (each corner folded to within 6 degrees of flat) the creases set, as real ones do, and it stays
flat; let go before that, it springs back up.

Run it:    inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:      python demo.py task.py out_dir
Second layout: set the environment variable CARTON_LOT=B before the command (default A).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from cartonkit import PANELS, Carton, finger_points, on_reset, on_step, set_crease_spring  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Flatten the empty cardboard box for recycling, so that it lies flat on the table, and let go of it."

KRAFT = (0.66, 0.50, 0.32)
G = 9.81

# -- the carton --------------------------------------------------------------------------------------------------------
ACROSS = 0.200          # the panel it lies on and the top panel (the carton's 200 mm length), near to far
TALL = 0.150            # the near and far walls (its 150 mm width), standing up
TUBE = 0.150            # along the tube, left to right (its 150 mm height)
FLAP = 0.075            # every flap is half the width deep
LONG = TUBE + 2 * FLAP  # each panel with its two flaps folded out: 300 mm
BOARD = 0.003           # B-flute single-wall board
GRAMMAGE = 0.55         # kg per square metre
PANEL_MASS = tuple(round(GRAMMAGE * w * LONG, 4) for w in (ACROSS, TALL, ACROSS, TALL))   # 33, 25, 33, 25 g
MASS = sum(PANEL_MASS)
CREASE = 0.12           # N m/rad per corner crease, toward square
FRICTION = dict(static_friction=0.45, dynamic_friction=0.38, combine="average")   # kraft board on the table top

LOT = os.environ.get("CARTON_LOT", "A")
LOTS = {"A": (0.215, 0.000), "B": (0.235, 0.030)}      # where the near bottom crease runs: x, and y of the middle
if LOT not in LOTS:
    raise ValueError(f"CARTON_LOT must be one of {sorted(LOTS)}, got {LOT!r}")
NEAR_X, MID_Y = LOTS[LOT]
SPAWN_GAP = 0.0003

CARTON = Carton("carton", pos=(NEAR_X, MID_Y, TOP + SPAWN_GAP), size=(ACROSS + BOARD, LONG, TALL + BOARD),
                mass=MASS, color=KRAFT, across=ACROSS, tall=TALL, long=LONG, flap=FLAP, board=BOARD,
                panel_mass=PANEL_MASS, crease=CREASE, crease_damping=0.004, rack_limit=88.0, **FRICTION)
OBJECTS = [CARTON]
NAMES = ["carton"]

# -- scoring constants -------------------------------------------------------------------------------------------------
SET_RACK = 84.0         # degrees of shear at which the creases set (each corner within 6 degrees of flat)
FLAT_RACK = 80.0        # a carton counts as flat at this much shear or more ...
FLAT_HEIGHT = 0.025     # ... with no part of it higher than this above the table
ON_TABLE = 0.010        # and its lowest point within this of the table top
REST_SPEED = 0.05       # m/s, every panel
CLEAR_XY = 0.03         # let go: no gripper's grasp point over the carton's footprint grown by this ...
CLEAR_Z = 0.06          # ... and lower than this above the table

_STATE: dict = {}


def _clear() -> None:
    _STATE.update(set=False, touched=False)


_clear()


@on_reset
def _reset(environment) -> None:
    _clear()
    set_crease_spring(environment, "carton", CREASE)


def _held(carton, hands) -> bool:
    lo, hi = np.asarray(carton.low, float), np.asarray(carton.high, float)
    for h in hands:
        h = np.asarray(h, float)
        if (lo[0] - CLEAR_XY <= h[0] <= hi[0] + CLEAR_XY and lo[1] - CLEAR_XY <= h[1] <= hi[1] + CLEAR_XY
                and h[2] < TOP + CLEAR_Z):
            return True
    return False


@on_step
def _watch(environment) -> None:
    """Runs after every control step: lets the creases set once the carton is pressed flat, and notes whether a
    gripper is on it."""
    from yam_arena.scene import object_states   # importable only once the simulator has started

    carton = object_states(environment, NAMES)["carton"]
    if not _STATE["set"] and abs(carton.rack_deg) >= SET_RACK:
        _STATE["set"] = True
        set_crease_spring(environment, "carton", 0.0)
    _STATE["touched"] = _held(carton, finger_points(environment))


def is_flat(carton) -> bool:
    """Sheared nearly flat, nothing higher than 2.5 cm above the table, and lying on the table."""
    return (abs(float(carton.rack_deg)) >= FLAT_RACK and float(carton.high[2]) - TOP <= FLAT_HEIGHT
            and float(carton.low[2]) >= TOP - ON_TABLE)


def is_still(carton) -> bool:
    return all(float(np.linalg.norm(v)) < REST_SPEED for _, _, v in carton.panels.values())


def score(objects) -> float:
    carton = objects["carton"]
    return 1.0 if is_flat(carton) and is_still(carton) and not _STATE["touched"] else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0


def report(objects) -> dict:
    """Where everything is, for result files."""
    c = objects["carton"]
    return {"rack_deg": round(float(c.rack_deg), 1), "height_cm": round(100 * (float(c.high[2]) - TOP), 2),
            "low_cm": round(100 * (float(c.low[2]) - TOP), 2), "flat": is_flat(c), "still": is_still(c),
            "creases_set": _STATE["set"], "gripper_on_it": _STATE["touched"],
            "origin_cm": [round(100 * float(v), 2) for v in c.pos],
            "moved_cm": round(100 * float(np.linalg.norm(np.asarray(c.pos[:2]) - np.array([NEAR_X, MID_Y]))), 2),
            "panels_cm": {p: [round(100 * float(v), 2) for v in c.panels[p][0]] for p in PANELS}}


def demo() -> dict:
    """yam_scripted plays straight-down pick-and-place only; the two-handed demo is demo.py. Returning no waypoints
    holds both arms still, which is a check of the scene by itself."""
    return {}
