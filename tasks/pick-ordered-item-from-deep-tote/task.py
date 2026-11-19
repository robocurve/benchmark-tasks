"""On the station table, in front of the left arm: a grey plastic parts tote with 12 cm walls, holding two small product
boxes of the same size, one red and one orange. To its right, in front of the gap between the arms: a shallow blue
order bin, empty. Nothing else is on the table."""

# A warehouse order picker's core operation (each picking): take exactly the ordered item from a storage tote and put
# it in the customer's order bin, leaving look-alike stock in the tote. Mis-picks (the wrong item shipped) are the costly
# error picking processes are built to prevent.
#
# Reliability stressors (one fixed level each): the tote's walls hide much of its floor from the station's top camera
# (check_task.py reports how much), and an orange look-alike of the ordered red item lies beside it.
#
# World frame: x forward (away from the arms), y left, z up, metres. The arm bases stand at x 0, y +-0.305.

import json
import math
import os
import sys

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Pick the red item from the tote into the order bin."

# -- the scene -------------------------------------------------------------------------------------------------------

ITEM_SIZE = (0.05, 0.08, 0.03)  # a small product carton, 80 x 50 x 30 mm, lying flat with its 80 mm side across (y); 150 g
ITEM_MASS = 0.15
RED = (0.80, 0.10, 0.10)
ORANGE = (0.95, 0.50, 0.10)
GREY = (0.55, 0.57, 0.60)
BLUE = (0.15, 0.35, 0.75)

WALL = 0.012
FLOOR = 0.01

# The tote: grey polypropylene, inside 24 x 18 cm, walls 12 cm tall above its 1 cm floor.
TOTE_XY = (0.255, 0.28)
TOTE_IN = (0.24, 0.18)
TOTE_WALL_H = 0.12
TOTE_FLOOR_TOP = TOP + FLOOR
TOTE_RIM = TOTE_FLOOR_TOP + TOTE_WALL_H

# The order bin: blue polypropylene, inside 16 x 12 cm, walls 6 cm tall above its 1 cm floor.
BIN_XY = (0.20, 0.0)
BIN_IN = (0.16, 0.12)
BIN_WALL_H = 0.06
BIN_FLOOR_TOP = TOP + FLOOR
BIN_RIM = BIN_FLOOR_TOP + BIN_WALL_H

# The items lie flat on the tote floor: the ordered red one near the tote's front-right corner (4 cm from the front
# wall, 5 mm from the right wall), where the walls hide part of it from the top camera; the orange look-alike 12 cm
# behind it. The red item's place leaves room for the gripper's housing, which reaches 5.1 cm either side of the tool
# point along the fingers' closing direction and 3.4 cm across it (from the robot's collision meshes): in simulation
# a corner placement put the housing on the front wall's rim.
RED_START = (TOTE_XY[0] - TOTE_IN[0] / 2 + 0.065, TOTE_XY[1] - TOTE_IN[1] / 2 + 0.045, TOTE_FLOOR_TOP + ITEM_SIZE[2] / 2)
ORANGE_START = (RED_START[0] + 0.12, RED_START[1], RED_START[2])


def _container(name, xy, inner, wall_h, color):
    cx, cy = xy
    ix, iy = inner
    ox, oy = ix + 2 * WALL, iy + 2 * WALL
    zc = TOP + FLOOR + wall_h / 2
    return [
        Box(f"{name}_floor", pos=(cx, cy, TOP + FLOOR / 2), size=(ox, oy, FLOOR), color=color),
        Box(f"{name}_wall_near", pos=(cx - ix / 2 - WALL / 2, cy, zc), size=(WALL, oy, wall_h), color=color),
        Box(f"{name}_wall_far", pos=(cx + ix / 2 + WALL / 2, cy, zc), size=(WALL, oy, wall_h), color=color),
        Box(f"{name}_wall_left", pos=(cx, cy + iy / 2 + WALL / 2, zc), size=(ix, WALL, wall_h), color=color),
        Box(f"{name}_wall_right", pos=(cx, cy - iy / 2 - WALL / 2, zc), size=(ix, WALL, wall_h), color=color),
    ]


RED_ITEM = Box("red_item", pos=RED_START, size=ITEM_SIZE, mass=ITEM_MASS, color=RED)
ORANGE_ITEM = Box("orange_item", pos=ORANGE_START, size=ITEM_SIZE, mass=ITEM_MASS, color=ORANGE)
TOTE = _container("tote", TOTE_XY, TOTE_IN, TOTE_WALL_H, GREY)
ORDER_BIN = _container("order_bin", BIN_XY, BIN_IN, BIN_WALL_H, BLUE)
OBJECTS = [RED_ITEM, ORANGE_ITEM, *TOTE, *ORDER_BIN]

# -- scoring ---------------------------------------------------------------------------------------------------------
#
# Milestones for the red (ordered) item, each needing the ones before it; the score is the highest reached:
#   0.25  lifted:      its lowest point at least 3 cm above the tote floor.
#   0.50  out of tote: its lowest point above the tote's rim, and its centre outside the tote's inside footprint.
#   0.75  in the bin:  its centre over the order bin's inside footprint and its lowest point below the bin's rim.
#   1.00  picked:      success (below), held for 1 s.
# Success: the red item lies entirely inside the order bin (seen from above, within 3 mm of the inner walls), entirely
# below the bin's rim, still (under 1 cm/s and 0.5 rad/s), released (no gripper tool point within 2.5 cm of it), and
# the orange item is still inside the tote.
# Mis-pick: the score is 0 if the orange item ever ends up over the order bin, or ever leaves the tote and rests
# outside it (it is not the ordered item). The score is also 0 if the red item falls off the table.

LIFT = 0.03
INSIDE_TOL = 0.003
RELEASED_MARGIN = 0.025
OFF_TABLE = 0.03
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
HZ_OFFLINE = 20.0


def _owner():
    """The yam_arena embodiment calling this hook (it holds the trial clock ``_t`` and the simulator ``_env``)."""
    try:
        frame = sys._getframe(2)
    except ValueError:
        return None
    for _ in range(16):
        if frame is None:
            return None
        owner = frame.f_locals.get("self")
        if owner is not None and hasattr(owner, "_t") and hasattr(owner, "task"):
            return owner
        frame = frame.f_back
    return None


# Per-trial memory. A new trial is recognised by the embodiment's clock going backwards. Offline (no simulator), the
# caller drives time with new_trial() and tick().
_MEM = {"key": None, "t": -1.0, "reached": 0.0, "mispick": False, "fell": False, "held_since": None,
        "offline_tcps": None, "offline_t": 0.0}


def new_trial(offline_tcps=None) -> None:
    _MEM.update(key=None, t=-1.0, reached=0.0, mispick=False, fell=False, held_since=None,
                offline_tcps=offline_tcps, offline_t=0.0)


def tick(dt: float = 1.0 / HZ_OFFLINE) -> None:
    """Offline only: advance the trial clock by one control step."""
    _MEM["offline_t"] += dt


def _now(owner) -> float:
    return float(owner._t) if owner is not None else float(_MEM["offline_t"])


def _tcps(owner) -> list:
    """World positions of both grippers' tool points."""
    if owner is None:
        fn = _MEM.get("offline_tcps")
        return list(fn()) if fn is not None else []
    env = getattr(owner, "_env", None)
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        raw = art.data.body_pos_w
        pos = (getattr(raw, "torch", raw)[0] - origin).detach().cpu().numpy().astype(float)
        g = pos[names.index("gripper")]
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]) / 2
        out.append(g + (mid - g) / (np.linalg.norm(mid - g) + 1e-12) * 0.134)
    return out


def _within(x, y, centre, inner, tol=0.0) -> bool:
    return (abs(x - centre[0]) <= inner[0] / 2 + tol) and (abs(y - centre[1]) <= inner[1] / 2 + tol)


def _box_inside(obj, centre, inner, rim) -> bool:
    lo, hi = np.asarray(obj.low, float), np.asarray(obj.high, float)
    return (_within(lo[0], lo[1], centre, inner, INSIDE_TOL) and _within(hi[0], hi[1], centre, inner, INSIDE_TOL)
            and float(hi[2]) <= rim)


def _released(obj, owner) -> bool:
    lo = np.asarray(obj.low, float) - RELEASED_MARGIN
    hi = np.asarray(obj.high, float) + RELEASED_MARGIN
    return not any(bool(np.all(t >= lo) and np.all(t <= hi)) for t in _tcps(owner))


def _picked(objects, owner) -> bool:
    red, orange = objects["red_item"], objects["orange_item"]
    return (_box_inside(red, BIN_XY, BIN_IN, BIN_RIM) and red.resting(0.01, 0.5) and _released(red, owner)
            and _box_inside(orange, TOTE_XY, TOTE_IN, TOTE_RIM + ITEM_SIZE[0]))


def _update(objects) -> None:
    owner = _owner()
    key = id(owner) if owner is not None else "offline"
    t = _now(owner)
    if key != _MEM["key"] or (owner is not None and t < _MEM["t"]):
        keep = _MEM.get("offline_tcps") if owner is None else None
        offline_t = _MEM["offline_t"]
        new_trial(keep)
        _MEM["offline_t"] = offline_t
        _MEM["key"] = key
    if t == _MEM["t"]:  # success() and score() both call this each step: update once per step
        return
    _MEM["t"] = t
    red, orange = objects["red_item"], objects["orange_item"]
    rx, ry, _ = (float(v) for v in red.pos)
    ox, oy, _ = (float(v) for v in orange.pos)
    red_low = float(red.low[2])
    if red_low < TOP - OFF_TABLE:
        _MEM["fell"] = True
    if _within(ox, oy, BIN_XY, BIN_IN):
        _MEM["mispick"] = True
    elif (not _within(ox, oy, TOTE_XY, TOTE_IN)) and orange.resting(0.01, 0.5) and float(orange.low[2]) < TOTE_RIM:
        _MEM["mispick"] = True
    r = _MEM["reached"]
    if r < 0.25 and red_low >= TOTE_FLOOR_TOP + LIFT:
        r = 0.25
    if r >= 0.25 and r < 0.5 and red_low > TOTE_RIM and not _within(rx, ry, TOTE_XY, TOTE_IN):
        r = 0.5
    if r >= 0.5 and r < 0.75 and _within(rx, ry, BIN_XY, BIN_IN) and red_low < BIN_RIM:
        r = 0.75
    _MEM["reached"] = r
    if r >= 0.75 and not (_MEM["mispick"] or _MEM["fell"]) and _picked(objects, owner):
        if _MEM["held_since"] is None:
            _MEM["held_since"] = t
    else:
        _MEM["held_since"] = None
    _trace(owner, objects, r)


def _held_long_enough() -> bool:
    hs = _MEM["held_since"]
    return hs is not None and (_MEM["t"] - hs) >= SUCCESS_HOLD_S - 1e-9


def _trace(owner, objects, reached) -> None:
    """With PICK_TRACE=<file> set, append one JSON line per control step (simulator only)."""
    path = os.environ.get("PICK_TRACE")
    if not path or owner is None:
        return
    rec = {"t": round(_now(owner), 3), "reached": reached, "mispick": _MEM["mispick"],
           "tcps": [[round(float(v), 4) for v in tcp] for tcp in _tcps(owner)]}
    for name in ("red_item", "orange_item"):
        o = objects[name]
        rec[name] = {"pos": [round(float(v), 4) for v in o.pos], "low": [round(float(v), 4) for v in o.low],
                     "high": [round(float(v), 4) for v in o.high]}
    with open(path, "a") as f:
        f.write(json.dumps(rec) + "\n")


def success(objects) -> bool:
    """True while the pick is complete (the embodiment ends the trial once this holds for SUCCESS_HOLD_S)."""
    _update(objects)
    if _MEM["mispick"] or _MEM["fell"] or _MEM["reached"] < 0.75:
        return False
    return _picked(objects, _owner())


def score(objects) -> float:
    """Partial credit at this step. 1.0 only once the completed pick has held for SUCCESS_HOLD_S: at the run limit, a
    pick completed less than 1 s earlier scores 0.75."""
    _update(objects)
    if _MEM["mispick"] or _MEM["fell"]:
        return 0.0
    if _MEM["reached"] >= 0.75 and _held_long_enough():
        return 1.0
    return _MEM["reached"]


def distance(objects) -> float:
    """Metres from the red item's centre to the order bin's centre, seen from above."""
    x, y, _ = (float(v) for v in objects["red_item"].pos)
    return math.hypot(x - BIN_XY[0], y - BIN_XY[1])


# -- the demonstration -----------------------------------------------------------------------------------------------
#
# Scripted, open loop, the left arm only, a straight-down grip turned -90 degrees so the fingers close along x, across
# the item's 50 mm width (closing along y would put a finger into the tote's right wall).

YAW = -math.pi / 2  # -90 rather than +90: the left wrist reaches the bin and the exit point only this way
GRIP_ABOVE_CENTRE = 0.01
PICK_Z = RED_START[2] + GRIP_ABOVE_CENTRE
CARRY_Z = TOTE_RIM + 0.045 + ITEM_SIZE[2] / 2 + GRIP_ABOVE_CENTRE  # over the tote: the item's bottom 4.5 cm above its rim
EXIT_Y = TOTE_XY[1] - TOTE_IN[1] / 2 - WALL - ITEM_SIZE[1] / 2 - 0.02  # the item's edge 2 cm clear of the tote's right wall
BIN_CARRY_Z = BIN_RIM + 0.03 + ITEM_SIZE[2] / 2 + GRIP_ABOVE_CENTRE  # over the bin: the item's bottom 3 cm above its rim
RELEASE_Z = BIN_RIM - 0.01 + ITEM_SIZE[2] / 2 + GRIP_ABOVE_CENTRE  # the item's bottom 1 cm below the bin rim
PARTLY_OPEN = FINGER_OPEN * 0.65  # pads about 61 mm apart: 5.5 mm clear of the 50 mm item on each side


def demo():
    left = [
        Waypoint(1.5, (RED_START[0], RED_START[1], CARRY_Z), PARTLY_OPEN, "over the red item", YAW),
        Waypoint(1.5, (RED_START[0], RED_START[1], PICK_Z), PARTLY_OPEN, "descend into the tote", YAW),
        Waypoint(0.8, (RED_START[0], RED_START[1], PICK_Z), FINGER_CLOSED, "grip", YAW),
        Waypoint(1.5, (RED_START[0], RED_START[1], CARRY_Z), FINGER_CLOSED, "lift out", YAW),
        Waypoint(1.2, (RED_START[0], EXIT_Y, CARRY_Z), FINGER_CLOSED, "clear of the tote", YAW),
        Waypoint(1.2, (BIN_XY[0], BIN_XY[1], BIN_CARRY_Z), FINGER_CLOSED, "over the order bin", YAW),
        Waypoint(1.2, (BIN_XY[0], BIN_XY[1], RELEASE_Z), FINGER_CLOSED, "lower into the bin", YAW),
        Waypoint(0.8, (BIN_XY[0], BIN_XY[1], RELEASE_Z), PARTLY_OPEN, "let go", YAW),
        Waypoint(1.0, (BIN_XY[0], BIN_XY[1], BIN_CARRY_Z), PARTLY_OPEN, "rise", YAW),
        Waypoint(1.2, (0.12, 0.10, BIN_CARRY_Z), FINGER_OPEN, "clear", YAW),
        Waypoint(2.5, (0.12, 0.10, BIN_CARRY_Z), FINGER_OPEN, "wait", YAW),
    ]
    return {"left": left}  # the right arm holds its ready pose
