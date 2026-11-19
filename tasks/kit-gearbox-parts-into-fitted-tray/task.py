"""On the station table: a kit tray with four fitted pockets in front of the gap between the arms; two look-alike square
shafts in front of the left arm, a gear and a bearing in front of the right arm. Nothing else is on the table."""

# An assembly-line kitter's core operation: picking a product's parts and placing each into its own fitted pocket of
# a kit tray, so the assembler finds a complete, correct kit at the station. A part in the wrong pocket is a kitting
# error: the assembler gets the wrong part, or none.
#
# Reliability stressor (one fixed level): look-alike parts. The two shafts have the same section and colour and differ
# only in length (120 vs 90 mm). The short shaft also fits the long shaft's pocket, so putting it there leaves the
# long shaft with nowhere to go.
#
# World frame: x forward (away from the arms), y left, z up, metres. The arm bases stand at x 0, y +-0.305.

import json
import math
import os
import sys

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Kit the gearbox parts into the kit tray, each part in its own fitted pocket."

# -- the scene -------------------------------------------------------------------------------------------------------

STEEL = (0.62, 0.64, 0.67)
DARK = (0.25, 0.26, 0.28)
SILVER = (0.80, 0.81, 0.83)
FOAM = (0.12, 0.30, 0.55)  # blue kit-tray foam
FLOOR = 0.01
WALL = 0.01
POCKET_DEPTH = 0.02
TRAY_FLOOR_TOP = TOP + FLOOR
TRAY_RIM = TRAY_FLOOR_TOP + POCKET_DEPTH

SHAFT_W = 0.025  # square section, 25 mm across
PARTS = {  # name: (kind, size, mass, colour)
    "long_shaft": ("box", (0.12, SHAFT_W, SHAFT_W), 0.20, STEEL),
    "short_shaft": ("box", (0.09, SHAFT_W, SHAFT_W), 0.15, STEEL),
    "gear": ("cyl", (0.03, 0.02), 0.07, DARK),  # radius, height
    "bearing": ("cyl", (0.0175, 0.012), 0.06, SILVER),
}


def half_height(name):
    kind, size, _, _ = PARTS[name]
    return size[2] / 2 if kind == "box" else size[1] / 2


# Fitted pockets in the kit tray, centred 27 cm forward: (centre x, centre y, inside x, inside y).
TRAY_X = 0.27
POCKETS = {
    "long_shaft": (TRAY_X, 0.14, 0.135, 0.04),  # 7.5 mm play at each end and each side
    "short_shaft": (TRAY_X, 0.06, 0.105, 0.04),  # the long shaft (120 mm) does not fit; the short one fits both
    "bearing": (TRAY_X, -0.02, 0.05, 0.05),
    "gear": (TRAY_X, -0.11, 0.075, 0.075),  # the gear (60 mm) does not fit the bearing pocket
}
PART_NAMES = list(PARTS)

# Parts on the table: shafts lying along x in front of the left arm, the gear and bearing in front of the right arm.
# Along y, the gripper's closing axis, neighbours are 9-10 cm apart so the housing (5.1 cm either side) clears them.
STARTS = {"long_shaft": (0.20, 0.30), "short_shaft": (0.20, 0.21), "gear": (0.20, -0.30), "bearing": (0.20, -0.20)}


def _part(name):
    kind, size, mass, col = PARTS[name]
    x, y = STARTS[name]
    z = TOP + half_height(name)
    if kind == "box":
        return Box(name, pos=(x, y, z), size=size, mass=mass, color=col)
    return Cylinder(name, pos=(x, y, z), radius=size[0], height=size[1], mass=mass, color=col)


def _tray():
    ys = [p[1] + s * p[3] / 2 for p in POCKETS.values() for s in (-1, 1)]
    xs = [p[0] + s * p[2] / 2 for p in POCKETS.values() for s in (-1, 1)]
    x0, x1 = min(xs) - WALL, max(xs) + WALL
    y0, y1 = min(ys) - WALL, max(ys) + WALL
    parts = [Box("tray_floor", pos=((x0 + x1) / 2, (y0 + y1) / 2, TOP + FLOOR / 2), size=(x1 - x0, y1 - y0, FLOOR),
                 color=FOAM)]
    zc = TRAY_FLOOR_TOP + POCKET_DEPTH / 2
    for name, (cx, cy, ix, iy) in POCKETS.items():
        ox, oy = ix + 2 * WALL, iy + 2 * WALL
        parts += [
            Box(f"pocket_{name}_near", pos=(cx - ix / 2 - WALL / 2, cy, zc), size=(WALL, oy, POCKET_DEPTH), color=FOAM),
            Box(f"pocket_{name}_far", pos=(cx + ix / 2 + WALL / 2, cy, zc), size=(WALL, oy, POCKET_DEPTH), color=FOAM),
            Box(f"pocket_{name}_left", pos=(cx, cy + iy / 2 + WALL / 2, zc), size=(ix, WALL, POCKET_DEPTH), color=FOAM),
            Box(f"pocket_{name}_right", pos=(cx, cy - iy / 2 - WALL / 2, zc), size=(ix, WALL, POCKET_DEPTH),
                color=FOAM),
        ]
    return parts


PART_OBJECTS = [_part(n) for n in PART_NAMES]
TRAY = _tray()
OBJECTS = [*PART_OBJECTS, *TRAY]

# -- scoring ---------------------------------------------------------------------------------------------------------
#
# A part is in a pocket when its centre lies inside the pocket's inside rectangle seen from above and is less than
# 1.5 cm above where it would rest on the tray floor. It is kitted when it is in its own pocket, still (its centre
# moved under 5 mm over the last 0.3 s), and no other part is in that pocket.
#   score = 0.2 x the number of kitted parts (0 to 0.8), at the end of the trial;
#   1.0   = success: all four parts kitted, none held (no gripper tool point within 4 cm of a kitted part's centre);
#           held for 1 s. Completed less than 1 s before the run limit scores 0.8.
# Defects that score 0: any part falling off the table.

SEATED = 0.015
STILL_S = 0.3
STILL_M = 0.005
RELEASED = 0.04
OFF_TABLE = 0.03
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
HZ_OFFLINE = 20.0


def _owner():
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


_MEM = {"key": None, "t": -1.0, "defect": False, "held_since": None, "kitted": [], "complete": False,
        "hist": {}, "offline_tcps": None, "offline_t": 0.0}


def new_trial(offline_tcps=None) -> None:
    _MEM.update(key=None, t=-1.0, defect=False, held_since=None, kitted=[], complete=False, hist={},
                offline_tcps=offline_tcps, offline_t=0.0)


def tick(dt: float = 1.0 / HZ_OFFLINE) -> None:
    """Offline only: advance the trial clock by one control step."""
    _MEM["offline_t"] += dt


def _now(owner) -> float:
    return float(owner._t) if owner is not None else float(_MEM["offline_t"])


def _tcps(owner) -> list:
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


def _pocket_of(name, obj):
    """The pocket the part is down in, or None."""
    x, y, z = (float(v) for v in obj.pos)
    if z - (TRAY_FLOOR_TOP + half_height(name)) >= SEATED:
        return None
    for pocket, (cx, cy, ix, iy) in POCKETS.items():
        if abs(x - cx) <= ix / 2 and abs(y - cy) <= iy / 2:
            return pocket
    return None


def _still(name, t) -> bool:
    h = _MEM["hist"].get(name, [])
    if not h or t - h[0][0] < STILL_S - 1e-9:
        return False
    ps = np.array([p for _, p in h])
    return float(np.max(np.linalg.norm(ps - ps[-1], axis=1))) < STILL_M


def _evaluate(objects, owner, t):
    where = {n: _pocket_of(n, objects[n]) for n in PART_NAMES}
    count = {}
    for p in where.values():
        if p is not None:
            count[p] = count.get(p, 0) + 1
    kitted = [n for n in PART_NAMES if where[n] == n and count[n] == 1 and _still(n, t)]
    tcps = _tcps(owner)
    free = all(np.linalg.norm(np.asarray(tc) - np.asarray(objects[n].pos, float)) > RELEASED
               for n in kitted for tc in tcps)
    return kitted, len(kitted) == 4 and free


def _update(objects) -> None:
    owner = _owner()
    key = id(owner) if owner is not None else "offline"
    t = _now(owner)
    if key != _MEM["key"] or (owner is not None and t < _MEM["t"]):
        keep, offline_t = (_MEM.get("offline_tcps") if owner is None else None), _MEM["offline_t"]
        new_trial(keep)
        _MEM["offline_t"] = offline_t
        _MEM["key"] = key
    if t == _MEM["t"]:  # success() and score() both call this each step: evaluate once per step
        return
    _MEM["t"] = t
    for name in PART_NAMES:
        h = _MEM["hist"].setdefault(name, [])
        h.append((t, np.asarray(objects[name].pos, float).copy()))
        while h and t - h[0][0] > STILL_S + 1e-9:
            h.pop(0)
        if float(objects[name].low[2]) < TOP - OFF_TABLE:
            _MEM["defect"] = True
    kitted, complete = _evaluate(objects, owner, t)
    _MEM["kitted"], _MEM["complete"] = kitted, complete
    if complete and not _MEM["defect"]:
        if _MEM["held_since"] is None:
            _MEM["held_since"] = t
    else:
        _MEM["held_since"] = None
    _trace(owner, objects)


def _trace(owner, objects) -> None:
    """With KIT_TRACE=<file> set, append one JSON line per control step (simulator only)."""
    path = os.environ.get("KIT_TRACE")
    if not path or owner is None:
        return
    rec = {"t": round(_now(owner), 3), "kitted": _MEM["kitted"], "complete": _MEM["complete"],
           "defect": _MEM["defect"], "tcps": [[round(float(v), 4) for v in tcp] for tcp in _tcps(owner)]}
    for name in PART_NAMES:
        rec[name] = [round(float(v), 4) for v in objects[name].pos]
    with open(path, "a") as f:
        f.write(json.dumps(rec) + "\n")


def success(objects) -> bool:
    _update(objects)
    return _MEM["complete"] and not _MEM["defect"]


def score(objects) -> float:
    """0.2 per kitted part; 1.0 only once the complete kit has held for SUCCESS_HOLD_S."""
    _update(objects)
    if _MEM["defect"]:
        return 0.0
    hs = _MEM["held_since"]
    if hs is not None and _MEM["t"] - hs >= SUCCESS_HOLD_S - 1e-9:
        return 1.0
    return round(0.2 * len(_MEM["kitted"]), 3)


def distance(objects) -> float:
    """Summed distance (m) from each part to its pocket, seen from above."""
    return sum(math.hypot(float(objects[n].pos[0]) - POCKETS[n][0], float(objects[n].pos[1]) - POCKETS[n][1])
               for n in PART_NAMES)


# -- the demonstration -----------------------------------------------------------------------------------------------
#
# Scripted, open loop, both arms. Straight-down grips with the pads closing along y (across each shaft's width, across
# the gear's and bearing's diameter), centred on each part; lift so the part clears the tray rim and the other parts
# by 3.5 cm; carry it over its pocket; lower until the finger tips are 6 mm above the pocket walls and let go. The left
# arm kits the long shaft, then the short one; the right arm the gear, then (after the left arm is clear of the
# centre) the bearing.

TIP_PAST_TCP = 0.013
TIP_ABOVE_TABLE = 0.003


def grip_z(name):
    """Tool point height for the grip: at the part's mid-height, but finger tips no lower than 3 mm above the table."""
    return max(TOP + half_height(name), TOP + TIP_ABOVE_TABLE + TIP_PAST_TCP)


TALLEST = SHAFT_W
CARRY_Z = TRAY_RIM + 0.035 + TALLEST + 0.02  # tool point: a carried part's bottom clears the rim by about 3.5 cm
RELEASE_Z = TRAY_RIM + 0.006 + TIP_PAST_TCP
OPEN = {"long_shaft": FINGER_OPEN * 0.6, "short_shaft": FINGER_OPEN * 0.6, "gear": FINGER_OPEN * 0.95,
        "bearing": FINGER_OPEN * 0.7}
ORDER = {"left": ["long_shaft", "short_shaft"], "right": ["gear", "bearing"]}
RIGHT_WAIT_S = 4.0  # before the bearing trip, so the two grippers are never both near the centre line


def _arm(side):
    out = []
    for k, name in enumerate(ORDER[side]):
        px, py = STARTS[name]
        cx, cy = POCKETS[name][:2]
        gz, op = grip_z(name), OPEN[name]
        if side == "right" and k == 1:
            out.append(Waypoint(RIGHT_WAIT_S, (0.12, -0.30, CARRY_Z), FINGER_OPEN, "wait for the left arm"))
        out += [
            Waypoint(1.4, (px, py, CARRY_Z), op, f"over the {name}"),
            Waypoint(1.0, (px, py, gz), op, "descend"),
            Waypoint(0.7, (px, py, gz), FINGER_CLOSED, "grip"),
            Waypoint(1.0, (px, py, CARRY_Z), FINGER_CLOSED, "lift"),
            Waypoint(1.6, (cx, cy, CARRY_Z), FINGER_CLOSED, "over its pocket"),
            Waypoint(0.8, (cx, cy, RELEASE_Z), FINGER_CLOSED, "lower to the pocket walls"),
            Waypoint(0.6, (cx, cy, RELEASE_Z), op, "let go"),
            Waypoint(0.7, (cx, cy, CARRY_Z), op, "rise"),
        ]
    py = 0.30 if side == "left" else -0.30
    out += [Waypoint(1.2, (0.12, py, CARRY_Z), FINGER_OPEN, "clear"),
            Waypoint(3.0, (0.12, py, CARRY_Z), FINGER_OPEN, "wait")]
    return out


def demo():
    return {"left": _arm("left"), "right": _arm("right")}
