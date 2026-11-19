"""On the station table, in front of the left arm: a shallow wooden field crate holding five apples in two rows, four
red and one bruised (brown). To its right, in front of the gap between the arms: an empty 2 x 2 pulp pocket tray.
Nothing else is on the table."""

# A packhouse packer's core operation: grade fruit by eye and pack the sound fruit into a pocket tray, one per pocket,
# leaving culls (bruised fruit) out of the pack. A bruised apple in a retail pack is a quality defect the packing line
# exists to prevent.
#
# Reliability stressor (one fixed level): a look-alike decoy. The bruised apple has the same size and shape as the good
# ones and differs only in colour.
#
# World frame: x forward (away from the arms), y left, z up, metres. The arm bases stand at x 0, y +-0.305.

import json
import math
import os
import sys

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Sphere

SCENE = "station"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Pack the good apples into the pocket tray, one per pocket."

# -- the scene -------------------------------------------------------------------------------------------------------

APPLE_D = 0.07  # 70 mm apples, 180 g each
APPLE_R = APPLE_D / 2
APPLE_MASS = 0.18
RED = (0.72, 0.08, 0.08)
BRUISED = (0.42, 0.24, 0.10)  # dull brown-red: bruised all over its upper side
WOOD = (0.70, 0.55, 0.35)
PULP = (0.80, 0.78, 0.72)
WALL = 0.01
FLOOR = 0.01

# The field crate: wood, inside 32 cm (forward) x 22 cm (across), lip 2 cm above a 1 cm floor.
CRATE_XY = (0.25, 0.23)
CRATE_IN = (0.32, 0.22)
CRATE_LIP = 0.02
CRATE_FLOOR_TOP = TOP + FLOOR

# The pocket tray: moulded pulp modelled as a 1 cm floor with 3 cm dividers forming a 2 x 2 grid of 9 cm pockets.
TRAY_XY = (0.25, 0.0)
CELL = 0.09
TRAY_WALL_H = 0.03
TRAY_FLOOR_TOP = TOP + FLOOR
TRAY_RIM = TRAY_FLOOR_TOP + TRAY_WALL_H
CELLS = [(TRAY_XY[0] + sx * (CELL + WALL) / 2, TRAY_XY[1] + sy * (CELL + WALL) / 2)
         for sx in (-1, 1) for sy in (1, -1)]  # near-left, near-right, far-left, far-right

# Apples on the crate floor, 10 cm apart in two rows along x (the gripper's housing, 3.4 cm across its closing axis,
# clears each neighbour). The bruised one sits in the middle of the near row; the far row's right-hand spot is empty.
APPLE_Z = CRATE_FLOOR_TOP + APPLE_R
GOOD_STARTS = [(0.15, 0.18), (0.15, 0.28), (0.25, 0.28), (0.35, 0.28)]
BRUISED_START = (0.25, 0.18)
GOOD = [Sphere(f"apple_{i + 1}", pos=(x, y, APPLE_Z), radius=APPLE_R, mass=APPLE_MASS, color=RED)
        for i, (x, y) in enumerate(GOOD_STARTS)]
BRUISED_APPLE = Sphere("bruised_apple", pos=(*BRUISED_START, APPLE_Z), radius=APPLE_R, mass=APPLE_MASS, color=BRUISED)


def _crate():
    cx, cy = CRATE_XY
    ix, iy = CRATE_IN
    ox, oy = ix + 2 * WALL, iy + 2 * WALL
    zc = TOP + FLOOR + CRATE_LIP / 2
    return [
        Box("crate_floor", pos=(cx, cy, TOP + FLOOR / 2), size=(ox, oy, FLOOR), color=WOOD),
        Box("crate_lip_near", pos=(cx - ix / 2 - WALL / 2, cy, zc), size=(WALL, oy, CRATE_LIP), color=WOOD),
        Box("crate_lip_far", pos=(cx + ix / 2 + WALL / 2, cy, zc), size=(WALL, oy, CRATE_LIP), color=WOOD),
        Box("crate_lip_left", pos=(cx, cy + iy / 2 + WALL / 2, zc), size=(ix, WALL, CRATE_LIP), color=WOOD),
        Box("crate_lip_right", pos=(cx, cy - iy / 2 - WALL / 2, zc), size=(ix, WALL, CRATE_LIP), color=WOOD),
    ]


def _tray():
    cx, cy = TRAY_XY
    inner = 2 * CELL + WALL  # two pockets and the divider between them
    outer = inner + 2 * WALL
    zc = TOP + FLOOR + TRAY_WALL_H / 2
    parts = [Box("tray_floor", pos=(cx, cy, TOP + FLOOR / 2), size=(outer, outer, FLOOR), color=PULP)]
    for sx, name in ((-1, "near"), (1, "far")):
        parts.append(Box(f"tray_wall_{name}", pos=(cx + sx * (inner / 2 + WALL / 2), cy, zc),
                         size=(WALL, outer, TRAY_WALL_H), color=PULP))
    for sy, name in ((1, "left"), (-1, "right")):
        parts.append(Box(f"tray_wall_{name}", pos=(cx, cy + sy * (inner / 2 + WALL / 2), zc),
                         size=(inner, WALL, TRAY_WALL_H), color=PULP))
    parts.append(Box("tray_divider_x", pos=(cx, cy, zc), size=(WALL, inner, TRAY_WALL_H), color=PULP))
    parts.append(Box("tray_divider_y", pos=(cx, cy, zc), size=(inner, WALL, TRAY_WALL_H), color=PULP))
    return parts


CRATE = _crate()
TRAY = _tray()
OBJECTS = [*GOOD, BRUISED_APPLE, *CRATE, *TRAY]
GOOD_NAMES = [a.name for a in GOOD]

# -- scoring ---------------------------------------------------------------------------------------------------------
#
# A good apple is packed when its centre lies inside a pocket's 9 cm square (seen from above), it rests on the tray
# floor (its centre less than 5 cm above it), it is still (under 1 cm/s and 0.5 rad/s), and no other apple shares
# that pocket.
#   score = 0.2 x the number of packed good apples (0 to 0.8), at the end of the trial;
#   1.0   = success: all four good apples packed, one per pocket, none held (no gripper tool point within 2 cm of
#           any of them), and the bruised apple not in the tray; held for 1 s. A pack completed less than 1 s before
#           the run limit scores 0.8.
# Defects that score 0: the bruised apple anywhere over the tray (packed as a cull), or any apple falling off the table.

PACKED_HEIGHT = 0.05
RELEASED_MARGIN = 0.02
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


_MEM = {"key": None, "t": -1.0, "defect": False, "held_since": None, "packed": 0, "complete": False,
        "offline_tcps": None, "offline_t": 0.0}


def new_trial(offline_tcps=None) -> None:
    _MEM.update(key=None, t=-1.0, defect=False, held_since=None, packed=0, complete=False,
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


def _cell_of(obj):
    """Index of the pocket whose square holds the object's centre, or None."""
    x, y, _ = (float(v) for v in obj.pos)
    for i, (cx, cy) in enumerate(CELLS):
        if abs(x - cx) <= CELL / 2 and abs(y - cy) <= CELL / 2:
            return i
    return None


def _over_tray(obj) -> bool:
    x, y, _ = (float(v) for v in obj.pos)
    half = CELL + WALL * 1.5
    return abs(x - TRAY_XY[0]) <= half and abs(y - TRAY_XY[1]) <= half


def _evaluate(objects, owner):
    """(packed good apples, whether the pack is complete)."""
    cells = {}
    for name in GOOD_NAMES + ["bruised_apple"]:
        c = _cell_of(objects[name])
        if c is not None:
            cells.setdefault(c, []).append(name)
    packed = []
    for name in GOOD_NAMES:
        a = objects[name]
        c = _cell_of(a)
        if (c is not None and len(cells[c]) == 1 and float(a.pos[2]) - TRAY_FLOOR_TOP < PACKED_HEIGHT
                and a.resting(0.01, 0.5)):
            packed.append(name)
    tcps = _tcps(owner)
    free = all(np.linalg.norm(np.asarray(t) - np.asarray(objects[n].pos, float)) > APPLE_R + RELEASED_MARGIN
               for n in packed for t in tcps)
    complete = len(packed) == 4 and free and not _over_tray(objects["bruised_apple"])
    return len(packed), complete


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
    if _over_tray(objects["bruised_apple"]):
        _MEM["defect"] = True
    for name in GOOD_NAMES + ["bruised_apple"]:
        if float(objects[name].low[2]) < TOP - OFF_TABLE:
            _MEM["defect"] = True
    packed, complete = _evaluate(objects, owner)
    _MEM["packed"], _MEM["complete"] = packed, complete
    if complete and not _MEM["defect"]:
        if _MEM["held_since"] is None:
            _MEM["held_since"] = t
    else:
        _MEM["held_since"] = None
    _trace(owner, objects)


def _trace(owner, objects) -> None:
    """With PACK_TRACE=<file> set, append one JSON line per control step (simulator only)."""
    path = os.environ.get("PACK_TRACE")
    if not path or owner is None:
        return
    rec = {"t": round(_now(owner), 3), "packed": _MEM["packed"], "complete": _MEM["complete"],
           "defect": _MEM["defect"], "tcps": [[round(float(v), 4) for v in tcp] for tcp in _tcps(owner)]}
    for name in GOOD_NAMES + ["bruised_apple"]:
        rec[name] = [round(float(v), 4) for v in objects[name].pos]
    with open(path, "a") as f:
        f.write(json.dumps(rec) + "\n")


def success(objects) -> bool:
    _update(objects)
    return _MEM["complete"] and not _MEM["defect"]


def score(objects) -> float:
    """0.2 per packed good apple; 1.0 only once the complete pack has held for SUCCESS_HOLD_S."""
    _update(objects)
    if _MEM["defect"]:
        return 0.0
    hs = _MEM["held_since"]
    if hs is not None and _MEM["t"] - hs >= SUCCESS_HOLD_S - 1e-9:
        return 1.0
    return round(0.2 * _MEM["packed"], 3)


def distance(objects) -> float:
    """Summed distance (m) from each good apple to the pocket the demo fills with it, seen from above."""
    return sum(math.hypot(float(objects[n].pos[0]) - cx, float(objects[n].pos[1]) - cy)
               for n, (cx, cy) in zip(GOOD_NAMES, DEMO_CELLS))


# -- the demonstration -----------------------------------------------------------------------------------------------
#
# Scripted, open loop, the left arm only: four picks, a straight-down grip closing across each apple's equator, each
# apple carried over its pocket and let go from just above the dividers so it settles onto the pocket floor.

TIP_PAST_TCP = 0.013
GRIP_Z = APPLE_Z  # tool point at the apple's centre: the pads close on its equator
# Carried apples pass over other apples (in the crate and already packed), so the carried apple's bottom clears the
# other apples' tops by 3.5 cm. In simulation a carry that only cleared the dividers knocked a packed apple off the
# table (PACK_TRACE).
APPLE_TOP = TRAY_FLOOR_TOP + APPLE_D
CARRY_Z = APPLE_TOP + 0.035 + APPLE_R  # tool point (= the carried apple's centre)
RELEASE_Z = TRAY_RIM + 0.006 + TIP_PAST_TCP  # finger tips 6 mm above the dividers; the apple drops about 1.5 cm
OPEN = FINGER_OPEN * 0.9  # pads about 85 mm apart around the 70 mm apple
# Pockets in fill order: far-right first, because the arm reaches it only up to about 12 cm above the table, so its
# trip is carried lower (CARRY_LOW_Z) while nothing is packed yet and the path passes over no other apple.
DEMO_CELLS = [CELLS[3], CELLS[2], CELLS[0], CELLS[1]]  # far-right, far-left, near-left, near-right
CARRY_LOW_Z = TOP + 0.12


def demo():
    left = []
    for i, ((ax, ay), (cx, cy)) in enumerate(zip(GOOD_STARTS, DEMO_CELLS)):
        carry = CARRY_LOW_Z if i == 0 else CARRY_Z
        left += [
            Waypoint(1.4, (ax, ay, CARRY_Z), OPEN, "over an apple"),
            Waypoint(1.0, (ax, ay, GRIP_Z), OPEN, "descend"),
            Waypoint(0.7, (ax, ay, GRIP_Z), FINGER_CLOSED, "grip"),
            Waypoint(1.0, (ax, ay, carry), FINGER_CLOSED, "lift"),
            Waypoint(1.6, (cx, cy, carry), FINGER_CLOSED, "over its pocket"),
            Waypoint(0.8, (cx, cy, RELEASE_Z), FINGER_CLOSED, "lower to the dividers"),
            Waypoint(0.6, (cx, cy, RELEASE_Z), OPEN, "let go"),
            Waypoint(0.7, (cx, cy, carry), OPEN, "rise"),
        ]
    left += [Waypoint(1.2, (0.12, 0.10, CARRY_Z), FINGER_OPEN, "clear"),
             Waypoint(2.5, (0.12, 0.10, CARRY_Z), FINGER_OPEN, "wait")]
    return {"left": left}  # the right arm holds its ready pose
