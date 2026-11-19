"""On the station table: a six-cell transplant strip running across the table between the arms, two of its cells
already planted; in front of each arm, a seedling flat holding two plugs. Nothing else is on the table."""

# A greenhouse grower's core operation: transplanting plugs, lifting each seedling plug out of the propagation flat
# and setting it into an empty cell of a cell strip, one plug per cell. Planting a second plug into an occupied cell
# wastes both seedlings, and knocking out a plant that is already in wastes it.
#
# Reliability stressor (one fixed level): partial state. The strip is part-planted (cells 2 and 5 of six already hold
# plugs), so the policy must find the four empty cells rather than fill cells in a fixed pattern. Each arm can reach
# only its own half of the strip, so both arms work.
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

INSTRUCTION = "Transplant the seedling plugs into the empty cells of the strip, one plug per cell."

# -- the scene -------------------------------------------------------------------------------------------------------

PLUG_R = 0.018  # 36 mm round plugs (root ball and growing medium), 55 mm tall, 40 g
PLUG_H = 0.055
PLUG_MASS = 0.04
SOIL = (0.30, 0.20, 0.12)
BLACK = (0.10, 0.10, 0.10)  # black plastic strip
FLAT = (0.20, 0.35, 0.22)  # green plastic seedling flat
FLOOR = 0.01

# The cell strip: black plastic, six 52 mm square cells in one row across the table, 3 cm deep, 6 mm walls.
STRIP_X = 0.27
CELL_IN = 0.052
CELL_WALL = 0.006
PITCH = CELL_IN + CELL_WALL
CELL_DEPTH = 0.03
STRIP_FLOOR_TOP = TOP + FLOOR
STRIP_RIM = STRIP_FLOOR_TOP + CELL_DEPTH
CELLS = [(STRIP_X, (2.5 - i) * PITCH) for i in range(6)]  # cell 1 at the left end ... cell 6 at the right end
PLANTED_CELLS = (1, 4)  # cells 2 and 5 (0-based 1 and 4) already hold plugs
EMPTY_CELLS = tuple(i for i in range(6) if i not in PLANTED_CELLS)

# The seedling flats: green plastic, inside 24 cm (forward) x 9 cm, a 1.5 cm lip, one in front of each arm.
FLAT_X = 0.20
FLAT_IN = (0.24, 0.09)
FLAT_LIP = 0.015
FLAT_WALL = 0.006
FLAT_FLOOR_TOP = TOP + FLOOR
FLAT_Y = {"left": 0.30, "right": -0.30}
# Plugs 10 cm apart along x, the gripper's closing axis: the housing (5.1 cm either side along that axis) clears the
# neighbouring plug.
PLUG_STARTS = {"left": [(0.15, 0.30), (0.25, 0.30)], "right": [(0.15, -0.30), (0.25, -0.30)]}
FLAT_PLUG_Z = FLAT_FLOOR_TOP + PLUG_H / 2
CELL_PLUG_Z = STRIP_FLOOR_TOP + PLUG_H / 2

NEW = [Cylinder(f"plug_{i + 1}", pos=(x, y, FLAT_PLUG_Z), radius=PLUG_R, height=PLUG_H, mass=PLUG_MASS, color=SOIL)
       for i, (x, y) in enumerate(PLUG_STARTS["left"] + PLUG_STARTS["right"])]
OLD = [Cylinder(f"planted_{c + 1}", pos=(*CELLS[c], CELL_PLUG_Z), radius=PLUG_R, height=PLUG_H, mass=PLUG_MASS,
                color=SOIL) for c in PLANTED_CELLS]
NEW_NAMES = [p.name for p in NEW]
OLD_NAMES = [p.name for p in OLD]
ALL_NAMES = NEW_NAMES + OLD_NAMES


def _strip():
    n = len(CELLS)
    length = n * PITCH + CELL_WALL  # along y
    width = CELL_IN + 2 * CELL_WALL  # along x
    zc = STRIP_FLOOR_TOP + CELL_DEPTH / 2
    parts = [Box("strip_floor", pos=(STRIP_X, 0.0, TOP + FLOOR / 2), size=(width, length, FLOOR), color=BLACK)]
    for sx, name in ((-1, "near"), (1, "far")):
        parts.append(Box(f"strip_wall_{name}", pos=(STRIP_X + sx * (CELL_IN + CELL_WALL) / 2, 0.0, zc),
                         size=(CELL_WALL, length, CELL_DEPTH), color=BLACK))
    for k in range(n + 1):  # end walls and the five dividers
        y = (n / 2 - k) * PITCH
        parts.append(Box(f"strip_divider_{k}", pos=(STRIP_X, y, zc), size=(CELL_IN, CELL_WALL, CELL_DEPTH),
                         color=BLACK))
    return parts


def _flat(side):
    cy = FLAT_Y[side]
    ix, iy = FLAT_IN
    w = FLAT_WALL
    ox, oy = ix + 2 * w, iy + 2 * w
    zc = TOP + FLOOR + FLAT_LIP / 2
    return [
        Box(f"flat_{side}_floor", pos=(FLAT_X, cy, TOP + FLOOR / 2), size=(ox, oy, FLOOR), color=FLAT),
        Box(f"flat_{side}_lip_near", pos=(FLAT_X - ix / 2 - w / 2, cy, zc), size=(w, oy, FLAT_LIP), color=FLAT),
        Box(f"flat_{side}_lip_far", pos=(FLAT_X + ix / 2 + w / 2, cy, zc), size=(w, oy, FLAT_LIP), color=FLAT),
        Box(f"flat_{side}_lip_left", pos=(FLAT_X, cy + iy / 2 + w / 2, zc), size=(ix, w, FLAT_LIP), color=FLAT),
        Box(f"flat_{side}_lip_right", pos=(FLAT_X, cy - iy / 2 - w / 2, zc), size=(ix, w, FLAT_LIP), color=FLAT),
    ]


STRIP = _strip()
FLATS = _flat("left") + _flat("right")
OBJECTS = [*NEW, *OLD, *STRIP, *FLATS]

# -- scoring ---------------------------------------------------------------------------------------------------------
#
# A plug is in a cell when its centre lies inside that cell's 58 mm pitch square seen from above. A new plug is
# planted when it is in one of the four empty cells, seated on the cell floor (its centre less than 2 cm above a
# standing plug's; a plug leaning on a cell wall counts), still (its centre moved under 5 mm over the last 0.3 s),
# and no other plug is down in that cell (centre less than
# 3 cm above a standing plug's; a plug carried over the cell does not count).
#   score = 0.2 x the number of planted new plugs (0 to 0.8), at the end of the trial;
#   1.0   = success: all four new plugs planted, one per empty cell, none held (no gripper tool point within
#           PLUG_H/2 + 2 cm of a planted plug's centre), and both original plants still in their cells; held for 1 s.
#           Completed less than 1 s before the run limit scores 0.8.
# Defects that score 0: an original plant knocked out (its centre leaves its cell or rises 3 cm), or any plug falling
# off the table.

STILL_S = 0.3
STILL_M = 0.005
SEATED = 0.02  # a plug leaning on a cell wall sits about 1 cm higher than a standing one (simulation)
UPROOTED = 0.03
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


_MEM = {"key": None, "t": -1.0, "defect": False, "held_since": None, "planted": 0, "complete": False,
        "hist": {}, "offline_tcps": None, "offline_t": 0.0}


def new_trial(offline_tcps=None) -> None:
    _MEM.update(key=None, t=-1.0, defect=False, held_since=None, planted=0, complete=False, hist={},
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
    """Index of the cell whose pitch square holds the object's centre, or None."""
    x, y, _ = (float(v) for v in obj.pos)
    for i, (cx, cy) in enumerate(CELLS):
        if abs(x - cx) <= PITCH / 2 and abs(y - cy) <= PITCH / 2:
            return i
    return None


def _still(name, t) -> bool:
    """Moved less than STILL_MM over the last STILL_S (contact jitter of a leaning cylinder does not count)."""
    h = _MEM["hist"].get(name, [])
    if not h or t - h[0][0] < STILL_S - 1e-9:
        return False
    ps = np.array([p for _, p in h])
    return float(np.max(np.linalg.norm(ps - ps[-1], axis=1))) < STILL_M


def _seated(obj) -> bool:
    return float(obj.pos[2]) - CELL_PLUG_Z < SEATED


def _evaluate(objects, owner, t):
    """(planted new plugs, whether the transplanting is complete, whether an original plant is knocked out)."""
    cells = {}
    for name in ALL_NAMES:  # occupants: plugs down in a cell, not plugs carried over it
        c = _cell_of(objects[name])
        if c is not None and float(objects[name].pos[2]) - CELL_PLUG_Z < UPROOTED:
            cells.setdefault(c, []).append(name)
    planted = [n for n in NEW_NAMES
               if (c := _cell_of(objects[n])) in EMPTY_CELLS and len(cells.get(c, [])) == 1 and _seated(objects[n])
               and _still(n, t)]
    knocked = any(_cell_of(objects[n]) != c or float(objects[n].pos[2]) - CELL_PLUG_Z > UPROOTED
                  for n, c in zip(OLD_NAMES, PLANTED_CELLS))
    tcps = _tcps(owner)
    free = all(np.linalg.norm(np.asarray(t) - np.asarray(objects[n].pos, float)) > PLUG_H / 2 + RELEASED_MARGIN
               for n in planted for t in tcps)
    return len(planted), len(planted) == 4 and free and not knocked, knocked


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
    for name in ALL_NAMES:
        h = _MEM["hist"].setdefault(name, [])
        h.append((t, np.asarray(objects[name].pos, float).copy()))
        while h and t - h[0][0] > STILL_S + 1e-9:
            h.pop(0)
    planted, complete, knocked = _evaluate(objects, owner, t)
    if knocked or any(float(objects[n].low[2]) < TOP - OFF_TABLE for n in ALL_NAMES):
        _MEM["defect"] = True
    _MEM["planted"], _MEM["complete"] = planted, complete
    if complete and not _MEM["defect"]:
        if _MEM["held_since"] is None:
            _MEM["held_since"] = t
    else:
        _MEM["held_since"] = None
    _trace(owner, objects)


def _trace(owner, objects) -> None:
    """With PLUG_TRACE=<file> set, append one JSON line per control step (simulator only)."""
    path = os.environ.get("PLUG_TRACE")
    if not path or owner is None:
        return
    rec = {"t": round(_now(owner), 3), "planted": _MEM["planted"], "complete": _MEM["complete"],
           "defect": _MEM["defect"], "tcps": [[round(float(v), 4) for v in tcp] for tcp in _tcps(owner)]}
    for name in ALL_NAMES:
        rec[name] = [round(float(v), 4) for v in objects[name].pos]
    with open(path, "a") as f:
        f.write(json.dumps(rec) + "\n")


def success(objects) -> bool:
    _update(objects)
    return _MEM["complete"] and not _MEM["defect"]


def score(objects) -> float:
    """0.2 per planted plug; 1.0 only once the complete transplanting has held for SUCCESS_HOLD_S."""
    _update(objects)
    if _MEM["defect"]:
        return 0.0
    hs = _MEM["held_since"]
    if hs is not None and _MEM["t"] - hs >= SUCCESS_HOLD_S - 1e-9:
        return 1.0
    return round(0.2 * _MEM["planted"], 3)


def distance(objects) -> float:
    """Summed distance (m) from each new plug to the cell the demo plants it in, seen from above."""
    return sum(math.hypot(float(objects[n].pos[0]) - CELLS[c][0], float(objects[n].pos[1]) - CELLS[c][1])
               for n, c in zip(NEW_NAMES, DEMO_CELLS))


# -- the demonstration -----------------------------------------------------------------------------------------------
#
# Scripted, open loop, both arms at once, each on its own half of the strip. For each plug: a straight-down grip on its
# upper part with the pads closing along x (so the housing never reaches across the row of cells), lift so the plug's
# bottom clears the tops of the planted plugs by 3.5 cm, carry it over its cell, lower until the finger tips are 6 mm
# above the cell walls and let go: the plug drops about 1 cm onto the cell floor.

TIP_PAST_TCP = 0.013
GRIP_BELOW_TOP = 0.015
GRIP_Z = FLAT_FLOOR_TOP + PLUG_H - GRIP_BELOW_TOP
PLUG_TOP = STRIP_FLOOR_TOP + PLUG_H  # a planted plug's top
CARRY_Z = PLUG_TOP + 0.035 + PLUG_H - GRIP_BELOW_TOP  # tool point when the carried plug's bottom clears by 3.5 cm
RELEASE_Z = STRIP_RIM + 0.006 + TIP_PAST_TCP
OPEN = FINGER_OPEN * 0.75  # pads about 70 mm apart around the 36 mm plug
YAW = {"left": -math.pi / 2, "right": math.pi / 2}  # pads close along x
# Plug -> cell, in order. The left arm fills the inner cell first while the right arm fills the outer one, then they
# swap, so the two grippers are never both near the centre line.
DEMO_PLAN = {"left": [(0, 2), (1, 0)], "right": [(2, 5), (3, 3)]}  # (index into NEW, cell index)
DEMO_CELLS = [c for _, c in sorted(DEMO_PLAN["left"] + DEMO_PLAN["right"])]


def _arm(side):
    yaw = YAW[side]
    out = []
    for plug, cell in DEMO_PLAN[side]:
        px, py = PLUG_STARTS["left" if plug < 2 else "right"][plug % 2]
        cx, cy = CELLS[cell]
        out += [
            Waypoint(1.4, (px, py, CARRY_Z), OPEN, "over a plug", yaw=yaw),
            Waypoint(1.0, (px, py, GRIP_Z), OPEN, "descend", yaw=yaw),
            Waypoint(0.7, (px, py, GRIP_Z), FINGER_CLOSED, "grip", yaw=yaw),
            Waypoint(1.0, (px, py, CARRY_Z), FINGER_CLOSED, "lift", yaw=yaw),
            Waypoint(1.6, (cx, cy, CARRY_Z), FINGER_CLOSED, "over its cell", yaw=yaw),
            Waypoint(0.8, (cx, cy, RELEASE_Z), FINGER_CLOSED, "lower to the cell walls", yaw=yaw),
            Waypoint(0.6, (cx, cy, RELEASE_Z), OPEN, "let go", yaw=yaw),
            Waypoint(0.7, (cx, cy, CARRY_Z), OPEN, "rise", yaw=yaw),
        ]
    py = FLAT_Y[side]
    out += [Waypoint(1.2, (0.12, py, CARRY_Z), FINGER_OPEN, "clear", yaw=yaw),
            Waypoint(3.0, (0.12, py, CARRY_Z), FINGER_OPEN, "wait", yaw=yaw)]
    return out


def demo():
    return {"left": _arm("left"), "right": _arm("right")}
