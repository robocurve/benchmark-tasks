"""On the station table: a two-row stepped bin rack between the arms (row A in front at table level, row B behind on a
step), three bays wide, rows marked with coloured tape and bays with tape dots; two bins already hold totes. An
orange tote waits on the receiving spot in front of the left arm. Nothing else is on the table."""

# A warehouse put-away associate's core operation: carrying received stock to the storage location its label calls
# for and setting it into that location, read from the rack's own markings (row and bay). Stock put away in the wrong
# location is lost to the system until found: the error put-away exists to prevent.
#
# Reliability stressor (one fixed level): a hidden rule. The location code B2 must be read from the rack: row B is the
# row marked with blue tape (row A has red), and bay 2 is the bay with two tape dots in front of it. Neighbouring
# locations are empty and look the same.
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

INSTRUCTION = "Put the tote away in location B2 of the bin rack."

# -- the scene -------------------------------------------------------------------------------------------------------

GREY = (0.55, 0.57, 0.60)  # steel rack
BIN = (0.18, 0.32, 0.60)  # blue plastic bins
RED_TAPE = (0.85, 0.12, 0.10)
BLUE_TAPE = (0.10, 0.35, 0.95)
YELLOW = (0.95, 0.80, 0.10)  # bay dots
ORANGE = (0.95, 0.50, 0.10)
STORED = (0.45, 0.45, 0.42)

TOTE = (0.06, 0.055, 0.05)  # small parts tote, 60 (forward) x 55 x 50 mm tall, 250 g
TOTE_MASS = 0.25

FLOOR = 0.01
BIN_IN = (0.09, 0.085)  # inside, forward x across
BIN_WALL = 0.006
BIN_H = 0.03  # wall height above the bin floor
PITCH_Y = BIN_IN[1] + BIN_WALL
BAYS_Y = [PITCH_Y, 0.0, -PITCH_Y]  # bay 1 left, bay 2 centre, bay 3 right
STEP_H = 0.04  # row B stands on a 4 cm step
ROWS = {"A": (0.22, TOP), "B": (0.22 + BIN_IN[0] + BIN_WALL, TOP + STEP_H)}  # row: (centre x, base height)


def floor_top(row):
    return ROWS[row][1] + FLOOR


def rim(row):
    return floor_top(row) + BIN_H


LOCATIONS = {f"{r}{b + 1}": (ROWS[r][0], BAYS_Y[b], r) for r in ROWS for b in range(3)}
TARGET = "B2"
STORED_IN = ("A1", "B3")  # already occupied

TOTE_START = (0.14, 0.27)
TOTE_OBJ = Box("tote", pos=(*TOTE_START, TOP + TOTE[2] / 2), size=TOTE, mass=TOTE_MASS, color=ORANGE)
STORED_TOTES = [Box(f"stored_{loc}", pos=(LOCATIONS[loc][0], LOCATIONS[loc][1], floor_top(LOCATIONS[loc][2]) + TOTE[2] / 2),
                    size=TOTE, mass=TOTE_MASS, color=STORED) for loc in STORED_IN]


def _rack():
    parts = []
    width = 3 * PITCH_Y + BIN_WALL
    depth = BIN_IN[0] + 2 * BIN_WALL
    xb = ROWS["B"][0]
    parts.append(Box("rack_step", pos=(xb, 0.0, TOP + STEP_H / 2), size=(depth, width, STEP_H), color=GREY))
    for row, (cx, base) in ROWS.items():
        zf = base + FLOOR / 2
        zc = base + FLOOR + BIN_H / 2
        parts.append(Box(f"row_{row}_floor", pos=(cx, 0.0, zf), size=(depth, width, FLOOR), color=BIN))
        for sx, name in ((-1, "front"), (1, "back")):
            parts.append(Box(f"row_{row}_wall_{name}", pos=(cx + sx * (BIN_IN[0] + BIN_WALL) / 2, 0.0, zc),
                             size=(BIN_WALL, width, BIN_H), color=BIN))
        for k in range(4):
            y = (1.5 - k) * PITCH_Y
            parts.append(Box(f"row_{row}_divider_{k}", pos=(cx, y, zc), size=(BIN_IN[0], BIN_WALL, BIN_H), color=BIN))
        # row tape along the top of the front wall
        tape = RED_TAPE if row == "A" else BLUE_TAPE
        parts.append(Box(f"row_{row}_tape", pos=(cx - (BIN_IN[0] + BIN_WALL) / 2, 0.0, base + FLOOR + BIN_H + 0.001),
                         size=(BIN_WALL, width, 0.002), color=tape))
    # bay dots on the table in front of row A: one, two and three dots
    x0 = ROWS["A"][0] - depth / 2 - 0.015
    for b, y in enumerate(BAYS_Y):
        n = b + 1
        for k in range(n):
            dy = (k - (n - 1) / 2) * 0.016
            parts.append(Box(f"bay_{n}_dot_{k}", pos=(x0, y + dy, TOP + 0.001), size=(0.01, 0.01, 0.002), color=YELLOW))
    return parts


RACK = _rack()
OBJECTS = [TOTE_OBJ, *STORED_TOTES, *RACK]
ALL_TOTES = ["tote"] + [t.name for t in STORED_TOTES]

# -- scoring ---------------------------------------------------------------------------------------------------------
#
# The tote is in a location when its centre lies inside that bin's inside rectangle seen from above and is less than
# 1.5 cm above where it would rest on the bin floor. Still means its centre moved less than 5 mm over the last 0.3 s.
#   score = 0.25 [tote lifted 3 cm off the table at any time] + 0.25 [tote over B2: centre inside B2's rectangle seen
#           from above, at any height] + 0.25 [tote in B2, still]                        (0 to 0.75), at the end;
#   1.0   = success: tote in B2, still, no gripper tool point within 4 cm of its centre, the stored totes still in
#           A1 and B3; held for 1 s. Completed less than 1 s before the run limit: 0.75.
# Defects that score 0: the tote ever resting in a wrong location (in a location other than B2 and still), a stored
# tote knocked out of its location, or any tote falling off the table.

LIFTED = 0.03
IN_HEIGHT = 0.015
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


_MEM = {"key": None, "t": -1.0, "defect": False, "held_since": None, "lifted": False, "over": False, "in": False,
        "complete": False, "hist": {}, "offline_tcps": None, "offline_t": 0.0}


def new_trial(offline_tcps=None) -> None:
    _MEM.update(key=None, t=-1.0, defect=False, held_since=None, lifted=False, over=False, complete=False, hist={},
                offline_tcps=offline_tcps, offline_t=0.0)
    _MEM["in"] = False


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


def _over(obj, loc) -> bool:
    cx, cy, _ = LOCATIONS[loc]
    x, y = float(obj.pos[0]), float(obj.pos[1])
    return abs(x - cx) <= BIN_IN[0] / 2 and abs(y - cy) <= BIN_IN[1] / 2


def _location_of(obj):
    for loc, (_, _, row) in LOCATIONS.items():
        if _over(obj, loc) and float(obj.pos[2]) - (floor_top(row) + TOTE[2] / 2) < IN_HEIGHT:
            return loc
    return None


def _still(name, t) -> bool:
    h = _MEM["hist"].get(name, [])
    if not h or t - h[0][0] < STILL_S - 1e-9:
        return False
    ps = np.array([p for _, p in h])
    return bool(float(np.max(np.linalg.norm(ps - ps[-1], axis=1))) < STILL_M)


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
    for name in ALL_TOTES:
        h = _MEM["hist"].setdefault(name, [])
        h.append((t, np.asarray(objects[name].pos, float).copy()))
        while h and t - h[0][0] > STILL_S + 1e-9:
            h.pop(0)
        if float(objects[name].low[2]) < TOP - OFF_TABLE:
            _MEM["defect"] = True
    tote = objects["tote"]
    if float(tote.low[2]) > TOP + LIFTED:
        _MEM["lifted"] = True
    if _over(tote, TARGET):
        _MEM["over"] = True
    loc = _location_of(tote)
    still = _still("tote", t)
    if loc is not None and loc != TARGET and still:
        _MEM["defect"] = True  # put away in the wrong location
    for st, loc0 in zip(STORED_TOTES, STORED_IN):
        if _location_of(objects[st.name]) != loc0:
            _MEM["defect"] = True
    _MEM["in"] = bool(loc == TARGET and still)
    free = all(np.linalg.norm(np.asarray(tc) - np.asarray(tote.pos, float)) > RELEASED for tc in _tcps(owner))
    _MEM["complete"] = bool(_MEM["in"] and free)
    if _MEM["complete"] and not _MEM["defect"]:
        if _MEM["held_since"] is None:
            _MEM["held_since"] = t
    else:
        _MEM["held_since"] = None
    _trace(owner, objects, loc)


def _trace(owner, objects, loc) -> None:
    """With PUTAWAY_TRACE=<file> set, append one JSON line per control step (simulator only)."""
    path = os.environ.get("PUTAWAY_TRACE")
    if not path or owner is None:
        return
    rec = {"t": round(_now(owner), 3), "lifted": bool(_MEM["lifted"]), "over": bool(_MEM["over"]),
           "in": bool(_MEM["in"]), "loc": loc, "complete": bool(_MEM["complete"]), "defect": bool(_MEM["defect"]),
           "tcps": [[round(float(v), 4) for v in tcp] for tcp in _tcps(owner)]}
    for name in ALL_TOTES:
        rec[name] = [round(float(v), 4) for v in objects[name].pos]
    with open(path, "a") as f:
        f.write(json.dumps(rec) + "\n")


def success(objects) -> bool:
    _update(objects)
    return bool(_MEM["complete"] and not _MEM["defect"])


def score(objects) -> float:
    """Milestones of 0.25 up to 0.75; 1.0 only once the tote has rested, released, in B2 for SUCCESS_HOLD_S."""
    _update(objects)
    if _MEM["defect"]:
        return 0.0
    hs = _MEM["held_since"]
    if hs is not None and _MEM["t"] - hs >= SUCCESS_HOLD_S - 1e-9:
        return 1.0
    return float(0.25 * _MEM["lifted"] + 0.25 * _MEM["over"] + 0.25 * _MEM["in"])


def distance(objects) -> float:
    """Distance (m) from the tote to where it rests in B2."""
    cx, cy, row = LOCATIONS[TARGET]
    return float(np.linalg.norm(np.asarray(objects["tote"].pos, float) - np.array((cx, cy, floor_top(row) + TOTE[2] / 2))))


# -- the demonstration -----------------------------------------------------------------------------------------------
#
# Scripted, open loop, the left arm only: a straight-down grip on the tote's upper part (pads closing across its 55 mm
# width, finger tips 1.3 cm below the tool point), lift so the tote's bottom clears row B's bin walls by 3 cm, carry it
# straight over B2 (passing over the empty bin A2), lower until the finger tips are 6 mm above the bin walls and let
# go: the tote drops about 2 cm onto the bin floor.

TIP_PAST_TCP = 0.013
GRIP_BELOW_TOP = 0.012
GRIP_ABOVE_BOTTOM = TOTE[2] - GRIP_BELOW_TOP
CARRY_Z = rim("B") + 0.03 + GRIP_ABOVE_BOTTOM
RELEASE_Z = rim("B") + 0.006 + TIP_PAST_TCP
OPEN = FINGER_OPEN * 0.85  # pads about 80 mm apart around the 55 mm tote


def demo():
    bx, by = TOTE_START
    cx, cy, _ = LOCATIONS[TARGET]
    gz = TOP + GRIP_ABOVE_BOTTOM
    left = [
        Waypoint(1.4, (bx, by, CARRY_Z), OPEN, "over the tote"),
        Waypoint(1.0, (bx, by, gz), OPEN, "descend"),
        Waypoint(0.7, (bx, by, gz), FINGER_CLOSED, "grip"),
        Waypoint(1.0, (bx, by, CARRY_Z), FINGER_CLOSED, "lift"),
        Waypoint(1.2, (ROWS["A"][0] - 0.08, cy, CARRY_Z), FINGER_CLOSED, "in front of bay 2"),
        Waypoint(1.4, (cx, cy, CARRY_Z), FINGER_CLOSED, "over B2"),
        Waypoint(0.8, (cx, cy, RELEASE_Z), FINGER_CLOSED, "lower to the bin walls"),
        Waypoint(0.6, (cx, cy, RELEASE_Z), OPEN, "let go"),
        Waypoint(0.8, (cx, cy, CARRY_Z), OPEN, "rise"),
        Waypoint(1.2, (0.12, 0.25, CARRY_Z), FINGER_OPEN, "clear"),
        Waypoint(3.0, (0.12, 0.25, CARRY_Z), FINGER_OPEN, "wait"),
    ]
    return {"left": left}
