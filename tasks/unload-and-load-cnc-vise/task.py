"""On the station table: a CNC machine vise between the arms holding a finished part; an output tray in front of the
right arm and a raw blank in front of the left arm. Nothing else is on the table."""

# A CNC machine operator's core operation between cycles: unload the finished part from the vise to the output tray,
# then load the next blank and seat it against the vise's fixed jaw and side stop, so the program machines it in the
# right place. A blank loaded off its stops is machined out of position (scrap); loading with the finished part still
# in the vise crashes the cycle.
#
# Reliability stressor (one fixed level): order plus precision. Unload must come before load, and the blank counts as
# seated only within 2 mm of both the fixed jaw and the side stop.
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

INSTRUCTION = "Unload the finished part to the output tray, then load the blank into the vise against both stops."

# -- the scene -------------------------------------------------------------------------------------------------------

IRON = (0.30, 0.33, 0.36)  # vise
BRIGHT = (0.85, 0.87, 0.90)  # machined finish
RAW = (0.55, 0.52, 0.47)  # saw-cut aluminium blank
TRAY_COL = (0.20, 0.45, 0.30)

PART = (0.05, 0.04, 0.03)  # finished part and blank: 50 x 40 x 30 mm, 150 g
PART_MASS = 0.15

# The vise: a fixed body whose top is the vise floor, a fixed jaw along its far edge and a side stop along its left
# edge, both 15 mm tall, so a part's upper half stands above them.
VISE_XY = (0.30, -0.03)
VISE_SIZE = (0.14, 0.14)
VISE_H = 0.04
VISE_FLOOR = TOP + VISE_H
STOP_H = 0.015
JAW_T = 0.02
JAW_FACE_X = VISE_XY[0] + VISE_SIZE[0] / 2 - JAW_T  # the fixed jaw's clamping face
STOP_FACE_Y = VISE_XY[1] + VISE_SIZE[1] / 2 - JAW_T  # the side stop's face
SEATED_XY = (JAW_FACE_X - PART[0] / 2, STOP_FACE_Y - PART[1] / 2)
SEATED_Z = VISE_FLOOR + PART[2] / 2

# The output tray: inside 12 x 12 cm, a 1.5 cm lip around a 1 cm floor, in front of the right arm.
TRAY_XY = (0.24, -0.27)
TRAY_IN = 0.12
TRAY_LIP = 0.015
TRAY_FLOOR = TOP + 0.01
WALL = 0.008

BLANK_START = (0.22, 0.27)

FINISHED = Box("finished_part", pos=(*SEATED_XY, SEATED_Z), size=PART, mass=PART_MASS, color=BRIGHT)
BLANK = Box("blank", pos=(*BLANK_START, TOP + PART[2] / 2), size=PART, mass=PART_MASS, color=RAW)


def _vise():
    vx, vy = VISE_XY
    sx, sy = VISE_SIZE
    zc = VISE_FLOOR + STOP_H / 2
    return [
        Box("vise_body", pos=(vx, vy, TOP + VISE_H / 2), size=(sx, sy, VISE_H), color=IRON),
        Box("vise_fixed_jaw", pos=(vx + sx / 2 - JAW_T / 2, vy, zc), size=(JAW_T, sy, STOP_H), color=IRON),
        Box("vise_side_stop", pos=(vx - JAW_T / 2, vy + sy / 2 - JAW_T / 2, zc), size=(sx - JAW_T, JAW_T, STOP_H),
            color=IRON),
    ]


def _tray():
    cx, cy = TRAY_XY
    o = TRAY_IN + 2 * WALL
    zc = TRAY_FLOOR + TRAY_LIP / 2
    return [
        Box("tray_floor", pos=(cx, cy, TOP + 0.005), size=(o, o, 0.01), color=TRAY_COL),
        Box("tray_lip_near", pos=(cx - TRAY_IN / 2 - WALL / 2, cy, zc), size=(WALL, o, TRAY_LIP), color=TRAY_COL),
        Box("tray_lip_far", pos=(cx + TRAY_IN / 2 + WALL / 2, cy, zc), size=(WALL, o, TRAY_LIP), color=TRAY_COL),
        Box("tray_lip_left", pos=(cx, cy + TRAY_IN / 2 + WALL / 2, zc), size=(TRAY_IN, WALL, TRAY_LIP), color=TRAY_COL),
        Box("tray_lip_right", pos=(cx, cy - TRAY_IN / 2 - WALL / 2, zc), size=(TRAY_IN, WALL, TRAY_LIP),
            color=TRAY_COL),
    ]


VISE = _vise()
TRAY = _tray()
OBJECTS = [FINISHED, BLANK, *VISE, *TRAY]

# -- scoring ---------------------------------------------------------------------------------------------------------
#
# A part is in the vise when its centre lies over the vise floor (inside the jaws and stops, seen from above) less than
# 1.5 cm above where it would rest on it. The finished part is unloaded once it has left the vise; it is in the tray
# when its centre lies inside the tray's inside square less than 1.5 cm above where it would rest on the tray floor.
# The blank is seated when it is in the vise, its far face is within 2 mm of the fixed jaw, its left face within 2 mm
# of the side stop, and it is square (its footprint no more than 3 mm larger than the part's own in either direction,
# about 3 degrees).
#   score = 0.25 [finished part out of the vise] + 0.25 [finished part in the tray, still]
#         + 0.25 [blank in the vise, still, finished part out]                       (0 to 0.75), at the end;
#   1.0   = success: finished part in the tray, blank seated, both still (moved under 5 mm over the last 0.3 s), no
#           gripper tool point within 4 cm of either; held for 1 s. Completed less than 1 s before the run limit: 0.75.
# Defects that score 0: the blank and the finished part both in the vise at once (loading over an unloaded part), or
# either part falling off the table.

SEAT_TOL = 0.002
SQUARE_TOL = 0.003
IN_HEIGHT = 0.015
STILL_S = 0.3
STILL_M = 0.005
RELEASED = 0.04
OFF_TABLE = 0.03
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
HZ_OFFLINE = 20.0
NAMES = ("finished_part", "blank")


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


_MEM = {"key": None, "t": -1.0, "defect": False, "held_since": None, "state": {}, "complete": False,
        "hist": {}, "offline_tcps": None, "offline_t": 0.0}


def new_trial(offline_tcps=None) -> None:
    _MEM.update(key=None, t=-1.0, defect=False, held_since=None, state={}, complete=False, hist={},
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


def _in_vise(obj) -> bool:
    x, y, z = (float(v) for v in obj.pos)
    x0 = VISE_XY[0] - VISE_SIZE[0] / 2
    y0 = VISE_XY[1] - VISE_SIZE[1] / 2
    return x0 <= x <= JAW_FACE_X and y0 <= y <= STOP_FACE_Y and z - SEATED_Z < IN_HEIGHT


def _in_tray(obj) -> bool:
    x, y, z = (float(v) for v in obj.pos)
    return (abs(x - TRAY_XY[0]) <= TRAY_IN / 2 and abs(y - TRAY_XY[1]) <= TRAY_IN / 2
            and z - (TRAY_FLOOR + PART[2] / 2) < IN_HEIGHT)


def _seated(obj) -> bool:
    lo, hi = np.asarray(obj.low, float), np.asarray(obj.high, float)
    ext = hi - lo
    gap_jaw, gap_stop = JAW_FACE_X - hi[0], STOP_FACE_Y - hi[1]  # -1 mm allows for contact penetration
    return (_in_vise(obj) and -0.001 <= gap_jaw <= SEAT_TOL and -0.001 <= gap_stop <= SEAT_TOL
            and ext[0] - PART[0] <= SQUARE_TOL and ext[1] - PART[1] <= SQUARE_TOL)


def _still(name, t) -> bool:
    h = _MEM["hist"].get(name, [])
    if not h or t - h[0][0] < STILL_S - 1e-9:
        return False
    ps = np.array([p for _, p in h])
    return float(np.max(np.linalg.norm(ps - ps[-1], axis=1))) < STILL_M


def _evaluate(objects, owner, t):
    fin, blank = objects["finished_part"], objects["blank"]
    s = {
        "unloaded": bool(not _in_vise(fin)),
        "in_tray": bool(_in_tray(fin) and _still("finished_part", t)),
        "loaded": bool(_in_vise(blank) and not _in_vise(fin) and _still("blank", t)),
        "seated": bool(_seated(blank) and _still("blank", t)),
    }
    tcps = _tcps(owner)
    free = all(np.linalg.norm(np.asarray(tc) - np.asarray(objects[n].pos, float)) > RELEASED
               for n in NAMES for tc in tcps)
    complete = bool(s["in_tray"] and s["seated"] and s["unloaded"] and free)
    return s, complete


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
    for name in NAMES:
        h = _MEM["hist"].setdefault(name, [])
        h.append((t, np.asarray(objects[name].pos, float).copy()))
        while h and t - h[0][0] > STILL_S + 1e-9:
            h.pop(0)
        if float(objects[name].low[2]) < TOP - OFF_TABLE:
            _MEM["defect"] = True
    if _in_vise(objects["finished_part"]) and _in_vise(objects["blank"]):
        _MEM["defect"] = True
    state, complete = _evaluate(objects, owner, t)
    _MEM["state"], _MEM["complete"] = state, complete
    if complete and not _MEM["defect"]:
        if _MEM["held_since"] is None:
            _MEM["held_since"] = t
    else:
        _MEM["held_since"] = None
    _trace(owner, objects)


def _trace(owner, objects) -> None:
    """With CNC_TRACE=<file> set, append one JSON line per control step (simulator only)."""
    path = os.environ.get("CNC_TRACE")
    if not path or owner is None:
        return
    rec = {"t": round(_now(owner), 3), **{k: bool(v) for k, v in _MEM["state"].items()},
           "complete": bool(_MEM["complete"]), "defect": bool(_MEM["defect"]),
           "tcps": [[round(float(v), 4) for v in tcp] for tcp in _tcps(owner)]}
    for name in NAMES:
        rec[name] = [round(float(v), 4) for v in objects[name].pos]
        rec[name + "_hi"] = [round(float(v), 4) for v in objects[name].high]
        rec[name + "_lo"] = [round(float(v), 4) for v in objects[name].low]
    with open(path, "a") as f:
        f.write(json.dumps(rec) + "\n")


def success(objects) -> bool:
    _update(objects)
    return bool(_MEM["complete"] and not _MEM["defect"])


def score(objects) -> float:
    """Milestones of 0.25 up to 0.75; 1.0 only once the complete changeover has held for SUCCESS_HOLD_S."""
    _update(objects)
    if _MEM["defect"]:
        return 0.0
    hs = _MEM["held_since"]
    if hs is not None and _MEM["t"] - hs >= SUCCESS_HOLD_S - 1e-9:
        return 1.0
    s = _MEM["state"]
    return float(0.25 * bool(s.get("unloaded")) + 0.25 * bool(s.get("in_tray")) + 0.25 * bool(s.get("loaded")))


def distance(objects) -> float:
    """Distance (m) of the finished part from the tray centre plus the blank from its seated position."""
    f, b = objects["finished_part"].pos, objects["blank"].pos
    return (math.hypot(float(f[0]) - TRAY_XY[0], float(f[1]) - TRAY_XY[1])
            + float(np.linalg.norm(np.asarray(b, float) - np.array((*SEATED_XY, SEATED_Z)))))


# -- the demonstration -----------------------------------------------------------------------------------------------
#
# Scripted, open loop. Grips are straight down with the pads closing along y, on the upper half of the part only
# (finger tips 3 mm above the stops' tops), so a held part can be slid into the corner without the fingers touching
# the stops. The right arm unloads the finished part to the tray. Then the left arm brings the blank over the vise,
# lowers it 1.2 cm short of both stops and 2 mm above the floor, slides it sideways onto the side stop (still 4 mm off
# the fixed jaw, so the jaw adds no friction), then forward onto the fixed jaw, commanding 12 and 6 mm past contact so
# it presses against both, and lets go. A single diagonal slide stopped 2.3 mm short of the side stop in simulation.

TIP_PAST_TCP = 0.013
GRIP_ABOVE_BOTTOM = STOP_H + 0.003 + TIP_PAST_TCP  # tool point height above the part's bottom face
CARRY_Z = VISE_FLOOR + 0.05 + GRIP_ABOVE_BOTTOM  # a carried part's bottom 5 cm above the vise floor (3.5 above stops)
OPEN = FINGER_OPEN * 0.75  # pads about 70 mm apart around the 40 mm part
SHORT = 0.012
JAW_CLEAR = 0.004  # stay this far off the jaw while sliding onto the side stop, so the jaw adds no friction
PRESS_X = 0.006  # commanded past contact; in simulation the arm lags about 7 mm when sliding a part under load
PRESS_Y = 0.012
LEFT_START_S = 9.0  # the left arm waits until the right arm has unloaded and left the vise


def _right():
    fx, fy = SEATED_XY
    tx, ty = TRAY_XY
    gz = VISE_FLOOR + GRIP_ABOVE_BOTTOM
    rz = TRAY_FLOOR + TRAY_LIP + 0.01 + GRIP_ABOVE_BOTTOM  # part bottom 1 cm above the tray lip
    return [
        Waypoint(1.4, (fx, fy, CARRY_Z), OPEN, "over the finished part"),
        Waypoint(1.0, (fx, fy, gz), OPEN, "descend"),
        Waypoint(0.7, (fx, fy, gz), FINGER_CLOSED, "grip"),
        Waypoint(1.0, (fx, fy, CARRY_Z), FINGER_CLOSED, "lift out of the vise"),
        Waypoint(1.6, (tx, ty, CARRY_Z), FINGER_CLOSED, "over the output tray"),
        Waypoint(0.8, (tx, ty, rz), FINGER_CLOSED, "lower"),
        Waypoint(0.6, (tx, ty, rz), OPEN, "let go"),
        Waypoint(0.7, (tx, ty, CARRY_Z), OPEN, "rise"),
        Waypoint(1.2, (0.12, -0.30, CARRY_Z), FINGER_OPEN, "clear"),
        Waypoint(16.0, (0.12, -0.30, CARRY_Z), FINGER_OPEN, "wait"),
    ]


def _left():
    bx, by = BLANK_START
    sx, sy = SEATED_XY
    gz = TOP + GRIP_ABOVE_BOTTOM
    lz = VISE_FLOOR + 0.002 + GRIP_ABOVE_BOTTOM
    return [
        Waypoint(LEFT_START_S, (0.12, 0.30, CARRY_Z), FINGER_OPEN, "wait for the unload"),
        Waypoint(1.4, (bx, by, CARRY_Z), OPEN, "over the blank"),
        Waypoint(1.0, (bx, by, gz), OPEN, "descend"),
        Waypoint(0.7, (bx, by, gz), FINGER_CLOSED, "grip"),
        Waypoint(1.0, (bx, by, CARRY_Z), FINGER_CLOSED, "lift"),
        Waypoint(1.6, (sx - SHORT, sy - SHORT, CARRY_Z), FINGER_CLOSED, "over the vise, short of the stops"),
        Waypoint(1.0, (sx - SHORT, sy - SHORT, lz), FINGER_CLOSED, "lower to just above the floor"),
        Waypoint(1.2, (sx - JAW_CLEAR, sy + PRESS_Y, lz), FINGER_CLOSED, "slide onto the side stop"),
        Waypoint(1.2, (sx + PRESS_X, sy + PRESS_Y, lz), FINGER_CLOSED, "slide onto the fixed jaw"),
        Waypoint(0.6, (sx + PRESS_X, sy + PRESS_Y, lz), FINGER_CLOSED, "press"),
        Waypoint(0.6, (sx + PRESS_X, sy + PRESS_Y, lz), OPEN, "let go"),
        Waypoint(0.8, (sx, sy, CARRY_Z), OPEN, "rise"),
        Waypoint(1.2, (0.12, 0.30, CARRY_Z), FINGER_OPEN, "clear"),
        Waypoint(3.0, (0.12, 0.30, CARRY_Z), FINGER_OPEN, "wait"),
    ]


def demo():
    return {"left": _left(), "right": _right()}
