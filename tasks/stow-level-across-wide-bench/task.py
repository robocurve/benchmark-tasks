"""On the station table: at the far left end, a flat grey tray with a yellow 30 cm spirit level lying on it, its length
pointing away from the arms; at the far right end, a wooden tool caddy with one long, deep, narrow slot, also pointing
away from the arms. Nothing else is on the table. The tray and the caddy are farther apart than either arm can reach:
the left arm reaches the level but not the caddy, the right arm the caddy but not the level."""

# The task file the simulator runs (Isaac Lab-Arena with the yam_arena embodiment, through Inspect Robots).
# World frame: x forward (away from the arms), y left, z up, metres. The arm bases stand at x 0, y +-0.305.
#
# Why the level must change hands: each YAM arm's tool point reaches at most about 79 cm from its base, in any pose,
# above the table (sampled over its joint limits; see check_task.py). Every part of the level on its tray is at least
# 84 cm from the right arm's base, and every point inside the caddy's slot at least 82 cm from the left arm's base.
#
# Why the slot is hard to see: its walls are 10 cm tall and it is 6 cm wide, so the station's top camera, high behind
# the arms, sees only part of a level lying in it (check_task.py reports how much). The wrist cameras see into it.

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

INSTRUCTION = "Put the spirit level away in the slot of the wooden tool caddy."

# -- the scene -------------------------------------------------------------------------------------------------------

# The spirit level: an aluminium box-beam level, 300 x 25 x 50 mm (length x thickness x height), 300 g, painted yellow.
LEVEL_SIZE = (0.30, 0.025, 0.05)
LEVEL_MASS = 0.30
LEVEL_X = 0.18  # its centre's distance forward of the arm bases, at the start and in the slot

# The inbound tray: flat grey plastic, 36 x 10 x 1.2 cm, taped to the table at the far left.
TRAY_SIZE = (0.36, 0.10, 0.012)
TRAY_XY = (LEVEL_X, 0.55)
TRAY_TOP = TOP + TRAY_SIZE[2]
LEVEL_START = (LEVEL_X, TRAY_XY[1], TRAY_TOP + LEVEL_SIZE[2] / 2)

# The tool caddy: 12 mm plywood. A floor 38.4 x 8.4 cm, and four walls rising 10 cm above it around a slot 36 cm long
# and 6 cm wide inside. Taped to the table at the far right.
WALL = 0.012
FLOOR = 0.012
SLOT = (0.36, 0.06)  # inside length (x) and width (y)
RIM_ABOVE_FLOOR = 0.10
CADDY_XY = (LEVEL_X, -0.55)
FLOOR_TOP = TOP + FLOOR
RIM = FLOOR_TOP + RIM_ABOVE_FLOOR  # the top of the walls
SLOT_LO = (CADDY_XY[0] - SLOT[0] / 2, CADDY_XY[1] - SLOT[1] / 2)
SLOT_HI = (CADDY_XY[0] + SLOT[0] / 2, CADDY_XY[1] + SLOT[1] / 2)

YELLOW = (0.95, 0.78, 0.10)
TRAY_GREY = (0.45, 0.47, 0.50)
PLYWOOD = (0.76, 0.60, 0.40)

LEVEL = Box("spirit_level", pos=LEVEL_START, size=LEVEL_SIZE, mass=LEVEL_MASS, color=YELLOW)
TRAY = Box("inbound_tray", pos=(*TRAY_XY, TOP + TRAY_SIZE[2] / 2), size=TRAY_SIZE, color=TRAY_GREY)
_OUT_X, _OUT_Y = SLOT[0] + 2 * WALL, SLOT[1] + 2 * WALL
_WALL_Z = FLOOR_TOP + RIM_ABOVE_FLOOR / 2
CADDY = [
    Box("caddy_floor", pos=(*CADDY_XY, TOP + FLOOR / 2), size=(_OUT_X, _OUT_Y, FLOOR), color=PLYWOOD),
    Box("caddy_wall_near", pos=(CADDY_XY[0] - SLOT[0] / 2 - WALL / 2, CADDY_XY[1], _WALL_Z),
        size=(WALL, _OUT_Y, RIM_ABOVE_FLOOR), color=PLYWOOD),
    Box("caddy_wall_far", pos=(CADDY_XY[0] + SLOT[0] / 2 + WALL / 2, CADDY_XY[1], _WALL_Z),
        size=(WALL, _OUT_Y, RIM_ABOVE_FLOOR), color=PLYWOOD),
    Box("caddy_wall_inner", pos=(CADDY_XY[0], CADDY_XY[1] + SLOT[1] / 2 + WALL / 2, _WALL_Z),
        size=(SLOT[0], WALL, RIM_ABOVE_FLOOR), color=PLYWOOD),
    Box("caddy_wall_outer", pos=(CADDY_XY[0], CADDY_XY[1] - SLOT[1] / 2 - WALL / 2, _WALL_Z),
        size=(SLOT[0], WALL, RIM_ABOVE_FLOOR), color=PLYWOOD),
]
OBJECTS = [LEVEL, TRAY, *CADDY]

# -- scoring ---------------------------------------------------------------------------------------------------------
#
# Milestones (each needs the ones before it to have been reached; the score is the highest reached):
#   0.25  lifted:   the level's lowest point at least 3 cm above the tray's top.
#   0.50  across:   the level's centre at least 30 cm right of the centre line between the arms (y <= -0.30) while
#                   its lowest point is at least 1 cm above the table (carried, not slid).
#   0.75  entering: the level's centre over the slot (inside its inner footprint) and its lowest point below the rim.
#   1.00  stowed:   success, below.
# Success: the whole level lies inside the slot, seen from above (within 3 mm of the inner walls), entirely below the
# rim, still (under 1 cm/s and 0.5 rad/s), and released (neither gripper's tool point within 2.5 cm of it), for 1 s.
# Failure: the score is 0 if the level ever falls below the table top by more than 3 cm (off the table).

LIFT = 0.03
ACROSS_Y = -0.30
CARRIED = 0.01
INSIDE_TOL = 0.003
RELEASED_MARGIN = 0.025
OFF_TABLE = 0.03
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"


def _owner():
    """The yam_arena embodiment calling this hook (it holds the trial clock ``_t`` and the simulator ``_env``), or
    None when called offline (tests, check_task.py)."""
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


# Per-trial memory: the milestones reached and whether the level fell. A new trial is recognised by the embodiment's
# clock going backwards (it resets ``_t`` to 0 at every reset). Offline, call new_trial() between runs.
_MEM = {"key": None, "t": -1.0, "reached": 0.0, "fell": False, "offline_tcps": None}


def new_trial(offline_tcps=None) -> None:
    """Forget the last trial. ``offline_tcps`` (a callable returning the two tool points) stands in for the
    simulator's grippers in offline checks."""
    _MEM.update(key=None, t=-1.0, reached=0.0, fell=False, offline_tcps=offline_tcps)


def _tcps(owner) -> list:
    """World positions of both grippers' tool points (13 mm short of the pad tips, between them)."""
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


def _released(level, owner) -> bool:
    lo = np.asarray(level.low, dtype=float) - RELEASED_MARGIN
    hi = np.asarray(level.high, dtype=float) + RELEASED_MARGIN
    return not any(bool(np.all(t >= lo) and np.all(t <= hi)) for t in _tcps(owner))


def _over_slot(x: float, y: float, tol: float = 0.0) -> bool:
    return SLOT_LO[0] - tol <= x <= SLOT_HI[0] + tol and SLOT_LO[1] - tol <= y <= SLOT_HI[1] + tol


def _inside_slot(level) -> bool:
    lo, hi = np.asarray(level.low, dtype=float), np.asarray(level.high, dtype=float)
    return (_over_slot(lo[0], lo[1], INSIDE_TOL) and _over_slot(hi[0], hi[1], INSIDE_TOL)
            and float(hi[2]) <= RIM)


def _stowed(level, owner) -> bool:
    return _inside_slot(level) and level.resting(0.01, 0.5) and _released(level, owner)


def _update(objects) -> None:
    owner = _owner()
    key, t = (id(owner), float(owner._t)) if owner is not None else ("offline", None)
    if key != _MEM["key"] or (t is not None and t < _MEM["t"]):
        new_trial(_MEM.get("offline_tcps") if owner is None else None)
        _MEM["key"] = key
    if t is not None:
        _MEM["t"] = t
    level = objects["spirit_level"]
    x, y, _ = (float(v) for v in level.pos)
    low = float(level.low[2])
    if low < TOP - OFF_TABLE:
        _MEM["fell"] = True
    r = _MEM["reached"]
    if r < 0.25 and low >= TRAY_TOP + LIFT:
        r = 0.25
    if r >= 0.25 and r < 0.5 and y <= ACROSS_Y and low >= TOP + CARRIED:
        r = 0.5
    if r >= 0.5 and r < 0.75 and _over_slot(x, y) and low < RIM:
        r = 0.75
    _MEM["reached"] = r
    _trace(owner, level, r)


def _trace(owner, level, reached) -> None:
    """With STOW_TRACE=<file> set, append one JSON line per control step (simulator only): time, the level's pose
    and bounding box, both tool points and the milestone reached. For diagnosing demo or policy runs."""
    path = os.environ.get("STOW_TRACE")
    if not path or owner is None:
        return
    rec = {
        "t": round(float(owner._t), 3),
        "pos": [round(float(v), 4) for v in level.pos],
        "quat_xyzw": [round(float(v), 4) for v in level.quat_xyzw],
        "low": [round(float(v), 4) for v in level.low],
        "high": [round(float(v), 4) for v in level.high],
        "tcps": [[round(float(v), 4) for v in tcp] for tcp in _tcps(owner)],
        "reached": reached,
    }
    with open(path, "a") as f:
        f.write(json.dumps(rec) + "\n")


def success(objects) -> bool:
    _update(objects)
    if _MEM["fell"] or _MEM["reached"] < 0.75:
        return False
    return _stowed(objects["spirit_level"], _owner())


def score(objects) -> float:
    """Partial credit at this step (the trial's score is this at the end, or 1.0 if success() held for 1 s)."""
    _update(objects)
    if _MEM["fell"]:
        return 0.0
    if _MEM["reached"] >= 0.75 and _stowed(objects["spirit_level"], _owner()):
        return 1.0
    return _MEM["reached"]


def distance(objects) -> float:
    """Metres from the level's centre to the slot's centre, seen from above."""
    x, y, _ = (float(v) for v in objects["spirit_level"].pos)
    return math.hypot(x - CADDY_XY[0], y - CADDY_XY[1])


# -- the demonstration -----------------------------------------------------------------------------------------------
#
# Scripted, open loop, straight-down grips (fingers closing across the level's 25 mm thickness). The left arm lifts the
# level by its rear part and carries it to the centre line; the right arm takes it near its middle; the left lets go
# and moves clear; the right carries it over the caddy, lowers it until its finger tips are just above the rim, lets go
# (the level drops about 8 cm to the slot's floor, its lower 2 cm already between the walls), rises and moves clear.

LEFT_GRIP_FROM_CENTRE = 0.10  # the left hand grips 10 cm behind the level's centre (5 cm from its rear end)
RIGHT_GRIP_FROM_CENTRE = 0.04  # the right hand grips 4 cm ahead of it, close to its balance point: the level hangs
# from a parallel grip about the fingers' closing axis, its weakest axis, so a grip far from the centre lets it swing
# round (seen in simulation with a 10 cm offset); the hands are 14 cm apart at the handover
GRIP_ABOVE_CENTRE = 0.015  # tool point above the level's centre: the pad tips 2 mm above it
LEFT_X = LEVEL_X - LEFT_GRIP_FROM_CENTRE
RIGHT_X = LEVEL_X + RIGHT_GRIP_FROM_CENTRE
PICK_Z = LEVEL_START[2] + GRIP_ABOVE_CENTRE
LIFT_Z = TRAY_TOP + 0.13  # tool point; the level's bottom 8 cm above the tray
HAND_Z = TOP + 0.10  # handover height: the level's bottom 5.5 cm above the table
TIP_PAST_TCP = 0.013
# Over the caddy the level's bottom is 4.5 cm above the rim as planned; in simulation it hangs about 2 cm lower in the
# grip than planned, and with less clearance it caught the caddy's inner wall (STOW_TRACE).
CARRY_Z = RIM + 0.045 + LEVEL_SIZE[2] / 2 + GRIP_ABOVE_CENTRE
RELEASE_Z = RIM + 0.008 + TIP_PAST_TCP  # the pad tips 8 mm above the rim
RIGHT_AIM_Y = CADDY_XY[1]  # the slot's centre line
PARTLY_OPEN = FINGER_OPEN * 0.6  # pads about 56 mm apart: clear of the 25 mm level without swinging wide


def demo():
    left = [
        Waypoint(1.5, (LEFT_X, TRAY_XY[1], LIFT_Z), FINGER_OPEN, "over the level"),
        Waypoint(1.2, (LEFT_X, TRAY_XY[1], PICK_Z), FINGER_OPEN, "descend"),
        Waypoint(0.8, (LEFT_X, TRAY_XY[1], PICK_Z), FINGER_CLOSED, "grip"),
        Waypoint(1.0, (LEFT_X, TRAY_XY[1], LIFT_Z), FINGER_CLOSED, "lift"),
        Waypoint(2.5, (LEFT_X, 0.0, HAND_Z), FINGER_CLOSED, "to the centre line"),  # t = 7.0
        Waypoint(3.0, (LEFT_X, 0.0, HAND_Z), FINGER_CLOSED, "hold for the right hand"),  # t = 10.0
        Waypoint(0.8, (LEFT_X, 0.0, HAND_Z), PARTLY_OPEN, "let go"),  # t = 10.8
        Waypoint(1.0, (LEFT_X, 0.0, HAND_Z + 0.05), PARTLY_OPEN, "rise"),
        Waypoint(1.2, (LEFT_X, 0.20, HAND_Z + 0.05), FINGER_OPEN, "clear"),
        Waypoint(9.0, (LEFT_X, 0.20, HAND_Z + 0.05), FINGER_OPEN, "wait"),
    ]
    right = [
        Waypoint(3.5, (RIGHT_X, -0.12, HAND_Z + 0.05), FINGER_OPEN, "wait beside the centre line"),
        Waypoint(3.5, (RIGHT_X, -0.12, HAND_Z + 0.05), FINGER_OPEN, "wait"),  # t = 7.0
        Waypoint(1.0, (RIGHT_X, 0.0, HAND_Z + 0.05), FINGER_OPEN, "over the level"),
        Waypoint(1.0, (RIGHT_X, 0.0, HAND_Z), FINGER_OPEN, "descend"),  # t = 9.0
        Waypoint(0.8, (RIGHT_X, 0.0, HAND_Z), FINGER_CLOSED, "grip"),  # t = 9.8
        Waypoint(2.2, (RIGHT_X, 0.0, HAND_Z), FINGER_CLOSED, "hold while the left lets go"),  # t = 12.0
        Waypoint(1.2, (RIGHT_X, -0.05, CARRY_Z), FINGER_CLOSED, "lift"),
        Waypoint(4.0, (RIGHT_X, RIGHT_AIM_Y, CARRY_Z), FINGER_CLOSED, "over the caddy"),  # t = 17.2
        Waypoint(1.0, (RIGHT_X, RIGHT_AIM_Y, CARRY_Z), FINGER_CLOSED, "settle"),
        Waypoint(1.5, (RIGHT_X, RIGHT_AIM_Y, RELEASE_Z), FINGER_CLOSED, "lower to the rim"),
        Waypoint(0.8, (RIGHT_X, RIGHT_AIM_Y, RELEASE_Z), FINGER_CLOSED, "settle"),  # t = 20.5
        Waypoint(0.8, (RIGHT_X, RIGHT_AIM_Y, RELEASE_Z), PARTLY_OPEN, "let go"),  # t = 21.3
        Waypoint(0.8, (RIGHT_X, RIGHT_AIM_Y, CARRY_Z), PARTLY_OPEN, "rise"),
        Waypoint(1.0, (RIGHT_X, -0.35, CARRY_Z), FINGER_OPEN, "clear"),
        Waypoint(2.0, (RIGHT_X, -0.35, CARRY_Z), FINGER_OPEN, "wait"),  # t = 25.1
    ]
    return {"left": left, "right": right}
