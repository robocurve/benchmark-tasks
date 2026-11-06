"""On the station table: two secondary-containment trays, each a white floor with a coloured rim. The acid tray
(red rim) is on the left, the base tray (blue rim) on the right; each is 15 x 15 cm inside with 3 cm walls.
Four 50 mL reagent bottles (4.5 cm wide, 7 cm tall): two red acid bottles and one blue base bottle stand in the
middle of the table, and a third red acid bottle stands in the base tray, where an earlier user left it.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 \\
           --instruction "<see task.json>" --scorer task_score ...
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Trays: fixed (no mass). Inner floor 15 x 15 cm, 5 mm thick; walls 5 mm thick, 30 mm above the floor.
INNER = 0.15
FLOOR_T = 0.005
WALL_T = 0.005
WALL_H = 0.030
FLOOR_TOP = TOP + FLOOR_T
TRAY_X = 0.30
TRAYS = {"acid": (TRAY_X, 0.20), "base": (TRAY_X, -0.20)}
RIM = {"acid": (0.80, 0.12, 0.12), "base": (0.15, 0.30, 0.85)}
WHITE = (0.92, 0.92, 0.92)


def _tray(kind):
    cx, cy = TRAYS[kind]
    zf, zw = TOP + FLOOR_T / 2, FLOOR_TOP + WALL_H / 2
    out = INNER + 2 * WALL_T
    off = INNER / 2 + WALL_T / 2
    color = RIM[kind]
    return [
        Box(f"{kind}_tray_floor", pos=(cx, cy, zf), size=(out, out, FLOOR_T), color=WHITE),
        Box(f"{kind}_tray_front", pos=(cx + off, cy, zw), size=(WALL_T, out, WALL_H), color=color),
        Box(f"{kind}_tray_back", pos=(cx - off, cy, zw), size=(WALL_T, out, WALL_H), color=color),
        Box(f"{kind}_tray_left", pos=(cx, cy + off, zw), size=(INNER, WALL_T, WALL_H), color=color),
        Box(f"{kind}_tray_right", pos=(cx, cy - off, zw), size=(INNER, WALL_T, WALL_H), color=color),
    ]


# Bottles: loose. Acids red, the base blue. acid_3 starts misplaced, standing on the base tray's floor.
R, H, MASS = 0.0225, 0.07, 0.10
RED, BLUE = (0.85, 0.12, 0.12), (0.15, 0.30, 0.85)
START = {
    "acid_1": (0.20, -0.05, TOP),
    "base_1": (0.20, 0.05, TOP),
    "acid_2": (0.27, 0.0, TOP),
    "acid_3": (TRAY_X + 0.035, -0.20, FLOOR_TOP),  # base tray, SPOTS["base"][1]
}
KIND = {"acid_1": "acid", "acid_2": "acid", "acid_3": "acid", "base_1": "base"}
BOTTLES = [
    Cylinder(name, pos=(x, y, z + H / 2), radius=R, height=H, mass=MASS, color=RED if KIND[name] == "acid" else BLUE)
    for name, (x, y, z) in START.items()
]

OBJECTS = [*_tray("acid"), *_tray("base"), *BOTTLES]

MAX_TILT_DEG = 15.0
SUCCESS_HOLD_S = 1.0


def _tilt_deg(state) -> float:
    """Angle between the bottle's axis and the vertical."""
    x, y, _, _ = state.quat_xyzw
    cos_tilt = 1.0 - 2.0 * (x * x + y * y)
    return float(np.degrees(np.arccos(np.clip(cos_tilt, -1.0, 1.0))))


def _tray_of(state) -> str | None:
    """The tray whose inner area holds the bottle's centre and whose floor it rests on, or None."""
    px, py, _ = state.pos
    for kind, (cx, cy) in TRAYS.items():
        inside = abs(px - cx) <= INNER / 2 and abs(py - cy) <= INNER / 2
        on_floor = state.low[2] < FLOOR_TOP + 0.015
        if inside and on_floor:
            return kind
    return None


def _stored(name, objects) -> bool:
    s = objects[name]
    return _tray_of(s) == KIND[name] and _tilt_deg(s) <= MAX_TILT_DEG


def _violation(objects) -> bool:
    """Any bottle in the other kind's tray (an acid in the base tray, or the base in the acid tray)."""
    return any(_tray_of(objects[n]) not in (None, KIND[n]) for n in START)


def success(objects):
    """Every bottle upright (within 15 degrees) in its own tray, none in the wrong tray, all bottles still."""
    return bool(
        not _violation(objects)
        and all(_stored(n, objects) for n in START)
        and all(objects[n].resting() for n in START)
    )


def distance(objects):
    """Sum over bottles of the horizontal distance (m) from the bottle to its tray's centre."""
    total = 0.0
    for n in START:
        cx, cy = TRAYS[KIND[n]]
        total += float(np.hypot(objects[n].pos[0] - cx, objects[n].pos[1] - cy))
    return total


def score(objects):
    """0 while any bottle sits in the other kind's tray (the starting layout included); otherwise 1/4 per bottle
    stored upright in its own tray."""
    if _violation(objects):
        return 0.0
    return sum(_stored(n, objects) for n in START) / len(START)


# Scripted demo. Grip each bottle 5 cm up and carry at 14 cm, so a carried bottle clears standing ones by 2 cm
# (higher, the left arm cannot reach across the middle). The arms take turns near the middle of the table:
#   1. left: acid_1 and acid_2 into the acid tray, then parks;
#   2. right: acid_3 out of the base tray to a staging spot in the middle (it cannot reach the acid tray),
#      then base_1 into the base tray, then parks;
#   3. left: acid_3 from the staging spot into the acid tray.
GRIP_Z = TOP + 0.05
TRAVEL_Z = TOP + 0.14
STAGE = (0.20, -0.04)
PARK = {"left": (0.20, 0.30, TRAVEL_Z), "right": (0.20, -0.30, TRAVEL_Z)}


# Spots inside a tray, as (dx, dy) from its centre: a triangle, so the fingers (which close along y) never
# straddle a neighbour. The acid tray takes three bottles; the base tray's second spot is acid_3's start.
SPOTS = {"acid": ((-0.04, -0.035), (-0.04, 0.035), (0.04, 0.0)), "base": ((-0.035, 0.0), (0.035, 0.0))}
FINGER_WIDE = -0.03  # each finger 3 cm out: a 6 cm opening, 7.5 mm either side of a 4.5 cm bottle


def _place(kind, k):
    """Place target for the k-th bottle in a tray (TCP height, bottle bottom just above the floor)."""
    cx, cy = TRAYS[kind]
    dx, dy = SPOTS[kind][k]
    return (cx + dx, cy + dy, GRIP_Z + FLOOR_T + 0.003)


def _pick(name):
    x, y, z = START[name]
    return (x, y, GRIP_Z + (z - TOP))


def _move(pick, place):
    """Top-down pick and place with the fingers opened only to FINGER_WIDE, not fully."""
    wps = pick_place_waypoints(pick, place, travel_z=TRAVEL_Z)
    return [Waypoint(w.duration, w.pos, FINGER_WIDE if w.finger == FINGER_OPEN else w.finger, w.label, w.yaw) for w in wps]


def _duration(wps):
    return sum(w.duration for w in wps)


def demo():
    stage_place = (*STAGE, GRIP_Z + 0.003)
    stage_pick = (*STAGE, GRIP_Z)

    left = _move(_pick("acid_1"), _place("acid", 0)) + _move(_pick("acid_2"), _place("acid", 1))
    left += [Waypoint(1.5, PARK["left"], FINGER_OPEN, "park")]
    t_left_parked = _duration(left)

    right = [Waypoint(t_left_parked, PARK["right"], FINGER_OPEN, "wait for the left arm")]
    right += _move(_pick("acid_3"), stage_place) + _move(_pick("base_1"), _place("base", 0))
    right += [Waypoint(1.5, PARK["right"], FINGER_OPEN, "park")]
    t_right_parked = _duration(right)

    left += [Waypoint(t_right_parked - t_left_parked, PARK["left"], FINGER_OPEN, "wait for the right arm")]
    left += _move(stage_pick, _place("acid", 2)) + [Waypoint(1.5, PARK["left"], FINGER_OPEN, "park")]
    right += [Waypoint(_duration(left) - t_right_parked, PARK["right"], FINGER_OPEN, "hold")]
    return {"left": left, "right": right}
