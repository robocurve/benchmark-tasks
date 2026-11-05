"""On the station table: two fixed yellow poles stand far apart on a diagonal, one in front of the left arm and
nearer the robot, one in front of the right arm and further out. No line is drawn between them. Three loose red
rods stand upright on the table in a row near the robot. Stand the three rods upright between the poles so that
all five are in one straight line, with every gap between neighbours
along the line from 7 to 16 cm.
"""

import numpy as np

from yam_arena.demo import FINGER_CLOSED, Waypoint
from yam_arena.task_file import TABLE_TOP, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m).
POLE_R, POLE_H = 0.008, 0.150
POLE_A = (0.20, 0.20)  # in front of the left arm, nearer the robot
POLE_B = (0.40, -0.20)  # in front of the right arm, further out
ROD_R, ROD_H, ROD_MASS = 0.012, 0.100, 0.04
ROD_START = {"rod_1": (0.14, 0.13), "rod_2": (0.14, 0.05), "rod_3": (0.14, -0.09)}

YELLOW, RED = (0.95, 0.78, 0.15), (0.80, 0.10, 0.10)

POLES = [
    Cylinder("pole_a", pos=(*POLE_A, TOP + POLE_H / 2), radius=POLE_R, height=POLE_H, color=YELLOW),
    Cylinder("pole_b", pos=(*POLE_B, TOP + POLE_H / 2), radius=POLE_R, height=POLE_H, color=YELLOW),
]
RODS = [Cylinder(n, pos=(*xy, TOP + ROD_H / 2), radius=ROD_R, height=ROD_H, mass=ROD_MASS, color=RED)
        for n, xy in ROD_START.items()]
OBJECTS = [*POLES, *RODS]
ROD_NAMES = tuple(ROD_START)

_A, _B = np.array(POLE_A), np.array(POLE_B)
LINE_LEN = float(np.linalg.norm(_B - _A))  # 0.447 m between pole centres
_U = (_B - _A) / LINE_LEN  # along the line, from pole A to pole B
_N = np.array([-_U[1], _U[0]])  # across the line

GAP_MIN, GAP_MAX = 0.070, 0.160  # every gap between neighbouring rods or poles along the line
TIERS = ((0.005, 1.0), (0.010, 0.6), (0.020, 0.3))  # worst rod's distance from the line -> score
MAX_TILT_DEG = 5.0
BOTTOM_TOL = 0.001  # a rod's bottom within 1 mm of the table top: standing on the table, not held up
MIN_HEIGHT = 0.090  # a standing rod's bounding box is about 100 mm tall; a fallen one about 24 mm


def along_across(xy) -> tuple[float, float]:
    """A point's distance along the line from pole A's centre, and its signed distance from the line (m)."""
    d = np.asarray(xy, float)[:2] - _A
    return float(d @ _U), float(d @ _N)


def _tilt_deg(st) -> float:
    x, y, z, w = st.quat_xyzw
    axis_z = 1.0 - 2.0 * (x * x + y * y)  # the rod axis's vertical component
    # The rod is the same at both ends, so standing on either end is upright.
    return float(np.degrees(np.arccos(np.clip(abs(axis_z), 0.0, 1.0))))


def _standing(st) -> bool:
    """Upright on either end (axis within 5 degrees of vertical), bottom on the table, and at rest."""
    return bool(
        _tilt_deg(st) <= MAX_TILT_DEG
        and st.high[2] - st.low[2] >= MIN_HEIGHT
        and abs(st.low[2] - TOP) <= BOTTOM_TOL
        and st.resting()
    )


def _worst_offset(objects) -> float | None:
    """The largest distance of any rod from the line, or None if the layout fails a hard rule: a rod not
    standing, outside the span between the poles, or any gap along the line outside 70 to 160 mm."""
    along = []
    worst = 0.0
    for n in ROD_NAMES:
        st = objects[n]
        if not _standing(st):
            return None
        t, off = along_across(st.pos)
        if not 0.0 < t < LINE_LEN:
            return None
        along.append(t)
        worst = max(worst, abs(off))
    stations = [0.0, *sorted(along), LINE_LEN]
    gaps = np.diff(stations)
    if np.any(gaps < GAP_MIN - 1e-9) or np.any(gaps > GAP_MAX + 1e-9):
        return None
    return worst


def score(objects):
    """1.0 if every rod is within 5 mm of the line through the pole centres, 0.6 within 10 mm, 0.3 within
    20 mm (judged by the worst rod); 0 if any rod is not standing at rest on the table, lies outside the span
    between the poles, or any gap between neighbours along the line is outside 70 to 160 mm."""
    worst = _worst_offset(objects)
    if worst is None:
        return 0.0
    for tol, s in TIERS:
        if worst <= tol + 1e-9:
            return s
    return 0.0


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    worst = _worst_offset(objects)
    return float("inf") if worst is None else worst


# Demo: rods go to the quarter points of the line, 111.8 mm apart along it.
TARGETS = {n: tuple(_A + f * (_B - _A)) for n, f in (("rod_1", 0.25), ("rod_2", 0.5), ("rod_3", 0.75))}
GRIP_Z = TOP + ROD_H - 0.020  # TCP 20 mm below the rod's top; finger tips 33 mm below it
PLACE_Z = GRIP_Z + 0.006  # rod bottom about 6 mm above the table at release: it drops onto its end, so a held rod never counts
TRAVEL_Z = TOP + 0.15  # a carried rod's bottom clears the table by about 7 cm
PARK_RIGHT = (0.12, -0.28, TRAVEL_Z)
FINGER_WIDE = -0.030  # partly open: enough to pass around a 24 mm rod without touching its neighbour


def _carry(start, goal, settle=1.0):
    (sx, sy), (gx, gy) = start, goal
    return [
        Waypoint(1.5, (sx, sy, TRAVEL_Z), FINGER_WIDE, "over the rod"),
        Waypoint(1.5, (sx, sy, GRIP_Z), FINGER_WIDE, "descend"),
        Waypoint(0.8, (sx, sy, GRIP_Z), FINGER_CLOSED, "grip"),
        Waypoint(1.2, (sx, sy, TRAVEL_Z), FINGER_CLOSED, "lift"),
        Waypoint(2.0, (gx, gy, TRAVEL_Z), FINGER_CLOSED, "carry"),
        Waypoint(1.5, (gx, gy, PLACE_Z), FINGER_CLOSED, "lower"),
        Waypoint(settle, (gx, gy, PLACE_Z), FINGER_CLOSED, "settle"),
        Waypoint(0.6, (gx, gy, PLACE_Z), FINGER_WIDE, "release"),
        Waypoint(1.2, (gx, gy, TRAVEL_Z), FINGER_WIDE, "up"),
    ]


# Measured on the L40S with the trace wrapper: rod 1, carried by the left arm near x 0.25 m, landed 3.9 and
# 4.2 mm off the line (further from the robot than aimed) when aimed at the line, and 3.3 mm off on the other
# side when aimed 4 mm short in x. The demo aims 2 mm short.
AIM_ROD_1 = (TARGETS["rod_1"][0] - 0.002, TARGETS["rod_1"][1])


def demo():
    left = _carry(ROD_START["rod_1"], AIM_ROD_1) + _carry(ROD_START["rod_2"], TARGETS["rod_2"])
    # The right arm parks out of the way: hovering over its rod it would block the left arm placing rod 2.
    right = _carry(ROD_START["rod_3"], TARGETS["rod_3"]) + [Waypoint(1.5, PARK_RIGHT, FINGER_WIDE, "park")]
    return {"left": left, "right": right}
