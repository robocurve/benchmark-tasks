"""On the station table, in front of the arms and to the left of the midline: a stack of three cardboard cartons of
the same size and look. Each carton is shifted toward the robot relative to the carton under it, so the stack leans
out toward the robot.
"""

import numpy as np

from yam_arena.demo import FINGER_CLOSED, Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m).
CARTON = (0.080, 0.080, 0.060)
CARTON_MASS = 0.20
BOTTOM_MASS = 1.00  # the bottom carton holds heavier goods
BASE_X, BASE_Y = 0.30, 0.10
# Each carton's shift toward the robot (-x) from the one below it, bottom to top. With these the cartons above each
# joint have their combined centre of mass 33 and 30 mm toward the robot from the centre of the carton under them,
# against 40 mm to its edge: the stack stands, with 7 mm to spare at the bottom joint.
SHIFTS = (0.018, 0.030)
NAMES = ("carton_1", "carton_2", "carton_3")  # 1 at the bottom
START_X = {NAMES[0]: BASE_X}
for _i, _s in enumerate(SHIFTS):
    START_X[NAMES[_i + 1]] = START_X[NAMES[_i]] - _s
START_Z = {n: TOP + CARTON[2] * (i + 0.5) for i, n in enumerate(NAMES)}

KRAFT = (0.72, 0.55, 0.36)
CARTONS = [Box(n, pos=(START_X[n], BASE_Y, START_Z[n]), size=CARTON, mass=BOTTOM_MASS if i == 0 else CARTON_MASS,
               color=KRAFT) for i, n in enumerate(NAMES)]
OBJECTS = CARTONS
SPEC = {o.name: o for o in CARTONS}

OVERHANG_TOL = 0.005  # a carton may stick out past the carton below it by at most 5 mm on any side
ON_TOL = 0.001  # resting on: the carton's lowest bottom corner within 1 mm of the top face below it (or the table)
UPRIGHT_DEG = 3.0  # upright: its 60 mm height axis within 3 degrees of vertical
LIFT_TOL = 0.005  # taken apart: a carton's whole underside ever more than 5 mm clear of the surface below it
RISE_TOL = 0.015  # ... or its lowest bottom corner ever more than 15 mm above the lowest it has been in the trial
# (picking up a fallen carton to restack it); rocking the stack 8 degrees raises a lowest corner by about 6 mm
BASE_TOL = 0.020  # the bottom carton's centre may never be more than 20 mm from its start, seen from above


def _moved(st, name) -> float:
    return float(np.linalg.norm(np.asarray(st.pos) - np.asarray(SPEC[name].pos)))


def _rot(st) -> np.ndarray:
    x, y, z, w = np.asarray(st.quat_xyzw, float) / np.linalg.norm(st.quat_xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tilt_deg(st) -> float:
    """Angle between the carton's 60 mm height axis and the vertical."""
    return float(np.degrees(np.arccos(np.clip(_rot(st)[2, 2], -1.0, 1.0))))


def _upright(st) -> bool:
    return tilt_deg(st) <= UPRIGHT_DEG + EPS


def _bottom_corners(st):
    hx, hy, hz = CARTON[0] / 2, CARTON[1] / 2, CARTON[2] / 2
    r = _rot(st)
    return [np.asarray(st.pos) + r @ np.array([sx * hx, sy * hy, -hz]) for sx in (-1, 1) for sy in (-1, 1)]


def gap(objects, i) -> float:
    """How far carton i's underside is clear of what it stands on (m): its lowest bottom corner's height above the
    plane of the top face of the carton below (for the bottom carton, above the table). Measured in the lower
    carton's own frame, so a stack rocking as a whole keeps every gap near zero."""
    corners = _bottom_corners(objects[NAMES[i]])
    if i == 0:
        return float(min(c[2] for c in corners) - TOP)
    lower = objects[NAMES[i - 1]]
    rl = _rot(lower)
    return float(min((rl.T @ (c - np.asarray(lower.pos)))[2] - CARTON[2] / 2 for c in corners))


def overhang(upper, lower) -> float:
    """How far any corner of the upper carton's footprint sticks out past a side of the lower carton's top face,
    measured square to that side (m); negative when it sits inside. Uses each carton's true turn about the
    vertical, so a turned carton is judged by its corners, not by its widened bounding box."""
    hx, hy = CARTON[0] / 2, CARTON[1] / 2
    ru, rl = _rot(upper), _rot(lower)
    corners = [np.asarray(upper.pos) + ru @ np.array([sx * hx, sy * hy, 0.0]) for sx in (-1, 1) for sy in (-1, 1)]
    worst = -1.0
    for c in corners:
        d = rl.T @ (c - np.asarray(lower.pos))
        worst = max(worst, abs(d[0]) - hx, abs(d[1]) - hy)
    return float(worst)


def _base_moved(objects) -> float:
    b = objects[NAMES[0]]
    return float(np.hypot(b.pos[0] - BASE_X, b.pos[1] - BASE_Y))


EPS = 1e-6  # every limit is inclusive: a value exactly at a stated limit passes

# Trial history: whether any carton has been lifted off what it stands on, or the bottom carton moved more than
# 20 mm, since the trial started. The scorer sees one step at a time, so it is kept here. It is cleared only by a
# scene reset: every carton back within 0.5 mm of its starting pose, upright and at rest, reached by a jump of more
# than 20 mm in a single step for some carton (the simulator's reset). Rebuilding the start by hand does not clear it.
RESET_TOL, RESET_JUMP = 0.0005, 0.020
_HISTORY = {"violated": False, "prev": None, "lowest": {}}


def _update(objects) -> None:
    prev = _HISTORY["prev"]
    _HISTORY["prev"] = {n: np.asarray(objects[n].pos, float).copy() for n in NAMES}
    at_start = all(_moved(objects[n], n) <= RESET_TOL and _upright(objects[n]) and objects[n].resting()
                   for n in NAMES)
    jumped = prev is None or any(np.linalg.norm(prev[n] - _HISTORY["prev"][n]) > RESET_JUMP for n in NAMES)
    if at_start and jumped:
        _HISTORY["violated"] = False
        _HISTORY["lowest"] = {}
    rose = False
    for n in NAMES:
        z = float(min(c[2] for c in _bottom_corners(objects[n])))
        low = min(_HISTORY["lowest"].get(n, z), z)
        _HISTORY["lowest"][n] = low
        rose = rose or z > low + RISE_TOL + EPS
    lifted = any(gap(objects, i) > LIFT_TOL + EPS for i in range(len(NAMES)))
    if lifted or rose or _base_moved(objects) > BASE_TOL + EPS:
        _HISTORY["violated"] = True


def _stacked(objects) -> bool:
    """Every carton upright and at rest, each standing on what is below it (lowest corner within 1 mm)."""
    if any(not (_upright(objects[n]) and objects[n].resting()) for n in NAMES):
        return False
    return all(abs(gap(objects, i)) <= ON_TOL + EPS for i in range(len(NAMES)))


def squared_joints(objects) -> int:
    return sum(overhang(objects[NAMES[i]], objects[NAMES[i - 1]]) <= OVERHANG_TOL + EPS for i in range(1, len(NAMES)))


def _tier(objects) -> float:
    _update(objects)
    if _HISTORY["violated"] or not _stacked(objects):
        return 0.0
    return {2: 1.0, 1: 0.5, 0: 0.0}[squared_joints(objects)]


def success(objects):
    """All three cartons stacked, upright and at rest, none overhanging the one below by more than 5 mm on any
    side; and at no time in the trial was any carton lifted more than 5 mm clear of what it stands on, or the bottom
    carton moved more than 20 mm."""
    return _tier(objects) == 1.0


def distance(objects):
    return float(sum(max(0.0, overhang(objects[NAMES[i]], objects[NAMES[i - 1]])) for i in range(1, len(NAMES))))


def score(objects):
    """1.0 as success; 0.5 with one of the two joints within 5 mm, the stack otherwise intact; 0 if any carton is
    off the stack, tipped, held above the carton below or still moving, and 0 for the whole trial once any carton
    was lifted more than 5 mm clear of what it stands on or the bottom carton moved more than 20 mm."""
    return _tier(objects)


# ---- scripted demo ------------------------------------------------------------------------------------------
# Left arm, closed fingers pointing down. Seen from the stack, the gripper is not flat: within the stack's width its
# front (+x) edge is 14-20 mm ahead of the TCP at the finger tips, 34 mm ahead from 50 to 85 mm above the TCP, and
# nothing reaches forward from 90 to 130 mm above it (measured from the arm's collision hulls). With the TCP 40 mm
# above the table, that 34 mm band spans 90 to 125 mm: the top 30 mm of the middle carton and the bottom 5 mm of
# the top one, while the tips stay short of the bottom carton. One slow push away from the robot carries both upper
# cartons to the same line, flush with the bottom carton's near face. Pushed higher, the whole stack rocks over
# instead of the middle carton sliding (seen in a trace with a light bottom carton).
TIP = 0.013
PUSH_Z = TOP + 0.040
BAND_AHEAD = 0.034  # the gripper's front edge ahead of the TCP at the pushing band
# Measured on the L40S with the trace wrapper: the stack rocks about 5 degrees away from the robot before the middle
# carton slides, and the arm stops about 30 mm short of its target; the demo aims 30 mm further. The cartons settle
# 3 mm short of flush.
PUSH_LAG = 0.030
APPROACH = 0.040


def _wp(t, x, y, z, label):
    return Waypoint(t, (float(x), float(y), float(z)), FINGER_CLOSED, label)


def demo():
    y = BASE_Y
    flush = START_X[NAMES[0]] - CARTON[0] / 2  # the bottom carton's near face
    top_face = START_X[NAMES[-1]] - CARTON[0] / 2
    x_start = top_face - BAND_AHEAD - APPROACH
    x_end = flush - BAND_AHEAD + PUSH_LAG
    return {"left": [
        _wp(2.0, x_start, y, TOP + 0.20, "high, in front of the stack"),
        _wp(2.0, x_start, y, PUSH_Z, "straight down"),
        _wp(5.0, x_end, y, PUSH_Z, "push both upper cartons flush"),
        _wp(1.0, x_end, y, PUSH_Z, "hold"),
        _wp(1.0, x_end - 0.03, y, PUSH_Z, "back off"),
        _wp(1.5, 0.15, y, TOP + 0.20, "clear of the stack"),
    ]}
