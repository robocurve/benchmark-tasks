"""On the station table: a small white washing machine stands square in front of its space. Its cabinet is a box
11 x 11 x 10 cm, too wide to grip and heavy for its size, on rubber feet; a grey control console 3 cm deep and
3.5 cm tall runs across the top of its back edge. The space is marked on the table by blue tape: two side lines
running away from the robot and a far line, open on the side nearest the robot.
"""

import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

W, HB = 0.11, 0.10  # cabinet footprint (square) and height, m
CON_D, CON_H = 0.030, 0.035  # console depth (x) and height; it spans the full width
MASS = 3.0  # kg
FRICTION = (1.2, 1.0)
START = (0.18, 0.0)  # cabinet centre at the start
TAPE_W, TAPE_T = 0.010, 0.001
SPACE_X0, SPACE_X1 = 0.165, 0.335  # the space: from the side lines' near ends to the far line's inner edge
SPACE_HALF_Y = 0.085  # inner edges of the side lines
TILT_LIMIT = np.radians(20.0)
UPRIGHT = np.radians(5.0)
LIFT_LIMIT = 0.002  # lifted: its lowest corner more than this above the table (the walk keeps it under 0.1 mm)
MOVED = 0.030
EPS = 1e-6
BLUE = (0.10, 0.30, 0.85)
CON_X = W / 2 - CON_D / 2  # console centre, forward of the cabinet centre

WASHER = parts.Part(
    "washer", pos=(START[0], START[1], TOP + HB / 2), mass=MASS, friction=FRICTION,
    shapes=[parts.box((W, W, HB), at=(0.0, 0.0, 0.0), color=(0.93, 0.93, 0.92)),
            parts.box((CON_D, W, CON_H), at=(CON_X, 0.0, HB / 2 + CON_H / 2), color=(0.45, 0.47, 0.50))],
)
_SIDE_LEN = SPACE_X1 + TAPE_W - SPACE_X0
_TAPE_Z = TOP + TAPE_T / 2
TAPE_L = Box("tape_left", pos=(SPACE_X0 + _SIDE_LEN / 2, SPACE_HALF_Y + TAPE_W / 2, _TAPE_Z),
             size=(_SIDE_LEN, TAPE_W, TAPE_T), color=BLUE)
TAPE_R = Box("tape_right", pos=(SPACE_X0 + _SIDE_LEN / 2, -SPACE_HALF_Y - TAPE_W / 2, _TAPE_Z),
             size=(_SIDE_LEN, TAPE_W, TAPE_T), color=BLUE)
TAPE_FAR = Box("tape_far", pos=(SPACE_X1 + TAPE_W / 2, 0.0, _TAPE_Z),
               size=(TAPE_W, 2 * SPACE_HALF_Y + 2 * TAPE_W, TAPE_T), color=BLUE)
OBJECTS = [WASHER, TAPE_L, TAPE_R, TAPE_FAR]


def _rot(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tilt(state) -> float:
    """Angle between the washer's own vertical and straight up (rad)."""
    return float(np.arccos(np.clip(_rot(state.quat_xyzw)[2, 2], -1.0, 1.0)))


def corners(state) -> np.ndarray:
    """The four bottom corners of the cabinet (world xyz). state.pos is the cabinet's centre."""
    r = _rot(state.quat_xyzw)
    c = np.array([[sx * W / 2, sy * W / 2, -HB / 2] for sx in (-1, 1) for sy in (-1, 1)])
    return np.asarray(state.pos) + c @ r.T


# Whole-trial record (largest tilt, largest lift), cleared only by a scene reset: the washer back within 0.5 mm of
# its start, upright and at rest, after a jump of more than 20 mm in one control step (the robot cannot move it
# that fast).
_REC = {"max_tilt": 0.0, "max_lift": 0.0, "prev": None}
_START = np.array([START[0], START[1], TOP + HB / 2])


def _update(objects):
    s = objects["washer"]
    pos = np.asarray(s.pos, float)
    prev = _REC["prev"]
    if (prev is not None and np.linalg.norm(pos - prev) > 0.020 and np.linalg.norm(pos - _START) < 0.0005
            and tilt(s) < 1e-3 and s.resting()):
        _REC["max_tilt"] = _REC["max_lift"] = 0.0
    _REC["prev"] = pos
    _REC["max_tilt"] = max(_REC["max_tilt"], tilt(s))
    _REC["max_lift"] = max(_REC["max_lift"], float(corners(s)[:, 2].min() - TOP))


def _clean(objects) -> bool:
    """Never tilted past 20 degrees and never lifted clear of the table during the trial."""
    _update(objects)
    return _REC["max_tilt"] <= TILT_LIMIT + EPS and _REC["max_lift"] <= LIFT_LIMIT + EPS


def _standing(s) -> bool:
    """Upright within 5 degrees, on the table (every bottom corner within 1.5 mm of it) and still."""
    return tilt(s) <= UPRIGHT + EPS and corners(s)[:, 2].max() <= TOP + 0.0015 + EPS and s.resting()


def _in_space(s) -> bool:
    f = corners(s)[:, :2]
    return bool(np.all(f[:, 0] >= SPACE_X0 - EPS) and np.all(f[:, 0] <= SPACE_X1 + EPS)
                and np.all(np.abs(f[:, 1]) <= SPACE_HALF_Y + EPS))


def success(objects):
    """In its space: standing upright and still, its whole footprint inside the taped lines, never tipped or lifted."""
    clean = _clean(objects)
    s = objects["washer"]
    return bool(clean and _standing(s) and _in_space(s))


def distance(objects):
    return float(max(0.0, SPACE_X0 + W / 2 - objects["washer"].pos[0]))


def score(objects):
    """1.0 when in its space (see success); 0.5 if standing upright and still, never tipped or lifted, and its centre
    at least 30 mm further into the space than at the start but not fully inside; 0 otherwise."""
    if success(objects):
        return 1.0
    s = objects["washer"]
    if not _clean(objects) or not _standing(s):
        return 0.0
    return 0.5 if s.pos[0] - START[0] >= MOVED - EPS else 0.0


# ---- demo: fixed waypoints only --------------------------------------------------------------------------------
# The right arm grips the console. Each step tips the washer a few degrees onto one far corner, turns it about that
# corner, and sets it back on all four feet. Four steps, alternating between the two far corners. All targets come from the start layout.
GRIP_Z = TOP + HB + 0.010 + 0.013  # pad tips 10 mm down the console
ABOVE_Z = TOP + HB + CON_H + 0.025
TILT_MOVE = 0.025  # grip moved this far toward the loaded corner (more than the tilt needs: the arm is soft)
SWING = np.radians(26.0)  # commanded turn per step
SWING_GOT = 0.9  # measured on the GPU: the washer turns about 90% of the commanded angle (its corner slips a little)
BASE_YAW = np.pi / 2  # fingers close along x, across the console


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


def _corner(c, yaw, sx, sy):
    return np.asarray(c) + _rz(yaw) @ np.array([sx * W / 2, sy * W / 2])


def _walk():
    c, yaw = np.array(START, float), 0.0
    grip = c + np.array([CON_X, 0.0])
    w = [Waypoint(1.5, (*grip, ABOVE_Z), FINGER_OPEN, "over the console", BASE_YAW),
         Waypoint(1.5, (*grip, GRIP_Z), FINGER_OPEN, "down", BASE_YAW),
         Waypoint(0.8, (*grip, GRIP_Z), FINGER_CLOSED, "grip", BASE_YAW)]
    for sy in (1.0, -1.0, 1.0, -1.0):  # far-left corner, then far-right, twice
        pivot = _corner(c, yaw, 1.0, sy)
        u = (pivot - c) / np.linalg.norm(pivot - c)
        s = sy  # turn: +1 counter-clockwise about the far-left corner, -1 clockwise about the far-right
        w.append(Waypoint(1.5, (*(grip + u * TILT_MOVE), GRIP_Z + 0.008), FINGER_CLOSED, "tip onto the corner",
                          BASE_YAW + yaw))
        n = 4
        for i in range(1, n + 1):
            a = s * SWING * i / n
            g = pivot + _rz(a) @ (grip + u * TILT_MOVE - pivot)
            w.append(Waypoint(0.8, (*g, GRIP_Z + 0.008), FINGER_CLOSED, "turn about the corner", BASE_YAW + yaw + a))
        g_set, y_set = pivot + _rz(s * SWING) @ (grip - pivot), BASE_YAW + yaw + s * SWING
        w.append(Waypoint(1.5, (*g_set, GRIP_Z), FINGER_CLOSED, "set it down", y_set))
        got = s * SWING * SWING_GOT  # where the next step expects it
        c = pivot + _rz(got) @ (c - pivot)
        grip = pivot + _rz(got) @ (grip - pivot)
        yaw += got
        w.append(Waypoint(0.8, (*g_set, GRIP_Z), FINGER_CLOSED, "settle", y_set))
    w += [Waypoint(0.8, (*g_set, GRIP_Z), FINGER_OPEN, "release", y_set),
          Waypoint(1.2, (*(g_set - np.array([0.03, 0.0])), GRIP_Z + 0.02), FINGER_OPEN, "up and back", y_set),
          Waypoint(1.5, (0.15, -0.20, ABOVE_Z), FINGER_OPEN, "away", 0.0),
          Waypoint(2.5, (0.15, -0.20, ABOVE_Z), FINGER_OPEN, "wait", 0.0)]
    return w


def _push(z, depth, face_x=START[0] - W / 2):
    """A plain straight push at TCP height z on the face at face_x (default the cabinet's near face), commanded depth
    past its surface."""
    x0 = face_x - 0.012
    return [Waypoint(1.5, (x0 - 0.04, 0.0, ABOVE_Z), FINGER_CLOSED, "in front of the washer"),
            Waypoint(1.0, (x0 - 0.02, 0.0, z), FINGER_CLOSED, "down"),
            Waypoint(3.0, (x0 + depth, 0.0, z), FINGER_CLOSED, "push straight on"),
            Waypoint(2.0, (x0 + depth, 0.0, z), FINGER_CLOSED, "hold"),
            Waypoint(1.0, (x0 - 0.04, 0.0, z), FINGER_CLOSED, "back"),
            Waypoint(2.0, (0.15, -0.20, ABOVE_Z), FINGER_CLOSED, "away")]


def demo():
    """The walk. To replay the failure cases, WASHER_DEMO=push plays a straight push at mid-height on the
    cabinet's front and WASHER_DEMO=high a straight push on the console's front; WASHER_DEMO=drag grips the console and drags it
    straight toward the space without tilting it."""
    mode = os.environ.get("WASHER_DEMO", "")
    if mode == "drag":  # grip as in the walk, then one straight level move 80 mm toward the space, no tilt
        g = np.array(START) + np.array([CON_X, 0.0])
        return {"right": [Waypoint(1.5, (*g, ABOVE_Z), FINGER_OPEN, "over the console", BASE_YAW),
                          Waypoint(1.5, (*g, GRIP_Z), FINGER_OPEN, "down", BASE_YAW),
                          Waypoint(0.8, (*g, GRIP_Z), FINGER_CLOSED, "grip", BASE_YAW),
                          Waypoint(4.0, (g[0] + 0.08, g[1], GRIP_Z), FINGER_CLOSED, "drag straight", BASE_YAW),
                          Waypoint(2.0, (g[0] + 0.08, g[1], GRIP_Z), FINGER_CLOSED, "hold", BASE_YAW),
                          Waypoint(0.8, (g[0] + 0.08, g[1], GRIP_Z), FINGER_OPEN, "release", BASE_YAW),
                          Waypoint(1.5, (0.15, -0.20, ABOVE_Z), FINGER_OPEN, "away", 0.0)]}
    if mode == "low":  # both arms push low on the front face, pad tips 5 mm above the table, 14 cm past the face
        x0 = START[0] - W / 2 - 0.012
        z = TOP + 0.005 + 0.013
        plan = {}
        for arm, y in (("left", 0.03), ("right", -0.03)):
            plan[arm] = [Waypoint(1.5, (x0 - 0.05, y, z + 0.03), FINGER_CLOSED, "in front, low", 0.0),
                         Waypoint(1.0, (x0 - 0.02, y, z), FINGER_CLOSED, "down", 0.0),
                         Waypoint(4.0, (x0 + 0.14, y, z), FINGER_CLOSED, "push hard and low", 0.0),
                         Waypoint(2.0, (x0 + 0.14, y, z), FINGER_CLOSED, "hold", 0.0),
                         Waypoint(1.5, (x0 - 0.05, y, z + 0.03), FINGER_CLOSED, "back", 0.0)]
        return plan
    if mode == "truck":  # grip the console, tip it back about 15 degrees onto its near edge, drag it 9 cm, set down
        g = np.array(START) + np.array([CON_X, 0.0])
        back = g + np.array([-0.034, 0.0])
        return {"right": [Waypoint(1.5, (*g, ABOVE_Z), FINGER_OPEN, "over the console", BASE_YAW),
                          Waypoint(1.5, (*g, GRIP_Z), FINGER_OPEN, "down", BASE_YAW),
                          Waypoint(0.8, (*g, GRIP_Z), FINGER_CLOSED, "grip", BASE_YAW),
                          Waypoint(2.0, (*back, GRIP_Z + 0.02), FINGER_CLOSED, "tip back onto the near edge", BASE_YAW),
                          Waypoint(4.0, (back[0] + 0.09, back[1], GRIP_Z + 0.02), FINGER_CLOSED, "drag on its edge",
                                   BASE_YAW),
                          Waypoint(2.0, (g[0] + 0.09, g[1], GRIP_Z), FINGER_CLOSED, "stand it up", BASE_YAW),
                          Waypoint(0.8, (g[0] + 0.09, g[1], GRIP_Z), FINGER_OPEN, "release", BASE_YAW),
                          Waypoint(1.5, (0.15, -0.20, ABOVE_Z), FINGER_OPEN, "away", 0.0)]}
    if mode == "push":
        return {"right": _push(TOP + HB / 2 + 0.013, 0.06)}
    if mode == "high":  # on the console's front face, pad tips 15 mm above the cabinet top
        return {"right": _push(TOP + HB + 0.015 + 0.013, 0.14, START[0] + CON_X - CON_D / 2)}
    return {"right": _walk()}
