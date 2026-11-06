"""In the middle of the station table: a small free-standing light-grey chest of two
drawers, not fixed to the table, its front facing the robot. It stands on two foot rails across its width, one under
the back edge and one set back under the front. The lower drawer is closed. The top drawer is an open tray with a
blue handle fin on its front; it slides out toward the robot. A red tin lies at the back of the top drawer. On the
table to the robot's left is a flat green pad.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import parts  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

GREY, LIGHT = (0.62, 0.63, 0.65), (0.78, 0.79, 0.80)
BLUE, DARK_BLUE = (0.20, 0.38, 0.80), (0.10, 0.18, 0.55)
RED, GREEN = (0.85, 0.12, 0.12), (0.15, 0.65, 0.25)

# The chest's frame: origin on the table under the middle of the carcass; x away from the robot.
X0, Y0 = 0.31, 0.0
DEPTH, WIDTH, HEIGHT = 0.14, 0.16, 0.165
FRONT = -DEPTH / 2  # front face, part frame
FOOT_FRONT = -0.035  # front edge of the front foot rail (the edge the chest tips over)
W2 = WIDTH / 2
FIN_Y = -0.045  # the handle fin sits right of the drawer front's centre

CARCASS = [
    parts.box((0.02, WIDTH, 0.01), at=(FOOT_FRONT + 0.01, 0.0, 0.005), color=GREY),  # front foot rail
    parts.box((0.02, WIDTH, 0.01), at=(0.06, 0.0, 0.005), color=GREY),  # back foot rail
    parts.box((DEPTH, WIDTH, 0.085), at=(0.0, 0.0, 0.0525), color=LIGHT),  # lower part with its closed drawer
    parts.box((DEPTH, WIDTH, 0.01), at=(0.0, 0.0, 0.16), color=LIGHT),  # top panel, z 0.155-0.165
    parts.box((DEPTH, 0.005, 0.06), at=(0.0, W2 - 0.0025, 0.125), color=LIGHT),  # side walls of the drawer opening
    parts.box((DEPTH, 0.005, 0.06), at=(0.0, -W2 + 0.0025, 0.125), color=LIGHT),
    parts.box((0.005, WIDTH, 0.06), at=(DEPTH / 2 - 0.0025, 0.0, 0.125), color=LIGHT),  # back wall
    parts.box((0.002, 0.15, 0.075), at=(FRONT - 0.001, 0.0, 0.0525), color=GREY),  # lower drawer's front (closed)
]

TRAY_FLOOR_TOP = 0.103
DRAWER = parts.Slider(
    "drawer",
    shapes=[
        parts.box((0.008, 0.15, 0.056), at=(FRONT + 0.004, 0.0, 0.125), color=BLUE),  # front, x -0.070..-0.062
        parts.box((0.122, 0.14, 0.006), at=(-0.001, 0.0, 0.100), color=BLUE),  # floor, x -0.062..0.060
        parts.box((0.006, 0.14, 0.040), at=(0.057, 0.0, 0.123), color=BLUE),  # back, x 0.054..0.060, to z 0.143
        parts.box((0.116, 0.006, 0.040), at=(-0.004, 0.067, 0.123), color=BLUE),  # sides, x -0.062..0.054
        parts.box((0.116, 0.006, 0.040), at=(-0.004, -0.067, 0.123), color=BLUE),
        parts.box((0.05, 0.012, 0.03), at=(FRONT - 0.025, FIN_Y, 0.125), color=DARK_BLUE),  # handle fin, 5 cm out
    ],
    axis=(-1.0, 0.0, 0.0),  # toward the robot
    travel=0.13,
    mass=0.15,
    damping=0.5,
)
CHEST = parts.Part("chest", pos=(X0, Y0, TOP), shapes=CARCASS, joints=[DRAWER], mass=0.6)

TIN_SIZE = (0.05, 0.05, 0.04)
TIN_X = 0.054 - TIN_SIZE[0] / 2 - 0.001  # part frame: 1 mm in front of the drawer's back wall
TIN = Box("red_tin", pos=(X0 + TIN_X, Y0, TOP + TRAY_FLOOR_TOP + TIN_SIZE[2] / 2 + 0.0005), size=TIN_SIZE, mass=0.35,
          color=RED)
PAD_SIZE = (0.10, 0.10, 0.004)
PAD = Box("green_pad", pos=(0.22, 0.17, TOP + PAD_SIZE[2] / 2), size=PAD_SIZE, color=GREEN)
OBJECTS = [CHEST, TIN, PAD]

TIP_DEG = 10.0  # ever tilted more than this: the trial scores 0
UPRIGHT_DEG = 3.0  # standing on both feet at the end
MAX_SHIFT = 0.03  # the chest's footprint may move at most this far
CLOSED = 0.01  # drawer front within 1 cm of closed
ON = 0.001  # tin bottom within 1 mm of the surface it rests on
EPS = 1e-6

_history = {"worst_tilt": 0.0, "last": None}


def _tilt_deg(quat_xyzw) -> float:
    x, y, z, w = np.asarray(quat_xyzw, float) / np.linalg.norm(quat_xyzw)
    up_z = 1 - 2 * (x * x + y * y)  # world z of the chest's own up axis
    return math.degrees(math.acos(float(np.clip(up_z, -1.0, 1.0))))


def _at_start(objects) -> bool:
    c, t = objects["chest"], objects["red_tin"]
    return bool(
        np.linalg.norm(c.pos - np.array(CHEST.pos)) < 0.0005
        and np.linalg.norm(t.pos - np.array(TIN.pos)) < 0.0005
        and abs(c.joints.get("drawer", 0.0)) < 0.0005
        and _tilt_deg(c.quat_xyzw) < 0.1
        and c.resting()
        and t.resting()
    )


def _update(objects) -> float:
    """Worst chest tilt over the trial. Reset only by a scene reset: every object back within 0.5 mm of its start,
    at rest, after a jump of more than 20 mm in one step (or at the first step)."""
    now = (np.array(objects["chest"].pos), np.array(objects["red_tin"].pos))
    last = _history["last"]
    jump = last is None or max(np.linalg.norm(now[0] - last[0]), np.linalg.norm(now[1] - last[1])) > 0.02
    if jump and _at_start(objects):
        _history["worst_tilt"] = 0.0
    _history["last"] = now
    _history["worst_tilt"] = max(_history["worst_tilt"], _tilt_deg(objects["chest"].quat_xyzw))
    return _history["worst_tilt"]


def _chest_ok(objects) -> bool:
    c = objects["chest"]
    shift = np.hypot(c.pos[0] - CHEST.pos[0], c.pos[1] - CHEST.pos[1])
    return _tilt_deg(c.quat_xyzw) <= UPRIGHT_DEG + EPS and shift <= MAX_SHIFT + EPS


FLAT = math.radians(5.0)  # the tin lies on one of its faces: a face normal within 5 degrees of vertical


def _tin_flat(objects) -> bool:
    x, y, z, w = np.asarray(objects["red_tin"].quat_xyzw, float) / np.linalg.norm(objects["red_tin"].quat_xyzw)
    third_row = np.array([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)])  # world z of each axis
    return bool(np.max(np.abs(third_row)) >= math.cos(FLAT) - EPS)


def _tin_on_pad(objects) -> bool:
    t = objects["red_tin"]
    pad_top = PAD.pos[2] + PAD_SIZE[2] / 2
    over = abs(t.pos[0] - PAD.pos[0]) <= PAD_SIZE[0] / 2 + EPS and abs(t.pos[1] - PAD.pos[1]) <= PAD_SIZE[1] / 2 + EPS
    return bool(over and abs(t.low[2] - pad_top) <= ON + EPS and t.resting() and _tin_flat(objects))


def _grade(objects) -> float:
    worst = _update(objects)
    if worst > TIP_DEG + EPS or not _chest_ok(objects) or not objects["chest"].resting():
        return 0.0
    closed = objects["chest"].joints["drawer"] <= CLOSED + EPS
    return 1.0 if closed and _tin_on_pad(objects) else 0.0


def success(objects):
    """Tin resting on the pad, drawer closed, chest upright near its start and never tipped."""
    return _grade(objects) == 1.0


def score(objects):
    """1.0 as success; else 0 (pass/fail): 0 if the chest ever tilted more than 10 degrees, is not upright within 3 cm
    of its start, the drawer is open, or the tin is anywhere but resting on the pad."""
    return _grade(objects)


# -- scripted demo ----------------------------------------------------------------------------------------------


def _world(p):
    return (X0 + p[0], Y0 + p[1], TOP + p[2])


TIP = 0.013  # finger tips below the TCP
PULL = 0.135  # 5 mm past the drawer's full travel (the arm lags)


def demo():
    """Right arm grips the handle fin, pulls the drawer fully out and keeps holding it up while the left arm lifts the
    tin out and sets it on the pad; then the right arm pushes the drawer shut and lets go."""
    up = 0.185
    fin_x, fin_z = FRONT - 0.03, 0.125 + TIP
    tin_x, tin_z = TIN_X - PULL, TRAY_FLOOR_TOP + TIN_SIZE[2] / 2 + TIP
    place = (PAD.pos[0], PAD.pos[1], TOP + PAD_SIZE[2] + 0.007 + TIN_SIZE[2] / 2 + TIP)
    shut = fin_x + 0.002  # 2 mm past closed: the drawer stops at its closed limit
    right = [
        Waypoint(1.5, _world((fin_x, FIN_Y, up)), FINGER_OPEN, "over the handle"),
        Waypoint(1.0, _world((fin_x, FIN_Y, fin_z)), FINGER_OPEN, "down to the handle"),
        Waypoint(0.8, _world((fin_x, FIN_Y, fin_z)), FINGER_CLOSED, "grip the handle"),
        Waypoint(3.0, _world((fin_x - PULL, FIN_Y, fin_z)), FINGER_CLOSED, "pull the drawer out"),
        Waypoint(9.5, _world((fin_x - PULL, FIN_Y, fin_z)), FINGER_CLOSED, "hold the drawer up"),
        Waypoint(3.0, _world((shut, FIN_Y, fin_z)), FINGER_CLOSED, "push the drawer shut"),
        Waypoint(0.6, _world((shut, FIN_Y, fin_z)), FINGER_OPEN, "let go"),
        Waypoint(1.0, _world((fin_x - 0.03, FIN_Y, up)), FINGER_OPEN, "up and back"),
    ]
    left = [
        Waypoint(6.3, (0.20, 0.20, TOP + up), FINGER_OPEN, "wait aside"),
        Waypoint(1.2, _world((tin_x, 0.0, up)), FINGER_OPEN, "over the tin"),
        Waypoint(1.2, _world((tin_x, 0.0, tin_z)), FINGER_OPEN, "down to the tin"),
        Waypoint(0.8, _world((tin_x, 0.0, tin_z)), FINGER_CLOSED, "grip the tin"),
        Waypoint(1.2, _world((tin_x, 0.0, up)), FINGER_CLOSED, "lift it out"),
        Waypoint(2.0, (place[0], place[1], TOP + up), FINGER_CLOSED, "carry to the pad"),
        Waypoint(1.2, place, FINGER_CLOSED, "lower"),
        Waypoint(0.6, place, FINGER_OPEN, "release"),
        Waypoint(1.0, (place[0], place[1], TOP + up), FINGER_OPEN, "up"),
    ]
    return {"left": left, "right": right}
