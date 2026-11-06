"""On the station table, in front of the robot: a fixed grey stop plate with a pivot post. An orange arm with a white
knob near its end turns on the post about a vertical axis; it starts pointing toward the robot's right and swings
only counter-clockwise seen from above, through the front and round toward the robot's left. Four square sockets in
the plate lie on a circle around the post, under the arm's swing. A blue work pad and a yellow walkway strip are
marked on the table beyond the plate. A loose steel stop pin stands upright on the table on the robot's left. A pin
standing in a socket is struck by the arm's side as it swings.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import parts  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m). The pivot axis is vertical through P.
PX, PY = 0.200, 0.040

GREY, DARK = (0.55, 0.56, 0.58), (0.30, 0.31, 0.33)
ORANGE, WHITE = (1.00, 0.50, 0.00), (0.95, 0.95, 0.95)
BLUE, YELLOW, STEEL = (0.15, 0.30, 0.85), (0.95, 0.80, 0.10), (0.62, 0.64, 0.67)

PLATE_T = 0.010  # stop plate thickness: the socket floors are its top
ARM_W, ARM_H, ARM_LEN, ARM_BACK = 0.030, 0.020, 0.200, 0.015  # arm: width, thickness, pivot to end, tail
ARM_Z0 = 0.050  # arm underside above the table
KNOB_R, KNOB_H, KNOB_S = 0.010, 0.025, 0.170  # white knob on top of the arm, KNOB_S from the pivot

SOCKET_R = 0.080  # socket centres on this circle round the pivot
SOCKET_DEG = (30, 70, 115, 160)  # measured from the arm's start direction, counter-clockwise seen from above
RIGHT = 115  # the socket that limits the arm correctly
HOLE, WALL, SOCKET_H = 0.028, 0.005, 0.035  # square hole width, wall thickness, wall height above the plate

PIN_R, PIN_H, PIN_MASS = 0.010, 0.075, 0.18  # steel pin, 20 mm x 75 mm
PIN_START = (0.170, 0.200)

PAD_DEG, PAD_SIZE = 68.0, 0.060  # blue pad centred under the knob's path at this angle
WALK_X, WALK_Y = (0.110, 0.330), (0.075, 0.125)  # yellow walkway, relative to the pivot


def direction(deg: float) -> np.ndarray:
    """Unit vector along the arm at swing angle ``deg`` (0: toward the robot's right)."""
    a = math.radians(deg)
    return np.array([math.sin(a), -math.cos(a)])


def socket_xy(deg: float) -> np.ndarray:
    """A socket centre relative to the pivot (m)."""
    return SOCKET_R * direction(deg)


def _socket_shapes(deg):
    cx, cy = socket_xy(deg)
    z = PLATE_T + SOCKET_H / 2
    out_w = HOLE + 2 * WALL
    off = HOLE / 2 + WALL / 2
    return [
        parts.box((out_w, WALL, SOCKET_H), at=(cx, cy + off, z), color=DARK),
        parts.box((out_w, WALL, SOCKET_H), at=(cx, cy - off, z), color=DARK),
        parts.box((WALL, HOLE, SOCKET_H), at=(cx + off, cy, z), color=DARK),
        parts.box((WALL, HOLE, SOCKET_H), at=(cx - off, cy, z), color=DARK),
    ]


PLATE_X, PLATE_Y = (-0.030, 0.105), (-0.090, 0.105)  # plate extent relative to the pivot
ARM_MID_Z = ARM_Z0 + ARM_H / 2
SWING = parts.Hinge(
    "swing",
    shapes=[
        parts.box((ARM_W, ARM_LEN + ARM_BACK, ARM_H), at=(0.0, -(ARM_LEN - ARM_BACK) / 2, ARM_MID_Z), color=ORANGE),
        parts.cylinder(KNOB_R, KNOB_H, at=(0.0, -KNOB_S, ARM_Z0 + ARM_H + KNOB_H / 2), color=WHITE, axis="z"),
    ],
    pivot=(0.0, 0.0, ARM_MID_Z),
    axis=(0.0, 0.0, 1.0),  # positive: counter-clockwise seen from above
    lower=0.0,
    upper=math.radians(160.0),
    mass=0.15,
    spring=0.0,
    damping=0.05,
)
STATION = parts.Part(
    "stop_plate",
    pos=(PX, PY, TOP),
    shapes=[
        parts.box((PLATE_X[1] - PLATE_X[0], PLATE_Y[1] - PLATE_Y[0], PLATE_T),
                  at=(sum(PLATE_X) / 2, sum(PLATE_Y) / 2, PLATE_T / 2), color=GREY),
        parts.cylinder(0.020, ARM_Z0 - PLATE_T, at=(0.0, 0.0, (ARM_Z0 + PLATE_T) / 2), color=DARK, axis="z"),
        *(s for d in SOCKET_DEG for s in _socket_shapes(d)),
    ],
    joints=[SWING],
)

_pad_c = np.array((PX, PY)) + KNOB_S * direction(PAD_DEG)
PAD = Box("work_pad", pos=(*_pad_c, TOP + 0.0005), size=(PAD_SIZE, PAD_SIZE, 0.001), color=BLUE)
WALKWAY = Box("walkway", pos=(PX + sum(WALK_X) / 2, PY + sum(WALK_Y) / 2, TOP + 0.0005),
              size=(WALK_X[1] - WALK_X[0], WALK_Y[1] - WALK_Y[0], 0.001), color=YELLOW)
PIN = Cylinder("stop_pin", pos=(*PIN_START, TOP + PIN_H / 2), radius=PIN_R, height=PIN_H, mass=PIN_MASS, color=STEEL)

OBJECTS = [STATION, PAD, WALKWAY, PIN]

FLOOR_Z = TOP + PLATE_T
EPS = 1e-6
SUCCESS_HOLD_S = 3.0  # long enough that a trial ends only after the pushing gripper has let go of the arm
FOOT_TOL = 0.001  # the pin's lowest point within 1 mm of the socket floor
MAX_TILT = 20.0  # deg from upright


def _tilt_deg(q) -> float:
    x, y, _, _ = np.asarray(q, float) / np.linalg.norm(q)
    return math.degrees(math.acos(float(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def pin_socket(objects):
    """The socket angle the pin stands in, or None. Standing in a socket: its foot centre inside the square hole,
    its lowest point within 1 mm of the socket floor, within 20 deg of upright, at rest."""
    pin = objects["stop_pin"]
    if not pin.resting():
        return None
    if not (FLOOR_Z - 0.003 <= pin.low[2] <= FLOOR_Z + FOOT_TOL + EPS):
        return None
    tilt = _tilt_deg(pin.quat_xyzw)
    if tilt > MAX_TILT + EPS:
        return None
    x, y, z, w = np.asarray(pin.quat_xyzw, float) / np.linalg.norm(pin.quat_xyzw)
    axis = np.array([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)])
    foot = np.asarray(pin.pos, float) - PIN_H / 2 * axis
    for d in SOCKET_DEG:
        c = np.array((PX, PY)) + socket_xy(d)
        if np.all(np.abs(foot[:2] - c) <= HOLE / 2 + EPS):
            return d
    return None


def arm_deg(objects) -> float:
    return math.degrees(objects["stop_plate"].joints["swing"])


def knob_on_pad(objects) -> bool:
    """The knob's centre, seen from above, is over the blue pad."""
    k = np.array((PX, PY)) + KNOB_S * direction(arm_deg(objects))
    return bool(np.all(np.abs(k - _pad_c) <= PAD_SIZE / 2 + EPS))


def _arm_on_start_side(objects, socket_deg) -> bool:
    """The arm is between its start position and the pin: it has not been swung past the pin's socket angle."""
    return arm_deg(objects) <= socket_deg + EPS


def success(objects):
    """The pin stands in the socket that lets the arm reach the pad but not the walkway, and the knob is over the
    pad (the arm is on the start side of the pin)."""
    return bool(pin_socket(objects) == RIGHT and _arm_on_start_side(objects, RIGHT) and knob_on_pad(objects))


def distance(objects):
    p = objects["stop_pin"].pos
    c = np.array((PX, PY)) + socket_xy(RIGHT)
    return float(np.linalg.norm(p[:2] - c))


def score(objects):
    """1.0 the success condition; 0.5 the pin in the right socket with the arm not yet on the pad (and not past the
    pin); else 0."""
    s = pin_socket(objects)
    if s == RIGHT and _arm_on_start_side(objects, RIGHT):
        return 1.0 if knob_on_pad(objects) else 0.5
    return 0.0


# -- demo --------------------------------------------------------------------------------------------------------

TIP = 0.013  # finger tips below the TCP
TRAVEL_Z = TOP + 0.14
PIN_GRIP_Z = TOP + PIN_H - 0.022 + TIP  # finger tips 22 mm below the pin's top
DROP = 0.006  # the pin's foot is released this far above the socket floor
PUSH_S = 0.110  # the arm's side is pushed this far from the pivot on the way round
BACK_S = 0.130  # and this far on the way back, clear of the pin
PUSH_OFF = 0.019  # TCP from the arm's centre line while pushing (m)
PUSH_Z = TOP + ARM_MID_Z + TIP  # finger tips at the arm's mid-height
PUSH_END = 106.0  # deg commanded: past where the pin stops the arm (about 97 deg), so the soft arm presses on it
# Measured in a GPU trace: lowering the pin at the 115 deg socket, the left arm ended 5.8 mm short in x and 4.4 mm to
# the left of its target, enough to set the pin on a socket wall. The demo aims off by that much.
PLACE_SAG = (0.0058, -0.0044)
BACK_END = 66.0  # deg: the arm is pushed back to here, over the pad


def _push_point(deg: float, off: float = PUSH_OFF, s: float = PUSH_S):
    """TCP (x, y) beside the arm at angle ``deg``, ``s`` from the pivot: behind its trailing side for off > 0, in
    front of its leading side for off < 0."""
    a = math.radians(deg)
    d = direction(deg)
    t = np.array([math.cos(a), math.sin(a)])  # the arm's direction of travel
    p = np.array((PX, PY)) + s * d - off * t
    return float(p[0]), float(p[1])


def _yaw(deg: float) -> float:
    """Grip yaw that follows the arm up to 60 deg: beyond that the wrist would pass its joint limit."""
    return math.radians(min(deg, 60.0))


def demo():
    """Left arm only: picks the pin from above, carries it to the 115 deg socket and drops it from 6 mm. Then, with
    closed fingers behind the orange arm's trailing side, pushes it round until the pin stops it (proving the stop),
    lifts over, and pushes it back from its leading side onto the pad."""
    sx, sy = PIN_START
    hx, hy = np.array((PX, PY)) + socket_xy(RIGHT) + np.array(PLACE_SAG)
    place_z = PIN_GRIP_Z + PLATE_T + DROP
    clear = PUSH_OFF + 0.03
    fwd = [12.0, 24.0, 36.0, 48.0, 60.0, 72.0, 84.0, 96.0, PUSH_END]
    back = [92.0, 84.0, 76.0, BACK_END]
    left = [
        Waypoint(1.5, (sx, sy, TRAVEL_Z), FINGER_OPEN, "over the pin"),
        Waypoint(1.5, (sx, sy, PIN_GRIP_Z), FINGER_OPEN, "descend"),
        Waypoint(0.8, (sx, sy, PIN_GRIP_Z), FINGER_CLOSED, "grip"),
        Waypoint(1.2, (sx, sy, TRAVEL_Z), FINGER_CLOSED, "lift"),
        Waypoint(2.0, (hx, hy, TRAVEL_Z), FINGER_CLOSED, "carry"),
        Waypoint(1.5, (hx, hy, place_z), FINGER_CLOSED, "lower into the socket"),
        Waypoint(0.6, (hx, hy, place_z), FINGER_OPEN, "release"),
        Waypoint(1.0, (hx, hy, TRAVEL_Z), FINGER_OPEN, "up"),
        Waypoint(1.8, (*_push_point(0.0, clear), TRAVEL_Z), FINGER_CLOSED, "over the arm's trailing side"),
        Waypoint(1.5, (*_push_point(0.0, clear), PUSH_Z), FINGER_CLOSED, "down behind the arm"),
        Waypoint(0.8, (*_push_point(0.0), PUSH_Z), FINGER_CLOSED, "to the arm"),
        *(Waypoint(0.5, (*_push_point(d), PUSH_Z), FINGER_CLOSED, f"push to {d:.0f}", _yaw(d))
          for d in fwd),
        Waypoint(1.0, (*_push_point(PUSH_END), PUSH_Z), FINGER_CLOSED, "press on the stop", _yaw(PUSH_END)),
        Waypoint(0.8, (*_push_point(85.0, clear), PUSH_Z), FINGER_CLOSED, "back off", _yaw(85.0)),
        Waypoint(1.0, (*_push_point(85.0, clear), TRAVEL_Z), FINGER_CLOSED, "up", _yaw(85.0)),
        Waypoint(1.5, (*_push_point(104.0, -clear, BACK_S), TRAVEL_Z), FINGER_CLOSED, "over the leading side",
                 _yaw(104.0)),
        Waypoint(1.5, (*_push_point(104.0, -clear, BACK_S), PUSH_Z), FINGER_CLOSED, "down in front of the arm",
                 _yaw(104.0)),
        Waypoint(0.8, (*_push_point(100.0, -PUSH_OFF, BACK_S), PUSH_Z), FINGER_CLOSED, "to the arm",
                 _yaw(100.0)),
        *(Waypoint(0.6, (*_push_point(d, -PUSH_OFF, BACK_S), PUSH_Z), FINGER_CLOSED, f"back to {d:.0f}",
                   _yaw(d)) for d in back),
        Waypoint(0.8, (*_push_point(BACK_END + 6.0, -clear, BACK_S), PUSH_Z), FINGER_CLOSED, "back off",
                 _yaw(BACK_END)),
        Waypoint(1.0, (*_push_point(BACK_END + 6.0, -clear, BACK_S), TRAVEL_Z), FINGER_CLOSED, "up",
                 _yaw(BACK_END)),
        Waypoint(1.5, (0.20, 0.25, TRAVEL_Z), FINGER_OPEN, "park"),
    ]
    return {"left": left}
