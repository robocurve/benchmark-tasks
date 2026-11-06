"""On the station table, a scale model of a room corner, in front of the left arm. A light grey wall runs across the
table, left to right, with a doorway in it. A red fire door is hung on a black hinge post at the doorway's left
end. It stands fully open, swung toward the robot, lying alongside a low side wall to its left. The door has an
closer inside the hinge. On the side wall, a black hook is hinged on a small bracket. Its arm lies across the
top edge of the open door, and its short end drops down 5 mm in front of the door's right-hand face. A small
black knob stands on top of the hook's arm. A heavy grey door-stop block stands on the floor against the door's
right-hand face.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# World frame: x away from the arm bases, y to the robot's left, z up (m). The room part's frame is at the hinge
# axis on the table top, unturned, so part-frame x/y are world offsets from the hinge.
HINGE = (0.400, 0.080)  # door hinge axis, in the wall's centre plane
WALL_T, WALL_H = 0.010, 0.120
DOOR = (0.150, 0.010, 0.120)  # width (along x when open), thickness, height
DOOR_Z0 = 0.005  # gap under the door
DOOR_NEAR = 0.010  # hinge-side edge of the leaf, from the hinge axis
DOOR_W_FROM_HINGE = DOOR_NEAR + DOOR[0]  # 0.160: free edge from the hinge axis
DOOR_TOP = DOOR_Z0 + DOOR[2]  # 0.125
POST_R = 0.008
SIDE_FACE = 0.020  # side wall's face, +y of the hinge axis (world y 0.100)
SIDE = (0.150, 0.010, 0.060)  # side wall: x length, thickness, height (low, so the arm can work over it)
SIDE_X0 = -0.155  # its near end, from the hinge (world x 0.245)

# Hook: hinged about an axis along x on a bracket at the side wall. Engaged, its arm rests on the door's top edge
# and its end drops 8 mm below the door's top, 5 mm off the door's closing (-y) face (2 mm or more of clearance
# from the door's top corner as it lifts). The pivot is about 6 mm below
# the centre of that contact, so the closer's push on the hook end turns the hook down, not up: the door cannot push
# it off.
HOOK_X = -0.145  # along the door, from the hinge (world x 0.255)
HOOK_PIVOT = (HOOK_X, 0.016, DOOR_TOP - 0.010)  # world y 0.096, 115 mm up
HOOK_ARM_Y0 = -0.016  # arm's outer end (world y 0.064)
HOOK_ARM = (0.008, 0.016 - HOOK_ARM_Y0, 0.006)  # x, y, z; world y 0.064..0.096, z 0.126..0.132
HOOK_END = (0.008, 0.006, 0.010)  # world y 0.064..0.070, z 0.117..0.127
HOOK_UPRIGHT = (0.008, 0.004, 0.017)  # from the pivot up to the arm
KNOB_R, KNOB_H = 0.006, 0.018
KNOB_Y = -0.002  # part frame (world y 0.078), on top of the arm, over the door's top edge
HOOK_MASS = 0.03

STOP = (0.040, 0.060, 0.030)  # door-stop block
STOP_MASS = 0.50
STOP_XY = (HINGE[0] - 0.085, HINGE[1] - DOOR[1] / 2 - STOP[1] / 2)  # (0.315, 0.045), touching the door face

CLOSER, PRELOAD = 0.10, 0.30  # N m/rad, rad past shut
DOOR_DAMP = 0.15
SHUT_Q = -math.pi / 2  # the door joint: 0 open (as written), -90 degrees shut against the frame

RED, GREY, DARK, BLACK = (0.75, 0.12, 0.10), (0.78, 0.78, 0.76), (0.12, 0.12, 0.13), (0.05, 0.05, 0.05)
STOP_GREY = (0.40, 0.41, 0.42)

_hz = HOOK_PIVOT[2]
_arm_z = DOOR_TOP + 0.001 + HOOK_ARM[2] / 2  # arm bottom 1 mm above the door top: it settles onto it
HOOK = parts.Hinge(
    "hook",
    shapes=[
        parts.box(HOOK_ARM, at=(HOOK_X, HOOK_ARM_Y0 + HOOK_ARM[1] / 2, _arm_z), color=BLACK),
        parts.box(HOOK_END, at=(HOOK_X, HOOK_ARM_Y0 + HOOK_END[1] / 2, DOOR_TOP + 0.002 - HOOK_END[2] / 2),
                  color=BLACK),
        parts.box(HOOK_UPRIGHT, at=(HOOK_X, 0.016 - HOOK_UPRIGHT[1] / 2, _hz + HOOK_UPRIGHT[2] / 2), color=BLACK),
        parts.cylinder(KNOB_R, KNOB_H, at=(HOOK_X, KNOB_Y, _arm_z + HOOK_ARM[2] / 2 + KNOB_H / 2), color=BLACK,
                       axis="z"),
    ],
    pivot=HOOK_PIVOT,
    axis=(-1.0, 0.0, 0.0),  # positive lifts the hook end
    lower=-math.pi / 2,  # let go off the door, it hangs straight down beside the side wall
    upper=math.radians(92.0),
    mass=HOOK_MASS,
    damping=0.005,
)
DOOR_LEAF = parts.Hinge(
    "door",
    shapes=[
        parts.box(DOOR, at=(-(DOOR_NEAR + DOOR[0] / 2), 0.0, DOOR_Z0 + DOOR[2] / 2), color=RED),
    ],
    pivot=(0.0, 0.0, DOOR_Z0 + DOOR[2] / 2),
    axis=(0.0, 0.0, -1.0),  # positive opens further; shut is -90 degrees
    lower=SHUT_Q,
    upper=0.05,
    mass=0.30,
    spring=CLOSER,
    damping=DOOR_DAMP,
)
_wall_lo, _wall_hi = -0.330, 0.170  # wall extent in y from the hinge (world -0.25 .. +0.25)
_gap_lo = -DOOR_W_FROM_HINGE - 0.005  # doorway's latch side (world y -0.085)
ROOM = parts.Part(
    "room",
    pos=(HINGE[0], HINGE[1], TOP),
    shapes=[
        parts.cylinder(POST_R, WALL_H + 0.010, at=(0.0, 0.0, (WALL_H + 0.010) / 2), color=BLACK, axis="z"),
        # door wall, hinge side: from the post to the left end
        parts.box((WALL_T, _wall_hi - POST_R, WALL_H), at=(0.0, (POST_R + _wall_hi) / 2, WALL_H / 2), color=GREY),
        # door wall, latch side, with the frame stop the door shuts against
        parts.box((WALL_T, _gap_lo - _wall_lo, WALL_H), at=(0.0, (_wall_lo + _gap_lo) / 2, WALL_H / 2), color=GREY),
        # low side wall along the open door
        parts.box(SIDE, at=(SIDE_X0 + SIDE[0] / 2, SIDE_FACE + SIDE[1] / 2, SIDE[2] / 2), color=GREY),
        # hook bracket on the side wall's top
        parts.box((0.016, SIDE[1] + SIDE_FACE - HOOK_PIVOT[1], _hz + 0.004 - SIDE[2]),
                  at=(HOOK_X, (HOOK_PIVOT[1] + SIDE_FACE + SIDE[1]) / 2, (SIDE[2] + _hz + 0.004) / 2),
                  color=DARK),
    ],
    joints=[DOOR_LEAF, HOOK],
)
DOOR_STOP = Box("door_stop", pos=(*STOP_XY, TOP + STOP[2] / 2), size=STOP, mass=STOP_MASS, color=STOP_GREY)
OBJECTS = [ROOM, DOOR_STOP]

# -- scoring ---------------------------------------------------------------------------------------------------

EPS = 1e-6
SHUT_TOL = math.radians(2.0)  # free edge within 5.6 mm of the frame stop
SWING_R = DOOR_W_FROM_HINGE + 0.005  # door's swept radius, with its corner
CLEAR = 0.005  # the door stop must be at least 5 mm outside the swing area
ON_TABLE = 0.001
SUCCESS_HOLD_S = 3.0


def _door_q(objects) -> float:
    return float(objects["room"].joints["door"])


def _shut(objects) -> bool:
    return _door_q(objects) <= SHUT_Q + SHUT_TOL + EPS


def _corners(st) -> np.ndarray:
    r = parts._quat_to_matrix(st.quat_xyzw)
    signs = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)], float)
    return (st.low + st.high) / 2 + (signs * np.asarray(STOP) / 2) @ r.T


def _swing_boundary() -> np.ndarray:
    """The swing area's outline, every 1 mm: the arc of radius SWING_R about the hinge from the open door's line to
    the shut door's line, and the two straight edges along those lines (both offset by half the door's thickness)."""
    hx, hy = HINGE[0] + DOOR[1] / 2, HINGE[1] + DOOR[1] / 2
    # The arc runs on past each door line by asin(5 mm / R), to meet the straight edges offset 5 mm from the hinge.
    ext = math.asin((DOOR[1] / 2) / SWING_R)
    n = int(SWING_R * (math.pi / 2 + 2 * ext) / 0.001)
    a = np.linspace(math.pi - ext, 1.5 * math.pi + ext, n)
    arc = np.c_[HINGE[0] + SWING_R * np.cos(a), HINGE[1] + SWING_R * np.sin(a)]
    t = np.linspace(0.0, 1.0, int(SWING_R / 0.001))[:, None]
    open_edge = np.array([arc[0, 0], hy]) * (1 - t) + np.array([hx, hy]) * t
    shut_edge = np.array([hx, arc[-1, 1]]) * (1 - t) + np.array([hx, hy]) * t
    return np.r_[arc, open_edge, shut_edge]


_OUTLINE = _swing_boundary()


def _in_swing(p) -> bool:
    return bool(p[0] <= HINGE[0] + DOOR[1] / 2 and p[1] <= HINGE[1] + DOOR[1] / 2
                and math.hypot(p[0] - HINGE[0], p[1] - HINGE[1]) <= SWING_R)



def _gap_to_swing(st) -> float:
    """Shortest distance (m) seen from above from the door stop's footprint (an 11 x 11 grid of points over it, its
    true orientation) to the swing area: the quarter circle of radius SWING_R about the hinge between the open and
    shut door lines. 0 if any point is inside."""
    r = parts._quat_to_matrix(st.quat_xyzw)
    centre = (st.low + st.high) / 2
    u = np.linspace(-0.5, 0.5, 11)
    pts = np.array([centre + r @ np.array([a * STOP[0], b * STOP[1], c * STOP[2]])
                    for a in u for b in u for c in (-0.5, 0.5)])[:, :2]
    if any(_in_swing(p) for p in pts):
        return 0.0
    d = np.linalg.norm(pts[:, None, :] - _OUTLINE[None, :, :], axis=2)
    return float(d.min())


def _stop_clear(objects) -> bool:
    """Released and at rest flat on the table, every part of it at least 5 mm outside the door's swing area."""
    st = objects["door_stop"]
    c = _corners(st)
    on_table = abs(float(c[:, 2].min()) - TOP) <= ON_TABLE + EPS
    return bool(st.resting() and on_table and _gap_to_swing(st) >= CLEAR - EPS)


def success(objects):
    """The door shut against its frame and the door stop out of its way, at rest on the table."""
    return bool(_shut(objects) and _stop_clear(objects))


def score(objects):
    """1.0 as success(); else 0 (pass/fail)."""
    return 1.0 if success(objects) else 0.0


@parts.on_reset
def _load_closer(sim):
    """A new episode: the door starts open; load its closer so it pulls the door 0.3 rad past shut."""
    sim.set_target("room", "door", SHUT_Q - PRELOAD)


# -- demo ------------------------------------------------------------------------------------------------------

TIP = 0.013
STOP_OPEN = FINGER_OPEN * 0.6  # about 56 mm between the pads: the fingers stay clear of the hook
KNOB_OPEN = FINGER_OPEN * 0.45  # about 42 mm
HIGH = TOP + 0.17


def _w(px, py, z):
    return (HINGE[0] + px, HINGE[1] + py, TOP + z)


def _knob_at(theta: float):
    """World TCP over the hook's knob with the hook lifted ``theta``: tips 4 mm above the arm's top."""
    rel = np.array([KNOB_Y - HOOK_PIVOT[1], (_arm_z + HOOK_ARM[2] / 2 + 0.004 + TIP) - HOOK_PIVOT[2]])
    c, s = math.cos(theta), math.sin(theta)
    y = rel[0] * c + rel[1] * s  # a turn about -x: the hook's outer end rises
    z = -rel[0] * s + rel[1] * c
    return _w(HOOK_X, HOOK_PIVOT[1] + y, HOOK_PIVOT[2] + z)


LIFT = 0.9  # rad commanded; the hook's end clears the door's top from about 0.3 rad (18 degrees)


def demo():
    """Left arm: picks the door stop up (fingers closing along x, 15 mm off its centre away from the door), sets it down well clear at (0.17, -0.06) with a
    7 mm drop, then grips the hook's knob, lifts the hook 0.9 rad, holds it while the door swings past, lets go
    and lifts away. The right arm stays where it starts."""
    sx, sy = STOP_XY
    sy -= 0.015  # grip 15 mm off centre, away from the door: the finger tips are wide
    grip = TOP + STOP[2] / 2 + TIP
    place = (0.170, -0.060)  # carried away from the hook, which stands between the door stop and the robot
    yaw = -math.pi / 2  # fingers close along x; +pi/2 puts joint 6 out of reach on the carry
    k0 = _knob_at(0.0)
    left = [
        Waypoint(1.5, (sx, sy, HIGH), STOP_OPEN, "over the door stop", yaw),
        Waypoint(1.4, (sx, sy, grip), STOP_OPEN, "down to it", yaw),
        Waypoint(0.8, (sx, sy, grip), FINGER_CLOSED, "grip", yaw),
        Waypoint(1.2, (sx, sy, HIGH), FINGER_CLOSED, "lift", yaw),
        Waypoint(1.8, (*place, HIGH), FINGER_CLOSED, "carry it clear", yaw),
        Waypoint(1.2, (*place, grip + 0.007), FINGER_CLOSED, "lower", yaw),
        Waypoint(0.6, (*place, grip + 0.007), STOP_OPEN, "release", yaw),
        Waypoint(1.0, (*place, HIGH), KNOB_OPEN, "up", yaw),
        Waypoint(1.5, (k0[0], k0[1], HIGH), KNOB_OPEN, "over the hook knob", yaw),
        Waypoint(1.2, k0, KNOB_OPEN, "down round the knob", yaw),
        Waypoint(0.6, k0, FINGER_CLOSED, "grip the knob", yaw),
        *(Waypoint(0.5, _knob_at(LIFT * k / 4), FINGER_CLOSED, f"lift the hook {k}", yaw) for k in range(1, 5)),
        Waypoint(2.0, _knob_at(LIFT), FINGER_CLOSED, "hold it while the door swings", yaw),
        Waypoint(0.6, _knob_at(LIFT), KNOB_OPEN, "let go", yaw),
        Waypoint(1.0, (_knob_at(LIFT)[0], _knob_at(LIFT)[1], HIGH), FINGER_OPEN, "up"),
        Waypoint(1.5, (0.20, 0.30, HIGH), FINGER_OPEN, "park"),
    ]
    return {"left": left}
