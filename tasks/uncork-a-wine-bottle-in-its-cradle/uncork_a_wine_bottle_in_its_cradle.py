"""On the station table, a corked 75 cl bottle of red wine lies in a wooden pouring cradle, tipped up 10 degrees with
its neck pointing back toward the right arm. A wing corkscrew has been set on its mouth, wings down: its bell over the
neck, the point of its worm on the middle of the cork.

Task (the instruction gives the goal only): open the bottle without taking it out of its cradle, and leave the cork off
it. The natural cork sits in the neck as tightly as a real one: it gives way only to a pull of 240 N. The corkscrew's
handle turns the worm into the cork, which draws the rack down and raises the two wings (each a lever on a pinion that
meshes with the rack); pressing the wings back down pulls the rack, the worm and the cork out with 6.4:1 leverage.

Why two hands: one arm can press one wing with at most about 20 N, which gives the cork about 130 N; it takes both
wings pressed together (about 19 N each) to start the cork.

Run it:  inspect-robots run --embodiment yam_arena -E task=uncork_a_wine_bottle_in_its_cradle.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, on_reset,  # noqa: E402
                     on_step, plate, rotation, segment, set_slide_target, set_thread_on, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- rotations ----------------------------------------------------------------------------------------------------


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _quat(m: np.ndarray) -> tuple[float, float, float, float]:
    """(x, y, z, w) of a rotation matrix (Shepperd: right for half turns too)."""
    m = np.asarray(m, float)
    t = float(np.trace(m))
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = (0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = ((m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s)
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = ((m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s)
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = ((m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s)
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return (x / n, y / n, z / n, w / n)


def _wxyz(m: np.ndarray) -> tuple[float, float, float, float]:
    x, y, z, w = _quat(m)
    return (w, x, y, z)


# -- the bottle: a 75 cl Bordeaux bottle of red wine. Its frame: origin on its axis at the body's middle, the neck along
# -y; it lies in the cradle tipped up 10 degrees, its neck toward the right arm and turned YAW toward the arms
TILT = math.radians(10.0)
YAW = math.radians(55.0)
BODY_R, BODY_Y = 0.0375, 0.095  # the body: 75 mm across, from y = -0.095 to +0.095 (190 mm)
NECK_Y0, LIP_Y = -0.125, -0.205  # the neck from the shoulder to the mouth (80 mm)
BORE_R, NECK_R, BEAD_R, BEAD_L = 0.00925, 0.0145, 0.0155, 0.010  # 18.5 mm bore, 29 mm neck, 31 mm bead (the lip)
BOTTLE_MASS = 1.25  # 0.5 kg of glass and 0.75 kg of wine
LIP = np.array([0.32, -0.02, TOP + 0.101])  # the middle of the bottle's mouth, in the world
BOTTLE_R = _rz(-YAW) @ _rx(-TILT)
BOTTLE_Q = _quat(BOTTLE_R)
BOTTLE_POS = LIP - BOTTLE_R @ np.array([0.0, LIP_Y, 0.0])
OUT = BOTTLE_R @ np.array([0.0, -1.0, 0.0])  # out of the bottle's mouth, along the neck
FRONT = BOTTLE_R @ np.array([1.0, 0.0, 0.0])  # level, square to the neck: toward the front wing
NORM = np.cross(OUT, FRONT)  # the wings' pivot axis (10 degrees off vertical)
R_C = np.column_stack([OUT, FRONT, NORM])  # the corkscrew's frame: x out along the axis, y to the front wing, z ~up
Q_C = _quat(R_C)

# -- the cork: natural cork, 38 x 24 mm, compressed into the 18.5 mm bore, its outer face flush with the lip ----------
CORK_L = 0.038
CORK_MASS = 0.010  # real: about 3.5 g; heavier for the solver (see fidelity)
F_PEAK = 240.0  # N to start it (literature: 230-400 N, 120-350 N for 38 x 24 mm natural corks)
NECK_K, NECK_D = 3.0e5, 2.0e3  # the friction hold's spring (N/m) and damping (N s/m), capped at the friction force
CORK_FREE = CORK_L + 0.0005  # pulled this far, the cork's inner end is past the lip: out of the neck
CORK_C = (0.72, 0.55, 0.36)

# -- the wing corkscrew (chrome-plated zinc alloy, steel worm): positions along its axis (u, m out from the lip face),
# toward the front wing (v) and up its pivot axis (w) ---------------------------------------------------------------
SKIRT = (-0.018, 0.0, 0.0165, 0.0195)  # the bell's skirt over the bead: from, to, inner and outer radius
SHOULDER = (0.0, 0.004, 0.011)  # the ring inside the bell that sits on the lip
BELL_U1, BELL_IN = 0.062, 0.016  # the bell's body, up to its head; inner radius
WINDOW = (0.012, 0.052)  # open windows in the bell's top and bottom: the worm and the cork show through
HEAD = (0.062, 0.078)
PIVOT_U, PIVOT_V = 0.070, 0.0235  # the wings' pivots
RACK_R, RACK_U = 0.006, (0.050, 0.126)  # the rack (ring teeth, 12 mm), where it starts
PINION_R = 0.0175  # the wings' pinions (pitch radius): 17.5 mm of rack per radian of wing
SHAFT_R, WORM_L = 0.004, 0.050
HANDLE_U, KNOB_L, KNOB_W = 0.142, 0.014, 0.016  # the handle: a square knob, 16 mm across its flats, 14 mm long
PAD_RHO, PAD_R, PAD_H = 0.1125, 0.012, 0.016  # each wing's paddle: how far from its pivot, radius, height
ARM_W, ARM_T, HEAD_R, HEAD_T = 0.010, 0.006, 0.019, 0.008
PHI_DOWN, PHI_UP = math.radians(175.0), math.radians(28.0)  # the wings' range (from the axis, outward)
TRAVEL = PINION_R * (PHI_DOWN - PHI_UP)  # 45 mm of rack
BELL_MASS, RACK_MASS, SPINDLE_MASS, WING_MASS = 0.085, 0.050, 0.045, 0.035
CHROME, CHROME_D, STEEL, BLACK = (0.80, 0.81, 0.84), (0.55, 0.56, 0.60), (0.66, 0.67, 0.70), (0.08, 0.08, 0.09)

# -- the worm's thread in the cork -----------------------------------------------------------------------------------
PITCH = 0.010  # m per turn (a 5-turn, 50 mm worm)
CATCH = math.radians(45.0)  # the point on the cork's face, turned 45 degrees clockwise: the thread has caught
CATCH_DEPTH, CATCH_RADIAL, CATCH_TILT = (-0.003, 0.006), 0.004, math.radians(8.0)
THREAD_K, THREAD_D, THREAD_MAX = 2.0e5, 500.0, 2000.0
THREAD_DRAG = 0.03  # N m s/rad
DEPTH_MAX = TRAVEL + 0.002

# -- the cradle -----------------------------------------------------------------------------------------------------
WOOD, WOOD_D, GLASS, LABEL, FOIL = (0.55, 0.36, 0.20), (0.42, 0.27, 0.15), (0.10, 0.25, 0.13), (0.92, 0.88, 0.78), \
    (0.45, 0.06, 0.10)

# -- scoring ----------------------------------------------------------------------------------------------------------
DEPTH_25 = 0.020  # the worm at least 20 mm (2 turns) into the cork
OUT_50 = 0.010  # the cork drawn at least 10 mm out of the neck
OFF_DIST = 0.060  # the cork's middle at least 6 cm from the mouth: off the bottle
IN_CRADLE_MOVE, IN_CRADLE_TILT = 0.02, math.radians(5.0)
LEFT_MOVE, LEFT_TILT = 0.05, math.radians(15.0)  # beyond this the bottle has been taken out of its cradle (latched)
HANDS_CLEAR, HANDS_STILL = 0.10, 0.002  # the trial ends with both hands this far from the job and moving less per step
SUCCESS_HOLD_S = 1.0


def _c(u: float, v: float = 0.0, w: float = 0.0) -> np.ndarray:
    """A point in the corkscrew's frame (as it starts), in the world."""
    return LIP + R_C @ np.array([u, v, w])


def _ring(r_in, r_out, u0, u1, color, n=16, skip=None, collide=True) -> list:
    """A tube along the corkscrew's (or the neck's) x axis, from u0 to u1: n boxes round it. ``skip(angle)`` leaves out
    boxes (windows)."""
    out, r = [], (r_in + r_out) / 2
    chord = 2 * r * math.tan(math.pi / n) * 1.08
    for k in range(n):
        a = 2 * math.pi * k / n
        if skip is not None and skip(a):
            continue
        q = (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))  # about x
        out.append(BoxShape(((u0 + u1) / 2, -r * math.sin(a), r * math.cos(a)), (abs(u1 - u0), chord, r_out - r_in),
                            color, quat_xyzw=q, collide=collide))
    return out


def _neck_ring(r_in, r_out, y0, y1, color, n=16) -> list:
    """A tube along the bottle's y axis (its neck), from y0 to y1."""
    out, r = [], (r_in + r_out) / 2
    chord = 2 * r * math.tan(math.pi / n) * 1.08
    for k in range(n):
        a = 2 * math.pi * k / n
        q = (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))  # about y
        out.append(BoxShape((r * math.sin(a), (y0 + y1) / 2, r * math.cos(a)), (chord, abs(y1 - y0), r_out - r_in),
                            color, quat_xyzw=q))
    return out


# -- the bodies -------------------------------------------------------------------------------------------------------

def _bottle() -> Body:
    shapes = [CylShape((0.0, 0.0, 0.0), BODY_R, 2 * BODY_Y, GLASS, axis="Y"),
              CylShape((0.0, -0.1025, 0.0), 0.031, 0.015, GLASS, axis="Y"),  # the shoulder
              CylShape((0.0, -0.1175, 0.0), 0.021, 0.015, GLASS, axis="Y")]
    shapes += _neck_ring(BORE_R, NECK_R, NECK_Y0, LIP_Y + BEAD_L, GLASS)
    shapes += _neck_ring(BORE_R, BEAD_R, LIP_Y + BEAD_L, LIP_Y, GLASS)
    shapes.append(CylShape((0.0, 0.0, 0.0), BODY_R + 0.0003, 0.090, LABEL, axis="Y", collide=False))  # the label
    shapes.append(CylShape((0.0, -0.16, 0.0), NECK_R + 0.0004, 0.050, FOIL, axis="Y", collide=False))  # cut capsule
    return Body.build("bottle", pos=tuple(BOTTLE_POS.tolist()), mass=BOTTLE_MASS, shapes=shapes, friction=(0.6, 0.5),
                      color=GLASS, start_xyzw=BOTTLE_Q, com=(0.0, 0.01, 0.0), solver_iterations=32)


def _cradle() -> Body:
    """A wooden pouring cradle, in the bottle's (tipped) frame: two slats under the body on two posts, a back board
    behind the bottle's base, and a base board on the table."""
    shapes, t, w = [], 0.008, 0.035
    for s in (-1.0, 1.0):
        d = np.array([s * math.sin(math.radians(40.0)), 0.0, -math.cos(math.radians(40.0))])
        m = (BODY_R + t / 2 + 0.0005) * d
        shapes.append(plate(m + [0.0, -0.080, 0.0], m + [0.0, 0.090, 0.0], w, t, d, WOOD))
    up = BOTTLE_R.T @ np.array([0.0, 0.0, 1.0])  # the world's vertical, in the bottle's frame
    for y in (-0.06, 0.07):
        top = np.array([0.0, y, -(BODY_R + 0.020)])
        foot_world = BOTTLE_POS + BOTTLE_R @ top
        foot = top - up * (foot_world[2] - TOP - 0.006)
        shapes.append(segment(top, foot, 0.016, WOOD))
    shapes.append(BoxShape((0.0, BODY_Y + 0.0065, -0.012), (0.070, 0.010, 0.060), WOOD_D))  # the back board
    base_centre = BOTTLE_R.T @ (np.array([BOTTLE_POS[0], BOTTLE_POS[1] + 0.005, TOP + 0.003]) - BOTTLE_POS)
    shapes.append(BoxShape(tuple(base_centre.tolist()), (0.09, 0.22, 0.006), WOOD, quat_xyzw=_quat(_rx(TILT))))
    return Body.build("cradle", pos=tuple(BOTTLE_POS.tolist()), mass=1.0, shapes=shapes, kinematic=True,
                      friction=(0.6, 0.5), color=WOOD, start_xyzw=BOTTLE_Q)


def _bell() -> Body:
    """The corkscrew's bell, frame at the middle of the bottle's mouth (as it starts): the skirt over the bead, the ring
    that sits on the lip, the body with a window top and bottom, and the head that carries the wings' pivots."""
    def window(a):  # the boxes at the top and bottom of the tube (w up/down)
        return abs(math.cos(a)) > 0.75

    shapes = _ring(SKIRT[2], SKIRT[3], SKIRT[0], SKIRT[1], CHROME)
    shapes += _ring(SHOULDER[2], SKIRT[3], SHOULDER[0], SHOULDER[1], CHROME)
    shapes += _ring(BELL_IN, SKIRT[3], SHOULDER[1], WINDOW[0], CHROME)
    shapes += _ring(BELL_IN, SKIRT[3], WINDOW[0], WINDOW[1], CHROME, skip=window)
    shapes += _ring(BELL_IN, SKIRT[3], WINDOW[1], HEAD[0], CHROME)
    hu = (HEAD[0] + HEAD[1]) / 2
    shapes.append(BoxShape((hu, 0.0, 0.0), (HEAD[1] - HEAD[0], 2 * (PIVOT_V + 0.006), 0.022), CHROME))
    for s in (-1.0, 1.0):  # the cheeks either side of each wing's head, and its pin
        for z in (-1.0, 1.0):
            shapes.append(BoxShape((PIVOT_U, s * PIVOT_V, z * (HEAD_T / 2 + 0.0025)), (0.024, 0.024, 0.004),
                                   CHROME_D, collide=False))
        shapes.append(CylShape((PIVOT_U, s * PIVOT_V, 0.0), 0.0025, HEAD_T + 0.011, STEEL, collide=False))
    return Body.build("bell", pos=tuple(LIP.tolist()), mass=BELL_MASS, shapes=shapes, friction=(0.5, 0.4),
                      color=CHROME, start_xyzw=Q_C, solver_iterations=64, contact_offset=0.001)


RACK_START = (RACK_U[0] + RACK_U[1]) / 2


def _rack() -> Body:
    """The rack: a 12 mm shaft with ring teeth, sliding along the bell's axis. Frame at its middle."""
    half = (RACK_U[1] - RACK_U[0]) / 2
    shapes = [CylShape((0.0, 0.0, 0.0), RACK_R - 0.001, 2 * half, STEEL, axis="X")]
    for k in range(int(2 * half / 0.004)):
        shapes.append(CylShape((-half + 0.002 + 0.004 * k, 0.0, 0.0), RACK_R, 0.002, STEEL, axis="X",
                               collide=False))
    return Body.build("rack", pos=tuple(_c(RACK_START).tolist()), mass=RACK_MASS, shapes=shapes, friction=(0.4, 0.3),
                      color=STEEL, start_xyzw=Q_C, ignore=("bell",), solver_iterations=64)


def _spindle() -> Body:
    """The worm (a 5-turn steel helix, 50 mm), its shaft through the rack, and the handle: a flat bar across the axis.
    Frame at the worm's point."""
    shapes = [CylShape((0.004, 0.0, 0.0), 0.0012, 0.008, STEEL, axis="X"),  # the point
              CylShape((0.008 + (WORM_L - 0.008) / 2, 0.0, 0.0), 0.0018, WORM_L - 0.008, STEEL, axis="X",
                       collide=False)]
    n = 60
    for k in range(n):  # the helix, right-handed, 4.5 mm out from the axis
        t0, t1 = k / n, (k + 1) / n
        p0 = np.array([0.006 + t0 * (WORM_L - 0.006), 0.0045 * math.cos(2 * math.pi * 5 * t0),
                       0.0045 * math.sin(2 * math.pi * 5 * t0)])
        p1 = np.array([0.006 + t1 * (WORM_L - 0.006), 0.0045 * math.cos(2 * math.pi * 5 * t1),
                       0.0045 * math.sin(2 * math.pi * 5 * t1)])
        shapes.append(segment(p0, p1, 0.0024, STEEL, collide=False))
    shapes.append(CylShape((WORM_L / 2 + 0.003, 0.0, 0.0), 0.0035, WORM_L - 0.006, None, axis="X"))  # its body
    shaft = (WORM_L, HANDLE_U - KNOB_L / 2)
    shapes.append(CylShape(((shaft[0] + shaft[1]) / 2, 0.0, 0.0), SHAFT_R, shaft[1] - shaft[0], STEEL, axis="X",
                           collide=False))
    q45 = (math.sin(math.pi / 8), 0.0, 0.0, math.cos(math.pi / 8))  # the knob's flats turned 45 degrees
    shapes.append(BoxShape((HANDLE_U, 0.0, 0.0), (KNOB_L, KNOB_W, KNOB_W), CHROME, quat_xyzw=q45))
    shapes.append(BoxShape((HANDLE_U + KNOB_L / 2 + 0.0002, 0.0, 0.0), (0.0004, KNOB_W - 0.004, KNOB_W - 0.004),
                           CHROME_D, quat_xyzw=q45, collide=False))
    return Body.build("spindle", pos=tuple(_c(0.0).tolist()), mass=SPINDLE_MASS, shapes=shapes, friction=(1.0, 0.9),
                      color=STEEL, start_xyzw=Q_C, ignore=("bell", "rack"), solver_iterations=64,
                      com=(0.10, 0.0, 0.0))


def _wing(name: str, side: int) -> Body:
    """One wing, frame at its pivot (axes as the corkscrew's): the pinion head, the lever and its paddle, laid out at
    the angle ``PHI_DOWN`` from the axis (wings down)."""
    phi = PHI_DOWN
    d = np.array([math.cos(phi), side * math.sin(phi), 0.0])
    shapes = [CylShape((0.0, 0.0, 0.0), HEAD_R, HEAD_T, CHROME_D)]
    for k in range(7):  # the pinion's teeth, toward the rack
        a = math.pi + side * (k - 3) * 0.32 + (0.0 if side > 0 else 0.0)
        c = np.array([HEAD_R * math.cos(a) * 1.0, HEAD_R * math.sin(a) * side, 0.0])
        shapes.append(BoxShape(tuple(c.tolist()), (0.003, 0.003, HEAD_T - 0.001), STEEL, collide=False,
                               quat_xyzw=_quat(_rz(a))))
    a0, a1 = HEAD_R - 0.003, PAD_RHO - PAD_R + 0.002
    shapes.append(BoxShape(tuple(((a0 + a1) / 2 * d).tolist()), (a1 - a0, ARM_W, ARM_T), CHROME,
                           quat_xyzw=_quat(_rz(math.atan2(d[1], d[0])))))
    shapes.append(CylShape(tuple((PAD_RHO * d).tolist()), PAD_R, PAD_H, CHROME))
    shapes.append(CylShape(tuple((PAD_RHO * d + [0.0, 0.0, PAD_H / 2 + 0.0002]).tolist()), PAD_R - 0.002, 0.0004,
                           BLACK, collide=False))
    pivot = _c(PIVOT_U, side * PIVOT_V)
    return Body.build(name, pos=tuple(pivot.tolist()), mass=WING_MASS, shapes=shapes, friction=(0.9, 0.8),
                      color=CHROME, start_xyzw=Q_C, ignore=("bell", "rack", "spindle"), solver_iterations=64,
                      contact_offset=0.001)


def _cork() -> Body:
    pos = BOTTLE_POS + BOTTLE_R @ np.array([0.0, LIP_Y + CORK_L / 2, 0.0])
    shapes = [CylShape((0.0, 0.0, 0.0), BORE_R, CORK_L, CORK_C, axis="Y"),
              CylShape((0.0, -CORK_L / 2 - 0.0001, 0.0), BORE_R - 0.0015, 0.0002, (0.45, 0.10, 0.12), axis="Y",
                       collide=False)]  # the wine-stained ... (outer face: a little darker)
    return Body.build("cork", pos=tuple(pos.tolist()), mass=CORK_MASS, shapes=shapes, friction=(0.8, 0.7),
                      color=CORK_C, start_xyzw=BOTTLE_Q, ignore=("bottle", "bell", "rack", "spindle", "wing_f",
                                                                  "wing_b"), solver_iterations=128)


CRADLE, BOTTLE, BELL, RACK, SPINDLE = _cradle(), _bottle(), _bell(), _rack(), _spindle()
WING_F, WING_B, CORK = _wing("wing_f", 1), _wing("wing_b", -1), _cork()
OBJECTS = [CRADLE, BOTTLE, BELL, RACK, SPINDLE, WING_F, WING_B, CORK]

# -- the joints --------------------------------------------------------------------------------------------------------
# Each is authored in the frames of its two bodies as they start (bodykit's own joints assume unrotated bodies).
# The joint frame's x axis is the slide or turn axis.

_POSE = {b.name: (np.array(b.pos), rotation(b.start_xyzw) if b.start_xyzw is not None else np.eye(3)) for b in OBJECTS}
_JOINTS: list[dict] = []


def _frame(body: str, anchor, r_joint) -> tuple:
    p, r = _POSE[body]
    return tuple((r.T @ (np.asarray(anchor, float) - p)).tolist()), _wxyz(r.T @ r_joint)


def _joint(kind: str, name: str, body0: str, body1: str, anchor, r_joint, **kw) -> None:
    _JOINTS.append(dict(kind=kind, name=name, body0=body0, body1=body1, f0=_frame(body0, anchor, r_joint),
                        f1=_frame(body1, anchor, r_joint), **kw))


R_HINGE = np.column_stack([NORM, OUT, FRONT])  # x along the pivot axis
# the cork in the neck: slides out along the neck, held by friction (a drive capped at the friction force)
_joint("prismatic", "neck", "bottle", "cork", _POSE["cork"][0], R_C, limits=(-0.002, CORK_L + 0.03),
       drive=(NECK_K, NECK_D, 0.0, F_PEAK))
# the rack slides in the bell along the axis (x out); it starts fully out (wings down). The wings turn on their pivots:
# wing_f's angle from the axis falls (turning -) as it rises, wing_b's (turning +). The rack and the pinions mesh:
# PhysX's rack-and-pinion joint cannot run in this GPU scene, so each of the three carries a stiff drive toward one
# shared gear position, re-aimed before every physics step (see _gear): a stiff, slightly springy mesh.
GEAR_K, GEAR_D = 6.0e4, 50.0  # N/m and N s/m at the rack; the wings' drives are the same seen through the pinion
_joint("prismatic", "rack_slide", "bell", "rack", _c(RACK_START), R_C, limits=(-TRAVEL, 0.0005),
       drive=(GEAR_K, GEAR_D, 0.0, 1.0e6))
# the worm's spindle turns in the rack (a bearing)
_joint("revolute", "spin", "rack", "spindle", _c(RACK_START), R_C, damping=1.0e-6)
_DEG = math.pi / 180.0
_joint("revolute", "pivot_f", "bell", "wing_f", _c(PIVOT_U, PIVOT_V), R_HINGE,
       limits=(-math.degrees(PHI_DOWN - PHI_UP) - 1.0, 0.5),
       angular=(GEAR_K * PINION_R ** 2 * _DEG, GEAR_D * PINION_R ** 2 * _DEG))
_joint("revolute", "pivot_b", "bell", "wing_b", _c(PIVOT_U, -PIVOT_V), R_HINGE,
       limits=(-0.5, math.degrees(PHI_DOWN - PHI_UP) + 1.0),
       angular=(GEAR_K * PINION_R ** 2 * _DEG, GEAR_D * PINION_R ** 2 * _DEG))
# the bell's collar on the neck: it keeps the bell on the neck's axis (free to turn on it and to slide off it, and it
# sits on the lip) until the collar has slid COLLAR_OFF off the bead; then the bell is free
COLLAR_OFF = 0.015
COLLAR_ROLL = 0.003  # N m s per degree: the collar's friction on the lip resists the bell turning on the neck
_joint("d6", "collar", "bottle", "bell", LIP, R_C, limits=(-0.0005, COLLAR_OFF + 0.01), lock=True, collide=True,
       roll_damping=COLLAR_ROLL)
# the worm in the cork: a joint that keeps the cork on the worm's axis and turns it against the thread's drag, its slide
# (the depth of the point in the cork) held by a drive. Its point resting on the cork's face, the drive keeps it from
# sinking in unturned but does not pull; turned 45 degrees clockwise there, the thread has caught and the drive's
# target follows the turns. (No joint between the cork and the rest is switched on or off while the cork is in the
# neck: PhysX does not carry drive forces reliably through joints switched at run time.)
thread("worm", "spindle", "cork", anchor0=(0.0, 0.0, 0.0), axis0=(1.0, 0.0, 0.0), anchor1=(0.0, -CORK_L / 2, 0.0),
       axis1=(0.0, -1.0, 0.0), slide_limits=(-0.003, DEPTH_MAX), turn_damping=THREAD_DRAG, slide_stiffness=THREAD_K,
       slide_damping=THREAD_D, slide_max_force=THREAD_MAX, always_on=True)
# the thread's slide coordinate is the depth: how far the cork's face is out from the worm's point

_JOINT_PATHS: dict[str, str] = {}


def _author_extra(stage, prim_path: str, spec) -> None:
    from pxr import Gf, UsdPhysics

    parent = prim_path.rsplit("/", 1)[0]
    for j in _JOINTS:
        if j["body1"] != spec.name:
            continue
        path = f"{prim_path}/{j['name']}"
        if j["kind"] == "prismatic":
            joint = UsdPhysics.PrismaticJoint.Define(stage, path)
        elif j["kind"] == "revolute":
            joint = UsdPhysics.RevoluteJoint.Define(stage, path)
        else:
            joint = UsdPhysics.Joint.Define(stage, path)  # a D6: only the slide along x is bounded
        joint.CreateBody0Rel().SetTargets([f"{parent}/{j['body0']}/body"])
        joint.CreateBody1Rel().SetTargets([f"{prim_path}/body"])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*j["f0"][0]))
        joint.CreateLocalRot0Attr(Gf.Quatf(*j["f0"][1]))
        joint.CreateLocalPos1Attr(Gf.Vec3f(*j["f1"][0]))
        joint.CreateLocalRot1Attr(Gf.Quatf(*j["f1"][1]))
        joint.CreateCollisionEnabledAttr(False)
        joint.CreateExcludeFromArticulationAttr(True)
        if j["kind"] == "d6":
            joint.CreateJointEnabledAttr(True)
            if j.get("collide"):
                joint.GetCollisionEnabledAttr().Set(True)
            limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "transX")
            limit.CreateLowAttr(float(j["limits"][0]))
            limit.CreateHighAttr(float(j["limits"][1]))
            if j.get("lock"):  # on the axis, not tilted: low > high locks
                for dof in ("transY", "transZ", "rotY", "rotZ"):
                    limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), dof)
                    limit.CreateLowAttr(1.0)
                    limit.CreateHighAttr(-1.0)
            if j.get("roll_damping"):
                drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "rotX")
                drive.CreateTypeAttr("force")
                drive.CreateStiffnessAttr(0.0)
                drive.CreateDampingAttr(float(j["roll_damping"]))
                drive.CreateTargetVelocityAttr(0.0)
                drive.CreateMaxForceAttr(1.0e6)
        else:
            joint.CreateAxisAttr("X")
            if "limits" in j:
                joint.CreateLowerLimitAttr(float(j["limits"][0]))
                joint.CreateUpperLimitAttr(float(j["limits"][1]))
            if "drive" in j:
                k, d, target, fmax = j["drive"]
                drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
                drive.CreateTypeAttr("force")
                drive.CreateStiffnessAttr(float(k))
                drive.CreateDampingAttr(float(d))
                drive.CreateTargetPositionAttr(float(target))
                drive.CreateMaxForceAttr(float(fmax))
            if "angular" in j:
                k, d = j["angular"]
                drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
                drive.CreateTypeAttr("force")
                drive.CreateStiffnessAttr(float(k))
                drive.CreateDampingAttr(float(d))
                drive.CreateTargetPositionAttr(0.0)
                drive.CreateMaxForceAttr(1.0e6)
            if j.get("damping"):
                drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
                drive.CreateTypeAttr("force")
                drive.CreateStiffnessAttr(0.0)
                drive.CreateDampingAttr(float(j["damping"]))
                drive.CreateTargetVelocityAttr(0.0)
                drive.CreateMaxForceAttr(1.0e6)
        _JOINT_PATHS[j["name"]] = path
    if spec.name == "cork":
        from pxr import UsdPhysics as _U

        _U.DriveAPI(stage.GetPrimAtPath(f"{prim_path}/worm"), "transX").GetTargetPositionAttr().Set(0.0)


_orig_author_sliders = bodykit._author_sliders


def _author_sliders(stage, prim_path, spec):
    _orig_author_sliders(stage, prim_path, spec)
    _author_extra(stage, prim_path, spec)


bodykit._author_sliders = _author_sliders

_ATTRS: dict = {}


def _neck_attrs() -> tuple:
    if "neck" not in _ATTRS:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        prim = get_current_stage().GetPrimAtPath(_JOINT_PATHS["neck"])
        drive = UsdPhysics.DriveAPI(prim, "linear")
        joint = UsdPhysics.Joint(prim)
        _ATTRS["neck"] = (joint.GetJointEnabledAttr() or joint.CreateJointEnabledAttr(True),
                          drive.GetTargetPositionAttr(), drive.GetMaxForceAttr())
    return _ATTRS["neck"]


def _set_neck(on: bool | None = None, target: float | None = None, fmax: float | None = None) -> None:
    on_attr, target_attr, fmax_attr = _neck_attrs()
    if on is not None and bool(on_attr.Get()) != bool(on):
        on_attr.Set(bool(on))
    if target is not None:
        target_attr.Set(float(target))
    if fmax is not None:
        fmax_attr.Set(float(fmax))


def _joint_attr(name: str, what: str):
    """A USD attribute of one of the joints above: "on", or a drive's "target" ("linear" or "angular" by kind)."""
    key = (name, what)
    if key not in _ATTRS:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        prim = get_current_stage().GetPrimAtPath(_JOINT_PATHS[name])
        if what == "on":
            j = UsdPhysics.Joint(prim)
            _ATTRS[key] = j.GetJointEnabledAttr() or j.CreateJointEnabledAttr(True)
        else:
            kind = "linear" if prim.IsA(UsdPhysics.PrismaticJoint) else "angular"
            _ATTRS[key] = UsdPhysics.DriveAPI(prim, kind).GetTargetPositionAttr()
    return _ATTRS[key]


def _set_on(name: str, on: bool) -> None:
    attr = _joint_attr(name, "on")
    if attr.Get() is None or bool(attr.Get()) != bool(on):
        attr.Set(bool(on))


_GEAR = {"base": None, "c": 0.0}


def _gear_targets(c: float) -> None:
    _joint_attr("rack_slide", "target").Set(float(c))
    _joint_attr("pivot_f", "target").Set(float(math.degrees(c / PINION_R)))
    _joint_attr("pivot_b", "target").Set(float(math.degrees(-c / PINION_R)))
    _GEAR["c"] = c


def _gear_state(base) -> tuple[float, float, float]:
    """The rack's slide (m, + out) and each wing's turn seen through its pinion (m of rack)."""
    origin = as_numpy(base.scene.env_origins[0])
    pe, re = _pose(base, origin, "bell")
    pr, _ = _pose(base, origin, "rack")
    _, rf = _pose(base, origin, "wing_f")
    _, rw = _pose(base, origin, "wing_b")
    q = float((pr - pe) @ re[:, 0]) - RACK_START
    a, b = re.T @ rf, re.T @ rw
    return q, PINION_R * math.atan2(a[1, 0], a[0, 0]), -PINION_R * math.atan2(b[1, 0], b[0, 0])


def _gear(base) -> None:
    """The mesh of the rack and the two pinions: before each physics step, aim the rack's and the wings' drives at the
    mean of where each of the three says the gear is. The drives then push the three toward each other like stiff
    teeth: a torque on a wing reaches the rack as that torque over the pinion's radius, and back."""
    q, qf, qb = _gear_state(base)
    _gear_targets((q + qf + qb) / 3.0)


def _hook_gear(base) -> None:
    scene = base.scene
    if getattr(scene, "_cork_gear", False):
        return
    plain = scene.write_data_to_sim

    def write_data_to_sim(*args, **kwargs):
        _gear(base)
        return plain(*args, **kwargs)

    scene.write_data_to_sim = write_data_to_sim
    scene._cork_gear = True


def _thread_on_state():
    try:
        return bodykit._thread_attrs("worm")[0].Get()
    except Exception:  # noqa: BLE001
        return None


def _friction(s: float) -> float:
    """The cork's grip in the neck (N) with ``s`` of it drawn out: the peak at the start, falling with the length still
    in the neck (friction on the contact length)."""
    return F_PEAK * min(1.0, max(0.0, (CORK_L - s) / CORK_L))


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"hands_done": False, "pre": 0.0, "caught": False, "bite": True, "rest": 0.0, "collar": True, "x0": 0.0,
            "turns": 0.0, "psi": None, "target": 0.0, "in_neck": True,
            "s_max": 0.0, "left_cradle": False, "tcps": None, "steps": 0, "view": None, "view_failed": None,
            "f_pad": [0.0, 0.0], "f_pad_max": [0.0, 0.0], "fmax_set": F_PEAK, "st": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("CORK_DEBUG"))}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    try:
        _set_neck(on=True, target=0.0, fmax=F_PEAK)
        _gear_targets(0.0)
        _set_on("collar", True)
        set_slide_target("worm", 0.0)
        set_thread_on("worm", True)
    except KeyError:
        pass


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _pad_forces(base) -> list[float]:
    """The push on each paddle (N, square to the wing) that the gear's drives are resisting: each wing's drive torque
    over the paddle's distance from the pivot (wing_f, wing_b)."""
    q, qf, qb = _gear_state(base)
    c = _GEAR["c"]
    return [GEAR_K * PINION_R * (qf - c) / PAD_RHO, GEAR_K * PINION_R * (qb - c) / PAD_RHO,
            GEAR_K * (c - q)]


def _spin_angle(r_spindle, r_rack) -> float:
    """The spindle's turn about the axis against the rack (rad, + counter-clockwise seen from the handle)."""
    rel = r_rack.T @ r_spindle
    return math.atan2(rel[2, 1], rel[1, 1])


def _wing_angle(r_wing, r_bell, side: int) -> float:
    """A wing's angle from the axis (outward), from its turn against the bell (rad)."""
    rel = r_bell.T @ r_wing
    turn = math.atan2(rel[1, 0], rel[0, 0])  # about the pivot axis (the frame's z)
    return PHI_DOWN + side * turn


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    _hook_gear(base)
    origin = as_numpy(base.scene.env_origins[0])
    _EP["steps"] += 1
    pb, rb = _pose(base, origin, "bottle")
    pk, rk = _pose(base, origin, "cork")
    ps, rs = _pose(base, origin, "spindle")
    pr, rr = _pose(base, origin, "rack")
    pe, re = _pose(base, origin, "bell")
    pf, rf = _pose(base, origin, "wing_f")
    pw, rw = _pose(base, origin, "wing_b")
    prev = _EP["tcps"]
    _EP["tcps"] = _tcps(base, origin)
    out_dir = rb @ np.array([0.0, -1.0, 0.0])
    cork0 = pb + rb @ np.array([0.0, LIP_Y + CORK_L / 2, 0.0])
    s = float((pk - cork0) @ out_dir)  # how far the cork has been drawn out of the neck
    _EP["s_max"] = max(_EP["s_max"], s)
    # the worm against the cork: depth of the point behind the cork's face, off its axis, tilt
    axis_s = rs[:, 0]  # out along the worm
    face = pk - (CORK_L / 2) * (rk @ np.array([0.0, 1.0, 0.0]))
    v = ps - face  # the point, from the face's middle
    n_in = rk @ np.array([0.0, 1.0, 0.0])
    depth = float(v @ n_in)
    radial = float(np.linalg.norm(v - depth * n_in))
    tilt = math.acos(max(-1.0, min(1.0, float(-axis_s @ n_in))))
    # the turn of the spindle in the rack (the rack does not turn): clockwise seen from the handle drives it in
    psi = _spin_angle(rs, rr)
    tighten = 0.0
    if _EP["psi"] is not None:
        tighten = -((psi - _EP["psi"] + math.pi) % (2 * math.pi) - math.pi)
    _EP["psi"] = psi
    if _EP["caught"]:
        _EP["turns"] += tighten / (2 * math.pi)
        target = _EP["x0"] + PITCH * _EP["turns"]
        if target < -0.003:  # unscrewed off the point: free of the cork
            set_thread_on("worm", False)
            _EP.update(caught=False, bite=False, pre=0.0, turns=0.0)
        else:
            set_slide_target("worm", min(target, DEPTH_MAX))
    elif _EP["bite"]:  # not caught: the point rests on the face, held from sinking, free to be drawn back
        if depth < -0.0025:  # drawn back out of the cork (its slide stops at 3 mm): free of it
            set_thread_on("worm", False)
            _EP["bite"] = False
        else:
            if depth < _EP["rest"] - 0.0002:
                _EP["rest"] = depth
                set_slide_target("worm", depth)
            lined = (tilt <= CATCH_TILT and radial <= CATCH_RADIAL and CATCH_DEPTH[0] <= depth <= CATCH_DEPTH[1])
            _EP["pre"] = _EP["pre"] + tighten if lined else 0.0
            if _EP["pre"] >= CATCH:
                x0 = max(_EP["rest"], 0.0)
                set_slide_target("worm", x0)
                _EP.update(caught=True, x0=x0, turns=0.0)
    # the bell's collar slid off the bead: the corkscrew is free of the bottle
    collar = float((pe - (pb + rb @ np.array([0.0, LIP_Y, 0.0]))) @ out_dir)
    if _EP["collar"] and collar > COLLAR_OFF:
        _set_on("collar", False)
        _EP["collar"] = False
    # the cork in the neck: held by friction; out of the neck it is free
    if _EP["in_neck"]:
        if s > CORK_FREE:
            _set_neck(on=False)
            _EP["in_neck"] = False
        else:
            f = max(_friction(max(s, 0.0)), 1.0)
            lo, hi = s - f / NECK_K, s + f / NECK_K
            t = min(max(_EP["target"], lo), hi)
            if t != _EP["target"] or abs(f - _EP["fmax_set"]) > 0.5:
                _set_neck(target=t, fmax=f)
                _EP["target"], _EP["fmax_set"] = t, f
    # the bottle taken out of its cradle (latched)
    moved = float(np.linalg.norm(pb - BOTTLE_POS))
    axis_off = math.acos(max(-1.0, min(1.0, float(out_dir @ OUT))))
    if moved > LEFT_MOVE or axis_off > LEFT_TILT:
        _EP["left_cradle"] = True
    fp = _pad_forces(base)
    _EP["f_pad"] = fp
    _EP["f_pad_max"] = [max(a, b) for a, b in zip(_EP["f_pad_max"], fp[:2])]
    q_rack = float((pr - pe) @ (re[:, 0])) - RACK_START  # the rack's slide (m, + out)
    phi_f, phi_b = _wing_angle(rf, re, 1), _wing_angle(rw, re, -1)
    lip = pb + rb @ np.array([0.0, LIP_Y, 0.0])
    near = min(float(np.linalg.norm(t - p)) for t in _EP["tcps"] for p in (lip, pk, pe, ps))
    still = prev is not None and max(float(np.linalg.norm(a - b)) for a, b in zip(_EP["tcps"], prev)) < HANDS_STILL
    _EP["hands_done"] = near >= HANDS_CLEAR and still
    _EP["st"] = dict(s=s, depth=depth, moved=moved, axis_off=axis_off, q_rack=q_rack, phi_f=phi_f, phi_b=phi_b,
                     cork_to_lip=float(np.linalg.norm(pk - (pb + rb @ np.array([0.0, LIP_Y, 0.0])))))
    if _DEBUG["on"] and _EP["steps"] % 5 == 0:
        tl, tr = _EP["tcps"]
        pad_f = pf + rf @ (PAD_RHO * np.array([math.cos(PHI_DOWN), math.sin(PHI_DOWN), 0.0]))
        pad_b = pw + rw @ (PAD_RHO * np.array([math.cos(PHI_DOWN), -math.sin(PHI_DOWN), 0.0]))
        handle = ps + rs @ np.array([HANDLE_U, 0.0, 0.0])
        print(f"[cork] t={_EP['steps'] / 20:5.2f} s={s * 1000:6.2f}mm fric={_friction(max(s, 0)):5.1f} "
              f"depth={depth * 1000:6.2f} rad={radial * 1000:4.1f} tilt={math.degrees(tilt):4.1f} "
              f"caught={_EP['caught']} turns={_EP['turns']:5.2f} pre={math.degrees(_EP['pre']):5.1f} "
              f"psi={math.degrees(psi):6.1f} q_rack={q_rack * 1000:6.2f} phi_f={math.degrees(phi_f):6.1f} "
              f"phi_b={math.degrees(phi_b):6.1f} mesh_f={(q_rack - PINION_R * (phi_f - PHI_DOWN)) * 1000:5.2f} "
              f"mesh_b={(q_rack - PINION_R * (phi_b - PHI_DOWN)) * 1000:5.2f} c={_GEAR['c'] * 1000:6.2f} "
              f"F_pad=[{fp[0]:5.1f},{fp[1]:5.1f}] F_rack={fp[2]:6.1f} max=[{_EP['f_pad_max'][0]:5.1f},{_EP['f_pad_max'][1]:5.1f}] "
              f"bottle moved={moved * 1000:4.1f} axis={math.degrees(axis_off):4.1f} "
              f"bell={np.round((pe - LIP) * 1000, 1).tolist()} "
              f"roll={math.degrees(math.atan2(float(re[:, 1] @ (rb @ np.array([0.0, 0.0, 1.0]))), float(re[:, 1] @ (rb @ np.array([1.0, 0.0, 0.0]))))):5.1f} "
              f"L-pad_b={np.round((tl - pad_b) * 1000).astype(int).tolist()} "
              f"R-pad_f={np.round((tr - pad_f) * 1000).astype(int).tolist()} "
              f"R-handle={np.round((tr - handle) * 1000).astype(int).tolist()} "
              f"cork-lip={_EP['st']['cork_to_lip'] * 1000:5.1f} score={_points(_status()):.2f} "
              f"neck=[{_neck_attrs()[0].Get()},{(_neck_attrs()[1].Get() or 0) * 1000:.2f},{_neck_attrs()[2].Get()}] "
              f"in_neck={_EP['in_neck']} thread={_thread_on_state()} rest={_EP['rest'] * 1000:.2f} "
              f"collar={_joint_attr('collar', 'on').Get()} "
              "",
              file=sys.stderr, flush=True)


# -- scoring -----------------------------------------------------------------------------------------------------------


def _status() -> dict:
    st = _EP["st"]
    if not st:
        return {"depth_ok": False, "drawn": False, "off": False, "in_cradle": True, "left": False}
    in_cradle = st["moved"] <= IN_CRADLE_MOVE and st["axis_off"] <= IN_CRADLE_TILT
    return {"depth_ok": _EP["caught"] and st["depth"] >= DEPTH_25,
            "drawn": st["s"] >= OUT_50,
            "off": (not _EP["in_neck"]) and st["s"] > CORK_FREE and st["cork_to_lip"] >= OFF_DIST,
            "in_cradle": in_cradle, "left": _EP["left_cradle"]}


def _points(st) -> float:
    if st["left"]:
        return 0.0
    if st["off"] and st["in_cradle"]:
        return 1.0
    if st["drawn"]:
        return 0.5
    return 0.25 if st["depth_ok"] else 0.0


def score(objects) -> float:
    """1.0: the cork out of the neck and off the bottle (its middle at least 6 cm from the mouth) with the bottle still
    in its cradle; 0.5: the cork drawn at least 10 mm out of the neck; 0.25: the worm turned at least 20 mm into the
    cork; 0.0 otherwise, and 0.0 for good once the bottle has been taken out of its cradle (moved more than 5 cm or
    its axis turned more than 15 degrees)."""
    return _points(_status())


def success(objects) -> bool:
    """Full score with the bottle, the cork and the corkscrew still, and both hands still and clear of them (at least
    10 cm from the bottle's mouth, the cork and the corkscrew)."""
    return (_points(_status()) >= 1.0 and objects["bottle"].resting() and objects["cork"].resting()
            and objects["bell"].resting() and _EP.get("hands_done", False))


# -- scripted demo -----------------------------------------------------------------------------------------------------

def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    seeds = itertools.product((-1.2, -0.5, 0.0, 0.5, 1.2), (0.8, 1.4, 2.1), (0.5, 1.2, 2.0), (-1.4, -0.7, 0.0, 0.7),
                              (0.0,), (-1.0, 0.0, 1.0))
    for seed in seeds:
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=150)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
            if room > 0.25:
                break
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


LEAD_S, LEAD_STEPS = 4.0, 16


def _lead_in(arm: str, pos, rot, label: str, finger=FINGER_CLOSED) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _reachable_lift(arm: str, pos, rot) -> float:
    for up in (0.06, 0.045, 0.03, 0.02, 0.01):
        try:
            _good_ik(arm, np.asarray(pos) + [0.0, 0.0, up], rot)
            return up
        except ValueError:
            pass
    return 0.0


def _home(arm: str, pos, rot, label: str, t: float = 3.0, n: int = 12) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = _good_ik(arm, pos, rot), np.array(READY, float)
    out = []
    for k in range(1, n + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / n, station.BASES[arm])
        out.append(Waypoint(t / n, tuple(float(v) for v in p), FINGER_CLOSED, label, r))
    return out


def pad_at(phi: float, side: int) -> np.ndarray:
    """A paddle's middle (world) with its wing at ``phi`` from the axis."""
    return _c(PIVOT_U + PAD_RHO * math.cos(phi), side * (PIVOT_V + PAD_RHO * math.sin(phi)))


def pad_tangent(phi: float, side: int) -> np.ndarray:
    """Which way a paddle moves as its wing is pressed down (phi rising)."""
    return R_C @ np.array([-math.sin(phi), side * math.cos(phi), 0.0])


DEPTH_IN = 0.040  # screw the worm this far into the cork
DEPTH_PLAN = 0.0415  # the hand's plan: each 90 degree stroke turns the knob about 86 degrees
PHI_START = PHI_DOWN - DEPTH_IN / PINION_R  # where the wings are then
PHI_AIM = PHI_START - math.radians(3.0)  # the hands start behind where the paddles may be
PUSH_LEAD = 0.04  # the hands aim this far ahead of the paddles (m)
PUSH_BACK = PAD_R + 0.013  # the TCP behind the paddle's middle when touching it
PUSH_Z = 0.006  # the TCP above the paddle's middle
PRESS_S, FINISH_S = 7.0, 2.5
PHI_TWO = math.radians(150.0)  # both hands press to here; then the left hand alone takes the wings down
BELL_GRIP_U, BELL_GRIP_Z = 0.045, 0.012  # the top-down grip on the bell (along the axis from the lip; TCP above it)
PUT = np.array([0.24, -0.20, TOP + 0.045])  # where the corkscrew is laid down
# turning the handle: a side grip pointing down the corkscrew's axis, its fingers closing across the flat handle
IN = -OUT
R_SIDE = np.column_stack([FRONT, np.cross(IN, FRONT), IN])
ROLL0, STROKE = math.radians(-45.0), math.radians(90.0)  # the knob's flats come round every 90 degrees
TURN_S, RELEASE_S, BACK_S, REGRIP_S = 1.2, 0.4, 0.6, 0.4
OPEN_H = -0.016  # fingers open round the knob (16 mm across its flats)
TCP_OUT = 0.006  # the TCP this far out from the knob's middle
SIDE_IN = 0.04  # the hand comes onto the knob sideways, from this far along its fingers' flat


def _side(roll: float) -> np.ndarray:
    return R_SIDE @ _rz(roll)


def _handle_tcp(depth: float) -> np.ndarray:
    """The right TCP on the handle with the worm ``depth`` into the cork (the rack drawn in that far)."""
    return _c(HANDLE_U + TCP_OUT - depth)


X0_PLAN = 0.0  # the point is on the cork's face when the thread catches


def _depth_after(turned: float) -> float:
    return X0_PLAN + PITCH * max(0.0, turned - CATCH) / (2 * math.pi)


def _push_yaw(phi: float, side: int, prev: float | None) -> float:
    """The yaw of a top-down grip whose flat fingers push along the paddle's path (either face), nearest ``prev``."""
    t = pad_tangent(phi, side)
    a = math.atan2(t[1], t[0])
    cands = [y for y in (a, a - math.pi, a + math.pi) if abs(y) <= math.pi]
    ref = prev if prev is not None else (0.6 if side > 0 else -0.6)
    return min(cands, key=lambda y: abs(y - ref))


def _push_point(phi: float, side: int, back: float) -> tuple[np.ndarray, np.ndarray]:
    t = pad_tangent(phi, side)
    th = np.array([t[0], t[1], 0.0]) / np.linalg.norm(t[:2])
    return pad_at(phi, side) - back * th + [0.0, 0.0, PUSH_Z], th


def _press(arm: str, side: int, phi0: float, phi1: float, total: float, n: int, lead: float,
           yaw0: float | None) -> tuple[list[Waypoint], float]:
    out, yaw = [], yaw0
    for k in range(1, n + 1):
        phi = phi0 + (phi1 - phi0) * k / n
        p, th = _push_point(phi, side, PUSH_BACK)
        yaw = _push_yaw(phi, side, yaw)
        out.append(_w(total / n, p + lead * th, FINGER_CLOSED, "press the wing down", grip_rotation(yaw)))
    return out, yaw


def _right() -> tuple[list[Waypoint], float]:
    """Turn the worm into the cork; returns the waypoints and the time the press starts."""
    rot0 = _side(ROLL0)
    aside = rot0[:, 1] if rot0[2, 1] > 0 else -rot0[:, 1]  # sideways off the knob, the upward way
    out = _lead_in("right", _handle_tcp(0.0) + aside * SIDE_IN, rot0, "beside the handle")
    out += [_w(1.0, _handle_tcp(0.0) + aside * SIDE_IN, OPEN_H, "open", rot0),
            _w(1.2, _handle_tcp(0.0), OPEN_H, "fingers round the knob", rot0),
            _w(0.5, _handle_tcp(0.0), FINGER_CLOSED, "take the handle", rot0)]
    turned = 0.0
    while _depth_after(turned) < DEPTH_PLAN - 1e-6:
        step = min(STROKE, CATCH + ((DEPTH_PLAN - X0_PLAN) / PITCH) * 2 * math.pi - turned)
        for j in range(1, 5):
            done = turned + step * j / 4
            out.append(_w(TURN_S * step / STROKE / 4, _handle_tcp(_depth_after(done)), FINGER_CLOSED,
                          "turn the handle clockwise", _side(ROLL0 + step * j / 4)))
        turned += step
        at = _handle_tcp(_depth_after(turned))
        out.append(_w(RELEASE_S, at, OPEN_H, "let go", _side(ROLL0 + step)))
        if _depth_after(turned) < DEPTH_PLAN - 1e-6:
            for j in range(1, 5):
                out.append(_w(BACK_S / 4, at, OPEN_H, "turn back", _side(ROLL0 + step * (1 - j / 4))))
            out.append(_w(REGRIP_S, at, FINGER_CLOSED, "take the handle again", rot0))
    end_roll = ROLL0 + step
    at = _handle_tcp(DEPTH_PLAN)
    out.append(_w(1.0, at + OUT * 0.03, OPEN_H, "back off the handle", _side(end_roll)))
    out.append(_w(1.0, at + OUT * 0.03 + [0.0, 0.0, 0.04], FINGER_CLOSED, "up", _side(end_roll)))
    # to the front wing's paddle: turn the hand to point down while clear of the wings, then over the paddle,
    # down beside it, and settle
    p0, _ = _push_point(PHI_AIM, 1, PUSH_BACK + 0.006)
    yaw0 = _push_yaw(PHI_AIM, 1, None)
    up = _reachable_lift("right", p0, grip_rotation(yaw0))
    clear = at + OUT * 0.06 + FRONT * 0.05
    clear[2] = p0[2] + up
    out.append(_w(2.0, clear, FINGER_CLOSED, "turn the hand down, clear of the wings", grip_rotation(yaw0)))
    out.append(_w(1.5, p0 + [0.0, 0.0, up], FINGER_CLOSED, "over the front wing", grip_rotation(yaw0)))
    out.append(_w(1.5, p0, FINGER_CLOSED, "down beside its paddle", grip_rotation(yaw0)))
    out.append(_w(1.5, p0, FINGER_CLOSED, "settle", grip_rotation(yaw0)))
    t_press = sum(w.duration for w in out)
    press, yaw = _press("right", 1, PHI_AIM, PHI_TWO, PRESS_S, 28, PUSH_LEAD, yaw0)
    out += press
    last = np.array(out[-1].pos)
    up = _reachable_lift("right", last, grip_rotation(yaw))
    out.append(_w(1.0, last + [0.0, 0.0, up], FINGER_CLOSED, "lift off the wing", grip_rotation(yaw)))
    out.append(_w(FINISH_S, last + [0.0, 0.0, up], FINGER_CLOSED, "wait", grip_rotation(yaw)))
    # take the corkscrew off the bottle by its bell (wings down), with the cork on its worm, and lay it on the table
    w_ang = math.atan2(FRONT[1], FRONT[0])  # the fingers close across the bell, toward the wings
    yaw_b = min((w_ang - math.pi / 2, w_ang + math.pi / 2), key=abs)  # grip_rotation(0) closes along y
    rot_b = grip_rotation(yaw_b)
    bell = _c(BELL_GRIP_U) + [0.0, 0.0, BELL_GRIP_Z]
    out += [_w(1.5, bell + [0.0, 0.0, 0.06], FINGER_OPEN, "over the bell", rot_b),
            _w(1.2, bell, FINGER_OPEN, "down round the bell", rot_b),
            _w(0.6, bell, FINGER_CLOSED, "take the corkscrew", rot_b),
            _w(1.5, bell + OUT * 0.05, FINGER_CLOSED, "draw it off the bottle with the cork", rot_b),
            _w(1.2, bell + OUT * 0.05 + [0.0, 0.0, 0.05], FINGER_CLOSED, "lift it clear", rot_b)]
    put = PUT
    out += [_w(2.0, put + [0.0, 0.0, 0.04], FINGER_CLOSED, "carry it aside", rot_b),
            _w(1.2, put, FINGER_CLOSED, "lower it to the table", rot_b),
            _w(0.6, put, FINGER_OPEN, "let go", rot_b),
            _w(1.0, put + [0.0, 0.0, 0.06], FINGER_OPEN, "up", rot_b)]
    out += _home("right", put + [0.0, 0.0, 0.06], rot_b, "arm home")
    out.append(_w(3.0, out[-1].pos, FINGER_CLOSED, "rest", out[-1].yaw))
    return out, t_press


def _left(t_press: float, total: float) -> list[Waypoint]:
    p0, _ = _push_point(PHI_AIM, -1, PUSH_BACK + 0.006)
    yaw0 = _push_yaw(PHI_AIM, -1, None)
    rot = grip_rotation(yaw0)
    up = _reachable_lift("left", p0, rot)
    high = p0 + [0.0, 0.0, up] - FRONT * 0.06  # out beyond the back wing, clear of it
    lead = _lead_in("left", high, rot, "out beyond the back wing")
    t_lead = sum(w.duration for w in lead)
    out = [_w(max(t_press - t_lead - 4.5, 0.5), _left_ready(), FINGER_CLOSED, "wait", _left_ready_rot())]
    out += lead
    out.append(_w(1.5, p0 + [0.0, 0.0, up], FINGER_CLOSED, "over the back wing", rot))
    out.append(_w(1.5, p0, FINGER_CLOSED, "down beside its paddle", rot))
    out.append(_w(max(t_press - sum(w.duration for w in out), 0.5), p0, FINGER_CLOSED, "settle", rot))
    press, yaw = _press("left", -1, PHI_AIM, PHI_TWO, PRESS_S, 28, PUSH_LEAD, yaw0)
    out += press
    finish, yaw = _press("left", -1, PHI_TWO, PHI_DOWN, FINISH_S, 10, PUSH_LEAD, yaw)
    out += finish
    last = np.array(out[-1].pos)
    up = _reachable_lift("left", last, grip_rotation(yaw))
    out.append(_w(1.0, last + [0.0, 0.0, up], FINGER_CLOSED, "lift off the wing", grip_rotation(yaw)))
    out += _home("left", last + [0.0, 0.0, up], grip_rotation(yaw), "arm home")
    used = sum(w.duration for w in out)
    out.append(_w(max(total - used, 0.5), out[-1].pos, FINGER_CLOSED, "rest", out[-1].yaw))
    return out


def _left_ready():
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES["left"])[0]


def _left_ready_rot():
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES["left"])[1]


def demo():
    right, t_press = _right()
    total = sum(w.duration for w in right)
    return {"left": _left(t_press, total), "right": right}
