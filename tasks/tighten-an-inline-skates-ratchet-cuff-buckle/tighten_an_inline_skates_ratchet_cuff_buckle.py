"""On the station table: a whole adult inline skate (a left Rollerblade Zetrablade, men's US 10 / EU 43, 80 mm wheels)
stands upright with its four wheels in a wooden wheel chock, its toe toward the robot's left, its outer side toward
the robot. A foam boot shaper fills the boot. The cuff closes with its cuff buckle: a toothed plastic ladder strap
riveted to the cuff's inner side wraps round the front of the cuff and enters a ratchet buckle riveted to the cuff's
outer side, facing the robot. The strap's tip has been pushed into the buckle's mouth, past the catch, as people
leave it when they put the skate on; the cuff is still loose. The buckle's red lever stands open, straight out from
the cuff toward the robot.

Task (instruction): close the skate's cuff buckle snugly. The instruction does not say how. A ratchet buckle is
tightened by pumping its lever: each time the lever is swung back toward the cuff, a drive pawl on it draws the ladder
into the buckle, and a holding catch in the buckle stops the ladder sliding back out while the lever is lifted for
the next stroke. When the cuff is snug the lever is folded flat, where it stays. Swung right open (past 120 degrees)
the lever lifts the catch and lets the ladder go: that is how the buckle is undone.

One arm: the skate stands in the chock and needs no holding. The hand skill is pumping a small lever (55 mm long)
through repeated arcs about its hinge pin against the strap's rising pull, turning with it, then folding it flat.

The skate is drawn in its own frame (x from heel to toe, y toward its outer side) and placed turned 90 degrees.
World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z TOP = 0.7475;
heights below are above the table.
- chock: two pine rails 35 x 2 x 1.5 cm along y, either side of the wheels (x 0.266 to 0.286 and 0.314 to 0.334,
  y -0.15 to 0.20), fixed to the table.
- skate: boot 32 cm long (heel at y -0.11, toe at y 0.21), 9.6 cm wide (x 0.252 to 0.348), on a 30 cm frame with four
  80 mm wheels (axles 81 mm apart at y -0.096, -0.015, 0.066, 0.147); cuff (an elliptic shell 12.4 cm along the foot,
  10.8 cm across) round (0.30, -0.03), 15 to 28.5 cm high; liner collar and boot shaper to 30 cm. Held fixed (the
  chock keeps it upright; lifting it out is not modelled).
- buckle base on the cuff's outer side: y -0.055 to 0.000 (its mouth, where the ladder enters, at y 0.000), outer
  face at x 0.242, centred 23.5 cm high.
- ladder strap: 15 mm tall, 2.5 mm thick, 5 mm tooth pitch; its tip starts at y -0.045 (4.5 cm into the buckle); it
  slides along y (drawn in toward -y).
- lever: 55 mm long, 18 mm tall, 6 mm thick (9 mm over its 14 mm grip end), on a vertical hinge pin at (0.236,
  -0.003), 23.5 cm high; folded flat it lies back along the ladder (tip at y -0.058); it starts 90 degrees open,
  pointing at the robot (tip at x 0.181).

Modelled as rules (README "What is modelled"): the ladder is a slide joint; the cuff and the boot shaper pulling it
back out are a spring on it (3 N at the start, plus 0.8 N per mm drawn in). The holding catch is the slide's outward
limit, moved in one tooth at a time as teeth pass it. The drive pawl is a rule: while the lever is swung toward the
cuff, the ladder is driven in by 9.55 mm of strap per radian of lever (one tooth per 30 degrees) from where that
stroke began, and the lever's hinge friction is set to the strap's pull times that radius, so the hand feels the
strap. Lifting the lever, the pawl rides back over the teeth and the ladder settles on the last tooth the catch took.

Score (the buckle's state at the end of the trial): 0.1 per tooth drawn in past the catch, up to 0.6 at 6 teeth
(30 mm: snug); 0.8 for 6 or more teeth with the lever folded flat (within 5 degrees); 1.0 for that with both
grippers 4 cm or more from the lever, the buckle and the ladder's tip, and the lever and ladder still for 1 s. A
ladder let go (lever swung past 120 degrees) slides out of the buckle and counts no teeth.
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, plate  # noqa: E402
from strapkit import Part, Strap, ids, set_drive, set_friction, set_limits  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
# The score is the buckle's state at the end (teeth can be let go again), so it is read at the end.
SCORE_OVER = "end"

# -- colours --------------------------------------------------------------------------------------------------------
SHELL = (0.07, 0.07, 0.08)  # black polypropylene shell
SHELL_HI = (0.62, 0.63, 0.65)  # silver trim
LINER = (0.36, 0.37, 0.40)  # grey liner
LINER_IN = (0.16, 0.16, 0.18)  # inside the boot
SOLE = (0.12, 0.12, 0.13)
FRAME = (0.09, 0.09, 0.10)
WHEEL = (0.92, 0.92, 0.90)  # 82A urethane
HUB = (0.30, 0.30, 0.32)
LACE = (0.85, 0.85, 0.85)
STRAPC = (0.07, 0.07, 0.08)  # black ladder strap
TOOTH = (0.22, 0.22, 0.24)
BUCKLE = (0.70, 0.71, 0.73)  # aluminium buckle base
LEVER = (0.80, 0.10, 0.10)  # red lever
GRIPC = (0.25, 0.04, 0.04)
PIN = (0.55, 0.55, 0.57)
WOOD = (0.72, 0.56, 0.36)
FOAM = (0.85, 0.78, 0.62)  # boot shaper

# -- layout -----------------------------------------------------------------------------------------------------------
# The skate is drawn in its own frame (x along the foot from heel to toe, y toward its outer side, heights above the
# table), with the cuff's centre at (XC, YC), and placed on the table turned 90 degrees: toe toward +y, outer side
# (with the buckle) toward the robot (-x), the cuff's centre at (XN, YN).
XC, YC = 0.21, 0.0  # cuff centre (skate frame)
XN, YN = 0.30, -0.03  # cuff centre (world)
CUFF_A, CUFF_B = 0.062, 0.054  # cuff half-lengths (x, y)
CUFF_Z0, CUFF_Z1 = 0.150, 0.285
H_B = 0.235  # buckle centre height above the table
Z_B = TOP + H_B
YF = 0.058  # buckle base outer face
BASE_X0, BASE_X1 = XC - 0.025, XC + 0.030  # buckle base (mouth at BASE_X1)
LAD_T, LAD_H = 0.0025, 0.015  # ladder thickness (y) and height (z)
LAD_Y = YF + LAD_T / 2
PITCH = 0.005
TIP0 = XC - 0.015  # the ladder's tip at the start (4.5 cm into the buckle)
LAD_FRONT = XC + 0.050  # the moving ladder's front end (under the fixed wrap beyond the mouth)
LEV_L, LEV_H, LEV_T = 0.055, 0.018, 0.006
LEV_GRIP_L, LEV_GRIP_T = 0.014, 0.009
XH, YH = XC + 0.027, YF + LAD_T + 0.0005 + LEV_T / 2  # the lever's hinge pin (vertical)
THETA0 = 90.0  # the lever starts this far open (degrees)

# -- the buckle's rules ---------------------------------------------------------------------------------------------
R_DRIVE = PITCH / math.radians(30.0)  # m of strap per radian of lever: one tooth per 30 degrees
T0, K_CUFF = 3.0, 800.0  # the strap's tension at the start (N) and its rise (N/m drawn in)
K_DRIVE, D_DRIVE = 20000.0, 60.0  # the drive pawl pulling the ladder during a stroke
LEVER_F0 = 0.02  # the lever's own hinge friction (N m): it stays where it is left
D_MAX = 0.045  # the most the ladder can be drawn in (the boot shaper fully squeezed): 9 teeth
Q_OUT = 0.050  # the ladder's position let go (its tip 5 mm out of the mouth)
RELEASE_DEG = 120.0  # the lever swung this far open lifts the catch
N_SNUG = 6  # teeth for a snug cuff (30 mm)
FLAT_DEG = 5.0
CLEAR = 0.040
STILL_STEPS, STILL_DEG, STILL_MM = 20, 1.0, 0.5
CONTROL_HZ = 20.0

_DEBUG = os.environ.get("SKATE_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("SKATE_DEBUG_EVERY", "10"))


def _box(x0, x1, y0, y1, z0, z1, color, origin, collide=True) -> BoxShape:
    """A box from world corners (heights above the table), in the frame of a part whose origin is ``origin``."""
    return BoxShape(((x0 + x1) / 2 - origin[0], (y0 + y1) / 2 - origin[1], (z0 + z1) / 2 + TOP - origin[2]),
                    (abs(x1 - x0), abs(y1 - y0), abs(z1 - z0)), color, collide=collide)


def _cyl(x, y, h, r, height, color, origin, axis="Z", collide=True) -> CylShape:
    return CylShape((x - origin[0], y - origin[1], h + TOP - origin[2]), r, height, color, collide, axis)


def _pl(p1, p2, width, thick, normal, color, origin, collide=True) -> BoxShape:
    o = np.array([origin[0], origin[1], origin[2] - TOP])
    return plate(np.asarray(p1, float) - o, np.asarray(p2, float) - o, width, thick, normal, color, collide)


def _ell(a: float, b: float, deg: float) -> np.ndarray:
    t = math.radians(deg)
    return np.array([XC + a * math.cos(t), YC + b * math.sin(t)])


def W(p) -> np.ndarray:
    """A point of the skate frame in the world."""
    p = np.asarray(p, float)
    return np.array([XN - (p[1] - YC), YN + (p[0] - XC), p[2]])


def W_dir(d) -> np.ndarray:
    d = np.asarray(d, float)
    return np.array([-d[1], d[0], d[2]])


R90 = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
Q90 = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def _turn(shape):
    """A shape of a part drawn in the skate frame, turned with it into the world (offsets from the part's origin)."""
    c = tuple(float(v) for v in R90 @ np.asarray(shape.centre, float))
    if isinstance(shape, CylShape):
        axis = {"X": "Y", "Y": "X", "Z": "Z"}[shape.axis]
        return CylShape(c, shape.radius, shape.height, shape.color, shape.collide, axis)
    return BoxShape(c, shape.size, shape.color, collide=shape.collide, quat_xyzw=_qmul(Q90, shape.quat()))


SK_O = (XC, YC, TOP)  # the skate's origin
WHEELS = (XC - 0.066, XC + 0.015, XC + 0.096, XC + 0.177)  # axles 81 mm apart; the rear wheel stands out behind the heel


def _skate_shapes() -> list:
    o = SK_O
    out = []
    # wheel chock: two pine rails either side of the wheels
    for s in (-1.0, 1.0):
        out.append(_box(XC - 0.12, XC + 0.23, s * 0.014, s * 0.034, 0.0, 0.015, WOOD, o))
    # wheels and frame
    for xw in WHEELS:
        out.append(_cyl(xw, 0.0, 0.040, 0.040, 0.024, WHEEL, o, "Y"))
        out.append(_cyl(xw, 0.0, 0.040, 0.016, 0.0262, HUB, o, "Y", collide=False))
        out.append(_cyl(xw, 0.0, 0.040, 0.007, 0.041, FRAME, o, "Y", collide=False))  # axle boss
        for s in (-1.0, 1.0):
            out.append(_box(xw - 0.007, xw + 0.007, s * 0.0135, s * 0.0185, 0.040, 0.060, FRAME, o, collide=False))
    for s in (-1.0, 1.0):
        out.append(_box(XC - 0.095, XC + 0.205, s * 0.0135, s * 0.0185, 0.058, 0.092, FRAME, o))
    out.append(_box(XC - 0.085, XC + 0.200, -0.0185, 0.0185, 0.084, 0.094, FRAME, o))
    # sole, heel and toe of the boot
    out.append(_box(XC - 0.080, XC + 0.200, -0.044, 0.044, 0.094, 0.108, SOLE, o))
    out.append(_cyl(XC + 0.195, 0.0, 0.101, 0.044, 0.014, SOLE, o))
    out.append(_box(XC - 0.078, XC + 0.060, -0.048, 0.048, 0.108, 0.160, SHELL, o))
    out.append(_box(XC + 0.060, XC + 0.195, -0.045, 0.045, 0.108, 0.150, SHELL, o))
    out.append(_cyl(XC + 0.195, 0.0, 0.129, 0.045, 0.042, SHELL, o))
    out.append(_box(XC + 0.100, XC + 0.200, -0.030, 0.030, 0.150, 0.156, LINER, o, collide=False))
    for s in (-1.0, 1.0):  # silver side panels and the heel counter's trim
        out.append(_box(XC - 0.060, XC + 0.150, s * 0.0451, s * 0.0462, 0.115, 0.135, SHELL_HI, o, collide=False))
        out.append(_box(XC - 0.070, XC + 0.050, s * 0.0481, s * 0.0492, 0.140, 0.150, SHELL_HI, o, collide=False))
    # liner tongue rising in front of the cuff, laces across the vamp, the 45-degree instep strap
    out.append(_pl((XC + 0.110, 0.0, 0.150), (XC + 0.060, 0.0, 0.292), 0.058, 0.016, (1.0, 0.0, 0.35), LINER, o))
    for k in range(4):
        x = XC + 0.165 - 0.022 * k
        z = 0.152 + 0.004 * k
        out.append(_pl((x - 0.004, -0.030, z), (x + 0.004, 0.030, z), 0.004, 0.003, (0.0, 0.0, 1.0), LACE, o, False))
        out.append(_pl((x + 0.004, -0.030, z), (x - 0.004, 0.030, z), 0.004, 0.003, (0.0, 0.0, 1.0), LACE, o, False))
    out.append(_pl((XC + 0.090, -0.052, 0.150), (XC + 0.065, 0.052, 0.205), 0.030, 0.004, (1.0, 0.0, 1.0), SHELL_HI,
                   o, False))
    # the cuff: an elliptic shell open at the front (the tongue), riveted to the boot at the ankle
    angles = np.linspace(38.0, 322.0, 17)
    for a0, a1 in zip(angles[:-1], angles[1:]):
        p0, p1 = _ell(CUFF_A, CUFF_B, a0), _ell(CUFF_A, CUFF_B, a1)
        mid = (a0 + a1) / 2
        n = (math.cos(math.radians(mid)) / CUFF_A, math.sin(math.radians(mid)) / CUFF_B, 0.0)
        zc = (CUFF_Z0 + CUFF_Z1) / 2
        out.append(_pl((*p0, zc), (*p1, zc), CUFF_Z1 - CUFF_Z0, 0.004, n, SHELL, o))
        q0, q1 = _ell(CUFF_A - 0.004, CUFF_B - 0.004, a0), _ell(CUFF_A - 0.004, CUFF_B - 0.004, a1)
        out.append(_pl((*q0, CUFF_Z1 + 0.008), (*q1, CUFF_Z1 + 0.008), 0.020, 0.010, n, LINER, o))
    out.append(_cyl(XC, YC, 0.195, 0.046, 0.170, FOAM, o, collide=False))  # the boot shaper inside the boot
    out.append(_cyl(XC, YC, 0.2915, 0.049, 0.003, LINER_IN, o, collide=False))  # the boot's dark opening
    out.append(_cyl(XC - 0.005, 0.0, 0.158, 0.008, 2 * CUFF_B + 0.012, SHELL_HI, o, "Y", collide=False))  # rivets
    # silver trim band round the cuff's top
    for a0, a1 in zip(angles[:-1], angles[1:]):
        p0, p1 = _ell(CUFF_A + 0.0022, CUFF_B + 0.0022, a0), _ell(CUFF_A + 0.0022, CUFF_B + 0.0022, a1)
        mid = (a0 + a1) / 2
        n = (math.cos(math.radians(mid)) / CUFF_A, math.sin(math.radians(mid)) / CUFF_B, 0.0)
        out.append(_pl((*p0, CUFF_Z1 - 0.010), (*p1, CUFF_Z1 - 0.010), 0.008, 0.001, n, SHELL_HI, o, False))
    # buckle base (aluminium, riveted to the cuff), with its mouth's top and bottom guides
    out.append(_box(BASE_X0, BASE_X1, 0.044, YF, H_B - 0.011, H_B + 0.011, BUCKLE, o))
    for s in (-1.0, 1.0):
        out.append(_box(BASE_X0 + 0.004, BASE_X1, YF, YF + LAD_T + 0.0008, H_B + s * (LAD_H / 2 + 0.0005),
                        H_B + s * (LAD_H / 2 + 0.0035), BUCKLE, o))
    out.append(_cyl(XH, YH, H_B, 0.0045, 0.024, PIN, o, collide=False))  # the lever's hinge knuckle
    # the ladder strap's fixed part: from the mouth round the front of the cuff to its rivet on the medial side
    ys = YF + LAD_T + 0.0006
    out.append(_pl((BASE_X1, ys, H_B), (LAD_FRONT + 0.002, ys, H_B), LAD_H + 0.001, 0.0012, (0.0, 1.0, 0.0), STRAPC,
                   o, False))
    pts = [np.array([LAD_FRONT + 0.002, ys])] + [_ell(CUFF_A + 0.006, CUFF_B + 0.008, a)
                                                  for a in (40.0, 20.0, 0.0, -20.0, -40.0, -60.0)]
    for p0, p1 in zip(pts[:-1], pts[1:]):
        m = (p0 + p1) / 2 - np.array([XC, YC])
        out.append(_pl((*p0, H_B), (*p1, H_B), LAD_H, LAD_T, (m[0] / CUFF_A ** 2, m[1] / CUFF_B ** 2, 0.0), STRAPC,
                       o, False))
    rv = _ell(CUFF_A + 0.007, CUFF_B + 0.009, -60.0)
    out.append(_cyl(rv[0], rv[1], H_B, 0.005, 0.004, SHELL_HI, o, collide=False))
    return out


LAD_O = ((TIP0 + LAD_FRONT) / 2, LAD_Y, TOP + H_B)


def _ladder_shapes() -> list:
    o = LAD_O
    out = [_box(TIP0, LAD_FRONT, YF, YF + LAD_T, H_B - LAD_H / 2, H_B + LAD_H / 2, STRAPC, o)]
    x = TIP0 + 0.003
    while x < LAD_FRONT - 0.002:  # the ladder's teeth (ridges every 5 mm)
        out.append(_box(x - 0.0007, x + 0.0007, YF + LAD_T, YF + LAD_T + 0.0006, H_B - LAD_H / 2 + 0.002,
                        H_B + LAD_H / 2 - 0.002, TOOTH, o, collide=False))
        x += PITCH
    return out


LEV_O = (XH, YH, TOP + H_B)


def _lever_shapes() -> list:
    """The lever in its start pose: 90 degrees open, pointing +y from its hinge."""
    o = LEV_O
    out = [_box(XH - LEV_T / 2, XH + LEV_T / 2, YH, YH + LEV_L - LEV_GRIP_L, H_B - LEV_H / 2, H_B + LEV_H / 2, LEVER,
                o),
           _box(XH - LEV_GRIP_T / 2, XH + LEV_GRIP_T / 2, YH + LEV_L - LEV_GRIP_L, YH + LEV_L, H_B - LEV_H / 2,
                H_B + LEV_H / 2, LEVER, o),
           _cyl(XH, YH, H_B, 0.0042, LEV_H, LEVER, o)]
    for k in range(3):  # grip ribs (shown only)
        y = YH + LEV_L - 0.003 - 0.004 * k
        out.append(_box(XH - LEV_GRIP_T / 2 - 0.0004, XH + LEV_GRIP_T / 2 + 0.0004, y - 0.0008, y + 0.0008,
                        H_B - LEV_H / 2 + 0.002, H_B + LEV_H / 2 - 0.002, GRIPC, o, collide=False))
    return out


def _wt(p) -> tuple:
    return tuple(float(v) for v in W(p))


SKATE = Strap.build("skate", parts=[
    Part("skate", tuple(_turn(sh) for sh in _skate_shapes()), 1.7, _wt(SK_O), material=(0.7, 0.6)),
    Part("ladder", tuple(_turn(sh) for sh in _ladder_shapes()), 0.012, _wt(LAD_O), parent="skate",
         joint="prismatic", axis="Y", anchor=_wt(LAD_O), limits=(-D_MAX, 0.0), stiffness=K_CUFF, damping=5.0,
         friction=0.3),
    Part("lever", tuple(_turn(sh) for sh in _lever_shapes()), 0.018, _wt(LEV_O), parent="skate", joint="revolute",
         axis="Z", anchor=_wt(LEV_O), limits=(-(130.0 - THETA0), THETA0 + 0.5), damping=0.002, friction=LEVER_F0,
         material=(0.9, 0.8)),
], filters=(("lever", "ladder"),), color=SHELL, floating=False, self_collision=True, solver_iterations=32)
OBJECTS = [SKATE]

# -- tracking -------------------------------------------------------------------------------------------------------
_EP: dict = {}
_IDS: dict = {}


@on_reset
def _reset() -> None:
    _EP.clear()
    _EP.update(tick=0, step=0, counted=-1, init=False, theta_prev=THETA0, theta_top=THETA0, pushing=False,
               held=0, drawn=0.0, drive=None, fric=None, lim=None, released=False, out=False, teeth=0, theta=THETA0,
               q=0.0, tension=T0, near=1.0, arms=[], hist=[], stage=0.0, strokes=0, max_teeth=0, max_theta=THETA0)
    _IDS.pop("left", None)
    _IDS.pop("right", None)


_reset()


def _grippers(base) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS

    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        if side not in _IDS:
            _IDS[side] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
        q = as_numpy(robot.data.joint_pos)[0]
        out.append(np.asarray(ik.tcp_pose(q[_IDS[side]], station.BASES[side])[0], float))
    return out


def _tension(drawn: float) -> float:
    return T0 + K_CUFF * max(0.0, drawn)


def _set_ladder(env, k: float, d: float, target: float) -> None:
    key = (round(k, 2), round(d, 2), round(target, 5))
    if _EP["drive"] != key:
        set_drive(env, SKATE, "ladder", k, d, target)
        _EP["drive"] = key


def _set_lim(env, low: float, high: float) -> None:
    key = (round(low, 5), round(high, 5))
    if _EP["lim"] != key:
        set_limits(env, SKATE, "ladder", low, high)
        _EP["lim"] = key


def _rules(env, q: float, theta: float) -> None:
    """The catch, the drive pawl and the release (see the module docstring). ``q``: the ladder's slide (m, + out);
    ``theta``: the lever's opening (degrees)."""
    drawn = -q
    if theta >= RELEASE_DEG:  # the lever swung right open lifts the catch: the cuff pushes the ladder out
        _EP["released"] = True
        _EP["pushing"] = False
        _set_lim(env, -D_MAX, Q_OUT)
        _set_ladder(env, K_CUFF, 5.0, Q_OUT + T0 / K_CUFF)
        _EP["theta_top"] = theta
    else:
        if _EP["released"]:
            _EP["released"] = False
            _EP["out"] = q > Q_OUT - 2 * PITCH  # the tip is out at the mouth: the catch has nothing to take
            _EP["held"] = int(math.floor((drawn + 0.0015) / PITCH))
        if _EP["out"]:
            _set_lim(env, -D_MAX, Q_OUT)
            _set_ladder(env, K_CUFF, 5.0, Q_OUT + T0 / K_CUFF)
            _EP["pushing"] = False
        else:
            dth = theta - _EP["theta_prev"]
            if dth > 0.3:  # lifting: the drive pawl rides back over the teeth, the catch holds the ladder
                if _EP["pushing"]:
                    _EP["pushing"] = False
                    _EP["theta_top"] = theta
                _EP["theta_top"] = max(_EP["theta_top"], theta)
            elif dth < -0.3 and theta < _EP["theta_top"]:  # swung toward the cuff: the pawl draws the ladder in
                if not _EP["pushing"]:
                    _EP["strokes"] += 1
                    _EP["d_start"] = _EP["held"] * PITCH
                _EP["pushing"] = True
            if _EP["pushing"]:
                want = min(D_MAX, _EP["d_start"] + R_DRIVE * math.radians(_EP["theta_top"] - max(theta, 0.0)))
                _set_ladder(env, K_DRIVE, D_DRIVE, -want)
            else:
                _set_ladder(env, K_CUFF, 5.0, T0 / K_CUFF)
            # the catch: it takes each tooth that passes it, so the ladder cannot slide back out past it
            _EP["held"] = max(_EP["held"], int(math.floor((drawn + 0.0015) / PITCH)))
            _set_lim(env, -D_MAX, -_EP["held"] * PITCH)
    tension = _tension(drawn)
    fr = LEVER_F0 + (tension * R_DRIVE if _EP["pushing"] else 0.0)
    if _EP["fric"] is None or abs(fr - _EP["fric"]) > 0.05 * max(fr, 0.02):
        set_friction(env, SKATE, "lever", fr)
        _EP["fric"] = fr
    _EP.update(theta_prev=theta, tension=tension, drawn=drawn)


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("SKATE_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _lever_dir(theta_deg: float) -> np.ndarray:
    t = math.radians(theta_deg)
    return np.array([-math.cos(t), math.sin(t), 0.0])


def _track_inner(env) -> None:
    _EP["tick"] += 1
    base = getattr(env, "unwrapped", env)
    asset = base.scene["skate"]
    idx = ids(asset, SKATE)
    qj = as_numpy(asset.data.joint_pos)[0]
    q = float(qj[idx["joint"]["ladder"]])
    theta = THETA0 - math.degrees(float(qj[idx["joint"]["lever"]]))
    if not (np.isfinite(q) and np.isfinite(theta)):
        return
    _rules(env, q, theta)
    teeth = 0 if (_EP["out"] or _EP["released"]) else max(0, _EP["held"])
    hinge = np.array([XH, YH, Z_B])
    pts = [hinge + _lever_dir(theta) * s for s in (0.0, 0.5 * LEV_L, LEV_L)]
    pts += [np.array([BASE_X0, YF, Z_B]), np.array([TIP0 - q, LAD_Y, Z_B])]
    pts = [W(p) for p in pts]
    arms = _grippers(base)
    near = min(float(np.linalg.norm(t - p)) for t in arms for p in pts)
    _EP.update(q=q, theta=theta, teeth=teeth, near=near, arms=arms, max_teeth=max(_EP["max_teeth"], teeth),
               max_theta=max(_EP["max_theta"], theta))
    _EP["hist"].append(np.array([theta / STILL_DEG, q * 1000 / STILL_MM]))
    del _EP["hist"][:-STILL_STEPS]
    _EP["stage"] = _stage()


def _still() -> bool:
    h = _EP["hist"]
    return len(h) == STILL_STEPS and max(float(np.abs(v - h[-1]).max()) for v in h) < 1.0


def _stage() -> float:
    n = _EP["teeth"]
    if n >= N_SNUG and _EP["theta"] <= FLAT_DEG:
        return 1.0 if (_EP["near"] >= CLEAR and _still()) else 0.8
    return round(0.1 * min(n, N_SNUG), 3)


def _debug() -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    print(f"DBG step={_EP['step']} theta={_EP['theta']:.1f}deg q={_EP['q'] * 1000:+.2f}mm teeth={_EP['teeth']} "
          f"held={_EP['held']} pushing={_EP['pushing']} top={_EP['theta_top']:.1f} T={_EP['tension']:.1f}N "
          f"fric={_EP['fric'] or 0.0:.3f} strokes={_EP['strokes']} rel={_EP['released']} out={_EP['out']} "
          f"near={_EP['near']:.3f} still={_still()} stage={_EP['stage']}", flush=True)
    if _EP["arms"]:
        print("DBG   " + " | ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f})"
                                    for k, v in zip(("left", "right"), _EP["arms"])), flush=True)


def score(objects) -> float:
    """The buckle's state now: 0.1 per tooth drawn in past the catch (up to 0.6 at 6 teeth); 0.8 with 6 or more
    teeth and the lever folded flat; 1.0 (success) with that, both grippers clear and everything still for 1 s."""
    if _DEBUG and _EP["counted"] != _EP["tick"]:
        _EP["counted"] = _EP["tick"]
        _debug()
    return float(_EP["stage"])


def success(objects) -> bool:
    """6 or more teeth held, the lever folded flat, both grippers 4 cm or more from it and the buckle, still 1 s."""
    return _EP["stage"] >= 1.0


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right gripper, level and pointing along the lever toward its hinge, takes the lever's grip end between its
# fingers (top and bottom edges) and pumps it: swings it toward the cuff to 20 degrees, lifts it back to 85 degrees,
# three times, turning with it about the hinge pin (the left arm stays put). Then it lets go, comes round to the
# lever's outer face and pushes it flat with its closed fingertips, and moves away. SKATE_WRONG=1 runs it the wrong way: the lever is first swung
# right open (to its 130 degree stop), which lets the ladder go, and then folded flat.
WRONG = os.environ.get("SKATE_WRONG") == "1"
N_STROKES = int(os.environ.get("SKATE_STROKES", "3"))
LOW_DEG = float(os.environ.get("SKATE_LOW", "20"))
HIGH_DEG = float(os.environ.get("SKATE_HIGH", "85"))
FWD_FLAT = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])  # level, pointing +x, closing along z
OPEN = -0.030
PINCH = -(LEV_H / 2 - 0.003)
GRIP_IN = float(os.environ.get("SKATE_GRIP_IN", "0.011"))  # TCP this far in from the lever's tip
PUSH_AT = 0.038  # the push on the lever's outer face, this far from the hinge
PUSH_PAST = float(os.environ.get("SKATE_PUSH_PAST", "0.020"))
TIP_PAST_TCP = 0.005


def _rz(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


HINGE = np.array([XH, YH, Z_B])


def _grip_pose(theta: float, out: float = 0.0):
    """The TCP on the lever's grip end at opening ``theta``, pointing along the lever toward the hinge."""
    u = _lever_dir(theta)
    return W(HINGE + u * (LEV_L - GRIP_IN + out)), R90 @ _rz(-min(theta, 90.0)) @ FWD_FLAT


def _good_ik(arm: str, pos, rot):
    from yam_arena import ik, station

    best = None
    for sd in itertools.product((-0.4, 0.0, 0.4), (0.8, 1.3), (0.6, 1.2), (-0.8, 0.8), (-1.2, 0.0, 1.2),
                                (-1.5, 0.0, 1.5)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(sd, float),
                             base_pos=station.BASES[arm], iters=150)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
                if room > 0.3:
                    break
    if best is None:
        raise ValueError(f"no IK solution for the {arm} arm at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, finger: float, label: str, seconds: float = 3.0, steps: int = 15) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(seconds / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, p, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in p), finger, label, rot)


def _swing(a: float, b: float, finger: float, label: str, deg_per_s: float = 50.0) -> list:
    n = max(2, int(math.ceil(abs(b - a) / 5.0)))
    out = []
    for k in range(1, n + 1):
        th = a + (b - a) * k / n
        p, r = _grip_pose(th)
        out.append(_w(abs(b - a) / n / deg_per_s, p, finger, label, r))
    return out


ARM = "right"


def _pump() -> list:
    p_pre, r0 = _grip_pose(THETA0, 0.020)
    out = _lead_in(ARM, p_pre, r0, OPEN, "level, in line with the open lever, beyond its grip end")
    p0, _ = _grip_pose(THETA0)
    out += [_w(1.2, p0, OPEN, "fingers over and under the lever's grip end", r0),
            _w(0.3, p0, OPEN, "settle", r0),
            _w(0.8, p0, PINCH, "take the lever's grip end", r0),
            _w(0.4, p0, PINCH, "settle", r0)]
    theta = THETA0
    if WRONG:
        out += _swing(theta, 128.0, PINCH, "swing the lever right open (lets the ladder go)")
        out += [_w(0.8, _grip_pose(128.0)[0], PINCH, "wait", _grip_pose(128.0)[1])]
        theta = 128.0
    for k in range(N_STROKES):
        out += _swing(theta, LOW_DEG, PINCH, f"stroke {k + 1}: swing the lever toward the cuff")
        out += [_w(0.4, _grip_pose(LOW_DEG)[0], PINCH, "settle", _grip_pose(LOW_DEG)[1])]
        theta = LOW_DEG
        if k < N_STROKES - 1:
            out += _swing(LOW_DEG, HIGH_DEG, PINCH, "lift the lever for the next stroke")
            theta = HIGH_DEG
    p_low, r_low = _grip_pose(LOW_DEG)
    out += [_w(0.8, p_low, OPEN, "let go", r_low),
            _w(1.0, _grip_pose(LOW_DEG, 0.035)[0], OPEN, "back off along the lever", r_low)]
    # push the lever flat with the closed fingertips on its outer face, pointing at the cuff (-y)
    r_push = R90 @ _rz(-90.0) @ FWD_FLAT
    t = math.radians(LOW_DEG)
    face = HINGE + _lever_dir(LOW_DEG) * PUSH_AT + np.array([math.sin(t), math.cos(t), 0.0]) * (LEV_T / 2)
    start = face + np.array([0.0, 0.030 + TIP_PAST_TCP, 0.0])
    flat = np.array([XH - PUSH_AT, YH + LEV_T / 2 - TIP_PAST_TCP - PUSH_PAST, Z_B])
    start, flat = W(start), W(flat)
    away = np.array([0.20, -0.16, TOP + 0.27])
    out += [_w(1.4, start, 0.0, "fingertips out beside the lever's outer face", r_push),
            _w(0.4, start, 0.0, "settle", r_push),
            _w(2.0, flat, 0.0, "push the lever flat", r_push),
            _w(0.8, flat, 0.0, "hold it flat", r_push),
            _w(1.0, flat + [-0.06, 0.0, 0.0], 0.0, "back away", r_push),
            _w(1.2, away, OPEN, "move away", r_push),
            _w(4.0, away, OPEN, "stay clear", r_push)]
    return out


def _idle(arm: str, total: float) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return [Waypoint(total, tuple(float(v) for v in pos), OPEN, "stay put", rot)]


def demo():
    work = _pump()
    return {ARM: work, "left": _idle("left", sum(w.duration for w in work))}
