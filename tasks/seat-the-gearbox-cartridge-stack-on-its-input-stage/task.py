"""A brushless motor stands upright in a stand on the bench, shaft up. A planetary gearbox's black input stage is
screwed to its face, and the gearbox input coupler sits on the motor shaft inside the input stage's bore; the shaft
sticks up out of it. Beside the motor, on a four-post cradle, stands the rest of the gearbox, already stacked: two
gear cartridges with the output stage on top. Where the cradle stands and how it is turned changes between trials.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots.py, so its reset hooks run first)
import lots  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6
TAU = 2.0 * math.pi

BLACK = (0.07, 0.07, 0.08)  # black anodised input and output stages
CART = (0.20, 0.20, 0.22)  # gear cartridge housings
STEEL = (0.72, 0.73, 0.75)
MOTOR = (0.58, 0.60, 0.63)
STAND = (0.92, 0.92, 0.90)  # printed stand and cradle
DARK = (0.02, 0.02, 0.02)
LINE = (0.85, 0.85, 0.85)

# -- the motor and the gearbox input (REV NEO V1.1; MAXPlanetary input stage REV-21-2106, coupler REV-21-2108) ------
XM, YM = 0.290, 0.0  # motor axis on the bench
STAND_FLOOR, STAND_H, STAND_W, POCKET = 0.005, 0.030, 0.080, 0.061
MOTOR_R, MOTOR_L = 0.030, 0.05825
Z_FACE = TOP + STAND_FLOOR + MOTOR_L  # the motor's face, where the input stage is screwed on
IN_H = 0.0191  # input stage height
Z_IN = Z_FACE + IN_H  # the input stage's top face: the stack's seat
HALF = 0.0254  # every stage is 50.8 mm square
BORE_HALF = 0.015  # input stage bore (29 mm round, drawn 30 mm square)
FLANGE = 0.0064  # the input stage's bottom flange carries its two ears
NOTCH_D, NOTCH_HW, NOTCH_IN, NOTCH_CH = 0.0014, 0.0103, 0.0231, 0.00025
SHAFT_R, SHAFT_L, SHAFT_TIP = 0.004, 0.035, 0.0005
Z_SHAFT_TOP = Z_FACE + SHAFT_L
Z_CT = Z_IN - 0.0012  # top of the coupler's internal spline (estimate: gives 3.0 mm of engagement when seated)
SPL_DEPTH = 0.0049

# -- the spline: 39 teeth (odd count, as the maker states), 20.5/19.1 mm male, 20.7/19.3 mm female (from the drawing) --
N_T = 39
PITCH = TAU / N_T
M_IN, M_OUT, M_W, M_CH = 0.00910, 0.01025, 0.00070, 0.00010  # male (cartridge input) teeth, tip chamfer
F_IN, F_OUT, F_W = 0.00965, 0.01045, 0.000652  # female (coupler) teeth
RING_IN, RING_OUT = 0.01035, 0.0127
BOSS = 0.0042  # the cartridge's input spline stands this far below its housing
BORE_R, BORE_CH, CORE_R = 0.00415, 0.0006, 0.00920  # the sun's 8.3 mm bore, its mouth chamfer, the core (12-sided)
SUN_TOP = 0.0127
ENGAGE = BOSS - (Z_IN - Z_CT)  # 3.0 mm: how far the splines overlap when the stack is seated

# -- the stack: 4:1 cartridge (REV-21-2102), 3:1 (REV-21-2101), 1/2 in hex output stage (REV-21-2104) --------------------
STACK_H = 0.0127 + 0.0127 + 0.0191
HOLE = 0.007  # the stack's through hole round the shaft (8.3 mm bores, hex socket; drawn 14 mm square)
TAB_HW, TAB_T, TAB_H = 0.0100, 0.0020, 0.0011  # the rear cartridge's two alignment tabs
EAR_X, EAR_Y = (0.0220, 0.03095), (0.0155, HALF)  # ears at the +x+y and -x-y corners
EAR_HOLE = (0.0270, 0.02145)  # the assembly screw holes (5.0 mm) in the ears
STACK_KG, SUN_KG = 0.378, 0.020  # 133 g + 155 g (maker) + 90 g output stage (estimate); sun and planets estimate
SUN_DRAG, MOTOR_DRAG = 0.6, 0.6  # N m s/rad: grease drag of the gear train, the motor's cogging and bearings
HALF_PITCH = PITCH / 2
QUARTER = PITCH / 4  # the start: a quarter tooth off meshing at both square turns (0 and 180 degrees)

# -- the cradle (loose) ----------------------------------------------------------------------------------------------
CR_PLATE, CR_POST, CR_POST_H, CR_POST_AT = 0.005, 0.012, 0.020, 0.019
Z_CRADLE_TOP = CR_PLATE + CR_POST_H
CRADLE_KG = 0.25
CRADLE_A = (0.200, -0.140)
CRADLE_B = (0.205, 0.140)
YAW_A, YAW_B = math.radians(30.0), math.radians(-30.0)


def _floats(v):
    """Plain Python floats: a position of NumPy floats crashes the simulator at reset."""
    return tuple(float(x) for x in v)


def _box(lo, hi, color, **kw):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return parts.box(tuple(hi - lo), at=tuple((lo + hi) / 2), color=color, **kw)


def _radial(r0, r1, width, z0, z1, angle, color, **kw):
    """A box from radius r0 to r1 (width across), z0 to z1, its centre line at ``angle`` round z."""
    rc = (r0 + r1) / 2
    return parts.box((r1 - r0, width, z1 - z0), at=(rc * math.cos(angle), rc * math.sin(angle), (z0 + z1) / 2),
                     color=color, yaw=angle, **kw)


def _ring(r0, r1, z0, z1, n, color, phase=0.0, collide=True):
    w = 2.0 * r1 * math.tan(math.pi / n)
    return [_radial(r0, r1, w, z0, z1, phase + TAU * k / n, color, collide=collide) for k in range(n)]


# -- fixed: stand, motor, input stage; the coupler and shaft turn on the motor's bearings ----------------------------
def _motor_shapes():
    s, h, p = STAND_W / 2, STAND_H, POCKET / 2
    zf = Z_FACE - TOP
    out = [
        _box((-s, -s, 0.0), (s, s, STAND_FLOOR), STAND),
        _box((p, -s, STAND_FLOOR), (s, s, h), STAND), _box((-s, -s, STAND_FLOOR), (-p, s, h), STAND),
        _box((-p, p, STAND_FLOOR), (p, s, h), STAND), _box((-p, -s, STAND_FLOOR), (p, -p, h), STAND),
        parts.cylinder(MOTOR_R, MOTOR_L, at=(0.0, 0.0, STAND_FLOOR + MOTOR_L / 2), color=MOTOR),
        parts.cylinder(MOTOR_R + 0.0003, 0.004, at=(0.0, 0.0, STAND_FLOOR + MOTOR_L - 0.006), color=BLACK,
                       collide=False),
    ]
    zb, zn, zt = zf, Z_IN - NOTCH_D - TOP, Z_IN - TOP
    b, H = BORE_HALF, HALF
    # body below the notch layer: a square frame round the bore
    out += [_box((b, -H, zb), (H, H, zn), BLACK), _box((-H, -H, zb), (-b, H, zn), BLACK),
            _box((-b, b, zb), (b, H, zn), BLACK), _box((-b, -H, zb), (b, -b, zn), BLACK)]
    # the top 1.4 mm: notches for the cartridge's tabs in the middle of the +y and -y sides, mouths chamfered
    out += [_box((b, -H, zn), (H, H, zt), BLACK), _box((-H, -H, zn), (-b, H, zt), BLACK)]
    c = NOTCH_CH
    for sy in (1, -1):
        ys = sorted((sy * b, sy * NOTCH_IN))
        out.append(_box((-b, ys[0], zn), (b, ys[1], zt), BLACK))
        yo = sorted((sy * NOTCH_IN, sy * H))
        for sx in (1, -1):
            xs = sorted((sx * NOTCH_HW, sx * b))
            xc = sorted((sx * (NOTCH_HW + c), sx * b))
            out.append(_box((xc[0], yo[0], zn), (xc[1], yo[1], zt), BLACK))
            out.append(_box((xs[0], yo[0], zn), (xs[1], yo[1], zt - c), BLACK))
            out.append(parts.box((c * math.sqrt(2), yo[1] - yo[0], c * math.sqrt(2)),
                                 at=(sx * (NOTCH_HW + c), (yo[0] + yo[1]) / 2, zt - c), color=BLACK,
                                 pitch=math.pi / 4))
    # the flange's two ears (the gearbox screws come up through them) and the holes' marks
    for sg in (1, -1):
        xs, ys = sorted((sg * EAR_X[0], sg * EAR_X[1])), sorted((sg * EAR_Y[0], sg * EAR_Y[1]))
        out.append(_box((xs[0], ys[0], zb), (xs[1], ys[1], zb + FLANGE), BLACK))
        out.append(parts.cylinder(0.0025, 0.0004, at=(sg * EAR_HOLE[0], sg * EAR_HOLE[1], zb + FLANGE + 0.0002),
                                  color=LINE, collide=False))
    return out


def _coupler_shapes():
    """The coupler (internal spline, 25.4 mm across) and the motor shaft, which turn together on the key."""
    zc, zt = Z_CT - TOP, Z_CT - SPL_DEPTH - TOP
    out = [_radial(F_IN, F_OUT, F_W, zt, zc, HALF_PITCH + PITCH * k, STEEL) for k in range(N_T)]
    out += _ring(RING_IN, RING_OUT, zt, zc, 24, STEEL)
    out += [parts.cylinder(RING_OUT, 0.0035, at=(0.0, 0.0, zt - 0.00175), color=STEEL),
            parts.cylinder(0.00635, 0.006, at=(0.0, 0.0, zt - 0.0035 - 0.003), color=STEEL)]
    zf, zs = Z_FACE - TOP, Z_SHAFT_TOP - TOP
    out += [parts.cylinder(SHAFT_R, zs - SHAFT_TIP - zf, at=(0.0, 0.0, (zf + zs - SHAFT_TIP) / 2), color=STEEL),
            parts.cylinder(SHAFT_R - SHAFT_TIP, SHAFT_TIP, at=(0.0, 0.0, zs - SHAFT_TIP / 2), color=STEEL)]
    return out


COUPLER = parts.Hinge("coupler", shapes=tuple(_coupler_shapes()), pivot=(0.0, 0.0, float(Z_CT - TOP)),
                      axis=(0.0, 0.0, 1.0), lower=-3.0, upper=3.0, mass=0.15, damping=MOTOR_DRAG,
                      friction=(0.3, 0.25), inertia=(4e-5, 4e-5, 4e-5))
MOTOR_PART = parts.Part("motor", pos=_floats((XM, YM, TOP)), shapes=_motor_shapes(), joints=[COUPLER],
                        friction=(0.4, 0.3), contact_offset=0.0005)


# -- loose: the stack (one body: its stages are pressed together) and the rear cartridge's sun, free on its gears -----
def _stack_shapes():
    H, h, t = HALF, HOLE, STACK_H
    out = [_box((h, -H, 0.0), (H, H, 0.0127), CART), _box((-H, -H, 0.0), (-h, H, 0.0127), CART),
           _box((-h, h, 0.0), (h, H, 0.0127), CART), _box((-h, -H, 0.0), (h, -h, 0.0127), CART),
           _box((h, -H, 0.0127), (H, H, 0.0254), CART), _box((-H, -H, 0.0127), (-h, H, 0.0254), CART),
           _box((-h, h, 0.0127), (h, H, 0.0254), CART), _box((-h, -H, 0.0127), (h, -h, 0.0254), CART),
           _box((h, -H, 0.0254), (H, H, t), BLACK), _box((-H, -H, 0.0254), (-h, H, t), BLACK),
           _box((-h, h, 0.0254), (h, H, t), BLACK), _box((-h, -H, 0.0254), (h, -h, t), BLACK)]
    for sy in (1, -1):  # the rear cartridge's tabs, below its face in the middle of the +y and -y sides
        ys = sorted((sy * (H - TAB_T), sy * H))
        out.append(_box((-TAB_HW, ys[0], -TAB_H), (TAB_HW, ys[1], 0.0), CART))
    for sg in (1, -1):  # ears and their screw holes
        xs, ys = sorted((sg * EAR_X[0], sg * EAR_X[1])), sorted((sg * EAR_Y[0], sg * EAR_Y[1]))
        out.append(_box((xs[0], ys[0], 0.0), (xs[1], ys[1], 0.0127), CART))
        out.append(_box((xs[0], ys[0], 0.0127), (xs[1], ys[1], 0.0254), CART))
        out.append(_box((xs[0], ys[0], 0.0254), (xs[1], ys[1], t), BLACK))
        out.append(parts.cylinder(0.0025, 0.0004, at=(sg * EAR_HOLE[0], sg * EAR_HOLE[1], t + 0.0002), color=LINE,
                                  collide=False))
    # the output: bearing ring and hex socket on top; seam lines between the stages
    out += [parts.cylinder(0.0145, 0.0006, at=(0.0, 0.0, t + 0.0003), color=STEEL, collide=False),
            parts.cylinder(0.0073, 0.0008, at=(0.0, 0.0, t + 0.0004), color=DARK, collide=False)]
    for z in (0.0127, 0.0254):
        out.append(_box((-H - 0.0002, -H - 0.0002, z - 0.0003), (H + 0.0002, H + 0.0002, z + 0.0003), DARK,
                        collide=False))
    return out


def _sun_shapes():
    """The rear cartridge's sun: its 8.3 mm bore (mouth chamfered) and the 39-tooth input spline below the housing."""
    out = []
    n, c = 12, BORE_CH
    w = 2.0 * CORE_R * math.tan(math.pi / n)
    for k in range(n):
        a = TAU * k / n
        out.append(_radial(BORE_R + c, CORE_R, w, -BOSS, SUN_TOP, a, STEEL))
        out.append(_radial(BORE_R, CORE_R, w, -BOSS + c, SUN_TOP, a, STEEL))
        r = BORE_R + c
        out.append(parts.box((c * math.sqrt(2), w, c * math.sqrt(2)),
                             at=(r * math.cos(a), r * math.sin(a), -BOSS + c), color=STEEL, yaw=a, pitch=math.pi / 4))
    ch = M_CH
    for k in range(N_T):
        a = PITCH * k
        ca, sa = math.cos(a), math.sin(a)
        rc = (M_IN + M_OUT) / 2
        out.append(_radial(M_IN, M_OUT, M_W, -BOSS + ch, 0.0, a, STEEL))
        out.append(_radial(M_IN, M_OUT, M_W - 2 * ch, -BOSS, -BOSS + ch, a, STEEL))
        for st in (1, -1):
            tt = st * (M_W / 2 - ch)
            out.append(parts.box((M_OUT - M_IN, ch * math.sqrt(2), ch * math.sqrt(2)),
                                 at=(rc * ca - tt * sa, rc * sa + tt * ca, -BOSS + ch), color=STEEL, yaw=a,
                                 roll=math.pi / 4))
    return out


SUN = parts.Hinge("sun", shapes=tuple(_sun_shapes()), pivot=(0.0, 0.0, 0.0), axis=(0.0, 0.0, 1.0), lower=-3.0,
                  upper=3.0, mass=SUN_KG, damping=SUN_DRAG, friction=(0.3, 0.25), inertia=(2e-5, 2e-5, 2e-5))
STACK_SHAPES = _stack_shapes()


def _yaw_quat(y):
    return (0.0, 0.0, math.sin(y / 2), math.cos(y / 2))


def _cradle_shapes():
    s, p, q = 0.035, CR_POST / 2, CR_POST_AT
    out = [_box((-s, -s, 0.0), (s, s, CR_PLATE), STAND)]
    for sx in (1, -1):
        for sy in (1, -1):
            out.append(_box((sx * q - p, sy * q - p, CR_PLATE), (sx * q + p, sy * q + p, Z_CRADLE_TOP), STAND))
    return out


def stack_start(xy, yaw):
    return _floats((xy[0], xy[1], TOP + Z_CRADLE_TOP + 0.0003))


STACK = parts.Part("stack", pos=stack_start(CRADLE_A, YAW_A), yaw=YAW_A, mass=STACK_KG, shapes=STACK_SHAPES,
                   joints=[SUN], friction=(0.4, 0.3), contact_offset=0.0005)
CRADLE = parts.Part("cradle", pos=_floats((CRADLE_A[0], CRADLE_A[1], TOP)), yaw=YAW_A, mass=CRADLE_KG,
                    shapes=_cradle_shapes(), friction=(0.6, 0.5), contact_offset=0.001)

OBJECTS = [MOTOR_PART, CRADLE, STACK]
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", cradle=CRADLE_A, turn=YAW_A, sun=QUARTER),
    lots.Lot("B", place={"cradle": _floats((CRADLE_B[0], CRADLE_B[1], TOP)), "stack": stack_start(CRADLE_B, YAW_B)},
             yaw={"cradle": YAW_B, "stack": YAW_B}, cradle=CRADLE_B, turn=YAW_B, sun=-QUARTER),
)
SUCCESS_HOLD_S = 1.5  # the 1 s hold is enforced in score() itself; this only keeps a run going past success


# -- geometry for scoring --------------------------------------------------------------------------------------------
CLEAR = 0.005  # let go: every sampled point of both hands more than 5 mm from the stack
STILL_M, HOLD_S = 0.001, 1.0


def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def stack_frame(st):
    """(bottom centre, rotation) of the stack's body."""
    return np.asarray(st.pos, float), _rot(st.quat_xyzw)


def tilt_deg(st) -> float:
    r = _rot(st.quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))


CORNERS = np.array([[sx * 0.0230, sy * 0.0230, 0.0] for sx in (-1, 1) for sy in (-1, 1)])


def corner_gaps(st) -> np.ndarray:
    """Height of the stack's bottom face above the input stage's top face at its four corners (m)."""
    p, r = stack_frame(st)
    return (p + CORNERS @ r.T)[:, 2] - Z_IN


def centre_gap(st) -> float:
    return float(stack_frame(st)[0][2] - Z_IN)


def axis_offset(st) -> float:
    """How far the stack's axis is from the motor axis, at the stack's bottom face (m)."""
    p, _ = stack_frame(st)
    return float(math.hypot(p[0] - XM, p[1] - YM))


def on_shaft(st) -> bool:
    """Threaded on the motor shaft: axis within 1.5 mm of it, upright within 5 degrees, the shaft at least 1 mm into
    the stack's bore (its bottom face less than 19.0 mm above the input stage)."""
    return bool(axis_offset(st) <= 0.0015 + EPS and tilt_deg(st) <= 5.0 + EPS and centre_gap(st) <= 0.0190 + EPS)


def spline_engaged(st) -> bool:
    """The input spline at least 1 mm into the coupler: bottom face within 2.0 mm of the input stage, axis within 1 mm,
    upright within 3 degrees."""
    return bool(axis_offset(st) <= 0.0010 + EPS and tilt_deg(st) <= 3.0 + EPS and centre_gap(st) <= 0.0020 + EPS)


IN_HOLES = np.array([[XM + EAR_HOLE[0], YM + EAR_HOLE[1]], [XM - EAR_HOLE[0], YM - EAR_HOLE[1]]])


def hole_offset(st) -> float:
    """The larger of the two distances (m, across the bench) from each of the stack's ear holes to the input stage's
    ear hole under it: the screws go up through both."""
    p, r = stack_frame(st)
    holes = [(p + r @ np.array([sg * EAR_HOLE[0], sg * EAR_HOLE[1], 0.0]))[:2] for sg in (1, -1)]
    return float(max(np.linalg.norm(IN_HOLES - h, axis=1).min() for h in holes))


def seated(st) -> bool:
    """Fully seated: all four corners within 0.3 mm of the input stage, and both ear holes within 0.5 mm of the input
    stage's (the screws can go in)."""
    g = corner_gaps(st)
    return bool(spline_engaged(st) and float(g.max()) <= 0.0003 + EPS and float(g.min()) >= -0.0005 - EPS
                and hole_offset(st) <= 0.0005 + EPS)


FALL_V, FALL_TILT = 0.5, 45.0


def dropped(st, gap) -> bool:
    """Let go of (hands more than 5 mm away) and falling faster than 0.5 m/s (about a 13 mm fall), or let go of and
    tipped past 45 degrees."""
    if gap <= CLEAR + EPS:
        return False
    return bool(float(st.lin_vel[2]) < -FALL_V - EPS or tilt_deg(st) > FALL_TILT + EPS)


# -- the trial record ----------------------------------------------------------------------------------------------
_rec = parts.__dict__.setdefault("gearbox_stack_record", {})


def reset_record() -> None:
    _rec.clear()
    _rec.update(steps=0, known=True, reason="", gap=None, since=None, held=False, snap=None, t=0.0, tier=0.0,
                dropped=False)


if not _rec:
    reset_record()


def tier(objects) -> float:
    """The end-state tier before the let-go and stillness rules: 0.25 on the shaft, 0.5 spline engaged, 1.0 seated."""
    st = objects["stack"]
    if seated(st):
        return 1.0
    if spline_engaged(st):
        return 0.5
    if on_shaft(st):
        return 0.25
    return 0.0


def _snap(objects) -> np.ndarray:
    st = objects["stack"]
    p, r = stack_frame(st)
    return np.concatenate([p + r @ np.array(v) for v in ((HALF, 0.0, 0.0), (0.0, HALF, 0.0), (0.0, 0.0, STACK_H))])


def observe(objects, gap, t) -> None:
    """One control step. ``gap``: the closest sampled hand point to the stack (m), or None when the hands could not
    be read, which ends all credit for the trial."""
    _rec["steps"] += 1
    _rec["t"] = float(t)
    try:
        st = objects["stack"]
        vals = [*st.pos, *st.quat_xyzw, st.joints["sun"]]
        if not all(math.isfinite(float(v)) for v in vals):
            raise ValueError("stack state unknown")
        if gap is None or not math.isfinite(gap):
            raise ValueError("hand state unknown")
        if dropped(st, gap):
            _rec["dropped"] = True
        tr = tier(objects)
        snap = _snap(objects)
    except (KeyError, ValueError, TypeError, AttributeError, IndexError) as e:
        _rec.update(known=False, reason=str(e), since=None, held=False)
        return
    _rec["gap"] = float(gap)
    if tr <= 0.0 or gap <= CLEAR + EPS:
        _rec.update(since=None, held=False, snap=None, tier=tr)
    elif _rec["since"] is None or tr != _rec["tier"] or float(np.abs(snap - _rec["snap"]).max()) > STILL_M + EPS:
        _rec.update(since=float(t), held=False, snap=snap, tier=tr)
    else:
        _rec["held"] = float(t) - _rec["since"] >= HOLD_S - EPS


# -- scoring ---------------------------------------------------------------------------------------------------------
def score(objects):
    """0 for the whole trial if the stack was let go of and fell (faster than 0.5 m/s) or tipped past 45 degrees at any
    step, or if the hands or the stack could not be read at any step. Otherwise, with both hands more than 5 mm from
    the stack and the stack moving no more than 1 mm for 1 s: 1.0 seated (no corner gap over 0.3 mm, ear holes within
    0.5 mm of the input stage's); 0.5 its input spline engaged in the coupler (bottom within 2.0 mm of the input stage,
    axis within 1.0 mm, upright within 3 degrees); 0.25 threaded on the motor shaft; 0 otherwise."""
    try:
        if not _rec["known"] or _rec["steps"] == 0 or not _rec["held"] or _rec["dropped"]:
            return 0.0
        if _rec["gap"] is None or _rec["gap"] <= CLEAR + EPS:
            return 0.0
        tr = tier(objects)
        if tr != _rec["tier"]:
            return 0.0
        if float(np.abs(_snap(objects) - _rec["snap"]).max()) > STILL_M + EPS:
            return 0.0
        return tr
    except (KeyError, ValueError, TypeError, AttributeError, IndexError):
        return 0.0


def success(objects):
    return score(objects) >= 1.0 - 1e-9


def distance(objects):
    try:
        st = objects["stack"]
        return abs(centre_gap(st)) + hole_offset(st)
    except (KeyError, ValueError, TypeError, AttributeError):
        return 1.0


# -- hands: hull faces sampled every 3 mm ----------------------------------------------------------------------------
_HAND_LINKS = ("gripper", "tip_left", "tip_right")
_FACES: dict = {}


def _hull_face_points(hull, step=0.003) -> np.ndarray:
    pts = [hull.vertices]
    for n, d in zip(hull.planes[:, :3], hull.planes[:, 3]):
        on = hull.vertices[np.abs(hull.vertices @ n - d) < 1e-6]
        if len(on) < 3:
            continue
        c = on.mean(axis=0)
        e1 = on[0] - c
        if np.linalg.norm(e1) < 1e-9:
            continue
        e1 = e1 / np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        ring = on[np.argsort(np.arctan2((on - c) @ e2, (on - c) @ e1))]
        for i in range(len(ring)):  # fan triangles from the face's centre, each filled with a grid
            a, b = ring[i], ring[(i + 1) % len(ring)]
            m = max(1, int(math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / step)))
            ii, jj = np.meshgrid(np.arange(m + 1), np.arange(m + 1), indexing="ij")
            keep = (ii + jj) <= m
            s_, t_ = ii[keep][:, None] / m, jj[keep][:, None] / m
            pts.append(c + (a - c) * s_ + (b - c) * t_)
    pts = np.concatenate(pts)
    _, idx = np.unique(np.round(pts / (step / 2)).astype(np.int64), axis=0, return_index=True)
    return np.concatenate([hull.vertices, pts[np.sort(idx)]])


def link_points(name) -> np.ndarray:
    if name not in _FACES:
        from yam_arena import analysis

        hs = analysis.link_hulls().get(name, [])
        if not hs:
            raise ValueError(f"no collision hull for {name}")
        _FACES[name] = np.concatenate([_hull_face_points(h) for h in hs])
    return _FACES[name]


def stack_boxes(st):
    """(centre, rotation, size) of the stack body's colliding shapes (the sun is inside it or below it)."""
    p, r = stack_frame(st)
    return [(p + r @ np.asarray(s.at), r @ s.turn, 2 * s.own_half) for s in STACK_SHAPES if s.collide]


def gap_to(pts, boxes) -> float:
    from yam_arena.analysis import box_signed_distance

    if len(pts) == 0:
        return 1.0
    return float(min(box_signed_distance(pts, c, r, s).min() for c, r, s in boxes))


def hand_gap(objects, pts) -> float:
    """Closest hand point to the stack (m; negative inside)."""
    st = objects["stack"]
    c = np.asarray(st.pos, float)
    near = np.linalg.norm(pts - c, axis=1) < 0.15
    return gap_to(pts[near], stack_boxes(st))


def _hand_points(sim):
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out, found = [], 0
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in _HAND_LINKS:
                out.append(link_points(n) @ _rot(poses[i, 3:]).T + (poses[i, :3] - origin))
                found += 1
    if found != 2 * len(_HAND_LINKS):
        return None
    pts = np.concatenate(out)
    return pts if np.isfinite(pts).all() else None


# -- simulator hooks -------------------------------------------------------------------------------------------------
def _write_pose(base, name, xyz, yaw) -> None:
    import torch

    obj = base.scene[name]
    q = _yaw_quat(yaw)
    pose = torch.tensor([[*xyz, *q]], dtype=torch.float32, device=base.device)
    pose[:, :3] += base.scene.env_origins[0][:3].to(pose.dtype)
    write = getattr(obj, "write_root_pose_to_sim_index", None) or obj.write_root_pose_to_sim
    write(root_pose=pose)
    vel = getattr(obj, "write_root_velocity_to_sim_index", None) or obj.write_root_velocity_to_sim
    vel(root_velocity=torch.zeros((1, 6), device=base.device))


def _write_start(base) -> None:
    lot = lots.current()
    cx, cy = lot.cradle
    _write_pose(base, "cradle", (cx, cy, TOP), lot.turn)
    _write_pose(base, "stack", stack_start(lot.cradle, lot.turn), lot.turn)
    parts._write_joint(base, "stack", "sun", lot.sun)
    parts._write_joint(base, "motor", "coupler", 0.0)


def _apply_exact(emb) -> bool:
    """A new trial's start: the cradle and stack written to their lot's place and turn, the sun and coupler to their
    angles, settled briefly, written again, settled and rendered, so the first frame shows the true start."""
    base = emb._base
    for _ in range(2):
        _write_start(base)
        for _ in range(max(1, round(0.25 / base.step_dt))):
            emb._env.step(emb._ready)
    for _ in range(lots.RENDERS):
        base.sim.render()
    return True


lots._apply = _apply_exact


@parts.on_reset
def _new_trial(sim) -> None:
    reset_record()
    _rec["last_t"] = None


@parts.on_step
def _step(sim) -> None:
    if _rec.get("last_t") == sim.t:  # a second copy of this module in the process: observe once per step
        return
    _rec["last_t"] = sim.t
    try:
        objects = sim.states()
    except Exception:  # noqa: BLE001  unknown state: no credit
        objects = {}
    g = None
    try:
        pts = _hand_points(sim)
        g = hand_gap(objects, pts) if pts is not None else None
    except Exception:  # noqa: BLE001
        g = None
    observe(objects, g, float(sim.t))
    log = str(pathlib.Path(os.environ["TRACE_OUT"]).with_name("stack.jsonl")) if os.environ.get("TRACE_OUT") else None
    if log and _rec["steps"] % 4 == 0 and objects:
        try:
            st, mo = objects["stack"], objects["motor"]
            r = _rot(st.quat_xyzw)
            row = dict(t=round(float(sim.t), 3), lot=lots.current().name, score=score(objects), tier=tier(objects),
                       pos=np.round(np.asarray(st.pos) - [XM, YM, Z_IN], 5).tolist(),
                       yaw=round(math.degrees(math.atan2(r[1, 0], r[0, 0])), 2), tilt=round(tilt_deg(st), 2),
                       gaps=np.round(corner_gaps(st) * 1000, 2).tolist(),
                       sun=round(math.degrees(float(st.joints["sun"])), 2),
                       coupler=round(math.degrees(float(mo.joints["coupler"])), 2),
                       holes=round(hole_offset(st) * 1000, 2), vz=round(float(st.lin_vel[2]), 3),
                       phase=round(math.degrees((float(st.joints["sun"]) + math.atan2(r[1, 0], r[0, 0])
                                                 - float(mo.joints["coupler"]) + HALF_PITCH) % PITCH
                                                - HALF_PITCH), 2),
                       hand=_rec["gap"], held=_rec["held"], known=_rec["known"], dropped=_rec["dropped"],
                       reason=_rec["reason"])
            base = sim.env.unwrapped
            origin = parts._np(base.scene.env_origins[0])
            for arm in ("left_robot", "right_robot"):
                a = base.scene[arm]
                poses = parts._np(a.data.body_link_pose_w)[0]
                q = parts._np(a.data.joint_pos)[0]
                row[arm] = dict(g=np.round(poses[list(a.body_names).index("gripper"), :3] - origin, 4).tolist(),
                                f=[round(float(q[list(a.joint_names).index(n)]), 4) for n in ("joint7", "joint8")
                                   if n in a.joint_names])
            with open(log, "a") as f:
                f.write(json.dumps(row) + "\n")
        except Exception as e:  # noqa: BLE001
            with open(log, "a") as f:
                f.write(json.dumps({"err": str(e)}) + "\n")


# -- close cameras for the videos (off unless TASK_VIEWS_DIR is set; never in the policy's observation) --------------
_SEAT = (XM, YM, Z_IN + 0.004)
parts.views([
    ("side_cam", (XM + 0.010, YM + 0.200, Z_IN + 0.012), _SEAT, 700.0, 1280, 720),  # level with the seat, from the left
    ("back_cam", (XM + 0.200, YM - 0.050, Z_IN + 0.040), _SEAT, 700.0, 640, 360),  # from beyond the motor, above it
])


# -- demo: open loop from the visible layout; the spline's phase is found by a light twist, as by hand ----------------
PARK = {"left": (0.17, 0.22, TOP + 0.16), "right": (0.17, -0.22, TOP + 0.16)}
GRIP_OPEN = -(HALF + 0.010)
GRIP = -(HALF - 0.003)
GRIP_BELOW_TOP = 0.004  # TCP this far below the stack's top: the pads hold the output stage
COMP = {"left": (0.0018, -0.0028), "right": (0.0024, 0.0028)}  # where each arm holds the stack off its target (m), from
# a trace: commanded over the shaft, the stack arrived 1.7 to 2.4 mm short in x and 2.8 mm out in y. (The loaded arm
# also sags about 11 mm there, so the heights over the shaft are reached about 11 mm low: the stack is lowered
# straight onto the shaft rather than held over it.)
SWEEP = math.radians(7.0)
FINE = math.radians(2.0)


def W(d, p, f, lab, yaw=0.0):
    return Waypoint(float(d), tuple(float(v) for v in p), float(f), lab, float(yaw))


def tcp_z(gap):
    """TCP height that puts the stack's bottom face ``gap`` above the input stage's top face."""
    return Z_IN + gap + STACK_H - GRIP_BELOW_TOP


# A check, not the demo: GEARBOX_DEMO=press plays the plain way instead: carry the stack over the motor square, press
# it straight down without twisting, let go. (Lot B's start phase is lot A's at half a turn, so the two lots cover a
# straight press at both square turns.)
VARIANT = os.environ.get("GEARBOX_DEMO", "")


def demo():
    lot = lots.current()
    arm = "right" if lot.cradle[1] < 0 else "left"
    other = "left" if arm == "right" else "right"
    cx, cy = lot.cradle
    y0 = lot.turn  # the stack's visible turn in its cradle; the fingers close across its sides
    z_pick = TOP + Z_CRADLE_TOP + STACK_H - GRIP_BELOW_TOP
    dx, dy = COMP[arm]
    mx, my = XM + dx, YM + dy
    z_carry = TOP + 0.165
    sq = 0.0
    plan = [
        W(1.8, (cx, cy, z_pick + 0.07), GRIP_OPEN, "over the stack", y0),
        W(1.4, (cx, cy, z_pick), GRIP_OPEN, "fingers either side of the output stage", y0),
        W(0.8, (cx, cy, z_pick), GRIP, "grip", y0),
        W(0.3, (cx, cy, z_pick), GRIP, "settle", y0),
        W(1.4, (cx, cy, z_carry), GRIP, "lift it off the cradle", y0),
        W(2.2, (mx, my, z_carry), GRIP, "over the motor, ears turned over the input stage's ears", sq),
        W(0.5, (mx, my, z_carry), GRIP, "settle", sq),
        W(1.8, (mx, my, tcp_z(0.0240)), GRIP, "down onto the shaft's end", sq),
        W(0.8, (mx, my, tcp_z(0.0240)), GRIP, "settle", sq),
        W(2.0, (mx, my, tcp_z(0.0145)), GRIP, "down the shaft", sq),
        W(1.5, (mx, my, tcp_z(0.0045)), GRIP, "slide down the shaft", sq),
        W(1.0, (mx, my, tcp_z(0.0015)), GRIP, "rest the spline on the coupler, lightly", sq),
    ]
    if VARIANT:
        plan += [W(1.2, (mx, my, tcp_z(-0.0010)), GRIP, "press straight down", sq),
                 W(1.0, (mx, my, tcp_z(-0.0010)), GRIP, "hold", sq)]
    else:
        plan += [
            W(1.2, (mx, my, tcp_z(0.0015)), GRIP, "twist one way to find the spline", SWEEP),
            W(2.4, (mx, my, tcp_z(0.0015)), GRIP, "and the other", -SWEEP),
            W(1.2, (mx, my, tcp_z(0.0015)), GRIP, "turn the housing back square", 0.0),
            W(1.2, (mx, my, tcp_z(-0.0010)), GRIP, "press it home", 0.0),
            W(0.6, (mx, my, tcp_z(-0.0010)), GRIP, "rock it onto its tabs", FINE),
            W(1.2, (mx, my, tcp_z(-0.0010)), GRIP, "rock it onto its tabs", -FINE),
            W(0.6, (mx, my, tcp_z(-0.0010)), GRIP, "square", 0.0),
            W(0.5, (mx, my, tcp_z(-0.0010)), GRIP, "hold", 0.0),
        ]
    plan += [
        W(0.8, (mx, my, tcp_z(0.0)), GRIP_OPEN, "let go", sq),
        W(1.2, (mx, my, tcp_z(0.0) + 0.035), GRIP_OPEN, "lift clear", sq),
        W(1.5, PARK[arm], FINGER_OPEN, "park", 0.0),
        W(6.0, PARK[arm], FINGER_OPEN, "wait", 0.0),
    ]
    t = sum(w.duration for w in plan)
    idle = [W(2.0, PARK[other], FINGER_OPEN, "park", 0.0), W(t, PARK[other], FINGER_OPEN, "wait", 0.0)]
    plan = [W(2.0, PARK[arm], FINGER_OPEN, "park", 0.0)] + plan
    return {arm: plan, other: idle}
