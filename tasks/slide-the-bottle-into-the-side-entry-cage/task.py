"""A full-size bicycle front-triangle fixture stands on a plywood base on the bench, across the table in front of the
robot with its left side toward the robot and its head tube to the robot's left. Its round tubes (down tube, top
tube, seat tube and head tube) follow an extra-small trail hardtail's frame at true size, and the frame is tipped
10 degrees nose-down. A black side-entry bottle cage is bolted to the down tube's upper two bottle bosses, with a
sprung front arm. An empty 550 ml cycling bottle with its cap closed lies on its side on the bench to the robot's
right. Where the bottle lies changes between trials.
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
import tilt  # noqa: E402  (the carrying grip is turned to the cage's axis)
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

ALU = (0.62, 0.64, 0.67)  # bare aluminium tube
CAGE = (0.06, 0.06, 0.07)  # black glass-fibre polyamide cage
PLY = (0.74, 0.60, 0.42)
BOTTLE_C = (0.15, 0.45, 0.85)  # translucent-blue bottle body (opaque in the simulator)
CAP_C = (0.92, 0.92, 0.94)
STEEL = (0.30, 0.31, 0.33)


def _f(v):
    """Plain Python floats: a position of NumPy floats crashes the simulator at reset."""
    return tuple(float(x) for x in v)


def _R(axis, ang) -> np.ndarray:
    return parts._rot(axis, ang)


def euler_zxy(m) -> tuple[float, float, float]:
    """(yaw, roll, pitch) with m = Rz(yaw) Rx(roll) Ry(pitch), as parts.py turns a shape."""
    m = np.asarray(m, float)
    roll = math.asin(max(-1.0, min(1.0, m[2, 1])))
    pitch = math.atan2(-m[2, 0], m[2, 2])
    yaw = math.atan2(-m[0, 1], m[1, 1])
    return yaw, roll, pitch


# -- the frame, in its own plane: u forward (toward the head tube), v up, w across (away from the robot) -------------
# Tube centrelines (m) traced from the maker's 1:1 drawing of the XS front triangle; origin = the middle of the
# bottle's base resting on the cage floor. The down tube's curved lower end is two straight pieces.
BB = (-0.060, -0.200)
SEAT = ((-0.077, -0.187), (-0.180, 0.177))
DOWN = (BB, (0.056, -0.046), (0.255, 0.312))
TOPT = ((-0.152, 0.088), (0.023, 0.206), (0.206, 0.323))
HEAD = ((0.280, 0.256), (0.244, 0.344))
D_DOWN, D_TOP, D_SEAT, D_HEAD = 0.0508, 0.0349, 0.0349, 0.0508  # 2 in, 1-3/8 in, 1-3/8 in, 2 in round tube
D_BB, W_BB = 0.046, 0.073
GUSSET_T = 0.012
GUSSETS = (((-0.140, 0.095), (0.090, 0.070), 30.0), ((0.245, 0.315), (0.100, 0.080), 20.0),
           ((-0.062, -0.190), (0.120, 0.070), 0.0))  # (centre u, v), (length, width), turn (deg)
AXIS_DEG = 61.0  # the cage's (and bottle's) axis in the frame's plane, from the frame's horizontal
BOSS_AT = ((0.0563, 0.0088), (0.0874, 0.0648))  # the two upper bottle bosses, 64 mm apart

# -- where the frame stands --------------------------------------------------------------------------------------------
PITCH = math.radians(-10.0)  # nose-down tip of the frame in its base, as a bike often sits clamped in a repair stand
FLOOR_X, FLOOR_Y = 0.430, -0.515  # world x, y of the middle of the cage floor (the bottle's seated base)
U_W = np.array([0.0, 1.0, 0.0])  # u before pitching: to the robot's left
V_W = np.array([0.0, 0.0, 1.0])
W_W = np.array([1.0, 0.0, 0.0])  # across the frame, away from the robot: the direction a bottle goes in
R_FRAME = np.c_[U_W, V_W, W_W] @ _R((0, 0, 1), PITCH)  # frame-local (u, v, w) -> world


def loc(u, v, w=0.0) -> np.ndarray:
    """A frame-local point as an offset in the world (from the cage floor's middle)."""
    return R_FRAME @ np.array([u, v, w], float)


A_L = np.array([math.cos(math.radians(AXIS_DEG)), math.sin(math.radians(AXIS_DEG)), 0.0])
B_L = np.array([-A_L[1], A_L[0], 0.0])  # in the plane, square to the axis, away from the down tube
AX = R_FRAME @ A_L  # cage axis (world), toward the bottle's cap
BX = R_FRAME @ B_L  # away from the down tube
WX = R_FRAME @ np.array([0.0, 0.0, 1.0])  # across the frame, away from the robot


def cage_pt(s, r, phi) -> np.ndarray:
    """A point s along the cage axis, r from it, at angle phi (rad) round it: 0 = away from the down tube, +90 deg =
    the far side (away from the robot), -90 deg = the near side, 180 deg = the down tube."""
    return s * A_L + r * (math.cos(phi) * B_L + math.sin(phi) * np.array([0.0, 0.0, 1.0]))


# -- bottle (Elite Fly 550 ml: 74 mm across, 184.5 mm tall, 51 g) ------------------------------------------------------
R_B, BODY_H, CAP_R, CAP_H = 0.037, 0.160, 0.0295, 0.0245
BOTTLE_KG = 0.051

# -- cage (Elite Custom Race X: 74 mm, 147.3 mm tall, 28.5 g); shapes in the frame's plane coordinates ----------------
CAGE_H = 0.1473
T_CAGE = 0.004  # wall thickness
R_IN = R_B + 0.0005  # the cage's inside radius
BAND_S = (0.048, 0.088)  # the sprung front arm's band, along the axis
HINGE_PHI = math.radians(75.0)  # where the front arm is hinged (far front)
TIP_PHI = math.radians(-40.0)  # where its tip ends (near front): it wraps 40 degrees past the front toward the robot
SPRING = 0.68  # N m/rad: 1.3 N at the tip closed (preload) rising to about 3.9 N at 15 degrees open (estimates)
PRELOAD = 0.13  # rad: the spring's rest point is this far past the closed stop
ARM_OPEN_MAX = math.radians(40.0)  # mechanical limit; past 30 degrees counts as over-bending the arm


def _shape_box(size, centre_l, rot_l, color, collide=True):
    """A box given in frame-local coordinates (centre, rotation), as a FRAME part shape."""
    yaw, roll, pitch = euler_zxy(R_FRAME @ rot_l)
    return parts.box(size, at=_f(R_FRAME @ np.asarray(centre_l, float)), color=color, yaw=yaw, roll=roll, pitch=pitch,
                     collide=collide)


def _tube(p0, p1, d, color=ALU):
    """A round tube from p0 to p1 (frame plane), as a regular octagonal prism of four boxes."""
    p0, p1 = np.array([*p0, 0.0]), np.array([*p1, 0.0])
    L = float(np.linalg.norm(p1 - p0))
    ang = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    base = _R((0, 0, 1), ang)  # box x along the tube
    out = []
    for k in range(4):
        r = base @ _R((1, 0, 0), k * math.pi / 4)
        out.append(_shape_box((L, d, d * math.tan(math.pi / 8)), (p0 + p1) / 2, r, color))
    return out


def _arc_box(s0, s1, phi, dphi, r_in, t, color=CAGE):
    """One tangent piece of a cage wall: covers s0..s1 along the axis and dphi of arc centred at phi."""
    c = cage_pt((s0 + s1) / 2, r_in + t / 2, phi)
    radial = math.cos(phi) * B_L + math.sin(phi) * np.array([0.0, 0.0, 1.0])
    tang = np.cross(A_L, radial)
    rot = np.c_[A_L, radial, tang]
    chord = 2 * (r_in + t) * math.tan(dphi / 2)
    return _shape_box((s1 - s0, t, chord), c, rot, color)


def _frame_shapes():
    out = []
    for a, b in zip(DOWN[:-1], DOWN[1:]):
        out += _tube(a, b, D_DOWN)
    for a, b in zip(TOPT[:-1], TOPT[1:]):
        out += _tube(a, b, D_TOP)
    out += _tube(*SEAT, D_SEAT)
    out += _tube(*HEAD, D_HEAD)
    # bottom-bracket shell, across the frame
    for k in range(4):
        out.append(_shape_box((D_BB * math.tan(math.pi / 8), D_BB, W_BB), (*BB, 0.0), _R((0, 0, 1), k * math.pi / 4),
                              ALU))
    # 12 mm plywood gusset plates on both faces of the three joints, bolted through the tubes
    for (u, v), (lu, lv), ang in GUSSETS:
        for side in (-1, 1):
            out.append(_shape_box((lu, lv, GUSSET_T), (u, v, side * (D_DOWN / 2 + GUSSET_T / 2)),
                                  _R((0, 0, 1), math.radians(ang)), PLY))
    # cage: back (spine) between bottle and down tube, bolted to both bosses
    out.append(_arc_box(-0.006, 0.110, math.pi, math.radians(24), R_IN + 0.001, T_CAGE + 0.001))
    out.append(_arc_box(0.110, CAGE_H - 0.006, math.pi, math.radians(16), R_IN + 0.001, T_CAGE + 0.001))
    # floor (a plate square to the axis under the base) with low lips at the front and the far side
    fc = cage_pt(-0.003, 0.0, 0.0) + B_L * (-0.012)
    out.append(_shape_box((0.006, 0.062, 0.050), fc, np.c_[A_L, B_L, [0, 0, 1]], CAGE))
    for phi in (0.0, math.radians(30), math.radians(60), math.radians(90), math.radians(120)):
        out.append(_arc_box(0.0, 0.010, phi, math.radians(30), R_IN, T_CAGE))
    # far-side wall (fixed), from the spine round to the hinge
    for deg in (100, 125, 150):
        out.append(_arc_box(0.015, 0.105, math.radians(deg), math.radians(25), R_IN, T_CAGE))
    # fixture: plywood base board, a block under the bottom bracket and a post under the head tube
    return out


ARM_PHIS = [math.radians(d) for d in (62.5, 37.5, 12.5, -12.5, -32.5)]
ARM_DPHI = [math.radians(25), math.radians(25), math.radians(25), math.radians(25), math.radians(15)]


def _arm_shapes():
    return [_arc_box(BAND_S[0], BAND_S[1], p, dp, R_IN, T_CAGE) for p, dp in zip(ARM_PHIS, ARM_DPHI)]


HINGE_PT_L = cage_pt((BAND_S[0] + BAND_S[1]) / 2, R_IN + T_CAGE / 2, HINGE_PHI)


# -- fixture base: plywood board on the bench, a block under the bottom bracket and a post up to the head tube ------
def _support_shapes(floor_z):
    out = []
    bb_w = loc(*BB)
    head_lo = loc(*HEAD[0])
    z_bb = floor_z + bb_w[2] - D_BB / 2  # underside of the bottom-bracket shell (world)
    board_t = 0.018
    out_board = (0.24, 0.62, board_t)
    cx, cy = FLOOR_X + 0.06, FLOOR_Y + 0.12
    out.append(parts.box(out_board, at=_f((cx - FLOOR_X, cy - FLOOR_Y, TOP + board_t / 2 - floor_z)), color=PLY))
    blk_h = z_bb - (TOP + board_t)
    out.append(parts.box((0.090, 0.080, blk_h), at=_f((0.0, bb_w[1], TOP + board_t + blk_h / 2 - floor_z)), color=PLY))
    post_top = floor_z + head_lo[2] - D_HEAD / 2 - 0.004
    post_h = post_top - (TOP + board_t)
    out.append(parts.box((0.045, 0.045, post_h), at=_f((0.0, head_lo[1], TOP + board_t + post_h / 2 - floor_z)),
                         color=PLY))
    return out, z_bb


def _lowest(shapes) -> float:
    zs = []
    for s in shapes:
        corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * s.own_half
        zs.append((corners @ s.turn.T + np.asarray(s.at))[:, 2].min())
    return float(min(zs))


_FS = _frame_shapes()
BB_CLEAR = 0.040  # the bottom bracket's underside stands this high above the bench (on the block)
FLOOR_Z = TOP + BB_CLEAR - (loc(*BB)[2] - D_BB / 2)  # world z of the cage floor's middle
FLOOR = np.array([FLOOR_X, FLOOR_Y, FLOOR_Z])
_SUP, _ = _support_shapes(FLOOR_Z)

FRONT_ARM = parts.Hinge("front_arm", shapes=tuple(_arm_shapes()), pivot=_f(R_FRAME @ HINGE_PT_L), axis=_f(AX),
                        lower=0.0, upper=ARM_OPEN_MAX, mass=0.010, spring=SPRING, damping=0.004, max_effort=2.0)
FRAME = parts.Part("frame", pos=_f(FLOOR), shapes=_FS + _SUP, joints=[FRONT_ARM], friction=(0.5, 0.4),
                   contact_offset=0.002)


FLAT = 2 * R_B * math.cos(math.pi / 16)  # the body as a 16-sided prism, corners on the 74 mm circle (flats 0.7 mm in)


def _bottle_shapes():
    """The body (a 16-sided prism, eight boxes, so the lying bottle does not roll away on its own) and the cap."""
    body = [parts.box((FLAT, BODY_H, FLAT * math.tan(math.pi / 16)), at=(0.0, BODY_H / 2, 0.0), color=BOTTLE_C,
                      pitch=k * math.pi / 8) for k in range(8)]
    return body + [parts.cylinder(CAP_R, CAP_H, at=(0.0, BODY_H + CAP_H / 2, 0.0), color=CAP_C, axis="y")]


BOTTLE_A = (0.250, -0.300, TOP + FLAT / 2)  # lot A: base centre; the bottle lies along +y (cap to the robot's left)
BOTTLE_B = (0.300, -0.170, TOP + FLAT / 2)
YAW_B = math.radians(25.0)
BOTTLE = parts.Part("bottle", pos=_f(BOTTLE_A), mass=BOTTLE_KG, shapes=_bottle_shapes(), friction=(0.6, 0.5),
                    contact_offset=0.002)

OBJECTS = [FRAME, BOTTLE]
LOTS = lots.declare(OBJECTS, lots.Lot("A", place={"bottle": BOTTLE_A}),
                    lots.Lot("B", place={"bottle": BOTTLE_B}, yaw={"bottle": YAW_B}))
SUCCESS_HOLD_S = 1.5  # the 1 s hold is enforced in score() itself; this only keeps a run going past success


# -- scoring geometry ----------------------------------------------------------------------------------------------
def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def bottle_frame(b):
    """(base centre, axis toward the cap) of the bottle, world."""
    r = _rot(b.quat_xyzw)
    return np.asarray(b.pos, float), r[:, 1]


def seat_measures(b):
    """(floor gap m: the base rim's lowest point above the cage floor along the cage axis, negative = below;
    off-axis m: the base centre's distance from the cage axis; tilt deg: the bottle's axis to the cage's)."""
    p, ax = bottle_frame(b)
    d = p - FLOOR
    along = float(d @ AX)
    off = float(np.linalg.norm(d - along * AX))
    cosang = max(-1.0, min(1.0, float(ax @ AX)))
    tilt_deg = math.degrees(math.acos(cosang))
    # the rim of the base: lowest point along AX
    perp = AX - (AX @ ax) * ax
    n = float(np.linalg.norm(perp))
    rim_low = along - (R_B * n if n > 1e-9 else 0.0)
    return rim_low, off, tilt_deg


def mid_in_cage(b) -> float:
    """Distance of the bottle's axis from the cage axis at the front arm's band height (m)."""
    p, ax = bottle_frame(b)
    s_band = (BAND_S[0] + BAND_S[1]) / 2
    # the bottle's axis point at the band's height along the cage axis
    denom = float(ax @ AX)
    if denom < 0.5:
        return 1.0
    t = (s_band - float((p - FLOOR) @ AX)) / denom
    q = p + t * ax - FLOOR
    return float(np.linalg.norm(q - (q @ AX) * AX))


FLOOR_TOL = 0.0010  # base rim within 1 mm of the floor
OFF_TOL = 0.0030
TILT_TOL = 3.0
ARM_SHUT = math.radians(5.0)  # the front arm back within 5 degrees of its closed stop
OVERBENT = math.radians(30.0)
IN_CAGE = 0.006


def tier(objects) -> float:
    """End-state tier before the release and stillness rules: 1.0 seated on the floor behind the shut arm; 0.5 behind
    the shut arm (inside the cage) but not seated on the floor."""
    b, fr = objects["bottle"], objects["frame"]
    arm = float(fr.joints["front_arm"])
    if arm > ARM_SHUT + EPS:
        return 0.0
    gap, off, tl = seat_measures(b)
    if abs(gap) <= FLOOR_TOL + EPS and off <= OFF_TOL + EPS and tl <= TILT_TOL + EPS:
        return 1.0
    if mid_in_cage(b) <= IN_CAGE + EPS and tl <= 10.0 + EPS and -0.002 <= gap <= 0.030:
        return 0.5
    return 0.0


# -- trial record ----------------------------------------------------------------------------------------------------
CLEAR = 0.005
STILL_M, STILL_RAD, HOLD_S = 0.001, math.radians(1.0), 1.0
_rec = parts.__dict__.setdefault("bottle_cage_record", {})


def reset_record() -> None:
    _rec.clear()
    _rec.update(steps=0, known=True, reason="", gap=None, since=None, held=False, snap=None, t=0.0, tier=0.0,
                worst_arm=0.0, bent=False)


if not _rec:
    reset_record()


def _snap(objects) -> np.ndarray:
    b = objects["bottle"]
    p, ax = bottle_frame(b)
    r = _rot(b.quat_xyzw)
    pts = [p, p + 0.18 * ax, p + R_B * r[:, 0]]
    return np.concatenate(pts + [np.array([float(objects["frame"].joints["front_arm"]) * 0.07])])


def observe(objects, gap, t) -> None:
    """One control step. ``gap``: closest sampled hand point to the bottle or the cage's front arm (m), or None when
    the hands could not be read, which ends all credit for the trial."""
    _rec["steps"] += 1
    _rec["t"] = float(t)
    try:
        b, fr = objects["bottle"], objects["frame"]
        arm = float(fr.joints["front_arm"])
        vals = [*b.pos, *b.quat_xyzw, arm]
        if not all(math.isfinite(float(v)) for v in vals):
            raise ValueError("bottle or cage state unknown")
        _rec["worst_arm"] = max(_rec["worst_arm"], arm)
        if arm > OVERBENT + EPS:
            _rec["bent"] = True
        if gap is None or not math.isfinite(gap):
            raise ValueError("hand state unknown")
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


def score(objects):
    """0 for the whole trial if the hands, the bottle or the cage could not be read at any step, or if the cage's front
    arm was ever bent open more than 30 degrees. Otherwise, with both hands more than 5 mm from the bottle and the
    front arm and nothing moving more than 1 mm for 1 s: 1.0 bottle seated (its base rim within 1 mm of the cage
    floor, base centre within 3 mm of the cage axis, axis within 3 degrees) with the front arm shut (within 5 degrees
    of closed); 0.5 bottle inside the cage behind the shut arm but not seated; 0 otherwise."""
    try:
        if not _rec["known"] or _rec["bent"] or _rec["steps"] == 0 or not _rec["held"]:
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
        gap, off, tl = seat_measures(objects["bottle"])
        return abs(gap) + off + tl / 1000.0
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
        for i in range(len(ring)):
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


def _bottle_dist(pts, b) -> np.ndarray:
    """Exact distance of points to the bottle (body and cap cylinders), negative inside."""
    p, ax = bottle_frame(b)
    d = pts - p
    s = d @ ax
    rad = np.linalg.norm(d - np.outer(s, ax), axis=1)
    out = np.full(len(pts), np.inf)
    for r, s0, s1 in ((R_B, 0.0, BODY_H), (CAP_R, BODY_H, BODY_H + CAP_H)):
        dr, ds = rad - r, np.maximum(s0 - s, s - s1)
        dist = np.where((dr > 0) & (ds > 0), np.hypot(dr, ds), np.maximum(dr, ds))
        out = np.minimum(out, dist)
    return out


def arm_boxes(fr):
    return [(c, r, s) for _, c, r, s in FRAME.world_boxes(_link_poses(fr, ("front_arm",)))]


def _link_poses(st, names):
    out = {}
    for n in names:
        p, q = st.links[n]
        out[n] = (np.asarray(p, float), _rot(q))
    return out


def hand_gap(objects, pts) -> float:
    """Closest hand point to the bottle or the cage's front arm (m; negative inside)."""
    from yam_arena.analysis import box_signed_distance

    b, fr = objects["bottle"], objects["frame"]
    near = np.linalg.norm(pts - np.asarray(b.pos, float), axis=1) < 0.40
    pts = pts[near]
    if len(pts) == 0:
        return 1.0
    g = float(_bottle_dist(pts, b).min())
    for c, r, s in arm_boxes(fr):
        g = min(g, float(box_signed_distance(pts, c, r, s).min()))
    return g


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
@parts.on_reset
def _new_trial(sim) -> None:
    reset_record()
    _rec["last_t"] = None


@parts.on_step
def _step(sim) -> None:
    if _rec.get("last_t") == sim.t:
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
    log = str(pathlib.Path(os.environ["TRACE_OUT"]).with_name("cage.jsonl")) if os.environ.get("TRACE_OUT") else None
    if log and _rec["steps"] % 5 == 0 and objects:
        try:
            b, fr = objects["bottle"], objects["frame"]
            gap, off, tl = seat_measures(b)
            row = dict(t=round(float(sim.t), 3), lot=lots.current().name, score=score(objects), tier=tier(objects),
                       bottle=np.round(np.asarray(b.pos), 4).tolist(), floor_gap=round(gap, 4), off=round(off, 4),
                       tilt=round(tl, 2), mid=round(mid_in_cage(b), 4),
                       arm_deg=round(math.degrees(float(fr.joints["front_arm"])), 2),
                       worst_arm_deg=round(math.degrees(_rec["worst_arm"]), 2), gap=_rec["gap"], held=_rec["held"],
                       known=_rec["known"], reason=_rec["reason"])
            base = sim.env.unwrapped
            origin = parts._np(base.scene.env_origins[0])
            a = base.scene["right_robot"]
            poses = parts._np(a.data.body_link_pose_w)[0]
            q = parts._np(a.data.joint_pos)[0]
            row["right"] = dict(g=np.round(poses[list(a.body_names).index("gripper"), :3] - origin, 4).tolist(),
                                f=[round(float(q[list(a.joint_names).index(n)]), 4) for n in ("joint7", "joint8")
                                   if n in a.joint_names])
            with open(log, "a") as f:
                f.write(json.dumps(row) + "\n")
        except Exception as e:  # noqa: BLE001
            with open(log, "a") as f:
                f.write(json.dumps({"err": str(e)}) + "\n")


# -- demo: open loop from the published layout --------------------------------------------------------------------
PSI = math.radians(60.0)  # the hand comes at the bottle from 60 degrees round from the front toward the robot's side
S_GRIP = 0.126  # grip this far up the bottle from its base (the pads cover about 92 to 160 mm)
F_GRIP, F_WIDE, F_LET = -0.0335, -0.0469, -0.0385
LIFT = 0.012  # carried this far above the floor while it goes in sideways
OUT = 0.150  # start of the sideways push, out from the frame


def insert_pose(dw=0.0, ds=0.0, back=0.0):
    """TCP position and rotation holding the bottle with its base dw across from the seat (negative: toward the
    robot) and ds up the cage axis; back: TCP drawn back along the fingers."""
    d_out = math.cos(PSI) * BX - math.sin(PSI) * WX  # from the bottle's axis toward the palm
    g = -d_out
    x = np.cross(AX, g)
    rot = np.c_[x, AX, g]
    p = FLOOR + (S_GRIP + ds) * AX + dw * WX + back * d_out
    return p, rot


def table_grip(pos, yaw):
    """TCP position and rotation to pick the lying bottle (base centre ``pos``, axis along yaw-turned +y)."""
    ax = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    g = np.array([0.0, 0.0, -1.0])
    rot = np.c_[np.cross(ax, g), ax, g]
    return np.asarray(pos, float) + S_GRIP * ax, rot


def _rest_pose(arm):
    from yam_arena import ik, station
    from yam_arena.demo import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return tuple(float(v) for v in pos), tilt._TiltedYaw(0.0, rot)


def W(d, p, f, lab, rot):
    return Waypoint(float(d), _f(p), float(f), lab, tilt._TiltedYaw(0.0, rot))


def demo():
    lot = lots.current()
    pos = lot.place.get("bottle", BOTTLE_A)
    yaw = lot.yaw.get("bottle", 0.0)
    gp, gr = table_grip(pos, yaw)
    up = np.array([0.0, 0.0, 1.0])
    p_out, r_ins = insert_pose(-OUT, LIFT)
    p_near, _ = insert_pose(-0.050, LIFT)
    p_in, _ = insert_pose(0.0, LIFT)
    p_seat, _ = insert_pose(0.0, -0.002)
    p_back, _ = insert_pose(0.0, 0.0, back=0.065)
    p_away, _ = insert_pose(-0.10, 0.0, back=0.065)
    rest, ryaw = _rest_pose("right")
    right = [
        W(1.5, gp + 0.10 * up, F_WIDE, "over the bottle", gr),
        W(1.2, gp, F_WIDE, "fingers either side of it", gr),
        W(0.8, gp, F_GRIP, "grip", gr),
        W(0.3, gp, F_GRIP, "settle", gr),
        W(1.2, gp + 0.12 * up, F_GRIP, "lift", gr),
        W(3.0, p_out, F_GRIP, "turn it to the cage's slope, beside the frame", r_ins),
        W(1.0, p_out, F_GRIP, "settle", r_ins),
        W(1.5, p_near, F_GRIP, "up to the frame", r_ins),
        W(3.0, p_in, F_GRIP, "push it in sideways past the front arm", r_ins),
        W(0.5, p_in, F_GRIP, "settle", r_ins),
        W(1.2, p_seat, F_GRIP, "lower it onto the floor", r_ins),
        W(0.4, p_seat, F_GRIP, "settle", r_ins),
        W(0.8, p_seat, F_LET, "let go", r_ins),
        W(1.2, p_back, F_LET, "draw back off the bottle", r_ins),
        W(1.5, p_away, F_LET, "out of the frame", r_ins),
        Waypoint(2.0, rest, F_LET, "back to the rest pose", ryaw),
        Waypoint(4.0, rest, F_LET, "wait", ryaw),
    ]
    left = [Waypoint(sum(w.duration for w in right), (0.20, 0.25, TOP + 0.22), FINGER_OPEN, "wait")]
    plans = {"left": left, "right": right}
    for arm, plan in plans.items():
        rest, ryaw = _rest_pose(arm)
        plan[:0] = [Waypoint(2.0, rest, FINGER_OPEN, "to the rest pose", ryaw),
                    Waypoint(0.5, rest, FINGER_OPEN, "settle at the rest pose", ryaw)]
    return plans
