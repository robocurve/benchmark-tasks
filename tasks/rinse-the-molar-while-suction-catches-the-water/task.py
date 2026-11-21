"""On the station table, an adult dental training head lies low with its mouth wide open toward the robot, as a
reclined patient's would: a printed skin-coloured shell (lips, cheeks, palate, floor of the mouth, tongue and throat)
around a full set of true-size typodont teeth. The patient's lower right first molar (on the robot's left) has an
orange smear of debris on its biting surface. Behind the last lower molar on each side the floor of the mouth dips
into a small pocket, and behind the pockets is the throat. To the robot's left, on a three-post rest, lies a saliva
ejector: a white tube bent down at its end, on a blue valve handle. To the robot's right, on a two-post rest, lies an
air/water syringe: a steel handle with a thin tip bent down at its end. The syringe's water runs by itself, at 1 mL
a second, from 20 s to 30 s into the trial. Where the head lies and how it is turned changes between trials.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots, so the reset hooks run before lots move objects)
import lots  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6
DT = 0.05  # control step (s)

SKIN, GUM, TOOTH, TONGUE = (0.86, 0.68, 0.58), (0.86, 0.47, 0.50), (0.96, 0.94, 0.87), (0.80, 0.42, 0.46)
THROAT, DEBRIS, WATER = (0.45, 0.16, 0.20), (0.95, 0.52, 0.08), (0.30, 0.62, 0.98)
STEEL, WHITE, BLUE, RESTC = (0.74, 0.76, 0.78), (0.93, 0.95, 0.97), (0.20, 0.40, 0.80), (0.35, 0.36, 0.38)


def _qz(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def _q(m):
    return tuple(float(v) for v in parts._matrix_to_quat(np.asarray(m, float)))


def _R(q) -> np.ndarray:
    return parts._quat_to_matrix(np.asarray(q, float))


def _Rz(a) -> np.ndarray:
    return parts._rot((0, 0, 1), a)


def _box_mesh(lo, hi):
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    pts = ((x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1))
    tris = ((0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6),
            (3, 0, 4), (3, 4, 7))
    return pts, tris


def _bx(lo, hi, color, collide=True):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return parts.box(tuple(hi - lo), at=tuple((lo + hi) / 2), color=color, collide=collide)


def _octagon(p0, p1, r, color):
    """An eight-sided rod from p0 to p1 (part frame), corner radius r, a flat face straight down when the rod lies
    level: a round handle or tube that lies still where it is put."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    ax = (p1 - p0) / np.linalg.norm(p1 - p0)
    up = np.array([0.0, 0.0, 1.0]) if abs(ax[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    s = np.cross(ax, up)
    s /= np.linalg.norm(s)
    u = np.cross(s, ax)
    ring = [r * (math.cos(math.radians(22.5 + 45 * k)) * s + math.sin(math.radians(22.5 + 45 * k)) * u)
            for k in range(8)]
    pts = [p0 + v for v in ring] + [p1 + v for v in ring]
    faces = [tuple(range(7, -1, -1)), tuple(range(8, 16))] + [(k, (k + 1) % 8, 8 + (k + 1) % 8, 8 + k) for k in range(8)]
    return parts.hull(pts, faces, color=color)


# --- the head (head frame H: origin at the middle between the lower first molars' biting surfaces; x from the lips
# toward the throat, y to the robot's left (the patient's right), z up; the lower biting plane is z = 0) -----------
H_OCC = 0.075  # m: the lower biting plane above the table
LIP_OUT, LIP_IN = -0.044, -0.034  # lips' outer and inner faces (x)
OPEN_HW, OPEN_LO, OPEN_HI = 0.028, -0.008, 0.052  # the lip opening: half width, lower and upper lip edges (z)
CHEEK, SHELL_HW = 0.036, 0.044  # cheeks' inner and outer faces (|y|)
BACK = 0.058  # back of the throat (x)
FLOOR_Z = -0.016  # floor of the mouth
ROOF_Z = 0.060  # palate
BASE_Z = -H_OCC
UPPER_Z = 0.050  # the upper teeth's biting edges (50 mm between the incisal edges: a wide-open adult mouth)
# Pocket behind each last lower molar: a dip in the floor whose back lip is lower than its sides, so what it cannot
# hold runs over into the throat.
PK_X = (0.017, 0.033)
PK_Y = (0.009, 0.031)
PK_FLOOR, PK_LIP = -0.024, -0.020
PK_AREA = (PK_X[1] - PK_X[0]) * (PK_Y[1] - PK_Y[0])
PK_CAP = PK_AREA * (PK_LIP - PK_FLOOR) * 1e6  # mL held before it runs into the throat (1.41 mL)
THROAT_X = 0.035  # past this, behind the pockets' back lips, is the throat
THROAT_FLOOR = -0.045
TONGUE_HW, TONGUE_X, TONGUE_TOP = 0.0085, (-0.022, 0.033), -0.004

# Lower teeth, the patient's right side (+y; the left side is the mirror image): name, centre x, y, length along the
# arch, width across it, crown height, angle of the arch there (rad from x). Crown sizes are typical adult sizes.
TEETH_R = [
    ("central", -0.0315, 0.0027, 0.0052, 0.0060, 0.009, math.radians(90)),
    ("lateral", -0.0295, 0.0080, 0.0058, 0.0062, 0.009, math.radians(70)),
    ("canine", -0.0245, 0.0135, 0.0068, 0.0075, 0.010, math.radians(50)),
    ("premolar1", -0.0170, 0.0180, 0.0070, 0.0075, 0.008, math.radians(25)),
    ("premolar2", -0.0095, 0.0210, 0.0072, 0.0082, 0.008, math.radians(10)),
    ("molar1", 0.0000, 0.0230, 0.0110, 0.0105, 0.0075, 0.0),
    ("molar2", 0.0110, 0.0245, 0.0105, 0.0100, 0.0070, 0.0),
]
TARGET = "molar1_R"


def _teeth():
    out = []
    for name, x, y, ln, wd, h, ang in TEETH_R:
        for side, sy in (("R", 1.0), ("L", -1.0)):
            a = ang if sy > 0 else -ang
            out.append((f"{name}_{side}", np.array([x, sy * y]), np.array([ln, wd]), h, a))
    return out


TEETH = _teeth()
TEETH_BY = {t[0]: t for t in TEETH}
T_XY = TEETH_BY[TARGET][1]
T_TOP = np.array([T_XY[0], T_XY[1], 0.0])  # the target's biting surface, middle


def _head_shapes():
    out = []
    for name, c, size, h, a in TEETH:  # lower crowns (biting surface at z = 0), their gum below, upper crowns above
        q = _qz(a)
        out.append(parts.box((size[0], size[1], h), at=(c[0], c[1], -h / 2), color=TOOTH, quat=q))
        out.append(parts.box((size[0] + 0.002, size[1] + 0.003, -h - FLOOR_Z), at=(c[0], c[1], (FLOOR_Z - h) / 2),
                             color=GUM, quat=q))
        hu = min(h, 0.0075)  # upper crowns, shown to the gum line under the palate
        out.append(parts.box((size[0], size[1], hu), at=(c[0], c[1], UPPER_Z + hu / 2), color=TOOTH, quat=q))
        out.append(parts.box((size[0] + 0.002, size[1] + 0.003, ROOF_Z - UPPER_Z - hu),
                             at=(c[0], c[1], (UPPER_Z + hu + ROOF_Z) / 2), color=GUM, quat=q))
    W = SHELL_HW
    # lips and the front of the face, with the opening
    out.append(_bx((LIP_OUT, -W, BASE_Z), (LIP_IN, W, OPEN_LO), SKIN))
    out.append(_bx((LIP_OUT, -W, OPEN_HI), (LIP_IN, W, ROOF_Z + 0.008), SKIN))
    for s in (1.0, -1.0):
        out.append(_bx((LIP_OUT, min(s * OPEN_HW, s * W), OPEN_LO), (LIP_IN, max(s * OPEN_HW, s * W), OPEN_HI), SKIN))
        out.append(_bx((LIP_IN, min(s * CHEEK, s * W), BASE_Z), (BACK + 0.008, max(s * CHEEK, s * W), ROOF_Z + 0.008), SKIN))
    out.append(_bx((BACK, -CHEEK, BASE_Z), (BACK + 0.008, CHEEK, ROOF_Z + 0.008), SKIN))  # back of the throat
    out.append(_bx((LIP_IN, -CHEEK, ROOF_Z), (BACK, CHEEK, ROOF_Z + 0.008), SKIN))  # palate
    # floor of the mouth, the two pockets, the throat
    out.append(_bx((LIP_IN, -CHEEK, BASE_Z), (PK_X[0], CHEEK, FLOOR_Z), GUM))
    for s in (1.0, -1.0):
        y0, y1 = sorted((s * PK_Y[0], s * PK_Y[1]))
        out.append(_bx((PK_X[0], y0, BASE_Z), (PK_X[1], y1, PK_FLOOR), GUM))  # pocket floor
        yo0, yo1 = sorted((s * PK_Y[1], s * CHEEK))
        out.append(_bx((PK_X[0], yo0, BASE_Z), (THROAT_X, yo1, FLOOR_Z), GUM))  # beside the pocket, cheek side
        out.append(_bx((PK_X[1], y0, BASE_Z), (THROAT_X, y1, PK_LIP), GUM))  # the pocket's low back lip
    out.append(_bx((PK_X[0], -PK_Y[0], BASE_Z), (THROAT_X, PK_Y[0], FLOOR_Z), GUM))  # under the tongue's back
    out.append(_bx((THROAT_X, -CHEEK, BASE_Z), (BACK, CHEEK, THROAT_FLOOR), THROAT))
    out.append(_bx((TONGUE_X[0], -TONGUE_HW, FLOOR_Z), (TONGUE_X[1], TONGUE_HW, TONGUE_TOP), TONGUE))
    return out


def _vis(name, lo, hi, color, link="base"):
    return parts.Visual(name, *_box_mesh(lo, hi), color=color, link=link)


def _head_visuals():
    v = []
    cx, cy = T_XY
    for k, half in enumerate((0.0045, 0.0030, 0.0018)):  # the debris smear, shrinking as it is rinsed off
        v.append(_vis(f"debris{k}", (cx - half, cy - half, 0.0002), (cx + half, cy + half, 0.0012), DEBRIS))
    for s, side in ((1.0, "R"), (-1.0, "L")):
        y0, y1 = sorted((s * PK_Y[0], s * PK_Y[1]))
        for k, ml in enumerate(POOL_LEVELS):
            h = ml * 1e-6 / PK_AREA
            v.append(_vis(f"pool{side}{k}", (PK_X[0] + 0.0005, y0 + 0.0005, PK_FLOOR), (PK_X[1] - 0.0005, y1 - 0.0005,
                                                                                         PK_FLOOR + h + 0.0004), WATER))
    v.append(_vis("throat", (THROAT_X + 0.001, -0.02, THROAT_FLOOR), (BACK - 0.001, 0.02, THROAT_FLOOR + 0.004), WATER))
    v.append(_vis("spill", (LIP_OUT - 0.0015, -0.03, -0.045), (LIP_OUT - 0.0002, 0.03, -0.012), WATER))
    return v


POOL_LEVELS = (0.25, 0.7, 1.4)  # mL drawn in a pocket
HEAD_MASS = 1.8  # kg: a typodont training head with its mounting
HEAD_XY = (0.400, 0.0)
HEAD = parts.Part("head", pos=(*HEAD_XY, TOP + H_OCC), mass=HEAD_MASS, shapes=_head_shapes(), visuals=_head_visuals(),
                  friction=(1.0, 0.9))

# --- saliva ejector (tool frame S: origin at the middle of the tip's opening; the tube runs back along -x at height
# DROP, then bends 60 degrees to the left at the lips onto the valve handle) ---------------------------------------
S_TUBE_R, S_HANDLE_R = 0.0034, 0.0075  # corner radii: 6.3 mm tube, 13.9 mm handle across flats
S_DROP = 0.031  # m: the tip's bent-down leg, from the opening up to the tube's axis
S_L1 = 0.0753  # m: tube from the tip leg back to the bend at the lips
S_BEND = math.radians(70.0)
S_U = np.array([-math.cos(S_BEND), math.sin(S_BEND), 0.0])  # from the bend along the outer tube and handle
S_OUT, S_HANDLE = 0.035, 0.080  # outer tube and valve handle lengths
S_MASS = 0.030
S_BEND_PT = np.array([-S_L1, 0.0, S_DROP])
S_GRIP = S_BEND_PT + S_U * (S_OUT + S_HANDLE / 2)  # the middle of the handle


def _ejector_shapes():
    tip_top = np.array([0.0, 0.0, S_DROP + S_TUBE_R * 0.9])
    out = [_octagon((0.0, 0.0, 0.0), tip_top, S_TUBE_R, WHITE),
           _octagon((S_TUBE_R * 0.9, 0.0, S_DROP), S_BEND_PT, S_TUBE_R, WHITE),
           _octagon(S_BEND_PT, S_BEND_PT + S_U * S_OUT, S_TUBE_R, WHITE),
           _octagon(S_BEND_PT + S_U * S_OUT, S_BEND_PT + S_U * (S_OUT + S_HANDLE), S_HANDLE_R, BLUE)]
    return out


# --- air/water syringe (tool frame Y: origin at the nozzle; x is the jet's heading, the tip bends 45 degrees down
# at its end) ---------------------------------------------------------------------------------------------------
Y_TIP_R, Y_HANDLE_R = 0.0029, 0.0075
Y_BENT = 0.025  # m: the bent end of the tip
Y_ANGLE = math.radians(45.0)
Y_JET = np.array([math.cos(Y_ANGLE), 0.0, -math.sin(Y_ANGLE)])
Y_BEND_PT = -Y_JET * Y_BENT
Y_STRAIGHT, Y_HEAD, Y_HANDLE = 0.055, 0.016, 0.110
Y_AX = Y_BEND_PT[2]  # height of the straight tip and handle's axis above the nozzle
Y_X1 = Y_BEND_PT[0] - Y_STRAIGHT
Y_X2 = Y_X1 - Y_HEAD
Y_X3 = Y_X2 - Y_HANDLE
Y_MASS = 0.090
Y_GRIP = np.array([-0.150, 0.0, Y_AX])  # at the balance point, the hose coupling counterweighting the tip
Y_COUPLING = 0.025  # m: the hose coupling on the handle's back end, 18 mm square


def _syringe_shapes():
    return [_octagon((0.0, 0.0, 0.0), Y_BEND_PT + Y_JET * 0.001, Y_TIP_R, STEEL),
            _octagon(Y_BEND_PT, (Y_X1, 0.0, Y_AX), Y_TIP_R, STEEL),
            _bx((Y_X2, -0.008, Y_AX - 0.008), (Y_X1, 0.008, Y_AX + 0.008), STEEL),
            _octagon((Y_X2, 0.0, Y_AX), (Y_X3, 0.0, Y_AX), Y_HANDLE_R, STEEL),
            _bx((Y_X3 - Y_COUPLING, -0.009, Y_AX - 0.009), (Y_X3, 0.009, Y_AX + 0.009), STEEL)]


JET_VIS = [parts.Visual("jet", *_box_mesh((0.0, -0.0008, -0.0008), (0.030, 0.0008, 0.0008)), color=WATER)]


def _jet_visual():
    pts, tris = JET_VIS[0].points, JET_VIS[0].triangles
    Ry = parts._rot((0, 1, 0), Y_ANGLE)  # turn the bar from +x onto the jet
    return [parts.Visual("jet", tuple(tuple(float(c) for c in Ry @ np.array(p)) for p in pts), tris, color=WATER)]


# --- in-use poses (head frame), from which the rest poses also follow --------------------------------------------
S_TIP_USE = np.array([0.025, 0.0145, -0.0215])  # the ejector's opening, 2.5 mm above the right pocket's floor
_lip_pt = np.array([-0.050, 0.008])
S_YAW_USE = math.atan2(S_TIP_USE[1] - _lip_pt[1], S_TIP_USE[0] - _lip_pt[0])
Y_HEADING = math.radians(30.0)
Y_REACH = 0.025  # m: nozzle to the target's middle, horizontally (and as far above it: a 45 degree jet)
Y_NOZ_USE = T_TOP + np.array([-Y_REACH * math.cos(Y_HEADING), -Y_REACH * math.sin(Y_HEADING), Y_REACH])

# rests (world, fixed): each tool lies at its in-use heading for lot A, its lowest point 6-10 mm above the table
S_REST_GRIP = np.array([0.255, 0.215])
Y_REST_GRIP = np.array([0.205, -0.215])
S_REST_Z = TOP + 0.006  # the ejector's opening
Y_REST_Z = TOP + 0.010  # the syringe's nozzle


def _rest_pose(grip_local, grip_xy, yaw, z):
    p = np.array([grip_xy[0], grip_xy[1], 0.0]) - _Rz(yaw) @ np.array([grip_local[0], grip_local[1], 0.0])
    return np.array([p[0], p[1], z])


S_REST = _rest_pose(S_GRIP, S_REST_GRIP, S_YAW_USE, S_REST_Z)
Y_REST = _rest_pose(Y_GRIP, Y_REST_GRIP, Y_HEADING, Y_REST_Z)
S_REST_YAW, Y_REST_YAW = S_YAW_USE, Y_HEADING


def _post(at_world_xy, top_z, size=0.010):
    x, y = at_world_xy
    return _bx((x - size / 2, y - size / 2, TOP), (x + size / 2, y + size / 2, top_z), RESTC)


def _ejector_rest():
    R = _Rz(S_REST_YAW)
    ap_h, ap_t = S_HANDLE_R * math.cos(math.radians(22.5)), S_TUBE_R * math.cos(math.radians(22.5))
    ax_z = S_REST_Z + S_DROP
    shapes = []
    for d in (S_OUT + 0.012, S_OUT + S_HANDLE - 0.012):  # two posts under the handle, one under the tube
        p = S_REST + R @ (S_BEND_PT + S_U * d)
        shapes.append(_post(p[:2], ax_z - ap_h))
    p = S_REST + R @ np.array([-0.035, 0.0, S_DROP])
    shapes.append(_post(p[:2], ax_z - ap_t, 0.008))
    base = [parts.box(s.size, at=tuple(np.asarray(s.at) - np.array([*S_REST_GRIP, TOP])), color=s.color) for s in shapes]
    return parts.Part("ejector_rest", pos=(*S_REST_GRIP, TOP), shapes=base)


def _syringe_rest():
    R = _Rz(Y_REST_YAW)
    ap_h = Y_HANDLE_R * math.cos(math.radians(22.5))
    shapes = []
    for x in (-0.110, -0.185):
        p = Y_REST + R @ np.array([x, 0.0, 0.0])
        shapes.append(_post(p[:2], Y_REST_Z + Y_AX - ap_h))
    base = [parts.box(s.size, at=tuple(np.asarray(s.at) - np.array([*Y_REST_GRIP, TOP])), color=s.color) for s in shapes]
    return parts.Part("syringe_rest", pos=(*Y_REST_GRIP, TOP), shapes=base)


EJECTOR = parts.Part("ejector", pos=tuple(float(v) for v in S_REST), yaw=float(S_REST_YAW), mass=S_MASS,
                     shapes=_ejector_shapes(), friction=(1.0, 0.9))
SYRINGE = parts.Part("syringe", pos=tuple(float(v) for v in Y_REST), yaw=float(Y_REST_YAW), mass=Y_MASS,
                     shapes=_syringe_shapes(), visuals=_jet_visual(), friction=(1.0, 0.9))
EJECTOR_REST = _ejector_rest()
SYRINGE_REST = _syringe_rest()
OBJECTS = [HEAD, EJECTOR, SYRINGE, EJECTOR_REST, SYRINGE_REST]

# Lots: where the head lies and how it is turned (a patient settles differently each time).
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"head": (*HEAD_XY, TOP + H_OCC)}),
    lots.Lot("B", place={"head": (0.395, -0.025, TOP + H_OCC)}, yaw={"head": math.radians(-8.0)}),
)

# --- water (a tracked volume) ------------------------------------------------------------------------------------
T_ON, T_OFF, RATE = 20.0, 30.0, 1.0  # s, s, mL/s: the syringe's timed water
SUCK = 2.0  # mL/s the ejector draws from a pocket while its opening is in that pocket's water
TAU = 0.4  # s: water striking the teeth reaches the pocket on its side with this time constant
DOSE_CLEAN = 3.0  # mL striking the target to wash the debris off
THROAT_MAX, SPILL_MAX, DRY_MAX = 1.0, 1.0, 0.2  # mL
_L: dict = sys.modules.setdefault("_dental_rinse_record", type(sys)("_dental_rinse_record")).__dict__.setdefault(
    "state", {})


def reset_state() -> None:
    _L.update(t=0.0, steps=0, dose=0.0, transit={"R": 0.0, "L": 0.0}, pool={"R": 0.0, "L": 0.0}, throat=0.0,
              spill=0.0, sucked=0.0, tip_throat=False, known=False, clear=False, hold=0, flowing=False)


def head_frame(head):
    return np.asarray(head.pos, float), _R(head.quat_xyzw)


def to_head(head, p_world):
    o, R = head_frame(head)
    return R.T @ (np.asarray(p_world, float) - o)


def _ray_box(o, d, c, size, a):
    """Distance along the ray (o, d; head frame) to the box (centre c, size, turned a about z), or None."""
    Rb = _Rz(a)
    ol, dl = Rb.T @ (o - c), Rb.T @ d
    half = np.asarray(size) / 2
    t0, t1 = 0.0, 1.0
    for k in range(3):
        if abs(dl[k]) < 1e-12:
            if abs(ol[k]) > half[k]:
                return None
            continue
        a_, b_ = (-half[k] - ol[k]) / dl[k], (half[k] - ol[k]) / dl[k]
        t0, t1 = max(t0, min(a_, b_)), min(t1, max(a_, b_))
        if t0 > t1:
            return None
    return t0


def inside_mouth(p) -> bool:
    return bool(LIP_IN <= p[0] <= BACK and abs(p[1]) <= CHEEK and THROAT_FLOOR <= p[2] <= UPPER_Z)


def jet_landing(head, syringe):
    """Where the syringe's jet lands: ("tooth", name, side), ("mouth", side), ("throat",) or ("spill",). The jet is
    a straight line from the nozzle (a fine stream at about 1 m/s falls under 1 mm over these distances)."""
    Rs = _R(syringe.quat_xyzw)
    o = to_head(head, syringe.pos)
    _, Rh = head_frame(head)
    d = Rh.T @ (Rs @ Y_JET)
    L = 0.25  # m: the ray's length
    if not inside_mouth(o):
        if d[0] <= 1e-9:
            return ("spill",)
        t = (LIP_IN - o[0]) / d[0]  # through the inner face of the lips
        if t > L:
            return ("spill",)
        p = o + d * t
        if not (abs(p[1]) <= OPEN_HW and OPEN_LO <= p[2] <= OPEN_HI):
            return ("spill",)
    best = None
    for name, c, size, h, a in TEETH:
        t = _ray_box(o, d * L, np.array([c[0], c[1], -h / 2]), (size[0], size[1], h), a)
        if t is not None and (best is None or t < best[0]):
            best = (t, name)
    if best is not None:
        return ("tooth", best[1], "R" if TEETH_BY[best[1]][1][1] > 0 else "L")
    if d[2] >= -1e-9:
        return ("mouth", "R" if o[1] >= 0 else "L")  # up into the palate: it runs down on that side
    over_tongue = None
    t = (TONGUE_TOP - o[2]) / d[2]
    if t >= 0:
        p = o + d * t
        if abs(p[1]) <= TONGUE_HW and TONGUE_X[0] <= p[0] <= TONGUE_X[1]:
            over_tongue = p
    p = over_tongue if over_tongue is not None else o + d * max(0.0, (FLOOR_Z - o[2]) / d[2])
    if p[0] >= THROAT_X:
        return ("throat",)
    if not inside_mouth(np.array([max(p[0], LIP_IN), p[1], max(p[2], THROAT_FLOOR)])):
        return ("spill",)
    return ("mouth", "R" if p[1] >= 0 else "L")


def ejector_pocket(head, ejector):
    """The pocket ("R" or "L") whose water the ejector's opening is in, or None."""
    p = to_head(head, ejector.pos)
    for side, s in (("R", 1.0), ("L", -1.0)):
        y0, y1 = sorted((s * PK_Y[0], s * PK_Y[1]))
        if PK_X[0] <= p[0] <= PK_X[1] and y0 <= p[1] <= y1 and PK_FLOOR - 0.001 <= p[2] <= PK_LIP + 0.001:
            return side
    return None


def tip_in_throat(head, obj, local=(0.0, 0.0, 0.0)) -> bool:
    p = to_head(head, np.asarray(obj.pos, float) + _R(obj.quat_xyzw) @ np.asarray(local, float))
    return bool(p[0] >= THROAT_X and abs(p[1]) <= CHEEK and p[2] <= FLOOR_Z)


def mouth_water() -> float:
    return float(sum(_L["transit"].values()) + sum(_L["pool"].values()))


def step_water(objects, dt: float = DT) -> None:
    head, syr, ej = objects["head"], objects["syringe"], objects["ejector"]
    t = _L["t"]
    _L["flowing"] = bool(T_ON - EPS <= t < T_OFF - EPS)
    if _L["flowing"]:
        q = RATE * dt
        hit = jet_landing(head, syr)
        if hit[0] == "spill":
            _L["spill"] += q
        elif hit[0] == "throat":
            _L["throat"] += q
        else:
            side = hit[-1]
            _L["transit"][side] += q
            if hit[0] == "tooth" and hit[1] == TARGET:
                _L["dose"] += q
    for side in ("R", "L"):
        moved = _L["transit"][side] * (1.0 - math.exp(-dt / TAU))
        _L["transit"][side] -= moved
        _L["pool"][side] += moved
        if _L["transit"][side] < 1e-4:
            _L["pool"][side] += _L["transit"][side]
            _L["transit"][side] = 0.0
        over = _L["pool"][side] - PK_CAP
        if over > 0.0:
            _L["pool"][side] = PK_CAP
            _L["throat"] += over
    side = ejector_pocket(head, ej)
    if side is not None:
        take = min(_L["pool"][side], SUCK * dt)
        _L["pool"][side] -= take
        _L["sucked"] += take
    if tip_in_throat(head, ej) or tip_in_throat(head, syr):
        _L["tip_throat"] = True
    _L["t"] = t + dt
    _L["steps"] += 1


# --- end state ---------------------------------------------------------------------------------------------------
REST_XY, REST_Z, REST_YAW, REST_TILT = 0.010, 0.003, math.radians(10.0), 8.0
HOLD_STEPS = 20  # 1 s at 20 Hz


def _yaw(q):
    R = _R(q)
    return math.atan2(R[1, 0], R[0, 0])


def _tilt(q):
    return math.degrees(math.acos(max(-1.0, min(1.0, float(_R(q)[2, 2])))))


def on_rest(obj, rest_pos, rest_yaw) -> bool:
    dyaw = (_yaw(obj.quat_xyzw) - rest_yaw + math.pi) % (2 * math.pi) - math.pi
    return bool(math.hypot(obj.pos[0] - rest_pos[0], obj.pos[1] - rest_pos[1]) <= REST_XY + EPS
                and abs(obj.pos[2] - rest_pos[2]) <= REST_Z + EPS and abs(dyaw) <= REST_YAW + EPS
                and _tilt(obj.quat_xyzw) <= REST_TILT + EPS)


def failed() -> bool:
    return bool(_L["throat"] > THROAT_MAX + EPS or _L["spill"] > SPILL_MAX + EPS or _L["tip_throat"])


def _rinsed() -> bool:
    return bool(_L["dose"] >= DOSE_CLEAN - EPS)


def _dry() -> bool:
    return bool(_L["t"] >= T_OFF - EPS and mouth_water() <= DRY_MAX + EPS)


def tools_home(objects) -> bool:
    return bool(on_rest(objects["ejector"], S_REST, S_REST_YAW) and on_rest(objects["syringe"], Y_REST, Y_REST_YAW))


def _done(objects) -> bool:
    return bool(not failed() and _rinsed() and _dry() and _L["clear"] and tools_home(objects)
                and objects["ejector"].resting() and objects["syringe"].resting())


def record(objects, clear: bool, dt: float = DT) -> None:
    """One control step: the water, whether every finger and gripper is clear of both tools, and how long the
    finished state has held."""
    _L["known"] = True
    step_water(objects, dt)
    _L["clear"] = bool(clear)
    _L["hold"] = _L["hold"] + 1 if _done(objects) else 0


if "t" not in _L:
    reset_state()


def score(objects):
    """0 once more than 1 mL has reached the throat or landed outside the mouth, or a tool tip has gone into the
    throat (all judged over the whole trial), or if the hands' state is unknown. Otherwise 1.0 when the debris is
    rinsed off (3 mL on the target tooth), the water has stopped and at most 0.2 mL is left in the mouth, and both
    tools lie on their rests, let go, still for 1 s; 0.5 for the rinse and the dry mouth without that; 0.25 for the
    rinse alone."""
    if not _L["known"] or failed():
        return 0.0
    if _L["hold"] >= HOLD_STEPS and _done(objects):
        return 1.0
    if _rinsed() and _dry():
        return 0.5
    if _rinsed():
        return 0.25
    return 0.0


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    return float(max(0.0, DOSE_CLEAN - _L["dose"]) / DOSE_CLEAN)


# --- simulator hooks: hands clear of the tools (every hull face sampled about every 3 mm) -------------------------
HAND_LINKS = ("tip_left", "tip_right", "gripper")
CLEAR = 0.005
SAMPLE = 0.003
_HAND_PTS: dict = {}


def hand_points(link: str) -> np.ndarray:
    """Points on a hand link's collision surfaces (link frame): every vertex plus each triangle face sampled about
    every 3 mm, so a flat face resting on a tool is seen even when no vertex is near it. Built once and cached."""
    if link not in _HAND_PTS:
        import xml.etree.ElementTree as ET

        from yam_arena import ASSET_DIR, YAM_URDF, analysis
        from yam_arena.urdf import rpy_matrix

        pts = []
        for lk in ET.parse(YAM_URDF).getroot().iter("link"):
            if lk.get("name") != link:
                continue
            for col in lk.findall("collision"):
                origin = col.find("origin")
                rot, off = np.eye(3), np.zeros(3)
                if origin is not None:
                    rot = rpy_matrix(np.array([float(v) for v in origin.get("rpy", "0 0 0").split()]))
                    off = np.array([float(v) for v in origin.get("xyz", "0 0 0").split()])
                tris = analysis.read_stl(ASSET_DIR / col.find("geometry/mesh").get("filename")) @ rot.T + off
                pts.append(sample_triangles(tris))
        _HAND_PTS[link] = np.concatenate(pts) if pts else np.zeros((0, 3))
    return _HAND_PTS[link]


def sample_triangles(tris) -> np.ndarray:
    """Each triangle's corners plus points across its face about every 3 mm, kept one per 1.5 mm cell."""
    pts = []
    for t in np.asarray(tris, float):
        edge = max(np.linalg.norm(t[1] - t[0]), np.linalg.norm(t[2] - t[1]), np.linalg.norm(t[0] - t[2]))
        n = max(1, int(math.ceil(edge / SAMPLE)))
        for i in range(n + 1):
            for j in range(n + 1 - i):
                pts.append(t[0] + (t[1] - t[0]) * i / n + (t[2] - t[0]) * j / n)
    return np.unique(np.round(np.array(pts) / 0.0015), axis=0) * 0.0015


def hands_clear_of(link_poses, st) -> bool:
    """True when every sampled point of every hand link (4x4 world poses by link name) is more than 5 mm from every
    shape of the ejector and the syringe (each shape's bounding box)."""
    from yam_arena import analysis

    pts = [hand_points(n) @ m[:3, :3].T + m[:3, 3] for n, m in link_poses]
    pts = [p for p in pts if len(p)]
    if not pts:
        return False
    cloud = np.concatenate(pts)
    lo, hi = cloud.min(axis=0), cloud.max(axis=0)
    for name, spec in (("ejector", EJECTOR), ("syringe", SYRINGE)):
        s = st[name]
        for _, c, r, size in spec.world_boxes({"base": (np.asarray(s.pos), _R(s.quat_xyzw))}):
            c = np.asarray(c)
            reach = 0.5 * float(np.linalg.norm(size)) + CLEAR + 0.001
            if np.any(c < lo - reach) or np.any(c > hi + reach):
                continue
            if analysis.box_signed_distance(cloud, c, np.asarray(r), np.asarray(size)).min() <= CLEAR + EPS:
                return False
    return True


def _hands_clear(sim, st) -> bool:
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    poses_out = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in HAND_LINKS:
                m = np.eye(4)
                m[:3, :3] = _R(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                poses_out.append((n, m))
    if len(poses_out) < 2 * len(HAND_LINKS):
        return False
    return hands_clear_of(poses_out, st)


_SHOWN: dict = {}


def _show(sim, part, names_on) -> None:
    shown = _SHOWN.setdefault(part, set())
    for n in shown - names_on:
        sim.set_visible(part, n, False)
    for n in names_on - shown:
        sim.set_visible(part, n, True)
    _SHOWN[part] = set(names_on)


def _draw(sim) -> None:
    on = set()
    left = DOSE_CLEAN - _L["dose"]
    if left > 2.0 + EPS:
        on.add("debris0")
    elif left > 1.0 + EPS:
        on.add("debris1")
    elif left > EPS:
        on.add("debris2")
    for side in ("R", "L"):
        k = max((i for i, ml in enumerate(POOL_LEVELS) if _L["pool"][side] >= 0.6 * ml), default=None)
        if k is not None and _L["pool"][side] > 0.05:
            on.add(f"pool{side}{k}")
    if _L["throat"] > 0.05:
        on.add("throat")
    if _L["spill"] > 0.05:
        on.add("spill")
    _show(sim, "head", on)
    _show(sim, "syringe", {"jet"} if _L["flowing"] else set())


@parts.on_reset
def _on_reset(sim) -> None:
    reset_state()
    _L["last_t"] = None
    _SHOWN.clear()
    for part, vis in (("head", HEAD.visuals), ("syringe", SYRINGE.visuals)):
        for v in vis:
            sim.set_visible(part, v.name, False)
    _draw(sim)


@parts.on_step
def _on_step(sim) -> None:
    if _L.get("last_t") == sim.t:  # a second copy of this module sharing the record: step it once
        return
    _L["last_t"] = sim.t
    st = sim.states()
    clear = False  # only measured when the rest of the finished state holds; unknown counts as not clear
    if not failed() and _rinsed() and tools_home(st):
        try:
            clear = _hands_clear(sim, st)
        except Exception:  # unknown hand state: no credit
            clear = False
    record(st, clear)
    if os.environ.get("TASK_DEBUG") and _L["steps"] % 20 == 0:
        e = to_head(st["head"], st["ejector"].pos)
        n = to_head(st["head"], st["syringe"].pos)
        print(f"[water] t={_L['t']:.1f} dose={_L['dose']:.2f} pool={_L['pool']} transit={_L['transit']} "
              f"throat={_L['throat']:.2f} spill={_L['spill']:.2f} sucked={_L['sucked']:.2f} "
              f"ej={np.round(e * 1000, 1)} noz={np.round(n * 1000, 1)} hit={jet_landing(st['head'], st['syringe'])} "
              f"clear={clear} hold={_L['hold']}", flush=True)
    _draw(sim)


# --- demo: the left hand takes the ejector to the right pocket, the right hand aims the syringe at the molar --------
TIP = 0.006  # TCP above a handle's axis: the pads hold the handle's full height
GRIP_EJ = -(2 * S_HANDLE_R * math.cos(math.radians(22.5)) - 0.003) / 2  # pads closed 3 mm past the handle's flats
GRIP_SY = -(2 * Y_HANDLE_R * math.cos(math.radians(22.5)) - 0.003) / 2
LET_GO = -0.025
LIFT = 0.060
SAG = 0.003  # m: the right arm settles about this far low holding the syringe out at arm's length


def _head_pose():
    lot = lots.current()
    return np.asarray(lot.pos("head"), float), lot.yaw_of("head")


def _tool_world(p_head, yaw_head):
    """A tool pose given in the head frame (position, yaw), in world coordinates (the head as the cameras show it)."""
    o, a = _head_pose()
    return o + _Rz(a) @ np.asarray(p_head, float), yaw_head + a


def _tcp(tool_pos, tool_yaw, grip_local):
    return np.asarray(tool_pos, float) + _Rz(tool_yaw) @ np.asarray(grip_local, float) + np.array([0.0, 0.0, TIP])


def _grip_yaw(handle_dir_yaw, lo=-math.pi / 2):
    """Close across the handle: yaw 0 closes the fingers along world y, so a handle along heading h is closed on
    with yaw h. The gripper is symmetric, so the yaw is taken in [lo, lo + 180 deg)."""
    return (handle_dir_yaw - lo) % math.pi + lo


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def demo():
    """Both arms, open loop from the head's pose as the cameras show it. The left hand lifts the ejector off its rest,
    brings it level through the lips high enough for the bent tip to clear the lower front teeth, then lowers the
    tip into the pocket behind the right lower molars and holds it there through the rinse. The right hand brings the
    syringe in through the lips with its nozzle 25 mm short of the molar and aims the jet at it while the water runs.
    Both then come out the way they went in, go back on their rests and let go."""
    # ejector
    s_tip, s_yaw = _tool_world(S_TIP_USE, S_YAW_USE)
    up = np.array([0.0, 0.0, 0.0285])
    ax = _Rz(s_yaw) @ np.array([1.0, 0.0, 0.0])
    s_high = s_tip + up
    s_out = s_high - ax * 0.085
    s_hyaw = _wrap(s_yaw + math.atan2(S_U[1], S_U[0]))  # the handle's direction
    gy_s = _grip_yaw(s_hyaw)
    gy_s_rest = _grip_yaw(S_REST_YAW + math.atan2(S_U[1], S_U[0]))
    r_tcp = _tcp(S_REST, S_REST_YAW, S_GRIP)
    left = [
        Waypoint(2.0, tuple(r_tcp + [0, 0, LIFT]), FINGER_OPEN, "over the ejector", gy_s_rest),
        Waypoint(1.5, tuple(r_tcp), FINGER_OPEN, "down astride its handle", gy_s_rest),
        Waypoint(0.8, tuple(r_tcp), GRIP_EJ, "pinch the handle", gy_s_rest),
        Waypoint(1.5, tuple(r_tcp + [0, 0, LIFT]), GRIP_EJ, "lift", gy_s_rest),
        Waypoint(3.0, tuple(_tcp(s_out, s_yaw, S_GRIP)), GRIP_EJ, "in front of the lips", gy_s),
        Waypoint(2.5, tuple(_tcp(s_high, s_yaw, S_GRIP)), GRIP_EJ, "in over the teeth", gy_s),
        Waypoint(2.0, tuple(_tcp(s_tip, s_yaw, S_GRIP)), GRIP_EJ, "tip down into the pocket", gy_s),
        Waypoint(19.7, tuple(_tcp(s_tip, s_yaw, S_GRIP)), GRIP_EJ, "hold through the rinse", gy_s),
        Waypoint(1.5, tuple(_tcp(s_high, s_yaw, S_GRIP)), GRIP_EJ, "tip up", gy_s),
        Waypoint(2.0, tuple(_tcp(s_out, s_yaw, S_GRIP)), GRIP_EJ, "out of the mouth", gy_s),
        Waypoint(3.0, tuple(r_tcp + [0, 0, LIFT]), GRIP_EJ, "over the rest", gy_s_rest),
        Waypoint(1.5, tuple(r_tcp + [0, 0, 0.004]), GRIP_EJ, "onto the rest", gy_s_rest),
        Waypoint(0.8, tuple(r_tcp + [0, 0, 0.004]), LET_GO, "let go", gy_s_rest),
        Waypoint(1.5, tuple(r_tcp + [0, 0, LIFT]), LET_GO, "up", gy_s_rest),
        Waypoint(5.0, tuple(r_tcp + [0, 0, LIFT]), LET_GO, "wait", gy_s_rest),
    ]
    # syringe
    y_noz, y_yaw = _tool_world(Y_NOZ_USE + np.array([0.0, 0.0, SAG]), Y_HEADING)
    hx = _Rz(y_yaw) @ np.array([1.0, 0.0, 0.0])
    y_out = y_noz - hx * 0.055
    gy_y = _grip_yaw(_wrap(y_yaw + math.pi), 0.0)
    gy_y_rest = _grip_yaw(_wrap(Y_REST_YAW + math.pi), 0.0)
    q_tcp = _tcp(Y_REST, Y_REST_YAW, Y_GRIP)
    right = [
        Waypoint(4.0, tuple(q_tcp + [0, 0, LIFT]), FINGER_OPEN, "over the syringe", gy_y_rest),
        Waypoint(1.5, tuple(q_tcp), FINGER_OPEN, "down astride its handle", gy_y_rest),
        Waypoint(0.8, tuple(q_tcp), GRIP_SY, "pinch the handle", gy_y_rest),
        Waypoint(1.5, tuple(q_tcp + [0, 0, LIFT]), GRIP_SY, "lift", gy_y_rest),
        Waypoint(3.5, tuple(_tcp(y_out, y_yaw, Y_GRIP)), GRIP_SY, "in front of the lips", gy_y),
        Waypoint(2.5, tuple(_tcp(y_out, y_yaw, Y_GRIP)), GRIP_SY, "wait for the ejector's tip to go down", gy_y),
        Waypoint(2.5, tuple(_tcp(y_noz, y_yaw, Y_GRIP)), GRIP_SY, "nozzle in, aimed at the molar", gy_y),
        Waypoint(14.5, tuple(_tcp(y_noz, y_yaw, Y_GRIP)), GRIP_SY, "hold the aim while the water runs", gy_y),
        Waypoint(2.0, tuple(_tcp(y_out, y_yaw, Y_GRIP)), GRIP_SY, "out of the mouth", gy_y),
        Waypoint(3.5, tuple(q_tcp + [0, 0, LIFT]), GRIP_SY, "over the rest", gy_y_rest),
        Waypoint(1.5, tuple(q_tcp + [0, 0, 0.004]), GRIP_SY, "onto the rest", gy_y_rest),
        Waypoint(0.8, tuple(q_tcp + [0, 0, 0.004]), LET_GO, "let go", gy_y_rest),
        Waypoint(1.5, tuple(q_tcp + [0, 0, LIFT]), LET_GO, "up", gy_y_rest),
        Waypoint(5.0, tuple(q_tcp + [0, 0, LIFT]), LET_GO, "wait", gy_y_rest),
    ]
    return {"left": left, "right": right}
