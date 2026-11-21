"""On the station table: a Universal Robots UR5e arm, powered off, stands at the far end of the table with its
joints folded so that its wrist points back toward the two arms, low over the table. On its tool flange, on its
coupling, is a Robotiq 2F-85 two-finger gripper with its fingers parked fully open (85 mm between the fingertip
pads). Its black rubber palm pad lies between the bases of the two fingers, and each fingertip carries a black
rubber pad on its inner face. All three pads are dusted with white chalk powder. In front of the gripper a grey
anti-static service mat lies on the table, and on it a folded blue microfibre cloth.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import parts  # noqa: E402  (before lots, so parts' reset hooks run before the lots' moves)
import lots  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
EPS = 1e-6

UR_GREY, UR_CAP, DARK = (0.80, 0.81, 0.82), (0.30, 0.33, 0.37), (0.16, 0.16, 0.17)
BLACK, RUBBER, ALU = (0.09, 0.09, 0.10), (0.03, 0.03, 0.03), (0.62, 0.63, 0.65)
CHALK, MAT, CLOTH_BLUE, FOLD = (0.90, 0.89, 0.86), (0.36, 0.42, 0.48), (0.20, 0.45, 0.80), (0.12, 0.30, 0.58)

# -- where things are (world frame: x away from the arms, y to the robot's left, z up) ------------------------------
Z_AX = TOP + 0.085  # the gripper's axis: 85 mm above the table, level, pointing at the arms (-x)
X_PALM = 0.385  # the palm pad's face
X_G0 = X_PALM + 0.090  # the gripper's base face (its coupling side): the palm face is 90 mm out from it (Fig. 5-1)
COUPLING_T = 0.014  # coupling GRP-ES-CPL-062: 16.9 mm thick, the gripper's 71 mm spigot sits 2.9 mm into it
X_FLANGE = X_G0 + COUPLING_T  # the UR5e's tool flange face

# UR5e kinematics (Universal Robots' published DH parameters, m)
UR_D1, UR_A2, UR_A3, UR_D4, UR_D5, UR_D6 = 0.1625, 0.425, 0.3922, 0.1333, 0.0997, 0.0996
UR_LATERAL = (0.138, 0.007, 0.134)  # upper arm, forearm and wrist 1 planes, out from the base axis along the joints
W2 = np.array([X_FLANGE + UR_D6, 0.0, Z_AX])  # wrist 3's centre on the wrist-2 axis (vertical)
W1 = W2 + np.array([0.0, 0.0, UR_D5])  # wrist 1's centre, up the wrist-2 axis
UR_BASE = np.array([W2[0] + 0.180, -UR_D4, TOP])  # base on the table, 180 mm beyond the wrist


def _ur_elbow():
    """The elbow (in the x-z plane) for the folded pose that puts wrist 1 at W1: elbow up, toward the base."""
    s = np.array([UR_BASE[0], UR_BASE[2] + UR_D1])
    p = np.array([W1[0], W1[2]])
    d = p - s
    L = float(np.linalg.norm(d))
    alpha = math.acos((UR_A2 ** 2 + L ** 2 - UR_A3 ** 2) / (2 * UR_A2 * L))
    phi = math.atan2(d[1], d[0]) - alpha  # elbow above the line from shoulder to wrist
    return s, s + UR_A2 * np.array([math.cos(phi), math.sin(phi)]), p


SHOULDER, ELBOW, WRIST1 = _ur_elbow()


def _q_to(direction) -> tuple[float, float, float, float]:
    """xyzw turning +x onto ``direction``."""
    return tuple(float(v) for v in parts._matrix_to_quat(parts._x_to(direction)))


def _q_z(angle) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))


def _tube(a, b, radius, color, sides=20):
    """A round tube from world point a to world point b (UR5e part frame: relative to its base)."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    return parts.prism(radius, float(np.linalg.norm(b - a)), sides, at=tuple((a + b) / 2 - UR_BASE), color=color,
                       quat=_q_to(b - a))


def _cyl(center, radius, length, axis, color, collide=True):
    return parts.cylinder(radius, length, at=tuple(np.asarray(center, float) - UR_BASE), color=color, axis=axis,
                          collide=collide)


def _housing(center, radius, length, axis, cap_end, cap_r, cap_t=0.006):
    """A joint housing (a cylinder along ``axis``) with a dark end cap flush on its ``cap_end`` (+1 or -1) end."""
    c = np.asarray(center, float)
    k = "xyz".index(axis)
    cap = c.copy()
    cap[k] += cap_end * (length / 2 + cap_t / 2)
    return [_cyl(c, radius, length, axis, UR_GREY), _cyl(cap, cap_r, cap_t, axis, UR_CAP)]


def _ur_shapes():
    """The UR5e arm from its base to wrist 2 (fixed: powered off, its joint brakes on)."""
    sx, sz = SHOULDER
    ex, ez = ELBOW
    wx, wz = WRIST1
    y_ua, y_fa, y_w1 = (UR_BASE[1] + v for v in UR_LATERAL)
    b = UR_BASE
    out = [
        _cyl((b[0], b[1], TOP + 0.006), 0.0745, 0.012, "z", DARK),  # mounting flange, 149 mm
        _cyl((b[0], b[1], TOP + 0.012 + 0.050), 0.0575, 0.100, "z", UR_GREY),  # base joint
        *_housing((b[0], b[1] + 0.065, sz), 0.0575, 0.245, "y", 1, 0.050),  # shoulder joint, out to the upper arm
        _tube((sx, y_ua, sz), (ex, y_ua, ez), 0.0445, UR_GREY),  # upper arm
        *_housing((ex, (y_ua + y_fa) / 2, ez), 0.0480, 0.215, "y", 1, 0.042),  # elbow joint
        _tube((ex, y_fa, ez), (wx, y_fa, wz), 0.0375, UR_GREY),  # forearm
        *_housing((wx, (y_fa + y_w1) / 2 + 0.005, wz), 0.0400, 0.205, "y", -1, 0.035),  # wrist 1
        *_housing((W2[0], 0.0, (W2[2] + W1[2]) / 2 + 0.010), 0.0400, UR_D5 + 0.055, "z", 1, 0.035),  # wrist 2
        _cyl(((X_FLANGE + W2[0] + 0.040) / 2 + 0.003, 0.0, Z_AX), 0.0375, W2[0] + 0.040 - X_FLANGE - 0.006, "x",
             UR_GREY),  # wrist 3
    ]
    return out


# -- the Robotiq 2F-85, opened (Robotiq manual Fig. 5-1): its own frame, mm, Z along the fingers from the base face,
# X across the fingers (+X to the robot's left here), Y through its thickness. Its part frame here is the world's
# axes, so a point (X, Y, Z) of the drawing is at world (X_G0 - Z, X, Z_AX - Y). ---------------------------------
def gp(X, Y, Z):
    """A drawing point (mm) in the world (m)."""
    return np.array([X_G0 - Z / 1000.0, X / 1000.0, Z_AX - Y / 1000.0])


def _gbox(X0, X1, Y0, Y1, Z0, Z1, color, collide=True):
    c = gp((X0 + X1) / 2, (Y0 + Y1) / 2, (Z0 + Z1) / 2) - UR_BASE
    return parts.box((abs(Z1 - Z0) / 1000, abs(X1 - X0) / 1000, abs(Y1 - Y0) / 1000), at=tuple(c), color=color,
                     collide=collide)


def _gbar(XA, ZA, XB, ZB, width, depth, color):
    """A link bar between two pivots of the drawing's X-Z plane, ``width`` across in that plane, ``depth`` thick."""
    a, b = gp(XA, 0, ZA), gp(XB, 0, ZB)
    d = b - a
    length = float(np.linalg.norm(d)) + width / 1000
    return parts.box((length, width / 1000, depth / 1000), at=tuple((a + b) / 2 - UR_BASE), color=color,
                     quat=_q_z(math.atan2(d[1], d[0])))


PAD_TIP = (42.5, 50.3, 111.3, 149.3)  # fingertip pads: inner face at |X| 42.5, outer 50.3, Z 111.3 to 149.3
PAD_FACE_W = 16.2  # the rubber contact face across (Y), on a 22 mm wide fingertip
PALM = (15.5, 11.0, 88.0, 90.0)  # palm pad: |X| <= 15.5 (31 mm, Fig. 5-1), |Y| <= 11 (estimate), Z 88 to 90


def _gripper_shapes():
    """The coupling, the chassis and both fingers' links and fingertip holders (not the three pads)."""
    out = [
        parts.cylinder(0.0315, 0.006, at=tuple(np.array([X_FLANGE + 0.003, 0.0, Z_AX]) - UR_BASE), color=ALU,
                       axis="x"),  # UR tool flange, 63 mm
        parts.cylinder(0.0375, COUPLING_T, at=tuple(np.array([X_G0 + COUPLING_T / 2, 0.0, Z_AX]) - UR_BASE),
                       color=ALU, axis="x"),  # coupling, 75 mm
        parts.cylinder(0.0375, 0.0363, at=tuple(gp(0, 0, 36.3 / 2) - UR_BASE), color=DARK, axis="x"),  # base, 75 mm
        _gbox(-35, 35, -13.5, 13.5, 36.3, 44.0, BLACK),  # neck
        _gbox(-42, 42, -13.5, 13.5, 44.0, 74.0, BLACK),  # chassis, finger pivots
        _gbox(-25, 25, -13.5, 13.5, 74.0, 88.0, BLACK),  # chassis under the palm
    ]
    for s in (-1, 1):
        out += [
            _gbar(s * 12.5, 61.0, s * 50.0, 104.7, 11.0, 30.0, BLACK),  # inner (proximal) link
            _gbar(s * 62.0, 51.0, s * 68.0, 98.0, 10.0, 35.0, BLACK),  # outer link
            _gbox(*sorted((s * 42.5, s * 61.6)), -17.5, 17.5, 98.0, 111.3, BLACK),  # fingertip holder
        ]
    return out


def _tip_shapes(s):
    x0, x1, z0, z1 = PAD_TIP
    return [_gbox(*sorted((s * x0, s * x1)), -11.0, 11.0, z0, z1, RUBBER)]


def _palm_shapes():
    x, y, z0, z1 = PALM
    return [_gbox(-x, x, -y, y, z0, z1, RUBBER)]


# -- dust: cells on each pad's face. Palm: 6 x 4 cells, two zones (left and right half). Fingertips: 3 across x 6
# along, three zones each (the third nearest the palm, the middle third, the third at the tip). ---------------------
CELL_LIFT = 0.3  # mm proud of the face (visual only)
CELLS: list[dict] = []  # each: pad, zone, point (world), normal (world, out of the face), corners (world)


def _cell(pad, zone, centre_mm, half_u, half_v, u_axis, v_axis, normal):
    c = gp(*centre_mm)
    U, V = np.asarray(u_axis, float) * half_u / 1000, np.asarray(v_axis, float) * half_v / 1000
    corners = [c - U - V, c + U - V, c + U + V, c - U + V]
    CELLS.append(dict(pad=pad, zone=zone, point=c, normal=np.asarray(normal, float), corners=corners))


def _make_cells():
    x, y, z0, z1 = PALM
    nx, ny = 6, 4
    for i in range(nx):
        for j in range(ny):
            X = -x + (i + 0.5) * 2 * x / nx
            Y = -y + (j + 0.5) * 2 * y / ny
            zone = 0 if X > 0 else 1  # palm zone 0: the left half (robot's left), 1: the right half
            _cell("palm_pad", zone, (X, Y, z1 + CELL_LIFT), 0.9 * x / nx, 0.9 * y / ny, (0, 1, 0), (0, 0, 1),
                  (-1, 0, 0))
    xi, xo, z0, z1 = PAD_TIP
    na, nl = 3, 6
    for s, pad, base_zone in ((1, "tip_l", 2), (-1, "tip_r", 5)):
        for i in range(nl):
            for j in range(na):
                Z = z0 + (i + 0.5) * (z1 - z0) / nl
                Y = -PAD_FACE_W / 2 + (j + 0.5) * PAD_FACE_W / na
                zone = base_zone + i // 2  # 2/5 nearest the palm, 3/6 middle, 4/7 at the tip
                _cell(pad, zone, (s * (xi - CELL_LIFT), Y, Z), 0.45 * (z1 - z0) / nl, 0.45 * PAD_FACE_W / na,
                      (1, 0, 0), (0, 0, 1), (0, -s, 0))


_make_cells()
N_ZONES = 8
ZONE_NAMES = ("palm, left half", "palm, right half", "left fingertip, palm end", "left fingertip, middle",
              "left fingertip, tip end", "right fingertip, palm end", "right fingertip, middle", "right fingertip, tip end")
CELL_PAD = np.array([c["pad"] for c in CELLS])
CELL_ZONE = np.array([c["zone"] for c in CELLS])
CELL_P0 = np.array([c["point"] for c in CELLS])  # at the start (joint 0); the pads are fixed to the flange
CELL_N0 = np.array([c["normal"] for c in CELLS])
PAD_ORIGIN = {"palm_pad": gp(0, 0, (PALM[2] + PALM[3]) / 2),
              "tip_l": gp((PAD_TIP[0] + PAD_TIP[1]) / 2, 0, (PAD_TIP[2] + PAD_TIP[3]) / 2),
              "tip_r": gp(-(PAD_TIP[0] + PAD_TIP[1]) / 2, 0, (PAD_TIP[2] + PAD_TIP[3]) / 2)}


def _dust_visuals():
    out = []
    for k, c in enumerate(CELLS):
        pts = tuple(tuple(float(v) for v in (q - UR_BASE)) for q in c["corners"])
        q0, q1, q2 = c["corners"][:3]
        out_facing = float(np.cross(q1 - q0, q2 - q0) @ c["normal"]) > 0  # wind each quad to face out of its pad
        tris = ((0, 1, 2), (0, 2, 3)) if out_facing else ((0, 2, 1), (0, 3, 2))
        out.append(parts.Visual(f"dust{k}", pts, tris, color=CHALK, link=c["pad"]))
    return out


WRIST3 = parts.Hinge(
    "wrist3", shapes=_gripper_shapes(), pivot=tuple(np.array([X_FLANGE, 0.0, Z_AX]) - UR_BASE), axis=(1.0, 0.0, 0.0),
    lower=-0.02, upper=0.02, mass=0.95, spring=2000.0, damping=20.0, max_effort=500.0)  # braked: held at 0
PADS = [parts.Fixed("palm_pad", shapes=_palm_shapes(), at=tuple(PAD_ORIGIN["palm_pad"] - UR_BASE), mass=0.01,
                    parent="wrist3"),
        parts.Fixed("tip_l", shapes=_tip_shapes(1), at=tuple(PAD_ORIGIN["tip_l"] - UR_BASE), mass=0.01,
                    parent="wrist3"),
        parts.Fixed("tip_r", shapes=_tip_shapes(-1), at=tuple(PAD_ORIGIN["tip_r"] - UR_BASE), mass=0.01,
                    parent="wrist3")]
PAD_NAMES = ("palm_pad", "tip_l", "tip_r")
ROBOT = parts.Part("ur5e", pos=tuple(float(v) for v in UR_BASE), shapes=_ur_shapes(), joints=[WRIST3, *PADS],
                   friction=(1.0, 0.9), report=PAD_NAMES, visuals=_dust_visuals(), contact_offset=0.002)

# -- the service mat (fixed) and the cloth (loose) ------------------------------------------------------------------
MAT_SIZE = (0.305, 0.610, 0.002)  # 12 x 24 in anti-static mat, 2 mm
MAT_CENTER = (0.2175, 0.0)
MAT_TOP = TOP + MAT_SIZE[2]
SERVICE_MAT = parts.Part("service_mat", pos=(MAT_CENTER[0], MAT_CENTER[1], TOP + MAT_SIZE[2] / 2),
                         shapes=[parts.box(MAT_SIZE, color=MAT)], friction=(0.8, 0.7))
CLOTH_SIZE = (0.120, 0.040, 0.028)  # folded strip: length (x), width (y), thickness (z)
CLOTH_MASS = 0.035
CLOTH_START = (0.180, 0.200, MAT_TOP + CLOTH_SIZE[2] / 2)


def _cloth_shapes():
    L, W, H = CLOTH_SIZE
    out = [parts.box(CLOTH_SIZE, color=CLOTH_BLUE)]
    for x in (-L / 6, L / 6):  # fold lines, drawn only
        out.append(parts.box((0.002, W + 0.0004, H + 0.0004), at=(x, 0.0, 0.0), color=FOLD, collide=False))
    return out


CLOTH = parts.Part("cloth", pos=CLOTH_START, shapes=_cloth_shapes(), mass=CLOTH_MASS, friction=(0.7, 0.6),
                   report=("base",), contact_offset=0.002, angular_damping=2.0)

OBJECTS = [ROBOT, SERVICE_MAT, CLOTH]

# Layout lots: trial k uses lot k mod 2. The cloth starts on the mat's left half (A) or right half (B).
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"cloth": CLOTH_START}),
    lots.Lot("B", place={"cloth": (CLOTH_START[0], -CLOTH_START[1], CLOTH_START[2])}),
)

# -- the cleaning rule and the trial record --------------------------------------------------------------------------
WIPE_TRAVEL = 0.020  # m of cloth sliding over a dust cell, under load, clears it
F_MIN = 0.5  # N between the cloth and that pad: at least this much to wipe
STEP_MIN, STEP_MAX = 0.00025, 0.012  # m a step: slower than 5 mm/s is a press, not a wipe; over 12 mm is a jump
COVER = 0.0015  # a cell is under the cloth when it lies inside the cloth's box grown by 1.5 mm
CLEAN_FRACTION = 0.9  # a zone is clean when at most 10% of its cells still carry dust
HAND_CLEAR = 0.030  # m: hands this far from the gripper and the cloth at the end
TOUCH_N = 0.5  # N: a gripper link or finger on a pad above this rules the trial out
HOLD_STEPS = int(SUCCESS_HOLD_S * 20)
_S = parts.__dict__.setdefault("_robotiq_pads_state", {})


def reset_state():
    hidden = set(_S.get("hidden", ()))
    _S.clear()
    _S.update(travel=np.zeros(len(CELLS)), prev=None, known=False, broken=False, dropped=False, touched=False,
              clear=False, cloth_free=False, hold=0, views=None, hidden=hidden, maxforce=0.0, path=0.0, log_t=-1.0)


if not _S:
    reset_state()
    _S["hidden"] = set()


def _cloth_frame(st):
    return np.asarray(st.pos, float), parts._quat_to_matrix(st.quat_xyzw)


def pad_cells(robot_state):
    """World points and normals of every dust cell, from the pads' current poses."""
    pts, nrm = CELL_P0.copy(), CELL_N0.copy()
    for pad in PAD_NAMES:
        p, q = robot_state.links[pad]
        r = parts._quat_to_matrix(q)
        m = CELL_PAD == pad
        pts[m] = (CELL_P0[m] - PAD_ORIGIN[pad]) @ r.T + np.asarray(p, float)
        nrm[m] = CELL_N0[m] @ r.T
    return pts, nrm


def record_wipe(forces, cloth_pose, cells):
    """One control step of cloth contact. ``forces``: N between the cloth and each pad (palm_pad, tip_l, tip_r);
    ``cloth_pose``: (centre, rotation) of the cloth; ``cells``: (points, normals) of the dust cells. A cell gains the
    distance the cloth slid over it since the last step, along the pad's face, while it lies under the cloth and
    that pad has carried at least F_MIN on both steps. Steps under STEP_MIN (5 mm/s: a held press that only
    jitters) or over STEP_MAX count nothing."""
    c, r = cloth_pose
    pts, nrm = cells
    local = (pts - c) @ r  # cells in the cloth's frame
    half = np.asarray(CLOTH_SIZE) / 2 + COVER
    under = np.all(np.abs(local) <= half + EPS, axis=1)
    loaded = np.array([forces.get(p, 0.0) >= F_MIN - EPS for p in CELL_PAD])
    prev = _S["prev"]
    _S["prev"] = (local, loaded)
    if prev is None:
        return
    p_local, p_loaded = prev
    slide = (local - p_local) @ r.T  # world displacement of the cloth relative to each cell
    tangential = slide - nrm * np.sum(slide * nrm, axis=1)[:, None]
    step = np.linalg.norm(tangential, axis=1)
    ok = under & loaded & p_loaded & (step >= STEP_MIN - EPS) & (step <= STEP_MAX + EPS)
    _S["travel"][ok] += step[ok]
    _S["path"] += float(step[ok].max()) if ok.any() else 0.0


def no_wipe():
    _S["prev"] = None


def zones_clean() -> list[bool]:
    done = _S["travel"] >= WIPE_TRAVEL - EPS
    return [bool(np.mean(done[CELL_ZONE == z]) >= CLEAN_FRACTION - EPS) for z in range(N_ZONES)]


def n_clean() -> int:
    return int(sum(zones_clean()))


# -- end-state geometry --------------------------------------------------------------------------------------------
def cloth_corners(st):
    c, r = _cloth_frame(st)
    h = np.asarray(CLOTH_SIZE) / 2
    signs = np.array([[a, b, d] for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)], float)
    return c + (signs * h) @ r.T


def cloth_on_mat(st) -> bool:
    """Lying on the mat: its lowest corner within 3 mm of the mat's top, flat within 10 degrees, all of it over
    the mat."""
    k = cloth_corners(st)
    r = parts._quat_to_matrix(st.quat_xyzw)
    flat = abs(float(r[2, 2])) >= math.cos(math.radians(10.0)) - EPS
    low = float(k[:, 2].min())
    inside = (np.all(np.abs(k[:, 0] - MAT_CENTER[0]) <= MAT_SIZE[0] / 2 + EPS)
              and np.all(np.abs(k[:, 1] - MAT_CENTER[1]) <= MAT_SIZE[1] / 2 + EPS))
    return bool(flat and inside and MAT_TOP - 0.001 - EPS <= low <= MAT_TOP + 0.003 + EPS)


def gripper_boxes(robot_state):
    """(centre, rotation, size) of every collider of the gripper, its coupling and its pads, where they are now."""
    poses = {"base": (np.asarray(robot_state.pos, float), parts._quat_to_matrix(robot_state.quat_xyzw))}
    for name, (p, q) in robot_state.links.items():
        poses[name] = (np.asarray(p, float), parts._quat_to_matrix(q))
    boxes = ROBOT.world_boxes(poses)
    return [(c, r, s) for link, c, r, s in boxes if link in ("wrist3", *PAD_NAMES)]


def _cloth_surface(st, step=0.004):
    c, r = _cloth_frame(st)
    h = np.asarray(CLOTH_SIZE) / 2
    g = [np.arange(-h[i], h[i] + 1e-9, step) for i in range(3)]
    pts = np.array([[a, b, d] for a in g[0] for b in g[1] for d in g[2]
                    if abs(abs(a) - h[0]) < 1e-9 or abs(abs(b) - h[1]) < 1e-9 or abs(abs(d) - h[2]) < 1e-9])
    return c + pts @ r.T


def cloth_off_gripper(objects) -> bool:
    """The cloth is at least 2 mm from every part of the gripper (not left in it or resting on it)."""
    from yam_arena import analysis

    pts = _cloth_surface(objects["cloth"])
    return all(float(analysis.box_signed_distance(pts, c, r, s).min()) >= 0.002 - EPS
               for c, r, s in gripper_boxes(objects["ur5e"]))


# -- scoring ---------------------------------------------------------------------------------------------------------
def _ruled_out() -> bool:
    return bool(not _S["known"] or _S["broken"] or _S["dropped"] or _S["touched"])


def score(objects):
    """Clean zones, judged once both hands are 30 mm clear of the gripper and the cloth, the cloth is off the
    gripper, and everything has been still for 1 s."""
    if _ruled_out() or not _S["clear"] or _S["hold"] < HOLD_STEPS or not objects["cloth"].resting():
        return 0.0
    if not cloth_off_gripper(objects):
        return 0.0
    n = n_clean()
    if n == N_ZONES and cloth_on_mat(objects["cloth"]):
        return 1.0
    if n == N_ZONES:
        return 0.75
    if n >= 5:
        return 0.5
    if n >= 2:
        return 0.25
    return 0.0


def success(objects):
    return score(objects) == 1.0


# -- simulator side: contact views, hands, the dust's look ------------------------------------------------------------
HAND_LINKS = ("gripper", "tip_left", "tip_right")
_FACE_PTS: dict[str, np.ndarray] = {}


def _arr(x) -> np.ndarray:
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy())
    return np.asarray(x)


def _descend(prim):
    for c in prim.GetAllChildren():
        yield c
        yield from _descend(c)


def _make_views():
    from isaaclab.sim.utils import get_current_stage
    from isaaclab_physx.physics import PhysxManager
    from pxr import UsdPhysics

    sv = PhysxManager.get_physics_sim_view()
    robot, cloth = parts._PATHS["ur5e"], parts._PATHS["cloth"]
    env = robot.rsplit("/", 1)[0]
    hands = []
    for prim in get_current_stage().GetPrimAtPath(env).GetAllChildren():
        if "Arm" not in prim.GetName():
            continue
        for p in [prim, *list(_descend(prim))]:
            if p.GetName() in HAND_LINKS and p.HasAPI(UsdPhysics.RigidBodyAPI):
                hands.append(p.GetPath().pathString)
    wipe = {pad: sv.create_rigid_contact_view(cloth, filter_patterns=[f"{robot}/{pad}"], max_contact_data_count=64)
            for pad in PAD_NAMES}
    guard = [sv.create_rigid_contact_view(f"{robot}/{pad}", filter_patterns=hands, max_contact_data_count=64)
             for pad in PAD_NAMES] if hands else []
    print(f"[pads] contact views: cloth->3 pads, 3 pads->{len(hands)} hand links", file=sys.stderr, flush=True)
    if len(hands) != 6:  # the hands-on-pads rule cannot be judged: no credit for this trial
        _S["broken"] = True
    return {"wipe": wipe, "guard": guard}


def _hull_surface(h, spacing=0.003):
    from scipy.spatial import ConvexHull

    v = h.vertices
    out = [v]
    for tri in ConvexHull(v).simplices:
        a, b, c = v[tri]
        n = max(1, int(math.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / spacing)))
        out.append(np.array([a + (b - a) * u / n + (c - a) * w / n for u in range(n + 1) for w in range(n + 1 - u)]))
    return np.unique(np.round(np.concatenate(out) / 0.0025).astype(np.int64), axis=0) * 0.0025


def hand_points(base) -> np.ndarray:
    """Points on the faces (every 3 mm) of both arms' gripper and finger collision hulls, in the world."""
    from yam_arena import analysis

    origin = parts._np(base.scene.env_origins[0])
    pts = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n not in HAND_LINKS:
                continue
            if n not in _FACE_PTS:
                _FACE_PTS[n] = np.concatenate([_hull_surface(h) for h in analysis.link_hulls().get(n, [])])
            pts.append(_FACE_PTS[n] @ parts._quat_to_matrix(poses[i, 3:]).T + (poses[i, :3] - origin))
    if len(pts) != 6:
        raise RuntimeError("hand links not found")
    return np.concatenate(pts)


def hands_clear(points, objects) -> bool:
    from yam_arena import analysis

    c, r = _cloth_frame(objects["cloth"])
    boxes = gripper_boxes(objects["ur5e"]) + [(c, r, np.asarray(CLOTH_SIZE))]
    return all(float(analysis.box_signed_distance(points, bc, br, bs).min()) >= HAND_CLEAR - EPS
               for bc, br, bs in boxes)


@parts.on_reset
def _on_reset(sim) -> None:
    views = _S.get("views")
    hidden = set(_S.get("hidden", ()))
    reset_state()
    _S["views"] = views
    for name in hidden:
        try:
            sim.set_visible("ur5e", name, True)
        except Exception as exc:  # noqa: BLE001
            print(f"[pads] could not show {name}: {exc}", file=sys.stderr)
    _S["hidden"] = set()


@parts.on_step
def _on_step(sim) -> None:
    try:
        base = sim.env.unwrapped
        if _S.get("views") is None:
            _S["views"] = _make_views()
        v = _S["views"]
        dt = base.physics_dt
        st = sim.states()
        cl, rb = st["cloth"], st["ur5e"]
        forces = {pad: float(np.linalg.norm(_arr(v["wipe"][pad].get_contact_force_matrix(dt=dt)).reshape(-1, 3)
                                            .sum(axis=0))) for pad in PAD_NAMES}
        _S["maxforce"] = max(_S["maxforce"], *forces.values())
        if max(forces.values()) > 0.05:
            record_wipe(forces, _cloth_frame(cl), pad_cells(rb))
        else:
            no_wipe()
        for g in v["guard"]:
            if float(np.linalg.norm(_arr(g.get_contact_force_matrix(dt=dt)).reshape(-1, 3), axis=1).max()) > TOUCH_N:
                _S["touched"] = True
        if float(cloth_corners(cl)[:, 2].min()) < TOP - 0.010 or abs(cl.pos[1]) > 0.64 or not (-0.18 < cl.pos[0] < 0.90):
            _S["dropped"] = True
        _S["clear"] = hands_clear(hand_points(base), st)
        _S["known"] = True
        _S["hold"] = _S["hold"] + 1 if (_S["clear"] and cl.resting()) else 0
        done = _S["travel"] >= WIPE_TRAVEL - EPS
        for k in np.flatnonzero(done):
            name = f"dust{k}"
            if name not in _S["hidden"]:
                sim.set_visible("ur5e", name, False)
                _S["hidden"].add(name)
        if sim.t - _S["log_t"] >= 1.0 - 1e-6:
            _S["log_t"] = sim.t
            print("[pads] " + json.dumps(dict(
                t=round(float(sim.t), 2), forces={k: round(f, 2) for k, f in forces.items()},
                maxforce=round(_S["maxforce"], 2), zones=[int(np.sum(done[CELL_ZONE == z])) for z in range(N_ZONES)],
                clean=n_clean(), touched=_S["touched"], dropped=_S["dropped"], clear=_S["clear"], hold=_S["hold"],
                cloth=[round(float(x), 4) for x in cl.pos], cloth_q=[round(float(x), 4) for x in cl.quat_xyzw],
                wrist3=round(float(rb.joints.get("wrist3", 0.0)), 5))), flush=True)
    except Exception as exc:  # unreadable contacts, hands or poses: no credit for this trial
        _S["broken"] = True
        print(f"[pads] step check failed: {exc!r}", file=sys.stderr, flush=True)


# -- the scripted demo: one arm, open loop from the visible layout (the cloth's place on the mat; the gripper and
# its pads are fixed and in view). -----------------------------------------------------------------------------------
GRIP_BACK = 0.018  # the TCP holds the cloth this far in from its rear end
GRIP = -0.015  # finger position: 30 mm between the pads, 5 mm into each side of the 40 mm cloth
TCP_ABOVE = 0.006  # TCP above the cloth's centre line when holding it
PUSH_X = 0.006  # commanded past the palm face (the arm gives way, so this sets a light press)
PUSH_Y = 0.009  # commanded past a fingertip pad's face
SWEEP_Y = 0.008  # palm sweep, either way
TIP_DEEP, TIP_STROKE = 0.016, 0.020  # fingertip wipe: the cloth's end from 16 to 36 mm short of the palm
REPS_PALM, REPS_TIP = 5, 10  # strokes each way: per height on the palm; per fingertip
UP = np.array([0.0, 0.0, 1.0])


def _tcp_for_front(front_x, y, z):
    """TCP for the cloth's front end at front_x, its centre line at (y, z)."""
    return np.array([front_x - CLOTH_SIZE[0] + GRIP_BACK, y, z + TCP_ABOVE])


def _wp(dur, pos, finger, label):
    return Waypoint(float(dur), tuple(float(v) for v in pos), float(finger), label, 0.0)


def _finger_wipe(arm, other):
    """PADS_DEMO=fingers: the shortcut of rubbing the fingertip pads with the closed robot fingers, no cloth (a
    check that the touch rule scores it 0, not the demo)."""
    x, z = X_PALM - 0.040, Z_AX + 0.004
    w = [_wp(2.5, (x - 0.06, 0.0, z + 0.06), -0.002, "over the gap, fingers closed"),
         _wp(1.5, (x, 0.0, z), -0.002, "down between the fingertips")]
    for s in (1, -1):
        for k in range(3):
            w.append(_wp(0.8, (x, s * 0.042, z), -0.002, "rub a fingertip pad"))
            w.append(_wp(0.8, (x - 0.01, s * 0.042, z), -0.002, "rub a fingertip pad"))
        w.append(_wp(0.8, (x, 0.0, z), -0.002, "back to the middle"))
    w += [_wp(1.5, (x - 0.06, 0.0, z + 0.06), -0.002, "out"),
          _wp(1.5, (0.15, (0.26 if arm == "left" else -0.26), TOP + 0.20), FINGER_OPEN, "clear"),
          _wp(2.0, (0.15, (0.26 if arm == "left" else -0.26), TOP + 0.20), FINGER_OPEN, "wait")]
    total = sum(x.duration for x in w)
    return {arm: w, other: [_wp(total, (0.15, (0.26 if other == "left" else -0.26), TOP + 0.20), FINGER_OPEN,
                                "stay clear")]}


def demo():
    lot = lots.current()
    cx, cy, cz = lot.pos("cloth")  # where the cloth lies on the mat, as the camera sees it
    arm = "left" if cy > 0 else "right"
    other = "right" if arm == "left" else "left"
    if os.environ.get("PADS_DEMO") == "fingers":
        return _finger_wipe(arm, other)
    pick = np.array([cx - CLOTH_SIZE[0] / 2 + GRIP_BACK, cy, cz + TCP_ABOVE])
    out = X_PALM - 0.080  # cloth front outside the fingertips (they end 59 mm in front of the palm)
    z = Z_AX
    w = [
        _wp(2.0, pick + 0.08 * UP, FINGER_OPEN, "over the cloth's near end"),
        _wp(1.5, pick, FINGER_OPEN, "down"),
        _wp(0.8, pick, GRIP, "pinch the cloth"),
        _wp(1.2, pick + 0.06 * UP, GRIP, "lift"),
        _wp(2.5, _tcp_for_front(out, 0.0, z), GRIP, "carry it level to the gripper's mouth"),
    ]
    # the palm pad: press the cloth's end on it and sweep across it, at two heights (the links in front of the
    # palm are 30 mm out from its centre line, so the 40 mm cloth sweeps no more than 8 mm either way)
    for dz in (0.004, -0.001):
        w.append(_wp(1.2, _tcp_for_front(X_PALM - 0.010, 0.0, z + dz), GRIP, "in toward the palm"))
        w.append(_wp(0.8, _tcp_for_front(X_PALM + PUSH_X, 0.0, z + dz), GRIP, "end on the palm pad"))
        for k in range(REPS_PALM):
            for sy in (SWEEP_Y, -SWEEP_Y):
                w.append(_wp(0.7, _tcp_for_front(X_PALM + PUSH_X, sy, z + dz), GRIP, "sweep the palm"))
        w.append(_wp(0.8, _tcp_for_front(X_PALM - 0.020, 0.0, z + dz), GRIP, "ease off"))
    # each fingertip pad: the cloth's side pressed on it and slid in and out along the finger, its end between 16
    # and 36 mm short of the palm (deeper, the finger's inner link stands in the way)
    zt = z
    for s, name in ((1, "left"), (-1, "right")):
        free_y = s * 0.010
        press_y = s * (PAD_TIP[0] / 1000 - CLOTH_SIZE[1] / 2 + PUSH_Y)
        deep, shallow = X_PALM - TIP_DEEP, X_PALM - TIP_DEEP - TIP_STROKE
        w.append(_wp(1.2, _tcp_for_front(out, free_y, z), GRIP, f"line up with the {name} fingertip"))
        w.append(_wp(1.0, _tcp_for_front(deep, free_y, zt), GRIP, "in between the fingers"))
        w.append(_wp(0.8, _tcp_for_front(deep, press_y, zt), GRIP, f"side of the cloth on the {name} pad"))
        for k in range(REPS_TIP):  # out along the finger and back in (strokes along the cloth do not tip it)
            w.append(_wp(0.8, _tcp_for_front(shallow, press_y, zt), GRIP, "wipe out along the pad"))
            w.append(_wp(0.8, _tcp_for_front(deep, press_y, zt), GRIP, "wipe back in"))
        w.append(_wp(0.5, _tcp_for_front(deep, press_y, zt), GRIP, "centre on the pad"))
        w.append(_wp(0.8, _tcp_for_front(deep, free_y, zt), GRIP, "off the pad"))
        w.append(_wp(1.0, _tcp_for_front(out, free_y, z), GRIP, "out of the gripper"))
    drop = np.array([cx - CLOTH_SIZE[0] / 2 + GRIP_BACK, cy, cz + TCP_ABOVE + 0.006])
    w += [
        _wp(2.5, drop + 0.06 * UP, GRIP, "carry the cloth back over the mat"),
        _wp(1.2, drop, GRIP, "lower it"),
        _wp(0.8, drop, FINGER_OPEN, "let go"),
        _wp(1.0, drop + 0.10 * UP, FINGER_OPEN, "up"),
        _wp(1.5, (0.15, (0.26 if arm == "left" else -0.26), TOP + 0.20), FINGER_OPEN, "clear"),
        _wp(2.0, (0.15, (0.26 if arm == "left" else -0.26), TOP + 0.20), FINGER_OPEN, "wait"),
    ]
    total = sum(x.duration for x in w)
    park = (0.15, (0.26 if other == "left" else -0.26), TOP + 0.20)
    return {arm: w, other: [_wp(total, park, FINGER_OPEN, "stay clear")]}
