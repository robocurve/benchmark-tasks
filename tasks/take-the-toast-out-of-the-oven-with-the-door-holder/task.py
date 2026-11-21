"""A stainless countertop air fryer oven with two glass French doors stands on the counter, facing the robot; its
display is lit. Both doors are shut; each has a vertical metal handle near the middle seam. A black magnetic door
holder is fixed to the oven's left front corner. Inside, on the wire rack in its lowest position, a small aluminium sheet pan
holds two slices of toast. A round cork trivet lies on the counter in front of the oven. Both arms start at their
ready pose with the grippers open.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys
from functools import lru_cache

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import parts  # noqa: E402  (before lots: its reset hooks run first)
import lots  # noqa: E402
import tilt  # noqa: E402
from yam_arena.demo import FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.1  # the 1 s release-and-still hold is counted inside score()
INSTRUCTION = "Put the pan of toast on the trivet."
MM = 1e-3
EPS = 1e-6
D = math.radians

STEEL, DARK, GLASS, BLACK = (0.70, 0.71, 0.72), (0.16, 0.16, 0.17), (0.20, 0.25, 0.27), (0.06, 0.06, 0.07)
LIT = (1.00, 0.45, 0.08)  # the control panel's lit display
ALU, TOAST, CRUMB, CORK, MAGNET = (0.80, 0.80, 0.78), (0.64, 0.40, 0.17), (0.86, 0.70, 0.45), (0.72, 0.55, 0.36), (0.55, 0.56, 0.58)

# -- the oven: Gourmia GTF7530, 553 W x 398 D x 439 H mm overall (retail listing), 8.5 kg, fixed -------------------
# World frame: x away from the arms, y to the robot's left, z up from the counter. The oven's front face is the plane
# x = FACE. The door pair is centred on y = 0; the control panel is on the oven's right (the robot's -y side).
FACE = 0.400
DEPTH, BODY_L, BODY_R, BODY_H = 0.398, 0.222, -0.331, 0.439
AP_Y, AP_Z0, AP_Z1 = 0.190, 0.075, 0.365  # front opening of the cooking cavity (half width, bottom, top)
CAV_BACK = FACE + 0.345  # inner face of the cavity's back wall
HX, HY = 0.388, 0.207  # hinge axes (vertical), 12 mm in front of the face
DW, DT, DZ0, DZ1 = 0.205, 0.020, 0.055, 0.400  # door: hinge axis to free edge, thickness, bottom and top
DBACK = 0.006  # door stile behind the hinge axis
SH, SO = 0.175, 0.045  # handle: along the door from the hinge axis, off the door's mid-plane (outer side)
BAR_R, BAR_Z0, BAR_Z1 = 0.008, 0.120, 0.340
OPEN_DEG = 105.0  # fully open: the left door's outer frame meets the holder's two magnets
RACK_TOP = 0.105  # wire rack, lowest of five positions (a flat plate here)
RACK_X1 = CAV_BACK - 0.010

# Door springs and the holder magnet (estimates; one drive on the left door acts on the linked pair).
SPRING_K = 0.30  # N m/rad, both doors together (0.15 each)
SPRING_REST = -D(95.0)  # rest point behind shut: 0.50 N m closing torque when shut, 1.05 N m at full open
# Holder magnets: two 22 mm magnets 28 mm from the left hinge axis, 93 N pull together at contact, falling to a quarter
# by a 2 mm gap (estimates). The pull, less the springs, is applied by the left door's drive while it wins.
MAG_R, MAG_PULL, MAG_FADE = 0.028, 93.0, 0.002
MAG_CATCH = D(OPEN_DEG - 2.0)  # the magnets take a door brought within 2 degrees of the holder
MAG_AIM, MAG_K_MAX = D(OPEN_DEG + 1.0), 150.0
DOOR_MASS, DOOR_DAMP = 0.70, 0.08


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _qz(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


ORIGIN = np.array([FACE, 0.0, 0.0])  # the oven part's frame: front face, middle of the door pair, counter level


def _p(x, y, z):
    """World (x, y, height) to the oven part's frame."""
    return (float(x - ORIGIN[0]), float(y - ORIGIN[1]), float(z))


def door_shapes(side):
    """One door shut (joint 0), in the oven frame. side +1: left door (hinge at +y), -1: right door."""
    yc = side * (HY - (DW - DBACK) / 2)  # door centre across y
    wide = DW + DBACK
    zc, tall = (DZ0 + DZ1) / 2, DZ1 - DZ0
    s = [parts.box((DT, wide, tall), _p(HX, yc, zc), GLASS)]  # the door slab (glass in a frame), collides
    for dz in (DZ0 + 0.012, DZ1 - 0.012):  # frame rails and stiles, drawn proud of the glass
        s.append(parts.box((DT + 0.002, wide, 0.024), _p(HX, yc, dz), DARK, collide=False))
    for yy in (side * (HY + DBACK - 0.012), side * (HY - DW + 0.012)):
        s.append(parts.box((DT + 0.002, 0.024, tall), _p(HX, yy, zc), DARK, collide=False))
    hy = side * (HY - SH)
    s.append(parts.cylinder(BAR_R, BAR_Z1 - BAR_Z0, _p(HX - SO, hy, (BAR_Z0 + BAR_Z1) / 2), STEEL))
    for zz in (BAR_Z0 + 0.010, BAR_Z1 - 0.010):
        s.append(parts.box((SO - DT / 2, 0.012, 0.012), _p(HX - (SO + DT / 2) / 2, hy, zz), STEEL))
    return tuple(s)


def _holder_shapes():
    """The recall door holder: a mounting leg on the left side wall and a magnet block facing the open door."""
    a = D(OPEN_DEG)
    d = np.array([-math.sin(a), -math.cos(a), 0.0])  # along the open left door, hinge to free edge
    n = np.array([-math.cos(a), math.sin(a), 0.0])  # the open door's outer face normal
    hinge = np.array([HX, HY, 0.0])
    yaw = math.atan2(d[1], d[0])  # block frame: x along the door, y off its face
    out = []
    face0 = hinge + 0.028 * d + (DT / 2 + 0.0005) * n  # magnet faces 0.5 mm off the open door's outer face
    block = face0 + 0.0125 * n
    for zc in (0.235,):
        out.append(parts.box((0.046, 0.016, 0.090), _p(block[0], block[1], zc), BLACK, quat=_qz(yaw)))
    for zc in (0.212, 0.258):  # two round magnets on the face toward the door
        m = face0 + 0.0025 * n
        cq = parts._matrix_to_quat(_rz(yaw) @ parts._rot((1, 0, 0), math.pi / 2))
        out.append(parts.Shape("cylinder", (0.011, 0.005, 0.0), _p(m[0], m[1], zc), MAGNET, "z", tuple(float(v) for v in cq)))
    out.append(parts.box((0.040, 0.022, 0.090), _p(FACE - 0.005, BODY_L + 0.011, 0.235), BLACK))  # leg on the side wall
    return out


def _body_shapes():
    def ab(x0, x1, y0, y1, z0, z1, col=STEEL, collide=True):
        return parts.box((x1 - x0, y1 - y0, z1 - z0), _p((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), col, collide=collide)

    back = FACE + DEPTH
    s = [ab(FACE, back, BODY_R, BODY_L, AP_Z1, BODY_H),  # top
         ab(FACE, back, BODY_R, BODY_L, 0.025, AP_Z0),  # floor (crumb tray level) and base
         ab(FACE, back, AP_Y, BODY_L, AP_Z0, AP_Z1),  # left wall
         ab(FACE, back, BODY_R, -AP_Y, AP_Z0, AP_Z1, DARK),  # right section behind the control panel
         ab(CAV_BACK, back, -AP_Y, AP_Y, AP_Z0, AP_Z1),  # back wall
         ab(FACE + 0.005, RACK_X1, -AP_Y + 0.005, AP_Y - 0.005, RACK_TOP - 0.004, RACK_TOP, STEEL)]  # wire rack
    for x0, y0 in ((FACE + 0.03, BODY_L - 0.04), (FACE + 0.03, BODY_R + 0.01), (back - 0.06, BODY_L - 0.04), (back - 0.06, BODY_R + 0.01)):
        s.append(ab(x0, x0 + 0.03, y0, y0 + 0.03, 0.0, 0.025, BLACK))  # feet
    s.append(ab(FACE - 0.002, FACE + 0.001, BODY_R + 0.015, -0.225, 0.300, 0.345, BLACK, collide=False))  # display
    s.append(ab(FACE - 0.004, FACE - 0.001, BODY_R + 0.030, -0.240, 0.312, 0.333, LIT, collide=False))  # its lit readout
    s.append(parts.cylinder(0.022, 0.012, _p(FACE - 0.006, -0.278, 0.230), BLACK, axis="x"))  # selector dial
    for zz in (0.120, 0.160):  # buttons
        s.append(ab(FACE - 0.004, FACE, -0.300, -0.255, zz, zz + 0.022, DARK, collide=False))
    return s + _holder_shapes()


OVEN = parts.Part(
    "oven", pos=tuple(float(v) for v in ORIGIN + np.array([0.0, 0.0, TOP])), shapes=_body_shapes(),
    contact_offset=0.002, friction=(0.30, 0.25),
    joints=[
        parts.Hinge("left_door", door_shapes(1), _p(HX, HY, 0.0), (0.0, 0.0, -1.0), 0.0, D(OPEN_DEG),
                    mass=DOOR_MASS, spring=SPRING_K, damping=DOOR_DAMP, target=SPRING_REST, max_effort=6.0),
        parts.Hinge("right_door", door_shapes(-1), _p(HX, -HY, 0.0), (0.0, 0.0, 1.0), 0.0, D(OPEN_DEG),
                    mass=DOOR_MASS, spring=0.0, damping=DOOR_DAMP, max_effort=6.0, gear=("left_door", 1.0))])

# -- the pan: Nordic Ware Naturals eighth sheet, 257 x 178 x 28 mm outside (maker), aluminium, about 0.2 kg -----------
# It sits with its long side across the oven (along y): PAN_L is its depth along x, PAN_W its width along y.
PAN_L, PAN_W, PAN_H = 0.178, 0.257, 0.028
PAN_FLOOR, PAN_WALL, BEAD = 0.002, 0.0015, 0.008  # floor and wall sheet; the rolled rim is an 8 mm bead
PAN_M = 0.20
PAN_FRONT = FACE + 0.030  # its near wall's outer face, on the rack
PAN_POS = (PAN_FRONT + PAN_L / 2, 0.0, TOP + RACK_TOP)  # frame origin: middle of the floor's underside


def _pan_shapes():
    L, W, H = PAN_L, PAN_W, PAN_H
    s = [parts.box((L - 2 * BEAD, W - 2 * BEAD, PAN_FLOOR), (0.0, 0.0, PAN_FLOOR / 2), ALU)]
    for sy in (1, -1):  # side walls under the bead, then the bead
        s.append(parts.box((L - 2 * BEAD, PAN_WALL, H - BEAD), (0.0, sy * (W / 2 - BEAD / 2), (H - BEAD) / 2), ALU))
        s.append(parts.box((L, BEAD, BEAD), (0.0, sy * (W / 2 - BEAD / 2), H - BEAD / 2), ALU))
    for sx in (1, -1):
        s.append(parts.box((PAN_WALL, W - 2 * BEAD, H - BEAD), (sx * (L / 2 - BEAD / 2), 0.0, (H - BEAD) / 2), ALU))
        s.append(parts.box((BEAD, W - 2 * BEAD, BEAD), (sx * (L / 2 - BEAD / 2), 0.0, H - BEAD / 2), ALU))
    return s


PAN = parts.Part("pan", pos=PAN_POS, mass=PAN_M, friction=(0.30, 0.25), contact_offset=0.0015, shapes=_pan_shapes())

# -- toast: two slices of sandwich bread, 95 x 90 x 15 mm, 28 g each (estimates), rigid ------------------------------
TOAST_SIZE = (0.095, 0.090, 0.015)
TOAST_M = 0.028
TOAST_DY = 0.050  # slice centres either side of the pan's middle across y


def _toast(name, dy):
    pos = (PAN_POS[0], dy, TOP + RACK_TOP + PAN_FLOOR + TOAST_SIZE[2] / 2)
    return parts.Part(name, pos=pos, mass=TOAST_M, friction=(0.50, 0.40), contact_offset=0.0015,
                      shapes=[parts.box(TOAST_SIZE, (0.0, 0.0, 0.0), TOAST),
                              parts.box((TOAST_SIZE[0] - 0.012, TOAST_SIZE[1] - 0.012, 0.0006),
                                        (0.0, 0.0, TOAST_SIZE[2] / 2 + 0.0003), CRUMB, collide=False)])


TOAST_A = _toast("toast_a", +TOAST_DY)
TOAST_B = _toast("toast_b", -TOAST_DY)

# -- the trivet: IKEA HEAT, cork, 191 mm across (maker), 10 mm thick and 70 g (estimates), loose ----------------------
TRIV_R, TRIV_H, TRIV_M = 0.0955, 0.010, 0.07
TRIVET = parts.Part("trivet", pos=(0.270, 0.0, TOP), mass=TRIV_M, friction=(0.60, 0.50), contact_offset=0.0015,
                    shapes=[parts.cylinder(TRIV_R, TRIV_H, (0.0, 0.0, TRIV_H / 2), CORK)])

OBJECTS = [OVEN, PAN, TOAST_A, TOAST_B, TRIVET]
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"trivet": TRIVET.pos}),
    lots.Lot("B", place={"trivet": (0.265, -0.050, TOP),
                         "pan": (PAN_POS[0] + 0.020, 0.0, PAN_POS[2]),
                         "toast_a": (TOAST_A.pos[0] + 0.020, TOAST_A.pos[1], TOAST_A.pos[2]),
                         "toast_b": (TOAST_B.pos[0] + 0.020, TOAST_B.pos[1], TOAST_B.pos[2])}),
)
DEMO_TOUCHES = {"oven"}  # the handle grasp touches the fixed oven's door handle

# -- scoring tolerances -----------------------------------------------------------------------------------------------
ON_HOLDER = D(OPEN_DEG - 2.0)  # the left door counts as on the holder within 2 degrees of fully open
OFF_HOLDER = D(OPEN_DEG - 3.0)  # and as having left it once more than 3 degrees off
CHECK_S = 1.0  # hands-free check of the held doors before any reach inside
DOOR_CLEAR = 0.020  # hand surface to either door or handle during that check
DOOR_STILL = D(1.0)  # door movement allowed during that check
HAND_CLEAR = 0.020  # hand surface to pan, toast and trivet at the end
HOLD_S = 1.0
STILL_V, DRIFT = 0.003, 0.001  # m/s and m over the final hold
FLAT, LEVEL, TIPPED = 3.0, 10.0, 30.0  # degrees
CENTRE = 0.040  # pan centre to trivet centre for full credit
DRAWN = 0.080  # pan moved toward the opening for the lowest tier
OUT_CLEAR = 0.010  # whole pan this far in front of the face
REST = 0.002  # pan underside to trivet top
DROP_V = 0.40  # m/s downward with no hand near: a drop
TOAST_LIFT = 0.015  # slice bottom above the pan floor: off the pan


def _R(q):
    return parts._quat_to_matrix(q)


def _tilt_deg(st):
    return math.degrees(math.acos(float(np.clip(_R(st.quat_xyzw)[2, 2], -1.0, 1.0))))


def _finite(st):
    try:
        return bool(np.isfinite(np.r_[st.pos, st.quat_xyzw, st.lin_vel, st.ang_vel]).all())
    except (TypeError, ValueError, AttributeError):
        return False


def pan_corners(st, z=0.0, inset=0.0):
    r = _R(st.quat_xyzw)
    hx, hy = PAN_L / 2 - inset, PAN_W / 2 - inset
    loc = np.array([[sx * hx, sy * hy, z] for sx in (-1, 1) for sy in (-1, 1)])
    return loc @ r.T + np.asarray(st.pos)


def pan_box_points(st):
    """The pan's outer box corners (floor underside to rim top)."""
    return np.concatenate([pan_corners(st, 0.0), pan_corners(st, PAN_H)])


def toast_on_pan(t, p):
    """The slice lies on the pan floor: centre inside the floor's inner rectangle, bottom within 15 mm of the floor,
    face up within 30 degrees."""
    r = _R(p.quat_xyzw)
    loc = r.T @ (np.asarray(t.pos) - np.asarray(p.pos))
    inner_x, inner_y = PAN_L / 2 - BEAD, PAN_W / 2 - BEAD
    bottom = loc[2] - TOAST_SIZE[2] / 2 - PAN_FLOOR
    return bool(abs(loc[0]) <= inner_x + EPS and abs(loc[1]) <= inner_y + EPS and bottom <= TOAST_LIFT + EPS
                and _tilt_deg(t) <= 30.0 + EPS)


def door_deg(objects):
    return math.degrees(float(objects[OVEN.name].joints["left_door"]))


def tier(objects):
    """The placement tier of a still state (hands, history and stillness are judged elsewhere)."""
    p, tv = objects[PAN.name], objects[TRIVET.name]
    if not (toast_on_pan(objects[TOAST_A.name], p) and toast_on_pan(objects[TOAST_B.name], p)):
        return 0.0
    tilt_p = _tilt_deg(p)
    if tilt_p > LEVEL + EPS:
        return 0.0
    under = pan_corners(p, 0.0)
    pts = pan_box_points(p)
    top = float(tv.pos[2]) + TRIV_H
    off = float(np.linalg.norm(np.asarray(p.pos[:2]) - np.asarray(tv.pos[:2])))  # pan frame origin: its middle
    triv_flat = _tilt_deg(tv) <= FLAT + EPS and abs(float(tv.pos[2]) - TOP) <= 0.002 + EPS
    on_trivet = triv_flat and off <= TRIV_R + EPS and float(under[:, 2].min()) - top <= REST + EPS \
        and float(under[:, 2].min()) - top >= -0.002 - EPS
    if on_trivet and tilt_p <= FLAT + EPS and off <= CENTRE + EPS:
        return 1.0
    if on_trivet:
        return 0.75
    if float(pts[:, 0].max()) <= FACE - OUT_CLEAR + EPS:
        return 0.5
    if float(p.pos[0]) <= lots.current().pos(PAN.name)[0] - DRAWN + EPS:  # the pan's middle drawn 80 mm forward
        return 0.25
    return 0.0


# -- the trial record (written only by the simulator's step and reset hooks) ------------------------------------------


def _fresh():
    return dict(known=False, invalid=False, zero=False, why="", t=None, checked=False, check_t=None, check_q=None,
                crossed=False, held=0.0, ref=None, value=None, clear=False, still=False, stamp=None, mode="free",
                inside=False, door_clear=None, hand_clear=None)


REC = _fresh()


def stamp(objects):
    out = []
    for name in (PAN.name, TOAST_A.name, TOAST_B.name, TRIVET.name):
        st = objects[name]
        out += [*np.round(np.asarray(st.pos, float), 9), *np.round(np.asarray(st.quat_xyzw, float), 9)]
    out.append(round(float(objects[OVEN.name].joints["left_door"]), 9))
    return tuple(float(v) for v in out)


def _zero(why):
    if not REC["zero"]:
        REC.update(zero=True, why=why)


def _box_dist(points, c, r, size):
    loc = (points - c) @ r
    q = np.abs(loc) - np.asarray(size) / 2
    return float((np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)).min())


def _link_boxes(objects, names):
    st = objects[OVEN.name]
    poses = {"base": (np.asarray(st.pos), _R(st.quat_xyzw))}
    poses.update({n: (np.asarray(p), _R(q)) for n, (p, q) in st.links.items()})
    return [(c, r, s) for n, c, r, s in OVEN.world_boxes(poses) if n in names]


def _part_boxes(spec, st):
    return [(c, r, s) for _, c, r, s in spec.world_boxes({"base": (np.asarray(st.pos), _R(st.quat_xyzw))})]


def _near(points, boxes, reach):
    """Smallest distance from points to any box, looking only within reach of each box (farther counts as reach)."""
    best = reach
    for c, r, s in boxes:
        lo = c - np.abs(r) @ (np.asarray(s) / 2) - reach
        hi = c + np.abs(r) @ (np.asarray(s) / 2) + reach
        keep = np.all((points >= lo) & (points <= hi), axis=1)
        if np.any(keep):
            best = min(best, _box_dist(points[keep], c, r, s))
    return best


def inside_opening(points):
    """Any point past the oven's front face within the cooking cavity's opening."""
    return bool(np.any((points[:, 0] > FACE + EPS) & (points[:, 0] < CAV_BACK) & (np.abs(points[:, 1]) < AP_Y)
                       & (points[:, 2] > TOP + AP_Z0) & (points[:, 2] < TOP + AP_Z1)))


def poses(objects):
    return {n: (np.asarray(objects[n].pos, float).copy(), _R(objects[n].quat_xyzw))
            for n in (PAN.name, TOAST_A.name, TOAST_B.name, TRIVET.name)}


def drifted(ref, now):
    if ref is None:
        return True
    for n, (p, r) in now.items():
        p0, r0 = ref[n]
        if np.linalg.norm(p - p0) > DRIFT + EPS:
            return True
        ang = math.degrees(math.acos(float(np.clip((np.trace(r0.T @ r) - 1.0) / 2.0, -1.0, 1.0))))
        if ang > 1.0 + EPS:
            return True
    return False


def slow(objects):
    return all(np.linalg.norm(np.asarray(objects[n].lin_vel)) <= STILL_V + EPS
               for n in (PAN.name, TOAST_A.name, TOAST_B.name, TRIVET.name))


def observe(objects, hands, t):
    """Update the trial record from one control step's states and hand surface points (None: unknown)."""
    try:
        ok = all(_finite(objects[n]) for n in (PAN.name, TOAST_A.name, TOAST_B.name, TRIVET.name))
        q = float(objects[OVEN.name].joints["left_door"])
        qr = float(objects[OVEN.name].joints["right_door"])
        ok = ok and math.isfinite(q) and math.isfinite(qr)
    except (KeyError, TypeError, ValueError, AttributeError):
        ok = False
    dt = None if REC["t"] is None else float(t) - REC["t"]
    if not ok or hands is None or (dt is not None and not 0.0 < dt <= 0.2):
        REC.update(known=False, invalid=True, held=0.0, t=float(t), stamp=None)
        return
    p = objects[PAN.name]
    pan_pts = pan_box_points(p)
    inside = inside_opening(hands)
    REC["inside"] = inside
    on_holder = q >= ON_HOLDER - EPS
    # 1. Reaching in: no hand past the front face before the doors have been seen held hands-free.
    if inside and not REC["checked"]:
        _zero("reached into the oven before the doors were seen held by the holder, hands off, for 1 s")
    # 2. A door leaving the holder while a hand is inside.
    if q < OFF_HOLDER - EPS and inside:
        _zero("the left door left the holder while a hand was inside the oven")
    if inside:
        REC["crossed"] = True
    # 3. The hands-free check of the held doors (kept until the left door leaves the holder).
    door_boxes = _link_boxes(objects, ("left_door", "right_door"))
    dclear = _near(hands, door_boxes, 0.10)
    REC["door_clear"] = dclear
    if on_holder and dclear >= DOOR_CLEAR - EPS:
        if REC["check_t"] is None or REC["check_q"] is None or abs(q - REC["check_q"]) > DOOR_STILL + EPS:
            REC["check_t"], REC["check_q"] = float(t), q
        elif float(t) - REC["check_t"] >= CHECK_S - EPS:
            REC["checked"] = True
    else:
        REC["check_t"], REC["check_q"] = None, None
    if q < OFF_HOLDER - EPS:
        REC["checked"] = False
    # 4. Drops and spills.
    for name in (TOAST_A.name, TOAST_B.name):
        if not toast_on_pan(objects[name], p):
            _zero("a slice of toast left the pan")
    if _tilt_deg(p) > TIPPED + EPS:
        _zero("the pan tipped more than 30 degrees")
    hclear_pan = _near(hands, _part_boxes(PAN, p), 0.10)
    if float(p.lin_vel[2]) < -DROP_V - EPS and hclear_pan >= 0.030 - EPS:
        _zero("the pan fell")
    if float(pan_pts[:, 2].min()) < TOP - 0.005:
        _zero("the pan left the counter")
    # 5. The end-state hold: hands clear of the pan, toast and trivet, everything still, tier unchanged.
    value = tier(objects)
    hclear = min(hclear_pan, _near(hands, _part_boxes(TOAST_A, objects[TOAST_A.name]) + _part_boxes(TOAST_B, objects[TOAST_B.name])
                                   + _part_boxes(TRIVET, objects[TRIVET.name]), 0.10))
    REC["hand_clear"] = hclear
    clear = hclear >= HAND_CLEAR - EPS
    now = poses(objects)
    calm = slow(objects)
    if clear and calm and value == REC["value"] and dt is not None and not drifted(REC["ref"], now):
        REC["held"] += dt
    else:
        REC["held"], REC["ref"] = 0.0, now
    REC.update(known=True, t=float(t), value=value, stamp=stamp(objects), clear=clear, still=calm)


def score(objects):
    """0 after any whole-trial fault; otherwise the placement tier of a released, still end state."""
    try:
        if not REC["known"] or REC["invalid"] or REC["zero"] or REC["stamp"] != stamp(objects):
            return 0.0
        if not (REC["clear"] and REC["still"] and REC["held"] >= HOLD_S - EPS):
            return 0.0
        return float(tier(objects))
    except (KeyError, TypeError, ValueError, AttributeError):
        return 0.0


def success(objects):
    return score(objects) >= 1.0 - EPS


# -- simulator hooks: hand samples, springs and magnet, the record -----------------------------------------------------


@lru_cache(maxsize=1)
def hand_samples():
    """Points on every collision face of the gripper, both fingers and the wrist link (link frames), <= 3 mm apart."""
    from yam_arena import analysis

    out = {}
    for name in ("gripper", "tip_left", "tip_right", "link5"):
        cloud = []
        for h in analysis.link_hulls().get(name, []):
            cloud.extend(h.vertices)
            for pl in np.unique(h.planes.round(7), axis=0):
                face = h.vertices[np.abs(h.vertices @ pl[:3] - pl[3]) < 2e-6]
                if len(face) < 3:
                    continue
                c = face.mean(0)
                u = face[0] - c
                if np.linalg.norm(u) < 1e-9:
                    continue
                u /= np.linalg.norm(u)
                v = np.cross(pl[:3], u)
                face = face[np.argsort(np.arctan2((face - c) @ v, (face - c) @ u))]
                for a, b in zip(face, np.roll(face, -1, axis=0)):
                    n = max(1, math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / 0.003))
                    for i in range(n + 1):
                        for j in range(n - i + 1):
                            cloud.append(c + (a - c) * (i / n) + (b - c) * (j / n))
        if not cloud:
            raise ValueError("missing hand geometry")
        pts = np.asarray(cloud)
        _, idx = np.unique(np.round(pts / 0.0015).astype(np.int64), axis=0, return_index=True)
        out[name] = pts[np.sort(idx)]
    return out


def hand_points(sim):
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    clouds = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        bodies = parts._np(asset.data.body_link_pose_w)[0]
        if bodies.ndim != 2 or bodies.shape[1] != 7 or not np.isfinite(bodies).all():
            raise ValueError("unknown robot body poses")
        for name, pts in hand_samples().items():
            pose = bodies[asset.body_names.index(name)]
            clouds.append(pts @ _R(pose[3:]).T + pose[:3] - origin)
    return np.concatenate(clouds)


def spring_torque(q):
    """Closing torque of the two door springs together (N m) at left-door angle q (rad)."""
    return SPRING_K * (q - SPRING_REST)


def magnet_torque(q):
    """Opening torque of the holder's magnets on the left door (N m): strongest at contact, fading with the gap."""
    gap = MAG_R * max(0.0, D(OPEN_DEG) - q)
    return MAG_R * MAG_PULL / (1.0 + gap / MAG_FADE) ** 2


def _door_drive(sim, objects):
    """The doors' closing springs; or, while the magnets out-pull the springs, the magnets' net pull toward the holder.
    The drive is a spring aimed 1 degree past the stop whose stiffness is reset each step so that its torque equals
    the magnets' pull less the springs' at the current angle."""
    q = float(objects[OVEN.name].joints["left_door"])
    net = magnet_torque(q) - spring_torque(q)
    mode = REC.get("mode", "free")
    if net <= 0.0 or (mode == "free" and q < MAG_CATCH - EPS):
        mode = "free"
    else:
        mode = "magnet"
    if mode == "magnet":
        k = min(MAG_K_MAX, net / max(MAG_AIM - q, D(0.2)))
        sim.set_stiffness(OVEN.name, "left_door", k)
        if REC.get("mode_set") != "magnet":
            sim.set_target(OVEN.name, "left_door", MAG_AIM)
    elif REC.get("mode_set") != "free":
        sim.set_stiffness(OVEN.name, "left_door", SPRING_K)
        sim.set_target(OVEN.name, "left_door", SPRING_REST)
    REC["mode_set"] = mode
    REC["mode"] = mode


@parts.on_reset
def _reset(sim):
    REC.clear()
    REC.update(_fresh())
    REC["mode_set"] = None
    sim.set_stiffness(OVEN.name, "left_door", SPRING_K)
    sim.set_target(OVEN.name, "left_door", SPRING_REST)
    REC["mode_set"] = "free"


_lots_apply = lots._apply


def _apply_lot(emb):
    moved = _lots_apply(emb)
    mode_set = REC.get("mode_set")
    REC.clear()
    REC.update(_fresh())
    REC["mode_set"] = mode_set
    return moved


lots._apply = _apply_lot


@parts.on_step
def _step(sim):
    objects = sim.states()
    try:
        hands = hand_points(sim)
    except (ValueError, AttributeError, KeyError, IndexError):
        hands = None
    try:
        _door_drive(sim, objects)
    except (KeyError, TypeError, ValueError, AttributeError) as e:
        REC.update(known=False, invalid=True, why="door drive: " + str(e))
    observe(objects, hands, float(sim.t))
    if os.environ.get("TRACE_OUT"):
        p = objects[PAN.name]
        row = dict(t=round(float(sim.t), 3), lot=lots.current().name, door=round(door_deg(objects), 2),
                   right=round(math.degrees(float(objects[OVEN.name].joints["right_door"])), 2), mode=REC.get("mode"),
                   checked=REC["checked"], crossed=REC["crossed"], inside=REC["inside"], zero=REC["zero"], why=REC["why"],
                   door_clear=REC["door_clear"], hand_clear=REC["hand_clear"], value=REC["value"], held=round(REC["held"], 2),
                   score=score(objects), pan=[round(float(v), 4) for v in p.pos], pan_tilt=round(_tilt_deg(p), 2),
                   toast=[[round(float(v), 4) for v in objects[n].pos] for n in (TOAST_A.name, TOAST_B.name)],
                   trivet=[round(float(v), 4) for v in objects[TRIVET.name].pos])
        with open(pathlib.Path(os.environ["TRACE_OUT"]).with_name("mechanism.jsonl"), "a") as f:
            f.write(json.dumps(row) + "\n")


# -- the scripted demo: open loop from the published layout --------------------------------------------------------------
GRIP_TILT, PAN_TILT = 45.0, 60.0
HANDLE_Z = 0.220
SQUEEZE_BAR, SQUEEZE_RIM, PARTLY, RIM_OPEN = -0.004, 0.0, -0.020, -0.014
PULL_TO = 109.0  # degrees commanded along the arc, past the stop, so the arm's sag still reaches the magnet


def _d(a, side=1):
    return np.array([-math.sin(a), -side * math.cos(a)])


def _nout(a, side=1):
    return np.array([-math.cos(a), side * math.sin(a)])


def _bar(a):
    return np.array([HX, HY]) + SH * _d(a) + SO * _nout(a)


def _lean(adeg):
    """Heading offset that keeps the left wrist in reach as the door swings: 0 to 45 degrees over 15 to 70 degrees."""
    return D(min(45.0, max(0.0, (adeg - 15.0) * 45.0 / 55.0)))


def _W(dt, xy, z, f, label, h=0.0, tl=GRIP_TILT):
    return tilt.Waypoint(float(dt), (float(xy[0]), float(xy[1]), TOP + float(z)), float(f), label,
                         yaw=float(h), tilt=float(tl), heading=float(h))


def demo():
    lot = lots.current()
    pan_x = lot.pos(PAN.name)[0]
    triv = lot.pos(TRIVET.name)
    left, right = [], []
    # Left hand: grip the left door's handle, swing both linked doors fully open onto the holder, let go, step back.
    h0 = _lean(0.0)
    b0 = _bar(0.0)
    back = np.array([math.cos(h0), math.sin(h0)])
    left += [_W(1.5, b0 - 0.07 * back, HANDLE_Z + 0.03, PARTLY, "to handle", h0),
             _W(1.0, b0 - 0.07 * back, HANDLE_Z, PARTLY, "line up", h0),
             _W(1.0, b0, HANDLE_Z, PARTLY, "onto handle", h0),
             _W(0.6, b0, HANDLE_Z, SQUEEZE_BAR, "grip handle", h0)]
    for adeg in np.arange(7.5, PULL_TO + 0.1, 7.5).tolist() + [PULL_TO]:
        a = D(adeg)
        left.append(_W(0.45, _bar(a), HANDLE_Z, SQUEEZE_BAR, "open doors", -a + _lean(adeg)))
    a1 = D(OPEN_DEG)
    h1 = -a1 + _lean(OPEN_DEG)
    back1 = np.array([math.cos(h1), math.sin(h1)])
    left += [_W(0.6, _bar(a1), HANDLE_Z, -0.026, "let go", h1),
             _W(1.0, _bar(a1) - 0.035 * back1, HANDLE_Z + 0.01, -0.026, "off handle", h1),
             _W(1.5, (0.13, 0.24), 0.20, PARTLY, "round the door", 0.0)]
    t_open = sum(w.duration for w in left)
    # Right hand: stand clear of the right door's swing.
    right += [_W(2.0, (0.16, -0.40), 0.28, PARTLY, "stand clear", -0.3),
              _W(t_open - 3.5, (0.16, -0.40), 0.28, PARTLY, "wait", -0.3),
              _W(1.5, (0.13, -0.24), 0.20, PARTLY, "round the door", 0.0)]
    # Both hands, hands-free check first: wait 1.5 s with both hands away from the doors.
    for plan, side in ((left, 1), (right, -1)):
        plan.append(_W(1.5, (0.13, side * 0.17), 0.20, PARTLY, "doors held, hands off", 0.0))
    # Both hands: pinch the pan's side rims 40 mm behind its near wall, slide it out, carry it onto the trivet.
    gx = pan_x - PAN_L / 2 + 0.040
    goff = gx - pan_x
    gz = RACK_TOP + PAN_H + 0.003
    rise = 0.012
    tz = TRIV_H + PAN_H + 0.003
    slide_to = FACE + 0.020 + goff  # grip x with the pan's middle 15 mm behind the rack's front edge
    for plan, side in ((left, 1), (right, -1)):
        y = side * (PAN_W / 2 - BEAD / 2)
        ty = triv[1] + y
        plan += [_W(2.0, (gx - 0.08, y), gz + 0.02, PARTLY, "to the pan", 0.0, PAN_TILT),
                 _W(1.2, (gx, y), gz + 0.02, RIM_OPEN, "over the rim", 0.0, PAN_TILT),
                 _W(1.0, (gx, y), gz, RIM_OPEN, "down to the rim", 0.0, PAN_TILT),
                 _W(0.6, (gx, y), gz, SQUEEZE_RIM, "pinch the rims", 0.0, PAN_TILT),
                 _W(2.5, (slide_to, y), gz + 0.002, SQUEEZE_RIM, "slide out", 0.0, PAN_TILT),
                 _W(2.0, (triv[0] + goff, ty), gz + rise, SQUEEZE_RIM, "lift out", 0.0, PAN_TILT),
                 _W(1.5, (triv[0] + goff, ty), tz + 0.005, SQUEEZE_RIM, "lower onto the trivet", 0.0, PAN_TILT),
                 _W(0.6, (triv[0] + goff, ty), tz + 0.005, RIM_OPEN, "let go", 0.0, PAN_TILT),
                 _W(0.8, (triv[0] + goff, ty), tz + 0.035, RIM_OPEN, "fingers up", 0.0, PAN_TILT),
                 _W(1.0, (triv[0] + goff + 0.03, ty + side * 0.02), tz + 0.09, PARTLY, "lift away", 0.0, PAN_TILT),
                 _W(1.5, (0.12, side * 0.21), 0.22, PARTLY, "hands away", 0.0),
                 _W(2.0, (0.12, side * 0.21), 0.22, PARTLY, "still", 0.0)]
    return {"left": left, "right": right}
