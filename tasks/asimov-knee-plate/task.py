"""Turn a loose Asimov 1 leg over on the table until the knee's bearing faces up, then seat the knee side plate flush in its pocket.

Menlo's Asimov 1, Left Leg Step 8, instruction 4, in short: turn the leg toward the right until the bearing on the thigh assembly's actuator
faces upward toward the ceiling, then fit part 600_08A onto the groove on part 600_10C. The leg (the thigh
assembly slotted into the knee housing 600_10C, cut off the body) lies loose on the table on its side, the plate's pocket facing away from the
robot. Nothing holds it. At 3.03 kg it is twice the arm's 1.5 kg payload, so it is turned on the table, not lifted: tipped toward the robot
over its lower edge until it lies on its back, the face up. Its knee joint is free, with the knee actuator's own friction (Menlo's actuator
table, EC-A4315-P2-36: 0.70 N m static, 0.02 N m moving, 0.034 kg m^2 reflected inertia, 0 to 1.50 rad). Then the 5 mm aluminium plate
600_08A goes into its 5 mm pocket (0.3 mm a side) while its pad enters the 35.0 mm bore (0.1 mm a side) and its boss the thigh's 12.0 mm
bearing (0.075 mm a side). Scored by effect: the leg on its back, how far the plate's top stands above the face all round, and whether its
holes are over their inserts.

Debug:  python demokit.py task.py OUT --no-record --stills 20 --mode full
Final:  python demokit.py task.py OUT --mode full
Gates:  KNEE_GATE=rest python demokit.py task.py OUT --no-record --mode gate --max-steps 200            (the leg on its side, left alone)
        KNEE_GATE=press KNEE_PUSH=10 KNEE_PUSH2=10 python demokit.py task.py OUT --no-record --mode gate --max-steps 200
        KNEE_GATE=drop KNEE_DROP=0.2,0,7,0,0 python demokit.py task.py OUT --no-record --mode gate --max-steps 200
"""

from __future__ import annotations

import dataclasses
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
SCENE = "station"
INSTRUCTION = "Turn the leg onto its back, seat the knee side plate flush in its pocket, then start and tighten M3 bolts in the two holes on its long axis nearest its narrow end, and move both hands clear."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own 1 s hold
HOLD_STEPS = 20
TOP = d.TABLE_TOP
TRACE = os.environ.get("KNEE_TRACE", "1") != "0"
ITERS = int(os.environ.get("KNEE_ITERS", "16"))  # the leg's and the plate's solver position iterations
GATE = os.environ.get("KNEE_GATE", "")
START = os.environ.get("KNEE_START", "")  # debug only: "seated" starts with the leg on its back and the plate seated

Z = np.load(HERE / "knee_plate.npz")  # meshes and colliders in the plate frame: origin on the boss axis at the plate's outer face, z out of the housing, x toward the narrow end

LOOKS = {
    "alu": m.Look(m.rgb(0.80, 0.81, 0.83), metallic=0.2, roughness=0.5),  # 7075, machined, bead-blasted
    "pa12": m.Look(m.rgb(0.17, 0.17, 0.18), roughness=0.85),  # MJF PA12, as printed: dark grey, matte
    "thigh": m.Look(m.rgb(0.36, 0.37, 0.39), metallic=0.5, roughness=0.45),
    "steel": m.Look(m.rgb(0.72, 0.72, 0.74), metallic=0.95, roughness=0.25),
    "pom": m.Look(m.rgb(0.92, 0.92, 0.89), roughness=0.5),  # the kitting post, acetal
    "brass": m.Look(m.rgb(0.78, 0.62, 0.30), metallic=0.9, roughness=0.35),  # the heat-set inserts
    "bolt": m.Look(m.rgb(0.80, 0.80, 0.82), metallic=0.4, roughness=0.3),  # the M3 flat bolts, bright steel
    "socket": m.Look(m.rgb(0.02, 0.02, 0.02), roughness=0.9),  # the bolts' hex sockets
    "tray": m.Look(m.rgb(0.20, 0.38, 0.62), roughness=0.7),  # the bolt holder, printed PLA
    "drv_red": m.Look(m.rgb(0.75, 0.08, 0.06), roughness=0.5),  # the screwdriver's handle
    "drv_black": m.Look(m.rgb(0.06, 0.06, 0.07), roughness=0.8),  # its band
}


def _rz(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _quat_xyzw(R: np.ndarray) -> tuple:
    w, x, y, z = gk.quat_wxyz(R)
    return (float(x), float(y), float(z), float(w))


# -- the leg (plate frame, m) ----------------------------------------------------------------------------------------------------------------------
# The thigh assembly (URDF left_hip_yaw_link: 1.949 kg, its centre of mass and inertia) and the housing 600_10C (1,069.5 cm3 of PA12 at
# 1.01 g/cm3: 1.080 kg; centre and inertia from its mesh), on the knee joint. The knee's axis is the boss's (URDF: left_knee_joint 0, 0,
# -0.19564 from left_hip_yaw_joint, along its y; here 49.25 mm under the face), so the bearing stays on the pocket's axis at any knee angle.
# The URDF's flexion (0 to 1.50 rad) turns the housing about the plate frame's -z: here the joint runs 0 to -1.50 rad.
THIGH_KG, HOUSING_KG = 1.949, 1.080
THIGH_COM, THIGH_I = (-0.07536, 0.00108, -0.05032), (0.00145, 0.00364, 0.00380)  # URDF, in the plate frame's axes
HOUSING_COM, HOUSING_I = (0.10431, -0.00021, -0.05043), (0.00176, 0.00866, 0.00868)
KNEE_AT = (0.0, 0.0, -0.04925)
KNEE_LO = -86.0  # degrees: 1.50 rad of flexion
# the knee actuator, unpowered (Menlo, Asimov 1 joint design and actuation, actuator 4, left_knee_joint, EC-A4315-P2-36)
STATIC = float(os.environ.get("KNEE_STATIC", "0.70"))  # N m: the torque that starts it turning
DYNAMIC = float(os.environ.get("KNEE_DYNAMIC", "0.02"))  # N m: the lower drag once it turns
ARMATURE = 0.0339552  # kg m^2: the rotor's inertia reflected through the gearbox
PIN_K, PIN_D = 50.0, 0.3  # the drive that stands in for dry friction (kit README: a joint's own friction grows with its load): N m/rad, N m s/rad
MU = float(os.environ.get("KNEE_MU", "0.3"))  # PA12 and aluminium on the table, the post and each other

# Its rest facets on the table, from the colliders below: outward normal n and offset d (n.p = d on the facet). On its
# side: the thigh's and the housing's +y side down, the face toward +x. On its back: face up, 4.3 degrees off level (no pad, nothing added).
# Between them it tips 85.9 degrees over the edge where the two meet, toward the robot; its centre of mass rises 62 -> 79 mm, then falls to 53.
N_SIDE, D_SIDE = np.array([0.04094, 0.99916, -0.00233]), 0.06269
N_BACK, D_BACK = np.array([0.02856, 0.06850, -0.99724]), 0.10309
EDGE_C, EDGE_U = np.array([-0.00749, 0.06282, -0.09928]), np.array([-0.99883, 0.04086, -0.02580])


def _lying(n_down, x_world) -> np.ndarray:
    """The rotation (plate frame into the world) that puts the plate frame's `n_down` straight down, with its x as near `x_world` as that allows."""
    e3 = -np.asarray(n_down, float) / np.linalg.norm(n_down)
    e1 = np.array([1.0, 0.0, 0.0]) - e3[0] * e3
    e1 /= np.linalg.norm(e1)
    w1 = np.asarray(x_world, float) - np.asarray(x_world, float)[2] * np.array([0.0, 0.0, 1.0])
    w1 /= np.linalg.norm(w1)
    W = np.column_stack([w1, np.cross([0.0, 0.0, 1.0], w1), [0.0, 0.0, 1.0]])
    E = np.column_stack([e1, np.cross(e3, e1), e3])
    return W @ E.T


def _axis_rot(u, deg: float) -> np.ndarray:
    return gk.rot(tuple(u), deg)


END_XY = np.array([0.31, -0.06])  # the pocket's axis once the leg is on its back
R_BACK = _lying(N_BACK, (0.0, -1.0, 0.0))  # on its back: the calf toward -y, the face up
O_BACK = np.array([END_XY[0], END_XY[1], TOP + D_BACK])
EDGE_W = R_BACK @ EDGE_U  # the edge in the world, the leg on its back
EDGE_P = O_BACK + R_BACK @ EDGE_C
TIP_DEG = 85.87  # the angle between the two facets' planes
R_SIDE = _axis_rot(EDGE_W, TIP_DEG) @ R_BACK  # the same leg on its side, tipped back over the edge: the face toward +x
O_SIDE = EDGE_P + _axis_rot(EDGE_W, TIP_DEG) @ (O_BACK - EDGE_P)
LEVEL_TOL = 10.0  # degrees: the face within this of level (on its back it rests 4.3 off; on its side, 90)
REST_TOL = 0.012  # m: the face's origin within this of its height on its back: the leg lies on the table, not held up
SLIDE_TOL = 0.015  # m: the leg moves less than this from where it lay when the plate first reached the pocket's mouth
ROCK_TOL = 3.0  # degrees: and its face turns less than this from how it lay then (pressed at the plate's edge it rocks 2 onto its other three points)
LEG0 = os.environ.get("KNEE_LEG0", "back" if (GATE in ("seated", "drop", "press", "back") or START == "seated") else "side")

POST = np.array([0.20, -0.17])
POST_H, POST_R, SOCKET_R, SOCKET_D = 0.030, 0.025, 0.018, 0.012  # an acetal kitting post, 50 mm across, 30 mm tall, with a 36 mm socket 12 mm deep
PLATE_KG = 0.1198  # 42.62 cm3 (the STEP) of 7075 at 2.81 g/cm3
PLATE0 = np.array([POST[0], POST[1], TOP + POST_H + 0.005])  # boss down in the socket, the sheet's underside on the post's rim
R_POST = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # the plate on the post: its x toward -y


def _mesh(key: str) -> m.Mesh:
    return m.Mesh(np.asarray(Z[f"{key}_v"], float), np.asarray(Z[f"{key}_f"], int))


def _hulls(prefix: str, which=None) -> list:
    n = int(Z[f"n_{prefix}"])
    return [m.col_hull(f"c_{prefix}{i}", _mesh(f"{prefix}{i}")) for i in (range(n) if which is None else which)]


def _hull2d(p: np.ndarray) -> np.ndarray:
    """The convex hull of 2D points, counter-clockwise (monotone chain)."""
    p = np.unique(np.round(p, 6), axis=0)
    if len(p) < 3:
        return p

    def half(pts):
        out = []
        for q in pts:
            while len(out) >= 2 and (out[-1][0] - out[-2][0]) * (q[1] - out[-2][1]) - (out[-1][1] - out[-2][1]) * (q[0] - out[-2][0]) <= 0:
                out.pop()
            out.append(q)
        return out

    lo, hi = half(p), half(p[::-1])
    return np.array(lo[:-1] + hi[:-1])


def _slab(name: str, pts: np.ndarray, x0: float, x1: float, keep=None):
    """A convex prism round the leg's points between plate x0 and x1 (their outline across the leg, at most 24 corners), as a collider."""
    sel = (pts[:, 0] >= x0) & (pts[:, 0] <= x1) & (np.ones(len(pts), bool) if keep is None else keep)
    if sel.sum() < 4:
        return None
    h = _hull2d(pts[sel][:, 1:3])
    if len(h) > 24:
        h = h[np.unique(np.linspace(0, len(h) - 1, 24).round().astype(int))]
    pr = m.prism([tuple(c) for c in h], x0, x1)  # built along z from an (a, b) outline: here (y, z) along x
    return m.col_hull(name, m.Mesh(np.c_[pr.v[:, 2], pr.v[:, 0], pr.v[:, 1]], pr.f))


def _housing_colliders() -> list:
    """The housing's outside, for the table and the hands: coarse prisms, kept clear of the pocket's fine colliders (plate x
    -49 to 110 mm, y within 49 mm, z above -16 mm), where the plate goes."""
    hv = np.asarray(Z["housing_v"], float)
    below = hv[:, 2] < -0.0175
    out = [_slab(f"c_hb{i}", hv, a, b, below) for i, (a, b) in enumerate([(-0.062, -0.035), (-0.035, -0.005), (-0.005, 0.025), (0.025, 0.055), (0.055, 0.085), (0.085, 0.112)])]
    for i, (a, b) in enumerate([(-0.050, -0.010), (-0.010, 0.030), (0.030, 0.070), (0.070, 0.112)]):
        out.append(_slab(f"c_hn{i}", hv, a, b, (hv[:, 1] < -0.0505) & ~below))  # the face's rim either side of the pocket
        out.append(_slab(f"c_hf{i}", hv, a, b, (hv[:, 1] > 0.0505) & ~below))
    out += [_slab(f"c_hc{i}", hv, a, b) for i, (a, b) in enumerate([(0.112, 0.150), (0.150, 0.190), (0.190, 0.230), (0.230, 0.266)])]  # the calf
    return [c for c in out if c]


def _thigh_colliders() -> list:
    tv = np.asarray(Z["thigh_v"], float)
    out = [_slab(f"c_th{i}", tv, a, b) for i, (a, b) in enumerate([(-0.273, -0.240), (-0.240, -0.200), (-0.200, -0.160), (-0.160, -0.120), (-0.120, -0.085), (-0.085, -0.050)])]
    return [c for c in out if c]


def _bearing() -> m.Mesh:
    """The flanged bearing 600_11X, moved onto the boss's axis with its 5 mm width from 11 to 16 mm under the face (its STEP mesh lies along x)."""
    b = _mesh("bearing")
    c = (b.v.min(0) + b.v.max(0)) / 2
    return m.Mesh((b.v - c) @ np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]]).T + [0.0, 0.0, -0.0135], b.f)


BEARING_HULLS = range(int(Z["n_lc"]) - 24, int(Z["n_lc"]))  # of the leg's colliders (the last 24): the bearing's 12.0 mm bore, 11 to 16 mm under the face (the rest are the pocket's)


def _leg():
    if LEG0 == "back":
        R, o = R_BACK, O_BACK + [0.0, 0.0, 0.0005]
    else:
        R, o = R_SIDE, O_SIDE + [0.0, 0.0, 0.0005]
    knee = m.hinge("knee", "Z", KNEE_AT, KNEE_LO, 0.0, [("housing", _mesh("housing"), "pa12")] + _insert_parts(),
                   _hulls("lc", [i for i in range(int(Z["n_lc"])) if i not in BEARING_HULLS]) + _housing_colliders(),
                   mass=HOUSING_KG, spring=PIN_K, damping=PIN_D, effort=STATIC, friction=0.0, armature=ARMATURE, gravity=True, com=HOUSING_COM, inertia=HOUSING_I)
    part = m.mechanism("leg", tuple(o), [knee] + _bolt_links(), LOOKS, frame_parts=[("thigh", _mesh("thigh"), "thigh"), ("bearing", _bearing(), "steel")],
                       frame_colliders=_thigh_colliders() + _hulls("lc", BEARING_HULLS), friction=(MU, MU), solver_iters=ITERS, free_mass=THIGH_KG,
                       start_quat=_quat_xyzw(R), frame_inertia=THIGH_I, start=[(f"seat{i}", PARK) for i in BOLT_HOLES])
    if part is None:  # check_score's stand-in kit
        return None
    build = part.build

    def with_com() -> str:  # mechkit gives a mechanism's frame no centre of mass of its own: the thigh's, from the URDF
        text = build()
        line = f"float physics:mass = {THIGH_KG}"
        assert text.count(line) == 1
        return text.replace(line, line + f"\n        point3f physics:centerOfMass = ({THIGH_COM[0]:.5f}, {THIGH_COM[1]:.5f}, {THIGH_COM[2]:.5f})")

    return dataclasses.replace(part, build=with_com)


def _post():
    prof = [(0.0, 0.0), (POST_R, 0.0), (POST_R, POST_H), (SOCKET_R, POST_H), (SOCKET_R, POST_H - SOCKET_D), (0.0, POST_H - SOCKET_D)]
    cols = [m.col_cylinder("c_floor", SOCKET_R + 0.001, POST_H - SOCKET_D, (0.0, 0.0, (POST_H - SOCKET_D) / 2))]
    n = 16
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        ri = SOCKET_R / math.cos(math.pi / n)
        wedge = [(ri * math.cos(a0), ri * math.sin(a0)), (POST_R * math.cos(a0), POST_R * math.sin(a0)), (POST_R * math.cos(a1), POST_R * math.sin(a1)), (ri * math.cos(a1), ri * math.sin(a1))]
        cols.append(m.col_hull(f"c_rim{k}", m.prism(wedge, 0.0, POST_H)))
    return m.fixed("post", (POST[0], POST[1], TOP), [("post", m.lathe(prof, 48), "pom")], cols, LOOKS, friction=(MU, MU))


# -- the fasteners (Menlo, Left Leg Step 8, instruction 5: "Take 9 of M3x10 Flat bolts and lightly tighten into all the holes on part 600_08A
# to align it with part 600_10C", then each out, threadlocker, and back in, in a star order) ----------------------------------------------------
# The bolts: DIN 7991 M3x10 (Menlo's part id m3-x-x-10mm-din-7991; a flat bolt is measured overall, head included): 3.0 mm thread, 0.5 mm pitch,
# head 6.0 mm across and 1.7 mm tall at 90 degrees, 2 mm hex socket. Seated, the head fills the plate's countersink (6.72 mm across, 90 degrees,
# from the STEP) and 5 mm of thread reaches into the insert. The inserts: brass M3 heat-set inserts in the housing's nine insert bores (the STEP:
# 4.4 mm across, 4.5 mm deep under a 0.5 mm lead-in, from 5.5 to 10 mm under the face), so modelled 4.4 mm across and 4.5 mm long with the M3
# thread's 2.5 mm minor bore. The bolts stand head up in a printed holder; a screwdriver stands in its tower (below). The right hand takes the screwdriver and two bolts.
BOLT_L, BOLT_D, BOLT_HEAD_D, BOLT_HEAD_K, BOLT_HEX = 0.010, 0.003, 0.006, 0.0017, 0.002
INSERT_D, INSERT_ID, INSERT_TOP, INSERT_BOT = 0.0044, 0.0025, -0.0055, -0.0100
HOLDER = np.array([0.13, -0.25])  # the printed holder on the table, right of the post, clear of the leg and the plate's path
HOLDER_W, HOLDER_H, HOLDER_PITCH = 0.040, 0.008, 0.012  # its deck: 40 x 40 x 8 mm, nine bolts standing in it on a 12 mm grid
SIT = 0.016  # a bolt standing in the deck, tip down in its 3.2 mm hole: its head's top is this high, 8 mm above the deck
TOWER = (-0.037, 0.0)  # its tower, holder frame: beside the deck on the robot's side, 34 mm square, 44 mm tall, a 14 mm square socket through it for the driver
TOWER_W, TOWER_H, TOWER_HOLE = 0.034, 0.044, 0.014

# The driver: a Bahco 808050S stubby ratchet screwdriver (1/4 in magnetic bit holder, 95 mm overall) with a Wera 840/1 Z Hex-Plus 2.0 x 25 mm bit
# (05056305001), 13 mm of it standing out of the holder. It stands in the tower, bit down, its handle on the tower's top. Its frame: origin at the
# bit's tip, +z along its axis toward the handle's end. Modelled: a lathed handle (red, a black ratchet ring), the steel nose and the bit; the
# handle's profile and the 90 g are estimates from the maker's photographs and its size.
DRV_KG = 0.090
BIT_TIP_L, BIT_SHANK_L, NOSE_R, NOSE_L, HANDLE_R, HANDLE_L = 0.008, 0.005, 0.006, 0.027, 0.018, 0.068
NOSE_Z = BIT_TIP_L + BIT_SHANK_L  # the nose starts here; the handle at HANDLE_Z
HANDLE_Z = NOSE_Z + NOSE_L
DRV_L = HANDLE_Z + HANDLE_L  # 108 mm with the bit
DRV0_Z = TOP + 0.0005 + TOWER_H - HANDLE_Z + 0.0003  # the bit's tip in the stand: the handle's end rests on the stand's top
RESQUARE_DEG = 6.0  # degrees: the driver squared again, off the head, when it leans more than this
TILT_GAIN = float(os.environ.get("KNEE_TILT_GAIN", "1.0"))
GRIP_Z = HANDLE_Z + 0.026  # the hand holds the handle low on its straight part, over its taper (on the taper the push slid it up through the fingers): over the leg the hand reaches only about 19 cm above the table

# The thread: each insert's bolt is a slider along the hole's axis (the seat, on a stiff spring through which the bit's push is read) and a
# hinge about it (the bolt's turn), both children of the housing. A bolt stands in the holder until the bit's magnet picks it up; set on a
# hole's mouth, upright, with the plate seated over that hole's insert, it drops through the 3.2 mm hole onto the insert's first thread:
# its head's top 4.5 mm above the plate. From there each turn of the bit, while seated in the socket, within 15 degrees of the axis and
# pushed in with at least 5 N, drives it down 0.5 mm (the M3 pitch); 9 turns bring the head down flush. The driver is a ratchet: only the
# tightening turn drives.
PITCH = 0.0005
START_S = INSERT_TOP + BOLT_L  # the bolt's head top above the plate when it rests on the insert's first thread: 4.5 mm
HOME_TURNS = START_S / PITCH  # 9 turns
BOLT_HOLES = (8, 7)  # the two holes on the plate's long axis nearest its narrow end: the bolts go here, the far one first
PARK = -0.012  # a bolt not yet in its hole: its link parked inside the housing, out of sight and out of reach
BIT_R = 0.001  # the 2 mm hex bit, as its inscribed radius
BIT_SOCKET_R = BIT_R + 0.00005  # the socket's flats 0.05 mm a side round the bit's: the 2 mm hex socket's width 2.014 to 2.074 mm (DIN 7991 / ISO 10642, s E11) on the bit's 1.96 to 2.00 mm (ISO 1173)
MOUTH_CH = 0.0002  # the socket's countersunk mouth, 0.2 mm at 45 degrees (ISO 10642: a countersink at the socket's mouth up to about 1.1 e)
FLOOR_UP = 0.0006  # the socket's floor and wall, as the bit meets them, are modelled 0.6 mm above the head's top, so they never touch the plate
SEAT_K = 40000.0  # N/m: the bolt's seat spring (10 N presses it 0.25 mm)
ENGAGE_LAT, ENGAGE_H, ENGAGE_DEG, PUSH_MIN = 0.0009, 0.0005, 10.0, float(os.environ.get("KNEE_PUSH_MIN", "5.0"))  # the bit drives only down in the hex socket: its tip within 0.9 mm of the socket's centre (the socket's wall stops it there) and at least 0.5 mm below the wall's top, its axis within 10 degrees of the bolt's, pushed with 10 N
RESIST = float(os.environ.get("KNEE_RESIST", "0.10"))  # N m: the bolt's running torque in the insert while it turns in
SNUG_K, TIGHT_T = 0.30 / math.radians(20.0), float(os.environ.get("KNEE_TIGHT_T", "0.30"))  # once the head is down: the torque rises 0.3 N m over a further 20 degrees of turn; home at 0.3 N m (M3, under the 1.37 N m maximum for 8.8 steel). Debug only: KNEE_PUSH_MIN, KNEE_RESIST and KNEE_TIGHT_T (at most 0.45, the 30 degree cap) set the range ends of these estimates
HOME_H = 0.0005  # a bolt is home with its head within 0.5 mm of the plate's face (the seat spring gives 0.35 mm under a 14 N push)
LAND_DEG = 3.0  # and square to the insert within 3 degrees: the 3 mm shank in the plate's 3.2 mm hole leans at most about 2.3 degrees over the hole's 5 mm
LAND_LAT = 0.0006  # the plate's hole within 0.6 mm of its insert's axis (the plate seated)
BIT_IN = 0.0012  # the bit stands 1.2 mm into a carried bolt's socket (the socket's depth, DIN 7991 M3: t min 1.1 mm)
PICK_LAT, PICK_H, PICK_DEG = 0.0009, 0.0012, 10.0  # the magnetic bit takes a bolt standing in the holder: its tip on the head, within 0.9 mm of the socket's axis (as for the drive: the bit over the 2 mm socket's mouth, which guides it in), no more than 1.2 mm above the head's top (the socket's depth; the bit's collider rests on the head about 0.8 mm up), its axis within 10 degrees of the bolt's
START_DEG = 5.0  # a carried bolt starts on its insert's first thread with the bit's axis within 5 degrees of the hole's (the 3.0 mm shank in the 3.2 mm hole of the 5 mm plate leans 2.3 degrees at most; the bolt can lean a little on the bit's magnet)
START_LO, START_HI = -0.0015, 0.0008  # and the bit's tip at the head's height on the first thread, from 1.5 mm below (the seat spring gives) to 0.8 mm above
START_HOVER, START_STEPS = 0.006, 5  # and it came straight down the hole's axis: before it starts, its tip hung at least 6 mm over the first thread, within ENGAGE_LAT of the axis and square, for 5 steps, and never left that axis since
MOVE_CAP, RECENTRES = 0.004, 3  # the demo's drive: no move to a target the arm misses by over 4 mm, at most 3 re-centres of the bit on a socket
PICK = os.environ.get("KNEE_PICK", "1") == "1"  # the bolts taken from the holder by the bit (debug KNEE_PICK=0: they come down loose in the plate's holes)
PICK_SLOTS = (2, 5)  # the holder's slots the demo takes its two bolts from, in the row away from the tower (any slot's bolt can be taken)


def _revolve(profile, n: int = 24, closed: bool = False) -> m.Mesh:
    """A surface of revolution about z from (radius, z) points; `closed` joins the last point to the first (a tube's section)."""
    pts = list(profile) + ([profile[0]] if closed else [])
    th = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    v = np.array([(r * math.cos(a), r * math.sin(a), z) for r, z in pts for a in th])
    f = []
    for i in range(len(pts) - 1):
        for k in range(n):
            a, b, c, e = i * n + k, i * n + (k + 1) % n, (i + 1) * n + (k + 1) % n, (i + 1) * n + k
            f += [(a, b, c), (a, c, e)]
    return m.Mesh(v, np.array(f, int))


def _moved(mesh, R=np.eye(3), t=(0.0, 0.0, 0.0)) -> m.Mesh:
    return m.Mesh(np.asarray(mesh.v, float) @ np.asarray(R, float).T + np.asarray(t, float), mesh.f)


def _bolt_mesh() -> m.Mesh:
    """DIN 7991 M3x10, the head's top at z 0, the tip at -10 mm."""
    r, hr = BOLT_D / 2, BOLT_HEAD_D / 2
    return _revolve([(1e-5, -BOLT_L), (r - 0.00025, -BOLT_L), (r, -BOLT_L + 0.00025), (r, -BOLT_HEAD_K), (hr, -0.0002), (hr, 0.0), (1e-5, 0.0)])


def _hexagon(across_flats: float) -> list:
    rc = across_flats / math.sqrt(3.0)
    return [(rc * math.cos(math.pi / 6 + k * math.pi / 3), rc * math.sin(math.pi / 6 + k * math.pi / 3)) for k in range(6)]


def _insert_parts() -> list:
    """The nine brass inserts, in the housing's frame (the plate frame at the knee's zero), each on its hole's axis."""
    tube = _revolve([(INSERT_D / 2, INSERT_BOT), (INSERT_D / 2, INSERT_TOP), (INSERT_ID / 2, INSERT_TOP), (INSERT_ID / 2, INSERT_BOT)], closed=True)
    return [(f"insert{i}", _moved(tube, t=(float(x), float(y), 0.0)), "brass") for i, (x, y) in enumerate(np.asarray(Z["holes"], float))]


BOLT_COL_R = BOLT_HEAD_D / 2  # the socket's colliders reach out to the head's rim


def _bolt_links() -> list:
    """Each insert's bolt (housing frame, the head's top at the plate's face when home): the seat slider and the bolt's hinge, children of the knee.
    The bolt's colliders are the socket's floor and its wall in four pieces, FLOOR_UP above the head's top: what the bit's tip meets."""
    links = []
    bolt, sock = _bolt_mesh(), m.prism(_hexagon(BOLT_HEX), -0.0012, 0.0001)
    r_in, r_out, wall = BIT_SOCKET_R, BOLT_COL_R, 0.0015
    for i in BOLT_HOLES:
        x, y = (float(v) for v in Z["holes"][i])
        cols = [m.col_cylinder(f"bfloor{i}", r_out, 0.0004, (x, y, FLOOR_UP - 0.0002))]  # 0.2 to 0.6 mm over the head's top: never below it, so it never meets the plate's face as the head goes home
        # a twelve-sided wall round the socket at the real fit (BIT_SOCKET_R: 0.05 mm a side round the bit), its top MOUTH_CH sloped out as the
        # socket's countersunk mouth
        side = 2 * (r_in + MOUTH_CH) * math.tan(math.radians(15.0)) + 0.0004
        low = wall - MOUTH_CH
        cols += [m.col_hull(f"bwall{i}_{j}", m.box((r_out - r_in, side, low), ((r_in + r_out) / 2, 0.0, 0.0)).turned(30.0 * j, "z").moved((x, y, FLOOR_UP + low / 2)))
                 for j in range(12)]
        for j in range(12):
            mouth = m.box((r_out - r_in, side, MOUTH_CH), ((r_in + r_out) / 2, 0.0, 0.0))
            v = mouth.v.copy()
            v[(v[:, 0] < (r_in + r_out) / 2) & (v[:, 2] > 0.0), 0] += MOUTH_CH  # the inner top edge out by the chamfer: a 45 degree lead-in
            cols.append(m.col_hull(f"bmouth{i}_{j}", m.Mesh(v, mouth.f).turned(30.0 * j, "z").moved((x, y, FLOOR_UP + low + MOUTH_CH / 2))))
        links.append(m.slider(f"seat{i}", "Z", PARK - 0.002, START_S + 0.002, [], [], at=(x, y, 0.0), mass=0.03, spring=SEAT_K, damping=20.0, effort=200.0, friction=0.0,
                              inertia=(1e-6, 1e-6, 1e-6), parent="knee"))
        links.append(m.hinge(f"bolt{i}", "Z", (x, y, 0.0), -1e5, 1e5, [(f"lbolt{i}", _moved(bolt, t=(x, y, 0.0)), "bolt"), (f"lsock{i}", _moved(sock, t=(x, y, 0.0)), "socket")],
                             cols, mass=0.03, spring=0.05, damping=0.002, friction=0.0, effort=5.0, inertia=(1e-6, 1e-6, 1e-6), parent=f"seat{i}"))
    return links


def _bolts():
    """The printed holder: nine bolts standing head up in its deck (each head a collider the bit touches) and the driver's tower beside them."""
    parts = [("holder", m.box((HOLDER_W, HOLDER_W, HOLDER_H), (0.0, 0.0, HOLDER_H / 2)), "tray")]
    cols = [m.col_box("c_holder", (HOLDER_W, HOLDER_W, HOLDER_H), (0.0, 0.0, HOLDER_H / 2))]
    tx, ty = TOWER
    side = (TOWER_W - TOWER_HOLE) / 2  # the socket's walls, four boxes round it
    for j, (cx, cy, sx, sy) in enumerate([(-(TOWER_HOLE + side) / 2, 0.0, side, TOWER_W), ((TOWER_HOLE + side) / 2, 0.0, side, TOWER_W),
                                          (0.0, -(TOWER_HOLE + side) / 2, TOWER_HOLE, side), (0.0, (TOWER_HOLE + side) / 2, TOWER_HOLE, side)]):
        parts.append((f"tower{j}", m.box((sx, sy, TOWER_H), (tx + cx, ty + cy, TOWER_H / 2)), "tray"))
        cols.append(m.col_box(f"c_tower{j}", (sx, sy, TOWER_H), (tx + cx, ty + cy, TOWER_H / 2)))
    bolt, sock = _bolt_mesh(), m.prism(_hexagon(BOLT_HEX), -0.0012, 0.0001)
    for i in range(9):
        x, y = _slot(i)
        parts += [(f"bolt{i}", _moved(bolt, t=(x, y, SIT)), "bolt"), (f"socket{i}", _moved(sock, t=(x, y, SIT)), "socket")]
        cols.append(m.col_cylinder(f"c_head{i}", BOLT_HEAD_D / 2, BOLT_HEAD_K, (x, y, SIT - BOLT_HEAD_K / 2)))
        cols.append(m.col_cylinder(f"c_shank{i}", BOLT_D / 2, SIT - BOLT_HEAD_K - HOLDER_H, (x, y, HOLDER_H + (SIT - BOLT_HEAD_K - HOLDER_H) / 2)))
    return m.free("bolts", (HOLDER[0], HOLDER[1], TOP + 0.0005), parts, cols, LOOKS, mass=0.050, friction=(MU, MU))  # one body: the printed holder (about 45 g) with its nine bolts (0.5 g each) standing in it


def _slot(i: int) -> tuple:
    """Slot i of the holder (holder frame): rows along x, from the far row."""
    return HOLDER_PITCH * (i % 3 - 1), HOLDER_PITCH * (1 - i // 3)


def _octagon(circumradius: float, scale: float = 1.0) -> list:
    """A regular octagon with a flat at local +x, -x, +y and -y (the handle's section)."""
    return [(scale * circumradius * math.cos(math.radians(22.5 + 45 * k)), scale * circumradius * math.sin(math.radians(22.5 + 45 * k))) for k in range(8)]


def _driver():
    """The ratchet driver standing in the holder's tower, bit down, the underside of its handle on the tower's top."""
    handle = m.lathe([(0.0, HANDLE_Z), (0.012, HANDLE_Z), (0.016, HANDLE_Z + 0.008), (HANDLE_R, HANDLE_Z + 0.020), (HANDLE_R, HANDLE_Z + 0.056),
                      (0.0165, HANDLE_Z + 0.065), (0.010, DRV_L), (0.0, DRV_L)], 24)
    parts = [("handle", handle, "drv_red"),
             ("ring", m.lathe([(0.0, HANDLE_Z - 0.001), (0.0125, HANDLE_Z - 0.001), (0.0125, HANDLE_Z + 0.006), (0.0, HANDLE_Z + 0.006)], 24), "drv_black"),
             ("nose", m.lathe([(0.0, NOSE_Z), (NOSE_R - 0.001, NOSE_Z), (NOSE_R, NOSE_Z + 0.002), (NOSE_R, HANDLE_Z), (0.0, HANDLE_Z)], 24), "steel"),
             ("shank", m.prism(_hexagon(0.00635), BIT_TIP_L, NOSE_Z), "steel"),
             ("bit", m.prism(_hexagon(0.002), 0.0, BIT_TIP_L), "steel"),
             ("cbolt", _moved(_bolt_mesh(), t=(0.0, 0.0, BIT_IN)), "bolt"), ("csock", _moved(m.prism(_hexagon(BOLT_HEX), -0.0012, 0.0001), t=(0.0, 0.0, BIT_IN)), "socket")]
    cols = [m.col_hull("c_handle", m.lathe([(0.0, HANDLE_Z), (0.012, HANDLE_Z), (HANDLE_R, HANDLE_Z + 0.020), (HANDLE_R, HANDLE_Z + 0.056), (0.010, DRV_L), (0.0, DRV_L)], 16)),
            m.col_cylinder("c_nose", NOSE_R, NOSE_L, (0.0, 0.0, NOSE_Z + NOSE_L / 2)),
            m.col_cylinder("c_shank", 0.0032, BIT_SHANK_L, (0.0, 0.0, BIT_TIP_L + BIT_SHANK_L / 2)),
            m.col_cylinder("c_bit", BIT_R, BIT_TIP_L, (0.0, 0.0, BIT_TIP_L / 2))]  # the 2 mm hex bit, as its 1 mm inscribed radius
    at = np.array([HOLDER[0] + TOWER[0], HOLDER[1] + TOWER[1], DRV0_Z])
    return m.free("driver", tuple(at), parts, cols, LOOKS, mass=DRV_KG, friction=(0.8, 0.7))


def _plate_start():
    """Where the plate starts: on the post; or, for the gates (the leg on its back), seated or dropped over the pocket."""
    if GATE in ("seated", "press") or START == "seated":
        return tuple(O_BACK), _quat_xyzw(R_BACK)
    if GATE == "drop":
        dx, dy, h, tilt, yaw = (float(v) for v in os.environ.get("KNEE_DROP", "0,0,7,0,0").split(","))
        R = R_BACK @ _rz(yaw) @ gk.rot((0.0, 1.0, 0.0), tilt)  # dx, dy in mm along the plate's x and y; tilt about its y, yaw about its axis, degrees
        return tuple(O_BACK + R_BACK @ np.array([dx, dy, h]) * 1e-3), _quat_xyzw(R)
    return tuple(PLATE0), _quat_xyzw(R_POST)


def _plate():
    pos, q = _plate_start()
    parts = [("plate", _mesh("plate"), "alu")] + [(f"{n}{k}", _moved(mesh, t=(float(Z["holes"][k][0]), float(Z["holes"][k][1]), 0.0)), look) for k in BOLT_HOLES
                                                   for n, mesh, look in (("bolt", _bolt_mesh(), "bolt"), ("socket", m.prism(_hexagon(BOLT_HEX), -0.0012, 0.0001), "socket"))]
    return m.free("plate", pos, parts, _hulls("pc"), LOOKS, mass=PLATE_KG, friction=(MU, MU), torsional_patch=0.015,
                  solver_iters=ITERS, start_quat=q)


OBJECTS = [_leg(), _post(), _plate(), _bolts(), _driver()]


def size_check() -> list:
    """The scene's key sizes, measured from the meshes it uses, against the maker's figures: (what, scene mm, published mm, source)."""
    pv, hv, H = np.asarray(Z["plate_v"], float), np.asarray(Z["housing_v"], float), np.asarray(Z["holes"], float)
    rp = np.hypot(pv[:, 0] - H[0, 0], pv[:, 1] - H[0, 1])
    rh = np.hypot(hv[:, 0] - H[0, 0], hv[:, 1] - H[0, 1])
    near_hole = (rh < 0.0030) & (hv[:, 2] < -0.00549) & (hv[:, 2] > -0.01001)  # under the 0.5 mm lead-in
    bolt = _bolt_mesh()
    bv = np.asarray(bolt.v, float)
    rows = [
        ("plate 600_08A thickness", 1000 * (pv[:, 2].max() - pv[(np.hypot(pv[:, 0], pv[:, 1]) > 0.025), 2].min()), 5.0, "600_08A STEP"),
        ("plate length", 1000 * (pv[:, 0].max() - pv[:, 0].min()), 149.4, "600_08A STEP"),
        ("plate hole", 2000 * rp[(rp < 0.0017) & (pv[:, 2] < -0.002)].max(), 3.2, "600_08A STEP"),
        ("plate countersink, top", 2000 * rp[(rp < 0.004) & (pv[:, 2] > -0.0001)].max(), 6.72, "600_08A STEP"),
        ("insert bore in housing 600_10C", 2000 * rh[near_hole].max(), 4.4, "600_10C STEP"),
        ("insert, outside / bore / length", f"{1000 * INSERT_D:.1f} / {1000 * INSERT_ID:.1f} / {1000 * (INSERT_TOP - INSERT_BOT):.1f}", "4.4 / 2.5 / 4.5", "600_10C STEP bore; M3 minor diameter"),
        ("holes and inserts (count)", str(len(H)), 9, "Menlo heat-insert drawing, 600_10C, inserts 1 to 9"),
        ("bolt length overall", 1000 * (bv[:, 2].max() - bv[:, 2].min()), 10.0, "Step 8: M3x10 Flat bolt, DIN 7991"),
        ("bolt thread / head / head height", f"{1000 * BOLT_D:.1f} / {1000 * BOLT_HEAD_D:.1f} / {1000 * BOLT_HEAD_K:.1f}", "3.0 / 6.0 / 1.7", "DIN 7991 M3"),
        ("bolt hex socket", 1000 * BOLT_HEX, 2.0, "DIN 7991 M3"),
        ("bolts (count)", "9", 9, "Step 8, instruction 5"),
        ("driver overall, without the bit", 1000 * (DRV_L - NOSE_Z), 95.0, "Bahco 808050S"),
        ("bit: hex across flats / standing out of the holder", f"{2.0:.1f} / {1000 * NOSE_Z:.0f}", "2.0 / 13 (25 long)", "Wera 840/1 Z Hex-Plus 2.0 x 25"),
        ("M3 pitch / turns from the insert's first thread to home", f"{1000 * PITCH:.1f} / {HOME_TURNS:.1f}", "0.5 / 9.0", "ISO 261 M3 coarse; 4.5 mm of thread in the insert"),
    ]
    for what, got, want, src in rows:
        unit = "" if "(count)" in what else " g" if "mass" in what else " mm"
        print(f"[size] {what}: scene {got if isinstance(got, str) else f'{got:.2f}'}{unit}, published {want}{unit} ({src})", flush=True)
    return rows


class Pin:
    """Dry friction in the knee actuator: the drive's rest stays put while its spring holds (up to STATIC); turned harder, the knee slips and
    the rest follows it at DYNAMIC's stretch, so a turning knee meets the lower drag."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.rest = 0.0

    def step(self, env):
        q = m.state(env, "leg").joint("knee")
        if abs(q - self.rest) > STATIC / PIN_K:
            self.rest = q - math.copysign(DYNAMIC / PIN_K, q - self.rest)
            m.set_target(env, "leg", "knee", self.rest)


PIN = Pin()
m.on_step(PIN.step)
m.on_reset(PIN.reset)


def pocket(leg) -> tuple[np.ndarray, np.ndarray]:
    """The pocket's frame in the world, from the leg's state: the housing turned on the knee from the thigh (the frame), about the axis
    through the pocket's centre, which so stays at the frame's origin. Returns its rotation and its origin (the pocket's axis at the face)."""
    return leg.rotation() @ _rz(math.degrees(leg.joint("knee"))), np.asarray(leg.pos, float).copy()


# -- the scorer ------------------------------------------------------------------------------------------------------------------------------------

# points on the plate's top face round its outline (plate frame, m): the round end, the fillets, the neck and its end. Their height above the
# housing's face is what a depth gauge or a straightedge with feeler gauges reads; seated, all are 0.
RIM = np.array([[-0.0385, 0.0, 0.0], [-0.027, 0.027, 0.0], [-0.027, -0.027, 0.0], [0.0, 0.0385, 0.0], [0.0, -0.0385, 0.0], [0.030, 0.021, 0.0],
                [0.030, -0.021, 0.0], [0.065, 0.0185, 0.0], [0.065, -0.0185, 0.0], [0.1085, 0.0, 0.0]])
HOLES = np.c_[np.asarray(Z["holes"], float), np.zeros(9)]  # the nine 3.2 mm holes, over the housing's nine M3 inserts when seated
MOUTH_H, ENTER_H, FLUSH_H = 0.0100, 0.0040, 0.0005  # the rim's height above the face, every point: on the mouth, 1 mm in, flush
ON_AXIS = 0.0015  # the boss's axis within 1.5 mm of the bearing's
HOLE_TOL = 0.00045  # each hole's axis within 0.45 mm of its insert's: a 2.4 mm pin gauge then passes the 3.2 mm hole (0.4 mm either way) into the M3 thread's 2.5 mm minor diameter (0.05 more). Read over the holes with no bolt started in them: a 3.0 mm bolt in the 3.2 mm hole holds that hole to 0.1 mm of its insert's axis, so the bolt itself shows it
HANDS_CLEAR = 0.10  # both grippers 10 cm from the plate and the face round it, so the last frames show the seated plate
STILL = 0.001  # m/s: under 1 mm in the 1 s hold
LEG_CLEAR = 0.08  # no fingertip within 8 cm of the leg's centre line (thigh's end, knee, calf's end): the leg is not held or propped at the end
LEG_STILL, LEG_SPIN = 0.002, 0.01  # m/s and rad/s: the leg still in the 1 s hold
OUTLINE = np.array([[0.0397 * math.cos(a), 0.0397 * math.sin(a), 0.0] for a in np.linspace(math.radians(37), math.radians(323), 30)]
                   + [[0.05, -0.0197, 0.0], [0.09, -0.0197, 0.0]] + [[0.09 + 0.0197 * math.cos(a), 0.0197 * math.sin(a), 0.0] for a in np.linspace(-math.pi / 2, math.pi / 2, 9)]
                   + [[0.09, 0.0197, 0.0], [0.05, 0.0197, 0.0], [0.0397 * math.cos(math.radians(37)), 0.0397 * math.sin(math.radians(37)), 0.0]])
CLEAR_RING = np.array([[0.055 * math.cos(a), 0.055 * math.sin(a), 0.0] for a in np.linspace(0, 2 * math.pi, 25)])  # the housing's face round the pocket
CALF = np.array([0.235, -0.006, -0.048])  # plate frame, the housing at the knee's zero: the middle of the calf's end
THIGH_END = np.array([-0.250, 0.004, -0.078])  # plate frame, the thigh: the middle of its top end
KNEE_C = np.array(KNEE_AT)  # the knee's centre, on its axis


def knee_now(env) -> dict:
    """What the scorer reads, every step: the plate in the pocket's frame as the leg lies now, and the leg's pose."""
    s = m.state(env, "plate")
    leg = m.state(env, "leg")
    R, o = pocket(leg)
    to_pocket = lambda p: (np.atleast_2d(p) - o) @ R  # noqa: E731
    rim = to_pocket(s.to_world(RIM))
    holes = to_pocket(s.to_world(HOLES))
    axis = to_pocket(s.pos)[0]
    Rp = R.T @ s.rotation()
    yaw = math.degrees(math.atan2(Rp[1, 0], Rp[0, 0]))
    tilt = math.degrees(math.acos(float(np.clip(Rp[2, 2], -1.0, 1.0))))
    face = math.degrees(math.acos(float(np.clip(R[2, 2], -1.0, 1.0))))  # the face's normal from straight up: 4.3 on its back, 90 on its side
    return {"h": rim[:, 2], "off": float(np.hypot(axis[0], axis[1])), "axis": axis, "holes": float(np.max(np.hypot(holes[:, 0] - HOLES[:, 0], holes[:, 1] - HOLES[:, 1])[[k not in BOLTS.started for k in range(len(HOLES))]])),
            "yaw": yaw, "tilt": tilt, "tips": m.fingertips(env), "outline": s.to_world(OUTLINE), "ring": o + CLEAR_RING @ R.T, "speed": float(np.linalg.norm(s.lin_vel)),
            "z": float(s.pos[2]), "xy": s.pos[:2].copy(), "face": face, "o": o, "leg_speed": float(np.linalg.norm(leg.lin_vel)), "leg_spin": float(np.linalg.norm(leg.ang_vel)),
            "ends": np.vstack([o + R @ CALF, leg.to_world(THIGH_END)]), "line": np.vstack([leg.to_world(THIGH_END), o + R @ KNEE_C, o + R @ CALF]),
            }


def _roll(R) -> float:
    """The driver's turn about its own axis (local z, toward the handle's end), from its rotation: positive is anticlockwise seen from the handle's end."""
    ax, side = R[:, 2], R[:, 0]
    ref = np.array([1.0, 0.0, 0.0]) - ax * ax[0]
    ref = ref / (np.linalg.norm(ref) + 1e-9)
    return math.atan2(float(np.dot(np.cross(ref, side), ax)), float(np.dot(ref, side)))


def _torque(env, part: str, tau) -> None:
    """Hold torque `tau` (N m, world) on a free part until changed: the bolt's resistance on the seated bit."""
    import torch

    asset = env.unwrapped.scene[part]
    t = torch.tensor([[[float(v) for v in tau]]], device=asset.device)
    asset.permanent_wrench_composer.set_forces_and_torques_index(forces=torch.zeros_like(t), torques=t, body_ids=[0], is_global=True)
    m._PUSHED.add((part, 0))


def _live(env) -> bool:
    return getattr(getattr(env, "unwrapped", None), "scene", None) is not None


class Bolts:
    """The bolts: which the bit carries, which stand in their holes, each one's turns, and the push on each seated bit (read at its seat spring)."""

    N = len(Z["holes"])

    def __init__(self) -> None:
        self.reset()

    def reset(self, env=None) -> None:
        self.started, self.taken, self.carried = set(), set(), None
        self.hover = {k: 0 for k in BOLT_HOLES}  # steps the carried bolt has hung on hole k's axis at least START_HOVER over its first thread
        self.turns, self.push, self.snug = np.zeros(self.N), np.zeros(self.N), np.zeros(self.N)
        self.last, self.engaged, self.log, self.n_home, self.ids = None, None, [], 0, None
        if env is not None and _live(env):
            for k in BOLT_HOLES:
                m.set_visible(f"{m.ENV_PATH}/leg/bolt{k}/visual", False)
                m.set_visible(f"{m.ENV_PATH}/plate/visual/bolt{k}", not PICK)
                m.set_visible(f"{m.ENV_PATH}/plate/visual/socket{k}", not PICK)
            for i in range(9):
                m.set_visible(f"{m.ENV_PATH}/bolts/visual/bolt{i}", True)
                m.set_visible(f"{m.ENV_PATH}/bolts/visual/socket{i}", True)
            for n in ("cbolt", "csock"):
                m.set_visible(f"{m.ENV_PATH}/driver/visual/{n}", False)

    def head(self, env, i: int) -> np.ndarray:
        """The top of the head of the bolt standing in the holder's slot i (world)."""
        x, y = _slot(i)
        return m.state(env, "bolts").to_world(np.array([[x, y, SIT]]))[0]

    def start_at(self, env, k: int) -> np.ndarray:
        """Where the bit's tip is when the bolt it carries rests on hole k's first thread (world)."""
        R, o = pocket(m.state(env, "leg"))
        return o + R @ np.array([HOLES[k, 0], HOLES[k, 1], START_S + FLOOR_UP])

    def target(self, k: int) -> float:
        return START_S - self.turns[k] * PITCH if k in self.started else PARK

    def driven(self) -> int:
        """Bolts started and driven at least DRIVEN_TURNS by the bit (each turn counts only seated in the socket, square and pushed)."""
        return int(sum(1 for k in self.started if self.turns[k] >= DRIVEN_TURNS - 1e-6))

    def home(self) -> int:
        return int(sum(1 for k in self.started if self.turns[k] >= HOME_TURNS - 1e-6))

    def _home_now(self, leg) -> int:
        """Bolts home: driven its full turns, its head down at the plate's face (the seat within 0.5 mm of it), and tightened to TIGHT_T."""
        return int(sum(1 for k in self.started if self.turns[k] >= HOME_TURNS - 1e-6 and leg.joint(f"seat{k}") <= HOME_H and SNUG_K * self.snug[k] >= TIGHT_T - 1e-9))

    def floor(self, env, k: int) -> np.ndarray:
        """Where bolt k's socket floor is, as the bit meets it (world)."""
        leg = m.state(env, "leg")
        R, o = pocket(leg)
        return o + R @ np.array([HOLES[k, 0], HOLES[k, 1], leg.joint(f"seat{k}") + FLOOR_UP])

    def _show(self, path: str, on: bool) -> None:
        m.set_visible(f"{m.ENV_PATH}/{path}", on)

    def step(self, env) -> None:
        drv, leg, plate = m.state(env, "driver"), m.state(env, "leg"), m.state(env, "plate")
        Rd = drv.rotation()
        tip, ax = np.asarray(drv.pos, float), Rd[:, 2]
        R, o = pocket(leg)
        u = R[:, 2]
        ph = (plate.to_world(HOLES) - o) @ R
        for k in BOLT_HOLES if not PICK else ():
            if k not in self.started and math.hypot(ph[k, 0] - HOLES[k, 0], ph[k, 1] - HOLES[k, 1]) <= LAND_LAT and ph[k, 2] <= START_S and float(plate.rotation()[:, 2] @ u) >= math.cos(math.radians(LAND_DEG)):
                self.started.add(k)
                self._place(env, k)
                self._show(f"plate/visual/bolt{k}", False), self._show(f"plate/visual/socket{k}", False)
                self._show(f"leg/bolt{k}/visual", True)
                self.log.append(f"landed in hole {k}")
                print(f"[bolts] the bolt in hole {k} landed on the insert's first thread: hole {1000 * math.hypot(ph[k, 0] - HOLES[k, 0], ph[k, 1] - HOLES[k, 1]):.2f} mm off, plate {1000 * ph[k, 2]:.2f} mm over seated", flush=True)
        # the pick: the bit's magnetic tip set on the head of a bolt standing in the holder, inside its socket, takes that bolt
        if PICK and self.carried is None:
            hb = m.state(env, "bolts")
            up = hb.rotation()[:, 2]
            for i in range(9):
                if i in self.taken:
                    continue
                rel = tip - self.head(env, i)
                h = float(rel @ up)
                if TRACE and np.linalg.norm(rel) < 0.004 and TRIAL.n % 5 == 0:
                    print(f"[pick] slot {i}: across {1000 * np.linalg.norm(rel - h * up):.2f} mm, up {1000 * h:.2f} mm, cos {float(ax @ up):.4f}", flush=True)
                if np.linalg.norm(rel - h * up) <= PICK_LAT and -0.0003 <= h <= PICK_H and float(ax @ up) >= math.cos(math.radians(PICK_DEG)):
                    self.carried = i
                    self.taken.add(i)
                    self._show(f"bolts/visual/bolt{i}", False), self._show(f"bolts/visual/socket{i}", False)
                    self._show("driver/visual/cbolt", True), self._show("driver/visual/csock", True)
                    self.log.append(f"picked slot {i}")
                    print(f"[bolts] the bit took the bolt from slot {i}", flush=True)
                    break
        # the start: the carried bolt's tip put into hole k of the seated plate (the plate's hole over its insert, within 1 mm of flush and square)
        # and let down onto the insert's first thread, the bit square to the hole: the bolt stands there on its thread (head top START_S up)
        sq = float(plate.rotation()[:, 2] @ u) >= math.cos(math.radians(LAND_DEG))
        if PICK and self.carried is not None:
            for k in BOLT_HOLES:
                if k in self.started or not (sq and math.hypot(ph[k, 0] - HOLES[k, 0], ph[k, 1] - HOLES[k, 1]) <= LAND_LAT and abs(ph[k, 2]) <= 0.001):
                    continue
                rel = tip - self.start_at(env, k)
                h = float(rel @ u)
                lat = float(np.linalg.norm(rel - h * u))
                if lat > ENGAGE_LAT or float(ax @ u) < math.cos(math.radians(START_DEG)):
                    self.hover[k] = 0  # off the hole's axis or not square: the bolt must line up above the hole again before it can start
                elif h >= START_HOVER:
                    self.hover[k] += 1
                if self.hover[k] >= START_STEPS and START_LO <= h <= START_HI:
                    self.started.add(k)
                    self._place(env, k)
                    self._show("driver/visual/cbolt", False), self._show("driver/visual/csock", False)
                    self._show(f"leg/bolt{k}/visual", True)
                    self.log.append(f"started in hole {k} (slot {self.carried})")
                    print(f"[bolts] the bolt from slot {self.carried} started in hole {k}: bit {1000 * np.linalg.norm(rel - h * u):.2f} mm off the axis, {1000 * h:.2f} mm over the first thread", flush=True)
                    self.carried = None
                    break
        roll = _roll(Rd)
        self.engaged = None
        for k in self.started:
            s = leg.joint(f"seat{k}")
            self.push[k] = SEAT_K * max(0.0, self.target(k) - s)
            rel = tip - (o + R @ np.array([HOLES[k, 0], HOLES[k, 1], s + FLOOR_UP]))
            h = float(rel @ u)
            if np.linalg.norm(rel - h * u) <= ENGAGE_LAT and -0.0025 <= h <= ENGAGE_H and float(ax @ u) >= math.cos(math.radians(ENGAGE_DEG)) and self.push[k] >= PUSH_MIN:
                self.engaged = k
        w = float(np.dot(drv.ang_vel, ax)) if self.engaged is not None else 0.0  # negative: turning the tightening way (clockwise from the handle's end)
        k = self.engaged
        tau = 0.0 if k is None else RESIST * float(np.clip(-w / 0.02, 0.0, 1.0)) + SNUG_K * self.snug[k]
        _torque(env, "driver", tau * ax)  # the bolt resists the tightening turn (and, once its head is down, springs back by its tightening torque); the ratchet frees the other way
        if self.last is not None and k is not None:
            dd = (roll - self.last + math.pi) % (2 * math.pi) - math.pi
            if dd < 0.0:  # the ratchet drives only the tightening turn: the thread first, then, with the head down, the tightening
                if self.turns[k] < HOME_TURNS - 1e-6:
                    self.turns[k] = min(HOME_TURNS, self.turns[k] - dd / (2 * math.pi))
                elif leg.joint(f"seat{k}") <= HOME_H:
                    self.snug[k] = min(self.snug[k] - dd, math.radians(30.0))
        self.last = roll
        self.n_home = self._home_now(leg)
        self._write(env)

    def _place(self, env, k: int) -> None:
        """The bolt in hole k on its first thread: its seat set to START_S."""
        asset, j = m._joint(env, "leg", f"seat{k}")
        asset.write_joint_state_to_sim(m._tensor(asset, START_S), m._tensor(asset, 0.0), joint_ids=[j])

    def _write(self, env) -> None:
        """Each bolt's seat and turn targets: the thread's advance follows the turns."""
        import torch

        if self.ids is None:
            self.ids = [m._joint(env, "leg", f"seat{k}")[1] for k in BOLT_HOLES] + [m._joint(env, "leg", f"bolt{k}")[1] for k in BOLT_HOLES]
        asset = env.unwrapped.scene["leg"]
        vals = [self.target(k) for k in BOLT_HOLES] + [-(self.turns[k] * 2 * math.pi + self.snug[k]) for k in BOLT_HOLES]
        asset.set_joint_position_target(torch.tensor([vals], device=asset.device, dtype=torch.float32), joint_ids=self.ids)


BOLTS = Bolts()
m.on_step(BOLTS.step)
m.on_reset(BOLTS.reset)


DRIVEN_TURNS = 2.0  # a bolt counts as started once the bit has driven it 2 full turns into the insert: its head 1 mm down
LADDER = gk.Ladder(("turned", 0.2), ("seated", 0.4), ("one", 0.6), ("two", 0.8), ("done", 1.0))  # the five rungs of the scoring text


class Trial:
    """The trial's latches and the score. Zero, latched: the plate or the leg off the table (below its top, or past its edge)."""

    def __init__(self):
        self.fell = gk.Latch("the plate or the leg fell off the table")
        self.hold = gk.EndHold(HOLD_STEPS)
        self.lifted = gk.Latch("a gripper lifted the plate out of the post")
        self.early = gk.Latch("the plate was set in the pocket before the leg was turned on its back")
        self.slid = gk.Latch("the leg slid or rocked once the plate had reached the pocket's mouth")
        self.reset()

    def reset(self, env=None):
        self.n, self.now, self.full, self.best, self.carry, self.mouth_at, self.sat = 0, None, False, 0.0, 0, None, False
        self.fell.reset(); self.hold.reset(); self.lifted.reset(); self.early.reset(); self.slid.reset()

    def step(self, env):
        self.n += 1
        self.now = knee_now(env)
        st = self.now
        off_table = lambda p: p[2] < TOP - 0.01 or abs(p[1]) > 0.62 or p[0] < -0.1 or p[0] > 0.75  # noqa: E731
        self.fell.trip(off_table(np.r_[st["xy"], st["z"]]) or off_table(st["o"]) or bool(np.any(st["ends"][:, 2] < TOP - 0.02)))
        # carried: 10 steps in a row 1 cm up out of the post with a fingertip within 3 cm of its outline (a grasp, not a flick or a one-sided prod)
        near = st["z"] > PLATE0[2] + 0.010 and float(gk.min_distance(st["tips"], [st["outline"]])) < 0.03
        self.carry = self.carry + 1 if near else 0
        self.lifted.trip(self.carry >= 10)
        mouth = st["off"] <= ON_AXIS and st["h"].max() <= MOUTH_H and st["h"].min() >= -0.001 and st["tilt"] < 20.0
        # order: the plate on the pocket's mouth while the leg is not on its back (set in first, the leg turned after) is not the act
        self.early.trip(mouth and not self._on_back(st))
        # the leg held still while the plate goes in: from the step the plate first reaches the mouth (never re-anchored), the pocket moves under
        # SLIDE_TOL and the face turns under ROCK_TOL; a leg that slides away or rocks under the press does not count the seat
        if mouth and self.mouth_at is None:
            self.mouth_at = (st["o"].copy(), st["face"])
        # until the plate is seated: the bolts are driven into a seated plate, and the driver's press may rock the leg on its points
        if self.mouth_at is not None and not self.sat:
            self.slid.trip(float(np.linalg.norm(st["o"] - self.mouth_at[0])) > SLIDE_TOL or abs(st["face"] - self.mouth_at[1]) > ROCK_TOL or st["face"] > LEVEL_TOL)
        rs = self.reached(st)
        self.sat = self.sat or rs["seated"]
        self.full = self.hold.step(rs["done"])
        sc = self.score()
        self.best = max(self.best, sc)
        if TRACE and (self.n % 20 == 0 or (GATE and (self.n < 60 and self.n % 5 == 0))):
            print(f"[knee] step {self.n}: face {st['face']:.2f} deg from up, at {np.round(1000 * (st['o'] - [0, 0, TOP]), 1)} mm; rim above the face {1000 * st['h'].min():.2f} to "
                  f"{1000 * st['h'].max():.2f} mm, boss axis off {1000 * st['off']:.3f} mm, holes off {1000 * st['holes']:.2f} mm, tilt {st['tilt']:.2f} deg; score {sc}", flush=True)

    @staticmethod
    def _on_back(st) -> bool:
        """The leg on its back on the table: the face within LEVEL_TOL of level, and at the height it rests at there (not held up off the table)."""
        return st["face"] <= LEVEL_TOL and abs(st["o"][2] - (TOP + D_BACK)) <= REST_TOL

    def reached(self, st) -> dict:
        h = st["h"]
        turned = self._on_back(st)
        mouth = turned and st["off"] <= ON_AXIS and h.max() <= MOUTH_H and h.min() >= -0.001 and st["tilt"] < 20.0  # boss down: a flipped plate (tilt 180) reads nothing
        entered = mouth and h.max() <= ENTER_H
        flush = entered and np.abs(h).max() <= FLUSH_H and st["tilt"] < 1.0
        seated = flush and st["holes"] <= HOLE_TOL
        clear = (gk.hands_clear(st["tips"], [st["outline"], st["ring"]], HANDS_CLEAR) and gk.hands_clear(st["tips"], [st["line"]], LEG_CLEAR))
        still = st["speed"] < STILL and st["leg_speed"] < LEG_STILL and st["leg_spin"] < LEG_SPIN
        n, d = BOLTS.n_home, BOLTS.driven()
        return {"turned": turned, "seated": seated, "one": seated and d >= 1, "two": seated and d >= 2, "done": seated and n >= 2 and clear and still}

    def score(self) -> float:
        if self.now is None:
            return 0.0
        r = self.reached(self.now)
        r["done"] = self.full and r["done"]  # the 1 s hold was met, and now: seated, hands clear, plate and leg still
        above = ["seated", "one", "two", "done"]
        if (self.early or self.slid) and not GATE:
            r.update({k: False for k in above})
        if not (GATE or self.lifted or START == "seated"):  # a plate that no gripper ever lifted from the post (thrown, tapped, dropped from far) scores at most 0.5
            r.update({k: False for k in above})
        names = {n for n, _ in LADDER.rungs}
        return LADDER.score({k: v for k, v in r.items() if k in names}, failed=bool(self.fell))


TRIAL = Trial()
m.on_step(TRIAL.step)
if os.environ.get("KNEE_PROF"):  # wall time per hook, every 100 steps (a speed check)
    import time as _time

    _PT = {}

    def _timed(fn, name):
        def run(env):
            t0 = _time.perf_counter()
            fn(env)
            _PT[name] = _PT.get(name, 0.0) + _time.perf_counter() - t0
            if name == "trial" and TRIAL.n % 100 == 0:
                print(f"[prof] step {TRIAL.n}: " + ", ".join(f"{k} {v:.2f} s" for k, v in _PT.items()), flush=True)
        return run

    m._STEP[:] = [_timed(f, n) for f, n in zip(m._STEP, ["pin", "bolts", "trial"])] if len(m._STEP) == 3 else m._STEP
m.on_reset(TRIAL.reset)


UNDO = {"on": False}  # the undo route runs on past the seat, so success must not end it there


def success(objects) -> bool:
    if GATE or UNDO["on"]:  # the gates watch the plate for their whole length; the undo goes on to put a hand back on the plate
        return False
    return bool(TRIAL.full and (TRIAL.lifted or START == "seated") and not TRIAL.early and not TRIAL.slid and TRIAL.score() >= 1.0 and not TRIAL.fell)


def score(objects) -> float:
    return TRIAL.score()


# -- the demo ------------------------------------------------------------------------------------------------------------------------------------

PINCH_X = 0.062  # the pinch across the neck's flats (39.4 mm apart), this far along the plate from the boss's axis
REGRIP_DEG = float(os.environ.get("KNEE_REGRIP_DEG", "8.0"))  # the plate tilted more than this in the vertical pinch (4 to 5 when pinched square): put it in the socket and pinch again
DRV_YAW = float(os.environ.get("KNEE_DRV_YAW", "-1"))  # the hand's heading over the screwdriver's handle (-1: the best reach, found by IK)
J6 = math.radians(float(os.environ.get("KNEE_J6", "80")))  # each wrist stroke turns joint 6 through 2 J6 (0.42 turn), from J6 behind to J6 past where it stands over the socket
STROKE_T, BACK_T = 2.5, 1.2  # s: the turning stroke, and the ratchet's free back stroke
PUSH_GOAL = float(os.environ.get("KNEE_PUSH_GOAL", "7.0"))
J6_LIMIT = math.radians(120.0)  # the YAM wrist roll (joint 6) stops at about +/-120 deg (the arm sticks at -120 in the runs)  # N: the push the hand keeps on the bit
GUIDE = os.environ.get("KNEE_GUIDE", "0") != "0"
REPRESS = os.environ.get("KNEE_REPRESS", "0") != "0"
GUIDE_H, GUIDE_GAP, GUIDE_BACK, GUIDE_TURN_BACK = float(os.environ.get("KNEE_GUIDE_H", "0.030")), float(os.environ.get("KNEE_GUIDE_GAP", "0.006")), 0.060, 0.080  # the left fingers round the nose 30 mm up from the bit's tip, 6 mm wider than the 12 mm nose (a lean of at most about 6 degrees), come in 60 mm along their axis, turned level 80 mm further back
STEADY_X, STEADY_DEPTH = -0.020, 0.0005  # the left fingertips rest on the plate's round end 20 mm behind the boss's axis, sent 0.5 mm below its top: a light touch
STEADY2 = os.environ.get("KNEE_STEADY2", "off")  # the left hand on the second bolt: "off" (it leaves the plate before the second pick), "far" (back on at STEADY_X2), "on" (stays)
STEADY_X2 = -0.045
PRESSES = 6  # press until the plate reads flush at every rim point, at most this many times
PICK_OPEN = -0.030  # the fingers 60 mm apart coming down over the neck: 10 mm either side of its 39.4 mm, and the outer finger clear of the leg lying beside the post
RELEASE_H = float(os.environ.get("KNEE_RELEASE", "0.00575"))  # let go with the plate this far above seated: its underside 0.75 mm over the face, the fingertips at the face beside the neck
PRESS_X = float(os.environ.get("KNEE_PRESS_X", "0.010"))  # press here on the plate's top, between the boss's axis and its centre of mass
PRESS_DEPTH = float(os.environ.get("KNEE_PRESS_DEPTH", "0.020"))  # the fingertips sent this far below flush
RELEASE_TOL = float(os.environ.get("KNEE_RELEASE_TOL", "0.00012"))
GRIP_CALF = np.array([0.235, -0.016, -0.0467])  # plate frame: the right hand's grip on the calf's end, across its 59 mm thickness, 21 mm in from its top side
GRIP_THIGH = np.array([-0.252, -0.028, -0.078])  # plate frame: the left hand's grip on the thigh's top end, across its 60 mm thickness
TURN_T = float(os.environ.get("KNEE_TURN_T", "5.0"))  # seconds for the turn
TURN_DEG = float(os.environ.get("KNEE_TURN_DEG", "66"))  # the hands turn the leg this far (its centre of mass passes over the edge at 38), then
# let go: their wrists cannot follow it the last 20 degrees, and it settles onto its back from there


def _tips(env, side: str = "right") -> np.ndarray:
    """The middle of one hand's two fingertips (the points 147 mm out along the gripper)."""
    t = m.fingertips(env)
    t = t[12:24] if side == "right" else t[0:12]
    return t[0::2].mean(axis=0)


def _tips_all(env) -> np.ndarray:
    """The right hand's six fingertip points."""
    return m.fingertips(env)[12:24][0::2]


def _check_frame(env):
    """The knee's sign against the simulator's own pose of the housing (once, in the gates and the demo)."""
    if getattr(env, "unwrapped", None) is None or getattr(env.unwrapped, "scene", None) is None:  # pathcheck's dry run: no simulator
        return
    asset = env.unwrapped.scene["leg"]
    i = asset.body_names.index("knee")
    pose = asset.data.body_link_pose_w[0, i].detach().cpu().numpy().astype(float)
    R_sim = m.quat_matrix(pose[3:7])
    R_want = pocket(m.state(env, "leg"))[0]
    err = math.degrees(math.acos(float(np.clip((np.trace(R_sim.T @ R_want) - 1.0) / 2.0, -1.0, 1.0))))
    print(f"[knee] frame check: knee {math.degrees(m.state(env, 'leg').joint('knee')):.2f} deg; the simulator's housing differs from the pocket's frame by {err:.3f} deg", flush=True)


def _leg_report(env, label: str) -> None:
    st = knee_now(env)
    leg = m.state(env, "leg")
    print(f"[knee] {label}: face {st['face']:.2f} deg from up, origin {np.round(1000 * (st['o'] - [0, 0, TOP]), 1)} mm (on its back it rests at z {1000 * D_BACK:.1f}), "
          f"knee {math.degrees(leg.joint('knee')):.2f} deg, leg speed {1000 * st['leg_speed']:.1f} mm/s, spin {st['leg_spin']:.3f} rad/s", flush=True)


DIAG = os.environ.get("KNEE_DIAG", "") == "1"


def _diag(env, arm, label: str) -> None:
    """KNEE_DIAG=1: the held plate against the pocket (axis offset, height, tilt about the pocket's x and y, yaw), the hand commanded against
    observed, and each fingertip's height above the face and where it is in the pocket's frame."""
    if not DIAG:
        return
    s, leg = m.state(env, "plate"), m.state(env, "leg")
    R, o = pocket(leg)
    a = (s.pos - o) @ R
    Rp = R.T @ s.rotation()
    rx = math.degrees(math.atan2(Rp[2, 1], Rp[2, 2]))
    ry = math.degrees(math.asin(float(np.clip(-Rp[2, 0], -1.0, 1.0))))
    yaw = math.degrees(math.atan2(Rp[1, 0], Rp[0, 0]))
    cmd = d.ik.tcp_pose(arm.q, arm.base)[0]
    lag = arm.tcp() - cmd
    t = m.fingertips(env)
    tp = (t[12:24][0::2] - o) @ R  # the right hand's six fingertip points, pocket frame
    tl = (t[0:12][0::2] - o) @ R
    sheet = (t[12:24][0::2] - s.pos) @ s.rotation()  # the same, in the plate's frame
    print(f"[diag] {label}: plate axis {np.round(1000 * a[:2], 2)} mm, h {1000 * a[2]:.2f} mm, tilt x {rx:.2f} y {ry:.2f} yaw {yaw:.2f} deg, speed {1000 * np.linalg.norm(s.lin_vel):.1f} mm/s; "
          f"hand lag {np.round(1000 * lag, 1)} mm (obs-cmd), trim {np.round(1000 * arm.trim, 1)}; right tips z over face {np.round(1000 * tp[:, 2], 1)} at x {np.round(1000 * tp[:, 0], 0)} y {np.round(1000 * tp[:, 1], 0)}; "
          f"tips in plate frame z {np.round(1000 * sheet[:, 2], 1)}; left tips min z over face {1000 * tl[:, 2].min():.0f}", flush=True)


def _props(env) -> dict:
    return {n: (np.asarray(m.state(env, n).pos, float).copy(), m.state(env, n).rotation().copy()) for n in ("driver", "bolts")}


def _props_moved(env, p0) -> str:
    """Each prop's start and end pose (x y z in mm, heading in deg) and how far it travelled."""
    out = []
    for n, (pos, R) in p0.items():
        s = m.state(env, n)
        end = np.asarray(s.pos, float)
        ang = math.degrees(math.acos(float(np.clip((np.trace(R.T @ s.rotation()) - 1.0) / 2.0, -1.0, 1.0))))
        out.append(f"{n} start {np.round(1000 * pos, 2)} end {np.round(1000 * end, 2)} mm, travel {1000 * float(np.linalg.norm(end - pos)):.2f} mm, turned {ang:.2f} deg")
    return "; ".join(out)


def demo_script(ctx):
    """Modes: full: both hands grip the leg's ends from above (the right the calf's end, the left the thigh's top end), and turn it together
    over its lower edge, toward the robot, until it lies on its back, then let go; the right hand pinches the plate across its neck, lifts it
    out of the post's socket, brings it over the pocket square and turned to it, lets go 0.75 mm above the face, and presses it home; then both
    hands clear. turn: the turn alone, then a report. gate: the arms wait (KNEE_GATE=press: the leg on its back and the plate seated, a push
    of KNEE_PUSH newtons at the boss for 2 s, then KNEE_PUSH2 at the plate's side, KNEE_PUSH_Y off the axis, for 2 s; the leg's motion after
    each). none: the arms wait. undo: full, then the right hand back on the seated plate's top to the end (the score falls to 0.75)."""
    l, r = ctx.left, ctx.right
    env = ctx.emb._env
    mode = ctx.mode
    plate = lambda: m.state(env, "plate")  # noqa: E731
    leg = lambda: m.state(env, "leg")  # noqa: E731
    if mode in ("none", "gate"):
        _check_frame(env)
        n = int(float(os.environ.get("KNEE_WAIT", "8")) * d.CONTROL_HZ)
        p1, p2, py = float(os.environ.get("KNEE_PUSH", "0")), float(os.environ.get("KNEE_PUSH2", "0")), float(os.environ.get("KNEE_PUSH_Y", "0.035"))
        o0 = None
        track = []
        for k in range(n):
            if k == 20:
                _leg_report(env, "settled")
                o0 = knee_now(env)["o"]
            if p1 and k == 20:
                m.push(env, "plate", (0.0, 0.0, -p1), at=plate().to_world(np.array([[PRESS_X, 0.0, 0.0]]))[0])
            if p1 and k == 60:
                m.push(env, "plate", (0.0, 0.0, 0.0))
                st = knee_now(env)
                print(f"[knee] GATE press {p1:.0f} N at the boss for 2 s: leg moved {1000 * np.linalg.norm(st['o'] - o0):.2f} mm, face {st['face']:.2f} deg from up", flush=True)
            if p2 and k == 80:
                m.push(env, "plate", (0.0, 0.0, -p2), at=plate().to_world(np.array([[PRESS_X, py, 0.0]]))[0])
            if p2 and k == 120:
                m.push(env, "plate", (0.0, 0.0, 0.0))
                st = knee_now(env)
                print(f"[knee] GATE press {p2:.0f} N at the plate's side, {1000 * py:.0f} mm off its axis, for 2 s: leg moved {1000 * np.linalg.norm(st['o'] - o0):.2f} mm, face {st['face']:.2f} deg from up", flush=True)
            if k >= 60:
                s = plate()
                track.append((s.pos.copy(), float(np.linalg.norm(s.lin_vel))))
            yield
        if track:
            p = np.array([t[0] for t in track])
            st = knee_now(env)
            print(f"[knee] GATE {GATE or mode} over {len(track) / d.CONTROL_HZ:.1f} s: plate drift {1000 * np.linalg.norm(p - p[0], axis=1).max():.4f} mm, top speed {1000 * max(t[1] for t in track):.3f} mm/s; "
                  f"end: leg moved {1000 * np.linalg.norm(st['o'] - o0):.2f} mm, face {st['face']:.3f} deg from up, rim {1000 * st['h'].min():.3f} to {1000 * st['h'].max():.3f} mm above the face, "
                  f"boss axis off {1000 * st['off']:.3f} mm, holes off {1000 * st['holes']:.3f} mm, tilt {st['tilt']:.3f} deg; score {TRIAL.score()}", flush=True)
        return
    if mode not in ("full", "turn", "undo", "undriven", "one_bolt", "no_press", "push_only", "ccw", "off_socket", "placed", "side_slide"):
        raise ValueError(f"unknown mode {mode!r}")
    UNDO["on"] = mode == "undo"
    _check_frame(env)
    size_check()
    yield from r.wait(1.0)  # the props settle onto the table before their pose is read
    props0 = _props(env)
    if START != "seated":  # debug only: KNEE_START=seated starts with the plate seated and runs the bolts alone
        # 1. the turn. Each hand comes down over one end of the leg, its fingers either side of that end's thickness (the leg lies on its side, the
        # face toward +x, so the thickness runs along x), and closes. Then both swing together about the edge the leg rests on, toward the robot:
        # the leg tips over that edge onto its back, held all the way, so it is lowered, not dropped
        yield from r.wait(0.5)
        _leg_report(env, "at the start")
        R_pk, o0 = pocket(leg())
        gr, gl = o0 + R_pk @ GRIP_CALF, leg().to_world(GRIP_THIGH)[0]
        edge_p, edge_u = leg().to_world(EDGE_C)[0], leg().rotation() @ EDGE_U
        yield from ctx.together(r.move(gr + [0.0, 0.0, 0.10], grip=d.down(90), finger=d.FINGER_OPEN, t=2.5, label="over the calf's end"),
                                l.move(gl + [0.0, 0.0, 0.05], grip=d.down(-90), finger=d.FINGER_OPEN, t=2.5, label="over the thigh's end"))
        yield from ctx.together(r.move(gr, t=1.5, label="fingers either side of the calf's end"), l.move(gl, t=1.0, label="fingers either side of the thigh's end"))
        yield from ctx.together(r.close(0.059, force=38.0, t=0.6, label="grip the calf's end"), l.close(0.060, force=38.0, t=0.6, label="grip the thigh's end"))
        yield from ctx.together(r.arc(edge_p, edge_u, -TURN_DEG, t=TURN_T, label="turn the leg over its edge"), l.arc(edge_p, edge_u, -TURN_DEG, t=TURN_T, label="turn the leg over its edge"))
        yield from r.wait(0.5)
        _leg_report(env, f"turned {TURN_DEG:.0f} deg, held")
        yield from ctx.together(r.open(t=0.4, label="let go of the calf"), l.open(t=0.4, label="let go of the thigh"))
        yield from ctx.together(r.move(r.at() - 0.06 * r.approach_axis(), t=1.0, label="back off the calf"), l.move(l.at() - 0.06 * l.approach_axis(), t=1.0, label="back off the thigh"))
        yield from ctx.together(r.move(r.at() + [0.0, 0.0, 0.10], t=1.0, label="up"), l.move(np.array([0.15, 0.25, TOP + 0.20]), grip=d.down(-90), t=1.5, label="the left hand away"))
        yield from r.wait(0.8)
        _leg_report(env, "turned, let go")
        if mode == "turn":
            yield from r.wait(2.0)
            _leg_report(env, "turned, 2 s later")
            return
    g = d.down(90)  # fingers closing along world x
    # 2. the plate: pinch it across its neck and lift it out of the post's socket
    at = lambda p: plate().to_world(np.array([p]))[0]  # noqa: E731  (a plate-frame point in the world, as the plate is now)
    tips = lambda: _tips(env)  # noqa: E731
    def pd(label):
        """KNEE_DIAG=1: the plate on or over the post: its tilt from level, its origin from where it sits in the socket, its yaw from the post's,
        and the right fingertips in its frame."""
        if not DIAG:
            return
        Rw = plate().rotation()
        tp = (_tips_all(env) - plate().pos) @ Rw
        yaw = math.degrees(math.atan2((R_POST.T @ Rw)[1, 0], (R_POST.T @ Rw)[0, 0]))
        print(f"[pdiag] {label}: tilt {math.degrees(math.acos(float(np.clip(Rw[2, 2], -1.0, 1.0)))):.2f} deg, at {np.round(1000 * (plate().pos - PLATE0), 2)} mm, yaw {yaw:.2f} deg; "
              f"tips in plate frame x {np.round(1000 * tp[:, 0], 0)} y {np.round(1000 * tp[:, 1], 1)} z {np.round(1000 * tp[:, 2], 1)}", flush=True)

    pd("on the post")
    pinch = at([PINCH_X, 0.0, -0.005 - 0.001])
    yield from r.move(pinch + [0.0, 0.0, 0.10], grip=g, finger=PICK_OPEN, t=2.0, label="over the plate's neck")
    yield from r.servo(pinch + [0.0, 0.0, 0.05], tol=0.002, timeout=1.5, gain=0.3, read=tips, label="fingertips over the neck")
    yield from r.move(r.at() + [0.0, 0.0, -0.05], t=1.2, label="fingers down either side of the neck")
    yield from r.servo(pinch, tol=0.001, timeout=1.5, gain=0.3, read=tips, label="fingertips at the neck")
    pd("after fingertips at the neck")
    yield from r.close(0.0394, force=38.0, t=0.6, label="pinch the neck across its flats")
    pd("after pinch the neck across its flats")
    yield from r.move(r.at() + [0.0, 0.0, 0.045], t=1.2, label="lift it out of the socket")
    pd("after lift it out of the socket")
    # check the grip on what is seen: the fingertips should sit about 1 mm under the sheet. A plate that slid up in the fingers as it left the
    # socket puts them lower, and they would meet the housing's face before the plate reaches the pocket: set it back, open, and pinch again
    # A plate that was not down in the socket (it rests cocked on the post's rim, its pad on the rim) is pinched cocked, and stays so in the
    # fingers: leveling it over the pocket then tilts the hand as far, and a fingertip meets the housing's face before the plate is low enough
    # to let go. So the plate's tilt in the vertical grip is read too: over REGRIP_DEG, level it in the hand, bring its pad over the socket,
    # lower it in and let go, so it lies square in the socket, then pinch again
    def reseat():
        yield from r.move(r.at() + [0.0, 0.0, 0.02], t=0.6, label="up, clear of the post")
        for _k in range(2):
            n = plate().rotation()[:, 2]
            ax = np.cross(n, [0.0, 0.0, 1.0])
            ang = math.degrees(math.asin(float(np.clip(np.linalg.norm(ax), 0.0, 1.0))))
            if ang > 0.3:
                yield from r.turn(ang, axis=ax / np.linalg.norm(ax), t=0.8, label="level the plate in the hand")
            yield from r.wait(0.4)
        for dz, tries in ((0.015, 4), (0.0015, 3)):
            for _k in range(tries):
                err = PLATE0 + [0.0, 0.0, dz] - plate().pos
                if np.linalg.norm(err[:2]) < 0.0003 and abs(err[2]) < 0.001:
                    break
                yield from r.move(r.at() + err, t=0.8, label="its pad over the socket")
                yield from r.wait(0.4)
        print(f"[knee] reseat: pad {np.round(1000 * (plate().pos - PLATE0), 2)} mm from in the socket, tilt {math.degrees(math.acos(float(np.clip(plate().rotation()[2, 2], -1.0, 1.0)))):.2f} deg", flush=True)
        yield from r.open(t=0.3, label="let go in the socket")
        yield from r.wait(0.3)
        pd("let go in the socket")
        yield from r.move(r.at() + [0.0, 0.0, 0.03], t=0.8, label="fingers up")
        yield from r.move(r.at() + [0.0, 0.0, 0.02], grip=g, finger=PICK_OPEN, t=1.0, label="fingers upright")
        yield from r.wait(0.3)
        pd("reseated, let go")

    for _ in range(3):
        low = float(at([PINCH_X, 0.0, -0.005])[2] - tips()[2])
        cocked = math.degrees(math.acos(float(np.clip(plate().rotation()[2, 2], -1.0, 1.0))))
        print(f"[knee] held: fingertips {1000 * low:.1f} mm under the sheet at the pinch, the plate {cocked:.1f} deg from level", flush=True)
        if low < 0.0025 and cocked < REGRIP_DEG:
            break
        if cocked >= REGRIP_DEG:
            yield from reseat()
        else:
            yield from r.move(r.at() + [0.0, 0.0, -0.045], t=1.2, label="set it back in the socket")
            yield from r.open(t=0.3, label="let go in the socket")
            yield from r.move(r.at() + [0.0, 0.0, 0.03], finger=PICK_OPEN, t=0.8, label="fingers up")
        pinch = at([PINCH_X, 0.0, -0.005 - 0.001])
        yield from r.servo(pinch + [0.0, 0.0, 0.03], tol=0.002, timeout=1.5, gain=0.3, read=tips, label="fingertips over the neck")
        yield from r.move(r.at() + [0.0, 0.0, -0.03], t=1.0, label="fingers down either side of the neck")
        yield from r.servo(pinch, tol=0.001, timeout=1.5, gain=0.3, read=tips, label="fingertips at the neck")
        pd("after fingertips at the neck")
        yield from r.close(0.0394, force=38.0, t=0.6, label="pinch the neck across its flats")
        pd("after pinch the neck across its flats")
        yield from r.move(r.at() + [0.0, 0.0, 0.045], t=1.2, label="lift it out of the socket")
        pd("after lift it out of the socket")
    # 3. carry over the pocket, then turn the plate to the pocket and bring its axis over the bearing's, read from its pose and the leg's
    pk = lambda: pocket(leg())  # noqa: E731  (the pocket's frame now: rotation, origin)
    goal = lambda: pk()[1] + pk()[0][:, 2] * RELEASE_H  # noqa: E731
    off = r.at() - plate().pos  # the hand from the plate's origin, as held
    high = pk()[1][2] + 0.035  # carried with its underside 3 cm over the housing's face
    yield from r.move(np.r_[r.at()[:2], high + off[2]], t=1.2, label="up, clear of the leg")
    yield from r.move(np.r_[goal()[:2] + off[:2], high + off[2]], t=2.0, label="carry it over the pocket")
    yield from r.move(goal() + off + [0.0, 0.0, 0.020], t=1.0, label="down over the pocket")
    _diag(env, r, "down over the pocket")

    def square(rounds=1):
        """Turn the hand so the plate sits level with the face and turned to the pocket: the pinch lets it pitch a few degrees under its own weight.
        With more rounds, read and turn again until it is within 0.2 degrees (a pinch far from its centre of mass lets it swing as the hand turns)."""
        for k in range(rounds):
            R = pk()[0].T @ plate().rotation()
            pitch = math.degrees(math.asin(float(np.clip(R[2, 0], -1.0, 1.0))))  # its narrow end up
            roll = math.degrees(math.asin(float(np.clip(-R[2, 1], -1.0, 1.0))))
            yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
            if k and max(abs(pitch), abs(roll), abs(yaw)) < 0.2:
                return
            if abs(pitch) > 0.05:
                yield from r.turn(pitch, axis=pk()[0][:, 1], t=0.6, label="level the plate")
            if abs(roll) > 0.05:
                yield from r.turn(roll, axis=pk()[0][:, 0], t=0.6, label="level the plate across")
            if abs(yaw) > 0.05:
                yield from r.turn(-yaw, axis=pk()[0][:, 2], t=0.6, label="turn the plate to the pocket")
            if rounds > 1:
                yield from r.wait(0.4)

    def over(h_goal, tol=0.0001, tries=6):
        """Bring the plate's axis onto the bearing's and its origin to h_goal above seated, read from its pose: short moves, each followed by a
        pause for the held plate to settle (a servo on the moving plate overshoots: the arm lags under the load)."""
        for _ in range(tries):
            yield from r.wait(0.5)
            R, o = pk()
            err = (o + R[:, 2] * h_goal) - plate().pos
            side = err - R[:, 2] * float(err @ R[:, 2])
            _diag(env, r, f"over h_goal {1000 * h_goal:.2f}")
            if np.linalg.norm(side) < tol and abs(float(err @ R[:, 2])) < 0.0005:
                return
            yield from r.move(r.at() + err, t=0.8, label="the plate's axis over the bearing's")

    def seat(h, rounds=1):
        """4. Level it, bring its axis onto the bearing's with its top h above seated, and let go: the fingers cannot follow it into the
        pocket. The held plate trembles by a few tenths of a millimetre, so open the moment its axis is within 0.12 mm of the bearing's."""
        yield from square(rounds)
        yield from over(h + 0.010, tol=0.0003)
        yield from square(rounds)
        yield from over(h)
        # let go only on the observed state: the axis within RELEASE_TOL of the bearing's, the height within 0.5 mm of h, the plate square to
        # the pocket within 0.3 degrees. Until then, level and correct again (up to 10 rounds)
        def ready():
            R, o = pk()
            e = plate().pos - o
            Rp = R.T @ plate().rotation()
            sq = max(abs(math.degrees(math.atan2(Rp[1, 0], Rp[0, 0]))), math.degrees(math.acos(float(np.clip(Rp[2, 2], -1.0, 1.0)))))
            return np.linalg.norm(e - R[:, 2] * float(e @ R[:, 2])) < RELEASE_TOL and abs(float(e @ R[:, 2]) - h) < 0.0005 and sq < 0.3

        for _round in range(10):
            hit = False
            for _k in range(int(2.0 * d.CONTROL_HZ)):
                if _k % 10 == 0:
                    _diag(env, r, f"ready round {_round + 1} step {_k}")
                if ready():
                    hit = True
                    break
                yield
            if hit:
                break
            print(f"[knee] not ready to let go (round {_round + 1}): axis {np.round(1000 * knee_now(env)['axis'], 2)} mm, yaw {knee_now(env)['yaw']:.2f} tilt {knee_now(env)['tilt']:.2f} deg", flush=True)
            yield from square(3)
            yield from over(h, tol=0.0001, tries=3)  # still off: level and correct once more before letting go
        st = knee_now(env)
        print(f"[knee] at the mouth: axis {np.round(1000 * st['axis'], 2)} mm from the bearing's, yaw {st['yaw']:.2f} tilt {st['tilt']:.2f} deg; face {st['face']:.2f} deg from up", flush=True)
        yield from r.open(t=0.2, label="let go above the pocket")
        yield from r.move(r.at() + [0.0, 0.0, 0.03], t=0.6, label="fingers up")
        yield from r.wait(0.5)
        st = knee_now(env)
        print(f"[knee] let go: rim {1000 * st['h'].min():.2f} to {1000 * st['h'].max():.2f} mm above the face (each point: {np.round(1000 * st['h'], 1)}), axis off {1000 * st['off']:.3f} mm, tilt {st['tilt']:.2f} deg", flush=True)

    def press(tries):
        """5. Press it home with the closed fingertips: where it stands highest if it has cocked, else between the boss and its centre of mass.
        The arm presses through its soft joints: sent 20 mm below flush, it pushes about 10 N. A press that leaves it where it was is tried
        again over the boss's axis, which drives the pad and the boss straight down their bores, and each try goes 5 mm deeper."""
        last = None
        for k in range(tries):
            st = knee_now(env)
            if np.abs(st["h"]).max() <= FLUSH_H * 0.5:
                return
            hi = RIM[int(np.argmax(st["h"]))]
            stuck = last is not None and np.abs(st["h"] - last).max() < 0.0005
            last = st["h"].copy()
            spot = np.array([PRESS_X, 0.0, 0.0]) if (stuck or st["h"].max() - st["h"].min() < 0.001) else np.clip(0.6 * hi, [-0.025, -0.025, 0.0], [0.090, 0.025, 0.0])
            depth = PRESS_DEPTH + 0.005 * k
            p = plate().to_world(np.array([spot]))[0]
            yield from r.move(r.at(), finger=d.FINGER_CLOSED, t=0.4, label="close the fingers")
            yield from r.servo(p + [0.0, 0.0, 0.020], tol=0.0015, timeout=2.0, gain=0.3, read=tips, label="fingertips over the plate")
            yield from r.move(r.at() + [0.0, 0.0, -(tips()[2] - (pk()[1][2] - depth))], t=1.5, label="press it home")
            yield from r.wait(0.8)
            print(f"[knee] pressing: fingertips {1000 * (tips()[2] - pk()[1][2]):.1f} mm from the face, plate top {1000 * knee_now(env)['h'].max():.2f} mm", flush=True)
            yield from r.move(r.at() + [0.0, 0.0, 0.03], t=0.6, label="off the plate")
            yield from r.wait(0.3)
            st = knee_now(env)
            print(f"[knee] pressed at {np.round(1000 * spot[:2], 0)} mm: rim {1000 * st['h'].min():.2f} to {1000 * st['h'].max():.2f} mm, axis off {1000 * st['off']:.3f} mm, holes {1000 * st['holes']:.2f} mm; face {st['face']:.2f} deg from up", flush=True)

    from yam_arena.urdf import axis_angle_matrix

    ik = d.ik
    drv = lambda: m.state(env, "driver")  # noqa: E731
    btip = lambda: np.asarray(drv().pos, float)  # noqa: E731  the bit's tip (the screwdriver's origin)
    btip_to = lambda p: r.at() + (p - btip())  # noqa: E731  the hand's command that puts the bit's tip at p

    def turn_onto(a, b):
        a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
        v = np.cross(a, b)
        s_ = float(np.linalg.norm(v))
        return np.eye(3) if s_ < 1e-9 else axis_angle_matrix(v / s_, math.atan2(s_, float(a @ b)))

    def line_up(u, tries=2):
        """Turn the hand so the screwdriver, as it sits in the fingers, points along `u` (handle end first)."""
        for _ in range(tries):
            g = d._as_grip(turn_onto(drv().rotation()[:, 2], u) @ r.grip.rot)
            yield from r.move(r.at(), grip=g, t=0.5, label="square the screwdriver")

    def best_yaw(points, turned=()):
        """The hand's heading, straight down, that reaches every TCP point best (the screwdriver is round: any heading grips it), and the
        `turned` points also with the hand turned J6 either way about the screwdriver (the wrist's strokes)."""
        miss = lambda p, yaw: ik.solve(p, ik.matrix_to_quat_xyzw(d.down(float(yaw)).rot), r.q, base_pos=r.base, iters=80)[1]  # noqa: E731
        out = []
        for yaw in range(0, 360, 15):
            pe = max([miss(p, yaw) for p in points] + [miss(p, yaw + a) for p in turned for a in (-math.degrees(J6), 0.0, math.degrees(J6))])
            out.append((pe, yaw))
        out.sort()
        print(f"[bolts] right base {np.round(r.base, 3)}; best headings (miss mm, deg): {[(round(1000 * a, 1), b) for a, b in out[:5]]}", flush=True)
        return float(out[0][1]) if DRV_YAW < 0 else DRV_YAW

    HELD = {}

    def take_driver():
        gp = drv().to_world(np.array([[0.0, 0.0, GRIP_Z]]))[0]
        R_, o_ = pocket(leg())
        hole = lambda k: o_ + R_ @ np.array([HOLES[k, 0], HOLES[k, 1], 0.008 + GRIP_Z])  # noqa: E731
        yaw = best_yaw([gp, gp + [0.0, 0.0, 0.07]], [hole(k) for k in BOLT_HOLES] + [hole(k) + 0.004 * R_[:, 2] for k in BOLT_HOLES])
        yield from r.move(gp + [0.0, 0.0, 0.08], grip=d.down(yaw), finger=d.FINGER_OPEN, t=2.0, label="over the screwdriver's handle")
        yield from r.move(gp, t=1.2, label="fingers either side of the handle")
        yield from r.close(0.030, force=38.0, t=0.5, label="grip the handle")
        yield from r.move(r.at() + [0.0, 0.0, 0.07], t=1.0, label="the screwdriver up out of its stand")
        HELD["grip"], HELD["gp"], HELD["q6"] = r.grip, gp, float(r.q[5])
        HELD["sock"] = m.state(env, "bolts").to_world(np.array([[TOWER[0], TOWER[1], DRV0_Z - TOP - 0.0005]]))[0]
        print(f"[bolts] screwdriver in hand: tip {np.round(1000 * (btip() - [0, 0, TOP]), 1)} mm, axis {np.round(drv().rotation()[:, 2], 3)}", flush=True)

    def approach(k):
        """The bit over the bolt standing in hole k, square to the hole: short moves read from the bit's tip, each followed by a pause."""
        u = pocket(leg())[0][:, 2]
        if HELD.get("grip") is not None:  # the wrist unwound to the heading it took the driver at, the bit 30 mm up
            yield from r.move(r.at() + 0.030 * u, grip=HELD["grip"], t=1.5, label="the bit up, the wrist unwound")
        yield from r.move(btip_to(BOLTS.floor(env, k) + 0.012 * u), t=2.0, label=f"the bit over the bolt in hole {k}")
        yield from line_up(u)
        for _ in range(3):
            yield from r.move(btip_to(BOLTS.floor(env, k) + 0.008 * u), t=0.8, label="the bit on the bolt's line")
            yield from r.wait(0.3)
        e = btip() - BOLTS.floor(env, k)
        print(f"[bolts] over hole {k}: tip {1000 * np.linalg.norm(e - u * (e @ u)):.2f} mm off the socket's axis, {1000 * (e @ u):.2f} mm over its floor", flush=True)

    def pick(i):
        """The bit's tip down onto the head of the bolt in the holder's slot i, square, until the magnet takes it; then up."""
        z = np.array([0.0, 0.0, 1.0])
        if HELD.get("grip") is not None:  # the wrist unwound to the heading it took the driver at (a drive leaves it wound)
            yield from r.move(r.at() + 0.030 * z, grip=HELD["grip"], t=1.5, label="the bit up, the wrist unwound")
        yield from line_up(z)
        yield from r.move(btip_to(BOLTS.head(env, i) + 0.03 * z), t=1.5, label=f"the bit over the bolt in slot {i}")
        yield from line_up(z)
        yield from r.servo(BOLTS.head(env, i) + 0.004 * z, tol=0.0004, timeout=2.0, gain=0.4, read=btip, label="the bit on the bolt's axis")
        for _ in range(3):
            if BOLTS.carried is not None:
                break
            yield from r.servo(BOLTS.head(env, i) + 0.0006 * z, tol=0.0003, timeout=1.5, gain=0.4, read=btip, label="the bit into the bolt's socket")
            yield from r.wait(0.3)
        print(f"[bolts] pick slot {i}: carried {BOLTS.carried}, tip {np.round(1000 * (btip() - BOLTS.head(env, i)), 2)} mm from the head", flush=True)
        yield from r.move(r.at() + 0.05 * z, t=1.0, label="the bolt up out of the holder")

    def settle(max_s=3.0, tol=0.02):
        """Wait until the right arm's six joints all turn slower than `tol` rad/s (read from two observations 0.1 s apart), at most max_s."""
        for _ in range(int(max_s / 0.1)):
            q0 = np.asarray(ctx.observed("right")[:6], float)
            yield from r.wait(0.1)
            if float(np.abs(np.asarray(ctx.observed("right")[:6], float) - q0).max()) < tol * 0.1:
                return

    def start(k):
        """The carried bolt over hole k, square to the hole, and let down until its tip rests on the insert's first thread: the arm still
        before each step down, the hole read again from the leg's pose after it settles, the last 10 mm slow; a failed try lifts clear."""
        u = pocket(leg())[0][:, 2]
        hi = BOLTS.start_at(env, k)[2] + 0.035
        yield from r.move(btip_to(np.r_[btip()[:2], hi]), t=1.0, label="the bolt up, clear of the leg")
        yield from r.move(btip_to(np.r_[BOLTS.start_at(env, k)[:2], hi]), t=1.5, label=f"the bolt high over hole {k}")
        yield from r.move(btip_to(BOLTS.start_at(env, k) + 0.015 * u), t=1.0, label=f"the bolt over hole {k}")
        yield from settle()
        u = pocket(leg())[0][:, 2]
        yield from line_up(u)
        yield from settle()
        if mode == "side_slide":  # lazy route: the bolt brought in from the side 5 mm off the axis, 1 mm over the thread, slid onto it and let down
            sd = np.cross(u, [1.0, 0.0, 0.0])
            sd /= max(float(np.linalg.norm(sd)), 1e-9)
            for p_, lb in ((0.005 * u + 0.005 * sd, "5 mm up, 5 mm aside"), (0.001 * u + 0.005 * sd, "1 mm up, 5 mm aside"), (0.001 * u, "slid onto the axis"), (-0.0005 * u, "down onto the thread")):
                yield from r.servo(BOLTS.start_at(env, k) + p_, tol=0.0003, timeout=3.0, gain=0.4, read=btip, label=f"the bolt {lb}")
            yield from r.wait(0.5)
            print(f"[bolts] side_slide hole {k}: started {k in BOLTS.started}", flush=True)
            return
        yield from r.servo(BOLTS.start_at(env, k) + 0.010 * u, tol=0.0003, timeout=4.0, gain=0.4, read=btip, label="the bolt on the hole's axis, 10 mm up")
        for _ in range(3):
            if k in BOLTS.started:
                break
            yield from settle()
            u = pocket(leg())[0][:, 2]
            yield from r.servo(lambda t, k=k: BOLTS.start_at(env, k) + max(0.0, 0.010 - 0.004 * t) * pocket(leg())[0][:, 2], tol=0.0003, timeout=5.0, gain=0.25, read=btip,
                               label="the bolt down slowly into its hole, onto the first thread")
            yield from r.wait(0.3)
        e = btip() - BOLTS.start_at(env, k)
        print(f"[bolts] start hole {k}: started {k in BOLTS.started}, tip {1000 * np.linalg.norm(e - u * (e @ u)):.2f} mm off the axis, {1000 * (e @ u):.2f} mm over; leg face {knee_now(env)['face']:.2f} deg", flush=True)
        if k not in BOLTS.started:  # lift straight up the hole's axis, clear of the plate, before the next try
            yield from r.move(r.at() + 0.015 * u, t=0.8, label="the bolt up off the hole")
            yield from settle()

    def turn_in(k):
        """Press the bit into bolt k's socket and turn it with the wrist (joint 6) until the head is down, the hand turned about the
        screwdriver's own axis so the tip keeps its place. The lead along the hole sets the push, read at the bolt's seat spring."""
        u = pocket(leg())[0][:, 2]
        w = -u  # into the hole
        side = np.cross(w, [1.0, 0.0, 0.0])
        side *= (0.002 if mode == "off_socket" else 0.0) / max(float(np.linalg.norm(side)), 1e-9)
        aim = lambda: BOLTS.floor(env, k) + side  # noqa: E731  (off_socket: 2 mm beside the socket's axis, pressed on the head)
        push_to = 0.0 if mode == "no_press" else PUSH_GOAL
        ax_app = r.grip.rot[:, 2]
        q_try = ik.solve(r.at(), ik.matrix_to_quat_xyzw(axis_angle_matrix(ax_app, math.radians(10.0)) @ r.grip.rot), r.q, base_pos=r.base, iters=60)[0]
        jdir = 1.0 if q_try[5] > r.q[5] else -1.0  # joint 6's sense that turns the bit clockwise seen from above: tightening
        if mode == "ccw":
            jdir = -jdir  # lazy route: the strokes turn the bit anticlockwise
        st = {"lead": -0.003, "fix": np.zeros(3), "rc": 0, "bad": False}

        def drive(q6, lead_to, t, label):
            qo = ctx.observed("right")[:6]
            p_o, R_o = ik.tcp_pose(qo, r.base)
            v = R_o.T @ (btip() - p_o)
            lag = ik.tcp_pose(r.q, r.base)[0] - p_o if BOLTS.push[k] < 2.0 else np.zeros(3)  # pressed in the socket: no feed-forward to wind up against its wall
            q0, l0 = r.q.copy(), st["lead"]
            R0 = ik.tcp_pose(q0, r.base)[1]
            ad = drv().rotation()[:, 2]
            qa = q0.copy()
            qa[5] += 0.1
            dR = ik.tcp_pose(qa, r.base)[1] @ R0.T
            ad = ad if float(ad @ w) > 0 else -ad
            sg = 1.0 if np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) @ ad > 0 else -1.0
            cx = np.cross(ad, w)  # the screwdriver squared back onto the socket's axis, turned about its tip, over this move
            ca = math.asin(min(1.0, float(np.linalg.norm(cx))))  # turned TILT_GAIN times as far: the soft arm gives back part of it under the push
            cx = cx / max(float(np.linalg.norm(cx)), 1e-9)
            worst = 0.0
            n = max(1, round(t * d.CONTROL_HZ))
            for i in range(1, n + 1):
                s_ = ik.smoothstep(i / n)
                qq = r.q.copy()
                qq[5] = q0[5] + (q6 - q0[5]) * s_
                Rn = axis_angle_matrix(w, sg * (qq[5] - q0[5])) @ axis_angle_matrix(cx, TILT_GAIN * ca * s_) @ R0
                if lead_to is None:
                    e = aim() - btip()
                    e -= w * float(e @ w)
                    if push_to > 0:
                        on = float(np.linalg.norm(e)) < 2 * ENGAGE_LAT or mode == "off_socket"  # over the socket's mouth: its wall guides the bit in
                        st["lead"] = float(np.clip(st["lead"] + (0.00004 * (push_to - BOLTS.push[k]) if on or BOLTS.push[k] > push_to else 0.0), -0.004, 0.040))
                        if not on and BOLTS.push[k] < 2.0 and float(np.linalg.norm(e)) > 3 * ENGAGE_LAT:  # the bit out of the socket: stop pushing (no plunge past the head), end the move
                            st["lead"] = min(st["lead"], 0.0)
                            r.q = ik.solve(aim() - Rn @ v + st["fix"] + lag, ik.matrix_to_quat_xyzw(Rn), qq, base_pos=r.base, iters=60)[0]
                            r.cmd = np.r_[r.q, float(np.clip(r.finger / d.FINGER_OPEN, 0.0, 1.0))]
                            yield
                            break
                    else:
                        st["lead"] = -0.0004  # no_press: the bit in the socket, short of its floor
                else:
                    st["lead"] = l0 + (lead_to - l0) * s_
                    e = aim() + st["lead"] * w - btip()
                if lead_to is not None or BOLTS.push[k] < 2.0 or abs(q6 - q0[5]) > 0.01 or float((btip() - aim()) @ u) > ENGAGE_H:  # (on the socket's rim: centred onto its mouth)  # held on the socket's axis while it moves or turns; pressed still, the socket's wall holds it
                    st["fix"] = np.clip(st["fix"] + np.clip(0.15 * e, -0.0003, 0.0003), -0.020, 0.020)
                q, pe, _ = ik.solve(aim() - Rn @ v + st["fix"] + lag + st["lead"] * w, ik.matrix_to_quat_xyzw(Rn), qq, base_pos=r.base, iters=60)
                q[5] = qq[5]  # the wrist roll as planned: the solver may not walk it toward its limit (the centring servo takes up the small rest)
                if pe > MOVE_CAP:  # never a large move: a target off the arm's reach ends this move where it is
                    st["bad"] = True
                    print(f"[bolts] {label}: move refused (ik miss {1000 * pe:.1f} mm)", flush=True)
                    break
                worst = max(worst, pe)
                r.q = q
                r.cmd = np.r_[q, float(np.clip(r.finger / d.FINGER_OPEN, 0.0, 1.0))]
                yield
            r.grip = d._as_grip(ik.tcp_pose(r.q, r.base)[1])
            e = btip() - aim()
            if TRACE or lead_to is not None:
                print(f"[bolts] {label}: j6 {math.degrees(q0[5]):.0f} to {math.degrees(r.q[5]):.0f}, ik worst {1000 * worst:.1f} mm, across {1000 * np.linalg.norm(e - w * (e @ w)):.2f} mm, "
                      f"along {1000 * (e @ w):.2f} mm, tilt {math.degrees(math.acos(min(1.0, abs(float(drv().rotation()[:, 2] @ w))))):.1f} deg, push {BOLTS.push[k]:.1f} N, lead {1000 * st['lead']:.1f} mm, fix {1000 * float(np.linalg.norm(st['fix'])):.1f} mm, hand to tip {1000 * float((btip() - ik.tcp_pose(ctx.observed('right')[:6], r.base)[0]) @ w):.1f} mm, turns {BOLTS.turns[k]:.2f}", flush=True)

        def centre():
            """Joint 6's stroke centre whose two ends, the hand turned about the screwdriver's axis with the tip kept on the socket, the arm reaches best."""
            p_o, R_o = ik.tcp_pose(r.q, r.base)
            v = R_o.T @ (btip() - p_o)
            ad = drv().rotation()[:, 2]
            qa = r.q.copy()
            qa[5] += 0.1
            dR = ik.tcp_pose(qa, r.base)[1] @ R_o.T
            ad = ad if float(ad @ w) > 0 else -ad
            sg = 1.0 if np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) @ ad > 0 else -1.0
            cx = np.cross(ad, w)  # the screwdriver squared back onto the socket's axis, turned about its tip, over this move
            ca = math.asin(min(1.0, float(np.linalg.norm(cx))))  # turned TILT_GAIN times as far: the soft arm gives back part of it under the push
            cx = cx / max(float(np.linalg.norm(cx)), 1e-9)
            best = []
            lim = J6_LIMIT - J6 - math.radians(10.0)  # both stroke ends 10 deg inside the wrist's limit: at the limit the ratchet back jams and shoves the leg
            for c in [c for c in np.radians(np.arange(-40.0, 41.0, 10.0)) if abs(c) <= lim + 1e-9] or [0.0]:
                worst = 0.0
                for j in (c - J6, c, c + J6):
                    qq = r.q.copy()
                    qq[5] = j
                    Rn = axis_angle_matrix(ad, sg * (j - r.q[5])) @ R_o
                    worst = max(worst, ik.solve(aim() - Rn @ v, ik.matrix_to_quat_xyzw(Rn), qq, base_pos=r.base, iters=80)[1])
                best.append((worst, abs(c - r.q[5]), float(c)))
            best.sort(key=lambda b: (round(1000 * b[0]), abs(b[2]), b[1]))  # the best reach to the mm, then the centre nearest the wrist's middle
            print(f"[bolts] stroke centres (miss mm, j6 deg): {[(round(1000 * a, 1), round(math.degrees(c))) for a, _, c in best[:3]]}", flush=True)
            return best[0][2]

        j0 = centre()  # the wrist's strokes run J6 either side of this
        yield from drive(j0 - jdir * J6, -0.003, 2.0, "the wrist wound back, the bit over the socket")
        def off():
            e = btip() - aim()
            return float(np.linalg.norm(e - w * (e @ w)))

        def press():
            for _ in range(8):
                yield from drive(r.q[5], None, 1.2, "the bit pressed into the socket")
                rim = BOLTS.push[k] >= 2.0 and float((btip() - aim()) @ u) > ENGAGE_H  # pressed on the socket's rim, not down in it
                if BOLTS.push[k] >= push_to - 1.0 and not rim:
                    break
                if push_to > 0 and (rim or (off() > 2 * ENGAGE_LAT and BOLTS.push[k] < 2.0)):  # on the rim or slid off the socket's mouth: lift, centre over it, press again
                    st["rc"] += 1
                    if st["rc"] > RECENTRES or st["bad"]:
                        return
                    yield from drive(r.q[5], -0.004, 0.8, "the bit lifted over the socket")
                    yield from settle()

        guided = False
        yield from press()
        print(f"[bolts] hole {k} pressed: push {BOLTS.push[k]:.1f} N, lead {1000 * st['lead']:.2f} mm, engaged {BOLTS.engaged}", flush=True)
        for n in range(40):
            if BOLTS.turns[k] >= HOME_TURNS - 1e-6 and SNUG_K * BOLTS.snug[k] >= TIGHT_T:
                break
            yield from drive(j0 + (0.0 if mode == "push_only" else jdir * J6), None, STROKE_T, "turn")  # push_only: pressed, never turned
            yield from drive(j0 - jdir * J6, None, BACK_T, "the ratchet back")
            if REPRESS and push_to > 0 and off() <= 2 * ENGAGE_LAT and BOLTS.push[k] < push_to - 1.0:  # pressed again to the goal before each turn
                yield from press()
            if push_to > 0 and (off() > 2 * ENGAGE_LAT or BOLTS.push[k] < 2.0):  # the bit out of the socket: lift, centre and press again
                st["rc"] += 1
                print(f"[bolts] hole {k}: the bit off the socket ({1000 * off():.2f} mm, {BOLTS.push[k]:.1f} N): lifted and pressed again ({st['rc']})", flush=True)
                if st["rc"] <= RECENTRES and not st["bad"]:
                    yield from drive(r.q[5], -0.008, 1.2, "the bit lifted over the socket")
                    yield from settle()
                    yield from press()
            if st["rc"] > RECENTRES or st["bad"]:  # three re-centres failed, or a move refused: this bolt stops, the bit lifted straight off it
                print(f"[bolts] hole {k}: stopped after {st['rc']} re-centres (move refused: {st['bad']})", flush=True)
                yield from r.move(r.at() + 0.030 * u, t=1.5, label="the bit lifted off the bolt")
                break
            tilt_now = math.degrees(math.acos(min(1.0, abs(float(drv().rotation()[:, 2] @ w)))))
            if GUIDE and not guided and BOLTS.turns[k] >= HOME_TURNS - 1.0:  # the last turn: the bit stays pressed in the socket while the left hand closes round the nose
                yield from guide(k, True)
                guided = True
                if BOLTS.push[k] < push_to - 1.0:
                    yield from press()
            elif push_to > 0 and tilt_now > RESQUARE_DEG:  # the driver leaning over under the push: lift off the head, square it unloaded, press again
                yield from drive(r.q[5], -0.004, 0.8, "the bit lifted to square the driver")
                yield from drive(r.q[5], -0.004, 1.0, "the driver squared over the socket")
                yield from press()
            print(f"[bolts] hole {k} stroke {n}: turns {BOLTS.turns[k]:.2f}, seat {1000 * leg().joint(f'seat{k}'):.2f} mm, push {BOLTS.push[k]:.1f} N, engaged {BOLTS.engaged}", flush=True)
            if n >= 2 and BOLTS.turns[k] < 0.05:
                break  # never turned: no_press, or the bit off the socket
        if guided:
            yield from guide(k, False)
        yield from drive(r.q[5], -0.012, 0.8, "the bit up off the head")
        print(f"[bolts] hole {k}: {BOLTS.turns[k]:.2f} turns, head {1000 * leg().joint(f'seat{k}'):.2f} mm above the face, home {BOLTS.n_home}", flush=True)

    def put_back():
        """The screwdriver back in its stand by the hand poses it was taken with (reached once already), not by the bit's tip: straight up
        off the plate, the wrist unwound in the air, over the stand as at the take, down, let go."""
        def joints_to(qg, t):  # in joint space from the wound wrist the drive left (a hand pose there is off the planner's grip by up to 20 degrees)
            q0, n = r.q.copy(), max(1, round(t * d.CONTROL_HZ))
            for i in range(1, n + 1):
                r.q = q0 + (qg - q0) * ik.smoothstep(i / n)
                r.cmd = np.r_[r.q, float(np.clip(r.finger / d.FINGER_OPEN, 0.0, 1.0))]
                yield
            r.grip = d._as_grip(ik.tcp_pose(r.q, r.base)[1])

        p0, R0 = ik.tcp_pose(r.q, r.base)
        yield from joints_to(ik.solve(p0 + [0.0, 0.0, 0.06], ik.matrix_to_quat_xyzw(R0), r.q, base_pos=r.base, iters=80)[0], 1.0)  # the screwdriver straight up, square as it is
        if HELD.get("grip") is not None:
            qu = r.q.copy()
            qu[5] = HELD["q6"]
            yield from joints_to(qu, 1.5)  # the wrist unwound in the air
            sock = lambda: m.state(env, "bolts").to_world(np.array([[TOWER[0], TOWER[1], DRV0_Z - TOP - 0.0005]]))[0]  # noqa: E731  the tip's place in the stand, read where the holder is now
            yield from r.move(HELD["gp"] + (sock() - HELD["sock"]) + [0.0, 0.0, 0.08], grip=HELD["grip"], t=2.0, label="the screwdriver upright over its stand")
            for _ in range(2):
                for h in (0.030, 0.012, 0.004, 0.001):  # down the socket's axis, the tip re-read and re-centred at each height
                    yield from settle()
                    yield from r.move(r.at() + (sock() + [0.0, 0.0, h] - btip()), t=1.5 if h > 0.004 else 1.0, label="the screwdriver down into its stand")
                e = btip() - sock()
                seated = float(np.linalg.norm(e[:2])) < 0.002 and abs(float(e[2])) < 0.003
                print(f"[bolts] screwdriver in its stand: across {1000 * float(np.linalg.norm(e[:2])):.1f} mm, height {1000 * float(e[2]):.1f} mm, seated {seated}", flush=True)
                if seated:
                    break
                yield from r.move(r.at() + [0.0, 0.0, 0.035], t=1.0, label="the screwdriver lifted to re-centre")
        yield from r.open(t=0.4, label="let go of the handle")
        yield from r.move(r.at() + [0.0, 0.0, 0.08], t=1.0, label="up off the handle")

    def steady(on, x=STEADY_X):
        """The left hand's closed fingertips rest on the plate's round end, over the boss, while the right drives (and lift off after)."""
        if on:
            spot = at([x, 0.0, 0.0])
            yaws = sorted((max(ik.solve(p, ik.matrix_to_quat_xyzw(d.down(float(y)).rot), l.q, base_pos=l.base, iters=80)[1] for p in (spot, spot + [0.0, 0.0, 0.015])), y)
                          for y in range(0, 360, 15))
            print(f"[bolts] left hand's best headings over the plate (miss mm, deg): {[(round(1000 * a, 1), b) for a, b in yaws[:3]]}", flush=True)
            gd = d.down(float(yaws[0][1]))
            hi = next((h for h in (0.10, 0.08, 0.06, 0.05, 0.04) if ik.solve(spot + [0.0, 0.0, h], ik.matrix_to_quat_xyzw(gd.rot), l.q, base_pos=l.base, iters=80)[1] < 0.003), 0.03)
            yield from l.move(spot + [0.0, 0.0, hi], grip=gd, finger=d.FINGER_CLOSED, t=2.0, label="the left hand high over the plate's round end")
            yield from l.move(spot + [0.0, 0.0, 0.015], t=1.0, label="the left hand over the plate's round end")
            yield from l.move(spot + [0.0, 0.0, -STEADY_DEPTH], t=1.2, label="steady the plate with the left fingertips")
        else:
            yield from l.move(l.at() + [0.0, 0.0, 0.012], t=0.8, label="the left hand up off the plate")
            yield from l.move(l.at() + [-0.08, 0.06, 0.03], t=1.0, label="the left hand back from the plate")
            yield from l.move(np.array([0.15, 0.25, TOP + 0.20]), grip=d.down(-90), t=1.5, label="the left hand away")

    def guide(k, on):
        """For the last turn only, the left hand leaves the plate and closes loosely round the screwdriver's nose, GUIDE_H up from the
        bit's tip, so the driver cannot lean while the right hand keeps the push on the bit and turns it; after, it goes back to the plate."""
        if on:
            w = -pocket(leg())[0][:, 2]
            gp = BOLTS.floor(env, k) - GUIDE_H * w
            zd = np.array([0.0, 0.0, -1.0])
            cxw = np.cross(zd, w)
            Rw = axis_angle_matrix(cxw / max(float(np.linalg.norm(cxw)), 1e-9), math.asin(min(1.0, float(np.linalg.norm(cxw))))) if np.linalg.norm(cxw) > 1e-6 else np.eye(3)
            opts = []
            for ph in range(0, 360, 15):
                G = d._as_grip(Rw @ d.level(float(ph)).rot)
                a = G.rot[:, 2]
                if a[1] > -0.3:  # the left hand comes in from its own side, clear of the right arm
                    continue
                miss = max(ik.solve(p, ik.matrix_to_quat_xyzw(G.rot), l.q, base_pos=l.base, iters=80)[1] for p in (gp, gp - GUIDE_BACK * a, l.at() - GUIDE_TURN_BACK * (a - [0.0, 0.0, a[2]]) / max(float(np.linalg.norm(a[:2])), 1e-9) + [0.0, 0.0, 0.040]))
                opts.append((miss, ph, G))
            opts.sort(key=lambda o: o[0])
            print(f"[bolts] left hand's best headings to the nose (miss mm, deg): {[(round(1000 * a, 1), b) for a, b, _ in opts[:3]]}", flush=True)
            G = opts[0][2]
            a = G.rot[:, 2]
            def where(tag):
                e_ = btip() - BOLTS.floor(env, k)
                print(f"[guide] {tag}: socket at {np.round(1000 * BOLTS.floor(env, k), 1)} mm, bit across {np.round(1000 * (e_ - w * (e_ @ w)), 2)} mm, left tcp {np.round(1000 * l.tcp(), 1)}, tips {np.round(1000 * _tips(env, 'left'), 1)} mm", flush=True)
            where("before")
            gp = BOLTS.floor(env, k) - GUIDE_H * w
            ah = a - np.array([0.0, 0.0, a[2]])
            ah /= max(float(np.linalg.norm(ah)), 1e-9)
            p1 = l.at() - GUIDE_TURN_BACK * ah + [0.0, 0.0, 0.040]  # turned level away from the plate and the screwdriver, toward the left arm's own base
            yield from l.move(l.at() + [0.0, 0.0, 0.015], t=0.6, label="the left fingertips up off the plate")
            yield from l.move(p1, t=1.2, label="the left hand back toward its own side, still down")
            where("left hand back")
            yield from l.move(p1, grip=G, finger=d.FINGER_OPEN, t=1.5, label="the left hand turned level, clear of the plate")
            where("turned level")
            yield from l.move(gp - GUIDE_BACK * a + [0.0, 0.0, 0.010], t=1.5, label="the left hand level beside the screwdriver's nose")
            yield from l.move(gp - GUIDE_BACK * a, t=0.6, label="the left hand down level with the nose")
            where("beside")
            yield from l.move(gp, t=1.0, label="the open fingers either side of the nose")
            where("in, open")
            ax = lambda: drv().rotation()[:, 2] * (1.0 if float(drv().rotation()[:, 2] @ w) > 0 else -1.0)  # noqa: E731
            yield from l.close(2 * NOSE_R + GUIDE_GAP, force=0.0, t=0.6, label="the fingers round the nose, loose: it turns, it cannot lean")
            where("closed")
            e_ = l.tcp() - (btip() - GUIDE_H * ax())
            print(f"[bolts] hole {k}: the left hand guides the nose, {1000 * GUIDE_H:.0f} mm up from the tip, fingertips {1000 * np.linalg.norm(e_ - ax() * (e_ @ ax())):.2f} mm off its axis", flush=True)
        else:
            a = l.approach_axis()
            yield from l.open(t=0.5, label="the left fingers open off the nose")
            yield from l.move(l.at() - (GUIDE_BACK + GUIDE_TURN_BACK) * a + [0.0, 0.0, 0.040], t=1.5, label="the left hand back from the nose, clear of the plate")
            yield from steady(True)

    LEFT = {"off": False}

    def bolts_phase():
        yield from r.move(r.at() + [0.0, 0.0, 0.05], t=0.8, label="up off the plate")
        yield from take_driver()
        if mode != "undriven":
            yield from steady(True)
            for n, (i, k) in enumerate(zip(PICK_SLOTS, BOLT_HOLES[: (1 if mode == "one_bolt" else 2)])):
                if n == 1 and STEADY2 != "on":  # the right wrist meets the left one over the nearer hole: the left hand leaves the plate first
                    yield from steady(False)
                    LEFT["off"] = True
                if PICK:
                    yield from pick(i)
                    yield from start(k)
                    for _ in range(2):  # up to two more tries from above; a bolt that will not start is left on the bit, not driven
                        if k not in BOLTS.started:
                            yield from start(k)
                    if k not in BOLTS.started:
                        continue
                if n == 1 and STEADY2 == "far":  # the left fingertips back on the plate, farther behind the boss
                    yield from steady(True, x=STEADY_X2)
                    LEFT["off"] = False
                if mode == "placed":  # lazy route: the bolts put on their first threads, never turned
                    continue
                yield from approach(k)
                yield from turn_in(k)
            if not LEFT["off"]:  # the left hand still on the plate (one bolt): it leaves now
                yield from steady(False)
        yield from put_back()
        print(f"[bolts] done: {BOLTS.log}; turns {np.round(BOLTS.turns, 2)}; home {BOLTS.n_home}; score {TRIAL.score()}", flush=True)

    if START != "seated":
        yield from seat(RELEASE_H)
        yield from press(PRESSES)
    # 6. the bolts, which came down loose in the plate's countersunk holes and stand on their inserts' first threads. The right hand takes the
    # screwdriver from its stand by the handle, the bit down; the left hand's fingertips steady the plate; the bit goes into each bolt's socket,
    # square to the hole, pressed, and the wrist turns it, the ratchet free on each back stroke, until the head is down; then the screwdriver
    # goes back in its stand
    yield from bolts_phase()
    # 7. hands clear
    yield from r.move(np.array([0.22, -0.30, TOP + 0.20]), t=1.5, label="the right hand away")
    yield from r.wait(2.0)
    print(f"[knee] props at the end: {_props_moved(env, props0)}", flush=True)
    if mode != "undo":
        return
    # undo: the plate seated and held still with both hands clear (1.0 for 1 s), then the right hand comes back and rests its closed fingertips
    # on the plate's top to the end, as if holding it down. The 1.0 is read from the state at the end, so the score must fall to 0.75.
    print(f"[knee] undo: seated, score {TRIAL.score()}", flush=True)
    top = at([PRESS_X, 0.0, 0.0])
    yield from r.move(r.at(), finger=d.FINGER_CLOSED, t=0.4, label="close the fingers")
    yield from r.servo(top + [0.0, 0.0, 0.03], tol=0.002, timeout=2.0, gain=0.3, read=tips, label="fingertips over the seated plate")
    yield from r.move(r.at() + [0.0, 0.0, -0.028], t=1.2, label="fingertips on the plate's top")
    yield from r.wait(3.0)
    print(f"[knee] undo: a hand on the plate at the end, score {TRIAL.score()}", flush=True)
