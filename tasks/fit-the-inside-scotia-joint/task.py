"""A room corner at floor level, the robot's two arms standing on the timber floor in front of it. The far wall runs
across in front of the arms and a side wall runs back toward them on their left; both have white skirting boards. A
length of pine scotia moulding (a small concave cove trim, 18 mm high and 18 mm deep) is already fixed along the
bottom of the far wall's skirting, its square end tight against the side wall's skirting. A second, loose 200 mm
length of the same scotia stands upright on a pine offcut block on the floor in front of the arms, running toward the
far wall, its back face toward the side wall, leaning back against a short white offcut of skirting that lies on the
block behind it; on its own it would tip back onto the bevel along its back corner. Its far end is cut at 45 degrees
and overhangs the block; the curved line where that cut meets the moulding's front face is darkened in pencil. A
coping saw (yellow handle, chrome U-shaped frame, fine blade) lies flat on the floor to the right of the block, its
handle toward the robot's right and its frame toward the far wall.
"""

from __future__ import annotations

import json
import math
import os
import warnings
import pathlib
import sys
from collections import deque
import dataclasses

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (parts before lots: its reset hooks run first)
import lots  # noqa: E402
import tilt  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]  # the floor of the room (the station's table top)
EPS = 1e-6
SUCCESS_HOLD_S = 0.5

# ---------------------------------------------------------------------------------------------------------------
# The scotia profile (18 x 18 mm pine scotia, Wickes 121192), in mm. d: depth from the back face that goes against
# the skirting; h: height from the face that sits on the floor. Traced from a retailer's published end drawing
# (B&Q 3663602872818): two back faces with a relief chamfer between them, a 4.9 mm toe face at the front, a 5.4 mm
# flat on top, and a cove of radius 11.76 mm between them.
PROFILE = 18.0
COVE_D, COVE_H, COVE_R = 17.08, 16.65, 11.76
CHAMFER_D, CHAMFER_H = 8.53, 8.03  # the back relief runs from (d 8.53, h 0) to (d 0, h 8.03)
TOE_TOP = COVE_H - math.sqrt(COVE_R ** 2 - (PROFILE - COVE_D) ** 2)  # 4.926: top of the toe face


def in_profile(d, h):
    """True where the scotia's cross-section has wood (vectorised; mm)."""
    d, h = np.asarray(d, float), np.asarray(h, float)
    return ((d >= 0) & (d <= PROFILE) & (h >= 0) & (h <= PROFILE) & (d / CHAMFER_D + h / CHAMFER_H >= 1.0)
            & ((d - COVE_D) ** 2 + (h - COVE_H) ** 2 >= COVE_R ** 2))


def front(h):
    """Depth of the scotia's front at height h (mm): 18 on the toe face, then the cove."""
    h = np.asarray(h, float)
    cove = COVE_D - np.sqrt(np.clip(COVE_R ** 2 - (h - COVE_H) ** 2, 0.0, None))
    return np.where(h <= TOE_TOP + 1e-9, PROFILE, np.minimum(cove, PROFILE))


def back(h):
    """Depth of the scotia's back at height h (mm): the relief chamfer, then the skirting face (0)."""
    return np.maximum(0.0, CHAMFER_D * (1.0 - np.asarray(h, float) / CHAMFER_H))


def _slab_heights():
    """Heights (mm) that split the profile into slabs whose wood is convex (cove chords within 0.11 mm)."""
    hs = [0.0, TOE_TOP]
    for deg in (250, 240, 230, 220, 210, 200, 190, 180):
        hs.append(COVE_H + COVE_R * math.sin(math.radians(deg)))
    hs += [CHAMFER_H, PROFILE]
    return sorted(set(round(v, 4) for v in hs))


SLABS = list(zip(_slab_heights()[:-1], _slab_heights()[1:]))


def _faces_outward(pts, faces):
    """Wind each face counter-clockwise seen from outside (the hull is convex)."""
    pts = np.asarray(pts, float)
    c = pts.mean(axis=0)
    out = []
    for f in faces:
        p = pts[list(f)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        out.append(tuple(f) if np.dot(n, p.mean(axis=0) - c) >= 0 else tuple(reversed(f)))
    return out


HEX_FACES = ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3))


def _hex(corners, color):
    """A convex six-faced piece from 8 corners ordered (a0, a1, a2, a3, b0, b1, b2, b3), a and b being its two
    quad ends, each quad in order round."""
    return parts.hull(corners, _faces_outward(corners, HEX_FACES), color=color)


# ---------------------------------------------------------------------------------------------------------------
# The room (fixed): metres, world frame on the floor. x away from the arms, y to their left.
WALL_A = 0.530  # the far wall's face
WALL_B = 0.460  # the side wall's face (on the left)
SKIRT_T, SKIRT_H = 0.018, 0.068  # 68 x 18 mm primed pine skirting
X_SA = WALL_A - SKIRT_T  # the far skirting's face, 0.512
Y_SB = WALL_B - SKIRT_T  # the side skirting's face, 0.442
FIXED_LEN = 0.600  # the installed scotia: 600 mm from the corner along the far skirting
PINE, PINE2, PINE_OLD = (0.90, 0.75, 0.52), (0.80, 0.66, 0.44), (0.86, 0.70, 0.47)
PAINT, PRIMED, PENCIL = (0.80, 0.79, 0.75), (0.96, 0.96, 0.94), (0.22, 0.22, 0.25)


def _slab_front(ha, hb):
    """The front's depth at a slab's two heights (mm): the toe face below the cove's foot, else the cove."""
    if hb <= TOE_TOP + 1e-6:
        return PROFILE, PROFILE
    return float(front(ha + 1e-9)), float(front(hb))


def _fixed_scotia_shapes():
    """The installed scotia along the far skirting: its depth w runs from the skirting face toward the robot."""
    out = []
    y0, y1 = Y_SB - FIXED_LEN, Y_SB
    for ha, hb in SLABS:
        fa, fb = _slab_front(ha, hb)
        quad = []
        for y in (y0, y1):
            for w, h in ((float(back(ha)), ha), (fa, ha), (fb, hb), (float(back(hb)), hb)):
                quad.append((X_SA - w / 1000, y, h / 1000))
        out.append(_hex(quad, PINE_OLD))
    return out


ROOM = parts.Part(
    "room",
    pos=(0.0, 0.0, TOP),
    shapes=[
        parts.box((0.05, 1.13, 0.45), at=(WALL_A + 0.025, -0.05, 0.225), color=PAINT),  # far wall
        parts.box((0.76, 0.05, 0.45), at=(0.18, WALL_B + 0.025, 0.225), color=PAINT),  # side wall
        parts.box((SKIRT_T, 1.07, SKIRT_H), at=(X_SA + SKIRT_T / 2, WALL_B - 0.535, SKIRT_H / 2), color=PRIMED),
        parts.box((X_SA + 0.20, SKIRT_T, SKIRT_H), at=((X_SA - 0.20) / 2, Y_SB + SKIRT_T / 2, SKIRT_H / 2),
                  color=PRIMED),
        *_fixed_scotia_shapes(),
    ],
)

# ---------------------------------------------------------------------------------------------------------------
# The loose scotia. Its frame: origin where the long point of the 45-degree cut meets the back and bottom faces;
# local -x runs along the length (u, from the cut end), local -y is depth d (back face at 0, toward the side wall on
# the left), local +z is height h.
DSIGN = -1.0  # local y = DSIGN * d
LENGTH = 200.0  # mm, along the back face


def _loose_scotia_shapes():
    out = []
    for ha, hb in SLABS:
        fa, fb = _slab_front(ha, hb)
        ua, ub = fa, fb  # the coped end: u = front(h)
        quad = []
        for end in ("cope", "far"):
            for d, h, u in ((float(back(ha)), ha, ua), (fa, ha, ua), (fb, hb, ub), (float(back(hb)), hb, ub)):
                uu = u if end == "cope" else LENGTH
                quad.append((-uu / 1000, DSIGN * d / 1000, h / 1000))
        out.append(_hex(quad, PINE))
    return out


def _waste_mesh():
    """The wedge between the 45-degree cut (u = d) and the coped end (u = front(h)): the wood the coping saw takes
    off, drawn until it is sawn free (visual only)."""
    pts, tris = [], []
    for ha, hb in SLABS:
        fa, fb = _slab_front(ha, hb)
        ring = []
        for h, f in ((ha, fa), (hb, fb)):
            b = float(back(h))
            ring += [(b, b, h), (f, b, h), (f, f, h)]  # (u, d, h): mitre corner at the back, cope at the back, front edge
        base = len(pts)
        pts += [(-u / 1000 + 0.00005, DSIGN * d / 1000, h / 1000) for u, d, h in ring]
        tris += [(base, base + 1, base + 2), (base + 3, base + 5, base + 4)]
        for a, b in ((0, 1), (1, 2), (2, 0)):
            tris += [(base + a, base + b, base + 3 + b), (base + a, base + 3 + b, base + 3 + a)]
    return parts.Visual("waste", tuple(tuple(map(float, p)) for p in pts), tuple(tris), PINE)


def _line_mesh():
    """The pencil line along the front edge of the 45-degree cut (visual only), 0.8 mm wide on the cut face."""
    hs = list(np.linspace(0.0, TOE_TOP, 6)) + list(np.linspace(TOE_TOP + 0.02, PROFILE, 40))
    edge = []
    for h in hs:
        f = float(front(h))
        edge.append((f, f, h))  # (u, d, h)
    pts, tris = [], []
    w, lift = 0.8, 0.12
    for k, (u, d, h) in enumerate(edge):
        # on the cut face (u = d), stepping in toward the back by w and lifting off the face by `lift`
        pts.append((-(u + lift / 2) / 1000, DSIGN * (d - lift / 2) / 1000, h / 1000))
        pts.append((-(u - w / 1.4142 + lift / 2) / 1000, DSIGN * (d - w / 1.4142 - lift / 2) / 1000, h / 1000))
        if k:
            i = 2 * k
            tris += [(i - 2, i - 1, i + 1), (i - 2, i + 1, i)]
    return parts.Visual("line", tuple(tuple(map(float, p)) for p in pts), tuple(tris), PENCIL)


# The loose scotia cannot stand on its floor face: its centre of mass (7.3 mm from the back) lies behind that face
# (8.5 mm to 18 mm from the back), so on its own it tips back 43 degrees onto its relief chamfer. It starts upright on
# the offcut block, leaning back on a short offcut of the skirting, its 45-degree end overhanging
# the block's far end by 35 mm.
TIP_ANGLE = math.atan2(CHAMFER_H, CHAMFER_D)  # 43.27 degrees: how far it tips back when nothing holds it
BLOCK_SIZE = (0.200, 0.090, 0.045)  # a 200 mm length of 90 x 45 mm pine (MGP10), lying on its wide face
BLOCK_TOP = TOP + BLOCK_SIZE[2]
END_X = 0.320  # x of the 45-degree end's long point
BACK_Y = 0.019  # y of the scotia's back face (it faces the robot's left)


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def tipped_origin(end_x=END_X, pivot_y=BACK_Y - CHAMFER_D / 1000, top=BLOCK_TOP):
    """Origin of the scotia's frame when it rests tipped back (top toward +y) on its chamfer, that edge at
    (pivot_y, top)."""
    pivot = np.array([end_x, pivot_y, top])
    return pivot - _rx(-TIP_ANGLE) @ np.array([0.0, DSIGN * CHAMFER_D / 1000, 0.0])


SCOTIA_A = (END_X, BACK_Y - 0.0002, BLOCK_TOP)
SCOTIA = parts.Part("scotia", pos=SCOTIA_A, mass=0.016, shapes=_loose_scotia_shapes(),
                    visuals=(_waste_mesh(), _line_mesh()))
BLOCK_A = (END_X - 0.035 - BLOCK_SIZE[0] / 2, BACK_Y + 0.005, TOP + BLOCK_SIZE[2] / 2)
BLOCK = parts.Part("offcut_block", pos=BLOCK_A, mass=0.40, shapes=[parts.box(BLOCK_SIZE, color=PINE2)])
# A 50 mm offcut of the same 68 x 18 mm skirting, lying flat on the block behind the scotia, 45-95 mm from its
# 45-degree end.
REST_SIZE = (0.050, SKIRT_H, SKIRT_T)
REST_A = (END_X - 0.070, BACK_Y + SKIRT_H / 2, BLOCK_TOP + SKIRT_T / 2)
REST = parts.Part("skirting_offcut", pos=REST_A, mass=0.031, shapes=[parts.box(REST_SIZE, color=PRIMED)])


# ---------------------------------------------------------------------------------------------------------------
# The coping saw: Stanley FatMax 0-15-106 (165 mm blade, 170 mm throat, about 0.35 kg). Its frame: origin at the middle
# of the handle, on the handle's axis, which is also the blade's line; local -y runs from the handle to the far end of
# the blade; local +x points from the blade to the back of the frame (the throat). Teeth face local -x.
HANDLE_LEN, HANDLE_FLATS = 0.100, 0.030
PIN_NEAR, PIN_FAR = -0.062, -0.227  # blade pins (165 mm apart)
THROAT = 0.170
TEETH = np.array([-1.0, 0.0, 0.0])
YELLOW, BLACK, CHROME, STEEL = (0.96, 0.76, 0.10), (0.08, 0.08, 0.09), (0.78, 0.79, 0.81), (0.30, 0.31, 0.33)


def _handle():
    r = HANDLE_FLATS / 2 / math.cos(math.pi / 8)
    ring = [(r * math.cos(math.pi / 8 + k * math.pi / 4), r * math.sin(math.pi / 8 + k * math.pi / 4)) for k in range(8)]
    pts = [(x, y, z) for y in (HANDLE_LEN / 2, -HANDLE_LEN / 2) for x, z in ring]
    faces = [tuple(range(8)), tuple(range(15, 7, -1))] + [(k, (k + 1) % 8, 8 + (k + 1) % 8, 8 + k) for k in range(8)]
    return parts.hull(pts, _faces_outward(pts, faces), color=YELLOW)


SAW_SHAPES = [
    _handle(),
    parts.cylinder(0.007, 0.012, at=(0.0, -HANDLE_LEN / 2 - 0.006, 0.0), color=BLACK, axis="y"),  # ferrule/spigot
    parts.box((THROAT + 0.010, 0.006, 0.008), at=(THROAT / 2, PIN_NEAR, 0.0), color=CHROME),  # near arm
    parts.box((THROAT + 0.010, 0.006, 0.008), at=(THROAT / 2, PIN_FAR, 0.0), color=CHROME),  # far arm
    parts.box((0.006, PIN_NEAR - PIN_FAR + 0.006, 0.008), at=(THROAT + 0.002, (PIN_NEAR + PIN_FAR) / 2, 0.0),
              color=CHROME),  # back of the frame
    parts.box((0.0025, PIN_NEAR - PIN_FAR, 0.0006), at=(-0.0005, (PIN_NEAR + PIN_FAR) / 2, 0.0), color=STEEL,
              collide=False),  # the blade (no collider: the cut rule below handles the wood)
]
# It lies flat on the floor to the right of the block, handle toward the robot's right, blade running toward the
# robot's middle, the frame's throat toward the far wall (the model turned over: its local -y is world +y).
SAW_A = (0.200, -0.280, TOP + HANDLE_FLATS / 2 + 0.0005)
_SAW_Q = (1.0, 0.0, 0.0, 0.0)
SAW_SIDE = -1.0  # the handle is on the robot's right (-y) of the blade
SAW = parts.Part("coping_saw", pos=SAW_A, quat=_SAW_Q, mass=0.35, shapes=SAW_SHAPES)

OBJECTS = [ROOM, BLOCK, REST, SCOTIA, SAW]

# Layout lots: trial k uses lot k mod 2. Lot B moves the block, the skirting offcut and the scotia 12 mm toward the
# robot and 20 mm to its right, and the saw 15 mm away from the robot and 12 mm to its right.
SHIFT_B = (-0.012, -0.020)
SAW_SHIFT_B = (0.015, -0.012)


def _shift(p, d):
    return (p[0] + d[0], p[1] + d[1], p[2])


LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place={"scotia": SCOTIA_A, "offcut_block": BLOCK_A, "skirting_offcut": REST_A, "coping_saw": SAW_A}),
    lots.Lot("B", place={"scotia": _shift(SCOTIA_A, SHIFT_B), "offcut_block": _shift(BLOCK_A, SHIFT_B),
                         "skirting_offcut": _shift(REST_A, SHIFT_B), "coping_saw": _shift(SAW_A, SAW_SHIFT_B)}),
)


# ---------------------------------------------------------------------------------------------------------------
# The cut (a declared rule, run every control step). The wood at the 45-degree end is a grid in the scotia's frame:
# 0.25 mm along the length (u 0-24 mm) and in height, 0.5 mm in depth. The blade is the line between its pins.
DU, DH, DD = 0.25, 0.25, 0.5
U_MAX = 24.0
NU, NH, ND = int(U_MAX / DU), int(PROFILE / DH), int(PROFILE / DD)
U_C = (np.arange(NU) + 0.5) * DU
H_C = (np.arange(NH) + 0.5) * DH
D_C = (np.arange(ND) + 0.5) * DD
KERF_R = 0.4  # mm: a 0.8 mm kerf (14 tpi coping blade, 0.5 mm plate plus set; estimate)
STROKE_WINDOW, STROKE_MIN = 10, 7.0  # sawing: the blade slid >= 7 mm along its length in the last 0.5 s
FEED_MAX = 1.5  # mm of new wood per control step (30 mm/s); faster is forcing the blade
REST_MAX = 0.3  # mm a blade not sawing may press into wood without harm
TEETH_CONE = 75.0  # degrees: the blade cuts only toward where its teeth face
TWIST_MAX = 30.0  # degrees between the blade and the scotia's depth while in the wood
ADV_HALF = 40  # control steps: the advance direction compares the blade's mean position over two 2 s windows
BLOCK_MAX = 3.0  # mm of wood swept without cutting (teeth facing away) allowed over 4 s


def _initial_wood():
    uu, hh = np.meshgrid(U_C, H_C, indexing="ij")
    wood = np.zeros((ND, NU, NH), bool)
    for k, d in enumerate(D_C):
        wood[k] = in_profile(np.full_like(hh, d), hh) & (uu >= d)  # the 45-degree cut: u >= d
    return wood


WOOD0 = _initial_wood()
_WASTE_IDEAL = WOOD0 & (U_C[None, :, None] < front(H_C)[None, None, :] - 0.6)
WASTE_IDEAL_N = int(_WASTE_IDEAL.sum())

_rec: dict = {}


def _clear_record() -> None:
    _rec.update(wood=WOOD0.copy(), version=0, prev=None, axial=deque(maxlen=STROKE_WINDOW), zero=None,
                hist=deque(maxlen=2 * ADV_HALF), blocked=deque(maxlen=2 * ADV_HALF), sawn=0.0, sawn_version=-1,
                main=None, main_version=-1, waste_off=False, piece_prev=None, steps=0, log=None, fit=None,
                fit_version=-1)


_clear_record()
# Grippers' clearance, measured every control step in the simulator (None = not measured: no credit that needs it).
_hands = parts.__dict__.setdefault("_scotia_hands", {"clear": None, "steps": 0, "saw_clear": None, "both_steps": 0})


def _rot(q):
    return parts._quat_to_matrix(q)


def _quat(m):
    """Rotation matrix -> quaternion (xyzw), robust at 180 degrees (Shepperd's method)."""
    m = np.asarray(m, float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = 2.0 * math.sqrt(1.0 + tr)
        q = ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    elif m[0, 0] >= m[1, 1] and m[0, 0] >= m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = (0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s)
    elif m[1, 1] >= m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = ((m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s)
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s)
    q = np.asarray(q)
    return tuple(map(float, q / np.linalg.norm(q)))


def _to_local(p_world, piece_pos, piece_r):
    """World points (m) -> the loose scotia's (u, d, h) in mm."""
    loc = (np.asarray(p_world, float) - piece_pos) @ piece_r
    return np.stack([-loc[..., 0], DSIGN * loc[..., 1], loc[..., 2]], axis=-1) * 1000.0


def blade_world(saw_pos, saw_r):
    """World end points of the blade (near pin, far pin)."""
    return saw_pos + saw_r @ np.array([0.0, PIN_NEAR, 0.0]), saw_pos + saw_r @ np.array([0.0, PIN_FAR, 0.0])


def _slice_points(a, b):
    """Where the blade line crosses each depth slice of the grid, as (u, h) mm, or NaN where it does not reach."""
    out = np.full((ND, 2), np.nan)
    dd = b[1] - a[1]
    if abs(dd) < 1e-9:
        return out
    t = (D_C - a[1]) / dd
    ok = (t >= 0) & (t <= 1)
    pts = a[None, :] + t[:, None] * (b - a)[None, :]
    out[ok] = pts[ok][:, [0, 2]]
    return out


def _segment_cells(p0, p1, r):
    """Grid cells (i, j) whose centres lie within r of the segment p0-p1 in one slice ((u, h) mm)."""
    lo = np.minimum(p0, p1) - r
    hi = np.maximum(p0, p1) + r
    i0, i1 = max(0, int(lo[0] / DU) - 1), min(NU, int(hi[0] / DU) + 2)
    j0, j1 = max(0, int(lo[1] / DH) - 1), min(NH, int(hi[1] / DH) + 2)
    if i0 >= i1 or j0 >= j1:
        return None
    uu, hh = np.meshgrid(U_C[i0:i1], H_C[j0:j1], indexing="ij")
    v = p1 - p0
    L2 = float(v @ v)
    if L2 < 1e-12:
        dist2 = (uu - p0[0]) ** 2 + (hh - p0[1]) ** 2
    else:
        t = np.clip(((uu - p0[0]) * v[0] + (hh - p0[1]) * v[1]) / L2, 0.0, 1.0)
        dist2 = (uu - p0[0] - t * v[0]) ** 2 + (hh - p0[1] - t * v[1]) ** 2
    return (slice(i0, i1), slice(j0, j1)), dist2 <= r * r


def _set_zero(reason: str) -> None:
    if _rec["zero"] is None:
        _rec["zero"] = reason


def _blade_damage(a_w, b_w, piece_pos, piece_r) -> None:
    """Whole-trial zero: the blade in the installed scotia, a skirting, a wall, the floor, or the loose scotia's length
    past the end being coped."""
    s = np.linspace(0.0, 1.0, 84)[:, None]
    pts = a_w[None, :] + s * (b_w - a_w)[None, :]
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2] - TOP
    if np.any(z < -0.0005):
        _set_zero("blade into the floor")
    if np.any((x >= X_SA - 0.0002) & (z <= SKIRT_H) & (y <= WALL_B)) or np.any((y >= Y_SB - 0.0002) & (z <= SKIRT_H)
                                                                               & (x <= X_SA)):
        _set_zero("blade into a skirting board")
    if np.any(x >= WALL_A) or np.any(y >= WALL_B):
        _set_zero("blade into a wall")
    w, h = (X_SA - x) * 1000, z * 1000
    if np.any(in_profile(w, h) & (y <= Y_SB + 0.0002) & (y >= Y_SB - FIXED_LEN)):
        _set_zero("blade into the installed scotia")
    loc = _to_local(pts, piece_pos, piece_r)
    if np.any(in_profile(loc[:, 1], loc[:, 2]) & (loc[:, 0] > U_MAX) & (loc[:, 0] <= LENGTH)):
        _set_zero("blade into the scotia's length")


def cut_step(saw_pos, saw_quat, piece_pos, piece_quat, held=None) -> dict:
    """Advance the cut by one control step from the saw's and the scotia's poses. `held`: a gripper touches the
    scotia (default: as measured at the last control step). Returns what happened."""
    if held is None:
        held = _hands.get("touch") is True
    saw_pos, piece_pos = np.asarray(saw_pos, float), np.asarray(piece_pos, float)
    saw_r, piece_r = _rot(saw_quat), _rot(piece_quat)
    a_w, b_w = blade_world(saw_pos, saw_r)
    _rec["steps"] += 1
    info = {"removed": 0, "stroking": False}
    # a teleported scotia (a scene reset or layout move) is not a blade movement
    if _rec["piece_prev"] is not None and np.linalg.norm(piece_pos - _rec["piece_prev"]) > 0.010:
        _rec["prev"] = None
    _rec["piece_prev"] = piece_pos
    centre = (a_w + b_w) / 2
    axis = (b_w - a_w) / np.linalg.norm(b_w - a_w)
    a, b = _to_local(a_w, piece_pos, piece_r), _to_local(b_w, piece_pos, piece_r)
    pts1 = _slice_points(a, b)
    prev = _rec["prev"]
    if prev is not None:
        _rec["axial"].append(abs(float((centre - prev["centre"]) @ axis)) * 1000)
    stroking = len(_rec["axial"]) == STROKE_WINDOW and sum(_rec["axial"]) >= STROKE_MIN
    info["stroking"] = stroking
    _blade_damage(a_w, b_w, piece_pos, piece_r)
    e = (b - a) / max(np.linalg.norm(b - a), 1e-9)
    twisted = abs(e[1]) < math.cos(math.radians(TWIST_MAX))
    tl = piece_r.T @ saw_r @ TEETH
    t2 = np.array([-tl[0], tl[2]])
    t2n = float(np.linalg.norm(t2))
    wood = _rec["wood"]
    pts0 = prev["pts"] if prev is not None else pts1
    worst_depth = 0.0
    blocked = 0.0
    # the direction the blade is advancing: its mean position over the last 2 s against the 2 s before (the
    # wobble of each stroke cancels out)
    hist = _rec["hist"]
    hist.append(pts1)
    adv = None
    if len(hist) == hist.maxlen:
        h = np.array(hist)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            adv = np.nanmean(h[ADV_HALF:], axis=0) - np.nanmean(h[:ADV_HALF], axis=0)
    for k in range(ND):
        p1 = pts1[k]
        if np.isnan(p1[0]):
            continue
        p0 = pts0[k] if not np.isnan(pts0[k][0]) else p1
        got = _segment_cells(p0, p1, KERF_R)
        if got is None:
            continue
        sl, near = got
        hit = near & wood[k][sl]
        n = int(hit.sum())
        if not n:
            continue
        depth = n * DU * DH / (2 * KERF_R)
        worst_depth = max(worst_depth, depth)
        step = adv[k] if adv is not None and not np.isnan(adv[k][0]) else p1 - p0
        moved = float(np.linalg.norm(step))
        toward = moved < 0.05 or t2n < 1e-6 or (step @ t2) / (moved * t2n) >= math.cos(math.radians(TEETH_CONE))
        if stroking and held and toward and not twisted and depth <= FEED_MAX + EPS:
            wood[k][sl][hit] = False
            info["removed"] += n
        elif stroking and held and not toward:
            blocked = max(blocked, depth)
        elif twisted and depth > REST_MAX + EPS:
            _set_zero("blade twisted in the cut")
        elif not held and depth > REST_MAX + EPS:
            _set_zero("sawing the scotia while nothing holds it")
        elif not stroking and depth > REST_MAX + EPS:
            _set_zero("blade pushed into the wood without sawing")
        elif depth > FEED_MAX + EPS:  # a sideways or backward rub of a blade that is sawing cuts nothing
            _set_zero("blade fed faster than it cuts" if toward else "blade forced sideways or backward through the wood")
    info["depth"] = worst_depth
    # a blade carried on through wood it is not cutting (its teeth facing away from the way it goes) is forced
    # through it: the wood it sweeps without cutting, over the last 4 s, may not add up to more than BLOCK_MAX
    _rec["blocked"].append(blocked)
    if sum(_rec["blocked"]) > BLOCK_MAX + EPS:
        _set_zero("blade carried through wood it is not cutting")
    if info["removed"]:
        _rec["version"] += 1
    _rec["prev"] = {"centre": centre, "pts": pts1}
    return info


def _label_main(wood):
    """Cells of wood still joined to the scotia's length (face-connected to the grid's far end)."""
    try:
        from scipy import ndimage

        lab, _ = ndimage.label(wood)
        keep = np.unique(lab[:, -1, :])
        keep = keep[keep > 0]
        return np.isin(lab, keep)
    except ImportError:  # plain flood fill
        main = np.zeros_like(wood)
        main[:, -1, :] = wood[:, -1, :]
        while True:
            grow = main.copy()
            grow[1:] |= main[:-1]
            grow[:-1] |= main[1:]
            grow[:, 1:] |= main[:, :-1]
            grow[:, :-1] |= main[:, 1:]
            grow[:, :, 1:] |= main[:, :, :-1]
            grow[:, :, :-1] |= main[:, :, 1:]
            grow &= wood
            if np.array_equal(grow, main):
                return main
            main = grow


def main_wood():
    if _rec["main_version"] != _rec["version"]:
        _rec["main"] = _label_main(_rec["wood"])
        _rec["main_version"] = _rec["version"]
        left = int((_rec["main"] & _WASTE_IDEAL).sum())
        _rec["waste_off"] = left <= 0.4 * WASTE_IDEAL_N
    return _rec["main"]


def waste_off() -> bool:
    """The offcut has come away: under 40% of the wood behind the pencil line is still joined to the scotia."""
    main_wood()
    return bool(_rec["waste_off"])


# ---------------------------------------------------------------------------------------------------------------
# The fit, judged on the wood the saw left: the coped end offered up to the installed scotia, slid along the side
# skirting until it stops, as a carpenter checks a cope.
GAP_TIERS = ((1.0, 1.0), (2.0, 0.75), (4.0, 0.5), (6.0, 0.25))
CRUSH = 0.4  # mm: pine left proud of the line by less than this crushes when the piece is pushed home (estimate)
CLEAR_M = 0.030  # grippers this far from the scotia and the saw
CLEAR_STEPS = 20  # for 1 s
PINCH_NEAR = 1.0  # mm: a fingertip this close to the scotia's wood touches it
PINCH_ACROSS = 3.0  # mm: the two fingertips of one gripper touch it at least this far apart across its depth
STILL = 0.005  # m/s


def _front_columns():
    """For each height row, the depth slice just inside the scotia's front."""
    ks = []
    for j, h in enumerate(H_C):
        k = int(min(ND - 1, max(0, math.floor((float(front(h)) - 0.26) / DD))))
        while k > 0 and not in_profile(D_C[k], h):
            k -= 1
        ks.append(k)
    return np.array(ks)


FRONT_K = _front_columns()
_COL_OK = in_profile(D_C[:, None], H_C[None, :])  # (ND, NH): slices x rows that have wood at all


def _u_front():
    """Nearest wood to the cut end (mm, cell centres) in every slice x row column; U_MAX if the grid has none left
    there (the wood continues past it)."""
    main = main_wood()
    any_ = main.any(axis=1)
    first = U_C[np.argmax(main, axis=1)]
    return np.where(any_, first, U_MAX)


def _front_polyline():
    """The installed scotia's front outline in its (depth, height) plane, mm: toe face, cove (chords within
    0.001 mm), top flat."""
    pts = [(PROFILE, h) for h in np.linspace(0.0, TOE_TOP, 12)]
    a0 = math.atan2(TOE_TOP - COVE_H, PROFILE - COVE_D)  # the toe's top corner, on the cove circle
    a1 = math.atan2(PROFILE - COVE_H, float(front(PROFILE)) - COVE_D)  # the top flat's front edge
    a0 = a0 if a0 > 0 else a0 + 2 * math.pi
    for a in np.linspace(a0, a1, 160):
        pts.append((COVE_D + COVE_R * math.cos(a), COVE_H + COVE_R * math.sin(a)))
    pts += [(w, PROFILE) for w in np.linspace(float(front(PROFILE)), 0.0, 12)]
    pts = np.array(pts)
    keep = np.r_[True, np.linalg.norm(np.diff(pts, axis=0), axis=1) > 1e-9]
    return pts[keep]


FRONT_LINE = _front_polyline()


def _gap_to_front(w, h):
    """Shortest distance (mm) from points (w, h) to the installed scotia's front outline, negative inside its wood."""
    p = np.stack([np.asarray(w, float), np.asarray(h, float)], axis=-1)
    a, b = FRONT_LINE[:-1], FRONT_LINE[1:]
    ab = b - a
    t = np.clip(((p[:, None, :] - a[None]) * ab[None]).sum(-1) / (ab * ab).sum(-1)[None], 0.0, 1.0)
    d = np.linalg.norm(p[:, None, :] - (a[None] + t[..., None] * ab[None]), axis=-1).min(axis=1)
    inside = in_profile(p[:, 0], p[:, 1])
    return np.where(inside, -d, d)


def fit() -> dict:
    """The coped end offered up: slid along its length into the installed scotia until its wood stops it (wood proud
    by under CRUSH mm across the joint crushes), then the widest gap (mm, across the joint) along its front edge.
    Depends only on the cut; cached per change."""
    if _rec.get("fit_version") == _rec["version"] and _rec.get("fit") is not None:
        return _rec["fit"]
    uf = _u_front()
    over = uf - front(H_C)[None, :]
    m = float(over[_COL_OK].min())  # along the length: the first wood to touch (negative: wood proud of the line)
    near = _COL_OK & (over <= m + 2.5)
    kk, jj = np.nonzero(near)
    u, h = uf[kk, jj], H_C[jj]

    def worst_pen(e):
        return float(-_gap_to_front(u - e, h).min())

    lo, hi = m - 0.01, m + 2.0  # slide e: the piece moved this far into the corner from the line's nominal place
    if worst_pen(hi) <= CRUSH:
        e = hi
    else:
        for _ in range(16):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if worst_pen(mid) <= CRUSH + 1e-9 else (lo, mid)
        e = lo
    rows = np.arange(NH)
    gaps = _gap_to_front(uf[FRONT_K, rows] - e, H_C)
    out = {"gap": float(max(0.0, gaps.max())), "slide": e, "first": m}
    _rec["fit"], _rec["fit_version"] = out, _rec["version"]
    return out


def _speed(st) -> float:
    return float(np.linalg.norm(getattr(st, "lin_vel", np.zeros(3))))


LINE_NEAR = 1.0  # mm: the saw has cut within this of the pencil line
LINE_HALF = 0.5  # share of the line sawn that earns the lowest level with the offcut still on


def line_sawn() -> float:
    """Share of the pencil line (rows of height up the scotia) along which the saw has cut within LINE_NEAR of it,
    in at least half the wood's depth there."""
    if _rec.get("sawn_version") == _rec["version"]:
        return _rec["sawn"]
    wood = _rec["wood"]
    sawn = total = 0
    for j, h in enumerate(H_C):
        d_line = float(front(h))
        ks = [k for k in range(ND) if D_C[k] <= d_line - LINE_NEAR and in_profile(D_C[k], h)]
        if not ks:
            continue
        total += 1
        i0, i1 = max(0, int((d_line - LINE_NEAR) / DU)), min(NU, int((d_line + LINE_NEAR) / DU) + 1)
        cut = sum(1 for k in ks if not wood[k, i0:i1, j].all())
        sawn += cut >= 0.5 * len(ks)
    _rec["sawn"], _rec["sawn_version"] = (sawn / total if total else 0.0), _rec["version"]
    return _rec["sawn"]


def _level(objects) -> float:
    if _rec["zero"] is not None:
        return 0.0
    if waste_off():
        gap = fit()["gap"]
        piece, saw = objects.get("scotia"), objects.get("coping_saw")
        done = (piece is not None and saw is not None and float(saw.low[2]) <= TOP + 0.006
                and _hands.get("clear") is True and _hands.get("saw_clear") is True
                and int(_hands.get("both_steps", 0)) >= CLEAR_STEPS)
        for limit, credit in GAP_TIERS:
            if gap <= limit + EPS and (credit < 1.0 or done):
                return credit
    return GAP_TIERS[-1][1] if line_sawn() >= LINE_HALF - EPS else 0.0


# ---------------------------------------------------------------------------------------------------------------
# Offline replay for near_misses.py (never built by the simulator): a blade path and the end state.

class Offline(parts.PartState):
    """The scotia's end state plus a blade path to replay (near_misses.py only; the simulator never builds one)."""


def offline(state, frames=(), cut_pose=(), hands_clear=False, saw_clear=False, held=True) -> Offline:
    """`state` (a PartState of the scotia) carrying a blade path: ((saw pos, saw quat), ...) at 20 Hz with the scotia
    at `cut_pose` (pos, quat), and whether both grippers were 3 cm clear of the scotia (and of the saw) for 1 s."""
    o = Offline(**{f.name: getattr(state, f.name) for f in dataclasses.fields(state)})
    for k, v in (("frames", tuple(frames)), ("cut_pose", tuple(cut_pose)), ("hands_clear", bool(hands_clear)),
                 ("saw_clear", bool(saw_clear)), ("held", bool(held))):
        object.__setattr__(o, k, v)
    return o


def _replay(piece: Offline) -> None:
    _clear_record()
    if piece.frames:
        pp, pq = piece.cut_pose
        for sp, sq in piece.frames:
            cut_step(sp, sq, pp, pq, held=getattr(piece, "held", True))
    _hands.update(clear=bool(piece.hands_clear), steps=CLEAR_STEPS if piece.hands_clear else 0,
                  saw_clear=bool(piece.saw_clear),
                  both_steps=CLEAR_STEPS if piece.hands_clear and piece.saw_clear else 0)


def success(objects):
    """The scotia coped so its end closes to 1 mm everywhere along the joint when offered up to the installed scotia,
    the saw put down on the floor, both still, both grippers 3 cm clear of the scotia and the saw for 1 s."""
    return bool(score(objects) == 1.0)


def score(objects):
    """0 for any whole-trial fault (blade forced, or into the installed scotia, a skirting, a wall, the floor or the
    scotia's length) or if the offcut never came away. Else by the widest gap when the coped end is offered up: 1.0
    at 1 mm (saw put down, scotia let go, grippers 3 cm clear for 1 s), 0.75 at 2 mm, 0.5 at 4 mm, 0.25 at 6 mm."""
    piece = objects.get("scotia")
    if hasattr(piece, "cut_pose"):  # an offline near-miss case
        _replay(piece)
    return _level(objects)


# ---------------------------------------------------------------------------------------------------------------
# Simulator hooks.
_HAND = {}


def _hand_points():
    """Points over every face of both arms' grippers and fingers (gripper-link frames), about 4 mm apart."""
    if "pts" in _HAND:
        return _HAND["pts"]
    from yam_arena import analysis

    out = {}
    for name in ("gripper", "tip_left", "tip_right"):
        cloud = []
        for hull in analysis.link_hulls().get(name, []):
            v = np.asarray(hull.vertices, float)
            cloud.extend(v)
            for pl in np.unique(np.round(hull.planes, 7), axis=0):
                face = v[np.abs(v @ pl[:3] - pl[3]) < 2e-6]
                if len(face) < 3:
                    continue
                c = face.mean(axis=0)
                for p in face:
                    n = max(1, math.ceil(np.linalg.norm(p - c) / 0.004))
                    for s in range(1, n + 1):
                        cloud.append(c + (p - c) * s / n)
                for p, q in zip(face, np.roll(face, -1, axis=0)):
                    n = max(1, math.ceil(np.linalg.norm(q - p) / 0.004))
                    for s in range(n):
                        cloud.append(p + (q - p) * s / n)
        cloud = np.asarray(cloud)
        _, keep = np.unique(np.floor(cloud / 0.003).astype(int), axis=0, return_index=True)
        out[name] = cloud[keep]
    _HAND["pts"] = out
    return out


def _box_distance(pts, centre, rot, half) -> float:
    q = np.abs((pts - centre) @ rot) - half
    return float(np.min(np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)))


def _touch_depth(pts_local):
    """Median depth d (mm) of the points within PINCH_NEAR of the scotia's wood (its length, its profile), or None."""
    p = pts_local[(pts_local[:, 0] >= 0.0) & (pts_local[:, 0] <= LENGTH)]
    if not len(p):
        return None
    near = np.zeros(len(p), bool)
    for dd in (-PINCH_NEAR, 0.0, PINCH_NEAR):
        for dh in (-PINCH_NEAR, 0.0, PINCH_NEAR):
            near |= in_profile(p[:, 1] + dd, p[:, 2] + dh)
    return float(np.median(p[near, 1])) if near.any() else None


def pinched(tips) -> bool:
    """A gripper pinches the scotia: both its fingertips touch the wood, PINCH_ACROSS apart across its depth.
    `tips`: per gripper, its two fingertip point clouds in the scotia's (u, d, h) mm."""
    for left_tip, right_tip in tips:
        a, b = _touch_depth(left_tip), _touch_depth(right_tip)
        if a is not None and b is not None and abs(a - b) >= PINCH_ACROSS - EPS:
            return True
    return False


def _measure_hands(sim, states) -> None:
    try:
        base = sim.env.unwrapped
        origin = parts._np(base.scene.env_origins[0])
        piece = states["scotia"]
        clouds, tips = [], []
        for arm in ("left_robot", "right_robot"):
            asset = base.scene[arm]
            poses = parts._np(asset.data.body_link_pose_w)[0]
            pair = []
            for name, p in _hand_points().items():
                pose = poses[asset.body_names.index(name)]
                cloud = p @ parts._quat_to_matrix(pose[3:]).T + pose[:3] - origin
                clouds.append(cloud)
                if name in ("tip_left", "tip_right"):
                    pair.append(_to_local(cloud, np.asarray(piece.pos, float), _rot(piece.quat_xyzw)))
            tips.append(tuple(pair))
        pts = np.concatenate(clouds)
        if not np.isfinite(pts).all():
            raise ValueError("hand surface unknown")
        r = _rot(piece.quat_xyzw)
        centre = np.asarray(piece.pos) + r @ np.array([-LENGTH / 2000, DSIGN * PROFILE / 2000, PROFILE / 2000])
        d_piece = _box_distance(pts, centre, r, np.array([LENGTH / 2000, PROFILE / 2000, PROFILE / 2000]))
        saw = states["coping_saw"]
        sr = _rot(saw.quat_xyzw)
        boxes = SAW.world_boxes({"base": (np.asarray(saw.pos), sr)})
        d_saw = min(_box_distance(pts, c, rr, s / 2) for _, c, rr, s in boxes)
    except Exception as e:  # unknown gripper state: no credit that needs it
        if not _HAND.get("warned"):
            print(f"scotia task: gripper clearance not measured ({e!r}); no credit that needs it", flush=True)
            _HAND["warned"] = True
        _hands.update(clear=None, steps=0, saw_clear=None, both_steps=0, touch=None)
        return
    clear = d_piece >= CLEAR_M - EPS
    saw_clear = d_saw >= CLEAR_M - EPS
    still = _speed(piece) <= STILL and _speed(saw) <= STILL
    _hands["clear"] = clear
    _hands["saw_clear"] = saw_clear
    _hands["steps"] = _hands.get("steps", 0) + 1 if clear else 0
    _hands["both_steps"] = _hands.get("both_steps", 0) + 1 if clear and saw_clear and still else 0
    _hands["touch"] = pinched(tips)
    _HAND["d"] = (round(d_piece, 4), round(d_saw, 4))


@parts.on_step
def _step(sim) -> None:
    states = sim.states()
    piece, saw = states["scotia"], states["coping_saw"]
    info = cut_step(saw.pos, saw.quat_xyzw, piece.pos, piece.quat_xyzw)
    was_off = _rec["waste_off"]
    off = waste_off()
    if off and not was_off:
        try:
            sim.set_visible("scotia", "waste", False)
            sim.set_visible("scotia", "line", False)
        except Exception as e:  # the look only
            print(f"scotia task: could not hide the offcut ({e!r})", flush=True)
    _measure_hands(sim, states)
    path = os.environ.get("SCOTIA_LOG")
    if path:
        a_w, b_w = blade_world(np.asarray(saw.pos), _rot(saw.quat_xyzw))
        mid = _to_local((a_w + b_w) / 2, np.asarray(piece.pos), _rot(piece.quat_xyzw))
        row = {"k": _rec["steps"], "saw": [round(float(v), 5) for v in saw.pos],
               "saw_q": [round(float(v), 4) for v in saw.quat_xyzw], "piece": [round(float(v), 5) for v in piece.pos],
               "piece_q": [round(float(v), 4) for v in piece.quat_xyzw],
               "blade_uhd": [round(float(v), 3) for v in mid], "stroke": info["stroking"],
               "removed": info["removed"], "depth": round(info.get("depth", 0.0), 3), "zero": _rec["zero"],
               "waste_off": off, "hands": _HAND.get("d"), "clear_steps": _hands.get("steps")}
        if off:
            row["fit"] = {k: round(v, 3) for k, v in fit().items()}
        with open(path, "a") as fh:
            fh.write(json.dumps(row) + "\n")


@parts.on_reset
def _reset(sim) -> None:
    _clear_record()
    _hands.update(clear=None, steps=0, saw_clear=None, both_steps=0, touch=None)
    for v in ("waste", "line"):
        try:
            sim.set_visible("scotia", v, True)
        except Exception:
            pass


# ---------------------------------------------------------------------------------------------------------------
# Scripted demo: open loop from the trial's published layout (where the block, scotia and saw lie). The right hand
# pinches the upright scotia across its depth and holds it on the block; the left hand takes the saw and copes the
# end, then puts the saw back; the right hand sets the scotia down by the side skirting, slides it along into the
# corner, lets go and pushes it back onto the skirting.
TIP = 0.013  # finger tips below the grip point
SAW_GRIP = -0.012  # pads 24 mm apart on the 30 mm handle
PINCH_H = 11.0  # mm: fingertips this high on the scotia, so both pads bear at the same height (back face and cove)
PINCH_W = float(front(PINCH_H))  # 6.8 mm: the scotia's depth there, from its back face to the cove
SCOTIA_GRIP = -(PINCH_W / 2 - 1.5) / 1000  # pads 1.5 mm past each side
LEAN = 48.0  # degrees commanded about the blade (the saw droops about 8 under its weight): teeth forward and down
STROKE = 0.006  # m each way along the blade
HALF_STROKE_S = 0.60
FEED_PER_HALF = 0.6  # mm of cut per half-stroke (1.5 mm/s)
KERF_SIDE = 0.45  # mm: the blade's centre this far on the waste side of the line (kerf edge 0.05 mm off it)
# Demo-only margins toward the waste, set from the blade path measured on the GPU (both lots): the stroke's swing
# reached 1.2 mm into the nearly level underside of the cove's foot, and about 0.9 mm past the line on the upper cove
# and the toe.
LOW_EXTRA = 0.7  # mm on the lower cove (arc past 226-234 degrees)
TOP_EXTRA = 0.3  # mm on the upper cove (arc before 197-205 degrees)
TOE_EXTRA = 0.3  # mm on the toe
LEVEL_DROP = 0.5  # mm lower on the level pass under the cove's foot (its upswing reached 1.6 mm above the plan)
DEMO_BIAS = 0.50  # mm more toward the waste in the demo: the blade jitters about 1 mm while sawing, and the fit
# tolerates 0.6 mm left proud (it crushes) and 1 mm cut into the kept wood
SAG = (0.0136, 0.0, 0.0217)  # m added to the grip point while the saw is held and leaned (measured droop)
ROLL = 3.4  # degrees: the saw droops this much in the pads about their axis; the wrist is turned to cancel it
YAW_FIX = -1.41  # degrees about the vertical: the measured turn of the held saw, cancelled
YAW_SAW = math.pi / 2  # fingers close along world x, across the handle
SAW_ARM = "right"
BLADE_SHIFT = 0.050  # m: the scotia sits this far from the blade's middle toward its far end (the saw held off to
# the right, clear of the left arm)
GRIP_OUT = -0.030  # m: the hand holds the handle this far from its middle toward its outer end (negative: toward
# the blade, nearer the saw's centre of mass, so the saw does not turn in the pads)
G_HOLD = 0.185  # m from the 45-degree end: where the left hand holds the scotia (for the cut and for fitting it)
PUSH_CONTACT = 0.0258  # m: grip point off the side skirting when the leaned fingertips first touch the seated toe
PUSH_TCP_Z = 0.0145  # m above the floor for those pushes (lowest fingertip 2.5 mm up)
AIR_STROKES = 4
RIGHT_PARK = (0.16, -0.24, TOP + 0.20)
LEFT_PARK = (0.16, 0.24, TOP + 0.20)


def cut_path(off=KERF_SIDE, low_extra=0.0, top_extra=0.0, toe_extra=0.0, level_drop=0.0):
    """The blade's line through the scotia (u, h mm) from above the top flat, round the cove, under the cove's foot
    (just below the toe's top) and down the toe, offset `off` to the waste side, points about FEED_PER_HALF apart."""
    rr = COVE_R + off
    h_tk = max(TOE_TOP - KERF_R + 0.05, COVE_H - rr)  # under the cove's foot: kerf edge 0.05 mm above the toe's top
    th0 = math.atan2(PROFILE - COVE_H, float(front(PROFILE)) - COVE_D)  # 173.4 deg: the top flat's front edge
    th1 = math.pi + math.asin(min(1.0, (COVE_H - h_tk) / rr))  # where the offset arc comes down to h_tk
    pts = []
    n = max(2, math.ceil(rr * (th1 - th0) / FEED_PER_HALF))
    for i in range(n + 1):
        t = th0 + (th1 - th0) * i / n
        deg = math.degrees(t)
        r = (rr + low_extra * min(1.0, max(0.0, (deg - 226.0) / 8.0))  # more margin on the lower cove
             + top_extra * min(1.0, max(0.0, (205.0 - deg) / 8.0)))  # and on the upper cove
        pts.append((COVE_D + r * math.cos(t), max(h_tk, COVE_H + r * math.sin(t))))
    lead = [(pts[0][0], PROFILE + 10.0 - k) for k in range(0, 10)]  # stroking from 10 mm above, down 1 mm a stroke
    pts = lead + pts
    u_toe, (u_last, _) = PROFILE - off - toe_extra, pts[-1]
    n = max(1, math.ceil((u_toe - u_last) / FEED_PER_HALF))
    pts += [(u_last + (u_toe - u_last) * i / n, h_tk - (level_drop if i < n else 0.0)) for i in range(1, n + 1)]
    n = max(1, math.ceil((h_tk + 4.0) / FEED_PER_HALF))
    pts += [(u_toe, h_tk - (h_tk + 4.0) * i / n) for i in range(1, n + 1)]
    return pts


def _world(seat, u, h):
    """World (x, z) of the upright scotia's (u, h) mm with its origin at `seat`."""
    return seat[0] - u / 1000, seat[2] + h / 1000


def _rz(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _roll(deg):
    """Rotation about world x by `deg`: raises the blade's far end (toward +y)."""
    return _rx(math.radians(deg))


def _saw_wp(dur, pos, label, lean=LEAN, finger=SAW_GRIP, sag=True):
    """A held-saw waypoint: `pos` is where the handle's middle (on the blade's line) should be."""
    s = SAG if sag and lean else (0.0, 0.0, 0.0)
    p = (pos[0] + s[0], pos[1] + s[1] + SAW_SIDE * GRIP_OUT, pos[2] + s[2])
    if not lean:
        return tilt.Waypoint(dur, p, finger, label, YAW_SAW, tilt=0.0, heading=0.0)
    rot = _rz(YAW_FIX) @ _roll(ROLL) @ tilt.rotation(YAW_SAW, lean, 0.0)
    return Waypoint(dur, p, finger, label, tilt._TiltedYaw(YAW_SAW, rot))


def cut_waypoints(seat, path, stroke=STROKE, half_s=HALF_STROKE_S, lean=LEAN, air_strokes=AIR_STROKES):
    """Right-arm waypoints that take the held saw from above the line, saw down `path` and drop it clear."""
    out = []
    yc = seat[1] + DSIGN * PROFILE / 2000 - SAW_SIDE * (PIN_NEAR + PIN_FAR) / 2 + SAW_SIDE * BLADE_SHIFT
    x0, z0 = _world(seat, *path[0])
    out.append(_saw_wp(2.4, (x0, yc, z0 + 0.10), "over the line, frame raised", lean=lean))
    out.append(_saw_wp(2.0, (x0, yc, z0), "down over the line", lean=lean))
    out.append(_saw_wp(0.6, (x0, yc + stroke, z0), "blade above the top", lean=lean))
    side = -1
    for k in range(2 * air_strokes):
        out.append(_saw_wp(half_s, (x0, yc + side * stroke, z0), f"stroke in the air {k}", lean=lean))
        side = -side
    for k, (u, h) in enumerate(path[1:]):
        x, z = _world(seat, u, h)
        out.append(_saw_wp(half_s, (x, yc + side * stroke, z), f"saw {k}", lean=lean))
        side = -side
    ue, he = path[-1]
    x, z = _world(seat, ue, he)
    out.append(_saw_wp(0.8, (x, yc, z - 0.012), "drop clear of the cut", lean=lean))
    out.append(_saw_wp(1.0, (seat[0] + 0.012, yc, z - 0.012), "out past the end", lean=lean))
    out.append(_saw_wp(1.2, (seat[0] - 0.02, yc + SAW_SIDE * 0.06, TOP + 0.12), "up and away", lean=lean))
    return out


def saw_frames(waypoints, held=True):
    """Offline: the saw's (pos, quat) at 20 Hz if it followed the commanded grip exactly (no arm dynamics, no droop),
    for a right-arm waypoint list during which the saw is held. Used by near_misses.py only."""
    from yam_arena import station
    from yam_arena.demo import ArmPlan, grip_rotation

    plan = ArmPlan(list(waypoints), station.BASES[SAW_ARM])
    g0 = grip_rotation(YAW_SAW)
    r0 = parts._quat_to_matrix(_SAW_Q)
    out = []
    for k in range(int(round(plan.duration * 20))):  # samples up to the plan's end, not the "done" pose after it
        pos, rot, _, label = plan.target(min((k + 1) / 20, plan.duration - 1e-6))
        lean = ("frame raised" in label or "blade above" in label or "down over" in label
                or label.startswith(("saw", "stroke", "drop", "out")))
        sag = np.asarray(SAG) if lean else np.zeros(3)
        mid = np.asarray(pos) - sag - np.array([0.0, SAW_SIDE * GRIP_OUT, 0.0])
        undo = _roll(-ROLL) @ _rz(-YAW_FIX) if lean else np.eye(3)  # the droop the corrections cancel
        out.append((tuple(map(float, mid)), _quat(undo @ rot @ g0.T @ r0)))
    return tuple(out)


def _dur(ws):
    return sum(w.duration for w in ws)


def _wait_until(ws, t, pos, finger, label, yaw=0.0, **lean):
    gap = t - _dur(ws)
    if gap > 1e-6:
        ws.append(tilt.Waypoint(gap, pos, finger, label, yaw, **lean) if lean else Waypoint(gap, pos, finger, label, yaw))


def demo():
    lot = lots.current()
    sp = lot.pos("coping_saw")
    seat = np.asarray(lot.pos("scotia"), float)  # upright, on the block, its back toward the robot's left
    left, right = [], []
    # left hand: pinch the scotia across its depth, high on it, and hold it on the block
    lx, ly, lz = seat[0] - G_HOLD, seat[1] + DSIGN * PINCH_W / 2000, seat[2] + TIP + PINCH_H / 1000
    left.append(Waypoint(2.0, (lx, ly, lz + 0.05), -0.025, "over the scotia"))
    left.append(Waypoint(1.2, (lx, ly, lz), -0.025, "down round it"))
    left.append(Waypoint(0.8, (lx, ly, lz), SCOTIA_GRIP, "hold it on the block"))
    held_at = _dur(left)
    # right hand: take the saw by its handle
    hy = sp[1] + SAW_SIDE * GRIP_OUT  # where the hand takes the handle
    right.append(Waypoint(2.0, (sp[0], hy, TOP + 0.10), FINGER_OPEN, "over the saw handle", YAW_SAW))
    right.append(Waypoint(1.5, (sp[0], hy, TOP + 0.017), FINGER_OPEN, "down round the handle", YAW_SAW))
    right.append(Waypoint(0.8, (sp[0], hy, TOP + 0.017), SAW_GRIP, "grip the handle", YAW_SAW))
    right.append(_saw_wp(1.2, (sp[0], sp[1], TOP + 0.12), "lift the saw", lean=0.0))
    _wait_until(right, held_at, (sp[0], hy, TOP + 0.12), SAW_GRIP, "wait for the hold", YAW_SAW)
    # cope the end, then put the saw back down where it was
    right += cut_waypoints(seat, cut_path(off=KERF_SIDE + DEMO_BIAS, low_extra=LOW_EXTRA, top_extra=TOP_EXTRA,
                                          toe_extra=TOE_EXTRA, level_drop=LEVEL_DROP))
    cut_done = _dur(right)
    right.append(_saw_wp(1.8, (sp[0], sp[1], TOP + 0.10), "back over its place", lean=0.0))
    right.append(_saw_wp(1.2, (sp[0], sp[1], TOP + 0.019), "lower the saw", lean=0.0))
    right.append(Waypoint(0.6, (sp[0], hy, TOP + 0.019), -0.020, "let go of the saw", YAW_SAW))
    right.append(Waypoint(1.0, (sp[0], hy, TOP + 0.10), -0.020, "up off the saw", YAW_SAW))
    right.append(Waypoint(1.5, RIGHT_PARK, FINGER_CLOSED, "park", 0.0))
    _wait_until(left, cut_done, (lx, ly, lz), SCOTIA_GRIP, "hold it while it is cut")
    # let go of the coped scotia on the block (it leans back on the skirting offcut) and move both hands clear
    left.append(Waypoint(0.6, (lx, ly, lz), -0.012, "let go"))
    left.append(Waypoint(0.8, (lx, ly, lz + 0.05), -0.012, "up"))
    left.append(Waypoint(1.5, LEFT_PARK, FINGER_CLOSED, "park"))
    _wait_until(right, _dur(left), RIGHT_PARK, FINGER_CLOSED, "wait clear", 0.0)
    _wait_until(left, _dur(right) + 3.0, LEFT_PARK, FINGER_CLOSED, "wait clear")
    _wait_until(right, _dur(left), RIGHT_PARK, FINGER_CLOSED, "wait clear", 0.0)
    return {"left": left, "right": right}
