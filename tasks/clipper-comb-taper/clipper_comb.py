"""Clipper-over-comb taper on a training head. A barber's training head on a stand faces away from the robot, so the
robot works on the back of the head. The back carries one continuous panel of hair, 35 mm long, from the nape up 8 cm
and 70 degrees across (about 11 cm). The barber has already cut the rest: beside the panel the hair is already at the
taper's length for its height, and above it 22 mm, so the panel has to blend into them. A comb stands in a rack and a hair
clipper with no guard lies on a holder in front of it.

Goal: a taper, as a barber cuts it: the hair 8 mm long at the bottom of the panel, getting longer evenly to 22 mm at
its top, everywhere in the panel, with no visible lines, and nothing shaved bare. The clipper has no guard: run
straight over the hair it lays it flat and shaves it to the scalp where it touches. The length is set only by holding
the comb in the hair, teeth toward the scalp, at the right distance from it, and running the clipper along the comb's
back. The head is curved across and up, and the length changes with height, so the comb has to be set section by
section, tilted to the taper, and moved up the head band by band, each pass overlapping the last.

Offline check (no GPU):  python clipper_comb.py
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import hingekit as hk  # noqa: E402
import yam_arena.demo as _demo  # noqa: E402
from yam_arena import ik  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Taper the long hair on the back of the training head with the clipper over the comb: 8 mm at its "
               "bottom edge, getting longer evenly to 22 mm at its top edge, blended into the hair around it with no "
               "lines. Then put the comb and the clipper back where they were.")

# -- the head (m) -------------------------------------------------------------------------------------------------
HX = float(os.environ.get("CC_HX", "0.40"))  # head axis, forward of the arm-base line; the back of the head faces the robot (-x)
HY = float(os.environ.get("CC_HY", "-0.055"))  # head axis, sideways (+ left): 5.5 cm right, so the comb arm reaches the right side clear of the clipper arm
HZ = float(os.environ.get("CC_HZ", "-0.025"))  # the whole head (and its stand) lowered this much
# The head's outline, round its vertical axis: radius at head height h (above the stand's foot, before HZ): an
# ellipse 18 cm across at its widest (the back of the skull, 6 cm above the neck), closing toward the crown.
E_Z, E_A, E_B = 0.150, 0.090, 0.120
H_LO, H_HI = 0.088, 0.255  # bottom of the head (top of the neck) and its flat top


def rad(h):
    q = (np.asarray(h, float) - E_Z) / E_B
    return E_A * np.sqrt(np.clip(1.0 - q * q, 1e-6, None))


def drad(h):
    q = (np.asarray(h, float) - E_Z) / E_B
    return -E_A * q / E_B / np.sqrt(np.clip(1.0 - q * q, 1e-6, None))


NECK = (0.035, H_LO)  # neck radius, from the stand up to the head
SLICE = float(os.environ.get("CC_SLICE", "0.003"))  # the head is drawn (and collides) as 3 mm horizontal slices of that outline
# the hair panel: head heights and azimuths (round the back of the head; + toward the robot's left)
PANEL_H = (0.100, 0.180)
PANEL_AZ = 35.0  # half-width, degrees
PANEL_C = float(os.environ.get("CC_PAZ", "0.0"))  # the panel's middle, degrees from straight back (+ toward the robot's left)
L0 = 0.035  # the panel's hair starts 35 mm everywhere
T_LO, T_HI = 0.008, 0.022  # the taper: 8 mm at the panel's bottom edge to 22 mm at its top, evenly
TOL_SHORT, TOL_LONG = 0.002, 0.003  # right from 2 mm short to 3 mm long of the length at that height
LINE = 0.002  # a step of more than 2 mm (beyond the taper's own 0.44 mm) between neighbouring points 2.5 mm apart is a visible line
GOUGE = 0.005  # anything under 5 mm is a bald patch: the cut can't be undone
FLAT_REACH = 0.006  # with no comb in the hair, the clipper shaves what it passes within 6 mm of the scalp
EAR_H = (0.125, 0.175)
SKIN = (0.86, 0.70, 0.58)
HAIR = (0.30, 0.20, 0.12)  # one brown all over the head, as on a real training head
HAIR_SHADES = ((0.36, 0.24, 0.14), (0.25, 0.16, 0.10), (0.40, 0.27, 0.16))  # single strands, a little lighter or darker
SCALP = (0.20, 0.13, 0.08)  # the scalp under the hair (and the cropped hair round the face and crown)
DARK = (0.16, 0.16, 0.18)
STEEL = (0.78, 0.78, 0.80)
MAT = (0.28, 0.30, 0.32)
COMB_C = (0.12, 0.12, 0.13)


def target(h):
    """The taper's length at head height h, in the panel."""
    return T_LO + (T_HI - T_LO) * (np.asarray(h, float) - PANEL_H[0]) / (PANEL_H[1] - PANEL_H[0])


kit = hk.Kit()
_head = [hk.cyl(NECK[0], NECK[1] + HZ, at=(0, 0, (NECK[1] + HZ) / 2), color=SKIN)]
_n = int(round((H_HI - H_LO) / SLICE))
_head += [hk.cyl(float(rad(H_LO + (k + 0.5) * SLICE)), SLICE, at=(0, 0, H_LO + (k + 0.5) * SLICE + HZ), color=SCALP)
          for k in range(_n)]  # short hair all over
_head.append(hk.box((0.010, 0.100, 0.115), at=(0.083, 0.0, 0.100 + HZ + 0.0575), color=SKIN))  # the face, in front
for sgn in (1, -1):  # ears: flat pads on the sides
    _head.append(hk.box((0.030, 0.008, EAR_H[1] - EAR_H[0]), at=(0.0, sgn * 0.094, sum(EAR_H) / 2 + HZ), color=SKIN))
_head.append(hk.cyl(0.07, 0.012, at=(0, 0, 0.006), color=DARK))  # stand foot
kit.part("head", origin=(HX, HY, TOP), color=SKIN, shapes=_head)
# clipper: a body with the blade face at local +x; part frame at the body's centre
CLIP_L, CLIP_W, CLIP_H = 0.100, 0.045, 0.025
BLADE_X = CLIP_L / 2 + 0.002  # the cutting face
BLADE_H = CLIP_H * 0.8  # the teeth's height across the face
HOLDER_H = float(os.environ.get("CC_HH", "0.050"))  # the clipper lies on a holder block
HOLDER_W = 0.060
CLIP_AT = np.array([float(os.environ.get("CC_CX", "0.17")), float(os.environ.get("CC_CY", "-0.30")), TOP + 0.003 + HOLDER_H + CLIP_H / 2])
# comb: a flat toothed plate with a handle, standing in a slotted rack. Part frame: the plate's centre on its inner
# face (the teeth's tips); local +x points into the hair (toward the scalp), y along the comb, z up.
COMB_T, COMB_W, COMB_L = 0.003, float(os.environ.get("CC_CW", "0.030")), 0.120
HANDLE = (0.010, 0.080, COMB_W)  # a thicker handle beyond the teeth, toward the robot's left
HANDLE_Y = COMB_L / 2 + HANDLE[1] / 2
RACK_H = float(os.environ.get("CC_RACK", "0.006"))
RACK_SLOT = 0.011  # the slot holds the 1 cm handle within about 10 degrees of upright
COMB_AT = np.array([0.20, 0.13, TOP + 0.003 + RACK_H - 0.005 + COMB_W / 2])  # 5 mm down in the rack's slot
_GREY = (0.35, 0.37, 0.40)
MAT_X = 0.20
kit.part("mat", origin=(MAT_X, 0.0, TOP), color=MAT, shapes=[
    hk.box((0.14, 0.68, 0.003), at=(0, 0, 0.0015)),
    hk.box((0.080, HOLDER_W, HOLDER_H), at=(CLIP_AT[0] - MAT_X, CLIP_AT[1], 0.003 + HOLDER_H / 2), color=_GREY),
    *[hk.box((0.016, COMB_L + HANDLE[1] + 0.02, RACK_H), at=(COMB_AT[0] - MAT_X - COMB_T / 2 + sg * (RACK_SLOT / 2 + 0.008), COMB_AT[1] + HANDLE[1] / 2,
                                                  0.003 + RACK_H / 2), color=_GREY) for sg in (-1, 1)],  # comb rack
])
# a barber's comb: a 7 mm spine along its lower edge and 40 teeth up from it (1.2 mm wide, 1.8 mm gaps, 23 mm long),
# which the hair stands up through
SPINE = 0.007
N_TEETH = 40
_PITCH = COMB_L / N_TEETH
kit.part("comb", origin=tuple(COMB_AT), mass=0.03, color=COMB_C, shapes=[
    hk.box((COMB_T, COMB_L, SPINE), at=(-COMB_T / 2, 0, -COMB_W / 2 + SPINE / 2)),
    *[hk.box((COMB_T, 0.0012, COMB_W - SPINE), at=(-COMB_T / 2, -COMB_L / 2 + _PITCH * (i + 0.5), SPINE / 2))
      for i in range(N_TEETH)],
    hk.box(HANDLE, at=(-COMB_T / 2, HANDLE_Y, 0)),
])
# a clipper: the body and its steel blade on the front, the cutting edge 22 teeth (1 mm wide, 1 mm gaps)
kit.part("clipper", origin=tuple(CLIP_AT), mass=0.20, color=DARK, shapes=[
    hk.box((CLIP_L, CLIP_W, CLIP_H), at=(0, 0, 0)),
    hk.box((0.002, CLIP_W, CLIP_H * 0.8), at=(CLIP_L / 2 - 0.001, 0, 0), color=STEEL),
    *[hk.box((0.002, 0.001, CLIP_H * 0.8), at=(CLIP_L / 2 + 0.001, -CLIP_W / 2 + 0.002 * (i + 0.5) + 0.0005, 0),
             color=STEEL) for i in range(22)],
])

EZ = np.array([0.0, 0.0, 1.0])


def _u(az_deg):
    a = math.radians(az_deg)
    return np.array([-math.cos(a), math.sin(a), 0.0])


def scalp(az_deg, h):
    """(point on the scalp, outward normal) at azimuth az (degrees) and head height h."""
    u = _u(az_deg)
    p = np.array([HX, HY, TOP + h + HZ]) + float(rad(h)) * u
    n = u - float(drad(h)) * EZ
    return p, n / np.linalg.norm(n)


# -- the hair -----------------------------------------------------------------------------------------------------
# Scored: a grid of points 2.5 degrees by 2.5 mm over the whole panel (28 x 32), each with its own length.
GRID_AZ = PANEL_C + np.linspace(-PANEL_AZ + 1.25, PANEL_AZ - 1.25, 28)
GRID_H = np.linspace(PANEL_H[0] + 0.00125, PANEL_H[1] - 0.00125, 32)
_G = [(a, h) for h in GRID_H for a in GRID_AZ]  # row-major: index = row * 28 + column
G_BASE = np.array([scalp(a, h)[0] for a, h in _G])
G_N = np.array([scalp(a, h)[1] for a, h in _G])
G_TARGET = target(np.array([h for a, h in _G]))
NR, NC = len(GRID_H), len(GRID_AZ)

# Drawn (display only; the grid above is what is cut and scored): the back of the head is covered in single strands of
# hair, about 25,000 of them (USD curves, which the RTX renderer draws as fine tubes): each grows from its own point of
# the scalp, leaves it more upright than it lies, bends down toward its tip, thins from 0.32 to 0.12 mm (drawn thicker
# and sparser than real hair, about 100 per square cm in the panel, so it shows on camera) and is up to 3 mm short of
# the rest at its tip. Combed, the hair lies down the head 80 degrees from standing straight out. The panel's strands
# are grouped in 7 x 16 locks (10 degrees by 5 mm) that stand straight out while the comb's plate is in them (as hair
# stands up through a comb) and lie down again when it leaves; each strand is drawn cut to the length of the scored
# point nearest its root. Around the panel the hair the barber has already cut is drawn the same way at its length:
# beside the panel the taper's length at that height, above it 22 mm, at the nape 6-7 mm. The strands don't move on
# their own and nothing collides with them.
LIE_DEG = float(os.environ.get("CC_LIE", "80"))  # combed down: 80 degrees from standing out
DROOP_DEG = LIE_DEG
DROOP_RATE = 12.0  # degrees per control step a lock swings toward lying or standing
H_LAYER = 0.0012  # the layer's thickness
STRAND = 0.0007  # a strand's thickness
RAGGED = (0.0, 0.0010, 0.0025, 0.0005, 0.0018)  # how far short of the lock's length each strand's tip ends


def _jitter(a, h):
    """A lock's own small, fixed irregularity (degrees): how much more it lies down, and how far it leans sideways, so
    the hair doesn't fall in rows like tiles."""
    k = int(round(a * 7 + h * 3001))
    return 4.0 * math.sin(k * 1.7), 8.0 * math.sin(k * 2.3 + 1.0)


def lock_frame(a, h, lie_deg=LIE_DEG):
    """(rotation, root) of a lock at azimuth a, height h: local x along the hair (out from the root), y across it, z
    through it. Lying `lie_deg` (plus its own jitter) down the head from standing straight out."""
    p, n = scalp(a, h)
    side = np.cross(EZ, _u(a))
    side /= np.linalg.norm(side)
    down = np.cross(n, side)  # in the head's surface, pointing down
    dl, lean = _jitter(a, h)
    lean = math.radians(lean) * lie_deg / LIE_DEG  # standing straight out (in the comb) it doesn't lean
    down, side = math.cos(lean) * down + math.sin(lean) * side, math.cos(lean) * side - math.sin(lean) * down
    t = math.radians(lie_deg + dl * lie_deg / LIE_DEG)
    d = math.cos(t) * n + math.sin(t) * down
    return np.column_stack([d, side, np.cross(d, side)]), p


HAIR_STYLE = os.environ.get("CC_HAIR", "strands")  # "strands": real strands (USD curves); "boxes": flat layers (fallback)
STRAND_W = (0.00032, 0.00012)  # a strand drawn 0.32 mm thick at the root, thinning to 0.12 mm at the tip


def _strands(length, width, along, n, seed, at=np.zeros(3), lie_deg=LIE_DEG):
    """n strands of a lock, in the lock frame (x out along the lock, y across it, z further down the head). Each grows
    from its own point of the scalp (spread over `width` across and `along` up and down the head), leaves the scalp
    more upright than the lock lies and bends down under its weight toward the tip, drifting a little sideways; each is
    up to 3 mm short of `length` at the tip (cut hair is never perfectly level)."""
    rng = np.random.default_rng(seed)
    t = math.radians(lie_deg)
    dn = np.array([math.sin(t), 0.0, math.cos(t)])  # down the head, in the lock frame
    nn = np.array([math.cos(t), 0.0, -math.sin(t)])  # straight out of the scalp
    k = np.linspace(0.0, 1.0, 6)
    pts, cnt, wid, col = [], [], [], []
    for i in range(n):
        root = at + rng.uniform(-width / 2, width / 2) * np.array([0, 1.0, 0]) + rng.uniform(-along / 2, along / 2) * dn \
            - 0.0003 * nn
        ln = length - (rng.uniform(0, 0.003) if rng.random() < 0.6 else 0.0)
        phi0, phi1 = rng.uniform(-0.45, -0.25), rng.uniform(0.15, 0.35)  # leaves upright, bends down (rad, about y)
        drift = rng.uniform(-0.12, 0.12)
        phi = phi0 + (phi1 - phi0) * k
        tang = np.stack([np.cos(phi), np.full_like(phi, drift), np.sin(phi)], axis=1)
        tang /= np.linalg.norm(tang, axis=1, keepdims=True)
        seg = ln / (len(k) - 1)
        p = [root]
        for j in range(1, len(k)):
            p.append(p[-1] + seg * (tang[j - 1] + tang[j]) / np.linalg.norm(tang[j - 1] + tang[j]))
        pts += p
        cnt.append(len(k))
        wid += list(STRAND_W[0] + (STRAND_W[1] - STRAND_W[0]) * k)
        g = rng.uniform(0.75, 1.3)
        col.append(np.clip(np.array(HAIR) * g + rng.uniform(-0.02, 0.02, 3), 0, 1))
    return pts, cnt, wid, col


def lock_shapes(length, width, at=(0.0, 0.0, 0.0), strands=4, seed=0, n_hair=80, along=0.005):
    """A lock from its root at `at` (lock frame) out to `length` along x."""
    at = np.asarray(at, float)
    if HAIR_STYLE == "strands":
        return [hk.curves(*_strands(length, width, along, n_hair, seed, at))]
    out = [hk.box((length - 0.0015, width, H_LAYER), at=at + ((length - 0.0015) / 2, 0, 0), color=HAIR)]
    for i, y in enumerate(np.linspace(-width / 2 + STRAND, width / 2 - STRAND, strands)):
        r = RAGGED[(i + seed) % len(RAGGED)] if i % 2 else 0.0  # every other strand reaches the lock's full length
        ln = length - r
        z = (H_LAYER + STRAND) / 2 * (1 if (i + seed) % 2 else -1)
        out.append(hk.box((ln, STRAND, STRAND), at=at + (ln / 2, y, z), color=HAIR_SHADES[(i + seed) % 3]))
    return out


def _width(h, deg):
    return float(rad(h)) * math.radians(deg) + 0.002  # the lock's arc, plus 2 mm overlap with its neighbours


LOCK_AZ = PANEL_C + np.linspace(-30.0, 30.0, 7)
LOCK_H = np.linspace(PANEL_H[0] + 0.0025, PANEL_H[1] - 0.0025, 16)
LOCKS = []  # (name, azimuth, height, root, normal, grid indices)
for r_, h in enumerate(LOCK_H):
    for c_, a in enumerate(LOCK_AZ):
        p, n = scalp(a, h)
        idx = [k for k, (ga, gh) in enumerate(_G) if abs(ga - a) <= 5.0 + 1e-9 and abs(gh - h) <= 0.0025 + 1e-9]
        name = f"hair_{r_:02d}_{c_}"
        LOCKS.append((name, a, h, p, n, idx))
        R, root = lock_frame(a, h)
        kit.part(name, origin=tuple(root), rot=R, color=HAIR, collide=False,
                 shapes=lock_shapes(L0, _width(h, 10.0), strands=6, seed=r_ * 7 + c_, n_hair=90))
# each strand belongs to the scored point nearest its root: it is drawn at that point's length, so the video shows
# exactly the cut that is scored (to the 2.5 degree x 2.5 mm grid)
STRAND_G = []
for lk in LOCKS:
    sh = kit.parts[lk[0]].shapes[0]
    if sh.kind != "curves":
        STRAND_G.append(None)
        continue
    pts, cnt = sh.data[0], sh.data[1]
    R, root = lock_frame(lk[1], lk[2])
    roots = pts[np.r_[0, np.cumsum(cnt)[:-1]]] @ R.T + root
    idx = np.array(lk[5])
    STRAND_G.append(idx[np.argmin(np.linalg.norm(roots[:, None, :] - G_BASE[idx][None], axis=2), axis=1)])
LOCK_ROOTS = np.array([lk[3] for lk in LOCKS])
LOCK_NORMALS = np.array([lk[4] for lk in LOCKS])


def _around(name, azs, rows, length_of):
    """The hair already cut around the panel, static: per azimuth, rows of locks grouped into parts of a few rows each
    (each group in the middle row's frame; within 2 cm of head the hair's direction turns by under 3 degrees)."""
    out = []
    for c_, a in enumerate(azs):
        for g, grp in enumerate(rows):
            R, p0 = lock_frame(a, grp[len(grp) // 2])
            shapes = []
            for j, h in enumerate(grp):
                p, _ = scalp(a, h)
                shapes += lock_shapes(length_of(h), _width(h, 10.0), at=R.T @ (p - p0), strands=3,
                                   seed=1000 + 97 * c_ + 13 * g + j, n_hair=60, along=float(grp[1] - grp[0]) if len(grp) > 1 else 0.005)
            out.append(kit.part(f"{name}_{c_}_{g}", origin=tuple(p0), rot=R, color=HAIR, collide=False, shapes=shapes))
    return out


_SIDE_H = [list(PANEL_H[0] + 0.0025 + 0.005 * np.arange(4 * g, 4 * g + 4)) for g in range(4)]
_TOP_H = [[0.190, 0.200, 0.210], [0.220, 0.230, 0.240]]
_NAPE_H = [[0.0925, 0.0975]]
AROUND = (_around("cut_side", [PANEL_C + s * a for s in (-1, 1) for a in (40.0, 50.0, 60.0, 70.0)], _SIDE_H,
                  lambda h: float(target(h)))
          + _around("cut_top", PANEL_C + np.arange(-70.0, 71.0, 10.0), _TOP_H, lambda h: T_HI)
          + _around("cut_nape", PANEL_C + np.arange(-70.0, 71.0, 10.0), _NAPE_H,
                    lambda h: 0.006 + 0.002 * (h - 0.090) / (PANEL_H[0] - 0.090)))
PB_NOCOLLIDE = [lk[0] for lk in LOCKS] + AROUND  # hair is drawn, not simulated: nothing collides with it

OBJECTS = kit.objects()


def lock_pose(k, length=None, droop_deg=DROOP_DEG):
    """Lock k at its root, `droop_deg` down the head from standing straight out (its length is in its points)."""
    name, a, h = LOCKS[k][:3]
    R, root = lock_frame(a, h, droop_deg)
    return hk.T(R, root)


def lock_points(k):
    """Lock k's strands, each cut to the length of its scored point: it keeps its root end and its shape, cut across
    where the blade passed (its own raggedness scaled with it); the same points per strand, so the curves are redrawn
    in place. (The boxes fallback can't be cut in place: it is moved out along itself instead, the rest in the head.)"""
    pts, cnt, wid, col = kit.parts[LOCKS[k][0]].shapes[0].data
    lens = MEM["length"][STRAND_G[k]]
    if float(lens.min()) >= L0 - 1e-6:
        return pts
    out = np.empty_like(pts)
    o = 0
    for i, n in enumerate(cnt):
        P = pts[o:o + n]
        arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
        keep = arc[-1] * float(lens[i]) / L0
        s_ = np.linspace(0.0, keep, n)
        for j in range(3):
            out[o:o + n, j] = np.interp(s_, arc, P[:, j])
        o += n
    return out


# -- the cut ------------------------------------------------------------------------------------------------------
CONTROL_HZ = 20.0
DEBUG = os.environ.get("CC_DEBUG") == "1"
MEM: dict = {}


def _reset() -> None:
    MEM.clear()
    MEM.update(t=0.0, length=np.full(len(_G), L0), ear=False, cuts=0, shaved=0, droop=[DROOP_DEG] * len(LOCKS))


_reset()
kit.on_reset(_reset)


def _cross(frame, origin_local, half_a, half_b, base, u):
    """Where each hair line base + s*u crosses a face of a part: the face through local point `origin_local` with local
    normal x, bounded by +-half_a along local y and +-half_b along local z. Returns s per line (NaN: misses)."""
    R, p = frame[:3, :3], frame[:3, 3]
    n = R[:, 0]
    c = R @ origin_local + p
    den = u @ n
    with np.errstate(divide="ignore", invalid="ignore"):
        s = ((c - base) @ n) / den
    q = base + s[:, None] * u - c
    ok = (np.abs(den) >= 0.3) & (s >= -0.002) & (s <= L0 + 0.02) & \
        (np.abs(q @ R[:, 1]) <= half_a) & (np.abs(q @ R[:, 2]) <= half_b)
    return np.where(ok, s, np.nan)


def cut(fc, fk) -> None:
    """One control step of cutting, from the clipper's and the comb's poses. Where the blade face crosses a hair line
    below the hair's length: if the comb crosses that line nearer the scalp (or within its own thickness of the
    blade), the hair stands out through the teeth and is cut at the blade, never shorter than the comb's back (the
    teeth shield what is inside them); with no comb there, the hair lies flat under the clipper, which shaves it to
    1 mm where the blade passes within 6 mm of the scalp and otherwise leaves it."""
    L = MEM["length"]
    sb = _cross(fc, np.array([BLADE_X, 0.0, 0.0]), CLIP_W / 2, BLADE_H / 2, G_BASE, G_N)
    hit = ~np.isnan(sb) & (sb < L)
    if not hit.any():
        return
    sc = _cross(fk, np.zeros(3), COMB_L / 2, COMB_W / 2, G_BASE, G_N)
    combed = hit & ~np.isnan(sc) & (np.nan_to_num(sc, nan=1.0) < sb + COMB_T + 0.002)
    flat = hit & ~combed & (sb <= FLAT_REACH)
    new = L.copy()
    new[combed] = np.maximum(sb[combed], sc[combed] + COMB_T)
    new[flat] = 0.001
    changed = new < L - 1e-4
    MEM["cuts"] += int(changed.sum())
    MEM["shaved"] += int((changed & flat).sum())
    MEM["length"] = np.minimum(L, new)


EAR_CLEAR = 0.010
EAR_POINTS = np.array([[HX + dx, HY + sgn * (0.094 + dy), TOP + sum(EAR_H) / 2 + HZ + dz]
                       for sgn in (1, -1) for dx in np.linspace(-0.015, 0.015, 4) for dy in (-0.004, 0.004)
                       for dz in np.linspace(-(EAR_H[1] - EAR_H[0]) / 2, (EAR_H[1] - EAR_H[0]) / 2, 6)])  # pad surface
_BOXES = {"clipper": (np.zeros(3), np.array([CLIP_L, CLIP_W, CLIP_H]) / 2),
          "comb": (np.array([0.0, HANDLE[1] / 2, 0.0]), np.array([HANDLE[0], COMB_L + HANDLE[1], COMB_W]) / 2)}


def ears(frames: dict) -> None:
    for part, (c, half) in _BOXES.items():  # a tool within 1 cm of an ear pad (any point of it) touches it
        t = frames[part]
        loc = (EAR_POINTS - t[:3, 3]) @ t[:3, :3] - c
        if float(np.min(np.linalg.norm(np.maximum(np.abs(loc) - half, 0), axis=1))) < EAR_CLEAR:
            if not MEM["ear"]:
                MEM["ear_t"] = (round(MEM["t"], 2), part)
            MEM["ear"] = True


def droop(fk) -> None:
    """Display: a lock stands straight out while the comb's plate is in it, and lies down the head otherwise."""
    sc = _cross(fk, np.zeros(3), COMB_L / 2, COMB_W / 2, LOCK_ROOTS, LOCK_NORMALS)
    for k in range(len(LOCKS)):
        goal = 0.0 if not np.isnan(sc[k]) and sc[k] < lock_length(k) else DROOP_DEG
        MEM["droop"][k] += float(np.clip(goal - MEM["droop"][k], -DROOP_RATE, DROOP_RATE))


def section_dev() -> list:
    """Debug: per quarter of the panel (right to left), the median length error (mm) of the points cut so far, and of
    its bottom and top halves: what to tune FACE / FACE_TILT with after an Isaac run."""
    d = (MEM["length"] - G_TARGET).reshape(NR, NC) * 1000
    cut_ = (MEM["length"] < L0 - CUT_BY).reshape(NR, NC)
    out = []
    for q in range(4):
        cols = slice(7 * q, 7 * q + 7)
        vals = [d[rows, cols][cut_[rows, cols]] for rows in (slice(0, NR), slice(0, NR // 2), slice(NR // 2, NR))]
        out.append([round(float(np.median(v)), 1) if v.size else None for v in vals])
    return out


def lock_length(k) -> float:
    return float(np.max(MEM["length"][LOCKS[k][5]]))


def monitor(objects, grippers=None) -> None:
    MEM["t"] += 1.0 / CONTROL_HZ
    if grippers is not None:
        MEM["tips"] = [[round(float(v), 3) for v in g] for g in grippers]
    fc, fk = kit.frame(objects, "clipper"), kit.frame(objects, "comb")
    if DEBUG and abs(MEM["t"] / 2.0 - round(MEM["t"] / 2.0)) < 1e-6:  # every 2 s in Isaac
        tc = [[round(float(v), 3) for v in g] for g in hk.tcps()]
        print(f"[clipper_comb dbg] t={MEM['t']:.1f} clip={np.round(fc[:3, 3], 3).tolist()} "
              f"comb={np.round(fk[:3, 3], 3).tolist()} comb_up={fk[2, 2]:.2f} tcps={tc} "
              f"right={right_count()} cuts={MEM['cuts']} min={1000 * float(MEM['length'].min()):.1f} ear={MEM['ear']} "
              f"dev_by_section={section_dev()} {fails()}", flush=True)
    cut(fc, fk)
    droop(fk)
    ears({"clipper": fc, "comb": fk})


kit.on_step(monitor)


def kinematic_poses(angles: dict | None = None) -> dict:
    if HAIR_STYLE != "strands":  # boxes: a lock is cut shorter by sliding it out along itself, the rest in the head
        return {LOCKS[k][0]: lock_pose(k, droop_deg=MEM["droop"][k]) @ hk.T(None, (lock_length(k) - L0, 0, 0))
                for k in range(len(LOCKS))}
    return {LOCKS[k][0]: lock_pose(k, droop_deg=MEM["droop"][k]) for k in range(len(LOCKS))}


def strand_points() -> dict:
    return {LOCKS[k][0]: lock_points(k) for k in range(len(LOCKS))} if HAIR_STYLE == "strands" else {}


kit.on_kinematic(kinematic_poses)
kit.on_curves(strand_points)


# -- scoring ------------------------------------------------------------------------------------------------------
CUT_BY = 0.001  # a point counts as cut once it is at least 1 mm shorter than it started
LONG_CREDIT = 0.25  # cut but still too long (or on a line): the technique worked and it can still be trimmed


def lines() -> np.ndarray:
    """Per grid point: whether it steps by more than 2 mm from a neighbour (up, down, left or right), beyond the
    taper's own change between them. The panel has to blend into the hair already cut around it: beside the panel and
    above it, the hair is already at the taper's length, so the panel's side and top edges are checked against that
    too (below the panel is the hairline: nothing to blend into)."""
    g = np.zeros((NR + 1, NC + 2))  # already-cut hair around the panel: on the taper exactly
    g[:NR, 1:-1] = (MEM["length"] - G_TARGET).reshape(NR, NC)
    bad = np.zeros_like(g, bool)
    dv = np.abs(np.diff(g, axis=0)) > LINE
    dh = np.abs(np.diff(g, axis=1)) > LINE
    bad[:-1] |= dv
    bad[1:] |= dv
    bad[:, :-1] |= dh
    bad[:, 1:] |= dh
    return bad[:NR, 1:-1].ravel()


def credits() -> np.ndarray:
    """Per grid point: 1 if right (2 mm short to 3 mm long of the taper at its height) and on no line; 0.25 if cut but
    still too long or on a line; 0 if uncut or too short."""
    L = MEM["length"]
    d = L - G_TARGET
    right = (d >= -TOL_SHORT) & (d <= TOL_LONG)
    line = lines()
    out = np.where(right & ~line, 1.0, 0.0)
    out = np.where((out == 0.0) & (L <= L0 - CUT_BY) & (d >= -TOL_SHORT), LONG_CREDIT, out)
    return out


def right_count() -> int:
    return int((credits() == 1.0).sum())


def fails() -> str:
    """Debug: the points not right, by kind, and where (rows bottom to top, columns right to left, as 8 x 7 blocks)."""
    L = MEM["length"]
    d = L - G_TARGET
    cr = credits()
    bad = cr < 1.0
    ok_len = (d >= -TOL_SHORT) & (d <= TOL_LONG)
    kinds = {"long": int((bad & (d > TOL_LONG)).sum()), "short": int((bad & (d < -TOL_SHORT)).sum()),
             "line": int((bad & ok_len).sum())}
    b = bad.reshape(NR, NC)
    blocks = [[int(b[r:r + 4, c:c + 4].sum()) for c in range(0, NC, 4)] for r in range(0, NR, 4)]
    return f"fails={kinds} blocks(bottom->top rows, right->left cols)={blocks}"


def on_mat(objects, part) -> bool:
    """A tool put back: put down where it started. The comb resting in or beside its rack (centre within
    3 cm of its start, sideways, no more than 1 cm higher); the clipper lying flat (within 20 degrees) on its holder
    or beside it (centre within 6 cm of its start, sideways, no more than 1 cm higher)."""
    t = kit.frame(objects, part)
    if part == "comb":
        return float(np.linalg.norm(t[:2, 3] - COMB_AT[:2])) < 0.03 and float(t[2, 3]) < COMB_AT[2] + 0.01
    flat = float(t[2, 2]) >= math.cos(math.radians(20))
    return float(np.linalg.norm(t[:2, 3] - CLIP_AT[:2])) < 0.06 and float(t[2, 3]) < CLIP_AT[2] + 0.01 and flat


def score(objects) -> float:
    """0 once any point is under 5 mm (a bald patch) or a tool has touched an ear. Otherwise 1.0 when every point is
    right and on no line and both tools are back; else 0.9 x the average point credit (0 to 0.9, rounded to 0.001)."""
    if MEM["ear"] or float(MEM["length"].min()) < GOUGE:
        return 0.0
    c = credits()
    if float(c.min()) == 1.0 and on_mat(objects, "comb") and on_mat(objects, "clipper"):
        return 1.0
    return round(0.9 * float(c.mean()), 3)


def success(objects) -> bool:
    return score(objects) >= 0.999


def distance(objects) -> float:
    return float(len(_G) - right_count())


def log_state(objects) -> dict:
    fc, fk = kit.frame(objects, "clipper"), kit.frame(objects, "comb")
    L = MEM["length"]
    d = (L - G_TARGET).reshape(NR, NC)
    return {"right": right_count(), "of": len(_G), "lines": int(lines().sum()), "min_mm": round(1000 * float(L.min()), 1),
            "err_mm_by_band": [[round(1000 * float(v), 1) for v in (d[r].min(), d[r].max())] for r in range(0, NR, 4)],
            "dev_by_section": section_dev(), "uncut": int((L > L0 - CUT_BY).sum()), "ear": MEM["ear"], "cuts": MEM["cuts"], "shaved": MEM["shaved"],
            "clip": [round(float(v), 3) for v in fc[:3, 3]], "comb": [round(float(v), 3) for v in fk[:3, 3]],
            "comb_up": round(float(fk[2, 2]), 3), "back": [on_mat(objects, "comb"), on_mat(objects, "clipper")],
            "ear_t": MEM.get("ear_t"), "tips": MEM.get("tips")}


# -- scripted demo ------------------------------------------------------------------------------------------------
def _target(self, t):
    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in self.waypoints:
        goal = np.asarray(w.pos, dtype=float)
        rot = getattr(w, "rot", None)
        rot = grip_rotation(w.yaw) if rot is None else np.asarray(rot, float)
        if t < w.duration:
            s = ik.smoothstep(t / w.duration) if getattr(w, "smooth", True) else t / w.duration
            return prev_pos + s * (goal - prev_pos), ik.slerp_matrix(prev_rot, rot, s), \
                prev_finger + s * (w.finger - prev_finger), w.label
        t -= w.duration
        prev_pos, prev_rot, prev_finger = goal, rot, w.finger
    return prev_pos, prev_rot, prev_finger, "done"


_demo.ArmPlan.target = _target


class RotWaypoint(Waypoint):
    def __init__(self, duration, pos, finger, label="", rot=None, smooth=True):
        super().__init__(duration, tuple(float(v) for v in pos), finger, label, 0.0)
        object.__setattr__(self, "rot", None if rot is None else np.asarray(rot, float))
        object.__setattr__(self, "smooth", smooth)


OPEN, SHUT = -0.040, 0.0
# grasps, as the TCP pose in the part frame (top-down on the mat)
_RX = (lambda a: np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]]))
COMB_GRASP = hk.T(_RX(math.radians(-float(os.environ.get("CC_CTILT", "30")))) @ grip_rotation(-math.pi / 2), (-COMB_T / 2, float(os.environ.get("CC_GY", "0.069")), COMB_W / 2 - float(os.environ.get("CC_GZ", "0.006"))))  # onto the handle by the teeth, over the centre of mass, tilted 30 degrees in from the left (keeps the two elbows apart)
CLIP_GRASP = hk.T(grip_rotation(0.0), (float(os.environ.get("CC_CGX", "0.0")), 0.0, float(os.environ.get("CC_CGZ", "0.008"))))
# Where the clipper actually sits relative to where the hand is told to go, once gripped: in Isaac it ends up 2.2 cm
# to one side (along the jaws) and 5 mm lower than a centred grip would put it, the same every run (fitted from two
# runs: the offset moved one-for-one with the hand). The demo aims with this, and puts the clipper back with it.
CLIP_HOLD = hk.T(grip_rotation(0.0), (0.0, float(os.environ.get("CC_HOLD_DY", "0.022")),
                                     float(os.environ.get("CC_HOLD_DZ", "0.013"))))  # down onto its middle
COMB_OUT = float(os.environ.get("CC_COMB_OUT", "-0.0005"))  # the comb's back (outer face) this far past the taper
FACE = float(os.environ.get("CC_FACE", "-0.0005"))  # the clipper's blade face this far past the taper (pressed on the comb's back) ...
FACE_TILT = float(os.environ.get("CC_FACE_TILT", "0.0015"))  # ... plus this per 20 degrees toward the robot's left (Isaac)
CLEAR = float(os.environ.get("CC_CLEAR", "0.012"))  # the clipper's stand-off from the comb between passes
CLIP_OUT = 0.05  # the clipper waits this far out while the comb changes section
COMB_SHIFT = float(os.environ.get("CC_SHIFT", "0.040"))  # the section's middle sits 4 cm from the comb's middle, toward its free end (keeps the hands apart)
# A barber's sequence: the back in vertical sections, each from the nape up in overlapping bands.
SECTIONS = tuple(PANEL_C + a for a in (-26.25, -8.75, 8.75, 26.25))  # section middles (degrees), 17.5 degrees wide, right to left
ALL_SECTIONS = SECTIONS
if os.environ.get("CC_SECTIONS"):  # offline testing only: a subset of the sections, e.g. "0,1"
    SECTIONS = tuple(SECTIONS[int(i)] for i in os.environ["CC_SECTIONS"].split(","))
BANDS = tuple(PANEL_H[0] + 0.008 + 0.016 * k for k in range(5))  # band middles: the blade cuts 2 cm, 1.6 cm apart
# The clipper's middle runs between these two points along the comb, measured from the section's middle (+ toward
# the comb's handle). It stays on the side away from the comb hand, so the two hands keep 7.5 cm apart; its 4.5 cm
# blade still sweeps the whole section and into the next one.
SLIDE = (float(os.environ.get("CC_SLIDE_A", "-0.024")), float(os.environ.get("CC_SLIDE_B", "-0.008")))


# The scripted demo aims its comb (and the clipper with it) closer than the taper by TRIM_LO mm at the panel's bottom
# edge, fading evenly to TRIM_HI at its top: in Isaac the arms hold the comb that much farther out when they reach low
# (measured over runs at 20 Hz). Scoring is unchanged.
TRIM_LO = float(os.environ.get("CC_TRIM_LO", "7.5")) / 1000
TRIM_HI = float(os.environ.get("CC_TRIM_HI", "0")) / 1000


# ... plus, per section (right to left), this much more at the bottom edge, fading to nothing at the top (mm).
TRIM_SEC = [float(v) / 1000 for v in os.environ.get("CC_TRIM_SEC", "0.8,0,-0.6,-0.4").split(",")]


def plan_taper(h):
    f = np.clip((np.asarray(h, float) - PANEL_H[0]) / (PANEL_H[1] - PANEL_H[0]), 0.0, 1.0)
    return target(h) - (TRIM_LO + (TRIM_HI - TRIM_LO) * f)


def section_taper(taper, az):
    """The demo's aim for section az: plan_taper with that section's own extra trim."""
    if taper is not plan_taper:
        return taper
    e = TRIM_SEC[int(np.argmin([abs(az - a) for a in ALL_SECTIONS]))]

    def t(h):
        f = np.clip((np.asarray(h, float) - PANEL_H[0]) / (PANEL_H[1] - PANEL_H[0]), 0.0, 1.0)
        return plan_taper(h) - e * (1.0 - f)
    return t


def frame_at(az, h, taper=target):
    """The comb's (and the clipper's) orientation for section az at band height h: x into the head along the scalp's
    normal, z up the head, leaning out from it as fast as the taper gets longer, y along the comb, across the head."""
    def outer(hh):
        p, n = scalp(az, hh)
        return p + n * float(taper(hh))
    up = outer(h + 0.002) - outer(h - 0.002)
    up /= np.linalg.norm(up)
    n = scalp(az, h)[1]
    y = np.cross(up, -n)
    y /= np.linalg.norm(y)
    x = np.cross(y, up)
    return np.column_stack([x, y, up])


def comb_pose(az, h, out=0.0, taper=target):
    """The comb in the hair: its back (outer face) at the taper's length (+COMB_OUT) from the scalp at (az, h), backed
    off `out` level away from the head, slid COMB_SHIFT along itself so the section is near its free end."""
    p, n = scalp(az, h)
    R = frame_at(az, h, taper)
    back = p + n * (float(taper(h)) + COMB_OUT)
    return hk.T(R, back + R[:, 0] * COMB_T + R[:, 1] * COMB_SHIFT + _u(az) * out)  # backs off level, not down the normal


def clip_pose(az, h, slide, out=0.0, taper=target, face=None, face_tilt=None):
    """The clipper with its blade face along the comb's back, `slide` along the comb from the section's middle,
    backed off `out` level away from the head."""
    p, n = scalp(az, h)
    R = frame_at(az, h, taper)
    tilt = FACE_TILT if face_tilt is None else face_tilt
    f = (float(taper(h)) + FACE + tilt * az / 20.0) if face is None else face
    blade = p + n * f + R[:, 1] * slide + _u(az) * out  # backs off level, not down the normal
    return hk.T(R, blade - R @ np.array([BLADE_X, 0.0, 0.0]))


def tcp(part_pose, grasp):
    t = part_pose @ grasp
    return t[:3, 3], t[:3, :3]


def demo(sections=SECTIONS, bands=BANDS, taper=plan_taper, comb=True, put_back=True, face=None, slide=SLIDE, face_tilt=None):
    ends = {-1.0: slide[0], 1.0: slide[1]}
    """Both arms pick up their tool. Then, section by section across the back, from the nape up band by band: the left
    arm sets the comb in the hair, tilted to the taper, and the right runs the clipper along its back across the
    section (back and forth, alternating), backs off while the comb moves up to the next band, and comes back in.
    Then both put their tools back. (The keyword arguments make the offline test strategies.)"""
    left, right = [], []

    def both(dur, lpose, rpose, lf, rf, label, smooth=True):
        lp_, lr_ = lpose
        rp_, rr_ = rpose
        left.append(RotWaypoint(dur, lp_, lf, label, lr_, smooth))
        right.append(RotWaypoint(dur, rp_, rf, label, rr_, smooth))

    c0, k0 = hk.T(None, COMB_AT), hk.T(None, CLIP_AT)
    lp, lr = tcp(c0, COMB_GRASP)
    rp, rr = tcp(k0, CLIP_GRASP)
    rh, rhr = tcp(k0, CLIP_HOLD)  # the hand that puts the clipper back exactly on its holder
    up = np.array([0, 0, 0.08])
    both(2.0, (lp + up, lr), (rp + up, rr), OPEN, OPEN, "to the tools")
    both(1.2, (lp, lr), (rp, rr), OPEN, OPEN, "onto them")
    both(0.8, (lp, lr), (rp, rr), SHUT, SHUT, "grip")
    both(1.2, (lp + up, lr), (rp + up, rr), SHUT, SHUT, "lift")
    away = (lp + up, lr)
    if comb is False:  # the comb stays on the mat: only the clipper works
        def C(az, h, out=0.0):
            return away
    else:
        def C(az, h, out=0.0):
            return tcp(comb_pose(az, h, out, section_taper(taper, az)), COMB_GRASP)

    def K(az, h, sl, out=0.0):
        return tcp(clip_pose(az, h, sl, out, section_taper(taper, az), face, face_tilt), CLIP_HOLD)

    first = True
    for s_i, az in enumerate(sections):
        sign = -1.0
        h0 = bands[0]
        tag = f"section {s_i + 1}"
        both(2.5 if first else 1.5, C(az, h0, CLEAR), K(az, h0, ends[sign], CLIP_OUT), SHUT, SHUT, f"{tag}: comb comes in")
        first = False
        both(0.8, C(az, h0), K(az, h0, ends[sign], CLIP_OUT), SHUT, SHUT, f"{tag}: comb into the hair at the nape")
        both(0.8, C(az, h0), K(az, h0, ends[sign], CLEAR), SHUT, SHUT, f"{tag}: clipper comes in")
        for b_i, h in enumerate(bands):
            btag = f"{tag} band {b_i + 1}"
            if b_i > 0:
                hp = bands[b_i - 1]
                both(0.4, C(az, hp), K(az, hp, ends[sign], CLEAR), SHUT, SHUT, f"{btag}: clipper off the comb")
                both(0.8, C(az, h), K(az, h, ends[sign], CLEAR), SHUT, SHUT, f"{btag}: comb up a band")
            both(0.6, C(az, h), K(az, h, ends[sign]), SHUT, SHUT, f"{btag}: clipper onto the comb")
            both(0.4, C(az, h), K(az, h, ends[sign]), SHUT, SHUT, f"{btag}: settle")
            both(1.8, C(az, h), K(az, h, ends[-sign]), SHUT, SHUT, f"{btag}: run the clipper along the comb", smooth=False)
            sign = -sign
        hl = bands[-1]
        both(0.4, C(az, hl), K(az, hl, ends[sign], CLEAR), SHUT, SHUT, f"{tag}: clipper off")
        both(0.6, C(az, hl), K(az, hl, ends[sign], CLIP_OUT), SHUT, SHUT, f"{tag}: clipper back")
        both(0.6, C(az, hl, CLEAR), K(az, hl, ends[sign], CLIP_OUT), SHUT, SHUT, f"{tag}: comb out")
    if not put_back:
        both(2.5, away, (rh + up, rhr), SHUT, SHUT, "hold the tools up")
        return {"left": left, "right": right}
    both(2.5, (lp + up, lr), (rh + up, rhr), SHUT, SHUT, "back over the mat")
    both(1.0, (lp + [0, 0, 0.03], lr), (rh + [0, 0, 0.03], rhr), SHUT, SHUT, "line up over the rack and holder")
    both(1.0, (lp + [0, 0, 0.03], lr), (rh + [0, 0, 0.03], rhr), SHUT, SHUT, "hold")
    seat = np.array([0, 0, float(os.environ.get("CC_SEAT", "-0.003"))])  # aim a little low: the comb seats fully
    rseat = np.array([0, 0, float(os.environ.get("CC_CSEAT", "-0.004"))])  # aim a little low: it rests on the holder
    both(1.5, (lp + seat, lr), (rh + rseat, rhr), SHUT, SHUT, "set down")
    both(1.5, (lp + seat, lr), (rh + rseat, rhr), SHUT, SHUT, "settle")
    both(0.8, (lp + seat, lr), (rh + rseat, rhr), OPEN, OPEN, "let go")
    both(1.2, (lp + up, lr), (rh + up, rhr), OPEN, OPEN, "away")
    return {"left": left, "right": right}


def replay(plans: dict) -> dict:
    """Offline: run the cut along a demo plan with each tool exactly where the plan puts it (no physics), from the
    grip to the let-go; returns the final tool poses."""
    from yam_arena import station
    arm = {side: _demo.ArmPlan(way, station.BASES[side]) for side, way in plans.items()}
    held = {"left": False, "right": False}
    grasp = {"left": COMB_GRASP, "right": CLIP_HOLD}
    pose = {"comb": hk.T(None, COMB_AT), "clipper": hk.T(None, CLIP_AT)}
    tool = {"left": "comb", "right": "clipper"}
    _reset()
    dur = max(a.duration for a in arm.values())
    t = 0.0
    while t < dur + 0.5:
        for side, a in arm.items():
            p, R, f, label = a.target(t)
            if label == "lift":
                held[side] = True
            if label in ("let go", "away", "done"):
                held[side] = False
            if held[side]:
                pose[tool[side]] = hk.T(R, p) @ hk.inv(grasp[side])
        MEM["t"] += 1.0 / CONTROL_HZ
        cut(pose["clipper"], pose["comb"])
        ears(pose)
        t += 1.0 / CONTROL_HZ
    return pose


def _objs(pose):
    d = {n: kit.state(n) for n in kit.parts}
    d["comb"] = kit.state("comb", pose=pose["comb"])
    d["clipper"] = kit.state("clipper", pose=pose["clipper"])
    return d


def _selftest() -> None:
    from yam_arena import station
    for side, way in demo().items():
        plan = _demo.ArmPlan(way, station.BASES[side])
        t = 0.0
        while t < plan.duration:
            plan.action7(t)
            t += 1 / CONTROL_HZ
        print(f"{side}: {plan.duration:.1f} s plan ({plan.duration * CONTROL_HZ:.0f} steps), "
              f"worst IK {plan.worst_ik_mm:.2f} mm / {plan.worst_ik_deg:.2f} deg")
    global FACE, TRIM_LO, TRIM_HI, TRIM_SEC
    isaac = (FACE, TRIM_LO, TRIM_HI, TRIM_SEC)
    FACE, TRIM_LO, TRIM_HI, TRIM_SEC = 0.0005, 0.0, 0.0, [0.0] * 4  # without the corrections for Isaac's arms
    pose = replay(demo(face_tilt=0.0))
    FACE, TRIM_LO, TRIM_HI, TRIM_SEC = isaac
    st = log_state(_objs(pose))
    print(f"scripted demo (no physics, Isaac corrections off): score {score(_objs(pose))}  right {st['right']}/{st['of']}  lines {st['lines']}  "
          f"min {st['min_mm']} mm  err by band {st['err_mm_by_band']}  ear {st['ear']}")


if __name__ == "__main__":
    _selftest()

# pybullet preview only: weld each tool to its gripper once gripped (tests the reach, the comb's placement and the
# cut rule, not the grasp; Isaac runs the real grasp), and release it at the end.
PB_ATTACH = {"left": ("comb", "lift"), "right": ("clipper", "lift")}
PB_RELEASE = {"left": "let go", "right": "let go"}
PB_GRASP = {"left": COMB_GRASP, "right": CLIP_HOLD}  # the preview puts the clipper where Isaac's hand holds it
PB_VIEW = ([float(os.environ.get("CC_VX", "0.05")), float(os.environ.get("CC_VY", "-0.30")), TOP + float(os.environ.get("CC_VZ", "0.40"))], [HX - 0.06, HY, TOP + 0.10])
