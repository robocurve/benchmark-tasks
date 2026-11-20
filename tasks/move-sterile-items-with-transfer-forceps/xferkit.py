"""xferkit: stainless thumb dressing forceps (two sprung blades), a stainless medicine cup, a 1 mL syringe and iris
scissors, for yam_arena task files.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from xferkit import Blade, Cup, Forceps, Scissors, Syringe, on_reset, on_step

Frames (every part starts with its axes along the world's unless a ``quat`` is given: x away from the robot, y left,
z up):

- :class:`Forceps`: one blade of the forceps (the one on the +y side). ``pos`` is the tail, where the two blades are
  joined; its x runs along the blade to the tip, its y is the direction the blade moves when the forceps open, its z
  is across the blade's width. Lying "on edge" (as in the task) the blade's width is upright.
- :class:`Blade`: the other blade, same frame convention, mirrored in y. A sprung hinge about z at the tail (``tail``)
  joins the two; it holds the tips ``F_OPEN`` apart until they are squeezed.
- :class:`Cup`: ``pos`` is the middle of the underside of its base; z is its axis.
- :class:`Syringe`: ``pos`` is its centre of mass, on its axis; x runs along the axis from the thumb rest to the tip,
  y along the finger flange's long side.
- :class:`Scissors`: ``pos`` is their centre of mass; x runs from the finger rings to the points, y across the two
  rings, z through their thickness (they lie flat with z up).

All shapes are convex pieces built from point sets in NumPy when the task file is read (``*_pieces()``), so the same
numbers serve the scene, the scoring and the offline checks. Isaac Sim is reached only when the scene is built.

The structure follows the kits used by tasks already in this repository (Box subclasses so the loader accepts the
parts, custom spawners, hooks installed through ``sys.meta_path``, passive joints authored afresh at every reset).
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import re
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["Blade", "Cup", "Forceps", "KitState", "Scissors", "Syringe", "item_points", "near_item", "shank_hold", "shank_yaw", "arm_points", "blade_pieces", "cup_pieces", "cup_points", "env",
           "finger_points", "forceps_points", "in_cup", "install", "on_reset", "on_step", "place", "to_local",
           "turn_z", "write_pose"]

# -- the forceps (m): 14 cm (5.5 in) stainless thumb dressing forceps with serrated tips ------------------------------
F_LEN = 0.140                 # tail to tips (the catalogue size)
F_TH = 0.0015                 # each blade's thickness (an estimate)
F_OPEN = 0.015                # between the tips' inner faces at rest (an estimate)
# each blade's outline: distance from the tail, half width (estimates, from pictures of the instrument)
F_STATIONS = ((0.000, 0.0040), (0.006, 0.0050), (0.030, 0.0055), (0.090, 0.0055), (0.118, 0.0035), (0.140, 0.0015))
F_BOW = 0.0025                # the blades are bowed: with the tips together they still stand this far apart from the
F_BOW_AT = 0.030              # centre plane at their widest point, this far from the tail (estimates)
F_MASS = 0.025                # both blades together (an estimate: 3.2 cm3 of steel)
F_GRIP = 0.050                # the ribbed finger grip is centred this far from the tail
F_CLOSE_N = 2.5               # N: a squeeze at the finger grip that just brings the tips together (an estimate)
F_ANGLE = F_OPEN / F_LEN      # rad between the blades at rest
F_MIN = 0.0005 / F_LEN        # rad: the hinge stops with the tips 0.5 mm apart (they cannot pass through each other)
F_STIFF = F_CLOSE_N * F_GRIP / F_ANGLE      # N m / rad of the tail's spring
WORK = 0.050                  # the working end: this far back from the tips. It may touch only sterile things.
HANDLE = 0.065                # the handle: this far forward from the tail. It is not sterile once it is held.

# -- the cup: a 2 oz (59 ml) stainless-steel graduated medicine cup -----------------------------------------------------
C_TOP_R = 0.0255              # outside radius of the rolled rim (catalogue: 2 in across)
C_H = 0.0445                  # height (catalogue: 1 3/4 in)
C_BASE_R = 0.0185             # outside radius at the base (an estimate)
C_WALL = 0.0008               # sheet thickness (an estimate)
C_BEAD = 0.0012               # the rolled rim stands this far out from the wall (an estimate) ...
C_BEAD_H = 0.002              # ... and is this tall
C_MASS = 0.035                # an estimate (4.4 cm3 of steel)
C_SEG = 20                    # flat pieces round the wall

# -- the syringe: a 1 mL luer slip syringe without needle, plunger pushed in --------------------------------------------
# A distributor's table for a clear polypropylene 1 mL luer slip syringe gives the barrel: OD 0.27 in, length 2 11/16 in.
S_BARREL_R = 0.00343          # half of 0.27 in
S_BARREL_L = 0.0683           # 2 11/16 in
S_TIP = (0.0020, 0.010)       # the slip tip: radius, length (estimates)
S_FLANGE = (0.0100, 0.0055, 0.0015, 0.0030)   # finger flange: half length, half height, thickness, corner cut (estimates)
S_ROD = (0.0025, 0.011)       # the plunger rod showing behind the flange: half width (square), length (estimates)
S_THUMB = (0.0050, 0.0020)    # the thumb rest: radius, thickness (estimates)
S_MASS = 0.002                # an estimate (no listing gives the weight)
S_BODY = 0.010                # the simulated body's mass: five times the real one, for the solver's sake. A steady upward
                              # force carries the extra 8 g, so the syringe weighs what the real one does.
PLASTIC, PLUNGER = (0.90, 0.92, 0.94), (0.82, 0.86, 0.92)

# -- the scissors: straight iris scissors, 115 mm (4.5 in), stainless, closed ----------------------------------------------
X_LEN = 0.115                 # overall (the catalogue size)
X_WIRE = 0.0035               # the finger rings are round bar this thick (an estimate, like every size below)
X_RING = (0.0130, 0.0110)     # a ring's outside half length (along the scissors) and half width
X_RING_Y = 0.0128             # each ring's middle is this far from the centre line (the closed rings stand 3.6 mm apart)
X_RING_SEG = 12               # straight pieces round each ring
X_PIVOT_AT = 0.070            # the screw is this far from the back of the rings
X_PIVOT = (0.0040, 0.0040)    # the joint: radius, thickness
X_SHANK = (0.0035, 0.0030)    # each shank: width, thickness
X_BLADE = (0.0035, 0.0006, 0.0032, 0.0016)   # closed blades: half width at the joint and at the points, thickness there
X_MASS = 0.020                # an estimate (the shape below holds 3.0 cm3)

STEEL, STEEL_DARK = (0.72, 0.73, 0.75), (0.55, 0.56, 0.58)
FRICTION = (0.6, 0.5)         # serrated steel tips on steel, and steel on the drape (estimates)
PAD_PATCH = 0.004             # radius of the patch over which a rubber gripper pad meets a blade (an estimate)
ARM_LINKS = ("link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")


def half_width(x: float) -> float:
    """Half the blade's width at ``x`` m from the tail."""
    xs, ws = zip(*F_STATIONS)
    return float(np.interp(x, xs, ws))


def bow(x: float) -> float:
    """How far a blade's inner face stands from the forceps' centre plane at ``x`` m from the tail, tips together."""
    return float(np.interp(x, (0.0, F_BOW_AT, F_LEN), (0.0, F_BOW, 0.0)))


# -- shapes (pure): every piece is a set of points whose convex hull is the piece -----------------------------------------

def blade_pieces(side: float = 1.0) -> list:
    """One blade as (points, colour) pieces in its own frame: ``side`` +1 for the Forceps part (which lies on the +y
    side of the hinge), -1 for the Blade part. The blade is a bowed strip: straight from the tail out to its widest
    point, then straight in to the tip."""
    out = []
    xs = sorted({x for x, _ in F_STATIONS} | {F_BOW_AT})
    for xa, xb in zip(xs, xs[1:]):
        pts = [[x, side * (bow(x) + d), s * half_width(x)] for x in (xa, xb) for d in (0.0, F_TH) for s in (-1.0, 1.0)]
        out.append((np.array(pts), STEEL_DARK if xa >= 0.118 or 0.030 <= xa < 0.075 else STEEL))
    return out


def _wall_r(z: float) -> float:
    """The outside radius of the cup's wall at height ``z`` (without the rolled rim)."""
    return C_BASE_R + (C_TOP_R - C_BEAD - C_BASE_R) * z / C_H


def cup_pieces() -> list:
    """The cup as (points, colour) pieces in its own frame: a base disc, flat wall pieces, and the rolled rim."""
    out = []
    ang = 2 * math.pi * np.arange(C_SEG + 1) / C_SEG
    ring = np.column_stack([np.cos(ang[:-1]), np.sin(ang[:-1])])
    base = np.vstack([np.column_stack([_wall_r(z) * ring, np.full(C_SEG, z)]) for z in (0.0, C_WALL)])
    out.append((base, STEEL))
    for a0, a1 in zip(ang, ang[1:]):
        wall, bead = [], []
        for a in (a0, a1):
            c, s = math.cos(a), math.sin(a)
            for z in (C_WALL, C_H - C_BEAD_H):
                for r in (_wall_r(z) - C_WALL, _wall_r(z)):
                    wall.append([r * c, r * s, z])
            for z in (C_H - C_BEAD_H, C_H):
                for r in (_wall_r(C_H) - C_WALL, C_TOP_R):
                    bead.append([r * c, r * s, z])
        out.append((np.array(wall), STEEL))
        out.append((np.array(bead), STEEL))
    return out


def turn_z(angle: float) -> tuple:
    """Quaternion (xyzw) of a turn of ``angle`` rad about z."""
    return (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))


def hull_faces(points: np.ndarray, eps: float = 1e-7) -> list:
    """The faces of the convex hull of ``points`` (4 to about 40 of them): each a list of point indices, wound
    counter-clockwise seen from outside. Brute force over triples; coplanar triangles are merged into one polygon."""
    p = np.asarray(points, float)
    n = len(p)
    centre = p.mean(0)
    planes: dict = {}
    for i in range(n - 2):
        for j in range(i + 1, n - 1):
            e1 = p[j] - p[i]
            for k in range(j + 1, n):
                nrm = np.cross(e1, p[k] - p[i])
                size = float(np.linalg.norm(nrm))
                if size < 1e-14:
                    continue
                nrm = nrm / size
                d = (p - p[i]) @ nrm
                if d.max() > eps and d.min() < -eps:
                    continue
                if d.max() > eps:
                    nrm = -nrm
                key = tuple(np.round(np.r_[nrm, nrm @ p[i]], 5))
                planes.setdefault(key, (nrm, set()))[1].update((i, j, k))
    faces = []
    for nrm, idx in planes.values():
        idx = sorted(idx)
        c = p[idx].mean(0)
        u = p[idx[0]] - c
        u /= np.linalg.norm(u)
        v = np.cross(nrm, u)
        ang = [math.atan2((p[m] - c) @ v, (p[m] - c) @ u) for m in idx]
        order = [m for _, m in sorted(zip(ang, idx))]
        if nrm @ (c - centre) < 0:
            order = order[::-1]
        faces.append(order)
    return faces


def _ngon(r: float, n: int, flat: bool = True) -> np.ndarray:
    """A regular n-gon's corners (y, z). ``flat``: ``r`` is the distance to its sides (one of which faces +y)."""
    rr = r / math.cos(math.pi / n) if flat else r
    a = 2 * math.pi * (np.arange(n) + (0.5 if flat else 0.0)) / n
    return np.column_stack([rr * np.cos(a), rr * np.sin(a)])


def _prism(x0: float, x1: float, outline) -> np.ndarray:
    return np.array([[x, y, z] for x in (x0, x1) for y, z in outline])


def hull_volume(points) -> tuple:
    """Volume and centroid of the convex hull of ``points``."""
    p = np.asarray(points, float)
    o = p.mean(0)
    vol, mom = 0.0, np.zeros(3)
    for f in hull_faces(p):
        for i in range(1, len(f) - 1):
            a, b, c = p[f[0]] - o, p[f[i]] - o, p[f[i + 1]] - o
            v = float(a @ np.cross(b, c)) / 6.0
            vol += v
            mom += v * (a + b + c) / 4.0
    return vol, o + mom / vol


def hull_planes(points) -> np.ndarray:
    """The hull's faces as rows (nx, ny, nz, d): a point is outside a face by n . p - d."""
    p = np.asarray(points, float)
    centre = p.mean(0)
    rows = []
    for f in hull_faces(p):
        q = p[f]
        n = np.zeros(3)
        for i in range(len(q)):                         # Newell's method: sound for any polygon
            a, b = q[i], q[(i + 1) % len(q)]
            n += np.cross(a, b)
        n /= np.linalg.norm(n)
        if n @ (q.mean(0) - centre) < 0:
            n = -n
        rows.append(np.r_[n, n @ q[0]])
    return np.array(rows)


def _centred(pieces: list) -> tuple:
    """Shift pieces so that their centre of mass (uniform density) is the origin. Returns (pieces, volume, shift)."""
    vols = [hull_volume(p) for p, _ in pieces]
    total = sum(v for v, _ in vols)
    com = sum(v * c for v, c in vols) / total
    return [(p - com, col) for p, col in pieces], total, com


def _syringe_raw() -> list:
    """The syringe before centring: x from the flange's front face (0) toward the tip."""
    fl, fh, ft, cut = S_FLANGE
    flange = [(fl, fh - cut), (fl - cut, fh), (-fl + cut, fh), (-fl, fh - cut), (-fl, -fh + cut), (-fl + cut, -fh),
              (fl - cut, -fh), (fl, -fh + cut)]
    rod = [(S_ROD[0], S_ROD[0]), (-S_ROD[0], S_ROD[0]), (-S_ROD[0], -S_ROD[0]), (S_ROD[0], -S_ROD[0])]
    back = -ft - S_ROD[1]
    return [(_prism(0.0, S_BARREL_L, _ngon(S_BARREL_R, 12)), PLASTIC),
            (_prism(S_BARREL_L, S_BARREL_L + S_TIP[1], _ngon(S_TIP[0], 8)), PLASTIC),
            (_prism(-ft, 0.0, flange), PLASTIC),
            (_prism(back, -ft, rod), PLUNGER),
            (_prism(back - S_THUMB[1], back, _ngon(S_THUMB[0], 12)), PLUNGER)]


def _ring_nodes(sign: float) -> list:
    """Cross-sections round one finger ring (before centring): each an octagon of the bar, in the plane across it.
    The ring is ``X_RING_SEG`` straight pieces of bar whose middles lie on the ring's oval; one piece is the ring's
    back, another its front."""
    grow = 1.0 / math.cos(math.pi / X_RING_SEG)
    a, b = (X_RING[0] - X_WIRE / 2) * grow, (X_RING[1] - X_WIRE / 2) * grow
    oct_ = _ngon(X_WIRE / 2, 8)
    nodes = []
    for k in range(X_RING_SEG):
        t = 2 * math.pi * (k + 0.5) / X_RING_SEG
        c = np.array([X_RING[0] - a * math.cos(t), sign * X_RING_Y + b * math.sin(t), X_WIRE / 2])
        out = np.array([-math.cos(t) / a, math.sin(t) / b, 0.0])     # the oval's outward normal there
        out /= np.linalg.norm(out)
        nodes.append(np.array([c + u * grow * out + [0.0, 0.0, w] for u, w in oct_]))
    return nodes


def _scissors_raw() -> list:
    """The closed scissors before centring: x from the back of the rings (0) to the points, z up from the underside."""
    out = []
    for sign in (1.0, -1.0):
        nodes = _ring_nodes(sign)
        for k in range(X_RING_SEG):
            out.append((np.vstack([nodes[k], nodes[(k + 1) % X_RING_SEG]]), STEEL))
        w, th = X_SHANK
        x0, y0 = 2 * X_RING[0] - X_WIRE, sign * X_RING_Y          # the front of the ring's opening
        pts = [[x, y + s * w / 2, z] for x, y in ((x0, y0), (X_PIVOT_AT, 0.0)) for s in (-1.0, 1.0) for z in (0.0, th)]
        out.append((np.array(pts), STEEL))
    r, th = X_PIVOT
    out.append((np.array([[X_PIVOT_AT + y, z_, zz] for y, z_ in _ngon(r, 8) for zz in (0.0, th)]), STEEL_DARK))
    w0, w1, t0, t1 = X_BLADE
    out.append((np.array([[x, s * w, z] for x, w, t in ((X_PIVOT_AT, w0, t0), (X_LEN, w1, t1)) for s in (-1.0, 1.0)
                          for z in (0.0, t)]), STEEL))
    return out


_ITEM: dict = {}


def _item(kind: str) -> dict:
    """Everything fixed about a kind of item ("syringe" or "scissors"), in its own (centre of mass) frame."""
    if kind not in _ITEM:
        pieces, vol, com = _centred({"syringe": _syringe_raw, "scissors": _scissors_raw}[kind]())
        verts = np.vstack([p for p, _ in pieces])
        _ITEM[kind] = {"pieces": pieces, "volume": vol, "com": com, "verts": verts,
                       "planes": [hull_planes(p) for p, _ in pieces]}
    return _ITEM[kind]


def syringe_pieces() -> list:
    return _item("syringe")["pieces"]


def scissors_pieces() -> list:
    return _item("scissors")["pieces"]


def item_local(kind: str, raw) -> np.ndarray:
    """A point given in an item's drawing frame (the syringe from its flange's front face, the scissors from the back
    of their rings and their underside), in its own centre-of-mass frame."""
    return np.asarray(raw, float) - _item(kind)["com"]


def ring_back(sign: float = -1.0) -> np.ndarray:
    """The middle of the bar at the back of one finger ring, in the scissors' own frame."""
    return item_local("scissors", (X_WIRE / 2, sign * X_RING_Y, X_WIRE / 2))


def shank_hold(sign: float = 1.0) -> np.ndarray:
    """Where the scissors balance on one shank (``sign`` +1: the one on their +y side): the point of its centre line
    nearest their centre of mass, at half its thickness, in the scissors' own frame."""
    a, b = np.array([2 * X_RING[0] - X_WIRE, sign * X_RING_Y]), np.array([X_PIVOT_AT, 0.0])
    c = _item("scissors")["com"][:2]
    f = a + ((c - a) @ (b - a)) / ((b - a) @ (b - a)) * (b - a)
    return item_local("scissors", (f[0], f[1], X_SHANK[1] / 2))


def shank_yaw(sign: float = 1.0) -> float:
    """How far the scissors are turned (rad about z, from pointing along x) when that shank lies along x."""
    return sign * math.atan2(X_RING_Y, X_PIVOT_AT - (2 * X_RING[0] - X_WIRE))


def syringe_rest() -> tuple:
    """How the syringe lies on a level surface: (pitch nose down in rad, height of its centre of mass)."""
    com = _item("syringe")["com"]
    fx, fz = -S_FLANGE[2] / 2, -S_FLANGE[1]                    # the flange's lower edge
    bx, bz = S_BARREL_L, -S_BARREL_R                           # the barrel's front rim
    pitch = math.atan2(bz - fz, bx - fx)
    c, s_ = math.cos(pitch), math.sin(pitch)
    lift = lambda x, z: -(x - com[0]) * s_ + (z - com[2]) * c  # noqa: E731  height of a raw point above the centre of mass
    return pitch, -lift(fx, fz)


def turn_y(angle: float) -> tuple:
    """Quaternion (xyzw) of a turn of ``angle`` rad about y (positive: the x axis dips)."""
    return (0.0, math.sin(angle / 2), 0.0, math.cos(angle / 2))


class _Part(Box):
    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")


@dataclass(frozen=True)
class Forceps(_Part):
    """The forceps' +y blade. ``blade`` names the other :class:`Blade`."""
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    mass: float | None = F_MASS / 2
    blade: str = "blade"


@dataclass(frozen=True)
class Blade(_Part):
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    mass: float | None = F_MASS / 2


@dataclass(frozen=True)
class Cup(_Part):
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    mass: float | None = C_MASS


@dataclass(frozen=True)
class Syringe(_Part):
    """``mass`` is the real syringe's; the simulated body is ``S_BODY`` with the difference carried by a force."""
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    mass: float | None = S_MASS


@dataclass(frozen=True)
class Scissors(_Part):
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    mass: float | None = X_MASS


@dataclass(frozen=True)
class KitState(ObjectState):
    """A kit object as the simulator has it; ``pos`` is its own origin (see each class)."""

    joints: dict = field(default_factory=dict)


# -- geometry (pure) -------------------------------------------------------------------------------------------------

def _rotation(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _quat(rot) -> tuple:
    m = np.asarray(rot, float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    else:
        i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(max(1e-12, 1.0 + m[i, i] - m[j, j] - m[k, k])) * 2
        q = [0.0, 0.0, 0.0, (m[k, j] - m[j, k]) / s]
        q[i], q[j], q[k] = 0.25 * s, (m[j, i] + m[i, j]) / s, (m[k, i] + m[i, k]) / s
    return tuple(float(v) for v in q)


def place(pos, quat, local) -> np.ndarray:
    """Points given in a part's own frame, in the world."""
    return np.asarray(pos, float) + np.asarray(local, float) @ _rotation(quat).T


def to_local(points, pos, quat) -> np.ndarray:
    return (np.asarray(points, float) - np.asarray(pos, float)) @ _rotation(quat)


def _blade_samples(side: float, x0: float, x1: float, step: float = 0.005) -> np.ndarray:
    """Points over a blade's faces and edges between ``x0`` and ``x1`` m from the tail, in the blade's frame."""
    xs = np.arange(x0, x1 + 1e-9, step)
    return np.array([[x, side * (bow(x) + d), s * half_width(x)] for x in xs for d in (0.0, F_TH) for s in (-1.0, 0.0, 1.0)])


_WORK = {s: _blade_samples(s, F_LEN - WORK, F_LEN, 0.001) for s in (1.0, -1.0)}
_HANDLE = {s: _blade_samples(s, 0.0, HANDLE, 0.0025) for s in (1.0, -1.0)}


def forceps_points(pos_a, quat_a, pos_b, quat_b) -> dict:
    """World points on the forceps from its two blades' poses: ``work`` (the working end), ``handle``, ``tip`` (the
    middle of the two tips) and ``tail``, plus ``gap`` (m between the tips' inner faces) and ``up`` (how far the tips
    are above the tail)."""
    work = np.vstack([place(pos_a, quat_a, _WORK[1.0]), place(pos_b, quat_b, _WORK[-1.0])])
    handle = np.vstack([place(pos_a, quat_a, _HANDLE[1.0]), place(pos_b, quat_b, _HANDLE[-1.0])])
    ta, tb = place(pos_a, quat_a, (F_LEN, 0.0, 0.0)), place(pos_b, quat_b, (F_LEN, 0.0, 0.0))
    tail = (np.asarray(pos_a, float) + np.asarray(pos_b, float)) / 2
    tip = (ta + tb) / 2
    return {"work": work, "handle": handle, "tip": tip, "tail": tail, "gap": float(np.linalg.norm(ta - tb)),
            "up": float(tip[2] - tail[2])}


_CUP_RING = 2 * math.pi * np.arange(16) / 16


def cup_points(pos, quat) -> np.ndarray:
    """World points on a cup: round the edge of its base and round its rim."""
    c, s = np.cos(_CUP_RING), np.sin(_CUP_RING)
    local = np.vstack([np.column_stack([C_BASE_R * c, C_BASE_R * s, np.zeros(16)]),
                       np.column_stack([C_TOP_R * c, C_TOP_R * s, np.full(16, C_H)])])
    return place(pos, quat, local)


def in_cup(points, pos, quat, margin: float) -> np.ndarray:
    """For each point: is it inside the cup, or within ``margin`` of its outside (the cup taken as a solid)?"""
    loc = to_local(np.atleast_2d(points), pos, quat)
    rho = np.hypot(loc[:, 0], loc[:, 1])
    r = C_BASE_R + (C_TOP_R - C_BASE_R) * np.clip(loc[:, 2] / C_H, 0.0, 1.0)
    return (loc[:, 2] >= -margin) & (loc[:, 2] <= C_H + margin) & (rho <= r + margin)


def cup_tilt(quat) -> float:
    """Angle between the cup's axis and upright (rad)."""
    return math.acos(max(-1.0, min(1.0, float(_rotation(quat)[2, 2]))))


def item_points(kind: str, pos, quat) -> np.ndarray:
    """World points on an item: the cup's base edge and rim, or every corner of the syringe's or scissors' pieces."""
    if kind == "cup":
        return cup_points(pos, quat)
    return place(pos, quat, _item(kind)["verts"])


def near_item(kind: str, points, pos, quat, margin: float) -> np.ndarray:
    """For each point: is it inside the item or within ``margin`` of it? The cup counts as a solid; the syringe and
    scissors are tested piece by piece (each piece grown by ``margin`` along its faces)."""
    if kind == "cup":
        return in_cup(points, pos, quat, margin)
    loc = to_local(np.atleast_2d(points), pos, quat)
    hit = np.zeros(len(loc), bool)
    lo, hi = _item(kind)["verts"].min(0) - margin, _item(kind)["verts"].max(0) + margin
    box = np.all((loc >= lo) & (loc <= hi), axis=1)
    if not box.any():
        return hit
    sub = loc[box]
    inside = np.zeros(len(sub), bool)
    for pl in _item(kind)["planes"]:
        inside |= np.all(sub @ pl[:, :3].T - pl[:, 3] <= margin, axis=1)
    hit[box] = inside
    return hit


def item_axis_tilt(quat) -> float:
    """Angle between an item's own z axis and upright (rad)."""
    return cup_tilt(quat)


# -- the arms' shapes (pure): points on each link's collision mesh, in the link's own frame ------------------------------

def _stl_points(path: Path) -> np.ndarray:
    b = path.read_bytes()
    n = struct.unpack("<I", b[80:84])[0]
    tri = np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return np.unique(tri["v"].reshape(-1, 3).astype(float).round(4), axis=0)


def _thin(points: np.ndarray, cell: float = 0.012) -> np.ndarray:
    """One point per ``cell``-sized box, keeping the extremes: enough to tell where a link is to a few millimetres."""
    keep = {}
    for p in points:
        keep.setdefault(tuple(np.floor(p / cell).astype(int)), p)
    extremes = [points[points[:, k].argmin()] for k in range(3)] + [points[points[:, k].argmax()] for k in range(3)]
    return np.vstack([np.array(list(keep.values())), np.array(extremes)])


def _link_points() -> dict:
    from yam_arena import YAM_URDF
    from yam_arena.urdf import rpy_matrix

    root = Path(str(YAM_URDF)).parent
    text = Path(str(YAM_URDF)).read_text()
    out = {}
    for name in ARM_LINKS:
        m = re.search(r'<link name="%s">(.*?)</link>' % name, text, re.S)
        pts = []
        for c in re.findall(r"<collision.*?</collision>", m.group(1) if m else "", re.S):
            o = re.search(r'origin xyz="([^"]+)" rpy="([^"]+)"', c)
            f = re.search(r'filename="([^"]+)"', c)
            if not (o and f):
                continue
            xyz = np.array([float(v) for v in o.group(1).split()])
            rot = rpy_matrix(np.array([float(v) for v in o.group(2).split()]))
            pts.append(_stl_points(root / f.group(1)) @ rot.T + xyz)
        cell = 0.005 if name in ("tip_left", "tip_right") else 0.012     # the fingers finely: they meet small things
        out[name] = _thin(np.vstack(pts), cell) if pts else np.zeros((0, 3))
    return out


_LINKS: dict = {}


def link_points() -> dict:
    """Points on each arm link's collision mesh, in the link's frame (read once from the robot's URDF)."""
    if not _LINKS:
        _LINKS.update(_link_points())
    return _LINKS


def segment_distance(p, a, b) -> float:
    p, a, b = (np.asarray(v, float) for v in (p, a, b))
    ab = b - a
    t = float(np.clip((p - a) @ ab / max(float(ab @ ab), 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def _material(sim_utils, sf, df, combine="average"):
    try:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df, friction_combine_mode=combine)
    except TypeError:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df)


# -- authoring helpers -----------------------------------------------------------------------------------------------

class _Author:
    def __init__(self, prim_path, translation, orientation):
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, UsdGeom

        self.sim_utils, self.Gf, self.UsdGeom = sim_utils, Gf, UsdGeom
        self.stage = get_current_stage()
        self.path = prim_path
        double = UsdGeom.XformOp.PrecisionDouble
        root = UsdGeom.Xform.Define(self.stage, prim_path)
        root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
        x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
        root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))
        self.root = root.GetPrim()
        self.looks, self.mats = {}, {}

    def look(self, color, rough=0.6, metal=0.0) -> str:
        from yam_arena.station import linear_rgb

        key = (tuple(color), rough, metal)
        if key not in self.looks:
            path = f"{self.path}/Looks/v{len(self.looks)}"
            vis = self.sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough, metallic=metal)
            vis.func(path, vis)
            self.looks[key] = path
        return self.looks[key]

    def mat(self, key, sf, df, combine="average") -> str:
        if key not in self.mats:
            path = f"{self.path}/Looks/phys_{key}"
            pm = _material(self.sim_utils, sf, df, combine)
            pm.func(path, pm)
            self.mats[key] = path
        return self.mats[key]

    def body(self, path, mass, lin_damp=0.05, ang_damp=0.1, iterations=None):
        from pxr import PhysxSchema, UsdPhysics

        prim = self.UsdGeom.Xform.Define(self.stage, path).GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.3)
        rb.CreateLinearDampingAttr(lin_damp)
        rb.CreateAngularDampingAttr(ang_damp)
        if iterations:
            rb.CreateSolverPositionIterationCountAttr(int(iterations))
            rb.CreateSolverVelocityIterationCountAttr(4)
        return prim

    def hull(self, path, points, look, mat=None, offset=0.0005, patch=0.0):
        """A convex piece: drawn as its hull's faces, and a convex-hull collider if ``mat`` is given."""
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from pxr import PhysxSchema, UsdPhysics, Vt

        pts = np.asarray(points, float)
        faces = hull_faces(pts)
        mesh = self.UsdGeom.Mesh.Define(self.stage, path)
        mesh.CreatePointsAttr(Vt.Vec3fArray([self.Gf.Vec3f(*(float(v) for v in p)) for p in pts]))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([len(f) for f in faces]))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([int(i) for f in faces for i in f]))
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        prim = mesh.GetPrim()
        if look:
            bind_visual_material(str(prim.GetPath()), look, stage=self.stage)
        if mat:
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
            cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            cp.CreateContactOffsetAttr(offset)
            cp.CreateRestOffsetAttr(0.0)
            if patch:                                # a rubber pad meets a blade over a patch, which resists twisting
                cp.CreateTorsionalPatchRadiusAttr(float(patch))
                cp.CreateMinTorsionalPatchRadiusAttr(float(patch))
            bind_physics_material(str(prim.GetPath()), mat, stage=self.stage)
        return prim

    def pieces(self, stem, pieces, mat, rough=0.35, metal=0.9, patch=0.0):
        for n, (pts, color) in enumerate(pieces):
            self.hull(f"{stem}_{n}", pts, self.look(color, rough=rough, metal=metal), mat, patch=patch)


# -- spawning ----------------------------------------------------------------------------------------------------------

_SPECS: dict = {}
_PATHS: dict = {}                      # where each kit object's prim was spawned


def body_mass(spec) -> float:
    """The mass the simulator gives a part's body (the syringe's is heavier than the syringe: see ``S_BODY``)."""
    return S_BODY if isinstance(spec, Syringe) else float(spec.mass)


def _start(cfg, prim_path, translation, orientation, iterations=None):
    spec = _SPECS[cfg.kit_name]
    _PATHS[spec.name] = prim_path
    a = _Author(prim_path, translation, orientation)
    a.body(prim_path, body_mass(spec), iterations=iterations)
    return spec, a, a.mat("part", *FRICTION)


def _spawn_forceps(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec, a, m = _start(cfg, prim_path, translation, orientation, iterations=32)
    a.pieces(f"{prim_path}/p", blade_pieces(1.0), m, patch=PAD_PATCH)
    return a.root


def _spawn_blade(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec, a, m = _start(cfg, prim_path, translation, orientation, iterations=32)
    a.pieces(f"{prim_path}/p", blade_pieces(-1.0), m, patch=PAD_PATCH)
    return a.root


def _spawn_cup(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec, a, m = _start(cfg, prim_path, translation, orientation, iterations=32)
    a.pieces(f"{prim_path}/p", cup_pieces(), m)
    return a.root


def _spawn_syringe(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec, a, m = _start(cfg, prim_path, translation, orientation, iterations=32)
    a.pieces(f"{prim_path}/p", syringe_pieces(), m, rough=0.5, metal=0.0)
    return a.root


def _spawn_scissors(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec, a, m = _start(cfg, prim_path, translation, orientation, iterations=32)
    a.pieces(f"{prim_path}/p", scissors_pieces(), m)
    return a.root


# -- the tail's sprung hinge ---------------------------------------------------------------------------------------------

def _pairs() -> list:
    """(forceps part, its blade part) names of every pair that has been spawned."""
    return [(n, s.blade) for n, s in _SPECS.items() if isinstance(s, Forceps) and n in _PATHS and s.blade in _PATHS]


def _joint_path(name: str) -> str:
    return f"{_PATHS[name]}/tail_joint"


def _author(a: str, b: str) -> None:
    """Author the hinge between blades ``a`` and ``b`` (about a's z, at both blades' own origins): disabled first,
    its frames set, enabled last. Its angle is b's turn about z relative to a: 0 with the blades together, -F_ANGLE
    at rest. A spring (a position drive) pulls it to rest."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdPhysics

    stage = get_current_stage()
    joint = UsdPhysics.RevoluteJoint.Define(stage, _joint_path(a))
    joint.CreateJointEnabledAttr().Set(False)
    joint.CreateBody0Rel().SetTargets([_PATHS[a]])
    joint.CreateBody1Rel().SetTargets([_PATHS[b]])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    joint.CreateExcludeFromArticulationAttr().Set(True)
    joint.CreateCollisionEnabledAttr().Set(False)
    joint.CreateAxisAttr().Set("Z")
    joint.CreateLowerLimitAttr().Set(float(-math.degrees(F_ANGLE)))
    joint.CreateUpperLimitAttr().Set(float(-math.degrees(F_MIN)))
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr().Set("force")
    drive.CreateStiffnessAttr().Set(float(F_STIFF * math.pi / 180.0))      # N m per degree
    drive.CreateDampingAttr().Set(float(0.02 * F_STIFF * math.pi / 180.0))
    drive.CreateTargetPositionAttr().Set(float(-math.degrees(F_ANGLE)))
    joint.CreateJointEnabledAttr().Set(True)       # last write, after the frames


def _rigid_object(spec, spawn):
    from isaaclab.sim.spawners.spawner_cfg import RigidObjectSpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class KitSpawnerCfg(RigidObjectSpawnerCfg):
        func: object = clone(spawn)
        kit_name: str = ""

    _SPECS[spec.name] = spec
    _PATHS.pop(spec.name, None)
    quat = tuple(float(v) for v in getattr(spec, "quat", (0.0, 0.0, 0.0, 1.0)))
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=KitSpawnerCfg(kit_name=spec.name),
                  initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=quat))


_SPAWNERS = {Forceps: _spawn_forceps, Blade: _spawn_blade, Cup: _spawn_cup, Syringe: _spawn_syringe,
             Scissors: _spawn_scissors}
_PIECES = {Forceps: lambda: blade_pieces(1.0), Blade: lambda: blade_pieces(-1.0), Cup: cup_pieces,
           Syringe: syringe_pieces, Scissors: scissors_pieces}
_BOUNDS: dict = {}


# -- run time ----------------------------------------------------------------------------------------------------------

def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    if hasattr(x, "numpy") and not hasattr(x, "detach"):
        return np.asarray(x.numpy(), dtype=np.float64)
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _origin(environment) -> np.ndarray:
    return _np(environment.unwrapped.scene.env_origins[0])


def write_pose(environment, name: str, pos, quat_xyzw=(0.0, 0.0, 0.0, 1.0)) -> None:
    """Put a loose object at ``pos`` (relative to the env origin), ``quat_xyzw``, at rest."""
    import torch

    asset = environment.unwrapped.scene[name]
    dev = _t(asset.data.root_link_pose_w).device
    pose = torch.tensor([[*(np.asarray(pos, float) + _origin(environment)), *quat_xyzw]], dtype=torch.float32, device=dev)
    vel = torch.zeros((1, 6), dtype=torch.float32, device=dev)
    for stem, kw in (("write_root_pose_to_sim", {"root_pose": pose}), ("write_root_velocity_to_sim", {"root_velocity": vel})):
        fn = getattr(asset, f"{stem}_index", None)
        if fn is not None:
            fn(env_ids=[0], **kw)
        else:
            getattr(asset, stem)(*kw.values())


def _disarm() -> None:
    """Switch every hinge off (before a reset puts the parts back)."""
    from isaaclab.sim.utils.stage import get_current_stage

    stage = get_current_stage()
    for a, _ in _pairs():
        prim = stage.GetPrimAtPath(_joint_path(a))
        if prim and prim.IsValid():
            prim.GetAttribute("physics:jointEnabled").Set(False)


def _arm(environment) -> None:
    """Join each pair of blades as the reset left them."""
    for a, b in _pairs():
        _author(a, b)


def _carry(environment) -> None:
    """Hold on every part whose body is heavier than the part the upward force that carries the difference."""
    import torch

    for n, spec in _SPECS.items():
        extra = (body_mass(spec) - float(spec.mass)) * 9.81
        if n not in _PATHS or extra <= 0.0:
            continue
        try:
            asset = environment.unwrapped.scene[n]
            ft = torch.tensor([[[0.0, 0.0, float(extra)]]], dtype=torch.float32, device=asset.device)
            asset.permanent_wrench_composer.set_forces_and_torques_index(forces=ft, torques=torch.zeros_like(ft), is_global=True)
        except Exception as err:                     # noqa: BLE001
            print(f"[xferkit] weight not carried for {n}: {err!r}", file=sys.stderr, flush=True)


def _local_bounds(spec) -> tuple:
    kind = type(spec)
    if kind not in _BOUNDS:
        pts = np.vstack([p for p, _ in _PIECES[kind]()])
        _BOUNDS[kind] = (pts.min(0), pts.max(0))
    return _BOUNDS[kind]


def _kit_state(environment, spec) -> KitState:
    asset = environment.unwrapped.scene[spec.name]
    pose = _np(asset.data.root_link_pose_w)[0]
    pos, quat = pose[:3] - _origin(environment), pose[3:7]
    vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    lo, hi = _local_bounds(spec)
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    pts = corners @ _rotation(quat).T + pos
    return KitState(pos=pos, quat_xyzw=quat, lin_vel=vel[:3], ang_vel=vel[3:6], low=pts.min(0), high=pts.max(0))


def finger_points(environment) -> list:
    """The grasp point of each gripper, left then right (world, relative to the env origin), and its rotation."""
    from yam_arena.constants import TCP_OFFSET

    base = environment.unwrapped
    origin = _origin(environment)
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = _np(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
        rot = _rotation(pose[3:7])
        out.append((pose[:3] + rot @ np.array(TCP_OFFSET, float) - origin, rot))
    return out


def arm_points(environment) -> np.ndarray:
    """Points on both arms' collision shapes from the upper arm to the fingertips (world, relative to the env
    origin): every link's mesh, placed where the simulator has the link."""
    base = environment.unwrapped
    origin = _origin(environment)
    local = link_points()
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        poses = _np(robot.data.body_link_pose_w)[0]
        names = list(robot.body_names)
        for link in ARM_LINKS:
            if link in names and len(local[link]):
                pose = poses[names.index(link)]
                out.append(local[link] @ _rotation(pose[3:7]).T + pose[:3] - origin)
    return np.vstack(out)


# -- hooking yam_arena -------------------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []
_ENV: dict = {"env": None}


def on_step(fn):
    _STEP.append(fn)
    return fn


def on_reset(fn):
    _RESET.append(fn)
    return fn


def env():
    return _ENV["env"]


def _patch_scene(scene) -> None:
    if getattr(scene, "_xferkit_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        for kind, spawn in _SPAWNERS.items():
            if isinstance(spec, kind):
                return _rigid_object(spec, spawn)
        return stock_object(spec)

    def object_states(environment, names):
        mine = {n for n in names if isinstance(_SPECS.get(n), tuple(_SPAWNERS))}
        plain = [n for n in names if n not in mine]
        out = stock_states(environment, plain) if plain else {}
        for n in mine:
            out[n] = _kit_state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._xferkit_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_xferkit_patched", False):
        return
    stock_step, stock_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        return stock_step(self, *a, **k)

    def reset(self, *a, **k):
        # The hinge has to be whole before the reset's settling steps and first picture: switch it off, let the
        # environment put the parts back, then author it afresh from where the parts are.
        base, saved, had = None, None, False
        try:
            self._ensure_sim(None)
            base = self._env.unwrapped
            env_reset = base.reset
            saved, had = vars(base).get("reset"), "reset" in vars(base)

            def reset_then_arm(*aa, **kk):
                _disarm()
                result = env_reset(*aa, **kk)
                _arm(self._env)
                _carry(self._env)
                return result

            base.reset = reset_then_arm
        except Exception as err:                     # noqa: BLE001
            print(f"[xferkit] hinge not re-armed around the reset: {err!r}", file=sys.stderr, flush=True)
            base = None
        try:
            obs = stock_reset(self, *a, **k)
        finally:
            if base is not None:
                if had:
                    base.reset = saved
                else:
                    del base.reset
        _ENV["env"] = self._env
        _carry(self._env)
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._xferkit_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    targets = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}

    def find_spec(self, fullname, path, target=None):
        if fullname not in self.targets:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        loader, patch = spec.loader, self.targets[fullname]
        stock_exec = loader.exec_module

        def exec_module(module):
            stock_exec(module)
            patch(module)

        loader.exec_module = exec_module
        return spec


def install() -> None:
    """Hook yam_arena (now for modules already imported, otherwise when they are). Idempotent."""
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
