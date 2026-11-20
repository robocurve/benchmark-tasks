"""eggmodel: the eggs, the bowl and the dish of the egg float test, as plain NumPy (no simulator).

An egg is a rigid shell of fixed volume. Its contents fill it except for the air cell at the blunt end (USDA Egg
Grading Manual: "The air cell is normally at the large end of the egg"; it grows as water evaporates through the shell).
So an older egg is lighter for the same volume, and its center of mass sits further toward the narrow end, away from
the air. Both follow here from one number per egg, the air cell's depth, through the egg's real shape:

- shape: Hugelschaffer's egg curve (the egg-shape model used by Narushin et al. 2021),
  r(x) = B/2 sqrt((L^2 - 4x^2) / (L^2 - 8wx + 4w^2)), its long axis along +x with the blunt end at +x, length L 56.7 mm
  and breadth B 42.1 mm (the measured means of 56.7 g eggs in Krivanek et al. 2016, Table I), the widest section set
  w = 2.5 mm toward the blunt end. That holds 52.54 cm3: a 57.0 g US Large egg (USDA: 24 oz per dozen) at a specific
  gravity of 1.085, inside the range of fresh eggs (UF/IFAS VM044: most fresh eggs fall in 1.075-1.085);
- contents: one uniform density, set so the fresh egg (a 3 mm air cell, USDA grade AA: 1/8 in or less) weighs 57.0 g;
- air cell: the contents are absent above a plane `cell` deep from the blunt tip.

Nothing else about an egg's behaviour in water is set: floatkit applies the water's buoyancy at the centroid of the
submerged shell, PhysX applies its weight at its center of mass, and whether it lies, stands or floats comes out of
the two. rest_tilt() finds the same answer offline (the resting pose on a flat floor under water), for the tests.
"""
from __future__ import annotations

import math

import numpy as np

RHO_W = 1000.0  # kg/m3: cold tap water
G = 9.81

EGG_L = 0.0567  # m: length
EGG_B = 0.0421  # m: breadth
EGG_W = 0.0025  # m: the widest section's offset toward the blunt end
FRESH_MASS = 0.0570  # kg: a US Large egg (USDA: 24 oz per dozen)
FRESH_CELL = 0.003  # m: a fresh egg's air cell depth (USDA grade AA: 1/8 inch or less)
_NX = 4001


def radius(x):
    """The egg's radius (m) at x along its axis (m from the middle of its length; blunt end at +L/2)."""
    x = np.asarray(x, float)
    L, B, w = EGG_L, EGG_B, EGG_W
    return B / 2 * np.sqrt(np.clip((L * L - 4 * x * x) / (L * L - 8 * w * x + 4 * w * w), 0.0, None))


_X = np.linspace(-EGG_L / 2, EGG_L / 2, _NX)
_DX = _X[1] - _X[0]
_A = math.pi * radius(_X) ** 2
VOLUME = float(_A.sum() * _DX)  # m3, the whole shell's outside
CENTROID_X = float((_X * _A).sum() * _DX / VOLUME)  # m: the volume's centroid (toward the blunt end)


def _cell(depth: float) -> tuple:
    m = _X > EGG_L / 2 - depth
    v = float(_A[m].sum() * _DX)
    c = float((_X[m] * _A[m]).sum() * _DX / v) if v > 0 else EGG_L / 2
    return v, c


RHO_CONTENTS = FRESH_MASS / (VOLUME - _cell(FRESH_CELL)[0])  # kg/m3 (shell and contents, averaged)


def mass(cell: float) -> float:
    """kg: an egg with an air cell `cell` deep (m)."""
    return RHO_CONTENTS * (VOLUME - _cell(cell)[0])


def com_x(cell: float) -> float:
    """m along the axis: its center of mass (the contents' centroid, the air cell left out)."""
    v, c = _cell(cell)
    return (VOLUME * CENTROID_X - v * c) / (VOLUME - v)


def specific_gravity(cell: float) -> float:
    return mass(cell) / (RHO_W * VOLUME)


def inertia(cell: float) -> tuple:
    """(Ixx, Iyy, Izz) kg m2 about the center of mass, axis x along the egg (the contents; the air cell has none)."""
    m = _X <= EGG_L / 2 - cell
    x, a, r2 = _X[m], _A[m], radius(_X[m]) ** 2
    rho = RHO_CONTENTS
    cx = com_x(cell)
    dm = rho * a * _DX
    ixx = float((dm * r2 / 2).sum())
    iyy = float((dm * (r2 / 4 + (x - cx) ** 2)).sum())
    return ixx, iyy, iyy


def surface_points(n_rings: int = 10, n_around: int = 24) -> np.ndarray:
    """Points on the shell (egg frame, about the middle of its length) for its convex collider: 10 rings of 24, spaced
    half evenly and half closer toward the round ends (so the waist, where the egg lies, and the tips, where it stands,
    are both smooth), and the two tips. 242 points, within PhysX's 255-vertex hull."""
    pts = [(-EGG_L / 2, 0.0, 0.0), (EGG_L / 2, 0.0, 0.0)]
    for k in range(1, n_rings + 1):
        u = k / (n_rings + 1)
        x = EGG_L / 2 * (0.5 * (2 * u - 1) - 0.5 * math.cos(math.pi * u))
        r = float(radius(x))
        for j in range(n_around):
            a = 2 * math.pi * (j + 0.5 * (k % 2)) / n_around
            pts.append((x, r * math.cos(a), r * math.sin(a)))
    return np.asarray(pts, float)


def volume_points(grid: float = 0.002) -> np.ndarray:
    """Points filling the shell on a grid (egg frame, about the middle of its length), each an equal share of VOLUME:
    the water acts on these (floatkit)."""
    ax = np.arange(-EGG_L / 2 + grid / 2, EGG_L / 2, grid)
    ar = np.arange(-EGG_B / 2 + grid / 2, EGG_B / 2, grid)
    P = np.stack(np.meshgrid(ax, ar, ar, indexing="ij"), -1).reshape(-1, 3)
    return P[np.hypot(P[:, 1], P[:, 2]) <= radius(P[:, 0])]


def rest_tilt(cell: float, rho_w: float = RHO_W, step_deg: float = 0.5) -> float:
    """Degrees the egg's axis rests above the horizontal (blunt end up) on a flat floor fully under water, from the
    least potential energy over tilts -90..90 (+90: standing on its narrow end). For an egg that sinks."""
    S = surface_points(40, 36)
    m, cm = mass(cell), com_x(cell)
    best = None
    for th in np.radians(np.arange(-90.0, 90.0 + 1e-9, step_deg)):
        zdir = np.array([math.sin(th), 0.0, math.cos(th)])  # world up in the egg frame, its axis tilted th up
        zmin = float((S @ zdir).min())
        u = m * (cm * zdir[0] - zmin) - rho_w * VOLUME * (CENTROID_X * zdir[0] - zmin)
        if best is None or u < best[0] - 1e-15:
            best = (u, math.degrees(th))
    return best[1]
