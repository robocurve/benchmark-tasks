"""Collision and visual shapes of the tiles and spacers. NumPy only.

Both are convex: each is the convex hull of the points below, so PhysX collides it as exactly that hull.

- Tile: a square slab whose four top edges are eased by a 45-degree chamfer, like the cushion edge of a
  glazed wall tile. Where two tiles meet, the eased edges open the 3 mm joint to a wider mouth at the glaze.
- Spacer: a flat rectangular tab whose four long edges are chamfered, like the rounded edges of a moulded
  plastic spacer. Its narrowed edge is what starts into the joint.
"""

from __future__ import annotations

import numpy as np


def box_corners(size) -> np.ndarray:
    """The eight corners of a box centred on the origin, (8, 3)."""
    half = np.asarray(size, dtype=float) / 2
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)
    return signs * half


def tile_points(size, chamfer: float) -> np.ndarray:
    """Hull points of a tile ``size`` = (x, y, thickness) with its top edges chamfered by ``chamfer``."""
    sx, sy, sz = (float(v) / 2 for v in size)
    if not 0 < chamfer < min(sx, sy, 2 * sz):
        raise ValueError("chamfer must be positive and smaller than the tile")
    points = []
    for x in (-sx, sx):
        for y in (-sy, sy):
            points.append((x, y, -sz))  # bottom face: square corners
            points.append((x, y, sz - chamfer))  # where the side face meets the chamfer
    for x in (-(sx - chamfer), sx - chamfer):
        for y in (-(sy - chamfer), sy - chamfer):
            points.append((x, y, sz))  # the glazed face, inset by the chamfer
    return np.array(points)


def spacer_points(size, chamfer: float) -> np.ndarray:
    """Hull points of a spacer ``size`` = (length, thickness, height) with its four long edges chamfered."""
    sx, sy, sz = (float(v) / 2 for v in size)
    if not 0 < chamfer < min(sy, sz):
        raise ValueError("chamfer must be positive and smaller than half the spacer's thickness")
    points = []
    for x in (-sx, sx):
        for y in (-sy, sy):
            for z in (-sz, sz):
                points.append((x, y * (sy - chamfer) / sy, z))  # top and bottom edges, narrowed
                points.append((x, y, z * (sz - chamfer) / sz))  # side faces, ending at the chamfer
    return np.array(points)


def width_at(points: np.ndarray, axis: int, z: float, tol: float = 1e-9) -> float:
    """Extent of a convex point set along ``axis`` at height ``z``: the hull's cross-section width there.

    Exact for the hulls above, whose faces are planar; used by the tests.
    """
    from itertools import combinations

    best_lo, best_hi = np.inf, -np.inf
    for a, b in combinations(points, 2):
        za, zb = a[2], b[2]
        if abs(za - zb) < tol:
            if abs(za - z) < tol:
                best_lo, best_hi = min(best_lo, a[axis], b[axis]), max(best_hi, a[axis], b[axis])
            continue
        s = (z - za) / (zb - za)
        if -tol <= s <= 1 + tol:
            v = a[axis] + s * (b[axis] - a[axis])
            best_lo, best_hi = min(best_lo, v), max(best_hi, v)
    return float(best_hi - best_lo)
