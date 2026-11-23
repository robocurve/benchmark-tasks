"""Shapes of the hole cover. NumPy only.

The cover is one rigid body made of four boxes: a square plywood plate, a handle block screwed to its top,
and two cleats screwed under it that drop into the floor opening and locate it. Its local frame is the
centre of the whole assembly's bounding box (what the simulator reports as the object's position), so
every part is placed relative to that.
"""

from __future__ import annotations

import numpy as np

# -- the cover's parts, in its local frame: (name, centre, size) -------------------------------------------
PLATE_SIZE = (0.280, 0.280, 0.018)  # 18 mm plywood, 280 mm square
HANDLE_SIZE = (0.150, 0.040, 0.040)  # a 150 mm offcut of 40 x 40 softwood, along x on top
CLEAT_SIZE = (0.190, 0.020, 0.018)  # two 18 mm plywood strips under the plate, along x
CLEAT_Y = 0.084  # cleat centre lines: outer faces 188 mm apart, 6 mm inside each side of the opening
CLEAT_CHAMFER = 0.008  # the cleats' four bottom edges planed off at 45 degrees: a lead-in that finds the opening
COVER_HEIGHT = CLEAT_SIZE[2] + PLATE_SIZE[2] + HANDLE_SIZE[2]  # 76 mm from the cleats' feet to the handle top
COVER_HALF_H = COVER_HEIGHT / 2

CLEAT_BOTTOM_Z = -COVER_HALF_H  # local z of the cleats' feet
PLATE_BOTTOM_Z = CLEAT_BOTTOM_Z + CLEAT_SIZE[2]  # local z of the plate's underside
PLATE_TOP_Z = PLATE_BOTTOM_Z + PLATE_SIZE[2]
HANDLE_TOP_Z = COVER_HALF_H

# (name, centre, size of the bounding box): the plate and handle are boxes, the cleats chamfered boxes
COVER_PARTS = (
    ("plate", (0.0, 0.0, PLATE_BOTTOM_Z + PLATE_SIZE[2] / 2), PLATE_SIZE),
    ("handle", (0.0, 0.0, PLATE_TOP_Z + HANDLE_SIZE[2] / 2), HANDLE_SIZE),
    ("cleat_left", (0.0, +CLEAT_Y, CLEAT_BOTTOM_Z + CLEAT_SIZE[2] / 2), CLEAT_SIZE),
    ("cleat_right", (0.0, -CLEAT_Y, CLEAT_BOTTOM_Z + CLEAT_SIZE[2] / 2), CLEAT_SIZE),
)


def box_corners(size, centre=(0.0, 0.0, 0.0)) -> np.ndarray:
    """The eight corners of a box, (8, 3)."""
    half = np.asarray(size, dtype=float) / 2
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)
    return np.asarray(centre, dtype=float) + signs * half


def chamfered_bottom_points(size, chamfer: float, centre=(0.0, 0.0, 0.0)) -> np.ndarray:
    """Hull points of a box whose four bottom edges are chamfered at 45 degrees by ``chamfer``, (12, 3)."""
    sx, sy, sz = (float(v) / 2 for v in size)
    if not 0 < chamfer < min(sx, sy, 2 * sz):
        raise ValueError("chamfer must be positive and smaller than the box")
    points = []
    for x in (-sx, sx):
        for y in (-sy, sy):
            points.append((x, y, sz))  # top face: square corners
            points.append((x, y, -sz + chamfer))  # where the side face meets the chamfer
    for x in (-(sx - chamfer), sx - chamfer):
        for y in (-(sy - chamfer), sy - chamfer):
            points.append((x, y, -sz))  # the foot, inset by the chamfer
    return np.asarray(centre, dtype=float) + np.array(points)


def cover_part_points() -> list[tuple[str, np.ndarray]]:
    """Each part's convex-hull points in the cover's local frame."""
    out = []
    for name, centre, size in COVER_PARTS:
        if name.startswith("cleat"):
            out.append((name, chamfered_bottom_points(size, CLEAT_CHAMFER, centre)))
        else:
            out.append((name, box_corners(size, centre)))
    return out


def cover_volume_m3() -> float:
    return float(sum(np.prod(size) for _, _, size in COVER_PARTS))


def plate_bottom_corners_local() -> np.ndarray:
    """The four corners of the plate's underside, (4, 3), local frame."""
    sx, sy = PLATE_SIZE[0] / 2, PLATE_SIZE[1] / 2
    return np.array([[x, y, PLATE_BOTTOM_Z] for x in (-sx, sx) for y in (-sy, sy)])


def cover_bounds_local() -> tuple[np.ndarray, np.ndarray]:
    """The assembly's local bounding box: its centre must be the origin."""
    pts = np.concatenate([box_corners(size, centre) for _, centre, size in COVER_PARTS])
    return pts.min(0), pts.max(0)
