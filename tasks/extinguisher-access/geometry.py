"""Shapes of the toolbox and the fire extinguisher. NumPy only.

Both are single rigid bodies built from convex parts. Each body's local frame is the centre of its parts'
bounding box (what the simulator reports as the object's position), and every part is placed relative to it.

The toolbox: a moulded plastic box with a carry handle along the top of its lid, standing on two posts with
room under the bar for the fingers. Local +x runs along its length (and the handle), +z up.

The extinguisher: a stored-pressure 2.5 lb ABC dry-chemical unit: a round steel body with a domed shoulder,
a valve with a carry handle and a squeeze lever above it, and a short nozzle. Local +x points at the wall
(the levers stand out toward the room, -x), +z up.
"""

from __future__ import annotations

import numpy as np

# -- the toolbox -----------------------------------------------------------------------------------------
BOX_BODY = (0.330, 0.170, 0.110)  # a 13-inch toolbox: length (x), width (y), height (z) of the box and lid
POST = (0.020, 0.022, 0.019)  # the handle's two posts on the lid
POST_X = 0.075  # their centres, either side of the middle
BAR = (0.180, 0.024, 0.016)  # the handle's bar, on the posts: length, width, height
TOOLBOX_HEIGHT = BOX_BODY[2] + POST[2] + BAR[2]  # 0.145 m to the top of the handle
TOOLBOX_SIZE = (BOX_BODY[0], BOX_BODY[1], TOOLBOX_HEIGHT)  # its bounding box

# -- the extinguisher -------------------------------------------------------------------------------------
EXT_RADIUS = 0.0415  # 83 mm across
EXT_BODY_H = 0.270  # the straight part of the body
EXT_SHOULDER_H = 0.030  # the domed shoulder, as a cone to the neck
EXT_NECK_R, EXT_NECK_H = 0.012, 0.020
EXT_VALVE = (0.050, 0.030, 0.030)  # the valve body on the neck
EXT_HANDLE = (0.085, 0.020, 0.008)  # the fixed carry handle, out toward the room
EXT_LEVER = (0.080, 0.020, 0.008)  # the squeeze lever above it
EXT_LEVER_GAP = 0.010  # between the carry handle's top and the lever's underside
EXT_NOZZLE = (0.012, 0.030, 0.012)  # a short nozzle out of the valve's side
SIDES = 24  # polygon sides for the round parts


def box_corners(size, centre=(0.0, 0.0, 0.0)) -> np.ndarray:
    """The eight corners of a box, (8, 3)."""
    half = np.asarray(size, dtype=float) / 2
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)
    return np.asarray(centre, dtype=float) + signs * half


def ring(radius: float, z: float, centre=(0.0, 0.0), sides: int = SIDES) -> np.ndarray:
    a = np.linspace(0.0, 2 * np.pi, sides, endpoint=False)
    return np.c_[centre[0] + radius * np.cos(a), centre[1] + radius * np.sin(a), np.full(sides, z)]


def frustum(r0: float, r1: float, z0: float, z1: float, centre=(0.0, 0.0)) -> np.ndarray:
    """A cylinder (r0 == r1) or cone frustum's hull points, axis vertical."""
    return np.concatenate([ring(r0, z0, centre), ring(r1, z1, centre)])


# -- toolbox parts ----------------------------------------------------------------------------------------
def toolbox_parts() -> list[tuple[str, np.ndarray, np.ndarray]]:
    """(name, centre, size) boxes, in the toolbox frame (its bounding box's centre)."""
    z0 = -TOOLBOX_HEIGHT / 2  # its underside
    lid = z0 + BOX_BODY[2]
    return [
        ("body", np.array([0.0, 0.0, z0 + BOX_BODY[2] / 2]), np.array(BOX_BODY)),
        ("post_a", np.array([-POST_X, 0.0, lid + POST[2] / 2]), np.array(POST)),
        ("post_b", np.array([POST_X, 0.0, lid + POST[2] / 2]), np.array(POST)),
        ("bar", np.array([0.0, 0.0, lid + POST[2] + BAR[2] / 2]), np.array(BAR)),
    ]


def toolbox_part_points() -> list[tuple[str, np.ndarray]]:
    return [(name, box_corners(size, centre)) for name, centre, size in toolbox_parts()]


def toolbox_bounds_local() -> tuple[np.ndarray, np.ndarray]:
    pts = np.concatenate([p for _, p in toolbox_part_points()])
    return pts.min(0), pts.max(0)


BAR_TOP_Z = TOOLBOX_HEIGHT / 2  # local z of the handle's top
BAR_BOTTOM_Z = BAR_TOP_Z - BAR[2]
LID_Z = -TOOLBOX_HEIGHT / 2 + BOX_BODY[2]


# -- extinguisher parts -----------------------------------------------------------------------------------
def _extinguisher_raw() -> list[tuple[str, np.ndarray]]:
    """Hull points with z = 0 at its base and the body's axis at x = y = 0."""
    z1 = EXT_BODY_H
    z2 = z1 + EXT_SHOULDER_H
    z3 = z2 + EXT_NECK_H
    z4 = z3 + EXT_VALVE[2]
    handle_x = -EXT_HANDLE[0] / 2 + 0.010  # from 10 mm behind the axis out toward the room
    lever_z = z4 - EXT_HANDLE[2] / 2 + EXT_HANDLE[2] + EXT_LEVER_GAP + EXT_LEVER[2] / 2
    return [
        ("body", frustum(EXT_RADIUS, EXT_RADIUS, 0.0, z1)),
        ("shoulder", frustum(EXT_RADIUS, EXT_NECK_R + 0.006, z1, z2)),
        ("neck", frustum(EXT_NECK_R, EXT_NECK_R, z2, z3)),
        ("valve", box_corners(EXT_VALVE, (0.0, 0.0, z3 + EXT_VALVE[2] / 2))),
        ("handle", box_corners(EXT_HANDLE, (handle_x, 0.0, z4 - EXT_HANDLE[2] / 2))),
        ("lever", box_corners(EXT_LEVER, (handle_x + 0.003, 0.0, lever_z))),
        ("nozzle", box_corners(EXT_NOZZLE, (0.0, EXT_VALVE[1] / 2 + EXT_NOZZLE[1] / 2, z3 + EXT_VALVE[2] / 2))),
    ]


def _ext_offset() -> np.ndarray:
    pts = np.concatenate([p for _, p in _extinguisher_raw()])
    return (pts.min(0) + pts.max(0)) / 2


def extinguisher_part_points() -> list[tuple[str, np.ndarray]]:
    """Hull points in the extinguisher frame (its bounding box's centre)."""
    off = _ext_offset()
    return [(name, pts - off) for name, pts in _extinguisher_raw()]


def extinguisher_bounds_local() -> tuple[np.ndarray, np.ndarray]:
    pts = np.concatenate([p for _, p in extinguisher_part_points()])
    return pts.min(0), pts.max(0)


EXT_SIZE = tuple(float(v) for v in (lambda b: b[1] - b[0])(extinguisher_bounds_local()))  # 117 x 87 x 372 mm
EXT_HEIGHT = EXT_SIZE[2]  # 0.372 m to the top of the squeeze lever
EXT_AXIS_LOCAL = -_ext_offset()  # the body's axis at its base, in the extinguisher frame
EXT_PART_COLORS = {"body": "red", "shoulder": "red", "neck": "black", "valve": "black", "handle": "black",
                   "lever": "black", "nozzle": "black"}  # fmt: skip
