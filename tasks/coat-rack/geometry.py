"""Shapes of the coat rack and its screws. NumPy only.

The rack is one rigid body: an oak rail with three cast hooks on its front and two keyhole slots routed into
its back. A keyhole slot is concave, so the back of the rail is assembled from convex pieces that leave the
slot open: a square entry pocket at the bottom, a channel above it for the screw head, and two lip strips
on the surface that leave the narrow throat the shank runs in. The rack's local frame is the centre of the
whole assembly's bounding box (what the simulator reports as the object's position); every part is placed
relative to that. +x is toward the back (the wall), +z is up, y runs along the rail.
"""

from __future__ import annotations

import numpy as np

# -- the rail ------------------------------------------------------------------------------------------
RAIL_LENGTH, RAIL_HEIGHT, RAIL_THICKNESS = 0.400, 0.080, 0.025
BACK_LAYER = 0.008  # the depth of the keyhole pockets: the back 8 mm of the rail is built in pieces
LIP = 0.0025  # the surface lip over the channel that holds the screw head
CHAMFER = 0.002  # the entry's bottom edge and the lips' bottom edges are eased 2 mm at 45 degrees: a lead-in

# -- the hooks (cast, on the front) ---------------------------------------------------------------------
HOOK_Y = (-0.160, 0.000, 0.160)
HOOK_PLATE = (0.003, 0.030, 0.040)  # base plate screwed to the rail's face: thickness (x), width (y), height (z)
HOOK_ARM = (0.050, 0.016, 0.010)  # the arm standing out from the face: length (x), width (y), thickness (z)
HOOK_TIP = (0.010, 0.016, 0.045)  # the upturned end: thickness (x), width (y), height (z), from the arm's underside
HOOK_ARM_Z = -0.020  # the arm's centre line, below the rail's centre (60 mm below its top edge)

# -- the keyhole slots (in the back) ---------------------------------------------------------------------
KEYHOLE_Y = (-0.150, 0.150)
KEYHOLE_ENTRY = 0.014  # the square entry pocket, 14 mm each way, the full back-layer depth
KEYHOLE_ENTRY_Z = 0.010  # its centre, 30 mm below the rail's top edge
CHANNEL_LENGTH = 0.012  # the channel for the head runs this far up from the entry
THROAT = 0.006  # the surface slot the shank runs in

ENTRY_BOTTOM_Z = KEYHOLE_ENTRY_Z - KEYHOLE_ENTRY / 2
ENTRY_TOP_Z = KEYHOLE_ENTRY_Z + KEYHOLE_ENTRY / 2
CHANNEL_TOP_Z = ENTRY_TOP_Z + CHANNEL_LENGTH

# -- the screws in the wall -------------------------------------------------------------------------------
SCREW_SHANK_D = 0.004
SCREW_HEAD_D = 0.008  # a No. 8 pan head: 8 mm across at its underside, domed to 5 mm at its top
SCREW_HEAD_TOP_D = 0.005
SCREW_HEAD_THICKNESS = 0.0025
SCREW_PROUD = 0.0075  # the head's top stands 7.5 mm off the wall: the lip fits under the head with 2.5 mm to spare
REST_Z = CHANNEL_TOP_Z - SCREW_HEAD_D / 2  # the screw's centre line (rack frame) when the rack hangs: the head's
# top edge rests against the channel's top, 2 mm before the shank would reach the throat's top

# -- the assembly's bounding box ---------------------------------------------------------------------------
RACK_DEPTH = RAIL_THICKNESS + HOOK_ARM[0]  # from the back face to the hooks' tips
RACK_SIZE = (RACK_DEPTH, RAIL_LENGTH, RAIL_HEIGHT)
BACK_X = RACK_DEPTH / 2  # local x of the back face
FRONT_X = BACK_X - RAIL_THICKNESS  # local x of the rail's face
TIP_X = -RACK_DEPTH / 2  # local x of the hooks' outer face
TIP_CENTRE_X = TIP_X + HOOK_TIP[0] / 2
TIP_TOP_Z = HOOK_ARM_Z - HOOK_ARM[2] / 2 + HOOK_TIP[2]


def box_corners(size, centre=(0.0, 0.0, 0.0)) -> np.ndarray:
    """The eight corners of a box, (8, 3)."""
    half = np.asarray(size, dtype=float) / 2
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)
    return np.asarray(centre, dtype=float) + signs * half


def _box(name, x0, x1, y0, y1, z0, z1):
    return (name, ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0))


EASED = {"below_entry": "top", "lip": "bottom"}  # which back pieces have their back edge eased, and which edge


def eased_back_edge_points(size, centre, which: str, chamfer: float = CHAMFER) -> np.ndarray:
    """Hull points of a box whose edge between its back face (+x) and its top or bottom face is chamfered."""
    sx, sy, sz = (float(v) / 2 for v in size)
    zs = sz if which == "top" else -sz
    sign = 1.0 if which == "top" else -1.0
    points = []
    for y in (-sy, sy):
        points += [(-sx, y, -sz), (-sx, y, sz), (sx, y, -zs)]  # the three untouched corners of this end
        points += [(sx - chamfer, y, zs), (sx, y, zs - sign * chamfer)]  # the chamfered edge
    return np.asarray(centre, dtype=float) + np.array(points)


def rack_parts() -> list[tuple[str, tuple, tuple]]:
    """(name, centre, size) of every box the rack is made of, in its local frame."""
    half_l, half_h = RAIL_LENGTH / 2, RAIL_HEIGHT / 2
    front_block_back = BACK_X - BACK_LAYER
    parts = [_box("rail", FRONT_X, front_block_back, -half_l, half_l, -half_h, half_h)]
    # the back layer, in pieces that leave the keyhole slots open
    column = KEYHOLE_ENTRY / 2
    edges = sorted([-half_l, *(k + s * column for k in KEYHOLE_Y for s in (-1, 1)), half_l])
    for i in range(0, len(edges) - 1, 2):  # the full-height pieces between the keyhole columns
        parts.append(_box(f"back_{i // 2}", front_block_back, BACK_X, edges[i], edges[i + 1], -half_h, half_h))
    for j, ky in enumerate(KEYHOLE_Y):
        y0, y1 = ky - column, ky + column
        parts.append(_box(f"below_entry_{j}", front_block_back, BACK_X, y0, y1, -half_h, ENTRY_BOTTOM_Z))
        parts.append(_box(f"above_channel_{j}", front_block_back, BACK_X, y0, y1, CHANNEL_TOP_Z, half_h))
        for side, (ly0, ly1) in enumerate(((y0, ky - THROAT / 2), (ky + THROAT / 2, y1))):
            parts.append(_box(f"lip_{j}_{side}", BACK_X - LIP, BACK_X, ly0, ly1, ENTRY_TOP_Z, CHANNEL_TOP_Z))
    # the hooks
    arm_bottom = HOOK_ARM_Z - HOOK_ARM[2] / 2
    for j, hy in enumerate(HOOK_Y):
        parts.append(_box(f"plate_{j}", FRONT_X - HOOK_PLATE[0], FRONT_X, hy - HOOK_PLATE[1] / 2, hy + HOOK_PLATE[1] / 2,
                          HOOK_ARM_Z - HOOK_PLATE[2] / 2 + 0.005, HOOK_ARM_Z + HOOK_PLATE[2] / 2 + 0.005))  # fmt: skip
        parts.append(_box(f"arm_{j}", TIP_X, FRONT_X - HOOK_PLATE[0], hy - HOOK_ARM[1] / 2, hy + HOOK_ARM[1] / 2,
                          arm_bottom, arm_bottom + HOOK_ARM[2]))  # fmt: skip
        parts.append(_box(f"tip_{j}", TIP_X, TIP_X + HOOK_TIP[0], hy - HOOK_TIP[1] / 2, hy + HOOK_TIP[1] / 2,
                          arm_bottom, arm_bottom + HOOK_TIP[2]))  # fmt: skip
    return parts


def rack_part_points() -> list[tuple[str, np.ndarray]]:
    """Each part's convex-hull points in the rack's local frame."""
    out = []
    for name, centre, size in rack_parts():
        eased = next((edge for prefix, edge in EASED.items() if name.startswith(prefix)), None)
        out.append((name, eased_back_edge_points(size, centre, eased) if eased else box_corners(size, centre)))
    return out


def rack_bounds_local() -> tuple[np.ndarray, np.ndarray]:
    pts = np.concatenate([box_corners(size, centre) for _, centre, size in rack_parts()])
    return pts.min(0), pts.max(0)


def back_face_corners_local() -> np.ndarray:
    """The four corners of the rail's back face, (4, 3)."""
    return np.array([[BACK_X, y, z] for y in (-RAIL_LENGTH / 2, RAIL_LENGTH / 2) for z in (-RAIL_HEIGHT / 2, RAIL_HEIGHT / 2)])


def screw_points(n: int = 12) -> tuple[np.ndarray, np.ndarray]:
    """Hull points of a screw's exposed shank and its domed head, in a frame whose origin is the wall's surface
    and whose -x axis points out of the wall: (shank, head)."""
    angles = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.stack([np.cos(angles), np.sin(angles)], axis=1)

    def disc(diameter, x):
        return np.column_stack([np.full(n, x), ring * diameter / 2])

    head_base = -(SCREW_PROUD - SCREW_HEAD_THICKNESS)
    shank = np.concatenate([disc(SCREW_SHANK_D, 0.0005), disc(SCREW_SHANK_D, head_base)])
    head = np.concatenate([disc(SCREW_HEAD_D, head_base), disc(SCREW_HEAD_TOP_D, -SCREW_PROUD)])
    return shank, head
