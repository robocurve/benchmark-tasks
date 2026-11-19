"""Geometry, materials and object builders for ceiling-tile-lay-in (real size, metres).

A section of 15/16 in exposed-tee suspended ceiling grid (2 ft x 2 ft bays, 2 rows x 3 columns) hung on a test frame
over the bench, with three strut channels across the plenum above it; loose lay-in tiles in the five neighbouring
bays; the sixth tile lies askew (turned 12 deg) across the tops of the four tees round its own opening.

World frame: x away from the arms, y to their left, z up; station bench top z = TOP.
Arm bases at x 0, y +-0.305, z 0.75.
"""
from __future__ import annotations

from simkit import Part, Piece

TOP = 0.7475

# -- grid -------------------------------------------------------------------------------------------------
MODULE = 0.6096                    # 24 in tee centres
FLANGE_W = 0.024                   # 15/16 in face
FLANGE_T = 0.002                   # folded steel face (collider a little thicker than the real ~0.8 mm)
WEB_T = 0.002                      # double-wall web
WEB_H = 0.038                      # 1-1/2 in tall tees (flange face to bulb top)
BULB_W, BULB_H = 0.003, 0.008      # rolled bulb on top of the web
GRID_H = 0.460                     # flange underside above the bench
FLANGE_Z = TOP + GRID_H + FLANGE_T  # flange top face: the tiles' bearing plane
XC, YC = 0.500, 0.0                # target bay centre
CLEAR = MODULE - FLANGE_W          # clear opening between flange edges (585.6 mm)
NX, NY = 2, 3                      # bays: rows along x (target in the near row), columns along y (target in the middle)
X_LINES = [XC - MODULE / 2 + i * MODULE for i in range(NX + 1)]
Y_LINES = [YC - 1.5 * MODULE + j * MODULE for j in range(NY + 1)]

# -- plenum sheet (stands for the structural deck / duct bottoms above the grid) --------------------------
DECK_GAP = 0.220                   # deck underside above the flange plane
DECK_T = 0.006
DECK_Z = FLANGE_Z + DECK_GAP
STRUT = 0.041                      # 1-5/8 in strut channel
STRUT_X = (XC - 0.20, XC + 0.20, XC + MODULE)

# -- tile -------------------------------------------------------------------------------------------------
TILE = 0.6033                      # 23-3/4 in square lay-in
TILE_T = 0.016                     # 5/8 in
TILE_MASS = 1.10
TILE_COLOR = (0.93, 0.92, 0.89)
# as left after plenum access: pushed up and turned, lying askew across the tops of the four tees round its opening
START_YAW_DEG = 12.0
TILE_START = (XC + 0.006, YC - 0.004, FLANGE_Z + WEB_H + TILE_T / 2 + 0.0005)

STEEL_WHITE = (0.95, 0.95, 0.94)
FRAME_GREY = (0.62, 0.63, 0.65)

TARGET = (0, 1)


def bay_centre(i, j):
    return (X_LINES[i] + MODULE / 2, Y_LINES[j] + MODULE / 2)


NEIGHBOURS = [(i, j) for i in range(NX) for j in range(NY) if (i, j) != TARGET]


def _rel(ps, root):
    return tuple(Piece(p.shape, p.size, (p.offset[0] - root[0], p.offset[1] - root[1], p.offset[2] - root[2]),
                       rot=p.rot, axis=p.axis, color=p.color, collide=p.collide, opacity=p.opacity) for p in ps)


def grid_object() -> Part:
    """All tees' webs and bulbs as one fixed body (hung from the test frame); the flanges are separate fixed
    bodies (flange_objects)."""
    ps, fl = [], []
    x0, x1 = X_LINES[0], X_LINES[-1]
    zf = FLANGE_Z - FLANGE_T / 2
    zw = FLANGE_Z + (WEB_H - BULB_H) / 2
    zb = FLANGE_Z + WEB_H - BULB_H / 2
    for y in Y_LINES:              # main tees along x (one collider per bay length)
        for i in range(NX):
            xa, xb = X_LINES[i] - FLANGE_W / 2, X_LINES[i + 1] + FLANGE_W / 2
            cx, L = (xa + xb) / 2, xb - xa
            fl.append(((L, FLANGE_W, FLANGE_T), (cx, y, zf)))
            ps.append(Piece("box", (L, WEB_T, WEB_H - BULB_H), (cx, y, zw), color=STEEL_WHITE))
            ps.append(Piece("box", (L, BULB_W, BULB_H), (cx, y, zb), color=STEEL_WHITE))
    for x in X_LINES:              # cross tees along y, between the mains
        for j in range(NY):
            ya, yb = Y_LINES[j] + FLANGE_W / 2, Y_LINES[j + 1] - FLANGE_W / 2
            cy, L = (ya + yb) / 2, yb - ya
            fl.append(((FLANGE_W, L, FLANGE_T), (x, cy, zf)))
            ps.append(Piece("box", (WEB_T, L, WEB_H - BULB_H), (x, cy, zw), color=STEEL_WHITE))
            ps.append(Piece("box", (BULB_W, L, BULB_H), (x, cy, zb), color=STEEL_WHITE))
    root = (XC, YC, FLANGE_Z)
    ps += [Piece("box", sz, c, color=STEEL_WHITE) for sz, c in fl]
    return Part("grid", pos=root, size=(0.004, 0.004, 0.002), exact_z=True, color=STEEL_WHITE, friction=(0.35, 0.30),
                friction_combine="min",
                contact_offset=0.002, pieces=_rel(ps, root), hide_root=True,
                collision="boundingCube")


def frame_object() -> Part:
    """The test frame: four 40 mm aluminium posts at the section's corners, rails, the clear plenum sheet."""
    x0, x1 = X_LINES[0] - 0.03, X_LINES[-1] + 0.03
    y0, y1 = Y_LINES[0] - 0.03, Y_LINES[-1] + 0.03
    hz = DECK_Z + DECK_T - TOP
    ps = []
    for x in (x0, x1):
        for y in (y0, y1):
            ps.append(Piece("box", (0.04, 0.04, hz), (x, y, TOP + hz / 2), color=FRAME_GREY))
    for z in (DECK_Z - 0.02,):
        for x in (x0, x1):
            ps.append(Piece("box", (0.04, y1 - y0, 0.04), (x, (y0 + y1) / 2, z), color=FRAME_GREY))
        for y in (y0, y1):
            ps.append(Piece("box", (x1 - x0, 0.04, 0.04), ((x0 + x1) / 2, y, z), color=FRAME_GREY))
    for y in Y_LINES:              # hanger wires (visual)
        for x in (XC - 0.1, XC + MODULE + 0.1):
            ps.append(Piece("cylinder", (0.001, DECK_GAP - 0.04, 0.001), (x, y, FLANGE_Z + 0.04 + (DECK_GAP - 0.04) / 2),
                            color=(0.7, 0.7, 0.7), collide=False))
    for x in STRUT_X:              # 1-5/8 in strut channels across the plenum, carried by the deck rails
        ps.append(Piece("box", (STRUT, y1 - y0, STRUT), (x, (y0 + y1) / 2, DECK_Z + STRUT / 2), color=(0.55, 0.57, 0.58)))
    root = ((x0 + x1) / 2, (y0 + y1) / 2, TOP + 0.002)
    return Part("frame", pos=root, size=(0.004, 0.004, 0.004), exact_z=True, color=FRAME_GREY, friction=(0.5, 0.4),
                contact_offset=0.002, pieces=_rel(ps, root), hide_root=True)


def tile_part(name, pos, yaw_deg=0.0) -> Part:
    import math
    return Part(name, pos=pos, yaw=math.radians(yaw_deg), size=(TILE, TILE, TILE_T), exact_z=True, mass=TILE_MASS, color=TILE_COLOR,
                material="matte", friction=(1.0, 0.9), contact_offset=0.002, collision="boundingCube")


def neighbour_name(i, j):
    return f"tile_{i}{j}"


def neighbour_objects():
    out = []
    for (i, j) in NEIGHBOURS:
        cx, cy = bay_centre(i, j)
        out.append(tile_part(neighbour_name(i, j), (cx, cy, FLANGE_Z + TILE_T / 2 + 0.0003)))
    return out


def scene_objects() -> tuple:
    return (grid_object(), frame_object(), tile_part("tile", TILE_START, START_YAW_DEG), *neighbour_objects())


OBJECT_NAMES = ("tile",) + tuple(neighbour_name(i, j) for (i, j) in NEIGHBOURS)


# -- the overview (recording-only) camera: below the ceiling, off to the right front, looking up at the bay ----
OVERVIEW_EYE = (1.12, -0.78, TOP + 0.10)
OVERVIEW_TARGET = (0.40, 0.02, TOP + 0.42)

