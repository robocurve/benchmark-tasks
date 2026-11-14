"""Every dimension of the task, in one place (metres, kilograms, the station's world frame), real size.

World frame: x forward from the line through the two arm bases, y to the left, z up. The arm bases stand at x 0,
y +-0.305; the station's table top is at z 0.7475. Heights below are above the table top unless named *_W.

A fixture plate (aluminium, 120 x 200 x 20 mm, fixed) on the midline has two locating holes 12.4 mm across, 100.0 mm
apart on a line along y (A on the robot's left, B on its right), 19 mm deep, and four low rest pads. Three loose steel
locating pins hang in two racks: two ROUND and one DIAMOND (relieved). Each pin is a 12 mm shank 18 mm long, a 22 mm
flange 3 mm thick and a 16 mm head 12 mm tall with a stepped lead-in chamfer. The diamond's head keeps two 2 mm lands
on the 16 mm circle and is relieved to 8 mm across its flats between them. A pin pushed home (flange within 0.5 mm of the plate, upright) is locked (a rule, see
task.py). A bracket (aluminium, 150 x 60 x 8 mm) has two 16.4 mm holes whose centres are 101.0 mm apart.

The round holes are 16-sided polygons whose flats are tangent to the true circle (every flat's distance from the
hole's centre is the true radius), built from turned box colliders, with a 45 degree lead-in funnel.
"""

import math

import numpy as np

import mechkit as mk

MM = 0.001
TOP = 0.7475  # station table top (world z)

# -- the fixture plate ------------------------------------------------------------------------------------------------
FIX_X, FIX_Y = 0.300, 0.000
FIX_LEN_X, FIX_LEN_Y, FIX_H = 0.120, 0.200, 0.020
FIX_TOP = FIX_H  # 20 mm above the table: the plate's top face
FIX_BASE = 0.001  # solid floor under the holes (the plate is one slab 1 mm thick under them)
HOLE_D = 0.0124  # locating holes
HOLE_R = HOLE_D / 2  # 6.2 mm
HOLE_DEPTH = 0.019  # from the plate's top: 1 mm more than the shank's length, so the shank's tip hangs clear of the floor
HOLE_SPACING = 0.100
HOLES = {"A": (FIX_X, FIX_Y + HOLE_SPACING / 2), "B": (FIX_X, FIX_Y - HOLE_SPACING / 2)}  # A left, B right
FUNNEL = 0.002  # 45 degree lead-in at a plate hole's mouth: 2 mm deep, 8.2 mm radius at the top
STAVE_T = 0.003  # radial thickness of the box staves that make a round hole
CUT_HALF = 0.0092  # the square the plate is cut away to around a hole: the staves fill its inside
SIDES = 16  # sides of a round hole (flat normals at multiples of 22.5 degrees: along x and y)
PAD_H = 0.005  # rest pads: 5 mm on the plate, so the bracket sits above the pins' 3 mm flanges
PAD_SIZE = 0.010
PAD_XY = ((0.021, 0.050), (0.021, -0.050), (-0.021, 0.050), (-0.021, -0.050))  # (dx, dy) from the fixture's centre
PAD_TOP = FIX_TOP + PAD_H  # 25 mm: the bracket's underside when seated

# -- the pins ---------------------------------------------------------------------------------------------------------
# Base frame at the centre of the flange's underside, local +z up the axis, local +x along the diamond head's wide
# axis (its 16 mm lands). The shank hangs below it.
SHANK_R, SHANK_L = 0.006, 0.018
FLANGE_R, FLANGE_T = 0.011, 0.003
HEAD_R, HEAD_H = 0.008, 0.012
HEAD_STEPS = ((0.0105, 0.008), (0.0005, 0.0075), (0.0005, 0.0070), (0.0005, 0.0065))  # (height, radius), from flange
DIAMOND_FLATS = 0.008  # the diamond head measures this across its flats (its narrow axis, local y)
DIAMOND_LAND = 0.001  # half width of each land: two 2 mm lands at the ends of the 16 mm axis (local x)
DIAMOND_BODY_X = 0.0055  # between the lands the relieved body is 11 mm by 8 mm
STEEL = 7850.0  # kg/m3
ALUMINIUM = 2700.0
ROUND = ("round_pin_1", "round_pin_2")
DIAMOND = "diamond_pin"
PIN_NAMES = (*ROUND, DIAMOND)
PIN_SEAT_Z = FIX_TOP  # a home pin's origin (flange underside) above the table
HOME_TOL = 0.0005  # flange within 0.5 mm of the plate
HOME_TILT_DEG = 3.0
HOME_XY_TOL = 0.001  # a home pin's axis within 1 mm of a hole's centre
HEAD_TOP = FLANGE_T + HEAD_H  # 15 mm above the flange's underside

# -- the racks and the starting marks ---------------------------------------------------------------------------------
RACK_H = 0.022  # the rack's top: the flange rests on it, the shank hangs in a hole
RACK_FLOOR = 0.002
RACK_HOLE = 0.014  # square holes 14 mm
RIGHT_RACK_X, RIGHT_RACK_Y, RIGHT_RACK_SIZE = 0.220, -0.265, (0.044, 0.100)
LEFT_RACK_X, LEFT_RACK_Y, LEFT_RACK_SIZE = 0.235, 0.250, (0.044, 0.044)
ROUND_START = {"round_pin_1": (0.220, -0.235), "round_pin_2": (0.220, -0.295)}  # 60 mm apart along y
DIAMOND_START = (LEFT_RACK_X, LEFT_RACK_Y)
DIAMOND_START_YAW = math.pi / 2  # wide axis along y: along the line between the holes (turned wrong)
RAIL_H = 0.012  # two rails the bracket lies on at the start, so the fingers can reach its sides
RAIL_X = (-0.016, 0.016)  # offsets from the bracket's centre
RAIL_W, RAIL_L = 0.010, 0.110

# -- the bracket ------------------------------------------------------------------------------------------------------
# Base frame at the centre of the underside, local +z up, local +y along its length.
BR_LEN_X, BR_LEN_Y, BR_T = 0.060, 0.150, 0.008
BR_HOLE_D = 0.0164
BR_HOLE_R = BR_HOLE_D / 2  # 8.2 mm
BR_HOLE_SPACING = 0.1010  # 101.0 mm: a fabrication error (1.0 mm) against the fixture's 100.0
BR_HOLE_Y = BR_HOLE_SPACING / 2  # 50.3 mm
BR_FUNNEL = 0.0015  # lead-in at the underside of each hole: 1.5 mm deep
BR_CUT_HALF = 0.0114
BR_START = (0.135, 0.110)
BR_MASS = ALUMINIUM * (BR_LEN_X * BR_LEN_Y * BR_T - 2 * math.pi * BR_HOLE_R ** 2 * BR_T)  # about 185 g
BR_CORNERS = tuple((sx * BR_LEN_X / 2, sy * BR_LEN_Y / 2, 0.0) for sx in (-1, 1) for sy in (-1, 1))

# -- what the geometry gives (see fitgeom.py and the tests) -------------------------------------------------------------
ROUND_CLEAR = BR_HOLE_R - HEAD_R  # 0.2 mm all round a round head in a bracket hole
SPACING_ERROR = BR_HOLE_SPACING - HOLE_SPACING  # 1.0 mm
# With the round pin in hole H the bracket is located with that hole concentric on the pin: the bracket's centre is
# SPACING_ERROR / 2 along the line from the fixture's centre, toward the other hole (y -0.5 mm with the round pin in A).

# -- rule thresholds (scoring.py) ------------------------------------------------------------------------------------
TURN_TOL_DEG = 15.0  # the diamond's wide axis within 15 degrees of square to the line between the holes
SEAT_POS_TOL = 0.0003  # the bracket's hole over the round pin: its centre within 0.3 mm of the pin's axis
SEAT_Z_TOL = 0.0003  # ... its underside within 0.3 mm of the pads' top
SEAT_TURN_DEG = 0.3  # ... square to the line within 0.3 degrees, level within 0.3 degrees
STILL_WINDOW_S = 1.0
STILL_MOVE = 0.0003  # 'at rest': the bracket's origin and the holes' centres move under 0.3 mm in the last 1 s
STILL_TURN_DEG = 0.3
LET_GO = 0.025  # both grippers' finger zones at least 25 mm from the bracket and the pins
OFF_TABLE_DROP = 0.010
TABLE_X0, TABLE_X1 = 0.36 - 0.55, 0.36 + 0.55
TABLE_Y0, TABLE_Y1 = -0.65, 0.65
CONTROL_HZ = 20.0
RESET_JUMP = 0.08
START_TOL = 0.045
EPS = 1e-9

# -- colours (sRGB) ---------------------------------------------------------------------------------------------------
FIXTURE_GREY = (0.62, 0.64, 0.67)  # bare aluminium
PAD_DARK = (0.30, 0.31, 0.33)
PIN_STEEL = (0.78, 0.79, 0.81)
HEAD_BLACK = (0.08, 0.08, 0.09)  # the diamond's head is blackened: the mark
BRACKET_BLUE = (0.16, 0.36, 0.78)
RACK_BLACK = (0.13, 0.13, 0.14)


# -- shape builders ---------------------------------------------------------------------------------------------------
def _rot(cols) -> np.ndarray:
    return np.array(cols, dtype=float).T


def ring(cx, cy, z0, z1, r_in, thick=STAVE_T, sides=SIDES, color=None):
    """A round hole's wall between heights z0 and z1: ``sides`` box staves whose inner faces are tangent to the circle
    of radius ``r_in`` about (cx, cy), flat normals at multiples of 360/sides degrees from +x."""
    out = []
    tang = 2 * r_in * math.tan(math.pi / sides) + 0.0006
    for k in range(sides):
        a = 2 * math.pi * k / sides
        u, t = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
        c = np.array([cx, cy]) + u * (r_in + thick / 2)
        out.append(mk.box((c[0], c[1], (z0 + z1) / 2), (thick, tang, z1 - z0), color, ypr=(a, 0.0, 0.0)))
    return out


def funnel(cx, cy, z_edge, depth, r_in, up=True, sides=SIDES, slab=0.0012, color=None):
    """A 45 degree lead-in of ``depth``: staves turned 45 degrees, their inner surface running from radius ``r_in`` at
    ``depth`` from the face to ``r_in + depth`` at the face. ``z_edge`` is the face's height; ``up``: the face is
    the top of the hole (a plate's mouth), else the bottom (the bracket's underside)."""
    out = []
    tang = 2 * (r_in + depth) * math.tan(math.pi / sides) + 0.0008
    s2 = math.sqrt(0.5)
    sign = 1.0 if up else -1.0
    for k in range(sides):
        a = 2 * math.pi * k / sides
        u, t = np.array([math.cos(a), math.sin(a), 0.0]), np.array([-math.sin(a), math.cos(a), 0.0])
        z = np.array([0.0, 0.0, 1.0])
        # in the (radial, up) plane the surface runs along d, the material lies on the side n
        d = s2 * u + sign * s2 * z
        n = s2 * u - sign * s2 * z
        mid = np.array([cx, cy, z_edge - sign * depth / 2]) + u * (r_in + depth / 2)
        centre = mid + n * slab / 2
        r = _rot([d, t, np.cross(d, t)])
        out.append(mk.box(tuple(centre), (depth * math.sqrt(2), tang, slab), color, quat=tuple(mk.matrix_to_quat(r))))
    return out


def hole_polygon(r_in=HOLE_R, sides=SIDES):
    """The free region of a ring hole: the intersection of half planes n . p <= r_in (flat normals at multiples of
    360/sides degrees from +x). Returns (normals (N, 2), r_in)."""
    a = 2 * math.pi * np.arange(sides) / sides
    return np.stack([np.cos(a), np.sin(a)], axis=1), r_in


def fixture_shapes():
    """The plate in its own frame: origin at the centre of its footprint on the table (x along the plate's 120 mm,
    y along its 200 mm, z up). A 1 mm floor slab, the plate from 1 to 20 mm cut to a square about each hole, the
    holes' staves and funnels, and the four rest pads."""
    shapes = [mk.box((0, 0, FIX_BASE / 2), (FIX_LEN_X, FIX_LEN_Y, FIX_BASE), FIXTURE_GREY)]
    holes = [(0.0, HOLE_SPACING / 2 * s, 2 * CUT_HALF, 2 * CUT_HALF) for s in (1, -1)]
    shapes += mk.plate_with_holes((0, 0, (FIX_BASE + FIX_TOP) / 2), (FIX_LEN_X, FIX_LEN_Y), holes, FIX_TOP - FIX_BASE,
                                  FIXTURE_GREY)
    for s in (1, -1):
        cy = HOLE_SPACING / 2 * s
        shapes += ring(0.0, cy, FIX_BASE, FIX_TOP - FUNNEL, HOLE_R, color=FIXTURE_GREY)
        shapes += funnel(0.0, cy, FIX_TOP, FUNNEL, HOLE_R, up=True, color=FIXTURE_GREY)
    for dx, dy in PAD_XY:
        shapes.append(mk.box((dx, dy, FIX_TOP + PAD_H / 2), (PAD_SIZE, PAD_SIZE, PAD_H), PAD_DARK))
    return shapes


def bracket_shapes():
    """The bracket in its own frame (origin at the centre of its underside): the plate cut to a square about each
    hole, each hole's staves and the funnel at the underside."""
    holes = [(0.0, BR_HOLE_Y * s, 2 * BR_CUT_HALF, 2 * BR_CUT_HALF) for s in (1, -1)]
    shapes = list(mk.plate_with_holes((0, 0, BR_T / 2), (BR_LEN_X, BR_LEN_Y), holes, BR_T, BRACKET_BLUE))
    for s in (1, -1):
        cy = BR_HOLE_Y * s
        shapes += ring(0.0, cy, BR_FUNNEL, BR_T, BR_HOLE_R, color=BRACKET_BLUE)
        shapes += funnel(0.0, cy, 0.0, BR_FUNNEL, BR_HOLE_R, up=False, color=BRACKET_BLUE)
    return shapes


def pin_shapes(diamond: bool):
    """A pin in its own frame (origin at the centre of the flange's underside)."""
    head_color = HEAD_BLACK if diamond else PIN_STEEL
    shapes = [
        mk.cylinder((0, 0, -(SHANK_L - 0.0005) / 2), SHANK_R, SHANK_L - 0.0005, "Z", PIN_STEEL),
        mk.cylinder((0, 0, -SHANK_L + 0.00025), SHANK_R - 0.0005, 0.0005, "Z", PIN_STEEL),  # lead-in at the tip
        mk.cylinder((0, 0, FLANGE_T / 2), FLANGE_R, FLANGE_T, "Z", PIN_STEEL),
    ]
    z = FLANGE_T
    for h, r in HEAD_STEPS:
        if not diamond:
            shapes.append(mk.cylinder((0, 0, z + h / 2), r, h, "Z", head_color))
        else:
            ly, bx = DIAMOND_LAND, min(DIAMOND_BODY_X, math.sqrt(r * r - (DIAMOND_FLATS / 2) ** 2))
            shapes.append(mk.box((0, 0, z + h / 2), (2 * math.sqrt(r * r - ly * ly), 2 * ly, h), head_color))  # lands
            shapes.append(mk.box((0, 0, z + h / 2), (2 * bx, DIAMOND_FLATS, h), head_color))  # relieved body
        z += h
    return shapes


def pin_mass(diamond: bool) -> float:
    """Solid steel (7.85 g/cm3), the chamfer steps ignored: about 44 g round, 36 g diamond."""
    v = math.pi * SHANK_R ** 2 * SHANK_L + math.pi * FLANGE_R ** 2 * FLANGE_T
    if diamond:
        ly, bx = DIAMOND_LAND, DIAMOND_BODY_X
        area = 4 * ly * math.sqrt(HEAD_R ** 2 - ly ** 2) + 2 * bx * DIAMOND_FLATS - 2 * bx * 2 * ly
        v += area * HEAD_H
    else:
        v += math.pi * HEAD_R ** 2 * HEAD_H
    return STEEL * v


def rack_shapes(size, holes):
    """A rack in its own frame (origin at the centre of its footprint on the table): a floor and a block 22 mm tall
    with square 14 mm holes at ``holes`` (dx, dy)."""
    sx, sy = size
    shapes = [mk.box((0, 0, RACK_FLOOR / 2), (sx, sy, RACK_FLOOR), RACK_BLACK)]
    shapes += mk.plate_with_holes((0, 0, (RACK_FLOOR + RACK_H) / 2), size, [(dx, dy, RACK_HOLE, RACK_HOLE) for dx, dy
                                                                            in holes], RACK_H - RACK_FLOOR, RACK_BLACK)
    return shapes
