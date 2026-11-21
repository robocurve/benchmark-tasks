"""Every dimension, mass and threshold of the cap-bare-splice task, in one place (metres, kilograms, radians).

World frame (yam_arena's): x forward from the line through the arm bases, y to the robot's left, z up; the arm bases
stand at y = +-0.305. ``TOP`` is the station table's top.

The scene is a surface-mounted junction box on a plywood backboard, as in a garage or utility room. A 3/4 in plywood
backboard stands on the table facing the robot (screwed to a 2x4 foot behind it). On it, centred 11 cm above the
table, is a 4 in square steel box, 2-1/8 in deep, its open face toward the robot. Two 14/2 NM-B cables run to it
along the board, one from above and one from below, and enter through knockouts in its top and bottom. It is a
pass-through box: in it the two cables' white conductors are twisted together under a yellow wire nut, their bare
grounds go to the box's green ground screw, and the two black (hot) conductors are twisted together but left bare,
with no nut: the one missing cap.

The box's flat blank cover hangs from its upper-left cover screw, swung up and out of the way (as electricians leave
it while they work). A small black parts tray beside the box holds four yellow winged wire nuts.

Conventions in this file: a part's "centre" is its geometric centre; sizes are full edge lengths.
"""

from __future__ import annotations

import math

import numpy as np

MM = 0.001
INCH = 0.0254
TABLE = 0.7475  # yam_arena.task_file.TABLE_TOP["station"]
TOP = TABLE

# -- colours (sRGB) ----------------------------------------------------------------------------------------------------
PLYWOOD = (0.80, 0.66, 0.46)
STUD_WOOD = (0.84, 0.72, 0.52)
STEEL = (0.70, 0.72, 0.74)  # pre-galvanized steel box and cover
SCREW_ZINC = (0.80, 0.81, 0.83)
GROUND_GREEN = (0.12, 0.55, 0.22)
SHEATH = (0.93, 0.92, 0.87)  # 14/2 NM-B: white outer sheath
BLACK_INS = (0.07, 0.07, 0.08)
WHITE_INS = (0.93, 0.93, 0.91)
COPPER = (0.80, 0.47, 0.26)
NUT_YELLOW = (0.98, 0.80, 0.08)  # Ideal Wing-Nut 451
TRAY_BLACK = (0.14, 0.14, 0.15)

# -- backboard (fixed) ---------------------------------------------------------------------------------------------------
BOARD_FACE_X = 0.460  # the plywood's front face
BOARD_T = 19.0 * MM  # 3/4 in plywood
BOARD_Y = (-0.20, 0.20)  # 40 cm wide
BOARD_H = 0.36  # 36 cm tall, standing on the table
FOOT = (89.0 * MM, 38.0 * MM)  # a 2x4 laid flat behind the board (depth along x, height)

# -- the box (fixed): RACO 232 style, 4 in square, 2-1/8 in deep, 1/16 in steel ---------------------------------------
BOX_Y, BOX_Z = 0.0, TOP + 0.11  # the centre of its open face
BOX_W = 4.0 * INCH  # 101.6 mm outside
BOX_D = 2.125 * INCH  # 54 mm deep
BOX_T = 1.6 * MM
RIM_X = BOARD_FACE_X - BOX_D  # the plane of its open face (rim)
BACK_IN_X = BOARD_FACE_X - BOX_T  # the inside face of its back
INNER_HALF = BOX_W / 2 - BOX_T  # the inside half-width (y and z)

# Cover screws (#8-32) at two diagonally opposite corners of the rim: upper-left (+y, +z) carries the hanging cover.
SCREW_OFF = 45.0 * MM
PIVOT = (RIM_X, BOX_Y + SCREW_OFF, BOX_Z + SCREW_OFF)  # upper-left screw: the cover's pivot
OTHER_SCREW = (RIM_X, BOX_Y - SCREW_OFF, BOX_Z - SCREW_OFF)
SCREW_SHANK_R, SCREW_HEAD_R = 2.1 * MM, 3.6 * MM
SCREW_HEAD_T = 2.0 * MM
EAR = 10.0 * MM  # the screw ears: tabs inside the two corners, at the rim

# -- the cover: RACO 752 style flat blank, 4 in square, 0.07 kg --------------------------------------------------------
COVER_W = 4.0 * INCH
COVER_T = 1.4 * MM
COVER_GAP = 0.2 * MM  # between its back and the rim (it does not collide with the box: see task.py)
COVER_X = RIM_X - COVER_GAP - COVER_T / 2  # its centre plane
COVER_MASS = 0.068
# Drawn (joint 0) hanging open: turned 180 deg about the pivot screw, up and to the left of the box. Closing swings it
# down past the box's left side and onto the box: joint -pi (CLOSED).
CLOSED = -math.pi
COVER_LIMITS = (-math.pi - 0.05, 0.05)
COVER_SPRING = 0.30  # N m/rad toward its drive target: with HOLD_BAND it stays where it is left (a snug screw)
COVER_HOLD_BAND = 0.15  # rad: the target follows the joint, lagging this much (holds about 0.045 N m)
COVER_DAMPING = 0.01
COVER_LATCH = math.radians(2.0)  # within this of closed, it is held closed (stands in for tightening both screws)
COVER_SCREW_HOLE = 5.0 * MM  # its slot round each screw, seen from the front (cosmetic)

# -- the cables (fixed): 14/2 NM-B, sheath 5.3 x 9.9 mm ------------------------------------------------------------------
SHEATH_T, SHEATH_W = 5.3 * MM, 9.9 * MM
SHEATH_IN = 8.0 * MM  # sheath reaching into the box past each knockout
COND_R = 1.5 * MM  # an insulated #14 solid conductor, 3.0 mm across
GROUND_R = 0.8 * MM  # bare #14 ground (1.63 mm)
GROUND_SCREW = (BACK_IN_X - 1.5 * MM, BOX_Y - 30.0 * MM, BOX_Z - 30.0 * MM)

# -- the two splices (each a pair of conductors twisted together), as jointed chains --------------------------------
# Each splice's conductors come in from the top and bottom knockouts, meet at a root near the box's back, and stick
# out of the box toward the robot, the pair side by side, then twisted together for the last 6 cm. The end of each
# twisted pair is stripped 3/8 in (Ideal's strip length for the 451): 9.5 mm of bare copper. The white splice's end is
# under a wire nut; the black's is bare.
ROOT_X = BACK_IN_X - 6.0 * MM
SPLICE_ROOT = {"black": (ROOT_X, BOX_Y - 4.0 * MM, BOX_Z + 4.0 * MM),
               "white": (ROOT_X, BOX_Y + 4.0 * MM, BOX_Z + 4.0 * MM)}
SPLICE_DIR = {"black": (-1.0, -0.33, 0.06), "white": (-1.0, 0.33, 0.06)}  # as drawn, before normalising: the black to the right
LINKS = 7
LINK_L = 20.0 * MM
TWISTED_FROM = 4  # links 4.. (0-based) are the twisted pair; 0..3 the pair side by side
PAIR_R = 1.5 * MM  # each conductor of the side-by-side pair
TWIST_R = 2.0 * MM  # the twisted pair, insulated
COPPER_LEN = 9.5 * MM  # 3/8 in
COPPER_R = 1.7 * MM  # the bare twisted pair
LINK_MASS = 0.003
WIRE_SPRING = 0.6  # N m/rad per joint toward its target
WIRE_HOLD_BAND = 0.08  # rad: bent further than this from its rest shape, the rest shape follows (solid copper stays bent)
WIRE_DAMPING = 0.004
WIRE_LIMIT = 2.6  # rad per joint

# -- winged wire nut: Ideal Wing-Nut 451 (yellow, 30-451): listed 1.02 in (26 mm) long, 0.69 in (17.5 mm) across the
# wings, 0.48 in (12 mm) across the skirt. The real shell is a ribbed taper; here a hexagonal tube with a wider skirt.
# Nut frame: the mouth's centre at the origin, its axis +x into the nut (toward the closed end).
NUT_MASS = 0.004
NUT_IN = 3.0 * MM  # inner apothem of its hexagonal bore
NUT_MOUTH_IN = 4.8 * MM  # inner apothem of the flared skirt (12 mm across its outside)
NUT_MOUTH_LEN = 5.0 * MM  # the skirt, first 5 mm
NUT_WALL = 1.2 * MM
NUT_BORE = 21.5 * MM  # bore depth, mouth to the cap
NUT_CAP = (4.0 * MM, 4.5 * MM)  # radius, length of the closed end (inside the hex's flats, so jaws on the flats clear it)
NUT_WING = (14.0 * MM, 4.5 * MM, 1.8 * MM)  # along the axis, radial, thickness: 2 x (3.0 + 1.2 + 4.5) = 17.4 mm across
NUT_WING_X = 15.0 * MM  # the wings' centre along the axis
NUTS = [  # four loose nuts in the tray: (x, y, yaw, upright). Upright ones stand on their skirts, closed end up, their
    # wings along the yaw direction; the others lie on their sides, their axis along the yaw direction.
    (0.322, -0.135, math.radians(0.0), True),
    (0.275, -0.128, math.radians(100.0), False),
    (0.280, -0.200, math.radians(90.0), True),
    (0.325, -0.205, math.radians(-35.0), False),
]
# An upright nut's body frame: the nut frame (capping.py) turned so its axis points up and its wings lie along body x.
NUT_UPRIGHT_R = np.column_stack([(0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)])
NUT_AXIS_H = NUT_MOUTH_IN + NUT_WALL + 0.3 * MM  # a lying nut's axis above the tray floor (it rests on its skirt)
NUT_STAND_H = 0.3 * MM  # an upright nut's mouth above the tray floor

# -- parts tray (fixed): a small black plastic tray -----------------------------------------------------------------------
TRAY_C = (0.302, -0.172)
TRAY = (0.100, 0.150, 0.025)  # outside x, y, z
TRAY_WALL = 3.0 * MM
TRAY_FLOOR = TOP + TRAY_WALL

# -- capping rule ------------------------------------------------------------------------------------------------------
ENGAGE_DEPTH = 6.0 * MM  # the copper's end this far into the bore, on its axis, starts the threads
RADIAL_MAX = 2.5 * MM  # the copper's end within this of the bore's axis
ALIGN_MAX = math.radians(25.0)  # the nut's axis within this of the copper's
TWIST_NEED = 4.0 * math.pi  # two full clockwise turns (seen from the nut's closed end) makes it bite
CAPPED_DEPTH = COPPER_LEN + 1.0 * MM  # the nut covers all the copper: its end at least this far into the bore
NUT_BREAK_FORCE = 30.0  # N: a nut that bit pulls off above this (a light tug is about 10 N)
NUT_SLIP = (25.0, 90.0)  # mm, deg: only a nut torn off (the score still needs the copper 10.5 mm in at the end)

# -- scoring -----------------------------------------------------------------------------------------------------------
INSIDE_MARGIN = 0.5 * MM  # the demo's fold check: wire points at least this far behind the rim plane
SUCCESS_HOLD_S = 1.0


def unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def splice_dir(name: str) -> np.ndarray:
    return unit(SPLICE_DIR[name])


def link_start(name: str, i: int) -> np.ndarray:
    """Where link ``i`` (0-based) of a splice begins, as drawn."""
    return np.asarray(SPLICE_ROOT[name], float) + i * LINK_L * splice_dir(name)


def splice_tip(name: str) -> np.ndarray:
    """The end of the splice's bare copper, as drawn."""
    return link_start(name, LINKS)


def white_nut_mouth() -> np.ndarray:
    """The white splice's nut, as drawn: its mouth 2 mm back from where the copper starts."""
    return splice_tip("white") - (COPPER_LEN + 2.0 * MM) * splice_dir("white")
