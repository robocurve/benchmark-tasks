"""Geometry and measurements for sealing a shipping carton with a hand tape gun.

On the bench: a filled carton whose two long top flaps are folded down but spring back up a little
(the board's memory), and a hand tape dispenser ("tape gun") with a roll of 48 mm packing tape
standing on its roller and foot. The carton is sealed with one strip of tape along the flaps' seam,
which runs away from the arms, lapping down both end faces.
Frame: metres, x away from the arm bases, y left, z up; TOP is the bench.
"""
from __future__ import annotations

import math

import numpy as np

from simkit import Hinge, Part, Piece

TOP = 0.7475

BOARD = (0.70, 0.55, 0.36)           # kraft corrugated board
BOARD_EDGE = (0.62, 0.48, 0.31)
TAPE_COL = (0.80, 0.62, 0.36)        # tan packing tape
GUN_COL = (0.75, 0.10, 0.10)
RUBBER = (0.08, 0.08, 0.09)
STEEL = (0.70, 0.71, 0.73)

# -- carton (free) with its long flaps hinged to it ---------------------------------------------
CARTON_C = (0.250, -0.050)
CARTON = (0.160, 0.160, 0.050)       # outside: x (along the seam) by y (across it) by z: a flat parts carton
WALL = 0.004
CARTON_MASS = 2.5                    # its contents
FLAP_W = CARTON[1] / 2               # each long flap reaches the seam
FLAP_POP_DEG = 18.0                  # how far an unheld flap springs back up
FLAP_K = 0.004                       # N m per degree, toward FLAP_POP_DEG
FLAP_MASS = 0.030
RIM_Z = TOP + CARTON[2]
TOP_SURF = RIM_Z + WALL              # the closed flaps' top face
FLAPS = ("flap_l", "flap_r")         # hinged on the left (+y) and right (-y) top edges
SEAM_Y = CARTON_C[1]
END_X = CARTON[0] / 2                # the end faces at x = centre +- END_X


def carton_pos():
    return np.array([CARTON_C[0], CARTON_C[1], TOP + WALL / 2])


def flap_sign(name):
    return 1 if name == "flap_l" else -1     # which side of the seam its hinge is on, in y


# -- the tape, as cells along the strip -------------------------------------------------------------
# s runs along the strip from the near top edge: negative down the near end face, 0..L along the top
# away from the arms, beyond L down the far end face.
TAPE_W = 0.048
CELL = 0.008
TAB_CELLS = 5                        # cells down each 50 mm end face (40 mm of it)
TOP_CELLS = int(round(CARTON[0] / CELL))
STRIP_L = CARTON[0]
CELLS = 2 * TAB_CELLS + TOP_CELLS
LEADER = 0.075                       # tape hanging from the gun's roller before it is started
TAB_MIN = 0.024                      # tab down each end face, for the tier: half the 50 mm end


def cell_s(k):
    """Centre of cell k along the strip."""
    return (k - TAB_CELLS + 0.5) * CELL


def cell_local(k, offset=0.0):
    """Cell k's centre in the carton's frame (root at the bottom's centre), and whether it is on a face."""
    s = cell_s(k)
    top = TOP_SURF - (TOP + WALL / 2)
    if s < 0:
        return np.array([-END_X - 0.0003, offset, top + s]), "near"
    if s > STRIP_L:
        return np.array([END_X + 0.0003, offset, top - (s - STRIP_L)]), "far"
    return np.array([-END_X + s, offset, top + 0.0003]), "top"


def carton_object() -> Part:
    """Bottom (root), four walls, the two short end flaps folded flat under the long ones, the contents,
    and the tape cells (hidden until laid)."""
    lx, ly, h = CARTON
    zc = h / 2
    p = [Piece("box", (WALL, ly, h - WALL), ((lx - WALL) / 2, 0.0, zc)),
         Piece("box", (WALL, ly, h - WALL), (-(lx - WALL) / 2, 0.0, zc)),
         Piece("box", (lx - 2 * WALL, WALL, h - WALL), (0.0, (ly - WALL) / 2, zc)),
         Piece("box", (lx - 2 * WALL, WALL, h - WALL), (0.0, -(ly - WALL) / 2, zc)),
         # short end flaps, folded in flat under the long ones
         Piece("box", (0.060, ly - 2 * WALL, WALL), (lx / 2 - 0.030 - WALL, 0.0, h - WALL)),
         Piece("box", (0.060, ly - 2 * WALL, WALL), (-(lx / 2 - 0.030 - WALL), 0.0, h - WALL)),
         # the contents, so the top is firm to press on
         Piece("box", (lx - 2 * WALL - 0.002, ly - 2 * WALL - 0.002, h - 3 * WALL - 0.004),
               (0.0, 0.0, (h - 3 * WALL - 0.004) / 2 + WALL / 2), color=(0.55, 0.45, 0.30))]
    for k in range(CELLS):
        c, where = cell_local(k)
        size = (CELL + 0.0004, TAPE_W, 0.0004) if where == "top" else (0.0004, TAPE_W, CELL + 0.0004)
        p.append(Piece("box", size, tuple(float(v) for v in c), color=TAPE_COL, opacity=0.9, collide=False))
    return Part("carton", pos=tuple(float(v) for v in carton_pos()), size=(lx, ly, WALL), exact_z=True,
                mass=CARTON_MASS, color=BOARD, friction=(0.7, 0.6), restitution=0.0, contact_offset=0.002,
                com=(0.0, 0.0, 0.05), pieces=tuple(p))


TAPE_PIECE0 = 7                      # index of the first tape cell among the carton's pieces


def flap_object(name) -> Part:
    s = flap_sign(name)
    lx, ly, h = CARTON
    hinge_y = CARTON_C[1] + s * ly / 2
    c = np.array([CARTON_C[0], hinge_y - s * FLAP_W / 2, RIM_Z + WALL / 2])
    pivot = np.array([CARTON_C[0], hinge_y, RIM_Z + WALL / 2])
    crease = Piece("box", (lx, 0.003, WALL + 0.0004), (0.0, s * (FLAP_W / 2 - 0.0015), 0.0), color=BOARD_EDGE,
                   collide=False)
    # about +x, the left flap's free edge (-y of its hinge) rises for negative angles, the right one's for positive
    lo, hi = (-100.0, 1.0) if s > 0 else (-1.0, 100.0)
    return Part(name, pos=tuple(float(v) for v in c), size=(lx, FLAP_W - 0.001, WALL), exact_z=True, mass=FLAP_MASS,
                color=BOARD, friction=(0.7, 0.6), restitution=0.0, contact_offset=0.0015, pieces=(crease,),
                hinge=Hinge(pivot=tuple(float(v) for v in pivot), axis="X", damping=0.002, lower_deg=lo, upper_deg=hi,
                            stiffness=FLAP_K, target_deg=-s * FLAP_POP_DEG, parent="carton",
                            parent_pos=tuple(float(v) for v in carton_pos())))


def flap_angle(name, pose):
    """How far the flap stands open (degrees, >= 0)."""
    r = quat_matrix(np.asarray(pose)[3:7])
    v = r @ np.array([0.0, -flap_sign(name), 0.0])     # from hinge toward its free edge
    return max(0.0, math.degrees(math.asin(np.clip(v[2], -1, 1))))


# -- tape gun (free) ------------------------------------------------------------------------------
# Local frame: x forward (roller and blade at the front), y across, z up toward the handle.
GUN_REST = (0.170, -0.220)
GUN_YAW = 0.0                        # front away from the arms: it is run from the near end to the far end
GUN_MASS = 0.35
ROLL_R = 0.040
ROLLER = (0.011, 0.054)              # radius, width
ROLLER_AT = (0.050, -0.042)          # roller axis, x z from the gun centre
BLADE_AT = (0.068, -0.050)           # blade edge, x z: just ahead of and above the roller's bottom
HANDLE = (0.014, 0.045)              # radius, length: a grip standing just behind the roll, over the balance point
HANDLE_AT = (-0.030, 0.062)          # handle centre, x z
PLATE = (0.100, 0.006, 0.090)
PLATE_X = -0.028                     # side plates stop short of the roller, so it shows
FOOT_AT = (-0.050, -0.053)           # foot bottom level with the roller's bottom
SIDE = TAPE_W / 2 + 0.006            # the frame's side plate, off the gun's centre plane
ROOT_LOCAL = (PLATE_X, SIDE, 0.0)    # the near side plate's centre (the gun's root) in the gun frame


def gun_ground():
    """Gun centre above the bench when standing on its roller and foot."""
    return -(ROLLER_AT[1] - ROLLER[0])


def gun_rot(yaw=GUN_YAW, pitch=0.0):
    """Gun orientation: yawed, then its nose pitched down by ``pitch`` about its own y axis."""
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    cp, sp = math.cos(pitch), math.sin(pitch)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    return rz @ ry


def gun_object() -> Part:
    g = np.array([GUN_REST[0], GUN_REST[1], TOP + gun_ground()])
    r = gun_rot()
    root_local = np.array(ROOT_LOCAL)

    def at(x, y, z):
        return tuple(float(v) for v in np.array([x, y, z]) - root_local)

    pieces = (
        Piece("box", PLATE, at(PLATE_X, -SIDE, 0.0)),                                           # far side plate
        Piece("box", (0.040, 0.005, 0.010), at(ROLLER_AT[0] - 0.016, SIDE, ROLLER_AT[1] + 0.004), collide=False),
        Piece("box", (0.040, 0.005, 0.010), at(ROLLER_AT[0] - 0.016, -SIDE, ROLLER_AT[1] + 0.004), collide=False),  # axle arms
        Piece("cylinder", (ROLL_R, TAPE_W, ROLL_R), at(-0.012, 0.0, 0.004), axis="Y", color=TAPE_COL, opacity=0.85,
              collide=False),                                                                    # tape roll
        Piece("cylinder", (0.019, TAPE_W + 0.002, 0.019), at(-0.012, 0.0, 0.004), axis="Y", color=(0.55, 0.45, 0.32),
              collide=False),                                                                    # its core
        Piece("box", (0.002, TAPE_W + 0.006, 0.012), at(BLADE_AT[0], 0.0, BLADE_AT[1] + 0.006), color=STEEL,
              collide=False),                                                                    # serrated blade
        Piece("box", (0.024, TAPE_W + 0.012, 0.006), at(FOOT_AT[0], 0.0, FOOT_AT[1] + 0.003), color=RUBBER),  # foot
        Piece("box", (2 * HANDLE[0] + 0.006, 2 * HANDLE[0], HANDLE[1]), at(HANDLE_AT[0], 0.0, HANDLE_AT[1]), color=RUBBER,
              friction=(1.4, 1.2)),                                                              # handle: a flat-sided grip
        Piece("box", (0.030, TAPE_W + 0.012, 0.008), at(HANDLE_AT[0] + 0.010, 0.0, HANDLE_AT[1] - HANDLE[1] / 2)),  # handle bridge
        Piece("box", (0.002, TAPE_W, 0.040), at(ROLLER_AT[0] + ROLLER[0] + 0.001, 0.0, ROLLER_AT[1] + 0.024),
              color=TAPE_COL, opacity=0.85, collide=False),                                      # tape coming off the roll
    )
    root = g + r @ root_local
    return Part("gun", pos=tuple(float(v) for v in root), size=PLATE, exact_z=True, mass=GUN_MASS, color=GUN_COL,
                friction=(0.6, 0.5), restitution=0.0, contact_offset=0.0015, yaw=GUN_YAW,
                com=at(-0.020, 0.0, 0.010), pieces=pieces)


def roller_object() -> Part:
    """The rubber pressure roller, on its own axle: a free-turning hinge to the gun's frame, so it rolls."""
    g = np.array([GUN_REST[0], GUN_REST[1], TOP + gun_ground()])
    r = gun_rot()
    c = g + r @ np.array([ROLLER_AT[0], 0.0, ROLLER_AT[1]])
    marks = []
    for sy in (-1, 1):                                              # yellow end caps with a black bar across: its turning shows
        y = sy * (ROLLER[1] / 2 + 0.0008)
        marks.append(Piece("cylinder", (ROLLER[0] + 0.0015, 0.0016, ROLLER[0] + 0.0015), (0.0, y, 0.0), axis="Y",
                           color=(0.98, 0.78, 0.05), collide=False))
        marks.append(Piece("box", (0.004, 0.0020, 2 * ROLLER[0] + 0.002), (0.0, y + sy * 0.0003, 0.0),
                           color=(0.05, 0.05, 0.05), collide=False))
    for k in range(4):                                              # white stripes along the tyre
        a = k * math.pi / 2
        marks.append(Piece("box", (0.0035, ROLLER[1] + 0.0004, 0.0012),
                           (ROLLER[0] * math.sin(a), 0.0, ROLLER[0] * math.cos(a)), rot=(0.0, a, 0.0),
                           color=(0.95, 0.95, 0.95), collide=False))
    mark = tuple(marks)
    return Part("roller", pos=tuple(float(v) for v in c), size=(ROLLER[0], ROLLER[1], ROLLER[0]), shape="cylinder",
                axis="Y", exact_z=True, mass=0.025, color=RUBBER, friction=(1.2, 1.0), restitution=0.0,
                contact_offset=0.0015, pieces=mark,
                hinge=Hinge(pivot=tuple(float(v) for v in c), axis="Y", damping=0.00002, lower_deg=-1e6, upper_deg=1e6,
                            parent="gun", parent_pos=tuple(float(v) for v in g + r @ np.array(ROOT_LOCAL))))


def gun_points(pose):
    """World roller axis centre, blade edge, the gun's forward and up axes, from its root pose."""
    p = np.asarray(pose, float)
    r = quat_matrix(p[3:7])
    centre = p[:3] - r @ np.array(ROOT_LOCAL)
    roller = centre + r @ np.array([ROLLER_AT[0], 0.0, ROLLER_AT[1]])
    blade = centre + r @ np.array([BLADE_AT[0], 0.0, BLADE_AT[1]])
    return roller, blade, r[:, 0], r[:, 2]


def quat_matrix(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def to_carton(carton_pose, p):
    c = np.asarray(carton_pose, float)
    return quat_matrix(c[3:7]).T @ (np.asarray(p, float) - c[:3])


TOP_LOCAL = TOP_SURF - (TOP + WALL / 2)


def roller_contact(carton_pose, roller):
    """Where the roller is pressing the tape on: (s along the strip, offset across it) or None."""
    c = to_carton(carton_pose, roller)
    r = ROLLER[0]
    if abs(c[1]) > 0.06:
        return None
    if abs(c[2] - (TOP_LOCAL + r)) < 0.003 and abs(c[0]) <= END_X + 0.002:
        return float(c[0] + END_X), float(c[1])
    if abs(c[0] - (END_X + r)) < 0.003 and TOP_LOCAL - 0.06 < c[2] < TOP_LOCAL:
        return float(STRIP_L + TOP_LOCAL - c[2]), float(c[1])
    if abs(c[0] - (-END_X - r)) < 0.003 and TOP_LOCAL - 0.06 < c[2] < TOP_LOCAL:
        return float(-(TOP_LOCAL - c[2])), float(c[1])
    for sgn in (1, -1):                                   # rounding an end edge: the roller wraps it
        d = math.hypot(c[0] - sgn * END_X, c[2] - TOP_LOCAL)
        if abs(d - r) < 0.003 and sgn * (c[0] - sgn * END_X) > 0 and c[2] > TOP_LOCAL:
            return (STRIP_L if sgn > 0 else 0.0), float(c[1])
    return None


def blade_cut(carton_pose, roller, blade, fwd):
    """The tape torn on the blade: the gun run on past an end edge, the tape stretched from the edge to
    the roller, then snapped up so the stretched tape drags across the serrated blade. Returns where along
    the strip the torn tail ends (s), or None."""
    c = to_carton(carton_pose, roller)
    if abs(c[1]) > 0.06 or c[0] < END_X + 0.008 or c[2] < TOP_LOCAL + ROLLER[0] + CUT_LIFT:
        return None
    bl = to_carton(carton_pose, blade)
    return float(STRIP_L + min(MAX_TAIL, float(np.linalg.norm(bl - np.array([END_X, bl[1], TOP_LOCAL])))))


CUT_LIFT = 0.012                     # roller snapped up this far beyond the edge tears the tape
MAX_TAIL = 0.060


def hand_press(carton_pose, tip):
    """Where a fingertip is smoothing tape down on an end face (s), or None."""
    c = to_carton(carton_pose, tip)
    if abs(c[1]) > 0.04 or not (TOP_LOCAL - 0.05 < c[2] < TOP_LOCAL + 0.002):
        return None
    if END_X - 0.002 < c[0] < END_X + 0.015:
        return float(STRIP_L + TOP_LOCAL - c[2])
    if -END_X - 0.015 < c[0] < -END_X + 0.002:
        return float(-(TOP_LOCAL - c[2]))
    return None


def cell_of(s):
    return int(math.floor(s / CELL)) + TAB_CELLS
