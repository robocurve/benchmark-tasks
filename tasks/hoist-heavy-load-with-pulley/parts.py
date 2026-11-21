"""Scene parts for the ratchet-block kettlebell hoist. NumPy only at import time.

World frame (YAM station): x forward from the arm bases, y left, z up, metres.
Arm bases sit at (0, +-0.305, 0.75); the table top is TABLE_TOP["station"] = 0.7475 m.

Everything here is a real, buyable item at full size:
  * a goal-post frame of 40 x 40 mm aluminium T-slot extrusion clamped to the table, with a 22 cm jib;
  * a 75 mm (3 in) sailing ratchet block hung from the jib on a shackle;
  * 12 mm double-braid polyester rope tied to the handle of a 6 kg cast-iron kettlebell with a bowline;
  * a 10 mm rubber mat under the kettlebell;
  * a white 12 mm board between the frame's uprights with tape lines every 10 cm above the mat, the 40 cm one red.
"""

from __future__ import annotations

import math
import os

import numpy as np

import hoistkit
import kit
import meshes as m
from hoistkit import HoistAssembly, PulleyBlock
from kit import Body, BoxShape, CylShape, MeshShape, SphereShape
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- visual materials ------------------------------------------------------------------------------------------------

ALU = (0.74, 0.75, 0.77)
ALU_DARK = (0.45, 0.46, 0.48)
CLAMP_RED = (0.72, 0.12, 0.10)
CLAMP_STEEL = (0.30, 0.31, 0.33)
RUBBER = (0.10, 0.10, 0.11)
BOARD_WHITE = (0.93, 0.93, 0.91)
TAPE_BLACK = (0.06, 0.06, 0.07)
TAPE_RED = (0.85, 0.08, 0.06)
BLOCK_BLACK = (0.12, 0.12, 0.13)
SHEAVE_GREY = (0.52, 0.54, 0.57)
STEEL_BRIGHT = (0.70, 0.72, 0.75)
IRON = (0.17, 0.17, 0.18)
ROPE_BLUE = (0.16, 0.30, 0.62)
ROPE_FLECK = (0.90, 0.90, 0.88)

kit.FINISH.update(
    {
        ALU: (0.35, 0.85),
        ALU_DARK: (0.45, 0.8),
        CLAMP_RED: (0.45, 0.1),
        CLAMP_STEEL: (0.35, 0.9),
        RUBBER: (0.95, 0.0),
        BOARD_WHITE: (0.55, 0.0),
        TAPE_BLACK: (0.5, 0.0),
        TAPE_RED: (0.5, 0.0),
        BLOCK_BLACK: (0.45, 0.0),
        SHEAVE_GREY: (0.35, 0.8),
        STEEL_BRIGHT: (0.25, 1.0),
        IRON: (0.6, 0.6),
        ROPE_BLUE: (0.9, 0.0),
        ROPE_FLECK: (0.9, 0.0),
    }
)


def _np(mesh):
    p, f = mesh
    return tuple(map(tuple, np.round(p, 6).tolist())), tuple(map(tuple, f.tolist()))


# -- 1. Rubber mat under the kettlebell ------------------------------------------------------------------------------

MAT_T = 0.010
MAT_SIZE = (0.20, 0.20)
MAT_CENTRE = (0.680, 0.0)
MAT_TOP = TOP + MAT_T

MAT = Body.build(
    "mat",
    pos=(MAT_CENTRE[0], MAT_CENTRE[1], TOP),
    mass=1.0,
    kinematic=True,
    friction=(0.9, 0.8),
    shapes=[BoxShape((0.0, 0.0, MAT_T / 2), (MAT_SIZE[0], MAT_SIZE[1], MAT_T), RUBBER)],
)

# -- 2. Aluminium extrusion goal-post frame with jib, and the marked height board ------------------------------------

EXT = 0.040  # 40 x 40 mm extrusion
UPRIGHT_X = 0.800  # centre line of the uprights
UPRIGHT_Y = 0.220  # +- y of the uprights' centre lines
FRAME_H = 0.900  # top of the crossbar above the table
JIB_TIP_X = 0.570  # front end of the jib
AXLE_X = 0.600  # the block hangs from the jib here
FOOT_X = (0.620, 0.890)  # base rails along x on the table

BOARD_T = 0.012
BOARD_X = UPRIGHT_X - EXT / 2 + BOARD_T / 2 + 0.001  # front face flush with the uprights' front faces
BOARD_FRONT_X = BOARD_X - BOARD_T / 2
BOARD_W = 2 * UPRIGHT_Y - EXT  # fits between the uprights
BOARD_H = 0.780
LINE_STEP = 0.10
TARGET_H = 0.40  # the red line: the bottom of the kettlebell must clear it (m above the mat)
LINE_HEIGHTS = (0.10, 0.20, 0.30, TARGET_H)
TAPE_W = 0.012  # 12 mm tape; each line's LOWER edge sits at its height above the mat


def _frame_shapes():
    """In table-top coordinates centred under the crossbar (x = UPRIGHT_X, y = 0, z = TOP)."""
    h = EXT / 2
    shapes = []
    for s in (-1, 1):
        y = s * UPRIGHT_Y
        # base rail along x on the table
        shapes.append(BoxShape(((FOOT_X[0] + FOOT_X[1]) / 2 - UPRIGHT_X, y, h), (FOOT_X[1] - FOOT_X[0], EXT, EXT), ALU))
        # upright
        shapes.append(BoxShape((0.0, y, EXT + (FRAME_H - 2 * EXT) / 2), (EXT, EXT, FRAME_H - 2 * EXT), ALU))
        # rear diagonal brace (45 deg) from the upright down to the back of the base rail
        shapes.append(kit.segment((0.0 + h, y, 0.26), (FOOT_X[1] - UPRIGHT_X - 0.01, y, EXT + 0.01), 0.030, ALU_DARK, collide=False))
        # corner brackets (visual)
        shapes.append(BoxShape((h + 0.012, y, EXT + 0.012), (0.024, EXT, 0.024), ALU_DARK, collide=False))
        shapes.append(BoxShape((0.0, y, FRAME_H - EXT - 0.012), (EXT + 0.004, EXT + 0.004, 0.024), ALU_DARK, collide=False))
        # two C-clamps holding each base rail to the table (visual): a steel frame and a red handle
        for cx in (FOOT_X[0] + 0.05 - UPRIGHT_X, FOOT_X[1] - 0.05 - UPRIGHT_X):
            shapes.append(BoxShape((cx, y + s * 0.0, EXT + 0.008), (0.030, EXT + 0.010, 0.016), CLAMP_STEEL, collide=False))
            shapes.append(BoxShape((cx, y + s * (EXT / 2 + 0.010), EXT / 2), (0.030, 0.012, EXT + 0.04), CLAMP_STEEL, collide=False))
            shapes.append(CylShape((cx, y, EXT + 0.028), 0.006, 0.030, CLAMP_RED, collide=False, axis="Z"))
    # crossbar along y
    shapes.append(BoxShape((0.0, 0.0, FRAME_H - h), (EXT, 2 * UPRIGHT_Y + EXT, EXT), ALU))
    # jib along x, under the crossbar's front face height, running forward to JIB_TIP_X
    jib_len = UPRIGHT_X - h - JIB_TIP_X
    shapes.append(BoxShape((-(h + jib_len / 2), 0.0, FRAME_H - h), (jib_len, EXT, EXT), ALU))
    # jib gusset brace (visual) from the jib down to the crossbar's front face
    shapes.append(kit.segment((-h - 0.10, 0.0, FRAME_H - EXT), (-h, 0.0, FRAME_H - EXT - 0.10), 0.024, ALU_DARK, collide=False))
    # end caps on the jib and the crossbar (visual)
    shapes.append(BoxShape((JIB_TIP_X - UPRIGHT_X - 0.002, 0.0, FRAME_H - h), (0.004, EXT, EXT), (0.1, 0.1, 0.1), collide=False))
    # slot lines on the uprights' front faces (visual)
    for s in (-1, 1):
        shapes.append(BoxShape((-h - 0.0005, s * UPRIGHT_Y, FRAME_H / 2), (0.001, 0.008, FRAME_H - 2 * EXT), ALU_DARK, collide=False))
    # the white board between the uprights and its tape lines
    bx = BOARD_X - UPRIGHT_X
    shapes.append(BoxShape((bx, 0.0, BOARD_H / 2 + 0.0005), (BOARD_T, BOARD_W, BOARD_H), BOARD_WHITE))
    for hh in (0.0, *LINE_HEIGHTS):
        red = abs(hh - TARGET_H) < 1e-6
        z_low = MAT_T + hh
        shapes.append(
            BoxShape(
                (bx - BOARD_T / 2 - 0.0006, 0.0, z_low + TAPE_W / 2),
                (0.001, BOARD_W - 0.02, TAPE_W),
                TAPE_RED if red else TAPE_BLACK,
                collide=False,
            )
        )
    # 1 cm tick ladder up the board's centre line, so heights between lines can be read off a photo
    for k in range(1, int(TARGET_H * 100) + 6):
        if k % 10 == 0:
            continue
        w = 0.030 if k % 5 == 0 else 0.016
        shapes.append(BoxShape((bx - BOARD_T / 2 - 0.0006, -0.12, MAT_T + 0.01 * k + 0.001), (0.001, w, 0.002), TAPE_BLACK, collide=False))
    return shapes


FRAME = Body.build(
    "frame",
    pos=(UPRIGHT_X, 0.0, TOP),
    mass=20.0,
    kinematic=True,
    friction=(0.4, 0.35),
    shapes=_frame_shapes(),
)

# -- 3. Ratchet block hanging from the jib ---------------------------------------------------------------------------

AXLE_Z = TOP + 0.760
PULLEY_AXLE = (AXLE_X, 0.0, AXLE_Z)
ROPE_R = 0.006  # 12 mm rope
R_FLANGE = 0.0375  # 75 mm sheave
R_GROOVE = 0.0290  # groove floor
CHANNEL_W = 0.014  # between the flanges
FLANGE_T = 0.003
R_PITCH = R_GROOVE + ROPE_R + float(os.environ.get("HOIST_PITCH_GAP", "0.0025"))  # rope centre line over the sheave (0.0355 m)
CHEEK_R = 0.052
CHEEK_T = 0.004
CHEEK_Y = CHANNEL_W / 2 + FLANGE_T + 0.0025 + CHEEK_T / 2

LOAD_MASS = float(os.environ.get("HOIST_MASS", "4.0"))
# rope-on-sheave friction (the sheave's material combines by "min", so this is the rope-on-sheave value)
SHEAVE_MU = float(os.environ.get("HOIST_SHEAVE_MU", "1.5"))
# rope against the fingers: PhysX averages it with the finger pads' default 0.5, giving 0.75
ROPE_MU_S = float(os.environ.get("HOIST_ROPE_MU", "1.5"))
ROPE_MU_D = float(os.environ.get("HOIST_ROPE_MU_D", "1.4"))
BEARING_DAMPING = 0.002  # N m s/rad: a ball-bearing sheave turns freely
# the ratchet: while engaged, a stiff spring holds the sheave at the angle the pawl caught it
RATCHET_K = float(os.environ.get("HOIST_PAWL_K", "400.0"))  # N m/rad: the engaged pawl holds the sheave on a stiff spring
RATCHET_D = float(os.environ.get("HOIST_PAWL_D", "3.0"))  # N m s/rad
RATCHET_EFFORT = 60.0  # N m, far beyond anything the rope can put on it
RATCHET_ON = os.environ.get("HOIST_RATCHET", "1") != "0"


def _block_frame_shapes():
    """Cheeks, keeper pins, swivel head and shackle, in axle-centred coordinates."""
    shapes = []
    jib_bottom = TOP + FRAME_H - EXT - AXLE_Z  # 0.100 above the axle
    for s in (-1, 1):
        # black cheek plates either side of the sheave (colliders keep the rope in the channel)
        shapes.append(CylShape((0.0, s * CHEEK_Y, 0.0), CHEEK_R, CHEEK_T, BLOCK_BLACK, axis="Y"))
        # strap from the cheek up to the swivel head
        shapes.append(BoxShape((0.0, s * CHEEK_Y, 0.058), (0.026, CHEEK_T, 0.030), BLOCK_BLACK, collide=False))
    # keeper pins across the cheeks just outside the rope, so the rope can never jump the flanges
    for ang in (35.0, 90.0, 145.0):
        a = math.radians(ang)
        r = R_PITCH + ROPE_R + 0.0045
        shapes.append(CylShape((r * math.cos(a), 0.0, r * math.sin(a)), 0.0030, 2 * CHEEK_Y, STEEL_BRIGHT, axis="Y"))
    # axle bolt heads
    shapes.append(CylShape((0.0, 0.0, 0.0), 0.007, 2 * CHEEK_Y + 0.006, STEEL_BRIGHT, collide=False, axis="Y"))
    # ratchet on/off switch on one cheek (visual)
    shapes.append(BoxShape((0.018, CHEEK_Y + CHEEK_T / 2 + 0.003, 0.018), (0.016, 0.006, 0.008), (0.85, 0.85, 0.85), collide=False))
    # swivel head and shackle up to the jib (visual)
    shapes.append(BoxShape((0.0, 0.0, 0.072), (0.024, 2 * CHEEK_Y + CHEEK_T, 0.010), BLOCK_BLACK, collide=False))
    shapes.append(CylShape((0.0, 0.0, 0.080), 0.008, 0.012, STEEL_BRIGHT, collide=False, axis="Z"))
    shapes.append(kit.segment((0.0, -0.010, 0.084), (0.0, -0.010, jib_bottom - 0.002), 0.006, STEEL_BRIGHT, collide=False))
    shapes.append(kit.segment((0.0, 0.010, 0.084), (0.0, 0.010, jib_bottom - 0.002), 0.006, STEEL_BRIGHT, collide=False))
    shapes.append(BoxShape((0.0, 0.0, jib_bottom - 0.002), (0.012, 0.030, 0.004), STEEL_BRIGHT, collide=False))
    return tuple(shapes)


def _block_sheave_shapes():
    hw = CHANNEL_W / 2
    fy = hw + FLANGE_T / 2
    prof = [
        (0.008, -(hw + FLANGE_T)),
        (R_FLANGE, -(hw + FLANGE_T)),
        (R_FLANGE, -hw),
        (R_GROOVE, -hw + 0.003),
        (R_GROOVE, hw - 0.003),
        (R_FLANGE, hw),
        (R_FLANGE, hw + FLANGE_T),
        (0.008, hw + FLANGE_T),
    ]
    wheel_mesh = m.transform(m.lathe(prof, 48), m.rot_x(-math.pi / 2))
    shapes = [
        MeshShape(*_np(wheel_mesh), SHEAVE_GREY, collide=False, smooth=True),
        CylShape((0.0, 0.0, 0.0), R_GROOVE, CHANNEL_W, None, axis="Y"),
        CylShape((0.0, -fy, 0.0), R_FLANGE, FLANGE_T, None, axis="Y"),
        CylShape((0.0, +fy, 0.0), R_FLANGE, FLANGE_T, None, axis="Y"),
    ]
    # dark radial marks on the sheave faces so its rotation shows in the videos
    for s in (-1, 1):
        y_face = s * (hw + FLANGE_T + 0.0006)
        for k in range(3):
            a = k * 2 * math.pi / 3
            q = (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))
            shapes.append(
                BoxShape((0.020 * math.cos(a), y_face, 0.020 * math.sin(a)), (0.022, 0.0012, 0.005), BLOCK_BLACK,
                         collide=False, quat_xyzw=q)
            )
    return tuple(shapes)


BLOCK = PulleyBlock(
    name="ratchet_block",
    pos=PULLEY_AXLE,
    size=(2 * CHEEK_R, 2 * CHEEK_Y + CHEEK_T, 2 * CHEEK_R),
    axle_pos=PULLEY_AXLE,
    frame_shapes=_block_frame_shapes(),
    sheave_shapes=_block_sheave_shapes(),
    sheave_mass=0.12,
    sheave_friction=(SHEAVE_MU, SHEAVE_MU),
    frame_friction=(0.15, 0.12),
    bearing_friction=RATCHET_EFFORT,  # effort limit of the joint's drive (the ratchet spring), not a friction
    bearing_damping=BEARING_DAMPING,
    bearing_armature=2e-4,
)

# -- 4. Kettlebell + rope articulation -------------------------------------------------------------------------------

# A 6 kg cast-iron kettlebell: bell 148 mm across, 210 mm tall, 33 mm handle. Origin at the bell's sphere centre.
BELL_R = 0.074
BELL_C = 0.076  # sphere centre above the base
BASE_R = 0.052
BASE_T = 0.010
HANDLE_R = 0.0165
HANDLE_Z = 0.1945 - BELL_C  # handle bar centre above the origin
HANDLE_HALF_W = 0.052
BELL_HEIGHT = HANDLE_Z + BELL_C + HANDLE_R  # 0.211 m

REST_X = 0.660  # where the bell stands on the mat (2.4 cm behind plumb under the block, clear of the hauling fall)
LOAD_POS0 = (REST_X, 0.0, MAT_TOP + BELL_C + 0.0005)
EYE_Z_LOCAL = HANDLE_Z + HANDLE_R  # the bowline sits on top of the handle bar

# lowest points of the kettlebell in its own frame: the base ring and the bell's underside
_ring = [(BASE_R * math.cos(a), BASE_R * math.sin(a), -BELL_C) for a in np.linspace(0, 2 * math.pi, 16, endpoint=False)]
_belly = [
    (BELL_R * math.sin(t) * math.cos(a), BELL_R * math.sin(t) * math.sin(a), -BELL_R * math.cos(t))
    for t in (math.radians(55), math.radians(75))
    for a in np.linspace(0, 2 * math.pi, 12, endpoint=False)
]
BOTTOM_PTS = tuple(_ring + _belly)


def _bell_shapes():
    prof = [(0.0, -BELL_C)]
    prof += [(BASE_R, -BELL_C), (BASE_R + 0.002, -BELL_C + 0.003)]
    for t in np.linspace(math.radians(46), math.radians(160), 24):
        prof.append((BELL_R * math.sin(t), -BELL_R * math.cos(t)))
    prof.append((0.0, BELL_R * 0.94))
    bell = m.lathe(prof, 48)
    # handle: two horns up from the shoulders and a straight bar across the top (y axis)
    path = []
    for t in np.linspace(math.pi, 0.0, 28):
        yy = -HANDLE_HALF_W * math.cos(t) * 1.0
        zz = 0.040 + (HANDLE_Z - 0.040) * math.sin(t) ** 0.35
        path.append((0.0, yy * 1.05, zz))
    handle = m.tube(np.asarray(path), HANDLE_R, segments=18)
    shapes = [
        MeshShape(*_np(bell), IRON, collide=False, smooth=True),
        MeshShape(*_np(handle), IRON, collide=False, smooth=True),
        # colliders
        SphereShape((0.0, 0.0, 0.0), BELL_R, None),
        CylShape((0.0, 0.0, -BELL_C + BASE_T / 2), BASE_R, BASE_T, None, axis="Z"),
        CylShape((0.0, 0.0, HANDLE_Z), HANDLE_R, 2 * HANDLE_HALF_W, None, axis="Y"),
        kit.segment((0.0, -HANDLE_HALF_W, 0.04), (0.0, -HANDLE_HALF_W, HANDLE_Z), 2 * HANDLE_R * 0.9, None),
        kit.segment((0.0, HANDLE_HALF_W, 0.04), (0.0, HANDLE_HALF_W, HANDLE_Z), 2 * HANDLE_R * 0.9, None),
        # the bowline's knot on the handle (visual)
        SphereShape((0.0, 0.0, HANDLE_Z + HANDLE_R + 0.004), 0.011, ROPE_BLUE, collide=False),
        CylShape((0.0, 0.0, HANDLE_Z), HANDLE_R + 0.0045, 0.020, ROPE_BLUE, collide=False, axis="Y"),
    ]
    return tuple(shapes)


# Hauling line once the rope is drawn taut: a straight 60-deg line from the sheave's front tangent down toward the arms
HAUL_SLOPE_RAD = math.radians(60.0)
_TAN_ANG = math.pi / 2 + HAUL_SLOPE_RAD  # 150 deg from +x in the x-z plane
P_TANGENT = np.array([AXLE_X + R_PITCH * math.cos(_TAN_ANG), 0.0, AXLE_Z + R_PITCH * math.sin(_TAN_ANG)])
U_ROPE_UP = np.array([math.cos(HAUL_SLOPE_RAD), 0.0, math.sin(HAUL_SLOPE_RAD)])  # up the rope toward the block
U_PULL = -U_ROPE_UP


def haul_point(d: float) -> np.ndarray:
    """Point on the taut hauling line ``d`` metres down the rope from where it leaves the sheave."""
    return P_TANGENT - float(d) * U_ROPE_UP


SEG_LEN = float(os.environ.get("HOIST_SEG", "0.022"))
ROPE_END_X = 0.200  # the free end lies on the table this far in front of the arm bases
_GROUND_ROPE_Z = TOP + ROPE_R + 0.0005
_BEND_R = 0.030


def _build_rope_nodes(seg_len: float = SEG_LEN) -> np.ndarray:
    """Rest layout: bowline on the handle -> up to the sheave -> 180 deg over it -> straight down its front ->
    a bend onto the table -> straight along the table (y = 0) toward the arms."""
    pts = []
    eye = np.array(LOAD_POS0) + np.array([0.0, 0.0, EYE_Z_LOCAL])
    top_back = np.array([AXLE_X + R_PITCH, 0.0, AXLE_Z])
    for t in np.linspace(0.0, 1.0, 40, endpoint=False):
        pts.append(eye + t * (top_back - eye))
    for ang in np.linspace(0.0, math.pi, 60, endpoint=False):
        pts.append([AXLE_X + R_PITCH * math.cos(ang), 0.0, AXLE_Z + R_PITCH * math.sin(ang)])
    x_drop = AXLE_X - R_PITCH
    z_bend = _GROUND_ROPE_Z + _BEND_R
    for z in np.linspace(AXLE_Z, z_bend, 50, endpoint=False):
        pts.append([x_drop, 0.0, z])
    bc_x = x_drop - _BEND_R
    for ang in np.linspace(0.0, -math.pi / 2, 16, endpoint=False):
        pts.append([bc_x + _BEND_R * math.cos(ang), 0.0, z_bend + _BEND_R * math.sin(ang)])
    for x in np.linspace(bc_x, ROPE_END_X, 40):
        pts.append([x, 0.0, _GROUND_ROPE_Z])
    pts = np.asarray(pts, float)
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    n = max(10, int(round(s[-1] / seg_len)))
    s_u = np.linspace(0.0, s[-1], n + 1)
    return np.column_stack([np.interp(s_u, s, pts[:, k]) for k in range(3)])


ROPE_NODES = _build_rope_nodes()
ROPE_LENS = np.linalg.norm(np.diff(ROPE_NODES, axis=0), axis=1)
ROPE_LENGTH = float(ROPE_LENS.sum())

HOIST = HoistAssembly(
    name="hoist",
    pos=LOAD_POS0,
    size=(2 * BELL_R, 2 * BELL_R, BELL_HEIGHT),
    color=IRON,
    load_pos=LOAD_POS0,
    load_half_size=(BELL_R, BELL_R, BELL_HEIGHT / 2),
    load_mass=LOAD_MASS,
    load_shapes=_bell_shapes(),
    load_friction=(0.7, 0.6),
    load_bottom_pts=BOTTOM_PTS,
    rope_nodes=tuple(map(tuple, np.round(ROPE_NODES, 6).tolist())),
    rope_radius=ROPE_R,
    rope_seg_mass=float(os.environ.get("HOIST_SEG_MASS", "0.02")),
    rope_friction=(ROPE_MU_S, ROPE_MU_D),
    rope_damping=0.006,
    rope_armature=3e-4,
    rope_max_bend_deg=80.0,
    collision_sides=8,
    capsule_links=os.environ.get("HOIST_ROPE_SHAPE", "capsule") == "capsule",
)

# -- 5. Holder for the rope's rendered skin --------------------------------------------------------------------------

FX = Body.build(
    "fx",
    pos=(0.0, 0.0, 0.0),
    mass=1.0,
    kinematic=True,
    shapes=[BoxShape((0.0, 0.0, -0.5), (0.01, 0.01, 0.01), None, collide=False)],
)


def _rope_start_skin():
    import ropeskin

    pos = (ROPE_NODES[:-1] + ROPE_NODES[1:]) / 2.0
    rots = np.stack([hoistkit._seg_frame(ROPE_NODES[i + 1] - ROPE_NODES[i]) for i in range(len(ROPE_LENS))])
    pts, sds, tan = ropeskin.centre_line(pos, rots, ROPE_LENS, start=ROPE_NODES[0])
    return ropeskin.skin(pts, sds, tan, ROPE_R)


kit.dynamic_mesh("rope_core", "fx", ROPE_BLUE, initial=lambda: _rope_start_skin()[0])
kit.dynamic_mesh("rope_strand", "fx", ROPE_FLECK, initial=lambda: _rope_start_skin()[1])

OBJECTS = [MAT, FRAME, BLOCK, HOIST, FX]
if os.environ.get("HOIST_DEBUG") == "noblock":
    OBJECTS = [MAT, FRAME, HOIST, FX]
if os.environ.get("HOIST_DEBUG") == "nohoist":
    OBJECTS = [MAT, FRAME, BLOCK, FX]
