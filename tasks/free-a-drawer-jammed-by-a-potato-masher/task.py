"""On the bench in front of the arms stands the top-drawer section of a 15 in kitchen base cabinet: its sides, back,
bottom, a top panel in place of the countertop, and a face frame with a top rail over the drawer opening. The drawer
(a 12 in deep box on full-extension side-mount slides, a bar pull on its front) is shut. Inside it, at the back under
the counter, lies an OXO Good Grips Nylon Potato Masher, handle toward the front. It lies on its side with its fin
head standing on one end, so it is 102 mm tall; the top rail's underside is only 85 mm above the drawer's floor.

Pulled, the drawer runs out about 21.5 cm and stops hard: the masher's forked neck catches the back of the top rail and
the masher is pinned between the rail and the drawer's back wall. Pulling harder only levers the masher up against
the rail. The way to open it is to reach into the open drawer, take the masher by its grip and roll it over, so the
head lies on its fins (66 mm tall, clear of the rail), then pull the drawer out to its stop.

The masher is a free rigid body (real contact with the drawer, the rail and the hands; no joint, capture or scripted
jam); the drawer is a link on a prismatic joint whose travel is the slide's; the cabinet is fixed.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import gripkit  # noqa: E402
import yamgeom as yg  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 2400
INSTRUCTION = "Open the kitchen drawer all the way, leaving everything in it."

# -- the cabinet section (its own frame: origin on the face frame's front at the middle of its width, on the bench top;
#    x into the cabinet, z up) ----------------------------------------------------------------------------------------
# A 15 in face-frame base cabinet's top-drawer section (estimated from maker photos and the usual 3/4 in face frame,
# 1-1/2 in stiles, 1/2 in side-mount slide clearance): 381 mm wide, 335 mm deep, a drawer opening 305 x 95 mm under a
# 50 mm top rail. It stands on the bench (the real one is under a counter at 36 in).
XF = 0.375  # world x of the face frame's front
CAB_Y = 0.0
CAB_W, CAB_D, CAB_H = 0.381, 0.335, 0.189
FF_T, STILE_W, SHEET = 0.019, 0.038, 0.019  # face frame thickness, stile width, side/bottom/top panel thickness
BACK_T = 0.006
OPEN_Z0, RAIL_Z0, RAIL_Z1 = 0.025, 0.120, 0.170  # the opening's bottom, the top rail's underside and top
OPEN_HALF_W = CAB_W / 2 - STILE_W  # 0.1525
# the drawer box (its parts in the cabinet frame with the drawer shut): 12 mm birch-ply sides, front and back, a 6 mm
# bottom; 280 mm wide (305 mm opening less the two 12.7 mm slides), 304 mm (12 in) deep, 76 mm tall
BOX_W, BOX_D, WALL = 0.280, 0.304, 0.012
BOX_Z0, FLOOR_T = 0.029, 0.006
FLOOR = BOX_Z0 + FLOOR_T  # the drawer floor's top (0.035 above the bench)
SIDE_TOP = 0.105  # the box's sides, front and back: 70 mm above its floor, 15 mm under the top rail
IN_X0, IN_X1 = WALL, BOX_D - WALL  # the inside, front to back (0.012 .. 0.292)
IN_HALF_W = BOX_W / 2 - WALL  # 0.128
FRONT_T, FRONT_HALF_W, FRONT_Z0, FRONT_Z1 = 0.019, 0.180, 0.022, 0.150  # the overlay drawer front
PULL_Z = 0.060  # the bar pull's height (below the front's middle, clear of the counter for a hand)
PULL_R, PULL_OUT, PULL_HALF = 0.006, 0.029, 0.064  # a 12 mm bar, its axis 29 mm proud of the front, 128 mm centres
TRAVEL = 0.300  # full-extension 12 in slides: the drawer's stop, 300 mm out
DRAWER_KG = 1.7  # the box in birch ply (650 kg/m^3) and the front (estimated)
SLIDE_DRAG = 20.0  # N s/m: the ball-bearing slides' rolling resistance, so a nudged drawer stops instead of coasting
# (assumed: 0.1 N at 5 mm/s, 0.4 N at 2 cm/s), on top of the joint's 3 N s/m damping
WOOD = (0.40, 0.30, "average")  # wood on wood, clean and dry (Engineering ToolBox, friction coefficients: 0.25-0.5)
CAB_LOOK = kit.Look((0.93, 0.92, 0.88), 0.6)  # painted cabinet
PLY_LOOK = kit.Look((0.86, 0.74, 0.55), 0.7)  # the drawer box's birch ply
STEEL_LOOK = kit.Look((0.80, 0.81, 0.83), 0.25, metallic=0.8)  # stainless
GRIP_LOOK = kit.Look((0.08, 0.08, 0.08), 0.7)  # the masher's black nylon


def _bx(lo, hi, look, collide=True):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c, s = tuple(float(v) for v in (lo + hi) / 2), tuple(float(v) for v in hi - lo)
    return kit.box(c, s, look), (kit.Collider(c, size=s) if collide else None)


_W2 = CAB_W / 2
_CAB_PARTS = [
    _bx((FF_T, -_W2, 0.0), (CAB_D, _W2, SHEET), CAB_LOOK),  # bottom panel
    _bx((0.0, -_W2, RAIL_Z1), (CAB_D, _W2, CAB_H), CAB_LOOK),  # top panel (the countertop's place)
    _bx((FF_T, -_W2, SHEET), (CAB_D, -_W2 + SHEET, RAIL_Z1), CAB_LOOK),  # sides
    _bx((FF_T, _W2 - SHEET, SHEET), (CAB_D, _W2, RAIL_Z1), CAB_LOOK),
    _bx((CAB_D - BACK_T, -_W2 + SHEET, SHEET), (CAB_D, _W2 - SHEET, RAIL_Z1), CAB_LOOK),  # back
    # the face frame: top rail, bottom rail, two stiles
    _bx((0.0, -OPEN_HALF_W, RAIL_Z0), (FF_T, OPEN_HALF_W, RAIL_Z1), CAB_LOOK),
    _bx((0.0, -OPEN_HALF_W, 0.0), (FF_T, OPEN_HALF_W, OPEN_Z0), CAB_LOOK),
    _bx((0.0, -_W2, 0.0), (FF_T, -OPEN_HALF_W, RAIL_Z1), CAB_LOOK),
    _bx((0.0, OPEN_HALF_W, 0.0), (FF_T, _W2, RAIL_Z1), CAB_LOOK),
]
_BH = BOX_W / 2
_DRAWER_PARTS = [
    _bx((0.0, -_BH, BOX_Z0), (BOX_D, _BH, FLOOR), PLY_LOOK),  # bottom
    _bx((0.0, -_BH, FLOOR), (BOX_D, -_BH + WALL, SIDE_TOP), PLY_LOOK),  # sides
    _bx((0.0, _BH - WALL, FLOOR), (BOX_D, _BH, SIDE_TOP), PLY_LOOK),
    _bx((0.0, -_BH + WALL, FLOOR), (WALL, _BH - WALL, SIDE_TOP), PLY_LOOK),  # front wall
    _bx((BOX_D - WALL, -_BH + WALL, FLOOR), (BOX_D, _BH - WALL, SIDE_TOP), PLY_LOOK),  # back wall
    _bx((-FRONT_T, -FRONT_HALF_W, FRONT_Z0), (0.0, FRONT_HALF_W, FRONT_Z1), CAB_LOOK),  # overlay front
]
_PULL_X = -FRONT_T - PULL_OUT  # the bar's axis (cabinet frame, drawer shut)
_PULL = [
    (kit.cylinder((_PULL_X, 0.0, PULL_Z), PULL_R, 2 * PULL_HALF + 0.012, STEEL_LOOK, axis="y", n=20),
     kit.Collider((_PULL_X, 0.0, PULL_Z), radius=PULL_R, height=2 * PULL_HALF + 0.012, axis="Y")),
    *((kit.cylinder(((_PULL_X - FRONT_T) / 2, s * PULL_HALF, PULL_Z), 0.005, PULL_OUT, STEEL_LOOK, axis="x", n=16),
       kit.Collider(((_PULL_X - FRONT_T) / 2, s * PULL_HALF, PULL_Z), radius=0.005, height=PULL_OUT, axis="X"))
      for s in (-1, 1)),
]
DRAWER = kit.Link(
    "drawer", "prismatic", "X", anchor=(0.0, 0.0, 0.0), lower=-TRAVEL, upper=0.0, start=0.0, mass=DRAWER_KG,
    damping=3.0, drag=SLIDE_DRAG, meshes=tuple(m for m, _ in _DRAWER_PARTS + _PULL),
    colliders=tuple(c for _, c in _DRAWER_PARTS + _PULL))
CABINET = kit.Mechanism(
    "cabinet", pos=(XF, CAB_Y, TOP), size=(CAB_D + FF_T + PULL_OUT, CAB_W, CAB_H),
    meshes=tuple(m for m, _ in _CAB_PARTS), colliders=tuple(c for _, c in _CAB_PARTS), links=(DRAWER,),
    material=WOOD)

# -- the masher (its own frame: origin at the head's middle; x along the handle toward its end, y along the head's long
#    side (the plane of its forked neck), z along its short side (across that plane)) ------------------------------------
# OXO Good Grips Nylon Potato Masher (Target listing, TCIN 80221526, UPC 719812862514): 10.2 in (259 mm) long, 3.9 in
# (99 mm) wide, 0.3 lb (136 g), all nylon, with a soft-grip handle in line with the neck. Its head is a row of seven
# nylon fins across the end of a forked (Y-shaped) neck, 3.9 in along the row. The fins' depth across the fork (64 mm),
# the head's length along the handle, the fork and the handle's thickness are estimated from maker photos. The fin row is modeled as one solid block, the fork's two arms as straight bars.
HEAD_T, HEAD_L, HEAD_S = 0.025, 0.099, 0.064  # the head: along the handle, long side (the listing's width), short side
ARM_A, ARM_B = (0.008, 0.0455), (0.105, 0.005)  # each fork arm, from the head to the neck (x, |y|)
ARM_W, ARM_D = 0.008, 0.012  # the arms' width in the fork's plane and their depth across it
NECK_R, NECK_X0, NECK_X1 = 0.009, 0.100, 0.145  # the neck, where the arms meet, to the handle
GRIP_R, GRIP_X1 = 0.014, 0.2465  # the soft grip (28 mm thick), from the neck to the masher's end (10.2 in overall)
HEAD_G, ARM_G, NECK_G, GRIP_G = 40.0, 22.0, 6.0, 68.0  # 136 g in all (split estimated from the parts' volumes)
MASHER_KG = (HEAD_G + ARM_G + NECK_G + GRIP_G) / 1000
NYLON = (0.35, 0.30, "average")  # nylon (Engineering ToolBox, friction coefficients: no nylon-on-wood row; nylon on
# nylon 0.15-0.25, on steel 0.4). PhysX averages the two surfaces: nylon on the drawer's wood 0.375 / 0.30
TPE = (0.80, 0.60, "average")  # the soft grip, rubbery (Engineering ToolBox: rubber on dry concrete 0.6-0.85)


def _arm_corners(sy: float) -> np.ndarray:
    """The 8 corners of one fork arm (sy = +1 or -1), masher frame."""
    a, b = np.array([ARM_A[0], sy * ARM_A[1]]), np.array([ARM_B[0], sy * ARM_B[1]])
    u = (b - a) / np.linalg.norm(b - a)
    n = np.array([-u[1], u[0]]) * ARM_W / 2
    return np.array([(p[0] + s * n[0], p[1] + s * n[1], t * ARM_D / 2)
                     for p in (a, b) for s in (-1, 1) for t in (-1, 1)])


def _box_tris(c: np.ndarray) -> tuple:
    """Outward-wound triangles of a hexahedron whose 8 corners are indexed by 3 bits (one per pair of opposite
    faces)."""
    mid, out = c.mean(axis=0), []
    for k in range(3):
        for v in (0, 1):
            ring = []
            for a, b in ((0, 0), (0, 1), (1, 1), (1, 0)):
                bits, j = [0, 0, 0], 0
                for m in range(3):
                    if m == k:
                        bits[m] = v
                    else:
                        bits[m] = (a, b)[j]
                        j += 1
                ring.append(bits[0] * 4 + bits[1] * 2 + bits[2])
            n = np.cross(c[ring[1]] - c[ring[0]], c[ring[2]] - c[ring[0]])
            if float(n @ (c[ring].mean(axis=0) - mid)) < 0:
                ring = ring[::-1]
            out += [(ring[0], ring[1], ring[2]), (ring[0], ring[2], ring[3])]
    return tuple(out)


def _arm(sy: float):
    c = _arm_corners(sy)
    mid = c.mean(axis=0)
    rel = tuple(tuple(float(v) for v in p - mid) for p in c)
    a, b = np.array([ARM_A[0], sy * ARM_A[1], 0.0]), np.array([ARM_B[0], sy * ARM_B[1], 0.0])
    look = kit.band([a, b], ARM_D, ARM_W, GRIP_LOOK, normal=(0, 0, 1))
    return look, kit.Collider(tuple(float(v) for v in mid), hull=rel, faces=_box_tris(c))


def _masher_mass():
    """(centre of mass, principal moments) from the parts, each filled with points of equal mass (the masher is
    symmetric about its two mid-planes, so its inertia's axes are the frame's)."""
    g = np.linspace(-0.5, 0.5, 9)
    cube = np.array([(a, b, c) for a in g for b in g for c in g])
    parts = [(HEAD_G, cube * np.array([HEAD_T, HEAD_L, HEAD_S]))]
    for sy in (-1, 1):
        c = _arm_corners(sy)
        w = np.array([(a + 0.5, b + 0.5, t + 0.5) for a, b, t in cube])
        p = (c[0] * (1 - w[:, :1]) + c[4] * w[:, :1]) + (c[2] - c[0]) * w[:, 1:2] + (c[1] - c[0]) * w[:, 2:3]
        parts.append((ARM_G / 2, p))
    for gm, r, x0, x1 in ((NECK_G, NECK_R, NECK_X0, NECK_X1), (GRIP_G, GRIP_R, NECK_X1, GRIP_X1)):
        q = cube[np.hypot(cube[:, 1], cube[:, 2]) <= 0.5]
        parts.append((gm, np.column_stack([x0 + (q[:, 0] + 0.5) * (x1 - x0), q[:, 1] * 2 * r, q[:, 2] * 2 * r])))
    pts = np.vstack([p for _, p in parts])
    m = np.concatenate([np.full(len(p), gm / 1000 / len(p)) for gm, p in parts])
    com = (m[:, None] * pts).sum(axis=0) / m.sum()
    d = pts - com
    inertia = tuple(float((m * (d[:, (k + 1) % 3] ** 2 + d[:, (k + 2) % 3] ** 2)).sum()) for k in range(3))
    return (float(com[0]), 0.0, 0.0), inertia


MASHER_COM, MASHER_INERTIA = _masher_mass()

# Its start: lying on its side, the head standing on one of its short ends (its long side upright) and the handle's end
# on the floor, so the handle slopes down from the head by BETA; the head stands 1 mm in front of the drawer's back wall.
BETA = math.atan2(HEAD_L / 2 - GRIP_R, GRIP_X1 - HEAD_T / 2)  # 8.6 degrees
MASHER_Y = 0.040  # the masher's line, left of the drawer's middle (cabinet frame)
SPAWN_GAP = 0.0005


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


# masher axes in the world: x (to the handle's end) toward the robot, y (the head's long side) up, z = x cross y
_R_LIE = np.column_stack([[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]])
MASHER_R0 = _ry(-BETA) @ _R_LIE
_H0 = (HEAD_L / 2) * math.cos(BETA) + (HEAD_T / 2) * math.sin(BETA)  # the head's middle above the floor
_X0 = IN_X1 - 0.001 - ((HEAD_T / 2) * math.cos(BETA) + (HEAD_L / 2) * math.sin(BETA))
MASHER_START_C = np.array([_X0, MASHER_Y, FLOOR + _H0])  # cabinet frame, drawer shut


def quat(R) -> tuple:
    R = np.asarray(R, float)
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return (x, y, z, w)


def rotm(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


MASHER_START = tuple(float(v) for v in np.array([XF, CAB_Y, TOP]) + MASHER_START_C + np.array([0, 0, SPAWN_GAP]))
_NECK_C, _GRIP_C = ((NECK_X0 + NECK_X1) / 2, 0.0, 0.0), ((NECK_X1 + GRIP_X1) / 2, 0.0, 0.0)
_ARMS = [_arm(-1.0), _arm(1.0)]
MASHER = kit.Prop(
    "masher", pos=MASHER_START, mass=MASHER_KG, com=MASHER_COM, inertia=MASHER_INERTIA,
    tilt_xyzw=quat(MASHER_R0), size=(GRIP_X1 + HEAD_T / 2, HEAD_L, HEAD_S),
    meshes=(kit.box((0, 0, 0), (HEAD_T, HEAD_L, HEAD_S), GRIP_LOOK), *(m for m, _ in _ARMS),
            kit.cylinder(_NECK_C, NECK_R, NECK_X1 - NECK_X0, GRIP_LOOK, axis="x", n=16),
            kit.cylinder(_GRIP_C, GRIP_R, GRIP_X1 - NECK_X1, GRIP_LOOK, axis="x", n=28)),
    colliders=(kit.Collider((0, 0, 0), size=(HEAD_T, HEAD_L, HEAD_S)), *(c for _, c in _ARMS),
               kit.Collider(_NECK_C, radius=NECK_R, height=NECK_X1 - NECK_X0, axis="X"),
               kit.Collider(_GRIP_C, radius=GRIP_R, height=GRIP_X1 - NECK_X1, axis="X", material=TPE)),
    material=NYLON, contact_offset=0.002, solver_iterations=(32, 8))

OBJECTS = [CABINET, MASHER]
gripkit.use_rubber_pads()  # the YAM's finger pads are rubber-faced

# -- scored thresholds --------------------------------------------------------------------------------------------------
FLAT_TOP = RAIL_Z0 - FLOOR - 0.002  # flat: the head's highest point at least 2 mm under the rail's underside (83 mm
# over the drawer's floor; lying on its fins it is 65.7 mm tall, standing on end 102; tilted 7 degrees off flat it is
# 77, 10 degrees 82, 13 degrees 86: the limit is crossed at about 10.6 degrees)
PITCH_FORCED_DEG = 30.0  # forced: the handle levered more than 30 degrees off level (it starts at 8.6) ...
PITCH_FROM = 0.205  # m: ... with the drawer out this far or more (at the snag, 0.215, where a yank levers it up)
RAIL_SLACK = 0.002  # m: a point over the rail's underside counts as under or past it 2 mm in front of its back
LIFTED = 0.040  # removed: the masher's lowest point more than 40 mm over the drawer's floor
IN_TOL = 0.003  # m: the masher's outline within the drawer's inside, give or take 3 mm
OPEN_PAST = 0.280  # m: open past the snag (standing on end, the head stops the drawer by 0.26)
OPEN_FULL = 0.295  # m: fully open (the stop is at 0.300)
CLEAR = 0.005  # m: hands clear: every arm link at least 5 mm from the masher and from the drawer's front and pull
STILL_STEP = 0.0005  # m per control step (1 cm/s), the masher and the drawer
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)
ARM_LINKS = ("link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
ROBOTS = ("left_robot", "right_robot")
LINK_POSES: dict = {}  # "right_robot/gripper" -> (pos, R) in the env frame at this control step (filled in the sim)


# -- reading the drawer and the masher -----------------------------------------------------------------------------------
def opening(objects) -> float:
    """How far the drawer is out (m, 0 shut)."""
    return -float(kit.joint(objects, "cabinet/drawer").pos)


def drawer_origin(objects) -> np.ndarray:
    """World position of the drawer's frame (the cabinet frame moved out with the drawer)."""
    return np.asarray(objects["cabinet"].pos, float) - np.array([opening(objects), 0.0, 0.0])


def masher_pose(objects):
    m = objects["masher"]
    return np.asarray(m.pos, float), rotm(m.quat_xyzw)


def _outline_local() -> tuple:
    """Masher-frame points on its outline: the head's edges and the fork arms' edges (the part behind the rail at the
    start), then the grip's end rings; and how many of them are the head's and arms'."""
    t = np.linspace(-0.5, 0.5, 9)
    pts = []
    for k in range(3):  # the head box's 12 edges
        for s1 in (-0.5, 0.5):
            for s2 in (-0.5, 0.5):
                e = np.zeros((len(t), 3))
                e[:, k] = t
                e[:, (k + 1) % 3], e[:, (k + 2) % 3] = s1, s2
                pts.append(e * np.array([HEAD_T, HEAD_L, HEAD_S]))
    ta = np.linspace(0.0, 1.0, 41)[:, None]  # every 2.6 mm along the arms (they rise toward the head)
    for sy in (-1, 1):  # the arms' 4 long edges
        c = _arm_corners(sy)
        for i in range(4):
            pts.append(c[i] + ta * (c[i + 4] - c[i]))
    n_head = sum(len(p) for p in pts)
    a = np.linspace(0, 2 * math.pi, 12, endpoint=False)
    for x in (NECK_X1, GRIP_X1):
        pts.append(np.column_stack([np.full(len(a), x), GRIP_R * np.cos(a), GRIP_R * np.sin(a)]))
    return np.vstack(pts), n_head


OUTLINE, N_HEAD = _outline_local()
# the head's and fork's outline points that stand higher than the rail's underside at the start: the part the rail
# catches, which gets under or in front of the rail only by being forced through it or after the masher is turned
CAUGHT = (np.arange(len(OUTLINE)) < N_HEAD) & ((MASHER_START_C + OUTLINE @ MASHER_R0.T)[:, 2] > RAIL_Z0)


def outline_points(objects) -> np.ndarray:
    """World points on the masher's outline."""
    p, R = masher_pose(objects)
    return p + OUTLINE @ R.T


def head_top(objects) -> float:
    """Height (m) of the masher's highest outline point over the drawer's floor."""
    return float(np.max((outline_points(objects) - drawer_origin(objects))[:, 2])) - FLOOR


def over_rail(objects) -> bool:
    """A point of the head or fork that the rail catches (CAUGHT) higher than the rail's underside while under the rail
    or in front of its back (it got there only by being forced through), or any other point (the fork's front, the
    grip) higher than it while under the rail itself (in front of the cabinet they are free to be lifted). RAIL_SLACK
    of give for contact."""
    q = outline_points(objects) - (np.asarray(objects["cabinet"].pos, float))
    in_front = q[:, 0] < FF_T - RAIL_SLACK
    under = in_front & (q[:, 0] >= -RAIL_SLACK)
    return bool(np.any((q[:, 2] > RAIL_Z0 + 0.001) & np.where(CAUGHT, in_front, under)))


def pitch_deg(objects) -> float:
    """Angle (degrees) of the handle from level."""
    _, R = masher_pose(objects)
    return math.degrees(math.asin(min(1.0, abs(float(R[2, 0])))))


def in_drawer(objects) -> bool:
    """The masher's whole outline inside the drawer box: between its front and back walls and its sides, none of it
    under the floor (give or take IN_TOL). (Standing, its head rises 29 mm over the sides; lifted off the floor is
    lowest_over_floor's.)"""
    q = outline_points(objects) - drawer_origin(objects)
    return bool(np.all(q[:, 0] >= IN_X0 - IN_TOL) and np.all(q[:, 0] <= IN_X1 + IN_TOL)
                and np.all(np.abs(q[:, 1] - CAB_Y) <= IN_HALF_W + IN_TOL) and np.all(q[:, 2] >= FLOOR - IN_TOL))


def lowest_over_floor(objects) -> float:
    return float(np.min((outline_points(objects) - drawer_origin(objects))[:, 2])) - FLOOR


def flat(objects) -> bool:
    return head_top(objects) <= FLAT_TOP


# -- the outlines the arms are checked against ----------------------------------------------------------------------------
def _seg_gap(q, a, b, r):
    """Distance (m, 0 inside) from points q to a flat-ended cylinder from a to b of radius r."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    L = float(np.linalg.norm(b - a))
    u = (b - a) / L
    d = q - a
    s = d @ u
    radial = np.linalg.norm(d - s[:, None] * u, axis=1)
    return np.hypot(np.maximum(radial - r, 0.0), np.maximum(np.maximum(-s, s - L), 0.0))


def _box_gap(q, lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    d = np.maximum(np.maximum(lo - q, q - hi), 0.0)
    return np.linalg.norm(d, axis=1)


def masher_gap(qm):
    """Distance from masher-frame points to the masher (head, fork arms, neck, grip)."""
    g = _box_gap(qm, (-HEAD_T / 2, -HEAD_L / 2, -HEAD_S / 2), (HEAD_T / 2, HEAD_L / 2, HEAD_S / 2))
    for sy in (-1, 1):
        r = math.hypot(ARM_W, ARM_D) / 2
        g = np.minimum(g, _seg_gap(qm, (ARM_A[0], sy * ARM_A[1], 0), (ARM_B[0], sy * ARM_B[1], 0), r))
    g = np.minimum(g, _seg_gap(qm, (NECK_X0, 0, 0), (NECK_X1, 0, 0), NECK_R))
    return np.minimum(g, _seg_gap(qm, (NECK_X1, 0, 0), (GRIP_X1, 0, 0), GRIP_R))


def front_gap(qd):
    """Distance from drawer-frame points to the drawer's front and its pull."""
    g = _box_gap(qd, (-FRONT_T, -FRONT_HALF_W, FRONT_Z0), (0.0, FRONT_HALF_W, FRONT_Z1))
    g = np.minimum(g, _seg_gap(qd, (_PULL_X, -PULL_HALF - 0.006, PULL_Z), (_PULL_X, PULL_HALF + 0.006, PULL_Z),
                               PULL_R))
    for s in (-1, 1):
        g = np.minimum(g, _seg_gap(qd, (_PULL_X, s * PULL_HALF, PULL_Z), (-FRONT_T, s * PULL_HALF, PULL_Z), 0.005))
    return g


def _parts(objects):
    p, R = masher_pose(objects)
    d = drawer_origin(objects) + np.array([0.0, 0.0, 0.0])
    return {"masher": (p, R, masher_gap, GRIP_X1 + 0.06),
            "front": (d + np.array([-0.03, 0.0, PULL_Z]), np.eye(3),
                      lambda q: front_gap(q + np.array([-0.03, 0.0, PULL_Z])), 0.20)}


_GAP_MEMO: dict = {}


def link_gap(objects, key, part) -> float:
    """Smallest gap (m) between arm link `key` ("right_robot/tip_left") as posed and `part` ("masher" or "front");
    0.02 for a link more than 2 cm away."""
    o, R = LINK_POSES[key]
    co, Rc, gap, reach = _parts(objects)[part]
    memo = (key, part, np.asarray(o).tobytes(), np.asarray(R).tobytes(), co.tobytes(), Rc.tobytes())
    if memo in _GAP_MEMO:
        return _GAP_MEMO[memo]
    link = key.split("/")[1]
    _, cen, rad = yg.cloud(link)
    o_, R_ = np.asarray(o, float), np.asarray(R, float)
    centre = (o_ + R_ @ cen - co) @ Rc
    if float(np.linalg.norm(centre)) - rad > reach + 0.02:
        out = 0.02
    else:
        q = yg.near(link, lambda x: (o_ + x @ R_.T - co) @ Rc, gap, 0.02)
        out = min(0.02, float(gap(q).min())) if len(q) else 0.02
    if len(_GAP_MEMO) > 1024:
        _GAP_MEMO.clear()
    _GAP_MEMO[memo] = out
    return out


def arm_gap(objects, part, robots=ROBOTS) -> float:
    keys = [k for k in LINK_POSES if k.split("/")[0] in robots]
    return min((link_gap(objects, k, part) for k in keys), default=0.02)


# -- the trial's ledger ----------------------------------------------------------------------------------------------------
class Ledger:
    """One trial: the forced and removed latches and the still/hold count."""

    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.forced = False
        self.removed = False
        self.clear = False
        self.last = None
        self.good = 0
        self.n = 0
        self.state = {}
        self.gaps = {}

    def step(self, objects) -> None:
        self.n += 1
        d = opening(objects)
        mp = np.asarray(objects["masher"].pos, float)
        is_flat, inside = flat(objects), in_drawer(objects)
        if d >= PITCH_FROM and pitch_deg(objects) > PITCH_FORCED_DEG:
            self.forced = True  # levered up against the rail (a yank)
        if (d >= OPEN_PAST and not is_flat) or over_rail(objects):
            self.forced = True  # a head too tall for the rail got under or past it: forced through
        if not inside or lowest_over_floor(objects) > LIFTED:
            self.removed = True  # taken out of the drawer (or lifted off its floor)
        gaps = {part: arm_gap(objects, part) for part in ("masher", "front")}
        self.gaps = gaps
        self.clear = min(gaps.values()) > CLEAR
        still = self.last is not None and max(float(np.linalg.norm(mp - self.last[0])),
                                              abs(d - self.last[1])) <= STILL_STEP
        self.last = (mp, d)
        self.state = dict(d=d, flat=is_flat, inside=inside)
        done = self.tier() == 0.5 and d >= OPEN_FULL and self.clear and still
        self.good = self.good + 1 if done else 0

    def tier(self) -> float:
        s = self.state
        if self.forced or self.removed or not s or not (s["flat"] and s["inside"]):
            return 0.0
        return 0.5 if s["d"] >= OPEN_PAST else 0.25

    def score(self) -> float:
        s = self.tier()
        if s == 0.5 and self.good >= HOLD_STEPS:
            return 1.0
        return s


_L = None


def _read_links(base) -> None:
    """Every arm link's pose (env frame) at this control step, into LINK_POSES. A missing arm or link, or finger pads
    left without their rubber, stops the trial: the contact checks and the grip would otherwise pass silently."""
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    for robot in ROBOTS:
        try:
            art = base.scene[robot]
        except KeyError as e:
            raise RuntimeError(f"masher: no {robot} in the scene for the contact checks") from e
        names = list(art.body_names)
        P = getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
        Q = getattr(art.data.body_quat_w, "torch", art.data.body_quat_w)[0].detach().cpu().numpy().astype(float)
        missing = [link for link in ARM_LINKS if link not in names]
        if missing:
            raise RuntimeError(f"masher: {robot} has no link(s) {missing} for the contact checks")
        for link in ARM_LINKS:
            i = names.index(link)
            LINK_POSES[f"{robot}/{link}"] = (P[i] - origin, rotm(Q[i]))
    if gripkit._STATE.get("bound", 0) < 0:
        raise RuntimeError("masher: the finger pads' rubber material could not be bound")
    if not _READ_ONCE:
        _READ_ONCE.append(1)
        print(f"masher: contact checks on {len(LINK_POSES)} arm links; pad shapes with rubber: "
              f"{gripkit._STATE.get('bound')}", file=sys.stderr, flush=True)


_READ_ONCE: list = []
kit.STATE_HOOKS.append(_read_links)
if "isaaclab" in sys.modules:  # in the simulator: read and check the hulls as the task loads, not in a trial's step
    yg.warm(ARM_LINKS)


def _update(objects):
    global _L
    if _L is None or _L.episode != kit.episode():
        _L = Ledger()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else ("obj", id(objects))
    if key == _L.key:
        return
    _L.key = key
    _L.step(objects)


def success(objects):
    """The drawer fully open with the masher lying flat inside it, hands clear, all still: this step counts toward the
    1 s hold."""
    _update(objects)
    return _L.good >= 1


def score(objects):
    """1.0: the drawer out at least 295 mm (its stop is at 300), the masher lying flat in it (its highest point at most
    83 mm over the drawer's floor, 2 mm under the rail; its whole outline inside the box), every arm link at least 5 mm
    clear of the masher and of the drawer's front and pull, all still (under 1 cm/s) for 1 s. 0.5: the masher flat in
    the drawer and the drawer out at least 280 mm (past the snag), but not fully out, not let go of, or not still.
    0.25: the masher lying flat in the drawer, the drawer not past the snag. 0: anything else: the masher still on end
    (doing nothing; the drawer pulled to the snag); at any moment (latched): levered more than 30 degrees off level
    with the drawer out 205 mm or more (a yank against the rail), any point of the head or fork higher than the rail's
    underside while under or in front of the rail, or of the grip while under the rail (forced through), the drawer past 280 mm while the masher is not flat, lifted more than 40 mm off the drawer's floor or
    out of the box."""
    _update(objects)
    return _L.score()


# -- scripted demo: open loop from the fixed layout --------------------------------------------------------------------------
# The left hand takes the bar pull from above and behind (fingers closing round the bar, the hand pointing forward
# and down), pulls the drawer out to the snag, eases it back to 185 mm open and lets go. The right hand reaches into
# the open drawer, takes the masher's grip from above (fingers closing across it), rolls it 40 degrees toward the
# drawer's middle, past the point where the head tips over, lets go and slides its fingers off the grip lengthwise: the
# head falls onto its fins. The left takes the pull again and draws the drawer out to its stop, lets go and both arms
# go home.
SNAG_PULL = 0.222  # m: pulled this far (the masher's fork meets the rail at about 0.215)
AT_STOP = TRAVEL - 0.002  # m: where the hand lets go, the drawer at its stop
EASE = 0.185  # m: eased back here, the masher clear of the rail while it is turned (it rises to 0.199)
FULL_PULL = 0.310  # m: drawn 1 cm into the stop (at 0.300): the hand takes the bar wherever the turn left the drawer
PULL_SQUEEZE = 0.002  # m a side: the left closes this far past the bar
PULL_TCP_UP = 0.004  # m: the left's tool point this far over the bar's axis
ROLL_DEG = 40.0  # the right's roll of the grip (the head tips over past 33 degrees; the hand's give adds more)
GRIP_AT = NECK_X1 + 0.012  # m: the right holds the grip this far from the head (12 mm along the grip)
GRIP_SQUEEZE = 0.003  # m a side
FINGER_HALF_OPEN = 0.047  # m: each finger's inner face from the tool point when fully open
YAW_X = -math.pi / 2  # fingers closing along world x ...
PULL_PITCH = math.radians(-60.0)  # ... with the hand pitched to point forward and down: it pinches the round bar
# from above and behind, its body kept back from the drawer front (straight down, the hand's body overhangs the front)


def finger_for(half: float) -> float:
    """Gripper command whose finger faces stand `half` m from the tool point."""
    return FINGER_CLOSED + (FINGER_OPEN - FINGER_CLOSED) * min(FINGER_HALF_OPEN, max(0.0, half)) / FINGER_HALF_OPEN


def pull_tcp(d: float, up: float = 0.0) -> tuple:
    """The left's tool point on the bar pull with the drawer out `d` m."""
    return (XF + _PULL_X - d, CAB_Y, TOP + PULL_Z + PULL_TCP_UP + up)


def grip_tcp(d: float, up: float = 0.0) -> tuple:
    """The right's tool point on the masher handle's axis (its start pose) with the drawer out `d` m."""
    p = MASHER_START_C + MASHER_R0 @ np.array([GRIP_AT, 0.0, 0.0])
    return (XF + float(p[0]) - d, CAB_Y + float(p[1]), TOP + float(p[2]) + up)


def _off_axis(p, roll: float, dist: float, up: float = 0.0) -> tuple:
    """`dist` m from p back along the right hand's own axis (rolled `roll` about x), so the fingers slide off the
    handle lengthwise instead of hooking it and rolling it back; then `up` m higher."""
    return (p[0], p[1] - dist * math.sin(roll), p[2] + dist * math.cos(roll) + up)


def _tw(duration, pos, finger, label, yaw=0.0, roll=0.0, pitch=0.0):
    return kit.Tilted(float(duration), tuple(float(v) for v in pos), float(finger), label, yaw=float(yaw),
                      roll=float(roll), pitch=float(pitch))


def _hold(seq, duration, label):
    """Repeat a sequence's last waypoint for `duration` s (waiting for the other hand)."""
    if duration > 1e-6:
        w = seq[-1]
        if isinstance(w, kit.Home):
            seq.append(kit.Home(float(duration), label=label))
        else:
            seq.append(kit.Tilted(float(duration), w.pos, w.finger, label, yaw=w.yaw, roll=w.roll, pitch=w.pitch))


def _t(seq):
    return sum(w.duration for w in seq)


def _along(d: float, back: float) -> tuple:
    """The left's tool point `back` m behind the bar along the hand's own axis (it points forward, 30 degrees below
    level), so it comes onto the bar and leaves it fingers first."""
    point = np.array([math.sin(-PULL_PITCH), 0.0, -math.cos(-PULL_PITCH)])
    return tuple(float(v) for v in np.array(pull_tcp(d)) - back * point)


def _take_pull(d: float, label: str) -> list:
    p_open, p_hold = finger_for(PULL_R + 0.010), finger_for(PULL_R - PULL_SQUEEZE)
    b = _along(d, 0.060 if d < 0.15 else 0.035)
    return [_tw(3.5, (b[0], b[1], b[2] + 0.08), p_open, f"high behind the pull ({label})", YAW_X, pitch=PULL_PITCH),
            _tw(1.5, b, p_open, "down behind the pull", YAW_X, pitch=PULL_PITCH),
            _tw(1.5, pull_tcp(d), p_open, "fingers either side of the bar", YAW_X, pitch=PULL_PITCH),
            _tw(0.4, pull_tcp(d), p_open, "settle", YAW_X, pitch=PULL_PITCH),
            _tw(1.0, pull_tcp(d), p_hold, "grip the bar", YAW_X, pitch=PULL_PITCH),
            _tw(0.4, pull_tcp(d), p_hold, "settle", YAW_X, pitch=PULL_PITCH)]


def _let_go_pull(d: float) -> list:
    p_open = finger_for(PULL_R + 0.010)
    back = _along(d, 0.060 if d < 0.15 else 0.030)  # drawn out, the hand is near its own base: a shorter back-off
    out = [_tw(1.0, pull_tcp(d), p_open, "let go of the pull", YAW_X, pitch=PULL_PITCH),
           _tw(1.2, back, p_open, "back off the pull", YAW_X, pitch=PULL_PITCH),
           _tw(1.3, (back[0], back[1], back[2] + 0.06), p_open, "up", YAW_X, pitch=PULL_PITCH)]
    if d >= 0.15:  # drawn out, the front stands just ahead of the hand: up over it, turning to point down
        out.append(_tw(2.0, (back[0] + 0.015, back[1], TOP + 0.21), p_open, "up over the front", YAW_X))
    return out


def demo():
    """Left pulls the drawer to the snag and eases it back; right rolls the masher flat by its handle; left pulls the
    drawer out to its stop. MASHER_VARIANT=nothing: both arms stay at the ready pose. =yank: the left pulls straight
    to the stop without the masher being turned. =lift: after turning it, the right lifts the masher out of the drawer
    before the left opens it. =close: the full demo, then the left pushes the drawer shut again. =hold: the full demo,
    the left never letting go of the pull."""
    variant = os.environ.get("MASHER_VARIANT", "")
    if variant == "nothing":
        return {"left": [kit.Home(10.0, label="stay at the ready pose")],
                "right": [kit.Home(10.0, label="stay at the ready pose")]}
    p_hold = finger_for(PULL_R - PULL_SQUEEZE)
    left = [kit.Home(0.5, label="start")] + _take_pull(0.0, "drawer shut")
    right = [kit.Home(0.5, label="start")]
    if variant == "yank":
        left += [_tw(4.0, pull_tcp(FULL_PULL), p_hold, "pull it straight out to the stop", YAW_X, pitch=PULL_PITCH),
                 _tw(2.0, pull_tcp(FULL_PULL), p_hold, "keep pulling", YAW_X, pitch=PULL_PITCH)]
        left += _let_go_pull(AT_STOP) + [kit.Home(3.5, label="back to the ready pose"), kit.Home(2.0, label="wait")]
        _hold(right, _t(left) - _t(right), "wait")
        return {"left": left, "right": right}
    left += [_tw(4.0, pull_tcp(SNAG_PULL), p_hold, "pull it out to the snag", YAW_X, pitch=PULL_PITCH),
             _tw(0.6, pull_tcp(SNAG_PULL), p_hold, "it stops", YAW_X, pitch=PULL_PITCH),
             _tw(1.5, pull_tcp(EASE), p_hold, "ease it back off the rail", YAW_X, pitch=PULL_PITCH)]
    left += _let_go_pull(EASE) + [kit.Home(4.0, label="out of the way")]
    # right: in over the handle while the left backs off, grip, roll, let go
    _hold(right, _t(left) - 4.0 - _t(right), "wait for the drawer")
    g_open, g_hold = finger_for(GRIP_R + 0.012), finger_for(GRIP_R - GRIP_SQUEEZE)
    g_free = finger_for(GRIP_R + 0.004)  # let go only a little: rolled, the lower finger opens toward the floor
    roll = math.radians(ROLL_DEG)
    right += [_tw(3.5, grip_tcp(EASE, up=0.06), g_open, "over the masher's handle"),
              _tw(2.0, grip_tcp(EASE), g_open, "fingers either side of the handle"),
              _tw(0.4, grip_tcp(EASE), g_open, "settle"),
              _tw(1.2, grip_tcp(EASE), g_hold, "grip the handle"),
              _tw(0.4, grip_tcp(EASE), g_hold, "settle"),
              _tw(3.0, grip_tcp(EASE, up=0.002), g_hold, "roll it toward the middle", roll=roll),
              _tw(0.4, grip_tcp(EASE, up=0.002), g_hold, "the head tips over", roll=roll),
              _tw(1.0, grip_tcp(EASE, up=0.006), g_free, "let go: it falls flat", roll=roll),
              _tw(1.5, _off_axis(grip_tcp(EASE, up=0.006), roll, 0.06), g_free, "back off along the hand", roll=roll),
              _tw(2.0, _off_axis(grip_tcp(EASE, up=0.006), roll, 0.06, up=0.04), g_open, "up out of the drawer")]
    if variant == "lift":  # take the masher out of the drawer and put it on the bench beside the cabinet
        flat_at = grip_tcp(EASE)
        right += [_tw(2.0, (flat_at[0], flat_at[1] - 0.02, flat_at[2] + 0.06), g_open, "over the handle again"),
                  _tw(1.5, (flat_at[0], flat_at[1] - 0.02, flat_at[2]), g_open, "down to it"),
                  _tw(1.0, (flat_at[0], flat_at[1] - 0.02, flat_at[2]), g_hold, "grip it"),
                  _tw(2.5, (flat_at[0] - 0.06, flat_at[1] - 0.02, flat_at[2] + 0.08), g_hold, "lift it out"),
                  _tw(3.0, (0.30, -0.25, TOP + 0.08), g_hold, "over the bench"),
                  _tw(1.5, (0.30, -0.25, TOP + 0.04), g_hold, "set it down"),
                  _tw(1.0, (0.30, -0.25, TOP + 0.04), g_open, "let go")]
    right += [kit.Home(3.5, label="back to the ready pose")]
    _hold(left, _t(right) - _t(left) - 3.0, "wait for the masher to be turned")
    left += _take_pull(EASE, "the masher turned")
    left += [_tw(3.0, pull_tcp(FULL_PULL), p_hold, "draw it out to its stop", YAW_X, pitch=PULL_PITCH),
             _tw(0.5, pull_tcp(FULL_PULL), p_hold, "settle", YAW_X, pitch=PULL_PITCH)]
    if variant == "close":
        left += [_tw(4.0, pull_tcp(0.02), p_hold, "push it back in", YAW_X, pitch=PULL_PITCH),
                 _tw(0.5, pull_tcp(0.02), p_hold, "settle", YAW_X, pitch=PULL_PITCH)]
        end = 0.02
    else:
        end = FULL_PULL
    if variant == "hold":
        left += [_tw(4.0, pull_tcp(end), p_hold, "keep hold of the pull", YAW_X, pitch=PULL_PITCH)]
    else:
        left += _let_go_pull(min(end, AT_STOP)) + [kit.Home(4.5, label="back to the ready pose"),
                                                   kit.Home(2.0, label="wait")]
    _hold(right, _t(left) - _t(right), "wait")
    _hold(left, _t(right) - _t(left), "wait")
    return {"left": left, "right": right}
