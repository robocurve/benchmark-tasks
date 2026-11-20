"""Post three letters through the pull-down hopper of a wall-mounted collection box.

On the station table stands a full-width mock of the hopper section of a Florence versatile 4C collection box
(4C11S-HOP, its drawing 4C11S-HOPCS): a 17 1/2 in wide plywood box, its face toward the arms, the hopper door in the
face. The hopper turns on a horizontal hinge rod along its bottom edge: pulled down by its handle, its top swings out
toward the user and the hopper opens into a V between the door and its floor; let go, two torsion springs on the rod
(Lee Spring LTR070M01S) swing it shut and its floor tips down into the box, so whatever lies in it slides off into the
compartment. Left of the box, three stuffed #10 envelopes stand on their long edge in the slot of a desk letter rack.

The job: get all three envelopes into the box through the hopper, none dropped outside, and the hopper shut at the end.

What makes it hard: the hopper shuts itself, so one hand has to pull it open by its handle and hold it against the
springs while the other hand takes the thin envelopes out of the rack and lowers them into the V, which is only about
14 cm across at its mouth. Let the hopper go before the envelopes are wholly in it, and it jams on them or they fall
outside; the springs then close it and do the posting. Nothing gets into the box except through the hopper.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import guardkit as gk  # noqa: E402
import yamkit as kit  # noqa: E402
import yamgeom as yg  # noqa: E402  (the arm's collision hulls, read from the environment at run time)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DT = 0.05  # s: one control step (20 Hz)
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
RUN_LIMIT_STEPS = 2400
INSTRUCTION = "Post the three letters from the letter rack in the drop box, and leave its hopper shut."

BOX_LOOK = kit.Look((0.23, 0.25, 0.28), 0.5)  # painted plywood, postal grey
FACE_LOOK = kit.Look((0.30, 0.32, 0.35), 0.45)
HOPPER_LOOK = kit.Look((0.55, 0.57, 0.60), 0.4)
HANDLE_LOOK = kit.Look((0.75, 0.75, 0.73), 0.25)  # stainless bar pull
RACK_LOOK = kit.Look((0.55, 0.38, 0.22), 0.6)  # wood
ENV_LOOK = kit.Look((0.96, 0.96, 0.94), 0.8)
STAMP_LOOK = kit.Look((0.75, 0.20, 0.20), 0.6)

# -- the box (its base frame: origin on the hinge axis, at the middle of the hopper; x into the box (world +x), y to
#    the left (world +y), z up; world-aligned). The hinge rod turns in two pillow blocks screwed to the face's front
#    beside the opening, so the axis is 12 mm in front of the face. ---------------------------------------------------
PIVOT_X = 0.408  # world x of the hinge axis
PIVOT_Z = 0.09  # the hinge axis this far above the table top
BOX_W = 0.4445  # 17 1/2 in: the unit's actual width (drawing), the face's width
FACE_OFF = 0.012  # the face's front plane, behind the hinge axis
FACE_T = 0.019  # 3/4 in plywood face
FACE_BACK = FACE_OFF + FACE_T
PLY_T = 0.0127  # 1/2 in plywood sides, top, back and floor
IN_HW = 0.1524  # the compartment: 12 in wide (drawing note 3) ...
IN_D = 0.381  # ... and 15 in deep behind the face
TOP_UP = 0.25  # the box's top this far above the hinge axis
LIP = 0.006  # the hopper opening's bottom edge above the hinge axis ...
OPEN_H = 0.19  # ... and its top edge; it is the compartment's width across
BOX_POS = (PIVOT_X, 0.0, TOP + PIVOT_Z)
BOX_D = FACE_OFF + FACE_T + IN_D + PLY_T  # from the hinge axis to the box's back
_HB = BOX_W / 2
# the compartment inside: x, |y| and z limits (box frame)
IN_X = (FACE_BACK, FACE_BACK + IN_D)
IN_Y = IN_HW
IN_Z = (-PIVOT_Z + PLY_T, TOP_UP - PLY_T)
BRACKET = ((-0.012, FACE_OFF), (0.168, 0.193), (-0.015, 0.015))  # each pillow block: x, |y| and z spans

# -- the hopper (link frame = box frame at shut, origin on the hinge axis). Its door (1/4 in plywood) overlays the
#    opening on the face's front; its floor, glued along the door's foot (one piece with the door, as the unit's
#    hopper is), runs in level through the face's thickness, then slopes down into the box; two side cheeks of 1/8 in
#    hardboard, each with a lobe round the hinge rod, which is pinned through them.
HOP_T = 0.0064  # 1/4 in plywood
CHEEK_T = 0.0032  # 1/8 in hardboard
DOOR_H = 0.20  # the door, hinge axis to its top edge (7.9 in; it overlaps the opening's top by 1 cm)
DOOR_Z0 = 0.007  # its bottom edge (clear of the 3/8 in rod)
DOOR_W = 0.33  # across (overlapping the opening 1.3 cm each side)
HOP_IN = 0.289  # between the cheeks: the drawing's 11 3/8 in largest bundle
FLOOR_A = FACE_BACK + 0.012  # the floor's first run, level (shut) through the face's thickness ...
FLOOR_SLOPE = math.radians(34.0)  # ... then down into the box at this slope when shut (about 11 degrees up open)
FLOOR_L = 0.14  # the sloped run's length (the mock's compartment is lower than the unit's)
OPEN_DEG = 45.0  # the hopper's stop: open this far (the drawing shows it about 45 degrees open)
HOP_KG = 0.70  # the mock hopper with its pull, weighed (washers glued under the floor bring it to 0.70 kg)
# The springs: two Lee Spring LTR070M01S torsion springs on the hinge rod, each 6.998 lb-in at 90 degrees of wind
# (8.79 N*mm per degree), wound 40 degrees at shut: 0.70 N*m holding it shut, 1.49 N*m at the 45-degree stop
SPRING_NMM_PER_DEG = 790.668 / 90.0
SPRINGS = 2
SPRING_K = SPRINGS * SPRING_NMM_PER_DEG / 1000.0 * 180.0 / math.pi  # N*m/rad
PRELOAD_DEG = 40.0
HINGE_DAMP = 0.02  # N*m*s/rad: the hinge rod's friction in its blocks
SHUT_DEG = 0.8  # the hopper is shut within 0.8 degrees of closed (its top edge within 4 mm of the face)

# The pull: an Amerock BP40517 bar pull (128 mm between centres, 187 mm long, 13 mm bar, 35 mm projection), level
# across the door just under its top edge, where the unit's user access handle is
BAR_D = 0.013
BAR_L = 0.187
POST_CC = 0.128
PULL_OUT = 0.035
DOOR_BACK = FACE_OFF - 0.001  # the door's inner face, just off the face's front at shut
DOOR_FRONT = DOOR_BACK - HOP_T
BAR_Z = DOOR_H - 0.022  # the bar's centre line above the hinge axis (its top 1.5 cm under the door's top edge)
BAR_X = DOOR_FRONT - PULL_OUT + BAR_D / 2  # ... and its x (link frame)

_ZA = LIP + 0.0015 + HOP_T / 2  # the level run's middle (its underside 1.5 mm over the opening's bottom edge)
_SB = (FLOOR_A + FLOOR_L * math.cos(FLOOR_SLOPE), _ZA - FLOOR_L * math.sin(FLOOR_SLOPE))  # the floor's far end

# The curved back: fixed in the box behind the hopper, a plate bent round the hinge axis (the drawing's curve) from
# the opening's top edge down past the floor's far end. Open, the hopper's floor ends against it, so the only way
# from the open hopper into the box is the hopper's own return; shut, the floor's end is below it and letters slide
# off underneath it.
BACK_R = 0.179  # its inner face's radius about the hinge axis
BACK_PSI = (20.0, 100.0)  # degrees from straight up toward the box: where it runs
BACK_SEGS = 8


def _slab(a, b, half_w, t):
    """A plate t thick (normal in the x-z plane) from point a to point b (x, z) across y +-half_w: hull points."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = (b - a) / np.linalg.norm(b - a)
    n = np.array([d[1], -d[0]]) * t / 2
    pts = []
    for p in (a + n, a - n, b - n, b + n):
        for y in (-half_w, half_w):
            pts.append((float(p[0]), float(y), float(p[1])))
    return tuple(pts)


def _prism(poly, y0, y1):
    """A convex polygon (x, z) extruded from y0 to y1: hull points."""
    return tuple((float(x), float(y), float(z)) for y in (y0, y1) for (x, z) in poly)


def _faces(pts):
    """Triangles of a convex point set (its hull, by brute force; small sets only)."""
    P = np.asarray(pts, float)
    c = P.mean(axis=0)
    out = []
    n = len(P)
    for i in range(n):
        for j in range(i + 1, n):
            for k in range(j + 1, n):
                nrm = np.cross(P[j] - P[i], P[k] - P[i])
                if np.linalg.norm(nrm) < 1e-12:
                    continue
                s = (P - P[i]) @ nrm
                if (s <= 1e-9).all() or (s >= -1e-9).all():
                    if np.dot(nrm, c - P[i]) > 0:
                        out.append((i, k, j))
                    else:
                        out.append((i, j, k))
    return tuple(out)


def _hull_part(pts, look):
    pts = tuple(pts)
    f = _faces(pts)
    mesh = kit._mesh(pts, [3] * len(f), [j for t in f for j in t], look)
    return mesh, kit.Collider((0.0, 0.0, 0.0), hull=pts, faces=f)


def _rod(length: float, d: float, n: int = 12):
    """A rod along y, `d` across (an n-sided section), as hull points about its middle."""
    ring = [(d / 2 * math.cos(2 * math.pi * (k + 0.5) / n), d / 2 * math.sin(2 * math.pi * (k + 0.5) / n))
            for k in range(n)]
    return tuple((x, sy * length / 2, z) for sy in (-1, 1) for (x, z) in ring)


def _hopper_parts():
    """The door, the floor (level run and sloped run), the two cheeks with their lobes, and the pull: meshes and
    colliders, link frame."""
    meshes, cols = [], []
    door = ((DOOR_FRONT + DOOR_BACK) / 2, 0.0, (DOOR_Z0 + DOOR_H) / 2)
    dsize = (HOP_T, DOOR_W, DOOR_H - DOOR_Z0)
    meshes.append(kit.box(door, dsize, HOPPER_LOOK))
    cols.append(kit.Collider(door, size=dsize))
    hw = HOP_IN / 2
    for pts in (_slab((DOOR_FRONT, _ZA), (FLOOR_A, _ZA), hw, HOP_T),  # glued along the door's foot: no slot
                _slab((FLOOR_A, _ZA), _SB, hw, HOP_T)):
        m, c = _hull_part(pts, HOPPER_LOOK)
        meshes.append(m)
        cols.append(c)
    # the cheeks: an upper part (from the door's back up to under the opening's top edge, out to the level run's end),
    # a lower one (down along the sloped floor) and a lobe round the rod, each convex
    top = (DOOR_BACK + 0.0005, OPEN_H - 0.012)

    def on_edge(x):  # the cheek's upper edge (top -> far end) at x
        t = (x - top[0]) / (_SB[0] - top[0])
        return top[1] + t * (_SB[1] - top[1])

    z0 = LIP + 0.0015
    upper = [(top[0], z0), top, (FLOOR_A, on_edge(FLOOR_A)), (FLOOR_A, z0)]
    lower = [(FLOOR_A, z0), (FLOOR_A, on_edge(FLOOR_A)), _SB]
    lobe = [(0.010 * math.cos(2 * math.pi * (k + 0.5) / 8), 0.010 * math.sin(2 * math.pi * (k + 0.5) / 8))
            for k in range(8)]
    for sy in (-1, 1):
        y0, y1 = sy * hw, sy * (hw + CHEEK_T)
        for poly in (upper, lower, lobe):
            m, c = _hull_part(_prism(poly, min(y0, y1), max(y0, y1)), HOPPER_LOOK)
            meshes.append(m)
            cols.append(c)
    # the pull: the bar (level, across the door) and its two posts
    bar = tuple((x + BAR_X, y, z + BAR_Z) for (x, y, z) in _rod(BAR_L, BAR_D))
    m, c = _hull_part(bar, HANDLE_LOOK)
    meshes.append(m)
    cols.append(c)
    post_len = DOOR_FRONT - BAR_X
    for sy in (-1, 1):
        pc, ps = ((DOOR_FRONT + BAR_X) / 2, sy * POST_CC / 2, BAR_Z), (post_len, 0.010, 0.010)
        meshes.append(kit.box(pc, ps, HANDLE_LOOK))
        cols.append(kit.Collider(pc, size=ps))
    return tuple(meshes), tuple(cols)


class _Springs:
    """The two torsion springs on the hinge rod, as PhysX's own (implicit) joint drive: a spring toward PRELOAD_DEG
    past shut (so it holds the hopper shut with SPRING_K x preload), and the rod's damping. Opening is negative (the
    top swings out, toward the arms)."""

    def drive(self, key, q, qd, J, mem):
        return math.radians(PRELOAD_DEG), SPRING_K, 0.0, HINGE_DAMP


_HM, _HC = _hopper_parts()
HOPPER = kit.Link("hopper", "revolute", "Y", anchor=(0.0, 0.0, 0.0), lower=-math.radians(OPEN_DEG), upper=0.0,
                  start=0.0, mass=HOP_KG, meshes=_HM, colliders=_HC, damping=HINGE_DAMP, law=_Springs())


def back_points(r: float = BACK_R):
    """Points of the curved back's inner face (x, z), at radius r, from BACK_PSI[0] to BACK_PSI[1]."""
    a0, a1 = (math.radians(v) for v in BACK_PSI)
    return [(r * math.sin(a0 + (a1 - a0) * k / BACK_SEGS), r * math.cos(a0 + (a1 - a0) * k / BACK_SEGS))
            for k in range(BACK_SEGS + 1)]


def _box():
    """The face (with its hopper opening), the compartment's sides, back, top and floor, the curved back and the two
    pillow blocks: meshes and colliders (box frame)."""
    zb, zt = -PIVOT_Z, TOP_UP
    fx = FACE_OFF + FACE_T / 2
    wall_x = (FACE_BACK + IN_X[1] + PLY_T) / 2
    parts = [((fx, 0.0, (zb + LIP) / 2), (FACE_T, BOX_W, LIP - zb)),  # face under the opening
             ((fx, 0.0, (OPEN_H + zt) / 2), (FACE_T, BOX_W, zt - OPEN_H))]  # over it
    for sy in (-1, 1):
        parts.append(((fx, sy * (IN_HW + _HB) / 2, (LIP + OPEN_H) / 2), (FACE_T, _HB - IN_HW, OPEN_H - LIP)))  # beside
        parts.append(((wall_x, sy * (IN_HW + PLY_T / 2), (zb + zt) / 2), (IN_X[1] + PLY_T - FACE_BACK, PLY_T, zt - zb)))
    parts += [((IN_X[1] + PLY_T / 2, 0.0, (zb + zt) / 2), (PLY_T, 2 * IN_HW, zt - zb)),  # back
              (((FACE_BACK + IN_X[1]) / 2, 0.0, zt - PLY_T / 2), (IN_D, 2 * IN_HW, PLY_T)),  # top
              (((FACE_BACK + IN_X[1]) / 2, 0.0, zb + PLY_T / 2), (IN_D, 2 * IN_HW, PLY_T))]  # floor
    for sy in (-1, 1):
        (x0, x1), (y0, y1), (z0, z1) = BRACKET
        parts.append((((x0 + x1) / 2, sy * (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0)))
    meshes = [kit.box(c, s, FACE_LOOK if c[0] <= FACE_BACK else BOX_LOOK) for c, s in parts]
    cols = [kit.Collider(c, size=s) for c, s in parts]
    # the curved back: a short roof from under the opening's top edge, then the bend round the hinge axis
    mid = back_points(BACK_R + HOP_T / 2)
    chain = [(FACE_BACK, OPEN_H + HOP_T / 2)] + mid
    for a, b in zip(chain[:-1], chain[1:]):
        m, c = _hull_part(_slab(a, b, IN_HW, HOP_T), BOX_LOOK)
        meshes.append(m)
        cols.append(c)
    return tuple(meshes), tuple(cols)


_BM, _BC = _box()
DROPBOX = kit.Mechanism("dropbox", pos=BOX_POS, size=(2 * BOX_D, BOX_W, 2 * max(PIVOT_Z, TOP_UP)), meshes=_BM,
                        colliders=_BC, links=(HOPPER,), friction=(0.4, 0.3))

# -- the letter rack and the envelopes ----------------------------------------------------------------------------------
ENV = (0.0006, 0.2413, 0.1048)  # a stuffed #10 envelope standing on its long edge: thickness (x), 9.5 in (y), 4 1/8 in
ENV_KG = 0.010  # a #10 envelope of 24 lb paper (about 5.5 g) holding one folded 20 lb letter sheet (4.5 g)
ENV_FRICTION = (0.4, 0.3)  # paper on paper, wood and painted plywood (static, dynamic)
ENVS = ("letter_a", "letter_b", "letter_c")
RACK_POS = (0.30, 0.30, TOP)
RACK_SLOT = 0.010  # the slot's width
RACK_WALL = (0.006, 0.040)  # the rack's walls: thickness, height above the slot's floor
RACK_BASE = 0.006
RACK_L = 0.27


def _rack() -> kit.Prop:
    s, (wt, wh), b = RACK_SLOT, RACK_WALL, RACK_BASE
    parts = [((0.0, 0.0, b / 2), (s + 2 * wt + 0.04, RACK_L, b))]
    for sx in (-1, 1):
        parts.append(((sx * (s / 2 + wt / 2), 0.0, b + wh / 2), (wt, RACK_L, wh)))
    return kit.Prop("rack", pos=RACK_POS, size=(s + 2 * wt + 0.04, RACK_L, b + wh),
                    meshes=tuple(kit.box(c, sz, RACK_LOOK) for c, sz in parts),
                    colliders=tuple(kit.Collider(c, size=sz) for c, sz in parts))


RACK = _rack()


def _lean():
    """The stack's lean in the slot (rad, top toward +x): its bottom against the near wall, its far face on the far
    wall's top edge."""
    t = len(ENVS) * ENV[0]
    a = 0.0
    for _ in range(50):
        a = math.atan((RACK_SLOT - t / math.cos(a)) / RACK_WALL[1])
    return a


LEAN = _lean()


def env_start(i: int):
    """Envelope i's start: its middle (world) and its tilt (a turn of LEAN about y), i = 0 nearest the arms."""
    a = LEAN
    u = np.array([math.sin(a), 0.0, math.cos(a)])  # up its face
    n = np.array([math.cos(a), 0.0, -math.sin(a)])  # through it, toward the far wall
    near_bottom = np.array([RACK_POS[0] - RACK_SLOT / 2 + 0.0004, RACK_POS[1], TOP + RACK_BASE + 0.0004 +
                            len(ENVS) * ENV[0] * math.sin(a)])
    mid = near_bottom + n * ENV[0] * (i + 0.5) + u * ENV[2] / 2
    return mid, (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))


def _envelope(i: int, name: str) -> kit.Prop:
    mid, q = env_start(i)
    return kit.Prop(name, pos=tuple(float(v) for v in mid), size=ENV, mass=ENV_KG, tilt_xyzw=q,
                    meshes=(kit.box((0.0, 0.0, 0.0), ENV, ENV_LOOK),
                            kit.box((-ENV[0] / 2 - 0.0001, -0.09, 0.03), (0.0002, 0.02, 0.025), STAMP_LOOK)),
                    colliders=(kit.Collider((0.0, 0.0, 0.0), size=ENV),))


OBJECTS = [DROPBOX, RACK] + [_envelope(i, n) for i, n in enumerate(ENVS)]
FRICTION = {n: ENV_FRICTION for n in ENVS}
FRICTION["rack"] = ENV_FRICTION

# -- geometry helpers -------------------------------------------------------------------------------------------------
_CORNERS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float) * np.array(ENV) / 2
ENV_PARTS = (gk.Block((0.0, 0.0, 0.0), ENV),)


def rotm(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def corners_world(objects, name: str) -> np.ndarray:
    s = objects[name]
    return np.asarray(s.pos, float) + _CORNERS @ rotm(s.quat_xyzw).T


def corners_box(objects, name: str) -> np.ndarray:
    """The envelope's 8 corners in the box frame, from the box's pose as the simulator reports it."""
    b = objects["dropbox"]
    R = rotm(b.quat_xyzw)
    return (corners_world(objects, name) - np.asarray(b.pos, float)) @ R


def in_box(objects, name: str) -> bool:
    """Wholly inside the box: every corner behind the face's back plane, inside the side walls, back, floor and top."""
    c = corners_box(objects, name)
    return bool(c[:, 0].min() > IN_X[0] and c[:, 0].max() < IN_X[1] and np.abs(c[:, 1]).max() < IN_Y
                and c[:, 2].min() > IN_Z[0] - 0.002 and c[:, 2].max() < IN_Z[1])


OUT_LOW = 0.004  # m: an envelope with a corner this near the table top (or below it), outside the box, fell outside


def fell_outside(objects, name: str) -> bool:
    """Down on the table or the floor outside the box (its middle outside the box's footprint, a corner within
    OUT_LOW of the table top or below it)."""
    w = corners_world(objects, name)
    c = corners_box(objects, name)
    mid = c.mean(axis=0)
    inside = FACE_OFF < mid[0] < BOX_D and abs(mid[1]) < _HB
    return (not inside) and float(w[:, 2].min()) < TOP + OUT_LOW


def hopper_angle(objects) -> float:
    """The hopper's opening (rad, 0 = shut, positive = open)."""
    return -kit.joint(objects, "dropbox/hopper").pos


def hopper_shut(objects) -> bool:
    return hopper_angle(objects) <= math.radians(SHUT_DEG)


# -- the arms' own shapes: the collision hulls the simulator collides, every link from the shoulder to the fingertips
ARM_LINKS = ("link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
ROBOTS = ("left_robot", "right_robot")
LINK_POSES: dict = {}  # "left_robot/link5" -> (pos, R) in the env frame, this control step (empty offline)
CLEAR = 0.005  # m: a hand is off a part when every arm link's hull is more than 5 mm from it
_HALF_DIAG = float(np.linalg.norm(ENV)) / 2


def _box_gap(x, c, sz):
    return np.linalg.norm(np.maximum(np.abs(x - c) - sz / 2, 0.0), axis=1)


def _gap_to_boxes(o_f, R_f, boxes, reach, upto):
    """The smallest gap (m) from any arm link's hulls to `boxes` [(centre, size)] in the frame (o_f, R_f)."""
    centre = o_f
    best = math.inf
    for key, (o, R) in LINK_POSES.items():
        link = key.split("/")[1]
        _, c, r = yg.cloud(link)
        if float(np.linalg.norm(o + R @ c - centre)) - r - reach > upto:
            continue

        def dist(x):
            return np.min([_box_gap(x, bc, bs) for bc, bs in boxes], axis=0)

        q = yg.near(link, lambda x, o=o, R=R: ((o + x @ R.T) - o_f) @ R_f, dist, upto)
        if len(q):
            best = min(best, float(dist(q).min()))
    return best


def env_gap(objects, name: str, upto: float = 0.02) -> float:
    s = objects[name]
    return _gap_to_boxes(np.asarray(s.pos, float), rotm(s.quat_xyzw),
                         [(np.zeros(3), np.asarray(ENV, float))], _HALF_DIAG, upto)


def _hopper_boxes():
    """The hopper's door and pull (bounding boxes, link frame): what a hand holds it by."""
    door = (np.array([(DOOR_FRONT + DOOR_BACK) / 2, 0.0, (DOOR_Z0 + DOOR_H) / 2]),
            np.array([HOP_T, DOOR_W, DOOR_H - DOOR_Z0]))
    pull = (np.array([(DOOR_FRONT + BAR_X - BAR_D / 2) / 2, 0.0, BAR_Z]),
            np.array([DOOR_FRONT - BAR_X + BAR_D / 2, BAR_L, BAR_D]))
    return [door, pull]


HOPPER_BOXES = _hopper_boxes()
HOPPER_REACH = float(max(np.linalg.norm(np.abs(c) + s / 2) for c, s in HOPPER_BOXES))  # m: the farthest the
# door or pull reaches from the hinge axis (the hands-off check's first cull)
HAND_R = 0.025  # m: offline tests only: a hand as a ball this big round its tool point


def hopper_frame(objects):
    """The hopper link's frame (origin on the hinge axis, rotation) from the box's pose and the joint angle."""
    b = objects["dropbox"]
    R = rotm(b.quat_xyzw)
    a = kit.joint(objects, "dropbox/hopper").pos
    Ry = np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])
    return np.asarray(b.pos, float), R @ Ry


def hopper_hands_off(objects) -> bool:
    """No arm link within CLEAR of the hopper's door or pull (the sim: every link's hulls; offline: a ball per hand)."""
    o_h, R_h = hopper_frame(objects)
    if LINK_POSES:
        return _gap_to_boxes(o_h, R_h, HOPPER_BOXES, HOPPER_REACH, 0.02) > CLEAR
    if _IN_SIM["on"]:
        raise RuntimeError("[dropbox] hands-off check: no arm link poses read from the simulator")
    for g in GRIPPERS:
        if g in objects:
            x = ((np.asarray(objects[g].pos, float) - o_h) @ R_h)[None, :]
            if min(float(_box_gap(x, bc, bs)[0]) for bc, bs in HOPPER_BOXES) <= CLEAR + HAND_R:
                return False
    return True


def hands_off(objects, name: str) -> bool:
    """No arm link within CLEAR of envelope `name` (the sim: every link's hulls; offline: guardkit's hand boxes)."""
    if LINK_POSES:
        return env_gap(objects, name) > CLEAR
    if _IN_SIM["on"]:
        raise RuntimeError("[dropbox] hands-off check: no arm link poses read from the simulator")
    return not gk.touching(objects, name, ENV_PARTS, CLEAR)


def link_pose(p, q_xyzw) -> tuple:
    """A link's (position, rotation) from the simulator's body pose: Isaac Lab reports quaternions as (x, y, z, w)."""
    return np.asarray(p, float), rotm(q_xyzw)


def corner_speed(objects, name: str) -> float:
    s = objects[name]
    v, w = np.asarray(s.lin_vel, float), np.asarray(s.ang_vel, float)
    r = _CORNERS @ rotm(s.quat_xyzw).T
    return float(np.linalg.norm(v + np.cross(w, r), axis=1).max())


# -- one trial's record -------------------------------------------------------------------------------------------------
GRIPPERS = ("left_gripper", "right_gripper")
REST_SPEED = 0.02  # m/s: at rest, no corner moves faster than this
STILL_STEP = 0.0005  # m: per control step, the most a resting envelope's corner moves
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)
PER_LETTER = 0.25
OPEN_CAP = 0.5  # the most a trial scores while the hopper is not shut


class Ledger:
    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.objs = None
        self.n = 0
        self.last = {}
        self.outside = {}  # envelope -> the step it fell outside (it then counts for nothing this trial)
        self.state = {e: False for e in ENVS}
        self.good = 0
        self.open_max = 0.0
        self.shut = True
        self.hopper_free = True

    def step(self, objects) -> None:
        self.n += 1
        self.open_max = max(self.open_max, hopper_angle(objects))
        for e in ENVS:
            pts = corners_world(objects, e)
            moved = float(np.linalg.norm(pts - self.last[e], axis=1).max()) if e in self.last else math.inf
            self.last[e] = pts
            if e not in self.outside and fell_outside(objects, e):
                self.outside[e] = self.n
            still = moved <= STILL_STEP and corner_speed(objects, e) <= REST_SPEED
            self.state[e] = (e not in self.outside and still and in_box(objects, e) and hands_off(objects, e))
        self.shut = hopper_shut(objects)
        self.hopper_free = hopper_hands_off(objects)
        done = all(self.state.values()) and self.shut and self.hopper_free
        self.good = self.good + 1 if done else 0
        if _TRACE:
            with open(_TRACE, "a") as f:
                f.write(f"{self.n} hop_deg={math.degrees(hopper_angle(objects)):.2f} free={int(self.hopper_free)} "
                        f"max={math.degrees(self.open_max):.1f} " + " ".join(
                            f"{e}:in={int(in_box(objects, e))} ok={int(self.state[e])} mid="
                            f"{np.round(corners_box(objects, e).mean(axis=0) * 1000).astype(int).tolist()}"
                            for e in ENVS) + f" outside={self.outside} good={self.good}\n")

    def tier(self) -> float:
        if self.good >= HOLD_STEPS:
            return 1.0
        n = PER_LETTER * sum(self.state.values())
        if not self.shut:
            n = min(n, OPEN_CAP)
        return round(n, 4)


_L = Ledger()
_TRACE = os.environ.get("DROPBOX_TRACE")


def _update(objects):
    global _L
    if _L.episode != kit.episode():
        _L = Ledger()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else None
    if (key is not None and key == _L.key) or (key is None and objects is _L.objs):
        return
    _L.key, _L.objs = key, objects
    _L.step(objects)


def success(objects):
    """All three envelopes inside the box's compartment, at rest, no hand on them, none ever down outside the box, and
    the hopper shut (within 1 degree) with no arm on it: held 1 s, the trial ends."""
    _update(objects)
    return _L.good >= HOLD_STEPS


def score(objects):
    """0.25 for each envelope inside the box's compartment at rest with no hand on it (one that fell to the table or
    floor outside the box counts for nothing); at most 0.5 while the hopper is not shut; 1.0 once all three are in and
    the hopper is shut with no arm on it, held 1 s. Possible scores: 0, 0.25, 0.5, 0.75, 1.0. Doing nothing scores
    0."""
    _update(objects)
    return _L.tier()


# -- the simulator hooks -------------------------------------------------------------------------------------------------
_IN_SIM = {"on": False, "told": False}


def _patch(scene) -> None:
    if getattr(scene, "_dropbox", False):
        return
    scene._dropbox = True
    yg.warm(ARM_LINKS)  # read and check the hulls now, after the app is up (never when the task file loads)
    inner_object, inner_states = scene.task_object, scene.object_states

    def task_object(spec):
        obj = inner_object(spec)
        if spec.name in FRICTION:
            s, d = FRICTION[spec.name]
            obj.spawn_cfg_addon["physics_material"] = scene.sim_utils.RigidBodyMaterialCfg(static_friction=s,
                                                                                           dynamic_friction=d)
            obj.object_cfg = obj._init_object_cfg()
        return obj

    def object_states(env, names):
        out = inner_states(env, names)
        if all(e in out for e in ENVS) and "dropbox" in out:
            _IN_SIM["on"] = True
            _read_links(env.unwrapped)
        return out

    scene.task_object, scene.object_states = task_object, object_states


def _read_links(base) -> None:
    """Every arm link's pose (env frame) this control step, into LINK_POSES. A missing robot or link is an error."""
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    for robot in ROBOTS:
        try:
            art = base.scene[robot]
        except KeyError as e:
            raise RuntimeError(f"[dropbox] hands-off check: no robot {robot!r} in the scene") from e
        names = list(art.body_names)
        missing = [link for link in ARM_LINKS if link not in names]
        if missing:
            raise RuntimeError(f"[dropbox] hands-off check: {robot} has no link(s) {missing} (bodies {names})")
        P = getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
        Q = getattr(art.data.body_quat_w, "torch", art.data.body_quat_w)[0].detach().cpu().numpy().astype(float)
        for link in ARM_LINKS:
            i = names.index(link)
            LINK_POSES[f"{robot}/{link}"] = link_pose(P[i] - origin, Q[i])
    if not _IN_SIM["told"]:
        _IN_SIM["told"] = True
        print(f"[dropbox] hands-off check: {len(LINK_POSES)} link poses from the sim, hull check on", file=sys.stderr,
              flush=True)


class _Hook(kit._Hook):
    def exec_module(self, module):
        self._inner.exec_module(module)
        kit._patch(module)
        _patch(module)


if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])
else:  # replace yamkit's import hook with one that adds ours after it
    sys.meta_path[:] = [f for f in sys.meta_path if not isinstance(f, kit._Hook)]
    sys.meta_path.insert(0, _Hook())


# -- scripted demo: open loop from the fixed layout -------------------------------------------------------------------
# Right hand: level, its open fingers pointing left along the level pull's bar, slides them either side of the bar's
# right-hand end past its post (the inner one into the gap behind it), closes and pulls: the hopper swings open along
# its arc to near its stop, the hand rolling with the door; it holds it there while the left hand posts; then lets it
# back to a few degrees open, opens and slides its fingers back off the bar: the springs shut it. Left hand: comes
# straight down over the three envelopes in the rack (its fingers along their lean), pinches the stack by its top
# edge, lifts it out, carries it over the open hopper leaning out over the V, brings it in over the door's top edge
# and down along the door, lets go (they slide into the V) and backs out.
FINGER_HALF_OPEN = 0.047  # m: each finger's inner face from the tool point when fully open
HOLD_DEG = 44.0  # the right hand holds the hopper this far open (just short of its 45-degree stop)
EASE_DEG = 4.0  # ... and lets it back to this before letting go
BAR_WIDE = BAR_D / 2 + 0.006  # open half-gap round the bar
LEVEL = -math.pi / 2  # Tilted pitch: fingers forward, level
GRIP_Y = -(POST_CC / 2 + 0.005 + 0.017)  # the right hand's tool point on the bar's right-hand end, past its post
HOOK_LAT = 0.05  # the open hand comes in along the bar from this far right of the grip
HOOK_TURN = math.radians(20)  # the fingers point 20 degrees into the door off the bar's line, so the hand's palm and
# wrist stand clear of the box's face
PINCH_DOWN = 0.040  # m: the left hand's tool point this far below the envelopes' top edge
ENV_Y = 0.02  # the stack's middle goes into the hopper this far left of its middle (clear of the right hand)
DOOR_GAP = 0.034  # m: the stack's bottom edge this far in from the open door's inner face (the near finger between)
CROSS_UP = 0.005  # m: it comes in over the open door's top edge this high above it
LET_GO_Z = 0.07  # m: and is let go with its bottom edge this high above the hinge axis: it slides down the door
DROP_PITCH = math.radians(-40.0)  # the hand and the stack lean out 40 degrees over the V (about along the door)


def finger_for(half: float) -> float:
    """Gripper command whose finger faces stand `half` m from the tool point."""
    return FINGER_CLOSED + (FINGER_OPEN - FINGER_CLOSED) * max(0.0, half) / FINGER_HALF_OPEN


def _down(d, pos, f, label, pitch=0.0):
    """Fingers pointing down (turned `pitch` about world y), closing along world x."""
    return kit.Tilted(float(d), tuple(float(v) for v in pos), float(f), label, math.pi / 2, 0.0, float(pitch))


def _hook(d, pos, f, label, deg):
    """Right hand level, fingers pointing left (+y) along the bar, closing square to the door; rolled with the hopper
    open `deg` degrees (its fingers' closing line stays square to the door)."""
    return kit.Tilted(float(d), tuple(float(v) for v in pos), float(f), label, math.radians(deg), 0.0, LEVEL,
                      math.pi / 2 - HOOK_TURN)


ALONG = np.array([math.sin(HOOK_TURN), math.cos(HOOK_TURN), 0.0])  # the right hand's finger direction


def bar_point(deg: float) -> np.ndarray:
    """World position of the pull's bar centre line at GRIP_Y, the hopper open `deg` degrees."""
    a = -math.radians(deg)
    x, z = BAR_X, BAR_Z
    xr, zr = x * math.cos(a) + z * math.sin(a), -x * math.sin(a) + z * math.cos(a)
    return np.array([BOX_POS[0] + xr, GRIP_Y, BOX_POS[2] + zr])


def right_plan() -> list:
    wide = finger_for(BAR_WIDE)
    b0 = bar_point(0.0)
    side = b0 - HOOK_LAT * ALONG
    out = [_hook(3.5, side - np.array([0.05, 0.0, 0.0]), wide, "in front of the pull, right of it", 0.0),
           _hook(1.5, side, wide, "fingers level with the bar's end", 0.0),
           _hook(2.0, b0, wide, "fingers either side of the bar's end, one in behind it", 0.0),
           _hook(1.0, b0, FINGER_CLOSED, "close on the bar", 0.0),
           _hook(0.5, b0, FINGER_CLOSED, "hold", 0.0)]
    for deg in list(range(5, int(HOLD_DEG), 5)) + [HOLD_DEG]:
        out.append(_hook(1.5 if deg == 5 else 0.9, bar_point(deg), FINGER_CLOSED, "pull the hopper open", deg))
    t_open = sum(w.duration for w in out)
    t_left = left_done_time()
    out.append(_hook(max(1.0, t_left - t_open + 1.0), bar_point(HOLD_DEG), FINGER_CLOSED, "hold it open", HOLD_DEG))
    for deg in (35, 25, 15, 8, EASE_DEG):
        out.append(_hook(1.0, bar_point(deg), FINGER_CLOSED, "let it back", deg))
    pe = bar_point(EASE_DEG)
    out += [_hook(0.6, pe, wide, "let go: the springs shut it", EASE_DEG),
            _hook(1.5, pe - (HOOK_LAT + 0.02) * ALONG, wide, "fingers back off the bar's end", EASE_DEG),
            kit.Home(3.0, FINGER_CLOSED)]
    return out


OPEN_BY = 3.5 + 1.5 + 2.0 + 1.0 + 0.5 + 1.5 + 0.9 * 8  # s: the right hand has the hopper open by then


def stack_top(lift: float = 0.0) -> np.ndarray:
    """World tool point pinching the stack's top edge (PINCH_DOWN below it, across its middle) at the start."""
    mids = [env_start(i)[0] for i in range(len(ENVS))]
    mid = np.mean(mids, axis=0)
    u = np.array([math.sin(LEAN), 0.0, math.cos(LEAN)])
    return mid + u * (ENV[2] / 2 - PINCH_DOWN) + np.array([0.0, 0.0, lift])


def door_inner_x(z: float, deg: float = HOLD_DEG) -> float:
    """Box-frame x of the open door's inner face at height z (above the hinge axis), the hopper open `deg`."""
    a = -math.radians(deg)
    h = (z + DOOR_BACK * math.sin(a)) / math.cos(a)  # up the door's inner face from the hinge
    return DOOR_BACK * math.cos(a) + h * math.sin(a)


def env_tool(bx: float, bz: float) -> np.ndarray:
    """World tool point holding the stack leaning out DROP_PITCH, its bottom edge's middle at box-frame (bx, bz)."""
    u = np.array([math.sin(DROP_PITCH), 0.0, math.cos(DROP_PITCH)])
    return np.array([BOX_POS[0] + bx, ENV_Y, BOX_POS[2] + bz]) + u * (ENV[2] - PINCH_DOWN)


def cross_point() -> np.ndarray:
    """Over the open door's top edge, the stack's bottom DOOR_GAP inside it."""
    a = -math.radians(HOLD_DEG)
    tz = -DOOR_BACK * math.sin(a) + DOOR_H * math.cos(a)  # the door's top edge (inner face)
    return env_tool(door_inner_x(tz) + DOOR_GAP, tz + CROSS_UP)


def let_go_point() -> np.ndarray:
    return env_tool(door_inner_x(LET_GO_Z) + DOOR_GAP, LET_GO_Z)


def left_pick() -> list:
    wide = finger_for(0.012)
    p = stack_top()
    return [_down(3.0, p + np.array([0.0, 0.0, 0.06]), wide, "over the letters", LEAN),
            _down(2.0, p, wide, "down round their top edge", LEAN),
            _down(1.0, p, FINGER_CLOSED, "pinch the stack", LEAN),
            _down(0.5, p, FINGER_CLOSED, "hold", LEAN),
            _down(2.0, stack_top(0.08), FINGER_CLOSED, "lift it out of the rack", LEAN)]


def left_done_time() -> float:
    return sum(w.duration for w in left_plan())


def left_plan() -> list:
    pick = left_pick()
    t_pick = sum(w.duration for w in pick)
    wait = max(0.5, OPEN_BY - t_pick + 0.5)
    wide = finger_for(0.006)  # opened a little: the near finger stays clear of the door
    P = DROP_PITCH
    c, g = cross_point(), let_go_point()
    return pick + [_down(wait, stack_top(0.08), FINGER_CLOSED, "wait for the hopper to open", LEAN),
                   _down(3.5, c + np.array([-0.06, 0.0, 0.04]), FINGER_CLOSED, "over the open hopper", P),
                   _down(1.5, c, FINGER_CLOSED, "in over the door's top edge", P),
                   _down(2.0, g, FINGER_CLOSED, "down along the door", P),
                   _down(0.8, g, wide, "let go: the letters slide down into the V", P),
                   _down(1.5, c + np.array([-0.02, 0.0, 0.03]), wide, "back out over the door", P),
                   kit.Home(3.0, FINGER_CLOSED)]


def demo():
    """Right hand: pulls the hopper open by its pull and holds it; left hand: posts the three envelopes as a stack;
    the right hand lets the hopper back and the springs shut it."""
    return {"left": left_plan(), "right": right_plan()}
