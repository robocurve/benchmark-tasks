"""In a living room, on a console table against the wall (the station table), in front of the arms: a 12 in round wall
clock (an Amazon Basics 12 in non-ticking wall clock: 305 mm across, 44 mm deep, 0.64 kg) stands on its rim on the
table, its back to the wall, after its battery was changed. Above it a #8 pan-head wood screw stands 5 mm proud of the
wall, where the clock hung.

The clock hangs by a steel keyhole hanger set flush into its back near the top: a round hole wide enough for the
screw's head, with a narrow slot above it that only the screw's shank fits. The screw is behind the clock, so it can't
be seen: the clock is held flat against the wall, its back pressed on the screw's head and slid down until the head
drops through the wide hole, and slid on down so the shank rides up the slot into its narrow end, where the head
holds the hanger and the clock hangs.

Every part is real contact: the hanger is a 1 mm steel plate with the keyhole cut out of it (convex pieces round the
hole and the slot), the screw is a fixed head and shank on a fixed wall, and the clock is a free rigid body. Nothing
captures, snaps or pulls the clock onto the screw.
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
SUCCESS_HOLD_S = 2.0
SCORE_OVER = "end"
DT = 1.0 / 20
RUN_LIMIT_STEPS = 2400
INSTRUCTION = "Hang the clock on its screw."

# -- the wall and the screw ----------------------------------------------------------------------------------------------
# A painted plywood wall panel standing on the table's back edge, its face at world x XW. The screw: a #8 pan-head wood
# screw (ASME B18.6.1: shank 0.164 in = 4.17 mm, pan head 0.311 in = 7.9 mm across, 0.115 in = 2.9 mm high) driven in
# level so its head's face stands 5 mm proud of the wall (2.1 mm of bare shank, then the head).
XW = 0.425
SCREW_Y = 0.0
SHANK_R = 0.00417 / 2
HEAD_R = 0.0079 / 2
HEAD_H = 0.0029
PROUD = 0.005
SHANK_L = PROUD - HEAD_H
HUNG_Z = 0.200  # the clock's center hangs this high over the table (the screw is placed for it)
WALL_T = 0.018
WALL_Y = (-0.45, 0.45)
WALL_TOP = 0.70
PAINT = (0.45, 0.40, "average")  # latex-painted plywood (ABS on painted wood, dry: about 0.4)
STEEL = (0.45, 0.40, "average")  # zinc-plated steel screw and hanger, dry steel on steel (Engineering ToolBox: 0.5-0.8
# static for clean steel; plated and lightly oiled parts sit lower, 0.45 is used)

# -- the clock (its own frame: origin at the middle of its back face, x out of the back toward the wall, y to its left as
#    the robot sees it, z up when it hangs upright) ----------------------------------------------------------------------
# Listing: 12 in (305 mm) across, 1.75 in (44 mm) deep, 0.64 kg (1.4 lb). The frame ring is 12 mm wide radially and stands
# 12 mm in front of the lens (estimated from maker photos).
R = 0.1525
DEPTH = 0.044
RING_W = 0.012
LENS_RECESS = 0.012
CLOCK_KG = 0.64
CLOCK_COM = (-0.020, 0.0, -0.005)  # the movement and battery sit a little below the middle
CLOCK_I = (0.5 * CLOCK_KG * R * R * 0.8, 0.25 * CLOCK_KG * R * R * 0.8, 0.25 * CLOCK_KG * R * R * 0.8)
N_RIM = 32
APOTHEM = R * math.cos(math.pi / N_RIM)
PLASTIC = (0.40, 0.35, "average")  # ABS
BACK_T = 0.002
# The keyhole hanger (estimated from maker photos of such hangers, sized for a #8 screw): a 1 mm steel plate flush with
# the back, a round hole 11 mm across for the head, a slot 5 mm wide above it (shank 4.17 mm), whose narrow end's
# center is 11 mm above the hole's center. A pocket 6 mm deep behind the plate takes the head.
PLATE_T = 0.001
HOLE_R = 0.0055
SLOT_HW = 0.0025
HOLE_Z = R - 0.046  # hole center, clock frame
SLOT_TOP = HOLE_Z + 0.011  # narrow end's center: where the shank sits when the clock hangs
PLATE_Y = 0.016
PLATE_Z = (HOLE_Z - 0.022, SLOT_TOP + 0.014)
POCKET = 0.006
SCREW_Z = TOP + HUNG_Z + SLOT_TOP  # world height of the screw's axis (the clock hangs with its narrow end on it)

FRAME_LOOK = kit.Look((0.08, 0.08, 0.09), 0.4)
DIAL_LOOK = kit.Look((0.97, 0.97, 0.95), 0.7)
HAND_LOOK = kit.Look((0.05, 0.05, 0.05), 0.5)
BACK_LOOK = kit.Look((0.15, 0.15, 0.16), 0.6)
STEEL_LOOK = kit.Look((0.75, 0.76, 0.78), 0.3, 1.0)
WALL_LOOK = kit.Look((0.90, 0.88, 0.82), 0.8)


def _prism_x(poly, x0, x1, center=(0.0, 0.0, 0.0)):
    """A convex prism along x from a convex (y, z) polygon (counterclockwise): hull points and triangles, relative to
    `center`."""
    n = len(poly)
    c = np.asarray(center, float)
    pts = [tuple(float(v) for v in np.array([x, y, z]) - c) for x in (x0, x1) for (y, z) in poly]
    faces = []
    for i in range(1, n - 1):
        faces.append((0, i + 1, i))
        faces.append((n, n + i, n + i + 1))
    for i in range(n):
        j = (i + 1) % n
        faces += [(i, j, n + j), (i, n + j, n + i)]
    return tuple(pts), tuple(faces)


def _hull(poly, x0, x1, material=None):
    poly = [(float(y), float(z)) for y, z in poly]
    cy = sum(p[0] for p in poly) / len(poly)
    cz = sum(p[1] for p in poly) / len(poly)
    cen = ((x0 + x1) / 2, cy, cz)
    pts, faces = _prism_x(poly, x0, x1, cen)
    return kit.Collider(cen, hull=pts, faces=faces, material=material)


def _hull_mesh(col, look):
    """A visible mesh with a hull collider's own triangles."""
    c = np.asarray(col.center, float)
    pts = [tuple(float(v) for v in np.asarray(p) + c) for p in col.hull]
    idx = [i for f in col.faces for i in f]
    return kit._mesh(pts, [3] * len(col.faces), idx, look)


def _clip(poly, lo, hi):
    """Sutherland-Hodgman clip of a (y, z) polygon to the box lo..hi."""
    out = list(poly)
    for k, bound, keep_ge in ((0, lo[0], True), (0, hi[0], False), (1, lo[1], True), (1, hi[1], False)):
        src, out = out, []
        if not src:
            break
        for i in range(len(src)):
            a, b = src[i], src[(i + 1) % len(src)]
            ina = a[k] >= bound if keep_ge else a[k] <= bound
            inb = b[k] >= bound if keep_ge else b[k] <= bound
            if ina:
                out.append(a)
            if ina != inb:
                t = (bound - a[k]) / (b[k] - a[k])
                out.append(tuple(a[j] + t * (b[j] - a[j]) for j in range(2)))
    return out


_DISC = [(R * math.cos(2 * math.pi * (i + 0.5) / N_RIM), R * math.sin(2 * math.pi * (i + 0.5) / N_RIM)) for i in
         range(N_RIM)]
_DISC_IN = [(0.97 * y, 0.97 * z) for y, z in _DISC]


def _back_pieces():
    """The back panel (x -BACK_T..0) with a rectangular opening where the hanger sits: four convex pieces."""
    big = 1.0
    boxes = [((-big, -big), (big, PLATE_Z[0])), ((-big, PLATE_Z[1]), (big, big)),
             ((-big, PLATE_Z[0]), (-PLATE_Y, PLATE_Z[1])), ((PLATE_Y, PLATE_Z[0]), (big, PLATE_Z[1]))]
    return [_hull(_clip(_DISC_IN, lo, hi), -BACK_T, 0.0) for lo, hi in boxes]


def _plate_pieces():
    """The hanger plate (x -PLATE_T..0) with the keyhole cut out: below the hole, above the slot's end, and bands
    either side whose inner edges are chords of the hole's circle, then the slot's straight sides."""
    x0, x1 = -PLATE_T, 0.0
    out = [_hull([(-PLATE_Y, PLATE_Z[0]), (PLATE_Y, PLATE_Z[0]), (PLATE_Y, HOLE_Z - HOLE_R),
                  (-PLATE_Y, HOLE_Z - HOLE_R)], x0, x1, STEEL)]
    # the slot's rounded end: the plate above z SLOT_TOP + SLOT_HW, and small convex pieces over the arch (chords)
    out.append(_hull([(-PLATE_Y, SLOT_TOP + SLOT_HW), (PLATE_Y, SLOT_TOP + SLOT_HW), (PLATE_Y, PLATE_Z[1]),
                      (-PLATE_Y, PLATE_Z[1])], x0, x1, STEEL))
    for k in range(6):
        a0, a1 = math.pi * k / 6, math.pi * (k + 1) / 6
        p0 = (SLOT_HW * math.cos(a0), SLOT_TOP + SLOT_HW * math.sin(a0))
        p1 = (SLOT_HW * math.cos(a1), SLOT_TOP + SLOT_HW * math.sin(a1))
        out.append(_hull([p0, (p0[0], SLOT_TOP + SLOT_HW), (p1[0], SLOT_TOP + SLOT_HW), p1], x0, x1, STEEL))
    # sides of the hole and the slot
    zs = HOLE_Z + math.sqrt(HOLE_R ** 2 - SLOT_HW ** 2)  # where the slot's sides meet the hole
    angs = np.linspace(-math.pi / 2, math.asin((zs - HOLE_Z) / HOLE_R), 7)
    for s in (-1.0, 1.0):
        for a0, a1 in zip(angs[:-1], angs[1:]):
            q0 = (s * HOLE_R * math.cos(a0), HOLE_Z + HOLE_R * math.sin(a0))
            q1 = (s * HOLE_R * math.cos(a1), HOLE_Z + HOLE_R * math.sin(a1))
            poly = [q0, (s * PLATE_Y, q0[1]), (s * PLATE_Y, q1[1]), q1]
            out.append(_hull(poly if s > 0 else poly[::-1], x0, x1, STEEL))
        poly = [(s * SLOT_HW, zs), (s * PLATE_Y, zs), (s * PLATE_Y, SLOT_TOP), (s * SLOT_HW, SLOT_TOP)]
        out.append(_hull(poly if s > 0 else poly[::-1], x0, x1, STEEL))
    return out


def _pocket():
    """The stamped pocket behind the hanger: a floor POCKET deep and four walls round the plate's opening."""
    y, (z0, z1) = PLATE_Y, PLATE_Z
    f = -POCKET - PLATE_T
    out = [_hull([(-y, z0), (y, z0), (y, z1), (-y, z1)], f - 0.001, f, STEEL)]
    for poly in ([(-y, z0 - 0.001), (y, z0 - 0.001), (y, z0), (-y, z0)], [(-y, z1), (y, z1), (y, z1 + 0.001), (-y, z1 + 0.001)],
                 [(-y - 0.001, z0), (-y, z0), (-y, z1), (-y - 0.001, z1)], [(y, z0), (y + 0.001, z0), (y + 0.001, z1), (y, z1)]):
        out.append(_hull(poly, f, -PLATE_T, STEEL))
    return out


def _rim():
    """The frame ring: N_RIM flat segments, RING_W wide, the full depth."""
    out = []
    for i in range(N_RIM):
        a0, a1 = 2 * math.pi * i / N_RIM, 2 * math.pi * (i + 1) / N_RIM
        ro, ri = R / math.cos(math.pi / N_RIM), (R - RING_W) / math.cos(math.pi / N_RIM)
        poly = [(ri * math.cos(a0), ri * math.sin(a0)), (ro * math.cos(a0), ro * math.sin(a0)),
                (ro * math.cos(a1), ro * math.sin(a1)), (ri * math.cos(a1), ri * math.sin(a1))]
        out.append(_hull(poly, -DEPTH, 0.0))
    return out


_LENS_X = -DEPTH + LENS_RECESS
_RIM = _rim()
_BACK = _back_pieces()
_PLATE = _plate_pieces()
_POCKET = _pocket()
_LENS = _hull([(0.99 * (R - RING_W) * math.cos(a), 0.99 * (R - RING_W) * math.sin(a))
               for a in np.linspace(0, 2 * math.pi, N_RIM, endpoint=False)], _LENS_X, _LENS_X + 0.002)
_CLOCK_COLS = tuple(_RIM + _BACK + _PLATE + _POCKET + [_LENS])
# The face, seen through the lens from the front: a round white dial 8 mm behind the lens, 12 black hour marks on it
# and two hands (10:10) between it and the lens. Looks only: the clock's colliders are the rim, back, hanger and lens.
_DIAL_X = _LENS_X + 0.008
_DIAL = kit.cylinder((_DIAL_X, 0.0, 0.0), 0.995 * (R - RING_W), 0.001, DIAL_LOOK, axis="x", n=64)
_TICKS = tuple(kit.band([(_DIAL_X - 0.001, r * math.cos(a), r * math.sin(a)) for r in (0.116, 0.132)], 0.001, 0.005,
                        HAND_LOOK) for a in np.linspace(0, 2 * math.pi, 12, endpoint=False))
_HANDS = tuple(kit.band([(x, -0.015 * math.cos(a), -0.015 * math.sin(a)), (x, L * math.cos(a), L * math.sin(a))], 0.001,
                        w, HAND_LOOK) for x, L, w, a in ((_DIAL_X - 0.0025, 0.075, 0.008, math.radians(25)),
                                                         (_DIAL_X - 0.0035, 0.112, 0.005, math.radians(150))))
_HUB = kit.cylinder((_DIAL_X - 0.004, 0.0, 0.0), 0.007, 0.001, HAND_LOOK, axis="x", n=24)
CLOCK_MESHES = (tuple(_hull_mesh(c, FRAME_LOOK) for c in _RIM) + tuple(_hull_mesh(c, BACK_LOOK) for c in _BACK)
                + tuple(_hull_mesh(c, STEEL_LOOK) for c in _PLATE + _POCKET) + (_DIAL, _HUB) + _TICKS + _HANDS)

# Start: standing on its rim on the table, upright, back 3 mm off the wall, under the screw.
START_GAP = 0.003
CLOCK_START = (XW - START_GAP, SCREW_Y, TOP + APOTHEM + 0.0002)
CLOCK = kit.Prop(
    "clock", pos=CLOCK_START, mass=CLOCK_KG, com=CLOCK_COM, inertia=CLOCK_I,
    size=(DEPTH, 2 * R, 2 * R), meshes=CLOCK_MESHES, colliders=_CLOCK_COLS, material=PLASTIC,
    contact_offset=0.0004, solver_iterations=(32, 8), max_depenetration=0.2)

WALL = kit.Prop(
    "wall", pos=(XW, SCREW_Y, TOP), mass=None, size=(WALL_T + PROUD, WALL_Y[1] - WALL_Y[0], WALL_TOP),
    meshes=(kit.box((WALL_T / 2, 0.0, WALL_TOP / 2), (WALL_T, WALL_Y[1] - WALL_Y[0], WALL_TOP), WALL_LOOK),
            kit.cylinder((-SHANK_L / 2, 0.0, SCREW_Z - TOP), SHANK_R, SHANK_L, STEEL_LOOK, axis="x", n=16),
            kit.cylinder((-SHANK_L - HEAD_H / 2, 0.0, SCREW_Z - TOP), HEAD_R, HEAD_H, STEEL_LOOK, axis="x", n=24)),
    colliders=(kit.Collider((WALL_T / 2, 0.0, WALL_TOP / 2), size=(WALL_T, WALL_Y[1] - WALL_Y[0], WALL_TOP),
                            material=PAINT),
               kit.Collider((-SHANK_L / 2, 0.0, SCREW_Z - TOP), radius=SHANK_R, height=SHANK_L, axis="X",
                            material=STEEL),
               kit.Collider((-SHANK_L - HEAD_H / 2, 0.0, SCREW_Z - TOP), radius=HEAD_R, height=HEAD_H, axis="X",
                            material=STEEL)),
    material=PAINT, contact_offset=0.0004)

OBJECTS = [WALL, CLOCK]
gripkit.use_rubber_pads()

# -- scored thresholds ------------------------------------------------------------------------------------------------------
HEAD_IN = -PLATE_T  # the head's middle deeper than the plate's front face (clock x): the head is behind the hanger
NARROW_Z = SLOT_TOP - 0.003  # the shank within 3 mm of the narrow end's center (the slot's sides keep it centered)
OFF_TABLE = 0.010  # m: hung, the clock's lowest point at least 1 cm over the table (not standing on it)
COVER_GAP = 0.010  # m: 0.25: the clock's back within 1 cm of the wall over the screw
FALL_DEG = 30.0  # the clock's face tipped more than 30 degrees from the wall's plane with no arm touching it: it fell
# (latched); tilting it while holding it is allowed
CLEAR = 0.005
TOUCH = 0.001  # an arm link this close touches the clock (it is held)
STILL_STEP = 0.0005
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)
ARM_LINKS = ("link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
ROBOTS = ("left_robot", "right_robot")
LINK_POSES: dict = {}


def rotm(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def screw_in_clock(objects) -> dict:
    """The screw seen from the clock: the head's middle and the shank's middle in the clock's frame."""
    c = objects["clock"]
    o, Rc = np.asarray(c.pos, float), rotm(c.quat_xyzw)
    head = Rc.T @ (np.array([XW - SHANK_L - HEAD_H / 2, SCREW_Y, SCREW_Z]) - o)
    shank = Rc.T @ (np.array([XW - SHANK_L / 2, SCREW_Y, SCREW_Z]) - o)
    return {"head": head, "shank": shank}


def head_behind(objects) -> bool:
    """The screw's head through the hanger: its middle deeper into the clock than the plate's front face, inside the
    keyhole's outline (the hole or the slot)."""
    s = screw_in_clock(objects)
    h = s["head"]
    in_hole = math.hypot(h[1], h[2] - HOLE_Z) <= HOLE_R
    in_slot = abs(h[1]) <= SLOT_HW + 0.0005 and HOLE_Z <= h[2] <= SLOT_TOP + SLOT_HW
    return h[0] < HEAD_IN and h[0] > -PLATE_T - POCKET - 0.002 and (in_hole or in_slot)


def in_narrow_end(objects) -> bool:
    """Head behind the hanger and the shank up the slot in its narrow end."""
    sh = screw_in_clock(objects)["shank"]
    return head_behind(objects) and abs(sh[1]) <= SLOT_HW and sh[2] >= NARROW_Z


def face_tilt(objects) -> float:
    """Degrees between the clock's face and the wall's (vertical) plane: how far its back axis points off level."""
    a = rotm(objects["clock"].quat_xyzw)[:, 0]
    return math.degrees(math.asin(min(1.0, abs(float(a[2])))))


def low_z(objects) -> float:
    c = objects["clock"]
    Rc = rotm(c.quat_xyzw)
    ang = np.linspace(0, 2 * math.pi, 48, endpoint=False)
    ring = np.stack([np.zeros_like(ang), R * np.cos(ang), R * np.sin(ang)], 1)
    pts = np.concatenate([ring, ring + np.array([-DEPTH, 0, 0])])
    return float((np.asarray(c.pos, float) + pts @ Rc.T)[:, 2].min()) - TOP


def covers_screw(objects) -> bool:
    """The clock's back within COVER_GAP of the wall, lifted, over the screw (the screw well inside its outline)."""
    sh = screw_in_clock(objects)["shank"]
    return sh[0] > -0.002 - PLATE_T - POCKET and sh[0] <= COVER_GAP + SHANK_L and math.hypot(sh[1], sh[2]) < R - 0.02 \
        and low_z(objects) > OFF_TABLE


def _seg_gap(q, a, b, r):
    a, b = np.asarray(a, float), np.asarray(b, float)
    L = float(np.linalg.norm(b - a))
    u = (b - a) / L
    d = q - a
    s = d @ u
    radial = np.linalg.norm(d - s[:, None] * u, axis=1)
    return np.hypot(np.maximum(radial - r, 0.0), np.maximum(np.maximum(-s, s - L), 0.0))


def clock_gap(qc):
    return _seg_gap(qc, (-DEPTH, 0, 0), (0, 0, 0), R)


def link_gap(objects, key) -> float:
    o, Rl = LINK_POSES[key]
    c = objects["clock"]
    co, Rc = np.asarray(c.pos, float), rotm(c.quat_xyzw)
    link = key.split("/")[1]
    _, cen, rad = yg.cloud(link)
    o_, R_ = np.asarray(o, float), np.asarray(Rl, float)
    if float(np.linalg.norm(o_ + R_ @ cen - co)) - rad > R + 0.05:
        return 0.02
    q = yg.near(link, lambda x: (o_ + x @ R_.T - co) @ Rc, clock_gap, 0.02)
    return min(0.02, float(clock_gap(q).min())) if len(q) else 0.02


def arm_gap(objects) -> float:
    return min((link_gap(objects, k) for k in LINK_POSES), default=0.02)


class Ledger:
    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.fell = False
        self.lifted = False
        self.last = None
        self.good = 0
        self.n = 0
        self.state = {}

    def step(self, objects) -> None:
        self.n += 1
        c = objects["clock"]
        p = np.asarray(c.pos, float)
        gap = arm_gap(objects)
        tilt = face_tilt(objects)
        if tilt > FALL_DEG and gap > TOUCH:
            self.fell = True
        hung = in_narrow_end(objects) and low_z(objects) > OFF_TABLE
        through = head_behind(objects)
        cover = covers_screw(objects)
        clear = gap > CLEAR
        still = self.last is not None and float(np.linalg.norm(p - self.last)) <= STILL_STEP
        self.last = p
        self.state = dict(hung=hung, through=through, cover=cover, clear=clear, still=still)
        self.good = self.good + 1 if (hung and clear and still and not self.fell) else 0

    def score(self) -> float:
        if self.fell:
            return 0.0
        if self.good >= HOLD_STEPS:
            return 1.0
        st = self.state
        if st.get("through"):
            return 0.5
        if st.get("cover"):
            return 0.25
        return 0.0


_L = None


def _read_links(base) -> None:
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    for robot in ROBOTS:
        art = base.scene[robot]
        names = list(art.body_names)
        P = getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
        Q = getattr(art.data.body_quat_w, "torch", art.data.body_quat_w)[0].detach().cpu().numpy().astype(float)
        missing = [link for link in ARM_LINKS if link not in names]
        if missing:
            raise RuntimeError(f"clock: {robot} has no link(s) {missing} for the contact checks")
        for link in ARM_LINKS:
            i = names.index(link)
            LINK_POSES[f"{robot}/{link}"] = (P[i] - origin, rotm(Q[i]))
    if gripkit._STATE.get("bound", 0) < 0:
        raise RuntimeError("clock: the finger pads' rubber material could not be bound")


kit.STATE_HOOKS.append(_read_links)
if "isaaclab" in sys.modules:
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
    """The clock hangs on the screw by its keyhole's narrow end, off the table, no arm link within 5 mm, still."""
    _update(objects)
    return _L.good >= 1


def score(objects):
    """1.0: hung (the screw's head behind the hanger, the shank in the slot's narrow end, the clock off the table) with
    every arm link at least 5 mm clear and the clock still (under 1 cm/s) for 2 s. 0.5: the head through the hanger
    (in the hole or the slot) at the end, but not hung clear for 2 s. 0.25: the clock lifted with its back within 1 cm
    of the wall over the screw. 0: anything else, or the clock's face tipped more than 30 degrees off the wall's plane
    while no arm link touches it, at any moment (it fell; latched)."""
    _update(objects)
    return _L.score()


# -- scripted demo: open loop from the fixed layout -------------------------------------------------------------------------
# Both hands point forward and grip the frame ring at 3 and 9 o'clock (one pad on the rim's outside, one inside the ring
# over the lens). They lift the clock off the wall a little, raise it until the hanger's hole is below the screw head's
# height, press its back onto the head and slide it straight down: the head drops through the hole into the pocket and
# the shank rides up the slot to its narrow end. Then they let go and back away.
PITCH_FWD = -math.pi / 2
FINGER_HALF_OPEN = 0.047
TIP_PAST = 0.013
GRIP_IN = 0.010  # the inner pad reaches 10 mm past the frame's front, into the recess over the lens
SQUEEZE = 0.002
PRESS = 0.001  # once the head is in the hole, the clock's back is aimed this far past the wall (presses it flat)
LIGHT = 0.002  # while finding the hole, the back is aimed this far past the head's face (pressing hard wedges the
# clock between the head and the wall and the grip slips)
SAG = 0.022  # m: the hands are aimed this much higher (the arms sag under the 0.64 kg clock)
RING_MID = R - RING_W / 2


def finger_for(half: float) -> float:
    return FINGER_CLOSED + (FINGER_OPEN - FINGER_CLOSED) * min(FINGER_HALF_OPEN, max(0.0, half)) / FINGER_HALF_OPEN


def _tw(d, pos, finger, label):
    return kit.Tilted(float(d), tuple(float(v) for v in pos), float(finger), label, pitch=PITCH_FWD)


def tcp_for(origin, side: float, back: float = 0.0) -> tuple:
    """The tool point gripping the ring at y = side * RING_MID of a clock whose frame origin is at `origin`."""
    ox, oy, oz = origin
    return (ox - DEPTH + GRIP_IN - TIP_PAST - back, oy + side * RING_MID, oz + SAG)


def demo():
    """CLOCK_VARIANT (cheat and partial modes for the failure table): "" the full demo; "nothing" both arms stay home
    (0); "hold" hangs it but never lets go (0.5); "partial" presses it flat with the head in the hole's wide end and
    holds it there, never sliding it down onto the shank (0.5); "miss" aims 25 mm to the side, so the head is beside the
    hole, slides it down and lets go: it drops (0)."""
    variant = os.environ.get("CLOCK_VARIANT", "")
    if variant == "nothing":
        return {"left": [kit.Home(10.0, label="stay")], "right": [kit.Home(10.0, label="stay")]}
    dy = 0.025 if variant == "miss" else 0.0
    opn, hold = finger_for(0.020), finger_for(RING_W / 2 - SQUEEZE)
    x0 = CLOCK_START[0]
    z0 = CLOCK_START[2]
    z_up = SCREW_Z - (HOLE_Z - 0.008)  # clock origin height with the head 8 mm under the hole's center
    z_end = SCREW_Z - SLOT_TOP - 0.006  # 6 mm past hung: the slot's end stops it, the arms' give takes the rest
    xa = XW - 0.015  # back clear of the head
    xp = XW + PRESS
    xl = XW - PROUD + LIGHT  # the back 2 mm past the head's face: a light press while it finds the hole
    z_hole = SCREW_Z - HOLE_Z  # the head level with the hole's center
    plans = {}
    for name, side in (("left", 1.0), ("right", -1.0)):
        o = lambda x, z: (x, SCREW_Y + dy, z)  # noqa: E731
        g0 = tcp_for((x0, SCREW_Y, z0), side)
        seq = [kit.Home(0.5, label="start"),
               _tw(3.0, (g0[0] - 0.06, g0[1], g0[2] + 0.02), opn, "in front of the ring"),
               _tw(2.0, (g0[0] - 0.03, g0[1], g0[2] - SAG), opn, "level with the ring's side"),
               _tw(2.0, (g0[0], g0[1], g0[2] - SAG), opn, "fingers either side of the ring"),
               _tw(1.5, (g0[0], g0[1], g0[2] - SAG), hold, "grip the ring"),
               _tw(1.0, (g0[0], g0[1], g0[2] - SAG / 2), hold, "take its weight"),
               _tw(2.0, tcp_for((XW - 0.020, SCREW_Y, z0 + 0.003), side), hold, "draw it off the wall, clear of the screw"),
               _tw(4.0, tcp_for(o(xa, z_up), side), hold, "raise it, hanger under the screw"),
               _tw(2.0, tcp_for(o(xl, z_up), side), hold, "set its back lightly on the screw's head"),
               _tw(3.0, tcp_for(o(xl, z_hole), side), hold, "slide it down until the head drops into the hole"),
               _tw(1.5, tcp_for(o(xp, z_hole), side), hold, "press it flat to the wall")]
        if variant == "partial":
            plans[name] = seq + [_tw(8.0, tcp_for(o(xp, z_hole), side), hold, "hold it with the head in the hole")]
            continue
        seq.append(_tw(4.0, tcp_for(o(xp, z_end), side), hold, "slide it down onto the shank"))
        if variant == "hold":
            seq.append(_tw(8.0, tcp_for(o(xp, z_end), side), hold, "hold it on the screw"))
        else:
            e = tcp_for(o(xp, z_end), side)
            seq += [_tw(1.0, e, hold, "settle"),
                    _tw(1.5, e, opn, "let go"),
                    _tw(2.5, (e[0] - 0.06, e[1], e[2]), opn, "back away"),
                    kit.Home(4.0, label="home"),
                    kit.Home(3.0, label="wait")]
        plans[name] = seq
    return plans
