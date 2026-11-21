"""A medium binder clip (32 mm wide, 16 mm capacity: black spring-steel body, two silver wire handles) stands on its
closed jaws on the desk with both handles up. A small mesh letter sorter (178 x 83 x 133 mm, three upright slots)
stands behind it; a loose stack of 40 sheets of A5 copy paper (80 g/m2, 148 x 210 mm, 4 mm, 100 g) stands on its
long edge in the front slot, leaning back on the first divider, its right end 50 mm past the sorter. Squeezing the
clip's two handles together levers its jaw open; let go, the spring shuts it. Each handle turns in its rolled lip
and folds down flat against the paper. Fixed layout; 20 Hz.
"""
import math
import os
import pathlib
import sys
from dataclasses import replace

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, box, cyl
from yam_joints import Hinge, Link
from yam_mesh import Mesh, MeshMechanism, MeshPart
from yam_arena import demo as _motion
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
DIAG = False  # True prints the demo's pose reads
# Demo variants for wrong-way copies: "" = the demo, "early" = let go with the paper only 5 mm into the jaw.
DEMO_WAY = os.environ.get("CLIP_DEMO_WAY", "")

RELEASE_CLEAR = 0.030
STILL_S = 1.0
TEST_GRIPPERS = None


# ------------------------------------------------------------------ run context, release and stillness
def _run_context():
    context = {"env": None, "t": None, "step": 0, "episode": 0}
    try:
        from yam_arena import inspect_embodiment as _ie
    except ImportError:
        return context
    cls = _ie.YamArenaEmbodiment
    if hasattr(cls, "_rc_run"):
        return cls._rc_run
    original_step, original_reset = cls._task_step, cls.reset

    def task_step(self, *args, **kwargs):
        context["env"], context["t"] = self._env, self._t
        context["step"] += 1
        return original_step(self, *args, **kwargs)

    def reset(self, *args, **kwargs):
        result = original_reset(self, *args, **kwargs)
        context["episode"] += 1
        context["t"] = 0.0
        context["env"] = self._env
        return result

    task_step._rc_wrapped = reset._rc_wrapped = True
    cls._rc_run = context
    cls._task_step, cls.reset = task_step, reset
    return context


_RUN = _run_context()
_MEMORY_EPISODE = None
_TIMERS = {}
_EVER = {}
_ANCHORS = {}


def _arm_state(arm):
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, LEFT_BASE_POS, RIGHT_BASE_POS
    env = _RUN["env"]
    bases = station.BASES if SCENE == "station" else {"left": LEFT_BASE_POS, "right": RIGHT_BASE_POS}
    art = env.unwrapped.scene[arm + "_robot"]
    arm_ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
    finger_ids = art.find_joints(FINGER_JOINTS, preserve_order=True)[0]
    joints = art.data.joint_pos
    q = getattr(joints, "torch", joints)[0]
    if hasattr(q, "detach"):
        q = q.detach().cpu().numpy()
    q = np.asarray(q)
    p, R = ik.tcp_pose(q[arm_ids], bases[arm])
    return np.asarray(p, float), np.asarray(R, float), float(np.mean(q[finger_ids]))


def grippers():
    if TEST_GRIPPERS is not None:
        return TEST_GRIPPERS
    if _RUN["env"] is None:
        return None
    out = {}
    for arm in ("left", "right"):
        p, _, finger = _arm_state(arm)
        out[arm] = (p, float(np.clip(finger / FINGER_OPEN, 0.0, 1.0)))
    return out


def released(state, clear=RELEASE_CLEAR):
    hands = grippers()
    if hands is None:
        return True  # Offline arm poses are unknown, so release is assumed.
    low, high = np.asarray(state.low), np.asarray(state.high)
    return all(np.linalg.norm(np.maximum(np.maximum(low - tcp, tcp - high), 0.0)) >= clear
               for tcp, _ in hands.values())


def _episode_memory():
    global _MEMORY_EPISODE
    if _MEMORY_EPISODE != _RUN["episode"]:
        _TIMERS.clear()
        _EVER.clear()
        _ANCHORS.clear()
        _MEMORY_EPISODE = _RUN["episode"]


def held_for(key, cond):
    """Elapsed time over consecutive observed control steps; repeat calls are harmless."""
    _episode_memory()
    t, step = _RUN["t"], _RUN["step"]
    if t is None:
        return math.inf if cond else 0.0
    previous_step, start = _TIMERS.get(key, (None, None))
    if not cond:
        _TIMERS[key] = (step, None)
        return 0.0
    if start is None or step not in (previous_step, previous_step + 1) or t < start:
        start = t
    _TIMERS[key] = (step, start)
    return float(t - start)


def ever(key, cond):
    """Remember a satisfied condition until the next episode; offline each call stands alone."""
    if _RUN["t"] is None:
        return bool(cond)
    _episode_memory()
    _EVER[key] = bool(_EVER.get(key, False) or cond)
    return _EVER[key]


# ------------------------------------------------------------------ geometry helpers
def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array(((1, 0, 0), (0, c, -s), (0, s, c)))


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array(((c, 0, s), (0, 1, 0), (-s, 0, c)))


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array(((c, -s, 0), (s, c, 0), (0, 0, 1)))


def _box_mesh_from_corners(name, corners, color, proxy=()):
    """8 corners ordered (sx, sy, sz) in (-1, 1)^3 -> a closed convex box mesh."""
    p = np.asarray(corners, float)
    idx = lambda sx, sy, sz: (sx > 0) * 4 + (sy > 0) * 2 + (sz > 0)
    quads = [((-1, -1, -1), (-1, -1, 1), (-1, 1, 1), (-1, 1, -1)), ((1, -1, -1), (1, 1, -1), (1, 1, 1), (1, -1, 1)),
             ((-1, -1, -1), (1, -1, -1), (1, -1, 1), (-1, -1, 1)), ((-1, 1, -1), (-1, 1, 1), (1, 1, 1), (1, 1, -1)),
             ((-1, -1, -1), (-1, 1, -1), (1, 1, -1), (1, -1, -1)), ((-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1))]
    faces = []
    for q in quads:
        a, b, c, d = (idx(*v) for v in q)
        faces += [(a, b, c), (a, c, d)]
    faces = np.array(faces, int)
    tri = p[faces]
    if np.einsum('ij,ij->i', tri[:, 0] - p.mean(0), np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])).sum() < 0:
        faces = faces[:, ::-1]
    return Mesh(name, p, faces, collision="convex", color=color, proxy=proxy)


def _bar(name, p0, p1, w, normal, color, proxy=()):
    """A straight square-section wire w x w from p0 to p1, one pair of faces normal to `normal`."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    ex = (p1 - p0) / np.linalg.norm(p1 - p0)
    ez = np.asarray(normal, float) - ex * float(np.dot(normal, ex))
    ez /= np.linalg.norm(ez)
    ey = np.cross(ez, ex)
    corners = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                base = p1 if sx > 0 else p0
                corners.append(base + ex * sx * w / 2 + ey * sy * w / 2 + ez * sz * w / 2)
    return _box_mesh_from_corners(name, corners, color, proxy)


def _plate(name, a, b, half_len, t, color, proxy=()):
    """A flat plate from line a to line b (points in the (y, z) plane of K), half_len along K x, thickness t."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = (b - a) / np.linalg.norm(b - a)
    n = np.array([d[1], -d[0]]) * t / 2
    corners = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                yz = (b if sy > 0 else a) + n * sz
                corners.append((sx * half_len, yz[0], yz[1]))
    return _box_mesh_from_corners(name, corners, color, proxy)


# ------------------------------------------------------------------ layout (metres, world frame)
TOP = TABLE_TOP[SCENE]
MM = 0.001

# Letter sorter (fixed): VANRA small letter sorter, 178 x 83 x 133 mm, three upright slots running left to right.
# Front wall F nearest the robot, dividers D1 and D2, back wall W; mesh panels as 3 mm solid panels (estimate).
RACK_X0 = 0.185
RACK_Y = (-0.080, 0.098)
WALL_T = 0.003
WALL_X = (RACK_X0, RACK_X0 + 0.027, RACK_X0 + 0.054, RACK_X0 + 0.080)       # front faces of F, D1, D2, W
WALL_H = (0.060, 0.085, 0.110, 0.133)
BASE_T = 0.003
FLOOR = TOP + BASE_T
RACK_COLOR = (0.12, 0.12, 0.13)
_rack_len = RACK_Y[1] - RACK_Y[0]
_rack_cy = (RACK_Y[0] + RACK_Y[1]) / 2
RACK_POS = (RACK_X0, _rack_cy, TOP)
RACK = Part("letter_sorter", RACK_POS,
            [box((0.0415, 0.0, BASE_T / 2), (0.083, _rack_len, BASE_T))]
            + [box((wx - RACK_X0 + WALL_T / 2, 0.0, h / 2), (WALL_T, _rack_len, h)) for wx, h in zip(WALL_X, WALL_H)],
            mass=None, color=RACK_COLOR, friction=(0.4, 0.35), contact_offset=0.001)

# Paper: 40 sheets of 80 g/m2 A5 copy paper (0.1 mm and 2.49 g a sheet, estimates), simulated as ONE rigid 4 mm block.
# Single sheets, their bending and their slip on one another are not modelled.
SHEETS = 40
STACK_T = 0.0040
PAPER_H, PAPER_L = 0.148, 0.210                   # A5 standing on its long edge: 148 tall, 210 long (along y)
STACK_MASS = SHEETS * 0.00249
PAPER_Y = (-0.130, 0.080)
PAPER_COLOR = (0.96, 0.96, 0.93)
PAPER_FRICTION = (0.5, 0.4)
FRONT_GAP = 0.0002
STACK_NAME = "paper_stack"


def _solve_lean():
    """Lean angle so the stack's back face rests on D1's top front edge with its foot on the slot floor."""
    cx, cz = WALL_X[1], TOP + WALL_H[1]
    x_front = WALL_X[0] + WALL_T + FRONT_GAP
    lo, hi = 0.0, 0.6
    for _ in range(80):
        b = (lo + hi) / 2
        x_back_at_c = x_front + STACK_T / math.cos(b) + (cz - FLOOR) * math.tan(b)
        if x_back_at_c < cx:
            lo = b
        else:
            hi = b
    return (lo + hi) / 2


LEAN = _solve_lean()
R_LEAN = _ry(LEAN)          # stack axes (x through the stack, y along its length, z up the sheets) in the part frame
U_UP = R_LEAN[:, 2]
N_BACK = R_LEAN[:, 0]
X_FRONT_FOOT = WALL_X[0] + WALL_T + FRONT_GAP


def stack_center():
    back_foot = np.array([X_FRONT_FOOT + STACK_T / math.cos(LEAN), 0.0, FLOOR])
    return back_foot - N_BACK * (STACK_T / 2) + U_UP * (PAPER_H / 2) + np.array([0.0, (PAPER_Y[0] + PAPER_Y[1]) / 2, 0.0])


def _stack_proxy():
    """Offline proxy: a staircase of thin axis-aligned boxes along the leaning stack."""
    steps, out = 12, []
    for i in range(steps):
        s0 = -PAPER_H / 2 + PAPER_H * i / steps
        s1 = s0 + PAPER_H / steps
        pieces = 7 if i >= steps - 3 else 1
        for j in range(pieces):
            y = -PAPER_L / 2 + PAPER_L * (j + 0.5) / pieces
            c = U_UP * ((s0 + s1) / 2) + np.array([0.0, y, 0.0])
            out.append(box(tuple(c), (STACK_T * math.cos(LEAN), PAPER_L / pieces, (s1 - s0) * math.cos(LEAN))))
    return tuple(out)


TILE_ROWS, TILE_COLS = 2, 2   # four convex tiles (105 x 74 x 4 mm): GPU PhysX cooks convex hulls only up to ~100:1


def _stack():
    meshes = []
    proxy = _stack_proxy()
    for r in range(TILE_ROWS):
        for c in range(TILE_COLS):
            z0, z1 = (-PAPER_H / 2 + PAPER_H * (r + e) / TILE_ROWS for e in (0, 1))
            y0, y1 = (-PAPER_L / 2 + PAPER_L * (c + e) / TILE_COLS for e in (0, 1))
            pts = [R_LEAN @ np.array((sx * STACK_T / 2, y1 if sy > 0 else y0, z1 if sz > 0 else z0))
                   for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
            meshes.append(_box_mesh_from_corners(f"paper_{r}{c}", pts, PAPER_COLOR,
                                                 proxy=proxy if (r, c) == (0, 0) else ()))
    return MeshPart(STACK_NAME, tuple(float(v) for v in stack_center()), (), mass=STACK_MASS, color=PAPER_COLOR,
                    friction=PAPER_FRICTION, contact_offset=0.002, meshes=tuple(meshes))


STACK = _stack()

# ------------------------------------------------------------------ the binder clip (medium, 32 mm, 16 mm capacity)
# Clip frame K (= part frame, yawed 90 deg so K x, the jaw line, runs along world y): x along the jaw line, y across
# the jaw (jaw_l on +y), z from the lips' centre line (z = 0) up to the spine (back plate).
CLIP_W = 0.032                 # published: 1-1/4 in wide
CAPACITY = 0.016               # published: 5/8 in capacity
STEEL_T = 0.0004               # estimate
BACK_W = 0.017                 # spine outer width (estimate)
JAW_H = 0.0195                 # lips' centre line to the spine's inner face (estimate)
CURL_R = 0.0013                # rolled lip outer radius (estimate)
CURL_Y = CURL_R                # closed: the two rolled lips touch on the centre plane
CORNER_Y = BACK_W / 2 - STEEL_T / 2
WIRE = 0.0014                  # handle wire (estimate), modelled square for flat pad contact
HANDLE_L = 0.037               # handle: hook in the lip to the loop's end (estimate, ~1.15 x the clip width)
_wall = np.array([CORNER_Y - CURL_Y, JAW_H])
WALL_L = float(np.linalg.norm(_wall))
_wall_dir = _wall / WALL_L
_wall_out = np.array([_wall_dir[1], -_wall_dir[0]])
# Raised, a handle's legs bear on the outside of the spine corner: the handle continues the line of the jaw wall.
_bear = np.array([CORNER_Y, JAW_H]) + (STEEL_T / 2 + WIRE / 2) * _wall_out
HANDLE_TILT = math.atan2(_bear[0] - CURL_Y, _bear[1])        # outward from K z, about 21.6 deg
_h_dir = np.array([math.sin(HANDLE_TILT), math.cos(HANDLE_TILT)])
# The loop (u along the handle from the hook, v along the jaw line), in mm: legs, waist, bulb, rounded end.
LOOP = ((8.0, 0.0), (4.0, 22.0), (6.5, 30.0), (5.0, 35.6), (3.0, 37.0))
HANDLE_TIP = np.array([CURL_Y, 0.0]) + HANDLE_L * _h_dir    # (y, z) of the loop's end, +y handle, q = 0

# Spring (estimates): lips pressed together at 3 N when empty; about 1 N more per mm of jaw opening, so the jaw
# clamps the 4 mm stack at about 7 N and opening it to its 16 mm capacity takes about 19.5 N at the lips
# (about 25 N at the handle tips). K_JAW is the torque about the spine corner per radian of jaw-wall turn.
PRELOAD_N = 3.0
FULL_LIP_N = 19.5


def _lip(a):
    """(y, z) of the +y lip's centre with the +y jaw wall turned a (rad, opening) about the spine corner."""
    rel = np.array([CURL_Y - CORNER_Y, -JAW_H])
    c, s = math.cos(a), math.sin(a)
    return np.array([CORNER_Y + rel[0] * c - rel[1] * s, JAW_H + rel[0] * s + rel[1] * c])


def jaw_angle_for_gap(gap):
    """Jaw hinge angle (deg, +y jaw) that puts the rolled lip's inner surface gap/2 off the centre plane."""
    lo, hi = 0.0, math.radians(40.0)
    for _ in range(60):
        a = (lo + hi) / 2
        if _lip(a)[0] - CURL_R < gap / 2:
            lo = a
        else:
            hi = a
    return math.degrees((lo + hi) / 2)


JAW_MAX_DEG = jaw_angle_for_gap(CAPACITY)                    # full open: the jaw at its 16 mm capacity
_a_full = math.radians(JAW_MAX_DEG)
_arm0 = JAW_H                                               # lever of a sideways lip force about the corner, shut
_arm_full = float(JAW_H - _lip(_a_full)[1])
_TAU0 = PRELOAD_N * _arm0
K_JAW = (FULL_LIP_N * _arm_full - _TAU0) / _a_full           # N m / rad
JAW_REST_DEG = -math.degrees(_TAU0 / K_JAW)
HANDLE_FOLD_DEG = 178.0
HANDLE_FRICTION = (6.0e-3, 5.0e-3)       # N m, the sprung wire hooks turning in the lips: a handle stays where it is put
JAW_DAMPING = 5e-3                       # N m s / rad on each jaw hinge: damps contact chatter of the clamped jaw (solver)
JAW_ARMATURE = 2e-5                      # kg m2 added inertia on each jaw hinge (solver setting, not a physical claim)

M_BACK, M_WALL, M_HANDLE = 0.0016, 0.0022, 0.0010            # about 8 g in all (estimate)
CLIP_BLACK = (0.05, 0.05, 0.06)
WIRE_SILVER = (0.80, 0.81, 0.83)
CLIP_FRICTION = (0.35, 0.3)


def tip_force_full():
    """Estimated squeeze at the handle tips to hold the jaw at its capacity (handles parallel)."""
    tau = K_JAW * (_a_full - math.radians(JAW_REST_DEG))
    tip = np.array([_bear[0], _bear[1]]) + (HANDLE_L - float(np.linalg.norm(_bear - [CURL_Y, 0.0]))) * _h_dir
    rel = tip - np.array([CORNER_Y, JAW_H])
    c, s = math.cos(_a_full), math.sin(_a_full)
    rz = rel[0] * s + rel[1] * c
    return tau / rz


def _lip_roll(name, side):
    """The rolled lip as a convex octagonal prism along K x (radius CURL_R), one flat face toward the paper."""
    s = 1.0 if side > 0 else -1.0
    prof = [(s * CURL_Y + CURL_R * math.cos(math.pi / 8 + k * math.pi / 4), CURL_R * math.sin(math.pi / 8 + k * math.pi / 4))
            for k in range(8)]
    corners_x = (-CLIP_W / 2, CLIP_W / 2)
    pts = np.array([(x, y, z) for x in corners_x for (y, z) in prof], float)
    n = len(prof)
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces += [(i, j, n + j), (i, n + j, n + i)]
    for k in range(1, n - 1):
        faces += [(0, k + 1, k), (n, n + k, n + k + 1)]
    faces = np.array(faces, int)
    tri = pts[faces]
    if np.einsum('ij,ij->i', tri[:, 0] - pts.mean(0), np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])).sum() < 0:
        faces = faces[:, ::-1]
    return Mesh(name, pts, faces, collision="convex", color=CLIP_BLACK,
                proxy=(box((0.0, s * CURL_Y, 0.0), (CLIP_W, 2 * CURL_R, 2 * CURL_R)),))


def _jaw_meshes(side):
    s = 1.0 if side > 0 else -1.0
    a = np.array([CURL_Y, 0.0]) + CURL_R * _wall_dir
    b = np.array([CORNER_Y, JAW_H])
    a, b = np.array([s * a[0], a[1]]), np.array([s * b[0], b[1]])
    nm = "l" if s > 0 else "r"
    wall = _plate(f"wall_{nm}", a, b, CLIP_W / 2, STEEL_T, CLIP_BLACK,
                  proxy=(box((0.0, s * (CURL_Y + CORNER_Y) / 2, JAW_H / 2), (CLIP_W, CORNER_Y - CURL_Y, JAW_H * 0.6)),))
    return (wall, _lip_roll(f"lip_{nm}", side))


def _handle_meshes(side):
    s = 1.0 if side > 0 else -1.0
    piv = np.array([0.0, s * CURL_Y, 0.0])
    d = np.array([0.0, s * _h_dir[0], _h_dir[1]])
    n = np.cross(np.array([1.0, 0.0, 0.0]), d)                  # normal of the handle's plane
    pts = [(piv + np.array([v * MM, 0.0, 0.0]) + d * (u * MM)) for v, u in LOOP]
    nm = "l" if s > 0 else "r"
    bars = []
    for i in range(len(LOOP) - 1):
        for sv in (-1, 1):
            p0, p1 = pts[i].copy(), pts[i + 1].copy()
            p0[0] *= sv
            p1[0] *= sv
            bars.append(_bar(f"wire_{nm}{i}{'a' if sv < 0 else 'b'}", p0, p1, WIRE, n, WIRE_SILVER))
    e0, e1 = pts[-1].copy(), pts[-1].copy()
    e0[0] = -e0[0]
    bars.append(_bar(f"wire_{nm}end", e0, e1, WIRE, n, WIRE_SILVER))
    return tuple(bars)


def _clip_links():
    links = []
    for side, name in ((1, "l"), (-1, "r")):
        s = float(side)
        jaw = Link(f"jaw_{name}",
                   [box((0.0, s * CURL_Y, 0.0), (CLIP_W - 0.002, CURL_R, CURL_R))],   # inside the convex roll
                   Hinge("x", (0.0, s * CORNER_Y, JAW_H),
                         0.0 if s > 0 else -JAW_MAX_DEG, JAW_MAX_DEG if s > 0 else 0.0,
                         damping=JAW_DAMPING, spring=K_JAW, rest=s * JAW_REST_DEG, initial=0.0, armature=JAW_ARMATURE),
                   mass=M_WALL, color=CLIP_BLACK, friction=CLIP_FRICTION)
        handle = Link(f"handle_{name}",
                      [box((0.0, s * HANDLE_TIP[0], HANDLE_TIP[1]), (2 * LOOP[-1][0] * MM, WIRE * 0.7, WIRE * 0.7))],
                      Hinge("x", (0.0, s * CURL_Y, 0.0),
                            -HANDLE_FOLD_DEG if s > 0 else 0.0, 0.0 if s > 0 else HANDLE_FOLD_DEG,
                            damping=1e-5, initial=0.0, armature=1e-5,
                            static_friction=HANDLE_FRICTION[0], dynamic_friction=HANDLE_FRICTION[1]),
                      mass=M_HANDLE, parent=f"jaw_{name}", color=WIRE_SILVER, friction=CLIP_FRICTION)
        links += [jaw, handle]
    return links


CLIP_YAW = math.pi / 2                           # K x (jaw line) -> world +y; K +y (jaw_l, front) -> world -x
R_WK = _rz(CLIP_YAW)
CLIP_START = (0.150, -0.165, TOP + CURL_R + 0.0002)   # standing on its closed jaws, both handles up


def _spine_mesh():
    """The spine (back plate) as a convex plate, its inner face JAW_H above the lips' centre line."""
    pts = [(sx * CLIP_W / 2, sy * BACK_W / 2, JAW_H + (STEEL_T if sz > 0 else 0.0))
           for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    return _box_mesh_from_corners("spine", pts, CLIP_BLACK)


def _clip(start=CLIP_START):
    links = _clip_links()
    return MeshMechanism(
        "binder_clip", tuple(float(v) for v in start),
        [box((0.0, 0.0, JAW_H + STEEL_T / 2), (CLIP_W, BACK_W, STEEL_T))],
        links, mass=M_BACK, color=CLIP_BLACK, yaw=CLIP_YAW, friction=CLIP_FRICTION, contact_offset=0.002, grip=0,
        meshes={"base": (_spine_mesh(),), "jaw_l": _jaw_meshes(1), "jaw_r": _jaw_meshes(-1),
                "handle_l": _handle_meshes(1), "handle_r": _handle_meshes(-1)})


CLIP = _clip()
PRESSED_PARTS = ("binder_clip",)
OBJECTS = [RACK, STACK, CLIP]
SOLVER_ITERS = 64


def _more_solver_iterations():
    """Solver setting, not a physical claim: 64 position iterations for the clip's articulation and the paper stack
    (the part kit's defaults are 32 and 16; the merged binder-clip task uses 64 for both)."""
    import yam_mesh
    if getattr(yam_mesh, "_clip_solver_iters", False):
        return
    part_spawn, mech_spawn = yam_mesh._spawn_mesh_part, yam_mesh._spawn_mesh_mechanism

    def _set(path, kwargs, articulation):
        import isaaclab.sim as sim_utils
        from pxr import PhysxSchema
        stage = kwargs.get("stage") or sim_utils.get_current_stage()
        prim = stage.GetPrimAtPath(str(path))
        api = PhysxSchema.PhysxArticulationAPI if articulation else PhysxSchema.PhysxRigidBodyAPI
        api.Apply(prim).CreateSolverPositionIterationCountAttr().Set(SOLVER_ITERS)

    def spawn_part(path, cfg, *args, **kwargs):
        prim = part_spawn(path, cfg, *args, **kwargs)
        if cfg.part_name == STACK_NAME:
            _set(path, kwargs, False)
        return prim

    def spawn_mechanism(path, cfg, *args, **kwargs):
        prim = mech_spawn(path, cfg, *args, **kwargs)
        if cfg.mech_name == "binder_clip":
            _set(path, kwargs, True)
        return prim

    yam_mesh._spawn_mesh_part, yam_mesh._spawn_mesh_mechanism = spawn_part, spawn_mechanism
    yam_mesh._clip_solver_iters = True


_more_solver_iterations()

# ------------------------------------------------------------------ scoring
BITE_MIN = 0.003          # the paper's top edge at least 3 mm past the lips
END_MARGIN = 0.003        # both ends of the clip at least 3 mm inside the paper's ends
FOLD_GAP = 0.010          # a handle is folded when its loop end lies 20 mm or more below the lips, within 10 mm
                          # of the paper's face on its own side
SLOT_Z = 0.010            # the stack's bottom edge more than 10 mm off the slot floor: no longer in the slot
OUT_OF_PLACE = 0.010      # the stack more than 10 mm off the front wall or off the divider it leans on: knocked out
D1_EDGE = np.array([WALL_X[1], 0.0, TOP + WALL_H[1]])   # the top front edge of the first divider (x, z)


def stack_axes(st):
    """(through the stack front to back, along its length, up the sheets) in world: the lean is in the part's meshes."""
    R = st.rotation() @ R_LEAN
    return R[:, 0], R[:, 1], R[:, 2]


def stack_place(st):
    """(foot height above the slot floor, how far the stack stands off its backstops) in metres: the front face's
    foot to the front wall, and the back face to the top edge of the divider it leans on."""
    a, b, c = stack_axes(st)
    ctr = np.asarray(st.origin, float)
    foot = ctr - c * PAPER_H / 2 - a * STACK_T / 2
    off_wall = abs(float(foot[0]) - X_FRONT_FOOT)
    back_face = ctr + a * STACK_T / 2
    d1 = D1_EDGE.copy()
    d1[1] = ctr[1]
    off_divider = abs(float((d1 - back_face) @ a))
    return float(foot[2] - FLOOR), max(off_wall, off_divider)


def clip_terms(objects):
    """Bite (the top edge's depth past the lips), across offset, ends inside the paper, twist and rock (deg)."""
    clip, st = objects["binder_clip"], objects[STACK_NAME]
    o, Rk = np.asarray(clip.origin, float), clip.rotation()
    a, b, c = stack_axes(st)
    ctr = np.asarray(st.origin, float)
    top = ctr + c * PAPER_H / 2
    pt = top + b * float(np.dot(o - top, b))                # the top edge's point under the clip's middle
    k = Rk.T @ (pt - o)
    ends_ok = all(abs(float(np.dot(o + Rk[:, 0] * e * CLIP_W / 2 - ctr, b))) <= PAPER_L / 2 - END_MARGIN
                  for e in (-1, 1))
    twist = math.degrees(math.acos(min(1.0, abs(float(Rk[:, 0] @ b)))))     # the lip line against the top edge
    return {"bite": float(k[2]), "across": float(k[1]), "ends_ok": ends_ok, "square_deg": twist}


def lip_gap(deg_l, deg_r):
    """Gap between the rolled lips' inner faces for the two jaw angles (deg, opening positive)."""
    return float(_lip(math.radians(deg_l))[0] + _lip(math.radians(deg_r))[0] - 2 * CURL_R)


def handle_folded(objects, name):
    """The handle's loop end lies 20 mm or more below the lips' line (along the sheets) and within FOLD_GAP of the
    paper's face on its own side (the front handle on the front face, the back one on the back face)."""
    clip, st = objects["binder_clip"], objects[STACK_NAME]
    s = 1.0 if name == "l" else -1.0                   # jaw_l / handle_l is the front side (toward the robot)
    a, b, c = stack_axes(st)
    end = np.asarray(clip.link_to_world(f"handle_{name}", np.array([0.0, s * HANDLE_TIP[0], HANDLE_TIP[1]])), float)
    off_face = -s * float((end - np.asarray(st.origin, float)) @ a) - STACK_T / 2
    below = float((end - np.asarray(clip.origin, float)) @ c)
    folded = below < -0.020 and off_face <= FOLD_GAP
    return bool(folded), (round(off_face * 1e3, 1), round(below * 1e3, 1))


def score_terms(objects):
    clip = objects["binder_clip"]
    t = clip_terms(objects)
    # the jaw must really be open round the paper (the paper cannot pass through a lip)
    jaw_gap = lip_gap(clip.angle_deg("jaw_l"), -clip.angle_deg("jaw_r"))
    on = (t["bite"] >= BITE_MIN and abs(t["across"]) < CAPACITY / 2 and t["ends_ok"]
          and jaw_gap >= STACK_T - 0.0015 and float(clip.origin[2]) > TOP + 0.030)
    foot_z, off = stack_place(objects[STACK_NAME])
    placed = foot_z <= SLOT_Z and off <= OUT_OF_PLACE
    seat_gap = JAW_H - t["bite"]
    folds = sum(handle_folded(objects, n)[0] for n in ("l", "r"))
    return on, placed, seat_gap, t["square_deg"], off, folds, t


def _ladder(on, placed, seat_gap, square_deg, off, folds):
    if not (on and placed):
        return 0.0
    seat = 1.0 if seat_gap <= 0.003 else 0.75 if seat_gap <= 0.006 else 0.5 if seat_gap <= 0.012 else 0.25
    square = 1.0 if square_deg <= 3.0 else 0.75 if square_deg <= 8.0 else 0.5
    stack = 1.0 if off <= 0.002 else 0.75 if off <= 0.005 else 0.5
    handles = 1.0 if folds == 2 else 0.75 if folds == 1 else 0.5
    return min(seat, square, stack, handles)


def score(objects):
    on, placed, seat_gap, square_deg, off, folds, _ = score_terms(objects)
    value = _ladder(on, placed, seat_gap, square_deg, off, folds)
    if value <= 0.0:
        return 0.0
    names = ["binder_clip", STACK_NAME]
    clear = all(released(objects[n]) for n in names)
    still = held_for("still", clear and all(_still(n, objects[n]) for n in names))
    return value if clear and still >= STILL_S else 0.0


def _still(name, st, tol=0.001):
    """In a run: the part has stayed within 1 mm of where it was when this still spell began (sub-millimetre contact
    chatter of the clamped jaw shows in the velocity readings, not in the pose). Offline: under 5 mm/s."""
    if _RUN["t"] is None:
        v = getattr(st, "lin_vel", None)
        return v is None or float(np.linalg.norm(v)) < 0.005
    _episode_memory()
    p = np.asarray(st.origin, float)
    anchor = _ANCHORS.get(name)
    if anchor is None or float(np.linalg.norm(p - anchor)) > tol:
        _ANCHORS[name] = p
        return False
    return True


def success(objects):
    return score(objects) >= 1.0


# ------------------------------------------------------------------ scripted demo (reads the simulator's object poses)
class GripOrientation(float):
    """A float yaw (for the offline checker's coarse replay) carrying the full grip rotation for the arm's IK."""

    def __new__(cls, matrix):
        obj = super().__new__(cls, math.atan2(-matrix[0, 0], matrix[1, 0]))
        obj.matrix = np.asarray(matrix, float).copy()
        return obj


_original_grip_rotation = getattr(_motion.grip_rotation, "_bc_original", _motion.grip_rotation)


def _grip_rotation(yaw=0.0):
    matrix = getattr(yaw, "matrix", None)
    if isinstance(matrix, np.ndarray) and matrix.shape == (3, 3):
        return matrix
    return _original_grip_rotation(yaw)


_grip_rotation._bc_original = _original_grip_rotation
_motion.grip_rotation = _grip_rotation


def _frame(closing, approach):
    """TCP rotation: x = closing axis, z = approach (TCP toward the fingertips), y = z x x."""
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(closing, float) - z * float(np.dot(closing, z))
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def _aa(axis, a):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(a) * k + (1.0 - math.cos(a)) * (k @ k)


def _T(p, R):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, p
    return T


def _inv(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


DEMO_PACE = 1.0
TIP_BEHIND_TCP = 0.008        # handle loop ends 8 mm behind the TCP: the whole handle past the spine lies on the pads
TILT_R = math.radians(15.0)   # right gripper turned about its closing axis, wrist toward -y (away from the left arm);
                              # closing along -K y (toward the back): both checked exact with the arena IK
CLIP_AT_Y = PAPER_Y[0] + 0.005 + CLIP_W / 2        # clip centre along the top edge, its end 5 mm in from the right end
OPEN_WIDE = -0.020            # pads 40 mm apart
CAPTURE_FINGER = -0.0175      # pads 35 mm apart: ~1 mm outside the raised handles' ends
SQUEEZE_FINGER = -0.0080      # aim the pads 16 mm apart: balances the spring at ~13.5 deg (lips ~9.5 mm apart,
                              # ~16.6 N at the handle ends, handles still leaning ~9 deg outward). Squeezing on to the
                              # stop is unstable: near parallel a 2 deg tilt puts one handle's pad on its fulcrum.
RELEASE_FINGER = -0.016       # pads 32 mm apart: the handles follow out until the jaw shuts on the paper
SEAT_OVERDRIVE = 0.0015         # push the spine 1.5 mm onto the edge before letting go
PUSH_PT = 0.010               # the fingertip point used to fold a handle: 10 mm past the TCP
FOLD_LEAN_DEG = 25.0
PARK = {"left": np.array([0.200, 0.220, TOP + 0.200]), "right": np.array([0.180, -0.230, TOP + 0.220])}
LEFT_AT_Y = PAPER_Y[1] - 0.012   # the left pads 12 mm in from the paper's left end ...
LEFT_DEPTH = 0.020               # ... their tips 20 mm below the top edge
PAD_TIP = 0.013                  # pad tips 13 mm past the TCP
TILT_L = math.radians(20.0)      # left gripper turned about its closing axis, wrist toward +y
LEFT_FINGER = -0.00075           # pads aimed 1.5 mm apart on the 4 mm stack: about 6 N of pinch
_READS = {"episode": None, "values": {}}


def _states(names):
    env = _RUN["env"]
    if env is None:
        return None
    try:
        from yam_arena import scene as arena_scene
        return arena_scene.object_states(env, list(names))
    except Exception as e:  # noqa: BLE001
        print(f"clip demo: object read failed: {e!r}", flush=True)
        return None


def _read(key, fn):
    if _READS["episode"] != _RUN["episode"]:
        _READS["episode"], _READS["values"] = _RUN["episode"], {}
    vals = _READS["values"]
    if key not in vals:
        vals[key] = fn()
    return vals[key]


def _stack_frame(st=None):
    """(top point on the stack's mid-plane at y = CLIP_AT_Y, b length axis, c up axis, a front-to-back normal)."""
    if st is None:
        a, b, c = N_BACK, np.array([0.0, 1.0, 0.0]), U_UP
        ctr = stack_center()
    else:
        a, b, c = stack_axes(st)
        ctr = np.asarray(st.origin, float)
    top = ctr + c * PAPER_H / 2
    y_at = CLIP_AT_Y + float(ctr[1] - stack_center()[1])      # 5 mm in from the paper's actual right end
    top = top + b * float(y_at - top[1]) / max(b[1], 1e-6)
    return top, b, c, a


def _stack_now():
    sts = _states([STACK_NAME])
    return None if sts is None else sts[STACK_NAME]


def _clip_goal(stack, above):
    """Clip pose (origin = lips' centre line, R = K axes) seated on the stack, raised `above` along the sheets."""
    top, b, c, a = stack
    Rk = np.column_stack([b, np.cross(c, b), c])
    return top - c * (JAW_H - SEAT_OVERDRIVE) + c * above, Rk


def _grip_in_k():
    """Nominal TCP pose in clip frame K while squeezing: on the centre line, the handle ends TIP_BEHIND_TCP behind the
    TCP, pointing down K's z with the wrist turned TILT_R toward -K x, closing across the jaw (along K y)."""
    p = np.array([0.0, 0.0, HANDLE_TIP[1] - TIP_BEHIND_TCP])
    approach = math.cos(TILT_R) * np.array([0.0, 0.0, -1.0]) + math.sin(TILT_R) * np.array([1.0, 0.0, 0.0])
    return p, _frame(np.array([0.0, -1.0, 0.0]), approach)


_G0 = _T(*_grip_in_k())                         # hand in clip frame, nominal
_CARRY = {"episode": None, "G": None}


def _hand_from_clip(o, Rk):
    """Hand pose for a clip pose, through the measured grasp (nominal until the first read)."""
    if _CARRY["episode"] != _RUN["episode"]:
        _CARRY.update(episode=_RUN["episode"], G=None)
    G = _CARRY["G"] if _CARRY["G"] is not None else _G0
    T = _T(o, Rk) @ G
    return T[:3, 3], T[:3, :3]


def _measure_grasp(key, commanded):
    """Read the clip while the hand is commanded to `commanded` (p, R): G = clip^-1 * commanded hand."""
    def read():
        sts = _states(["binder_clip"])
        if sts is None:
            return None
        st = sts["binder_clip"]
        A = _T(np.asarray(st.origin, float), st.rotation())
        G = _inv(A) @ _T(*commanded)
        dp = (G[:3, 3] - _G0[:3, 3]) * 1e3
        dang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(_G0[:3, :3].T @ G[:3, :3]) - 1) / 2))))
        if DIAG:
            print(f"CLIP_READ {key} t={_RUN['t']} grasp_shift_mm={tuple(round(float(v), 1) for v in dp)} "
                  f"grasp_rot_deg={dang:.1f} jaw=({st.angle_deg('jaw_l'):.1f},{st.angle_deg('jaw_r'):.1f}) "
                  f"handles=({st.angle_deg('handle_l'):.1f},{st.angle_deg('handle_r'):.1f})", flush=True)
        return G
    G = _read(key, read)
    if G is not None:
        if _CARRY["episode"] != _RUN["episode"]:
            _CARRY.update(episode=_RUN["episode"], G=None)
        _CARRY["G"] = G
    return G


class LiveWaypoint(Waypoint):
    """A waypoint whose hand pose is worked out when it is played (fn -> (pos, R))."""

    def __init__(self, duration, fn, finger, label, keep=False):
        self.duration = duration * (1.0 if keep else DEMO_PACE)
        self.finger, self.label, self._fn = finger, label, fn
        self._cache = (None, None)

    def _pose(self):
        """Worked out once per episode, when the player first reaches this waypoint, then frozen (the player
        re-reads every earlier waypoint each tick; a later grasp read must not move them)."""
        if self._cache[0] == _RUN["episode"] and _RUN["env"] is not None:
            return self._cache[1]
        p, R = self._fn()
        value = (tuple(float(v) for v in p), GripOrientation(R))
        self._cache = (_RUN["episode"], value)
        return value

    @property
    def pos(self):
        return self._pose()[0]

    @property
    def yaw(self):
        return self._pose()[1]


def _face_psi(Rk, c, s):
    """Angle (from the clip's K z toward side s) of the paper's face running down from the lips, in the clip frame."""
    d = -np.asarray(c, float)
    # near 180 deg: written as pi - atan2(.) so a face exactly along -K z reads +pi, never -pi (a -pi end angle
    # sends the sweep the wrong way round, over the other handle)
    return math.pi - math.atan2(float(d @ (s * Rk[:, 1])), -float(d @ Rk[:, 2]))


def _clip_start_now():
    sts = _states(["binder_clip"])
    if sts is None:
        return np.asarray(CLIP_START, float), R_WK
    st = sts["binder_clip"]
    return np.asarray(st.origin, float), st.rotation()


def _right_plan():
    W = []
    start = lambda: _read("clip_start", _clip_start_now)

    def at_start(dz=0.0):
        def f():
            o, Rk = start()
            p, R = (lambda T: (T[:3, 3], T[:3, :3]))(_T(o, Rk) @ _G0)
            return p + np.array([0.0, 0.0, dz]), R
        return f

    planned = lambda dz: (lambda T: (T[:3, 3] + np.array([0.0, 0.0, dz]), T[:3, :3]))(
        _T(np.asarray(CLIP_START, float), R_WK) @ _G0)
    W.append(LiveWaypoint(3.0, lambda: planned(0.060), OPEN_WIDE, "over the clip standing on the desk"))
    W.append(LiveWaypoint(1.0, at_start(0.025), OPEN_WIDE, "line up over the clip"))
    W.append(LiveWaypoint(1.2, at_start(0.0), OPEN_WIDE, "pads down either side of the raised handles"))
    W.append(LiveWaypoint(0.8, at_start(0.0), CAPTURE_FINGER, "pads in to just outside the handles", keep=True))
    W.append(LiveWaypoint(1.5, at_start(0.0), SQUEEZE_FINGER, "squeeze the handles flat: the jaw opens", keep=True))
    W.append(LiveWaypoint(0.8, at_start(0.0), SQUEEZE_FINGER, "hold the squeeze"))
    W.append(LiveWaypoint(1.5, at_start(0.060), SQUEEZE_FINGER, "lift the open clip"))

    stack = lambda: _read("stack", lambda: _stack_frame(_stack_now()))
    lifted_cmd = at_start(0.060)

    def over(above, key=None, cmd=None):
        def f():
            if key is not None:
                _measure_grasp(key, cmd())
            return _hand_from_clip(*_clip_goal(stack(), above))
        return f

    above_hi, above_lo = 0.020 + JAW_H, 0.006 + JAW_H
    seat = 0.0 if DEMO_WAY != "early" else JAW_H - 0.005
    W.append(LiveWaypoint(0.6, lifted_cmd, SQUEEZE_FINGER, "settle"))
    W.append(LiveWaypoint(3.0, over(above_hi, "lifted", lifted_cmd), SQUEEZE_FINGER,
                          "carry it over the paper's top edge"))
    W.append(LiveWaypoint(0.8, over(above_hi), SQUEEZE_FINGER, "settle"))
    hi_cmd = over(above_hi)
    W.append(LiveWaypoint(1.5, over(above_lo, "over", hi_cmd), SQUEEZE_FINGER,
                          "lower until the lips are 6 mm over the edge"))
    W.append(LiveWaypoint(0.8, over(above_lo), SQUEEZE_FINGER, "settle"))
    lo_cmd = over(above_lo)
    W.append(LiveWaypoint(2.0, over(seat, "lineup", lo_cmd), SQUEEZE_FINGER,
                          "slide the open jaw down over the edge to the back of the jaw"))
    W.append(LiveWaypoint(0.6, over(seat), SQUEEZE_FINGER, "hold"))
    W.append(LiveWaypoint(1.6, over(seat), RELEASE_FINGER, "let go slowly: the jaw closes on the paper",
                          keep=True))
    W.append(LiveWaypoint(0.4, over(seat), OPEN_WIDE, "open"))
    W.append(LiveWaypoint(1.2, over(seat + 0.045), OPEN_WIDE, "rise clear of the handles"))

    # fold each handle down: a closed fingertip sweeps it round its rolled lip and presses it on the paper
    def fold_frame(side):
        """Clip, pivots, handle angles and face angles, read once per side just before that side's fold."""
        def read():
            sts = _states(["binder_clip"])
            if sts is None:
                o, Rk = _clip_goal(_stack_frame(None), 0.0)
                piv = {s: o + Rk @ np.array([0.0, s * (STACK_T / 2 + CURL_R), 0.0]) for s in (1, -1)}
                ang = {s: HANDLE_TILT - math.radians(jaw_angle_for_gap(STACK_T)) for s in (1, -1)}
                face = {s: _face_psi(Rk, U_UP, s) for s in (1, -1)}
                return o, Rk, piv, ang, face
            st = sts["binder_clip"]
            stack = _stack_now()
            o = np.asarray(st.origin, float)
            if stack is None:
                Rk = st.rotation()
            else:   # sweep in the paper's own frame (the clip may sit rocked a few degrees on the edge)
                a, b, c = stack_axes(stack)
                Rk = np.column_stack([b, -a, c])
            piv, ang = {}, {}
            for s, nm in ((1, "handle_l"), (-1, "handle_r")):
                pv = np.asarray(st.link_to_world(nm, np.array([0.0, s * CURL_Y, 0.0])), float)
                tip = np.asarray(st.link_to_world(nm, np.array([0.0, s * HANDLE_TIP[0], HANDLE_TIP[1]])), float)
                v = Rk.T @ (tip - pv)
                piv[s] = pv
                ang[s] = math.atan2(s * v[1], v[2])
            c = stack_axes(stack)[2] if stack is not None else Rk[:, 2]
            face = {s: _face_psi(Rk, c, s) for s in (1, -1)}
            if DIAG:
                print(f"CLIP_READ folds t={_RUN['t']} handle_deg=({math.degrees(ang[1]):.1f},"
                      f"{math.degrees(ang[-1]):.1f}) face_deg=({math.degrees(face[1]):.1f},"
                      f"{math.degrees(face[-1]):.1f})", flush=True)
            return o, Rk, piv, ang, face
        return _read("folds_front" if side > 0 else "folds_back", read)

    def fold_pose(s, psi, radius, lift=0.0, out=0.0):
        def f():
            o, Rk, piv, ang, face = fold_frame(s)
            side = Rk[:, 1] * s
            up = Rk[:, 2]
            pt = piv[s] + radius * (math.sin(psi) * side + math.cos(psi) * up) + up * lift + side * out
            lean = max(0.0, min(1.0, (psi - math.radians(80)) / math.radians(70))) * math.radians(FOLD_LEAN_DEG)
            approach = _aa(Rk[:, 0], -s * lean) @ (-up)
            approach = math.cos(TILT_R) * approach + math.sin(TILT_R) * Rk[:, 0]
            R = _frame(-Rk[:, 1], approach)
            return pt - R[:, 2] * PUSH_PT, R
        return f

    # the front handle is pushed near its loop's end; the back one, which must be pressed up against the
    # overhanging back face, nearer the middle of its loop (wider there) from further inside
    for s, name, r, trail in ((1, "front", HANDLE_L - 0.002, math.radians(10.0)),
                              (-1, "back", HANDLE_L - 0.007, math.radians(20.0))):
        def start_psi(s=s, trail=trail):
            return fold_frame(s)[3][s] - trail
        W.append(LiveWaypoint(1.5, lambda s=s, r=r: fold_pose(s, start_psi(s), r, lift=0.020)(), FINGER_CLOSED,
                              f"fingertip over the inside of the {name} handle"))
        W.append(LiveWaypoint(1.0, lambda s=s, r=r: fold_pose(s, start_psi(s), r)(), FINGER_CLOSED,
                              f"fingertip down against the inside of the {name} handle"))
        steps = 8

        def end_psi(s=s):
            return fold_frame(s)[4][s]                # the fingertip's line ends along the paper's face
        for i in range(1, steps + 1):
            def sweep(s=s, i=i, r=r):
                p0 = start_psi(s)
                return fold_pose(s, p0 + (end_psi(s) - p0) * i / steps, r)()
            W.append(LiveWaypoint(0.45, sweep, FINGER_CLOSED, f"fold the {name} handle down ({i}/{steps})"))
        W.append(LiveWaypoint(0.6, lambda s=s, r=r: fold_pose(s, end_psi(s), r, out=-0.001)(), FINGER_CLOSED,
                              f"press the {name} handle on the paper"))
        W.append(LiveWaypoint(0.8, lambda s=s, r=r: fold_pose(s, end_psi(s) - math.radians(5.0), r, out=0.015)(),
                              FINGER_CLOSED, f"back off the {name} handle"))
        W.append(LiveWaypoint(1.2, lambda s=s: fold_pose(s, math.radians(60.0), HANDLE_L + 0.033, lift=0.020)(),
                              FINGER_CLOSED, f"rise clear of the {name} side"))
    return W


def _left_pinch(open_out=0.0, planned=False):
    """Pads either side of the stack 12 mm in from its left end, pad tips LEFT_DEPTH below the top edge, pointing
    down the sheets with the wrist turned TILT_L toward +y (away from the right arm), closing through the stack."""
    def f():
        top, b, c, a = _stack_frame(None) if planned else _read("stack_left", lambda: _stack_frame(_stack_now()))
        p = top + b * (LEFT_AT_Y - top[1]) / max(b[1], 1e-6) - c * (LEFT_DEPTH - PAD_TIP) + c * open_out
        approach = math.cos(TILT_L) * (-c) - math.sin(TILT_L) * b
        return p, _frame(-a, approach)
    return f


def _park_pose(arm):
    if arm == "right":
        R = _frame(np.array([1.0, 0.0, 0.0]), math.cos(TILT_R) * np.array([0.0, 0.0, -1.0])
                   + math.sin(TILT_R) * np.array([0.0, 1.0, 0.0]))
    else:
        R = _frame(np.array([-1.0, 0.0, 0.0]), math.cos(TILT_L) * np.array([0.0, 0.0, -1.0])
                   - math.sin(TILT_L) * np.array([0.0, 1.0, 0.0]))
    return lambda: (PARK[arm], R)


def demo():
    right = _right_plan()
    left = []
    left.append(LiveWaypoint(3.0, _left_pinch(open_out=0.050, planned=True), OPEN_WIDE, "over the stack's left end"))
    left.append(LiveWaypoint(1.5, _left_pinch(), OPEN_WIDE, "pads down either side of the stack"))
    left.append(LiveWaypoint(1.0, _left_pinch(), LEFT_FINGER, "pinch the stack square and hold it", keep=True))
    t_right = sum(w.duration for w in right)
    t_left = sum(w.duration for w in left)
    left.append(LiveWaypoint(max(0.5, t_right - t_left), _left_pinch(), LEFT_FINGER, "hold the stack still"))
    left.append(LiveWaypoint(0.8, _left_pinch(), OPEN_WIDE, "let go of the stack", keep=True))
    left.append(LiveWaypoint(1.0, _left_pinch(open_out=0.050), OPEN_WIDE, "rise clear"))
    right.append(LiveWaypoint(2.5, _park_pose("right"), OPEN_WIDE, "park"))
    left.append(LiveWaypoint(2.5, _park_pose("left"), OPEN_WIDE, "park"))
    pad = max(0.0, sum(w.duration for w in left) - sum(w.duration for w in right))
    right.append(LiveWaypoint(pad + 3.0, _park_pose("right"), OPEN_WIDE, "parked: still"))
    left.append(LiveWaypoint(max(0.0, sum(w.duration for w in right) - sum(w.duration for w in left)),
                             _park_pose("left"), OPEN_WIDE, "parked: still"))
    return {"right": right, "left": left}


def demo_duration():
    d = demo()
    return max(sum(w.duration for w in d["right"]), sum(w.duration for w in d["left"]))


# ------------------------------------------------------------------ reference layouts for the scorer tests (states.yaml)
def _quat_xyzw(Rm):
    from yam_arena.ik import matrix_to_quat_xyzw
    return tuple(float(v) for v in matrix_to_quat_xyzw(np.asarray(Rm, float)))


JAW_ON_STACK = jaw_angle_for_gap(STACK_T)            # deg: the jaw shut on the 4 mm stack
_pivot_y = float(_lip(math.radians(JAW_ON_STACK))[0])
FOLD_DEG = -float(HANDLE_TILT * 180 / math.pi - JAW_ON_STACK
                  - (180.0 + math.degrees(math.asin((_pivot_y - STACK_T / 2 - WIRE / 2) / HANDLE_L))))
_GOAL_O, _GOAL_R = _clip_goal(_stack_frame(None), -SEAT_OVERDRIVE)   # fully seated: top edge on the jaw back
GOAL_CLIP = tuple(float(v) for v in _GOAL_O)
GOAL_QUAT = _quat_xyzw(_GOAL_R)
GOAL_QUAT_TWIST5 = _quat_xyzw(_aa(N_BACK, math.radians(5.0)) @ _GOAL_R)   # turned 5 deg in the paper's plane
UP = tuple(float(v) for v in U_UP)
BACKWARD = tuple(float(v) for v in N_BACK)
STACK_POS = tuple(float(v) for v in stack_center())
