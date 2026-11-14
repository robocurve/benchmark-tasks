"""A floating click-lock floor, one row laid: the table top is the subfloor. The laid row (a 300 mm cut piece and a
full 600 x 300 mm rigid-core vinyl tile) is fixed, its long groove edge toward the robot along x = 0.460. One loose
tile (600 x 300 x 5 mm, 1.55 kg) has been set against the row: its long tongue edge rests on the laid row's lower
lip, 8 mm short of home, and its near edge lies on a pine batten, so the tile is tipped up about 9 degrees. The tile
must be angled in: lifted at its near edge, its tongue pushed home under the laid tiles' upper lip along the whole
600 mm, slid along the row until its end meets the wall spacer, and lowered flat so its locking tooth drops behind
the laid row's locking bump. The click profile is modelled as convex collision pieces taken from an angle-in
(Valinge 2G type) long-edge section scaled to the 4 mm core; the lock is pure contact geometry, nothing scripted.
Zero object jitter; 20 Hz.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, box
from yam_mesh import Mesh, MeshPart
from yam_arena import demo as _motion
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.task_file import TABLE_TOP

SCENE = "station"

# Release-and-stillness block, verbatim from hang_holder_on_keyhole_studs.py (see release-stillness.md).
# The task module must define: import math; import numpy as np; from yam_arena.constants import FINGER_OPEN;
# and SCENE ("station" or "tabletop"). Paste after the imports.

# Release and stillness
# Released means every gripper TCP is at least RELEASE_CLEAR from the object's box.
# Still means below the task's linear and angular speed limits for STILL_S seconds.
# Evaluate every sticky condition and timer at the top of score() on every call,
# before any early return, so interrupted conditions cannot retain elapsed time.
RELEASE_CLEAR = 0.030
STILL_S = 1.0
TEST_GRIPPERS = None


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


def grippers():
    if TEST_GRIPPERS is not None:
        return TEST_GRIPPERS
    env = _RUN["env"]
    if env is None:
        return None
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, LEFT_BASE_POS, RIGHT_BASE_POS

    bases = station.BASES if SCENE == "station" else {"left": LEFT_BASE_POS, "right": RIGHT_BASE_POS}
    out = {}
    for arm in ("left", "right"):
        art = env.unwrapped.scene[arm + "_robot"]
        arm_ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
        finger_ids = art.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        joints = art.data.joint_pos
        q = getattr(joints, "torch", joints)[0]
        if hasattr(q, "detach"):
            q = q.detach().cpu().numpy()
        q = np.asarray(q)
        tcp = ik.tcp_pose(q[arm_ids], bases[arm])[0]
        opening = float(np.clip(np.mean(q[finger_ids]) / FINGER_OPEN, 0.0, 1.0))
        out[arm] = (tcp, opening)
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
    if _RUN["t"] is None: return bool(cond)
    _episode_memory()
    _EVER[key] = bool(_EVER.get(key, False) or cond)
    return _EVER[key]


# ------------------------------------------------------------------ geometry helpers
def _ry(a):
    """Rotation about world y; a positive angle lifts the -x side (the tile's near edge) and lowers the +x side."""
    c, s = math.cos(a), math.sin(a)
    return np.array(((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c)))


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array(((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c)))


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))


def _prism(name, outline_xz_mm, y0, y1, R=np.eye(3), t=(0.0, 0.0, 0.0), color=None, proxy=()):
    """Convex prism: a convex (x, z) outline in millimetres (tile frame), extruded along y from y0 to y1 (metres),
    then rotated by R and shifted by t (metres). Collision: PhysX convex hull."""
    prof = [(float(x) / 1000.0, float(z) / 1000.0) for x, z in outline_xz_mm]
    n = len(prof)
    pts = np.array([(x, y, z) for y in (y0, y1) for (x, z) in prof], float)
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
    pts = pts @ np.asarray(R, float).T + np.asarray(t, float)
    return Mesh(name, pts, faces, collision="convex", color=color, proxy=tuple(proxy))


# ------------------------------------------------------------------ the click profile (millimetres)
# Long-edge section of a 5 mm rigid-core tile: 1.0 mm IXPE pad (z 0..1) under a 4.0 mm stone-polymer core
# (z 1..5). Angle-in profile of the Valinge 2G type (US 9,314,936 Figs. 1a-2c: upper lip, tongue and strip
# body each about 1 mm in a 3 mm panel; here 1.3-1.4 mm in the 4 mm core). The groove edge carries an upper
# lip over a 1.4 mm groove and a strip that runs 7 mm past the joint with a 1.0 mm locking bump at its end.
# The tongue edge carries a 1.0 mm tongue (2.6 mm long, bevelled underneath so it can swing in) and a locking
# tooth that drops behind the bump. Clearances: 0.2 mm over and under the tongue, 0.2 mm locking play, 0.1 mm
# between the tooth and the strip. The locking faces lean back 62.5 deg, tangent to the arc about the joint's top
# edge, so the tooth can swing in and only swings out by lifting the tile again.
LOCK_SLOPE = 0.52      # mm of run per mm of rise of the locking faces
LOCK_PLAY = 0.20       # mm
BUMP_H = 1.5           # locking bump height above the strip (mm): the tongue gets under the lip only when the
                       # tile is tipped 12 deg or more, so a tile dropped from a low tilt rides on the bump
T_MM = 5.0             # tile thickness (pad + core)


def groove_edge(x0, body_to):
    """Groove-edge pieces of a tile whose joint plane (top edge) is at x0 mm, material toward +x. The locking bump
    is one convex column with the strip under it (its locking face carried down to the floor), so no piece is
    thinner than about 1 mm in section."""
    face0 = x0 - 5.4 + LOCK_SLOPE * 2.2          # the locking face's line where it meets z = 0
    top = 2.2 + BUMP_H
    return {
        "bump": [(x0 - 7.0, 0.0), (face0, 0.0), (x0 - 5.4 - LOCK_SLOPE * (BUMP_H - 0.3), top - 0.3),
                 (x0 - 6.35, top), (x0 - 6.7, top), (x0 - 7.0, top - 0.3)],
        "strip": [(face0, 0.0), (x0 + 3.0, 0.0), (x0 + 3.0, 2.0), (x0 + 0.4, 2.2), (x0 - 5.4, 2.2)],
        "lip": [(x0, 3.6), (x0 + 3.0, 3.6), (x0 + 3.0, T_MM), (x0, T_MM)],
        "body": [(x0 + 3.0, 0.0), (body_to, 0.0), (body_to, T_MM), (x0 + 3.0, T_MM)],
    }


def tongue_edge(x1):
    """Tongue-edge pieces of a tile whose joint plane (top edge) is at x1 mm, material toward -x."""
    def bump_face(z):
        return (x1 - 5.4) - LOCK_SLOPE * (z - 2.2)
    tz = 2.2 + BUMP_H + 0.15                      # underside of the head, over the bump
    return {
        "head": [(x1 - 7.3, tz), (x1, tz), (x1, T_MM), (x1 - 7.3, T_MM)],
        "tooth": [(bump_face(2.3) + LOCK_PLAY, 2.3), (x1 - 2.0, 2.3), (x1, 2.626), (x1, tz),
                  (bump_face(tz) + LOCK_PLAY, tz)],
        "tongue": [(bump_face(2.3) + LOCK_PLAY + 0.5, 2.3), (x1 - 2.0, 2.3), (x1 + 2.6, 3.05), (x1 + 2.6, 3.15),
                   (x1 + 2.35, 3.4), (bump_face(3.4) + LOCK_PLAY + 0.5, 3.4)],
    }


def _inradius_mm(outline):
    """Smallest distance from the outline's centroid to its edges (mm): PhysX's internal radius of the prism."""
    pts = np.asarray(outline, float)
    c = pts.mean(0)
    d = []
    for a, b in zip(pts, np.roll(pts, -1, axis=0)):
        e = b - a
        n = np.array((-e[1], e[0])) / np.linalg.norm(e)
        d.append(abs(float(n @ (c - a))))
    return min(d)


HULL_RATIO = 70.0      # PhysX builds GPU convex hulls only when (farthest vertex / internal radius) < 100


def _segments(outline, y0, y1):
    """Lengths along y short enough for a GPU-compatible convex hull of this section."""
    r = _inradius_mm(outline) / 1000.0
    span = float(np.ptp(np.asarray(outline, float)[:, 0])) / 1000.0
    max_len = 2.0 * math.sqrt(max((HULL_RATIO * r) ** 2 - (span / 2) ** 2, 1e-12))
    n = max(1, int(math.ceil((y1 - y0) / max_len)))
    ys = np.linspace(y0, y1, n + 1)
    return list(zip(ys[:-1], ys[1:]))


# ------------------------------------------------------------------ layout (metres, world frame)
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

TILE_D, TILE_L, TILE_T = 0.300, 0.600, 0.005     # MSI Trecento 12 x 24 in (11.81 x 23.62 in), 5 mm with pad
TILE_MASS = 1.55                                  # 35 lb case of 10 less the carton; 2.0-2.1 g/cm3 core
X_SEAM = 0.460                                    # the laid row's groove edge (top joint line)
WALL_FACE = -0.3095                               # wall face (y); a 3/8 in (9.5 mm) spacer puts tiles at -0.300
SPACER_T = 0.0095
TARGET = np.array((X_SEAM - TILE_D / 2, 0.0, TOP))  # tile-frame origin when laid: bottom centre
WHITE = (0.86, 0.86, 0.84)       # White Ocean: white marble look
LAID = (0.80, 0.80, 0.78)
CORE = (0.30, 0.30, 0.32)        # dark stone-polymer core, seen in the profile
PINE = (0.82, 0.66, 0.42)
PLY = (0.70, 0.56, 0.38)
SPACER_BLUE = (0.15, 0.35, 0.80)

# Tile frame T: x from -150 mm (near groove edge's top line) to +150 mm (far tongue edge's top line), y along the
# tile (+-300 mm), z up from the bottom of the pad. Grip, scoring and the demo all use this frame.
NEAR_X_MM, FAR_X_MM = -TILE_D * 500, TILE_D * 500


def _tile_pieces():
    near = groove_edge(NEAR_X_MM, FAR_X_MM - 7.3)
    far = tongue_edge(FAR_X_MM)
    return near, far


# Start pose: the tongue rests on the laid row's locking bump 8 mm short of home; the near edge on the batten.
START_DX = -0.008          # far top edge 8 mm short of the seam
START_DZ = 0.00106         # far top edge raised 1.06 mm by the bump (2D contact model of the profile)
START_DY = 0.010           # 10 mm away from the spacer: the end gap starts at 19.5 mm
SPAWN_LIFT = 0.0002        # spawned 0.2 mm clear of its supports
BATTEN_FACE = 0.1515       # far face of the batten (x); the laid tile's near lip end stops at 0.153
BATTEN = (0.047, 0.200, 0.050)   # 47 x 50 mm sawn pine batten (UK "2x2"), 200 mm long, on its 47 mm face
PF_T = np.array((TILE_D / 2, 0.0, TILE_T))   # the far top edge line in the tile frame
GRIP_Y = 0.180                    # the demo grips 180 mm either side of the tile's middle, clear of the batten


def _start_pivot():
    return np.array((X_SEAM + START_DX, START_DY, TOP + TILE_T + START_DZ))


def _batten_tilt():
    """Tilt at which the tile's bottom plane touches the batten's top far edge, pivoting on the tongue."""
    pf = _start_pivot()
    lo, hi = math.radians(2.0), math.radians(20.0)
    for _ in range(80):
        th = (lo + hi) / 2
        c, s = math.cos(th), math.sin(th)
        u = (BATTEN_FACE - pf[0] + s * TILE_T) / c          # tile-frame run from the far edge line to the batten
        z = pf[2] - s * u - c * TILE_T
        if z < TOP + BATTEN[2]:
            lo = th
        else:
            hi = th
    return (lo + hi) / 2


TILT0 = _batten_tilt()                    # about 9.4 deg
R0 = _ry(TILT0)                           # start tilt, authored into the tile's mesh (the part frame stays level)
TILE_START = tuple(float(v) for v in (_start_pivot() - R0 @ PF_T + np.array((0.0, 0.0, SPAWN_LIFT))))


def _tile_proxy():
    """Offline proxy only: 31 inscribed boxes stepping along the tipped tile (the checker's boxes are level)."""
    out = []
    c, s = math.cos(TILT0), math.sin(TILT0)
    edges = np.linspace(NEAR_X_MM / 1000 - 0.007, FAR_X_MM / 1000, 32)
    # The two steps at the near edge are cut into lengths so each grip point has its own 30 mm grip box.
    cuts = (-0.300, -GRIP_Y - 0.015, -GRIP_Y + 0.015, GRIP_Y - 0.015, GRIP_Y + 0.015, 0.300)
    for i, (xa, xb) in enumerate(zip(edges[:-1], edges[1:])):
        x_lo, x_hi = c * xa + s * TILE_T / 2, c * xb + s * TILE_T / 2
        z_lo, z_hi = -s * xa, TILE_T * c - s * xb
        for ya, yb in (zip(cuts[:-1], cuts[1:]) if i < 2 else ((-0.300, 0.300),)):
            out.append(box(((x_lo + x_hi) / 2, (ya + yb) / 2, (z_lo + z_hi) / 2), (x_hi - x_lo, yb - ya, z_hi - z_lo)))
    return tuple(out)


def _tile_meshes():
    near, far = _tile_pieces()
    meshes = []
    for side, pieces in (("n", near), ("f", far)):
        for key, outline in pieces.items():
            color = WHITE if key in ("lip", "head", "body") else CORE
            for k, (y0, y1) in enumerate(_segments(outline, -TILE_L / 2, TILE_L / 2)):
                proxy = _tile_proxy() if (key == "body" and k == 0) else ()
                meshes.append(_prism(f"{side}_{key}_{k}", outline, y0, y1, R0, color=color, proxy=proxy))
    return tuple(meshes)


TILE = MeshPart("tile", TILE_START, (), mass=TILE_MASS, color=WHITE, friction=(0.4, 0.3),
                contact_offset=0.0005, meshes=_tile_meshes())     # offline grip: the proxy box nearest each TCP


def _laid_meshes():
    """The laid row: a 300 mm cut piece (y -0.300..0) and a full tile (y 0..0.600), groove edge at the seam.
    Its far (tongue) edge and the rows beyond are not modelled: the row is fixed, as if locked to them."""
    out, proxy_boxes = [], []
    pieces = groove_edge(0.0, TILE_D * 1000)
    for key, outline in pieces.items():
        if key == "bump":
            continue
        xs = [p[0] / 1000 for p in outline]
        zs = [p[1] / 1000 for p in outline]
        proxy_boxes.append(box(((min(xs) + max(xs)) / 2, 0.150, (min(zs) + max(zs)) / 2),
                         (max(xs) - min(xs), 0.900, max(zs) - min(zs))))
    for tile_name, (ya, yb) in (("cut", (-0.300, 0.000)), ("full", (0.000, 0.600))):
        for key, outline in pieces.items():
            color = LAID if key in ("lip", "body") else CORE
            for k, (y0, y1) in enumerate(_segments(outline, ya, yb)):
                proxy = tuple(proxy_boxes) if (tile_name == "cut" and key == "body" and k == 0) else ()
                out.append(_prism(f"{tile_name}_{key}_{k}", outline, y0, y1, color=color, proxy=proxy))
    return tuple(out)


LAID_ROW = MeshPart("laid_row", (X_SEAM, 0.0, TOP), (), color=LAID, friction=(0.4, 0.3),
                    contact_offset=0.0005, meshes=_laid_meshes())

WALL = Part("wall", (0.480, WALL_FACE - 0.009, TOP + 0.050), [box((0, 0, 0), (0.600, 0.018, 0.100))],
            color=PLY, friction=(0.5, 0.4))
SPACER_X = (0.410, 0.610)        # new row's spacer, and the laid row's spacer under its end
SPACERS = Part("spacers", (0.0, WALL_FACE + SPACER_T / 2, TOP + 0.015),
               [box((x, 0.0, 0.0), (0.040, SPACER_T, 0.030)) for x in SPACER_X],
               color=SPACER_BLUE, friction=(0.5, 0.4))
BATTEN_PART = Part("batten", (BATTEN_FACE - BATTEN[0] / 2, 0.0, TOP + BATTEN[2] / 2), [box((0, 0, 0), BATTEN)],
                   color=PINE, friction=(0.5, 0.4))
PRESSED_PARTS = ("tile",)
OBJECTS = [LAID_ROW, WALL, SPACERS, BATTEN_PART, TILE]


# ------------------------------------------------------------------ tile poses (for the scorer tests and the demo)
def tile_pose(tilt_deg=0.0, dx=0.0, dy=0.0, dz=0.0, roll_deg=0.0, yaw_deg=0.0):
    """(part origin, part quaternion xyzw) for the tile turned tilt_deg about its far top edge (positive lifts
    the near edge), that edge's midpoint at the seam + (dx, dy, dz), then rolled about x and yawed about z
    through that midpoint."""
    pf = np.array((X_SEAM + dx, dy, TOP + TILE_T + dz))
    R_tile = _rz(math.radians(yaw_deg)) @ _rx(math.radians(roll_deg)) @ _ry(math.radians(tilt_deg))
    origin = pf - R_tile @ PF_T
    R_part = R_tile @ R0.T
    return tuple(float(v) for v in origin), tuple(float(v) for v in matrix_to_quat_xyzw(R_part))


GOAL_POSE = tile_pose()
RIDE_POSE = tile_pose(-math.degrees(math.atan(0.0014 / 0.297)), dx=-0.004, dz=0.0014)
OFF_SPACER_POSE = tile_pose(dy=0.0155)
NEAR_SPACER_POSE = tile_pose(dy=0.002)
RACKED_POSE = tile_pose(-0.135, dx=-0.003, dz=0.0007, roll_deg=math.degrees(math.atan(0.0014 / 0.58)),
                        yaw_deg=math.degrees(math.atan(0.006 / 0.58)))
TIPPED_HOME_POSE = tile_pose(20.0)
HOVER_POSE = tile_pose(dz=0.005)
ON_TOP_POSE = tile_pose(dx=0.050, dz=TILE_T)
SHORT_POSE = tile_pose(dx=-0.030)

# ------------------------------------------------------------------ scoring
SEAM_LOCK = 0.0004      # locked: top joint gap at most 0.4 mm ...
LIP_LOCK = 0.0005       # ... and the tile's top within 0.5 mm of the laid tiles' top beside the seam ...
FLAT = 0.001            # ... with its near edge down (within 1 mm of lying flat)
SEAM_ROW = 0.010        # against the row: seam at most 10 mm, near edge within 2 mm of lying flat
FLAT_ROW = 0.002
GAP_FULL = 0.012        # end gap (wall face to the tile's end) for full credit; the spacer gives 9.5 mm
GAP_PART = 0.040
END_Y = 0.290           # seam and lippage are measured 10 mm in from each end
DROP_Z = TOP - 0.050    # sticky zero: the tile off the table
STEP_M = 0.0005
TEST_STICKY = ()


def sticky(key, cond):
    return ever(key, bool(cond) or key in TEST_STICKY)


def tile_frame(tile):
    """World origin and rotation of the tile frame (x near->far, z up from the pad's underside)."""
    return np.asarray(tile.origin, float), tile.rotation() @ R0


def to_world(tile, p_tile):
    o, R = tile_frame(tile)
    return o + R @ np.asarray(p_tile, float)


def joint_state(tile):
    """Per end (y = -0.29, +0.29): seam (m, + = open), lippage (m, + = tile above), near-edge height (m)."""
    out = []
    for y in (-END_Y, END_Y):
        far = to_world(tile, (TILE_D / 2, y, TILE_T))
        near = to_world(tile, (-TILE_D / 2, y, TILE_T))
        out.append((float(X_SEAM - far[0]), float(far[2] - (TOP + TILE_T)), float(near[2] - (TOP + TILE_T))))
    return out


def end_gap(tile):
    return float(to_world(tile, (0.0, -TILE_L / 2, TILE_T / 2))[1] - WALL_FACE)


def assess(tile):
    ends = joint_state(tile)
    locked = [(-0.002 <= s <= SEAM_LOCK and abs(lip) <= LIP_LOCK and near <= FLAT) for s, lip, near in ends]
    at_row = [(-0.002 <= s <= SEAM_ROW and near <= FLAT_ROW) for s, lip, near in ends]
    return locked, at_row, end_gap(tile)


_POSE_EPISODE = None
_POSE_TOKEN = None
_PREVIOUS = None
_QUIET = False


def at_rest(tile):
    """Pose differences between consecutive control steps (contact chatter makes speeds unreliable)."""
    global _POSE_EPISODE, _POSE_TOKEN, _PREVIOUS, _QUIET
    if _RUN["t"] is None:
        return tile.resting()
    if _POSE_EPISODE != _RUN["episode"]:
        _PREVIOUS, _POSE_TOKEN, _QUIET = None, None, False
        _POSE_EPISODE = _RUN["episode"]
    token = (_RUN["step"], float(_RUN["t"]))
    if token != _POSE_TOKEN:
        pose = np.r_[to_world(tile, (-TILE_D / 2, -END_Y, TILE_T)), to_world(tile, (-TILE_D / 2, END_Y, TILE_T)),
                     to_world(tile, (TILE_D / 2, 0.0, TILE_T))]
        _QUIET = False
        if _PREVIOUS is not None:
            step, old = _PREVIOUS
            _QUIET = bool(token[0] == step + 1 and np.all(np.abs(pose - old) < STEP_M))
        _PREVIOUS = (token[0], pose)
        _POSE_TOKEN = token
    return _QUIET


def score(objects):
    tile = objects["tile"]
    dropped = sticky("tile off the table", tile.origin[2] < DROP_Z)
    locked, at_row, gap = assess(tile)
    settled = released(tile) and at_rest(tile)
    seconds = held_for("released and still", settled)
    if dropped or not (settled and seconds + 1e-9 >= STILL_S):
        return 0.0
    if all(locked):
        return 1.0 if gap <= GAP_FULL else 0.75 if gap <= GAP_PART else 0.5
    if any(locked) and all(at_row):
        return 0.5
    return 0.0


def success(objects):
    return score(objects) >= 1.0


# ------------------------------------------------------------------ scripted demo
# Tilted-wrist waypoints: a float yaw (for the offline checker's coarse replay) carrying the full grip rotation
# for the arm's IK, the same pattern the oximeter and hoist demos used.
class GripOrientation(float):
    def __new__(cls, matrix):
        obj = super().__new__(cls, math.atan2(-matrix[0, 0], matrix[1, 0]))
        obj.matrix = np.asarray(matrix, float).copy()
        return obj


_original_grip_rotation = getattr(_motion.grip_rotation, "_tile_original", _motion.grip_rotation)


def _grip_rotation(yaw=0.0):
    matrix = getattr(yaw, "matrix", None)
    if isinstance(matrix, np.ndarray) and matrix.shape == (3, 3):
        return matrix
    return _original_grip_rotation(yaw)


_grip_rotation._tile_original = _original_grip_rotation
_motion.grip_rotation = _grip_rotation


def grip_matrix(pitch, arm):
    """Approach along the tile toward its far edge, pitched down by `pitch` (rad); pads close across the tile."""
    a = np.array((math.cos(pitch), 0.0, -math.sin(pitch)))
    c = np.array((math.sin(pitch), 0.0, math.cos(pitch)))
    if arm == "left":
        c = -c          # the left wrist reaches this pose rolled half a turn from the right one
    return np.column_stack([c, np.cross(a, c), a])


GRIP_T = np.array((-TILE_D / 2 + 0.003, 0.0, TILE_T / 2))   # TCP 3 mm in from the near top edge, mid-thickness
OPEN_PINCH = -0.025                  # pads 50 mm apart for the approach (fully open would reach the table)
DROP_OPEN = -0.025                   # opening that lets the near edge down toward the floor
PUSH_OVER = 0.010                    # commanded past home along +x (the soft arm needs the error to push)
SLIDE_OVER = 0.006                   # commanded past the spacer along -y
TIP_DEG = 20.0
LOWER_DEG = 8.0
MIN_PITCH = 12.0                     # the wrist stays pitched at least this much (gripper body clear of table)
DEMO_PACE = 1.0


def grip_point(arm, tilt_deg, dx, dy, dz):
    pf = np.array((X_SEAM + dx, dy, TOP + TILE_T + dz))
    side = 1.0 if arm == "left" else -1.0
    return pf + _ry(math.radians(tilt_deg)) @ (GRIP_T + np.array((0.0, side * GRIP_Y, 0.0)) - PF_T)


def demo():
    G = GripOrientation
    plan = {"left": [], "right": []}
    st = {}

    def add(dt, label, back=0.0, up=0.0, **changes):
        st.update(changes)
        p = math.radians(st["pitch"])
        a = np.array((math.cos(p), 0.0, -math.sin(p)))
        for arm in ("left", "right"):
            pos = grip_point(arm, st["tilt"], st["dx"], st["dy"], st["dz"]) - back * a + np.array((0.0, 0.0, up))
            plan[arm].append(Waypoint(dt * DEMO_PACE, tuple(float(v) for v in pos), st["finger"], label,
                                      G(grip_matrix(p, arm))))

    th0 = math.degrees(TILT0)
    st.update(tilt=th0, dx=START_DX, dy=START_DY, dz=START_DZ, pitch=th0, finger=OPEN_PINCH)
    add(3.0, "approach behind the near edge", back=0.060, up=0.030)
    add(1.5, "line up with the near edge", back=0.035)
    add(1.5, "pads over and under the near edge", back=-0.002)
    add(1.0, "pinch the near edge", back=-0.002, finger=FINGER_CLOSED)
    # Tip up while leaning forward, so the tongue stays on the bump instead of being drawn back off it.
    add(1.5, "tip up on the tongue, leaning in", tilt=14.0, dx=-0.005, dz=0.0011, pitch=14.0)
    add(1.5, "tip up to 20 deg, tongue over the bump", tilt=TIP_DEG, dx=-0.003, dz=0.0005, pitch=TIP_DEG)
    add(2.0, "slide it along the row to the spacer", dy=-SLIDE_OVER)
    add(2.5, "push the tongue home under the lip", dx=PUSH_OVER, dz=0.0)
    add(0.8, "hold it home")
    add(2.5, "lower it toward flat", tilt=LOWER_DEG, dx=0.004, pitch=MIN_PITCH)
    add(1.2, "open: the near edge drops", finger=DROP_OPEN)
    add(1.5, "draw the pads back out", back=0.050)
    add(1.0, "rise clear", back=0.070, up=0.060)
    add(3.5, "stillness tail", back=0.070, up=0.060)
    return plan


def demo_duration():
    return sum(w.duration for w in demo()["right"])
