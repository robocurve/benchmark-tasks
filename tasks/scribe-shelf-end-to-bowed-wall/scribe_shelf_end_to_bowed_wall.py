"""Scribe a pine shelf's end to an out-of-flat wall with a pencil compass.

On the table: a white-painted MDF wall section stands along the robot's left, its face toward the robot's right.
The face is not flat: it has two humps and a hollow. A pine 1x6 shelf board lies flat on the table, its square-cut
end against the wall. A brass-plated pencil compass, locked 10 mm open, hangs by its hinge in a fork stand in front
of the shelf's near-left corner, steel point toward the wall, pencil toward the robot.

Scribing copies the wall's line onto the shelf: hold the shelf against the wall with one hand; with the other run the
compass from the shelf's front edge to its back edge with the steel point riding on the wall and the pencil marking
the shelf's top face, the compass kept upright and square to the wall. Then hang the compass back in its stand and
take both hands away. The trial's seed sets where along the wall the shelf lies, so each trial copies a different
stretch of the wall.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, PartState, ball, box, cyl  # noqa: E402
from yam_mesh import Mesh, MeshPart, prism  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"

# Release-and-stillness block, verbatim from the parts kit's docs (release_stillness_block.py).
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


# ---------------------------------------------------------------- layout (metres; x forward, y left, z up)
TOP = TABLE_TOP[SCENE]
DEMO_PACE = 1.0
SUCCESS_HOLD_S = 1.0
RUN_LIMIT_STEPS = 2400

# Wall section: 25 mm MDF on edge, 260 long, 100 tall, its face routed to the profile below and painted white,
# screwed to a 19 mm MDF foot behind it; the foot is clamped to the table (fixed in the simulation).
WALL_X0, WALL_X1 = 0.100, 0.360
WALL_H = 0.100
Y_FACE0 = 0.212            # face line where the profile offset is zero
Y_BACK = Y_FACE0 + 0.021   # flat back of the board (thickness 0.021 + offset: 19.8 to 25.0 mm)
FOOT_DEPTH, FOOT_T = 0.070, 0.019
# The face stands out from Y_FACE0 toward the shelf by the sum of these smooth bumps (centre x, height, width),
# in metres; negative height is a hollow. The README lists the offset every 5 mm, as cut.
BUMPS = ((0.148, 0.0040, 0.021), (0.228, -0.0012, 0.028), (0.302, 0.0036, 0.025), (0.352, 0.0012, 0.014))
FACE_STEP = 0.002


def wall_offset(x):
    """How far the wall face stands out toward the shelf (-y) at world x, metres."""
    x = np.asarray(x, dtype=float)
    return sum(h * np.exp(-0.5 * ((x - c) / w) ** 2) for c, h, w in BUMPS)


def face_y(x):
    """World y of the wall face at world x."""
    return Y_FACE0 - wall_offset(x)


WALL_POS = np.array([(WALL_X0 + WALL_X1) / 2, Y_FACE0, TOP])
_FACE_X = np.linspace(WALL_X0, WALL_X1, int(round((WALL_X1 - WALL_X0) / FACE_STEP)) + 1)
_outline = [(x - WALL_POS[0], face_y(x) - WALL_POS[1]) for x in _FACE_X]
_outline += [(WALL_X1 - WALL_POS[0], Y_BACK - WALL_POS[1]), (WALL_X0 - WALL_POS[0], Y_BACK - WALL_POS[1])]
# Offline proxies (never colliders): 10 mm slices whose front is the most recessed face point in the slice,
# so a proxy never stands proud of the true face.
_slices = np.linspace(WALL_X0, WALL_X1, 27)
WALL_PROXIES = []
for _a, _b in zip(_slices[:-1], _slices[1:]):
    _front = float(face_y(np.linspace(_a, _b, 11)).max())
    WALL_PROXIES.append(box(((_a + _b) / 2 - WALL_POS[0], (_front + Y_BACK) / 2 - WALL_POS[1], WALL_H / 2),
                            (_b - _a, Y_BACK - _front, WALL_H)))
_face = prism(_outline, WALL_H, name="face", collision="exact", proxy=[box((0, 0, 0), (0.01, 0.01, 0.01))])
WALL_FACE = Mesh("face", _face.points + np.array([0.0, 0.0, WALL_H / 2]), _face.faces, collision="exact",
                 proxy=tuple(WALL_PROXIES))
WALL_WHITE = (0.93, 0.93, 0.90)
WALL = MeshPart("wall", WALL_POS,
                [box(((WALL_X0 + WALL_X1) / 2 - WALL_POS[0], Y_BACK + FOOT_DEPTH / 2 - WALL_POS[1], FOOT_T / 2),
                     (WALL_X1 - WALL_X0, FOOT_DEPTH, FOOT_T), color=(0.62, 0.58, 0.52))],
                color=WALL_WHITE, friction=(0.50, 0.40), contact_offset=0.001, meshes=[WALL_FACE])

# Shelf: planed pine 1x6, 19 x 140 (actual), 300 long, eastern white pine about 400 kg/m3. Frame: origin at the
# centre of its bottom face; x across its depth (front edge at -D/2), y along its length (wall end at +L/2).
SHELF_D, SHELF_L, SHELF_T = 0.140, 0.300, 0.019
SHELF_MASS = 0.32
SHELF_TOP = TOP + SHELF_T
PINE = (0.86, 0.70, 0.45)
END_GAP = 0.00005          # the shelf's end starts this close to the wall's most proud point
LAYOUT_FRONT_X = (0.120, 0.147, 0.174, 0.201)   # front edge x for seed mod 4 = 0, 1, 2, 3


def _window(front_x, n=1401):
    return np.linspace(front_x, front_x + SHELF_D, n)


def shelf_end_y(k):
    """World y of the shelf's end in layout k: touching the wall where the wall stands out most."""
    return float(face_y(_window(LAYOUT_FRONT_X[k])).min()) - END_GAP


def shelf_origin(k):
    return np.array([LAYOUT_FRONT_X[k] + SHELF_D / 2, shelf_end_y(k) - SHELF_L / 2, TOP + 0.001])


SHELF = Part("shelf", shelf_origin(0), [box((0, 0, SHELF_T / 2), (SHELF_D, SHELF_L, SHELF_T))],
             mass=SHELF_MASS, color=PINE, friction=(0.50, 0.40), contact_offset=0.001)

# Compass: wing-nut pencil compass and scriber (General Tools 843/1 class), brass-plated steel, hardened steel pin,
# a 60 mm HB pencil stub in its clamp, locked 10.0 mm open, 168 mm overall, 26 g (the maker lists 0.05 lb without a pencil). Frame: origin at the hinge
# knuckle's centre; x across the compass's thickness; +y toward the point leg; z up along the legs.
BRASS, STEEL, GRAPHITE, PENCIL_YELLOW = (0.80, 0.62, 0.25), (0.55, 0.57, 0.60), (0.18, 0.18, 0.20), (0.95, 0.78, 0.10)
POINT_CENTRE, POINT_R = np.array([0.0, 0.0044, -0.1260]), 0.0006
LEAD_CENTRE, LEAD_R = np.array([0.0, -0.0050, -0.1280]), 0.0006
COMPASS_SHAPES = [
    box((0, 0, 0.0227), (0.009, 0.009, 0.0334)),                       # 0 knurled top grip, 9 mm square
    cyl((0, 0, 0.0), 0.005, 0.014, axis="x"),                          # 1 hinge knuckle
    box((0, 0.0005, -0.0525), (0.003, 0.002, 0.095)),                  # 2 point leg
    box((0, -0.0015, -0.044), (0.003, 0.002, 0.078)),                  # 3 pencil leg
    box((0.0055, 0, -0.040), (0.008, 0.006, 0.006)),                   # 4 wing nut on the locking arc
    cyl((0, 0.0020, -0.111), 0.00125, 0.022, color=STEEL),             # 5 steel pin
    cyl((0, 0.0036, -0.124), 0.0008, 0.004, color=STEEL),              # 6 pin taper
    ball(POINT_CENTRE, POINT_R, color=STEEL),                          # 7 point
    cyl((0, -0.005, -0.089), 0.005, 0.012),                            # 8 pencil clamp ring
    cyl((0, -0.005, -0.101), 0.00375, 0.039, color=PENCIL_YELLOW),     # 9 pencil
    cyl((0, -0.005, -0.1225), 0.0025, 0.004, color=(0.85, 0.72, 0.52)),  # 10 sharpened wood
    cyl((0, -0.005, -0.12625), 0.0012, 0.0035, color=GRAPHITE),        # 11 lead cone
    ball(LEAD_CENTRE, LEAD_R, color=GRAPHITE),                          # 12 lead tip
]
COMPASS_MASS = 0.026
GRIP_FRICTION = (1.0, 0.9)   # the knurled top grip against the finger pads (estimate); the rest of the compass 0.25 / 0.20
SETTING = (POINT_CENTRE[1] + POINT_R) - LEAD_CENTRE[1]   # 10.0 mm, point's wall side to the pencil line
TCP_LOCAL = np.array([0.0, 0.0, 0.021])                  # grip point on the top grip
TCP_TO_LEAD = TCP_LOCAL[2] - (LEAD_CENTRE[2] - LEAD_R)   # 149.6 mm

# Stand: pine base and post with a steel fork at the top; the compass hangs by its knuckle on the two tines,
# legs down between them. Frame: origin on the table under the fork's centre; the fork opens toward -y.
# Each tine carries two small steel steps 6 mm apart: the knuckle sits in the notch between them on their inner
# edges, so it cannot roll along the tines and a swing on the stand dies out by friction at those edges.
STAND_XY = np.array([0.080, 0.110])
FORK_TOP = 0.132                     # tine tops above the table
TINE_X, TINE_W, TINE_T, TINE_Y0, TINE_Y1 = 0.007, 0.006, 0.003, -0.021, 0.021
NOTCH_W, STEP_L, STEP_H = 0.006, 0.002, 0.0015   # gap between a tine's two steps; each step's length and height
KNUCKLE_R = 0.005
STAND = Part("stand", (*STAND_XY, TOP), [
    box((0, 0.045, 0.0095), (0.050, 0.050, 0.019), color=PINE),                         # base
    box((0, 0.030, (0.019 + FORK_TOP) / 2), (0.020, 0.020, FORK_TOP - 0.019), color=PINE),  # post
    box((TINE_X, (TINE_Y0 + TINE_Y1) / 2, FORK_TOP - TINE_T / 2), (TINE_W, TINE_Y1 - TINE_Y0, TINE_T), color=STEEL),
    box((-TINE_X, (TINE_Y0 + TINE_Y1) / 2, FORK_TOP - TINE_T / 2), (TINE_W, TINE_Y1 - TINE_Y0, TINE_T), color=STEEL),
] + [box((sx * TINE_X, sy * (NOTCH_W + STEP_L) / 2, FORK_TOP + STEP_H / 2), (TINE_W, STEP_L, STEP_H), color=STEEL)
     for sx in (-1, 1) for sy in (-1, 1)],
    color=PINE, friction=(0.60, 0.50), contact_offset=0.001)
# knuckle centre when it sits on the steps' inner edges
HANG_ORIGIN = np.array([*STAND_XY, TOP + FORK_TOP + STEP_H + math.sqrt(KNUCKLE_R ** 2 - (NOTCH_W / 2) ** 2)])
COMPASS = Part("compass", HANG_ORIGIN + [0, 0, 0.001], COMPASS_SHAPES, mass=COMPASS_MASS, color=BRASS,
               friction=(0.25, 0.20), contact_offset=0.001, grip=0)

OBJECTS = [WALL, SHELF, COMPASS, STAND]
# The holding hand presses on the shelf's top face; replay cannot move a pressed part, so those touches are advisory.
PRESSED_PARTS = ("shelf",)

# ---------------------------------------------------------------- layouts and the target line
STATIONS = 0.007 + 0.010 * np.arange(14)   # measured from the shelf's front edge
N_STATIONS = len(STATIONS)
TEST_LAYOUT = None
ACTIVE_LAYOUT = 0                          # set from the seed at every reset (seed mod 4)


def current_layout():
    return int(TEST_LAYOUT if TEST_LAYOUT is not None else ACTIVE_LAYOUT)


def wall_gap(k, s):
    """Gap between the wall and the shelf's end at distance s from the front edge, layout k (metres)."""
    return face_y(LAYOUT_FRONT_X[k] + np.asarray(s, dtype=float)) - (shelf_end_y(k) + END_GAP)


def targets(k):
    """Where the scribed line belongs at each station: its distance from the shelf's end (metres)."""
    return SETTING - wall_gap(k, STATIONS)


def _gate_triple(k):
    """The three stations whose middle one sits furthest off the straight line through the outer two."""
    t, best = targets(k), (0.0, (0, 1, 2))
    for a in range(N_STATIONS):
        for c in range(a + 2, N_STATIONS):
            for b in range(a + 1, c):
                dev = t[b] - (t[a] + (t[c] - t[a]) * (STATIONS[b] - STATIONS[a]) / (STATIONS[c] - STATIONS[a]))
                if abs(dev) > abs(best[0]) + 1e-12:
                    best = (dev, (a, b, c))
    return best


GATES = tuple(_gate_triple(k) for k in range(len(LAYOUT_FRONT_X)))

# ---------------------------------------------------------------- scoring constants
MARK_ABOVE, MARK_BELOW = 0.0002, 0.0020    # the lead marks within 0.2 mm above to 2 mm below the top face
STATION_SPREAD = 0.0010                    # crossings of one station further apart than this: unmarked
DOT_HALF = 0.00025                         # a single touch reads at a station within this distance (line credit
                                           # still needs it to be part of one unbroken line: GAP_MAX)
TIERS = ((1.0, N_STATIONS, 0.0005), (0.5, 12, 0.0010), (0.25, 10, 0.0020))
GATE_MIN = 0.0015                          # the line must bend at least this much where the wall bends
GAP_MAX = 0.0020                           # line credit needs one line from edge to edge, through the marks read at
                                           # the stations, with no unmarked stretch longer than this (line_credit)
GAP_SEARCH = 0.0050                        # marks further apart than this are never joined (the gap is then "over 5 mm")
SHOVE_MAX = 0.030
FALL_DEPTH = 0.020
CLEAR_GAP = 0.002


def lead_point(compass):
    """World position of the pencil lead's lowest point."""
    return compass.to_world(LEAD_CENTRE) - np.array([0.0, 0.0, LEAD_R])


def marking(compass, shelf):
    """(True, shelf-local x, y) while the lead touches the shelf's top face."""
    q = shelf.to_local(lead_point(compass))
    on = bool(-MARK_BELOW <= q[2] - SHELF_T <= MARK_ABOVE
              and abs(q[0]) <= SHELF_D / 2 and abs(q[1]) <= SHELF_L / 2)
    return on, q


def station_readings(strokes, crossers=None):
    """Each station's reading: the line's distance from the shelf's end, or None if unmarked or ambiguous.
    crossers, if a list, gets each station's set of crossing strokes (by index)."""
    out = []
    for s in STATIONS:
        xs, ys, hit = -SHELF_D / 2 + s, [], set()
        for n, stroke in enumerate(strokes):
            p = np.asarray(stroke, dtype=float).reshape(-1, 2)
            if len(p) == 0:
                continue
            if len(p) == 1:
                if abs(p[0, 0] - xs) <= DOT_HALF:
                    ys.append(p[0, 1])
                    hit.add(n)
                continue
            a, b = p[:-1], p[1:]
            lo, hi = np.minimum(a[:, 0], b[:, 0]), np.maximum(a[:, 0], b[:, 0])
            along = (lo <= xs) & (xs <= hi)
            if along.any():
                hit.add(n)
            for i in np.flatnonzero(along):
                dx = b[i, 0] - a[i, 0]
                if abs(dx) < 1e-9:
                    ys.extend((a[i, 1], b[i, 1]))
                else:
                    ys.append(a[i, 1] + (xs - a[i, 0]) / dx * (b[i, 1] - a[i, 1]))
        if crossers is not None:
            crossers.append(hit)
        if not ys:
            out.append(None)
            continue
        d = SHELF_L / 2 - np.asarray(ys)
        # crossings within 1 mm read as one band, at its centre; further apart, the station is unmarked
        out.append(None if d.max() - d.min() > STATION_SPREAD + 1e-12 else float((d.max() + d.min()) / 2))
    return out


def line_quality(strokes, k):
    """(credit cap from the line alone, readings, errors, the wall-following gate)."""
    readings, t = station_readings(strokes), targets(k)
    errors = [None if r is None else abs(r - tt) for r, tt in zip(readings, t)]
    dev_t, (a, b, c) = GATES[k]
    gate = False
    if None not in (readings[a], readings[b], readings[c]):
        ra, rb, rc = readings[a], readings[b], readings[c]
        dev = rb - (ra + (rc - ra) * (STATIONS[b] - STATIONS[a]) / (STATIONS[c] - STATIONS[a]))
        gate = bool(dev * dev_t > 0 and abs(dev) >= GATE_MIN - 1e-12)
    cap = 0.0
    if gate:
        for credit, count, tol in TIERS:
            if sum(e is not None and e <= tol + 1e-12 for e in errors) >= count:
                cap = credit
                break
    return cap, readings, errors, gate


def _point_to_segment(p, s0, s1):
    d = s1 - s0
    dd = np.einsum("ij,ij->i", d, d)
    u = np.clip(np.einsum("ij,ij->i", p - s0, d) / np.where(dd > 0, dd, 1.0), 0.0, 1.0)
    return np.linalg.norm(p - (s0 + u[:, None] * d), axis=1)


def _segment_distance(a0, a1, b0, b1):
    """Distance between the 2D segments a0-a1 and b0-b1, pair by pair (0 where they cross)."""
    dist = np.minimum.reduce([_point_to_segment(a0, b0, b1), _point_to_segment(a1, b0, b1),
                              _point_to_segment(b0, a0, a1), _point_to_segment(b1, a0, a1)])

    def side(o, p, q):
        return (p[:, 0] - o[:, 0]) * (q[:, 1] - o[:, 1]) - (p[:, 1] - o[:, 1]) * (q[:, 0] - o[:, 0])
    crossing = (side(b0, b1, a0) * side(b0, b1, a1) < 0) & (side(a0, a1, b0) * side(a0, a1, b1) < 0)
    return np.where(crossing, 0.0, dist)


class Marks:
    """The pencil marks on the shelf's top face, for the line's gap. A stroke is drawn as straight pieces between its
    recorded points (a one-point stroke is a dot). Each new piece is measured once against the earlier strokes, so
    the live record costs one pass over the marks per control step."""

    def __init__(self, strokes=()):
        self.pieces, self.owner, self.n = np.zeros((64, 4)), np.zeros(64, dtype=int), 0
        self.span, self.near, self.tip = {}, {}, {}
        for i, stroke in enumerate(strokes):
            for point in np.asarray(stroke, dtype=float).reshape(-1, 2):
                self.add(i, point)

    def add(self, stroke, point):
        """Point appended to stroke (shelf-local x, y)."""
        p = np.asarray(point, dtype=float)
        a = self.tip.get(stroke, p)
        self.tip[stroke] = p
        lo, hi = min(a[0], p[0]), max(a[0], p[0])
        span = self.span.setdefault(stroke, [lo, hi])
        span[0], span[1] = min(span[0], lo), max(span[1], hi)
        pieces, owner = self.pieces[:self.n], self.owner[:self.n]
        near = ((owner != stroke) & (np.minimum(pieces[:, 0], pieces[:, 2]) <= hi + GAP_SEARCH)
                & (np.maximum(pieces[:, 0], pieces[:, 2]) >= lo - GAP_SEARCH))
        if near.any():
            other = pieces[near]
            d = _segment_distance(np.broadcast_to(a, (len(other), 2)), np.broadcast_to(p, (len(other), 2)),
                                  other[:, :2], other[:, 2:])
            for j in np.unique(owner[near]):
                key = (min(int(j), stroke), max(int(j), stroke))
                self.near[key] = min(self.near.get(key, math.inf), float(d[owner[near] == j].min()))
        if self.n == len(self.owner):
            self.pieces = np.concatenate((self.pieces, np.zeros_like(self.pieces)))
            self.owner = np.concatenate((self.owner, np.zeros_like(self.owner)))
        self.pieces[self.n], self.owner[self.n] = np.r_[a, p], stroke
        self.n += 1

    def gap(self, through=()):
        """The line's longest gap (metres). Follow the marks across the shelf from its front edge to its back edge,
        through every stroke in `through` (the marks read at the stations), stepping from one mark to another across
        unmarked board; the gap is the longest step needed on the route that makes it smallest, counting the step from
        the front edge to the first mark and from the last mark to the back edge. A step is the straight-line distance
        on the board between the two marks' nearest points. One unbroken stroke from edge to edge has gap 0; inf means
        no route with steps of at most GAP_SEARCH."""
        front, back = "front", "back"
        need = {front, back, *through}
        edges = [(d, i, j) for (i, j), d in self.near.items()]
        for i, (lo, hi) in self.span.items():
            edges += [(lo + SHELF_D / 2, front, i), (SHELF_D / 2 - hi, i, back)]
        parent = {}

        def root(x):
            while parent.setdefault(x, x) != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x
        for w, u, v in sorted(edges, key=lambda e: e[0]):
            if w > GAP_SEARCH + 1e-12:
                break
            parent[root(u)] = root(v)
            if len({root(x) for x in need}) == 1:
                return max(float(w), 0.0)
        return math.inf


def line_gap(strokes, through=()):
    """The longest gap of the line through these strokes (Marks.gap); the live record keeps its own Marks."""
    return (_MARKS if strokes is _STROKES else Marks(strokes)).gap(through)


def line_credit(strokes, k):
    """(line credit, the line's longest gap): line_quality's credit when the line read at the stations runs from the
    front edge to the back edge with no gap longer than GAP_MAX; otherwise 0."""
    crossers = []
    readings = station_readings(strokes, crossers)
    gap = line_gap(strokes, set().union(*(c for r, c in zip(readings, crossers) if r is not None)))
    cap = line_quality(strokes, k)[0] if gap <= GAP_MAX + 1e-12 else 0.0
    return cap, gap


def _boxes_apart(a, b, gap):
    lo_a, hi_a, lo_b, hi_b = (np.asarray(v, dtype=float) for v in (a.low, a.high, b.low, b.high))
    return bool(np.any(lo_a > hi_b + gap) or np.any(lo_b > hi_a + gap))


def put_down(compass, shelf, wall):
    """The compass is clear of the shelf and the wall (its box at least 2 mm from theirs)."""
    return _boxes_apart(compass, shelf, CLEAR_GAP) and _boxes_apart(compass, wall, CLEAR_GAP)


# ---------------------------------------------------------------- the pencil's path (recorded every control step)
TEST_STROKES = None
_STROKES = []
_MARKS = Marks()
_PATH = {"episode": None, "token": None, "marking": False, "start": None}


def reset_path():
    global _MARKS
    _STROKES.clear()
    _MARKS = Marks()
    _PATH.update(episode=_RUN["episode"], token=None, marking=False, start=None)


def record_path(objects):
    """Append the lead's shelf-local position while it marks; a lift or a skipped step starts a new stroke."""
    if _RUN["t"] is None:
        return
    if _PATH["episode"] != _RUN["episode"]:
        reset_path()
    token = (_RUN["episode"], _RUN["step"])
    if token == _PATH["token"]:
        return
    previous = _PATH["token"]
    contiguous = previous is not None and token[1] == previous[1] + 1
    shelf, compass = objects["shelf"], objects["compass"]
    if _PATH["start"] is None:
        _PATH["start"] = np.asarray(shelf.origin, dtype=float).copy()
    on, q = marking(compass, shelf)
    if on:
        if not (_PATH["marking"] and contiguous):
            _STROKES.append([])
        _STROKES[-1].append((float(q[0]), float(q[1])))
        _MARKS.add(len(_STROKES) - 1, q[:2])
    elif _PATH["marking"]:
        _stroke_report(objects)
    _PATH["marking"], _PATH["token"] = on, token
    if token[1] % 20 == 0:
        _diagnostic(objects, on, q)


def point_gap(compass):
    """Diagnostic only: the point's distance from the wall face (negative inside it), metres."""
    p = compass.to_world(POINT_CENTRE)
    return float(face_y(p[0]) - (p[1] + POINT_R))


def _stroke_report(objects):
    p = np.asarray(_STROKES[-1]) if _STROKES and _STROKES[-1] else np.zeros((0, 2))
    if len(p):
        print(f"SCRIBE_STROKE t={_RUN['t']:.2f} n={len(p)} x_mm={p[0, 0] * 1000:.1f}..{p[-1, 0] * 1000:.1f} "
              f"layout={current_layout()}", flush=True)


def _diagnostic(objects, on, q):
    shelf, compass = objects["shelf"], objects["compass"]
    _, readings, errors, gate = line_quality(_STROKES, current_layout())
    cap, gap = line_credit(_STROKES, current_layout())
    moved = np.asarray(shelf.origin) - shelf_origin(current_layout())
    worst = max((e for e in errors if e is not None), default=None)
    print(f"SCRIBE t={_RUN['t']:.2f} layout={current_layout()} marking={int(on)} "
          f"lead_local_mm=({q[0] * 1000:.2f},{q[1] * 1000:.2f},{(q[2] - SHELF_T) * 1000:.2f}) "
          f"point_gap_mm={point_gap(compass) * 1000:.2f} compass_tilt_deg={compass.tilt_deg():.2f} "
          f"shelf_moved_mm=({moved[0] * 1000:.2f},{moved[1] * 1000:.2f}) strokes={len(_STROKES)} "
          f"marked={sum(r is not None for r in readings)} worst_err_mm={'-' if worst is None else round(worst * 1000, 2)} "
          f"gate={int(gate)} gap_mm={'over 5' if gap > GAP_SEARCH else round(gap * 1000, 2)} "
          f"cap={cap}", flush=True)


# ---------------------------------------------------------------- scorer
def score(objects):
    record_path(objects)
    wall, shelf, compass = objects["wall"], objects["shelf"], objects["compass"]
    k = current_layout()
    strokes = TEST_STROKES if TEST_STROKES is not None else _STROKES
    shoved = float(np.linalg.norm((np.asarray(shelf.origin) - shelf_origin(k))[:2])) > SHOVE_MAX
    violation = any([ever("shelf_shoved", shoved),
                     ever("shelf_off_table", shelf.origin[2] < TOP - FALL_DEPTH),
                     ever("compass_dropped", compass.origin[2] < TOP - FALL_DEPTH)])
    cap, _ = line_credit(strokes, k)
    finished = (put_down(compass, shelf, wall) and released(compass) and released(shelf)
                and compass.resting() and shelf.resting(max_speed=0.002, max_spin=0.05))
    still_s = held_for("finished", finished and cap >= 1.0 and not violation)
    if violation:
        return 0.0
    if cap >= 1.0:
        return 1.0 if still_s >= STILL_S else 0.75
    return cap


def success(objects):
    return score(objects) >= 1.0


# ---------------------------------------------------------------- offline fixtures (tests only; the demo never uses them)
def ideal_line(k, bias=0.0, x0=0.0, x1=SHELF_D, step=0.0005):
    """The line a square, wall-riding compass draws in layout k, shelf-local, shifted bias metres from the end."""
    s = np.arange(x0, x1 + 1e-9, step)
    d = SETTING - wall_gap(k, s) + bias
    return np.column_stack((-SHELF_D / 2 + s, SHELF_L / 2 - d))


def nominal_strokes(k=0, bias=0.0):
    return [ideal_line(k, bias)]


def straight_strokes(k=0):
    """The best straight line through layout k's targets (least squares)."""
    line = ideal_line(k)
    a = np.polyfit(line[:, 0], line[:, 1], 1)
    return [np.column_stack((line[:, 0], np.polyval(a, line[:, 0])))]


def memorised_strokes(k_from, k_to=0):
    """Layout k_from's line drawn, from memory, on layout k_to's shelf (same shelf-local path)."""
    return nominal_strokes(k_from)


def broken_strokes(k=0, gap=0.0021, at=0.061):
    """Layout k's line with the pencil lifted over `gap` metres of the shelf's depth, starting `at` from the front
    edge (between stations 57 and 67)."""
    return [ideal_line(k, 0.0, 0.0, at), ideal_line(k, 0.0, at + gap, SHELF_D)]


def station_dots(k=0):
    """A single touch on target at each of the 14 stations, nothing between them."""
    return [[(-SHELF_D / 2 + s, SHELF_L / 2 - t)] for s, t in zip(STATIONS, targets(k))]


def dashes_and_dots(k=0, off=0.003):
    """Station dots on target, joined by dashes `off` metres off the line that stop 0.5 mm short of each station."""
    dashes = [ideal_line(k, off, s + 0.0005, s + 0.0095) for s in STATIONS[:-1]]
    return station_dots(k) + [ideal_line(k, off, 0.0, STATIONS[0] - 0.0005)] + dashes + \
        [ideal_line(k, off, STATIONS[-1] + 0.0005, SHELF_D)]


def wavy_strokes(k=0, amplitude=0.0007, period=0.020):
    line = ideal_line(k)
    line[:, 1] += amplitude * np.sin(2 * np.pi * (line[:, 0] + SHELF_D / 2) / period)
    return [line]


def goal_objects(k=0):
    """Hung compass, shelf at layout k (offline goal layout)."""
    from yam_arena.ik import matrix_to_quat_xyzw

    def state(spec, origin):
        lo, hi = spec.aabb(np.asarray(origin, float), np.eye(3))
        return PartState((lo + hi) / 2, matrix_to_quat_xyzw(np.eye(3)), np.zeros(3), np.zeros(3), lo, hi,
                         np.asarray(origin, float))
    return {"wall": state(WALL, WALL.pos), "shelf": state(SHELF, shelf_origin(k) - [0, 0, 0.001]),
            "compass": state(COMPASS, HANG_ORIGIN), "stand": state(STAND, STAND.pos)}


# ---------------------------------------------------------------- scripted demo (positions follow the seed's layout)
PRESS_DOWN = 0.0010      # lead commanded this far into the shelf's top face
PRESS_WALL = 0.004       # point commanded this far into the wall
STROKE_STEP = 0.004      # waypoint spacing along the stroke
STROKE_STEP_S = 0.40     # seconds per step (10 mm/s)
# The lead lands and ends about 2.3-3.8 mm behind the hand on the GPU (the arm's steady error, larger at full reach,
# and the compass leaning under a degree in the pinch), so the hand runs from 4 mm to 3 mm past the back edge: the
# lead lands 0.2-1.7 mm in from the front edge (inside the 2 mm the line may leave unmarked there) and ends at or
# near the back edge.
STROKE_X0, STROKE_X1 = 0.004, SHELF_D + 0.003
LIFT_OFF_FORK = 0.010
OUT_OF_FORK = 0.035
AISLE_X = 0.108          # between the stand (x <= 0.105) and the shelf's front edge (x >= 0.120)
GRIP_Z = HANG_ORIGIN[2] + TCP_LOCAL[2]
Z_DRAW = SHELF_TOP + TCP_TO_LEAD - PRESS_DOWN
Z_OVER_SHELF = SHELF_TOP + TCP_TO_LEAD + 0.008
HOLD_FROM_FRONT, HOLD_Y = 0.040, -0.065
HOLD_PRESS = 0.003       # holding fingertips commanded this far into the shelf's top face
RIGHT_WAIT = np.array([0.180, -0.220, TOP + 0.180])   # the right hand waits here while the left carries the compass in
HOLD_ABOVE = SHELF_TOP + 0.060


def tcp_on_wall(k, s, into_wall):
    """Left TCP that puts the pencil at distance s from the front edge with the point into_wall past the face."""
    x = LAYOUT_FRONT_X[k] + s
    return np.array([x, face_y(x) - (POINT_CENTRE[1] + POINT_R) + into_wall, Z_DRAW])


def knuckle_offset():
    """Where the compass's knuckle sits under the left hand, minus where an unturned compass's knuckle would be
    (x and y, metres; zero offline). The stroke can turn the compass a few degrees in the pinch; the demo reads the
    simulator's compass and hand poses once, after the stroke, so it aims the knuckle (not the hand) at the fork."""
    env, hands = _RUN["env"], grippers()
    if env is None or not hands:
        return np.zeros(3)
    try:
        from yam_arena import scene as arena_scene
        knuckle = np.asarray(arena_scene.object_states(env, ["compass"])["compass"].origin, dtype=float)
        d = knuckle - (np.asarray(hands["left"][0], dtype=float) - TCP_LOCAL)
        d[2] = 0.0
        n = float(np.linalg.norm(d))
        if n > 0.008:
            d *= 0.008 / n
        print(f"SCRIBE_REHANG t={_RUN['t']:.2f} knuckle_offset_mm=({d[0] * 1000:.2f},{d[1] * 1000:.2f})", flush=True)
        return d
    except Exception as e:  # noqa: BLE001 - an unreadable pose falls back to the open-loop path
        print(f"SCRIBE_REHANG failed: {e!r}", flush=True)
        return np.zeros(3)


class LayoutWaypoint(Waypoint):
    """A Waypoint whose position is worked out when played, from the layout the seed chose at reset."""

    def __init__(self, duration, where, finger, label, yaw=0.0, keep=False):
        self.duration = duration * (1.0 if keep else DEMO_PACE)
        self.where, self.finger, self.label, self.yaw = where, finger, label, yaw

    @property
    def pos(self):
        return tuple(float(v) for v in self.where(current_layout()))


def demo():
    """Left arm: compass off the stand, one stroke along the wall, compass back. Right arm: holds the shelf down."""
    plans, clock = {"left": [], "right": []}, {"left": 0.0, "right": 0.0}
    shift = {"d": None}

    def hung(p):
        """A fork waypoint for the knuckle: the hand target less the knuckle's offset in the hand (read once)."""
        def where(k):
            if shift["d"] is None:
                shift["d"] = knuckle_offset()
            return np.asarray(p, dtype=float) - shift["d"]
        return where

    def add(arm, dt, where, label, finger=FINGER_CLOSED, keep=False):
        """keep: this move keeps its nominal duration at any DEMO_PACE (the fingers closing, the pencil's touchdown)."""
        fn = where if callable(where) else (lambda k, p=np.asarray(where, float): p)
        plans[arm].append(LayoutWaypoint(dt, fn, finger, label, keep=keep))
        clock[arm] += plans[arm][-1].duration

    def wait_until(arm, t, label):
        """Hold the last target until time t (seconds of the run, already paced)."""
        if t > clock[arm] + 1e-9:
            last = plans[arm][-1]
            add(arm, t - clock[arm], last.where, label, last.finger, keep=True)

    grip = np.array([*STAND_XY, GRIP_Z])
    out = grip + [0, -OUT_OF_FORK, LIFT_OFF_FORK]
    over_out = np.array([out[0], out[1], Z_OVER_SHELF])
    beside = np.array([AISLE_X, out[1], Z_OVER_SHELF])   # clear of the stand in x before turning toward the wall
    add("left", 1.6, grip + [0, 0, 0.025], "compass hover", FINGER_OPEN)
    add("left", 1.0, grip, "compass descend", FINGER_OPEN)
    add("left", 0.8, grip, "compass close", keep=True)   # the fingers need the full 0.8 s to close at any pace
    add("left", 0.8, grip + [0, 0, LIFT_OFF_FORK], "lift off the tines")
    add("left", 1.0, out, "out of the fork")
    add("left", 0.8, over_out, "rise")
    add("left", 0.8, beside, "clear of the stand")
    add("left", 1.8, lambda k: tcp_on_wall(k, STROKE_X0, -0.002) + [0, 0, Z_OVER_SHELF - Z_DRAW], "to the front edge",
        keep=True)
    t_down = clock["left"]
    add("left", 1.0, lambda k: tcp_on_wall(k, STROKE_X0, -0.002), "pencil down", keep=True)
    add("left", 0.8, lambda k: tcp_on_wall(k, STROKE_X0, PRESS_WALL), "point onto the wall", keep=True)
    add("left", 0.4, lambda k: tcp_on_wall(k, STROKE_X0, PRESS_WALL), "settle", keep=True)
    n = int(round((STROKE_X1 - STROKE_X0) / STROKE_STEP))
    for i in range(1, n + 1):
        add("left", STROKE_STEP_S, lambda k, s=STROKE_X0 + i * (STROKE_X1 - STROKE_X0) / n: tcp_on_wall(k, s, PRESS_WALL),
            "scribe")
    add("left", 0.6, lambda k: tcp_on_wall(k, STROKE_X1, PRESS_WALL), "dwell")
    up = PRESS_DOWN + 0.003                   # lead 3 mm clear of the board before the point leaves the wall
    add("left", 0.5, lambda k: tcp_on_wall(k, STROKE_X1, PRESS_WALL) + [0, 0, up], "pencil up")
    add("left", 0.6, lambda k: tcp_on_wall(k, STROKE_X1 - 0.010, -0.003) + [0, 0, up], "point off the wall")
    t_off = clock["left"]
    add("left", 0.8, lambda k: tcp_on_wall(k, STROKE_X1 - 0.010, -0.003) + [0, 0, Z_OVER_SHELF - Z_DRAW], "lift")
    add("left", 2.2, beside, "carry back")
    add("left", 0.8, hung(over_out), "in front of the fork")
    add("left", 0.8, hung(out + [0, 0, 0.006 - LIFT_OFF_FORK]), "down to the fork")
    add("left", 1.2, hung(grip + [0, 0, 0.006]), "into the fork")
    add("left", 0.8, hung(grip - [0, 0, 0.001]), "onto the tines")
    add("left", 0.8, hung(grip - [0, 0, 0.001]), "let go", FINGER_OPEN)
    add("left", 1.2, hung(grip + [0.048, 0, 0.004]), "slide the open pads off", FINGER_OPEN)

    hold = lambda k, z: np.array([LAYOUT_FRONT_X[k] + HOLD_FROM_FRONT, HOLD_Y, z])  # noqa: E731
    add("right", t_down - 2.8 * DEMO_PACE, RIGHT_WAIT, "wait clear of the left arm", FINGER_OPEN, keep=True)
    add("right", 1.6, lambda k: hold(k, HOLD_ABOVE + 0.040), "hand over the shelf")
    add("right", 1.0, lambda k: hold(k, HOLD_ABOVE), "lower")
    add("right", 1.0, lambda k: hold(k, SHELF_TOP + PAD_TIP_PAST_TCP - HOLD_PRESS), "press the shelf down")
    wait_until("right", t_off + 0.3, "hold the shelf")
    add("right", 0.8, lambda k: hold(k, HOLD_ABOVE), "hand up")
    add("right", 1.0, lambda k: hold(k, HOLD_ABOVE) + [-0.030, -0.060, 0.020], "park")

    finish = max(clock.values()) + 3.5
    for arm in plans:
        wait_until(arm, finish, "stillness tail")
    return plans


import scribe_runtime  # noqa: E402

scribe_runtime.install(sys.modules.get(__name__) or globals())
