"""Check a bent steel tube on its checking fixture: lower it level with both hands so all four legs drop into their
nest blocks together, then let go.

On the bench (the station table) stands a tube checking fixture, bolted down: a 1/2 in aluminium base plate carrying
four black acetal nest blocks, one under each horizontal leg of one bent-tube part number. Each block has an open-top
slot 14.7 mm wide (the 12.7 mm tube plus the part's +-1.0 mm profile tolerance) with 45-degree lead-in chamfers; the
blocks' heights set the tube's three leg heights. A tube that is within tolerance drops into all four slots under its
own weight, each leg lying on its slot's floor; offered one leg at a time, a rigid 3-D tube catches on a block's rim.

The tube (1/2 in x 0.035 in welded steel tube, yellow zinc plated, five bends on a 1.5 in centre-line radius, 535 mm
developed, 0.139 kg) lies on the bench 115 mm in front of the fixture, turned the way it sits in the fixture: it rests
on its two low legs with its left end leg 35 mm and its right end leg 55 mm up. Metres; x forward from the arm bases,
y left, z up; zero jitter for demos.
"""
import math
import pathlib
import sys
from collections import deque

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, ball, box, cyl  # noqa: E402
from yam_mesh import Mesh, MeshPart  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

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


TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S, SCORE_OVER = 1.0, "end"
RUN_LIMIT_STEPS = 2400  # 120 s at 20 Hz
DEMO_PACE = 1.0  # scales every demo duration except the fingers' close and open (0.8 for the timing-margin copy)

YELLOW_ZINC, ACETAL_BLACK, ALUMINIUM = (0.80, 0.68, 0.30), (0.07, 0.07, 0.08), (0.72, 0.73, 0.75)


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def _rot(axis, angle):
    """Rotation matrix about a unit axis (Rodrigues)."""
    x, y, z = axis
    k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


# ---------------------------------------------------------------- the tube
# 1/2 in OD x 0.035 in wall welded low-carbon steel tube (SAE J526), yellow zinc plated, bent on a rotary-draw bender
# with a 1.5 in (38.1 mm) centre-line-radius die. 0.1738 lb/ft = 0.2587 kg/m.
OD, TUBE_WALL, CLR = 0.0127, 0.000889, 0.0381
R = OD / 2
KG_PER_M = 0.2587
N_SIDES = 24  # each convex piece's cross-section: a 24-gon whose flats touch the true circle (48 vertices <= 64)
BEND_STEP = math.radians(12.0)  # bends are cut into wedges of at most 12 degrees (0.21 mm chord sag)
# The part's centre line as the bender's intersection points (XYZ data), where it sits in the fixture: world x and y,
# z above the table top. A and E are the cut ends; between them five bends.
TUBE_X0 = 0.270  # the tube frame's origin when seated: (TUBE_X0, 0, table top), axes along the world's
IP_SEATED = np.array([
    (0.270, 0.227, 0.06135),   # A: left end
    (0.270, 0.145, 0.06135),   # bend 1
    (0.315, 0.090, 0.02635),   # bend 2
    (0.235, 0.000, 0.02635),   # bend 3
    (0.315, -0.090, 0.02635),  # bend 4
    (0.270, -0.147, 0.08135),  # bend 5
    (0.270, -0.229, 0.08135),  # E: right end
])
IP = IP_SEATED - np.array([TUBE_X0, 0.0, 0.0])  # in the tube frame


def _centreline(ip):
    """Straights (tangent point to tangent point) and bends (tangent-in point, centre, axis, angle, direction in)."""
    d = [_unit(ip[i + 1] - ip[i]) for i in range(len(ip) - 1)]
    straights, bends = [], []
    start = ip[0]
    for k in range(len(d)):
        if k + 1 < len(d):
            angle = math.acos(float(np.clip(np.dot(d[k], d[k + 1]), -1.0, 1.0)))
            tangent = CLR * math.tan(angle / 2)
            end = ip[k + 1] - d[k] * tangent
            straights.append((start, end))
            inward = _unit(d[k + 1] - d[k] * np.dot(d[k + 1], d[k]))
            bends.append((end, end + inward * CLR, _unit(np.cross(d[k], d[k + 1])), angle, d[k]))
            start = ip[k + 1] + d[k + 1] * tangent
        else:
            straights.append((start, ip[-1]))
    return d, straights, bends


DIRS, STRAIGHTS, BENDS = _centreline(IP)


def bend_point(b, s):
    """(point, tangent) at angle s into bend b."""
    t_in, centre, axis, _, u = b
    rot = _rot(axis, s)
    return centre + rot @ (t_in - centre), rot @ u


def _samples(step=0.002):
    pts = []
    for k, (a, b) in enumerate(STRAIGHTS):
        n = max(2, int(math.ceil(np.linalg.norm(b - a) / step)))
        pts += [a + (b - a) * i / n for i in range(n)]
        if k < len(BENDS):
            angle = BENDS[k][3]
            m = max(2, int(math.ceil(CLR * angle / step)))
            pts += [bend_point(BENDS[k], angle * i / m)[0] for i in range(m)]
    pts.append(STRAIGHTS[-1][1])
    return np.array(pts)


CENTRE_LINE = _samples()  # tube frame, 2 mm apart
DEVELOPED_LENGTH = float(np.linalg.norm(np.diff(CENTRE_LINE, axis=0), axis=1).sum())
TUBE_MASS = round(KG_PER_M * DEVELOPED_LENGTH, 4)


def _ring(p, t):
    """The 24-gon around centre-line point p with tangent t; a flat faces straight down on a horizontal run."""
    e1 = _unit(np.cross(t, (0.0, 0.0, 1.0)))
    e2 = np.cross(e1, t)
    rc = R / math.cos(math.pi / N_SIDES)
    phi = (2 * np.arange(N_SIDES) + 1) * math.pi / N_SIDES
    return p + rc * (np.cos(phi)[:, None] * e1 + np.sin(phi)[:, None] * e2)


def _loft(name, r0, r1, color, proxy=()):
    """A closed convex solid between two rings of equal vertex count, capped at both ends."""
    n = len(r0)
    pts = np.vstack([r0, r1])
    faces = []
    for i in range(n):
        k = (i + 1) % n
        faces += [(i, k, n + k), (i, n + k, n + i)]
    for i in range(1, n - 1):
        faces += [(0, i + 1, i), (n, n + i, n + i + 1)]
    faces = np.array(faces, int)
    tri = (pts - pts.mean(0))[faces]
    if np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum() < 0:
        faces = faces[:, ::-1]
    return Mesh(name, pts, faces, collision="convex", color=color, proxy=tuple(proxy))


def _ball_chain(points):
    return [ball(tuple(float(v) for v in p), R) for p in points]


def _chain_points(a, b, spacing=0.008):
    n = max(1, int(math.ceil(np.linalg.norm(b - a) / spacing)))
    return [a + (b - a) * (i + 0.5) / n for i in range(n)]


# Nest blocks (acetal, 20 long along the leg x 40 wide) and where each end leg is pinched: one block per horizontal
# leg; on each end leg the block sits 4 mm past the bend and the pinch 25 mm past the block (the pads are 13.2 wide).
LEGS = (0, 2, 3, 5)  # the straights that carry a nest block: the left end leg, the two low legs, the right end leg
LEG_NAMES = ("left end leg", "left low leg", "right low leg", "right end leg")
BLOCK_L, BLOCK_W = 0.020, 0.040
BLOCK_PAST_BEND, GRIP_PAST_BLOCK, GRIP_LEN = 0.004, 0.025, 0.026


def _leg_frame(k):
    a, b = STRAIGHTS[k]
    return a, b, _unit(b - a)


def _end_leg_points(k, from_bend_end):
    """(block centre, pinch point) along end leg k, measured from the end that meets the bend."""
    a, b, d = _leg_frame(k)
    bend_end, direction = (a, d) if from_bend_end == "start" else (b, -d)
    block = bend_end + direction * (BLOCK_PAST_BEND + BLOCK_L / 2)
    grip = bend_end + direction * (BLOCK_PAST_BEND + BLOCK_L + GRIP_PAST_BLOCK)
    return block, grip


BLOCK_A, GRIP_A = _end_leg_points(0, "end")    # left end leg: its bend is at its far (b) end
BLOCK_E, GRIP_E = _end_leg_points(5, "start")  # right end leg: its bend is at its near (a) end


def _tube_parts():
    shapes, meshes = [], []
    # The two end legs run along y: real cylinders, each cut into its pinch section and the rest.
    for k, grip in ((0, GRIP_A), (5, GRIP_E)):
        a, b, _ = _leg_frame(k)
        y0, y1 = sorted((a[1], b[1]))
        cuts = sorted((y0, grip[1] - GRIP_LEN / 2, grip[1] + GRIP_LEN / 2, y1))
        for lo, hi in zip(cuts, cuts[1:]):
            if hi - lo > 0.001:
                shapes.append(cyl((a[0], (lo + hi) / 2, a[2]), R, hi - lo, axis="y", color=YELLOW_ZINC))
    # The four sloping and low straights: convex 24-gon prisms. Offline, chains of balls stand in for them.
    for k in (1, 2, 3, 4):
        a, b, d = _leg_frame(k)
        meshes.append(_loft(f"straight_{k + 1}", _ring(a, d), _ring(b, d), YELLOW_ZINC,
                            _ball_chain(_chain_points(a, b))))
    # The five bends: wedges of at most 12 degrees.
    for j, bend in enumerate(BENDS):
        angle = bend[3]
        n = max(1, int(math.ceil(angle / BEND_STEP)))
        for i in range(n):
            s0, s1 = angle * i / n, angle * (i + 1) / n
            p0, t0 = bend_point(bend, s0)
            p1, t1 = bend_point(bend, s1)
            pm = bend_point(bend, (s0 + s1) / 2)[0]
            meshes.append(_loft(f"bend_{j + 1}_{i}", _ring(p0, t0), _ring(p1, t1), YELLOW_ZINC, _ball_chain([pm])))
    return shapes, meshes


TUBE_SHAPES, TUBE_MESHES = _tube_parts()
START_SHIFT = np.array([-0.115, 0.0, -0.020])  # where it lies on the bench, from where it sits in the fixture
TUBE_SEATED = np.array([TUBE_X0, 0.0, TOP])
TUBE_START = TUBE_SEATED + START_SHIFT
TUBE_FRICTION = (0.25, 0.20)  # zinc-plated steel on acetal (estimate)
CONTACT_OFFSET = 0.0008
TUBE = MeshPart("tube", tuple(TUBE_START), shapes=TUBE_SHAPES, mass=TUBE_MASS, color=YELLOW_ZINC,
                friction=TUBE_FRICTION, contact_offset=CONTACT_OFFSET, meshes=TUBE_MESHES)

# ---------------------------------------------------------------- the checking fixture (bolted to the bench)
PLATE_T = 0.0127  # 6061 aluminium tooling plate, 1/2 in
PLATE_X, PLATE_Y = (0.232, 0.322), (-0.205, 0.205)
SLOT_W = OD + 2 * 0.0010  # 14.7 mm: the tube plus the part's +-1.0 mm profile tolerance
SLOT_HALF = SLOT_W / 2
WALL_UP = R + 0.006  # straight slot walls rise 6 mm above a seated tube's centre ...
CHAMFER = 0.006  # ... then 45-degree lead-ins widen the mouth by 6 mm a side (26.7 mm)
SLOT_DEPTH = WALL_UP + CHAMFER
BLOCK_FRICTION = (0.25, 0.20)
PLATE = Part("plate", ((PLATE_X[0] + PLATE_X[1]) / 2, (PLATE_Y[0] + PLATE_Y[1]) / 2, TOP),
             [box((0.0, 0.0, PLATE_T / 2), (PLATE_X[1] - PLATE_X[0], PLATE_Y[1] - PLATE_Y[0], PLATE_T))],
             mass=None, color=ALUMINIUM, friction=BLOCK_FRICTION, contact_offset=CONTACT_OFFSET)


def _wall_section(side, floor):
    """A slot wall's cross-section (block y, z) with its lead-in chamfer, outward-ordered for the +y or -y side."""
    s = 1.0 if side > 0 else -1.0
    return [(s * SLOT_HALF, floor), (s * BLOCK_W / 2, floor), (s * BLOCK_W / 2, floor + SLOT_DEPTH),
            (s * (SLOT_HALF + CHAMFER), floor + SLOT_DEPTH), (s * SLOT_HALF, floor + WALL_UP)]


def _block(index, leg, centre_local):
    """A nest block: frame origin on the table top under the slot's centre, x along the leg; static."""
    a, b, d = _leg_frame(leg)
    heading = math.atan2(d[1], d[0])
    floor = float(centre_local[2]) - R  # a seated leg lies on the slot floor
    world = TUBE_SEATED + centre_local
    shapes = [box((0.0, 0.0, (PLATE_T + floor) / 2), (BLOCK_L, BLOCK_W, floor - PLATE_T))]
    meshes = []
    for side, name in ((1, "wall_left"), (-1, "wall_right")):
        sec = _wall_section(side, floor)
        ring0 = np.array([(-BLOCK_L / 2, y, z) for y, z in sec])
        ring1 = np.array([(BLOCK_L / 2, y, z) for y, z in sec])
        ys = [y for y, _ in sec]
        proxy = box((0.0, (min(ys) + max(ys)) / 2, floor + SLOT_DEPTH / 2), (BLOCK_L, max(ys) - min(ys), SLOT_DEPTH))
        meshes.append(_loft(name, ring0, ring1, ACETAL_BLACK, (proxy,)))
    part = MeshPart(f"nest_{index}", (float(world[0]), float(world[1]), TOP), shapes=shapes, mass=None,
                    color=ACETAL_BLACK, yaw=heading, friction=BLOCK_FRICTION, contact_offset=CONTACT_OFFSET,
                    meshes=meshes)
    return part, {"index": index, "leg": leg, "name": LEG_NAMES[index - 1], "centre": world[:2].copy(),
                  "heading": heading, "floor": TOP + floor}


def _block_centres():
    out = []
    for leg in LEGS:
        if leg == 0:
            out.append(BLOCK_A)
        elif leg == 5:
            out.append(BLOCK_E)
        else:
            a, b, _ = _leg_frame(leg)
            out.append((a + b) / 2)
    return out


_BLOCKS = [_block(i + 1, leg, c) for i, (leg, c) in enumerate(zip(LEGS, _block_centres()))]
NESTS = [spec for _, spec in _BLOCKS]
OBJECTS = [PLATE, *[part for part, _ in _BLOCKS], TUBE]

# ---------------------------------------------------------------- scoring
SEAT_HIGH = 0.0015  # seated: the leg's underside at most 1.5 mm above its slot's floor at the block's middle ...
SEAT_LOW = 0.0020   # ... and no more than 2 mm into it (contact tolerance against a fixed surface)
SEAT_SIDE = 0.0020  # its centre line within 2 mm of the slot's centre line (the walls allow 1.0)
SEAT_TURN_DEG = 10.0  # and the leg running along the slot
STILL_WINDOW, STILL_MOVE = 0.5, 0.0010  # at rest: no reference point moves more than 1 mm over 0.5 s
LINEAR_SPEED, ANGULAR_SPEED = 0.010, 0.5  # offline stand-in for the window
DROP_SPEED = 0.60  # falling faster than this (a free fall of 18 mm) is a dropped tube
TABLE_X, TABLE_Y = (-0.19, 0.91), (-0.65, 0.65)
REF_POINTS = np.array([IP[0], STRAIGHTS[2][1], IP[-1]])  # both cut ends and the low corner


def tube_pose(tube):
    return np.asarray(tube.origin, float), np.asarray(tube.rotation(), float)


def tube_points(tube, local=CENTRE_LINE):
    o, rot = tube_pose(tube)
    return o + np.asarray(local) @ rot.T


def nest_terms(tube):
    """Per nest block: (seated, underside height above the slot floor, centre line off the slot's line)."""
    o, rot = tube_pose(tube)
    out = []
    for nest in NESTS:
        a, b, _ = _leg_frame(nest["leg"])
        a, b = o + rot @ a, o + rot @ b
        d = _unit(b - a)
        e = np.array([math.cos(nest["heading"]), math.sin(nest["heading"])])
        n = np.array([-e[1], e[0]])
        ab = (b - a)[:2]
        span = float(ab @ ab)
        s = float((nest["centre"] - a[:2]) @ ab / span) if span > 1e-12 else -1.0
        p = a + s * (b - a)
        underside = p[2] - R * math.sqrt(max(0.0, 1.0 - d[2] ** 2))
        gap = float(underside - nest["floor"])
        side = float((p[:2] - nest["centre"]) @ n)
        along = abs(float(d[:2] @ e)) / max(float(np.linalg.norm(d[:2])), 1e-9)
        seated = bool(0.0 <= s <= 1.0 and -SEAT_LOW <= gap <= SEAT_HIGH and abs(side) <= SEAT_SIDE
                      and along >= math.cos(math.radians(SEAT_TURN_DEG)))
        out.append({"seated": seated, "gap": gap, "side": side, "s": s})
    return out


def released_tube(tube, clear=RELEASE_CLEAR):
    """Both grippers' centre points at least `clear` from the tube's surface (not its bounding box: the bent tube's
    box spans the whole fixture)."""
    hands = grippers()
    if hands is None:
        return True
    pts = tube_points(tube)
    return all(float(np.min(np.linalg.norm(pts - np.asarray(tcp, float), axis=1))) - R >= clear
               for tcp, _ in hands.values())


_WINDOW = {"episode": None, "hist": deque()}


def at_rest(tube):
    """No reference point of the tube moved more than STILL_MOVE over the last STILL_WINDOW seconds (a pose window,
    not an instantaneous speed, so solver jitter on a resting round tube does not count as motion). Offline, where
    there is no history, the speeds stand in."""
    t = _RUN["t"]
    if t is None:
        return bool(np.linalg.norm(tube.lin_vel) < LINEAR_SPEED and np.linalg.norm(tube.ang_vel) < ANGULAR_SPEED)
    hist = _WINDOW["hist"]
    if _WINDOW["episode"] != _RUN["episode"] or (hist and t < hist[-1][0]):
        hist.clear()
        _WINDOW["episode"] = _RUN["episode"]
    ref = tube_points(tube, REF_POINTS)
    if not hist or t > hist[-1][0] + 1e-9:
        hist.append((t, ref))
    while len(hist) > 1 and hist[1][0] <= t - STILL_WINDOW + 1e-9:
        hist.popleft()
    if t - hist[0][0] < STILL_WINDOW - 0.051:
        return False
    return all(float(np.max(np.linalg.norm(r - ref, axis=1))) <= STILL_MOVE for _, r in hist)


def off_bench(tube):
    pts = tube_points(tube)
    o = pts.mean(0)
    return bool(pts[:, 2].min() < TOP - 0.005 or not TABLE_X[0] <= o[0] <= TABLE_X[1]
                or not TABLE_Y[0] <= o[1] <= TABLE_Y[1])


def score_terms(objects):
    tube = objects["tube"]
    nests = nest_terms(tube)
    seated = sum(n["seated"] for n in nests)
    dropped = ever("tube dropped (fell faster than 0.6 m/s)", float(-tube.lin_vel[2]) > DROP_SPEED)
    fell = ever("tube off the bench", off_bench(tube))
    free = released_tube(tube)
    rest = at_rest(tube)
    four_s = held_for("all four legs seated, released, at rest", seated == 4 and free and rest)
    some_s = held_for("two or three legs seated, released, at rest", 2 <= seated <= 3 and free and rest)
    return {"nests": nests, "seated": seated, "dropped": dropped, "fell": fell, "released": free, "at_rest": rest,
            "four_s": four_s, "some_s": some_s}


def score(objects):
    t = score_terms(objects)
    if t["dropped"] or t["fell"]:
        return 0.0
    if t["seated"] == 4 and t["four_s"] + 1e-10 >= STILL_S:
        return 1.0
    if t["seated"] == 4:
        return 0.5
    if 2 <= t["seated"] <= 3 and t["some_s"] + 1e-10 >= STILL_S:
        return 0.25
    return 0.0


def success(objects):
    return score(objects) >= 1.0


def rolled(angle_deg, pivot_y):
    """Offline test layouts: the seated tube rolled angle_deg about the world x line through y = pivot_y at the low
    slots' floor height (one side's legs dip, the other's rise). Returns (origin, xyzw quaternion), world."""
    a = math.radians(angle_deg)
    pivot = np.array([TUBE_SEATED[0], pivot_y, NESTS[1]["floor"]])
    origin = pivot + _rot(np.array([1.0, 0.0, 0.0]), a) @ (TUBE_SEATED - pivot)
    return [float(v) for v in origin], [math.sin(a / 2), 0.0, 0.0, math.cos(a / 2)]


# ---------------------------------------------------------------- demo
# Both hands at once, one pinch on each end leg (left hand on the left leg, fingers closing across it along x), the
# tube kept level: lift it straight up off the bench, carry it forward over the fixture, lower it until its legs are
# just above the blocks' mouths, then slowly straight down into all four slots together until the legs are 0.5 mm
# above the slot floors, let go (the tube drops the last 0.5 mm under its own weight; it is never pressed in) and rise
# clear. Every move is the same for both hands, so the two pinches never pull against each other.
# Twice the demo reads where the tube is (after the lift, and again just above the blocks) and shifts both hands'
# remaining targets by the difference, so the in-hand sag (a held part rides 3-4 mm low) and any slip are taken out
# before the legs reach the slots.
GRIP_ABOVE = 0.003  # the TCP 3 mm above the leg's centre line: the pad tips 10 mm below it
GRIPS = {"left": GRIP_A + np.array([0.0, 0.0, GRIP_ABOVE]), "right": GRIP_E + np.array([0.0, 0.0, GRIP_ABOVE])}
YAWS = {"left": -math.pi / 2, "right": math.pi / 2}  # fingers close along x, across the legs (wrists within 70 deg)
HOVER, LIFT, ABOVE_NESTS, LET_GO_ABOVE = 0.060, 0.060, 0.020, 0.0005
CORRECTION_MAX = 0.008
PARK = {"left": np.array([0.200, 0.300, TOP + 0.180]), "right": np.array([0.200, -0.300, TOP + 0.180])}
FINAL_HOLD_S = 4.0
_READS = {"episode": None, "values": {}}


def _tube_origin_now():
    env = _RUN["env"]
    if env is None:
        return None
    try:
        from yam_arena import scene as arena_scene
        return np.asarray(arena_scene.object_states(env, ["tube"])["tube"].origin, float)
    except Exception as e:  # noqa: BLE001 - an unreadable pose falls back to the open-loop path
        print(f"TUBE_READ failed: {e!r}", flush=True)
        return None


def _correction(key, planned_origin, prior):
    """The shift added to both hands' targets from this read on: the earlier shift plus (planned - actual tube
    origin), the new part at most 8 mm; read once per episode, the first time a hand reaches a waypoint needing it."""
    if _READS["episode"] != _RUN["episode"]:
        _READS["episode"], _READS["values"] = _RUN["episode"], {}
    values = _READS["values"]
    if key not in values:
        base = prior() if prior else np.zeros(3)
        actual = _tube_origin_now()
        # the hands were sent to planned + grip + base; the tube's offset from that is (actual - planned - base), so
        # the new shift is base + (planned - actual)
        step = np.zeros(3) if actual is None else np.asarray(planned_origin, float) - actual
        n = float(np.linalg.norm(step))
        if n > CORRECTION_MAX:
            step *= CORRECTION_MAX / n
        values[key] = base + step
        if actual is not None:
            print(f"TUBE_READ {key} t={_RUN['t']:.2f} tube_mm={tuple(round(float(v) * 1000, 2) for v in actual)} "
                  f"shift_mm={tuple(round(float(v) * 1000, 2) for v in values[key])}", flush=True)
    return values[key]


class TubeWaypoint(Waypoint):
    """A Waypoint whose position is worked out when it is played (it may include a read correction)."""

    def __init__(self, duration, where, finger, label, yaw, keep=False):
        self.duration = duration * (1.0 if keep else DEMO_PACE)
        self.where, self.finger, self.label, self.yaw = where, finger, label, float(yaw)

    @property
    def pos(self):
        return tuple(float(v) for v in self.where())


def demo_origins():
    """[(label, seconds, tube origin, finger, read key, keep)] for the whole plan; the hands follow the origin."""
    O, C = FINGER_OPEN, FINGER_CLOSED
    start, seated = TUBE_START, TUBE_SEATED
    lifted = start + np.array([0.0, 0.0, LIFT])
    carried = np.array([seated[0], seated[1], lifted[2]])
    above = seated + np.array([0.0, 0.0, ABOVE_NESTS])
    in_slots = seated + np.array([0.0, 0.0, LET_GO_ABOVE])  # every leg inside its slot's walls, 0.5 mm off the floor
    return [
        ("hands over the end legs", 2.5, start + np.array([0.0, 0.0, HOVER]), O, None, False),
        ("pads down beside the legs", 1.5, start, O, None, False),
        ("pinch both legs", 1.2, start, C, None, True),
        ("settle the pinch", 0.4, start, C, None, False),
        ("lift level off the bench", 2.0, lifted, C, None, False),
        ("hold lifted", 0.8, lifted, C, None, False),
        ("carry over the fixture", 3.0, carried, C, "lifted", False),
        ("lower to just above the blocks", 1.5, above, C, "lifted", False),
        ("hold above the blocks", 0.6, above, C, "lifted", False),
        ("lower into all four slots together", 3.0, in_slots, C, "above", False),
        ("hold just above the floors", 0.8, in_slots, C, "above", False),
        ("let go", 1.2, in_slots, O, "above", True),
        ("rise clear", 1.5, seated + np.array([0.0, 0.0, LIFT]), O, "above", False),
    ]


def demo():
    plan = demo_origins()
    planned_at = {"lifted": TUBE_START + np.array([0.0, 0.0, LIFT]),
                  "above": TUBE_SEATED + np.array([0.0, 0.0, ABOVE_NESTS])}
    prior = {"lifted": None, "above": lambda: _correction("lifted", planned_at["lifted"], None)}

    def shift(key):
        return np.zeros(3) if key is None else _correction(key, planned_at[key], prior[key])

    out = {}
    for arm in ("left", "right"):
        wps = []
        for label, secs, origin, finger, key, keep in plan:
            where = (lambda o=np.asarray(origin, float), k=key, g=GRIPS[arm]: o + g + shift(k))
            wps.append(TubeWaypoint(secs, where, finger, label, YAWS[arm], keep))
        wps.append(TubeWaypoint(2.0, lambda p=PARK[arm]: p, FINGER_OPEN, "park", YAWS[arm]))
        wps.append(TubeWaypoint(FINAL_HOLD_S, lambda p=PARK[arm]: p, FINGER_OPEN, "hold clear", YAWS[arm], True))
        out[arm] = wps
    return out
