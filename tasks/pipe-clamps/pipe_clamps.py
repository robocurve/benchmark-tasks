"""A 600 mm length of 1/2 in copper tube lies across two black cradles in front of the robot. Behind it, two
self-closing pipe clamps are screwed to a wooden plank, 360 mm apart, mouths up, each with its two arms standing
open. Fix the tube in both clamps as the clamp maker's instructions say: insert the tube into each clamp and press it
in until the clamp double-clicks. The first click holds the tube; the second click locks it, with the arms closed round
it. Both clamps must lock on the tube, with the tube lying straight along the plank through both. Then let go and move
both grippers at least 10 mm from the tube.
"""

import functools
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Joint, Link, Shape, body, hull, mechanism, on_reset, on_step  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
D2R = math.pi / 180
IN = 0.0254

# -- the tube: 1/2 in Type L copper, ASTM B88 table (Cooney Brothers) [SOURCE] -------------------------------------------
TUBE_R = 0.625 * IN / 2          # OD 5/8 in [SOURCE]
TUBE_L = 0.600                   # a 600 mm length [CHOICE]
TUBE_KG = 0.285 * 0.453592 / 0.3048 * TUBE_L   # .285 lb/ft [SOURCE]: 0.254 kg [ARITH]

# -- the clamps: Oatey 34338 1/2 in CTS self-closing pipe clamp, spec sheet LCS1913B [SOURCE] -----------------------------
CLAMP_B = 1.1 * IN               # B 1.1 in: the base's width across the tube (the sheet's front view, with D) [SOURCE]
CLAMP_H = 1.4 * IN               # H 1.4 in: base to the closed arms' top [SOURCE]
AXIS_H = 0.94 * IN               # h 0.94 in: base to the tube's centre [SOURCE, read as the seated tube's axis height]
CLAMP_L = 0.028                  # along the tube [CHOICE: the sheet's side view gives 0.79 in for 34344 and 1.54 in for
#                                  34345/6, nothing for 34338]
BASE_T = 0.0095                  # the clamp's foot [CHOICE]
JAW_IN = TUBE_R + 0.0005         # closed arms' inner radius about the seated axis [CHOICE]
JAW_OUT = CLAMP_H - AXIS_H       # 11.7 mm, so the closed arms top out at H [ARITH]
JAW_Y = 0.0095                   # arms span +-9.5 mm along the tube
JAW_ARC = (-25.0, 84.0)          # +x arm's arc (deg about the seated axis, 0 = +x); the -x arm is its mirror image
PIVOT_X = 0.012                  # arm pivots, +-x of the centre line
TAIL = 0.010                     # each arm's tail runs 10 mm in from its pivot, under the trigger pad
TAIL_R = 0.001
PAD_T = 0.003
TRAVEL = 0.006                   # the trigger pad's travel from open to locked [CHOICE: the sheet gives none]
CLICK1 = 0.003                   # the first click 3 mm down, the second (lock) at the end of travel [CHOICE]
PAD_TOP_LOCK = AXIS_H - TUBE_R   # the seated tube rests on the pad [ARITH]
PIVOT_Z = PAD_TOP_LOCK - PAD_T - TAIL_R   # the tails lie flat under the locked pad [ARITH]
OPEN = math.asin(TRAVEL / TAIL)  # 36.9 deg: the arms' opening when the pad is up [ARITH]
PAD_X, PAD_Y0, PAD_Y1 = 0.006, -0.0095, 0.0135
BAR_Y = (0.0105, 0.0135)         # the detent bar hangs from the pad's +y end, past the arms
BAR_X0, BAR_X1 = -0.002, 0.002   # the bar's +x face carries two V notches
BAR_H = 0.010
NOTCH_D = 0.0008      # the lock notch's depth [CHOICE]
NOTCH_D1 = 0.0004     # the first-click notch's depth [CHOICE]. WHY half the lock notch: Oatey's first click "holds
#                       pipe in place with the ability to position", so it is the lighter of the two
NOTCH_HALF = 0.0012   # both notches' half-width
PLUNGER_R = 0.0015
PLUNGER_Z = PAD_TOP_LOCK - PAD_T + TRAVEL - BAR_H + 0.002   # the plunger's height (clamp frame) [ARITH test]
K_PAD = 100.0     # the pad's return spring (N/m), preloaded up against its top stop [CHOICE]
PAD_PRE = 0.002
K_JAW = 0.004     # each arm's opening spring (N m/rad), preloaded open [CHOICE]
JAW_PRE = 0.3
K_PLUNGER = 3000.0
PLUNGER_PRE = 0.0030   # the detent plunger [CHOICE]: stiffness works in this stack, joint friction
#                                           does not (LESSONS 2026-11-09). WHY 3 mm: in a notch it presses 7.5 N, so
#                                           the 34 deg flanks hold about 5 N against the 1.5 N the pad spring and arms push up
SEAT_CLEAR = 0.00005

# -- layout --------------------------------------------------------------------------------------------------------------
PLANK = (0.0381, 0.1397)         # a 2x6: 1-1/2 x 5-1/2 in dressed [CHOICE of stock]
PLANK_X = 0.335                  # the clamps' centre line
Z0 = TOP + PLANK[0]              # the plank's top: the clamps' base
CLAMP_Y = (0.180, -0.180)        # 360 mm apart [CHOICE]
CLAMPS = ("L", "R")
TUBE_START = (0.200, 0.0, TOP + 0.030)   # the tube's centre, axis along y, on two V cradles
CRADLE_Y = (0.200, -0.200)

BROWN, COPPER, WHITE, BLACK, GREY = ((0.55, 0.40, 0.25), (0.72, 0.42, 0.25), (0.92, 0.92, 0.90), (0.08, 0.08, 0.08),
                                     (0.55, 0.55, 0.58))
MATERIALS = {"default": (1.2, 1.0),   # the scene's friction on its own assets (rules.md 3)
             "nylon": (0.35, 0.30),   # PA6 on copper [EST]
             "copper": (0.35, 0.30),  # the tube: the same PA6-on-copper pair [EST]
             "slick": (0.10, 0.10)}   # the detent plunger and bar


def _box(lo, hi, color, material="default", mass=0.0):
    c = tuple(float(v) for v in (np.asarray(lo, float) + np.asarray(hi, float)) / 2)
    s = tuple(float(v) for v in np.asarray(hi, float) - np.asarray(lo, float))
    return Shape("box", c, s, color, material=material, mass=mass)


def _prism_y(pts_xz, y0, y1, color, material="default", mass=0.0):
    """A convex prism along y from a convex (x, z) polygon (any winding)."""
    p = np.asarray(pts_xz, float)
    area = sum(z0 * x1 - x0 * z1 for (x0, z0), (x1, z1) in zip(p, np.roll(p, -1, axis=0)))
    if area < 0:
        p = p[::-1]
    ring = lambda y: [(float(x), float(y), float(z)) for x, z in p]  # noqa: E731
    return hull(ring(y0) + ring(y1), color, material=material, mass=mass)


def _turn(pts_xz, pivot, a):
    """Points (x, z) turned by a about +y round pivot (x' = x cos a + z sin a, z' = -x sin a + z cos a)."""
    d = np.asarray(pts_xz, float) - pivot
    c, s = math.cos(a), math.sin(a)
    return np.c_[d[:, 0] * c + d[:, 1] * s, -d[:, 0] * s + d[:, 1] * c] + pivot


def jaw_profiles(side, opening):
    """The convex (x, z) pieces of one arm in the clamp frame (origin on the plank, centre line x = 0), opened by
    ``opening`` rad from closed. side +1: the +x arm; -1: its mirror image."""
    t0, t1 = (a * D2R for a in JAW_ARC)
    ts = np.linspace(t0, t1, 8)
    pieces = []
    for a, b in zip(ts[:-1], ts[1:]):
        pieces.append([(r * math.cos(t), AXIS_H + r * math.sin(t)) for r, t in
                       ((JAW_IN, a), (JAW_OUT, a), (JAW_OUT, b), (JAW_IN, b))])
    lo = (JAW_IN * math.cos(t0), AXIS_H + JAW_IN * math.sin(t0))
    lo2 = (JAW_OUT * math.cos(t0), AXIS_H + JAW_OUT * math.sin(t0))
    pieces.append([lo, lo2, (PIVOT_X + 0.0015, PIVOT_Z), (PIVOT_X - 0.0015, PIVOT_Z)])   # strut to the pivot
    pieces.append([(PIVOT_X, PIVOT_Z - TAIL_R), (PIVOT_X, PIVOT_Z + TAIL_R),              # the tail
                   (PIVOT_X - TAIL, PIVOT_Z + TAIL_R), (PIVOT_X - TAIL, PIVOT_Z - TAIL_R)])
    out = []
    for p in pieces:
        q = _turn(p, np.array([PIVOT_X, PIVOT_Z]), opening)
        q[:, 0] *= side
        out.append(q)
    return out


def notch_centres():
    """Clamp-frame heights of the bar's two notches at rest: they reach the plunger at the two clicks."""
    return (PLUNGER_Z + CLICK1, PLUNGER_Z + TRAVEL)


def bar_face_x(z):
    """The detent bar's face x at rest-frame height z: BAR_X1, or inside a V notch its flank."""
    x = BAR_X1
    for c, dep in zip(notch_centres(), (NOTCH_D1, NOTCH_D)):
        x = min(x, BAR_X1 - dep * max(0.0, 1.0 - abs(z - c) / NOTCH_HALF))
    return x


def bar_profiles():
    z_top = PAD_TOP_LOCK - PAD_T + TRAVEL
    zs = sorted({z_top - BAR_H, z_top} | {c + d for c in notch_centres() for d in (-NOTCH_HALF, 0.0, NOTCH_HALF)})
    return [[(BAR_X0, a), (bar_face_x(a), a), (bar_face_x(b), b), (BAR_X0, b)] for a, b in zip(zs[:-1], zs[1:])]


def tip_seat_x():
    """x of the plunger tip's centre touching both flanks of a notch (both flanks at the same slope)."""
    s = NOTCH_D / NOTCH_HALF
    return BAR_X1 - NOTCH_D + PLUNGER_R * math.sqrt(1 + s * s)


def clamp_parts(k):
    """(base shapes, links, joints) of clamp k ("L" or "R") at y = CLAMP_Y[k], in the world frame."""
    cy = CLAMP_Y[CLAMPS.index(k)]
    w = lambda x, y, z: (PLANK_X + x, cy + y, Z0 + z)  # noqa: E731
    base = [_box(w(-CLAMP_B / 2, -CLAMP_L / 2, 0), w(CLAMP_B / 2, 0.0095, BASE_T), WHITE, "nylon"),
            _box(w(-CLAMP_B / 2, 0.0095, 0), w(-0.004, CLAMP_L / 2, BASE_T), WHITE, "nylon"),
            _box(w(0.004 + 2 * PLUNGER_R + 0.004, 0.0095, 0), w(CLAMP_B / 2, CLAMP_L / 2, BASE_T), WHITE, "nylon")]
    links, joints = [], []
    for side, tag in ((1, "p"), (-1, "m")):
        shapes = [_prism_y(np.c_[q[:, 0] + PLANK_X, q[:, 1] + Z0], cy - JAW_Y, cy + JAW_Y, WHITE, "nylon", 0.0005)
                  for q in jaw_profiles(side, OPEN)]
        links.append(Link(f"jaw{k}{tag}", 0.0005 * len(shapes), tuple(shapes)))
        # opening the +x arm turns it +y; its spring pulls it open, past its open stop (preload)
        joints.append(Joint(f"jaw{k}{tag}", "revolute", f"jaw{k}{tag}", w(side * PIVOT_X, 0.0, PIVOT_Z), "Y",
                            *((-OPEN - 0.05, 0.0) if side > 0 else (0.0, OPEN + 0.05)),
                            stiffness=K_JAW, damping=0.0004, target=side * JAW_PRE))
    zt = PAD_TOP_LOCK + TRAVEL
    pad = [_box(w(-PAD_X, PAD_Y0, zt - PAD_T), w(PAD_X, PAD_Y1, zt), WHITE, "nylon", 0.002)]
    pad += [_prism_y([(PLANK_X + x, Z0 + z) for x, z in p], cy + BAR_Y[0], cy + BAR_Y[1], WHITE, "slick", 0.0005)
            for p in bar_profiles()]
    links.append(Link(f"pad{k}", 0.002 + 0.0005 * len(bar_profiles()), tuple(pad)))
    joints.append(Joint(f"pad{k}", "prismatic", f"pad{k}", w(0.0, 0.0, zt), "Z", -(TRAVEL + 0.0002), 0.0,
                        stiffness=K_PAD, damping=0.5, target=PAD_PRE))
    x_tip = BAR_X1 + PLUNGER_R + SEAT_CLEAR          # on the flat face at rest
    plunger = [Shape("cylinder", w(x_tip, sum(BAR_Y) / 2, PLUNGER_Z), (PLUNGER_R, BAR_Y[1] - BAR_Y[0]), GREY,
                     axis="Y", material="slick", mass=0.002),
               _box(w(x_tip, BAR_Y[0], PLUNGER_Z - 0.001), w(x_tip + 0.006, BAR_Y[1], PLUNGER_Z + 0.001), GREY,
                    "slick", 0.001)]
    links.append(Link(f"plunger{k}", 0.003, tuple(plunger)))
    joints.append(Joint(f"plunger{k}", "prismatic", f"plunger{k}", w(x_tip, sum(BAR_Y) / 2, PLUNGER_Z), "X",
                        -0.0040, 0.0002, stiffness=K_PLUNGER, damping=4.0, target=-PLUNGER_PRE))
    return base, links, joints


def cradle_shapes(y):
    """A 90 deg V block carrying the tube's start axis."""
    x, _, z = TUBE_START
    r = TUBE_R
    out = []
    for s in (-1.0, 1.0):
        pts = [(0.0015, TOP - z), (0.020, TOP - z), (0.020, -r / math.sqrt(2)), (r / math.sqrt(2), -r / math.sqrt(2)),
               (0.0015, 0.0015 - math.sqrt(2) * r)]
        out.append(_prism_y([(x + s * a, z + b) for a, b in pts], y - 0.015, y + 0.015, BLACK))
    return out


def _fixture():
    base_shapes = [_box((PLANK_X - PLANK[1] / 2, -0.30, TOP), (PLANK_X + PLANK[1] / 2, 0.30, Z0), BROWN)]
    links, joints = [], []
    for k in CLAMPS:
        b, l_, j = clamp_parts(k)
        base_shapes += b
        links += l_
        joints += j
    for y in CRADLE_Y:
        base_shapes += cradle_shapes(y)
    return mechanism("fixture", (0.0, 0.0, 0.0), Link("fixture", 10.0, tuple(base_shapes)), links, joints, MATERIALS)


FIXTURE = _fixture()
TUBE = body("tube", TUBE_START, [Shape("cylinder", (0.0, 0.0, 0.0), (TUBE_R, TUBE_L), COPPER, axis="Y",
                                       material="copper", mass=TUBE_KG)], MATERIALS, color=COPPER)
OBJECTS = [FIXTURE, TUBE]

# -- scoring --------------------------------------------------------------------------------------------------------------
CLICK_TOL = 0.0008     # m: the pad down to within 0.8 mm of a click's depth counts as clicked (the notch half width 1.2 mm)
CLOSED_TOL = 3.0       # deg: each arm within 3 deg of closed when locked
SEAT_TOL = 0.0012      # m: the tube's axis within 1.2 mm of the seated axis at each clamp (x and z)
STRAIGHT_DEG = 3.0     # deg: the tube's axis within 3 deg of the plank's line
CLEAR_MIN = 0.010      # m: both grippers this far from the tube
STILL_V, STILL_W = 0.005, 0.05
LOST_Z = TOP - 0.030


def _rot3(q):
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tube_axis(objects):
    return _rot3(objects["tube"].quat_xyzw) @ np.array([0.0, 1.0, 0.0])


def tube_at(objects, y):
    """The tube's axis point at world y, or None when the axis runs within 60 deg of across the plank."""
    t = objects["tube"]
    a = tube_axis(objects)
    if abs(a[1]) < math.cos(60 * D2R):
        return None
    p0 = np.asarray(t.pos, float)
    s = (y - p0[1]) / a[1]
    if abs(s) > TUBE_L / 2 - JAW_Y:     # WHY: the tube must run right through the clamp's arms, not end inside them
        return None
    return p0 + s * a


def seated(objects, k):
    """The tube's axis passes through clamp k within SEAT_TOL of the seated axis, and the tube lies straight."""
    p = tube_at(objects, CLAMP_Y[CLAMPS.index(k)])
    if p is None:
        return False
    a = tube_axis(objects)
    straight = math.degrees(math.acos(min(1.0, abs(float(a[1]))))) <= STRAIGHT_DEG
    return bool(straight and abs(p[0] - PLANK_X) <= SEAT_TOL and abs(p[2] - (Z0 + AXIS_H)) <= SEAT_TOL)


def pad_depth(objects, k):
    return -float(objects["fixture"].joints[f"pad{k}"])


def jaws_closed(objects, k):
    j = objects["fixture"].joints
    return bool(abs(OPEN + j[f"jaw{k}p"]) <= CLOSED_TOL * D2R and abs(OPEN - j[f"jaw{k}m"]) <= CLOSED_TOL * D2R)


def clicks(objects, k):
    """0, 1 or 2: the clicks clamp k has made (2 = locked: the pad at its last notch and both arms closed)."""
    d = pad_depth(objects, k)
    if d >= TRAVEL - CLICK_TOL and jaws_closed(objects, k):
        return 2
    return 1 if d >= CLICK1 - CLICK_TOL else 0


def pose(objects):
    t = objects["tube"]
    return np.asarray(t.pos, float), tube_axis(objects)


def rates(prev, now, dt):
    (p0, a0), (p1, a1) = prev, now
    return float(np.linalg.norm(p1 - p0)) / dt, math.acos(float(np.clip(a0 @ a1, -1.0, 1.0))) / dt


def still(r):
    return bool(r is not None and r[0] <= STILL_V and r[1] <= STILL_W)


# -- the grippers, read every control step (scene keys left_robot / right_robot) ------------------------------------------
HAND_LINKS = ("gripper", "tip_left", "tip_right")
_HANDS = {"hands": None, "steps": 0, "error": None}


@functools.lru_cache(maxsize=1)
def hand_hulls():
    """Each hand link's URDF collision hulls in its own frame: (link, vertices, distinct outward face normals,
    bounding-sphere centre, radius)."""
    import warnings

    from yam_arena.analysis import link_hulls

    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        hulls = link_hulls()
    out = []
    for name in HAND_LINKS:
        for h in hulls[name]:
            n = np.unique(h.planes[:, :3].round(9), axis=0)
            c = (h.vertices.min(axis=0) + h.vertices.max(axis=0)) / 2
            out.append((name, h.vertices, n, c, float(np.linalg.norm(h.vertices - c, axis=1).max())))
    return tuple(out)


def _np(x):
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


@on_step
def read_hands(env):
    """Both hands' collision hulls in the task frame (world less the env origin)."""
    try:
        base = env.unwrapped
        origin = _np(base.scene.env_origins[0])
        hands = []
        for arm in ("left_robot", "right_robot"):
            robot = base.scene[arm]
            names = list(robot.body_names)
            poses = _np(robot.data.body_link_pose_w)[0]
            for link, v, n, c, r in hand_hulls():
                p = poses[names.index(link)]
                rot = _rot3(p[3:7])
                hands.append((v @ rot.T + p[:3] - origin, n @ rot.T, rot @ c + p[:3] - origin, r))
        _HANDS["hands"] = hands
        _HANDS["error"] = None
    except Exception as e:  # WHY: no hand reading means no clearance, so no full score; never stop the run
        _HANDS["hands"], _HANDS["error"] = None, repr(e)
    _HANDS["steps"] += 1


_CLOCK = {"dt": None}


@on_step
def read_clock(env):
    """This control step's length (s), for still(). Unread means not still."""
    try:
        _CLOCK["dt"] = float(env.unwrapped.step_dt)
    except Exception:  # WHY: never stop the run; no step length means no full score
        _CLOCK["dt"] = None


_PRISM = 16


def _tube_convex(objects):
    """The tube as the 16-sided prism round it (so a gap to it is a lower bound): vertices, normals, centre, radius."""
    t = objects["tube"]
    rot, c = _rot3(t.quat_xyzw), np.asarray(t.pos, float)
    rr, hl = TUBE_R / math.cos(math.pi / _PRISM), TUBE_L / 2
    ang = [2 * math.pi * (k + 0.5) / _PRISM for k in range(_PRISM)]
    loc = np.array([(rr * math.cos(u), y, rr * math.sin(u)) for y in (-hl, hl) for u in ang])
    nl = np.array([(math.cos(2 * math.pi * k / _PRISM), 0.0, math.sin(2 * math.pi * k / _PRISM))
                   for k in range(_PRISM)] + [(0.0, 1.0, 0.0)])
    return loc @ rot.T + c, nl @ rot.T, c, float(np.linalg.norm(loc, axis=1).max())


def _sat_gap(hv, hn, kv, kn):
    """A lower bound on the distance between two convex sets: the largest gap of their projections on either set's
    face normals; negative when they overlap on every one of those axes."""
    ax = np.vstack([hn, kn])
    ph, pk = hv @ ax.T, kv @ ax.T
    return float(np.max(np.maximum(pk.min(axis=0) - ph.max(axis=0), ph.min(axis=0) - pk.max(axis=0))))


def _seg_dist(points, a, b):
    ab = b - a
    t = np.clip((points - a) @ ab / float(ab @ ab), 0.0, 1.0)
    return np.linalg.norm(points - (a + t[:, None] * ab), axis=1)


def hand_gap(objects, hands=None, need=None):
    """Smallest distance (m) from the hands to the tube, -inf when the hands were not read. ``hands`` is an (n, 3)
    array of points (exact distance) or the hulls read_hands stores (a lower bound). With ``need``, the search stops
    at the first hull under it."""
    hs = _HANDS["hands"] if hands is None else hands
    if hs is None or len(hs) == 0:
        return -math.inf
    t = objects["tube"]
    a = tube_axis(objects) * TUBE_L / 2
    p = np.asarray(t.pos, float)
    if isinstance(hs, np.ndarray):
        return float((_seg_dist(hs, p - a, p + a) - TUBE_R).min())
    kv, kn, kc, kr = _tube_convex(objects)
    best = math.inf
    for hv, hn, hc, hr in hs:
        g = float(np.linalg.norm(hc - kc)) - hr - kr
        if g < (CLEAR_MIN + 0.01 if need is None else need):
            g = max(g, _sat_gap(hv, hn, kv, kn))
        if need is not None and g < need:
            return g
        best = min(best, g)
    return best


def clear(objects):
    return bool(hand_gap(objects, need=CLEAR_MIN) >= CLEAR_MIN)


def _fresh():
    return {"last": None, "fail": None, "now": None, "pose": None, "rates": None}


_MEM = _fresh()


@on_reset
def reset_memory():
    _MEM.clear()
    _MEM.update(_fresh())
    _HANDS.update({"hands": None, "steps": 0, "error": None})
    _CLOCK["dt"] = None


def failure(objects):
    """Latched by update(): the tube off the table is lost."""
    if float(objects["tube"].pos[2]) < LOST_Z:
        return "lost"
    return None


def update(objects):
    """One control step: latch a failure and judge this step. WHY the identity check: the embodiment passes the same
    states dict to success() and then score() each step (inspect_embodiment.py _task_step)."""
    if objects is _MEM["last"]:
        return _MEM
    _MEM["last"] = objects
    why = failure(objects)
    if why and not _MEM["fail"]:
        _MEM["fail"] = why
    p, dt = pose(objects), _CLOCK["dt"]
    _MEM["rates"] = rates(_MEM["pose"], p, dt) if _MEM["pose"] is not None and dt else None
    _MEM["pose"] = p
    on = {k: (clicks(objects, k) if seated_or_entered(objects, k) else 0) for k in CLAMPS}
    _MEM["now"] = {"clicks": on, "seated": {k: seated(objects, k) for k in CLAMPS}, "clear": clear(objects),
                   "still": still(_MEM["rates"])}
    return _MEM


def seated_or_entered(objects, k):
    """The tube is in clamp k's mouth: its axis over the pad within 3 mm in x and at most TRAVEL + 1.5 mm over the
    seated height (a click only counts with the tube in the clamp, not a finger on the pad)."""
    p = tube_at(objects, CLAMP_Y[CLAMPS.index(k)])
    if p is None:
        return False
    dz = p[2] - (Z0 + AXIS_H)
    return bool(abs(p[0] - PLANK_X) <= 0.003 and -SEAT_TOL <= dz <= TRAVEL + 0.0015)


def locked_both(m):
    n = m["now"]
    return all(n["clicks"][k] == 2 and n["seated"][k] for k in CLAMPS)


def success(objects):
    """Both clamps locked on the seated tube, both hands clear, the tube still, no latched failure (held 1 s by
    yam_arena)."""
    m = update(objects)
    n = m["now"]
    return bool(not m["fail"] and locked_both(m) and n["clear"] and n["still"])


def score(objects):
    """Partial credit at the last step (success held 1 s scores 1.0 and ends the trial). 0 after a latched failure.
    Both clamps locked on the seated tube, hands clear and still: 0.8; locked and clear: 0.7; locked, a hand within
    10 mm of the tube (or the hands unread): 0.55. Otherwise by each clamp's clicks (counted only with the tube in that
    clamp's mouth): both at the second click but the tube not seated in both 0.45; one at the second and one at the
    first 0.4; both at the first 0.3; one at the second 0.2; one at the first 0.1; else 0."""
    m = update(objects)
    if m["fail"]:
        return 0.0
    n = m["now"]
    if locked_both(m):
        if n["clear"]:
            return 0.8 if n["still"] else 0.7
        return 0.55
    c = sorted(n["clicks"].values())
    if c == [2, 2]:
        return 0.45
    if c == [1, 2]:
        return 0.4
    if c == [1, 1]:
        return 0.3
    if 2 in c:
        return 0.2
    return 0.1 if 1 in c else 0.0


# -- demo -----------------------------------------------------------------------------------------------------------------
YAW_L, YAW_R = -math.pi / 2, math.pi / 2     # fingers close along x, across the tube [as in the bolt-seal task, RUN IK]
GRIP_DZ = 0.012           # TCP over the tube's axis: pad tips 1 mm under it (PAD_TIP_PAST_TCP 13 mm)
SQUEEZE = 0.002
GRIP = -(TUBE_R - SQUEEZE)
OPEN_F = -0.030
RELEASE = -0.016
GRIP_Y = (0.105, -0.105)  # tube-frame y of the left and right grips: 61 mm inside each clamp's arms
PRESS = 0.012             # the TCP aims this far under the seated axis: the pad's stop takes the tube, the arm's
#                           compliance takes the rest. WHY 12 mm: at 4 and 8 mm the arms' position error gave too
#                           little force to pass the first notch on one side (sim probes p1/p2, 2026-11-13)
LIFT_OFF = 0.030
PARK_L = (0.20, 0.38, TOP + 0.20)
PARK_R = (0.20, -0.38, TOP + 0.20)


def _w(t, p, f, label, yaw):
    return Waypoint(float(t), tuple(float(v) for v in p), float(f), label, float(yaw))


def plan(delay=0.0):
    """(left, right, info): both hands take the tube together, carry it over the clamps, lower it into both mouths,
    press it down through both clicks, let go and leave."""
    x0, _, z0 = TUBE_START
    seat = Z0 + AXIS_H
    arms = []
    for gy, yaw, park in ((GRIP_Y[0], YAW_L, PARK_L), (GRIP_Y[1], YAW_R, PARK_R)):
        g0 = np.array([x0, gy, z0 + GRIP_DZ])
        gs = np.array([PLANK_X, gy, seat + GRIP_DZ])
        arms.append([_w(2.0 + delay, g0 + [0, 0, 0.060], OPEN_F, "over the tube", yaw),
                     _w(0.5, g0 + [0, 0, 0.060], OPEN_F, "settle over", yaw),
                     _w(1.2, g0, OPEN_F, "descend", yaw),
                     _w(0.8, g0, GRIP, "close", yaw),
                     _w(0.4, g0, GRIP, "settle grip", yaw),
                     _w(1.2, g0 + [0, 0, 0.060], GRIP, "lift", yaw),
                     _w(2.0, gs + [0, 0, TRAVEL + 0.030], GRIP, "carry over the clamps", yaw),
                     _w(1.0, gs + [0, 0, TRAVEL + 0.002], GRIP, "into the mouths", yaw),
                     _w(0.6, gs + [0, 0, TRAVEL + 0.002], GRIP, "settle on the pads", yaw),
                     _w(1.5, gs + [0, 0, -PRESS], GRIP, "press through both clicks", yaw),
                     _w(0.8, gs + [0, 0, -PRESS], GRIP, "hold", yaw),
                     _w(0.6, gs + [0, 0, -PRESS], RELEASE, "open", yaw),
                     _w(0.3, gs + [0, 0, -PRESS], RELEASE, "settle open", yaw),
                     _w(0.8, gs + [0, 0, LIFT_OFF], RELEASE, "lift off", yaw),
                     _w(1.5, park, OPEN_F, "park", yaw)])
    left, right = arms
    times = {}
    for arm, ws in (("left", left), ("right", right)):
        t = 0.0
        for w in ws:
            times[(arm, w.label)] = (t, t + w.duration)
            t += w.duration
    return left, right, {"times": times}


def demo(delay=0.0):
    """Both hands take the tube from its cradles, carry it level over the clamps, lower it into both open mouths,
    press it down until both clamps lock, let go and park."""
    left, right, _ = plan(delay)
    return {"left": left, "right": right}
