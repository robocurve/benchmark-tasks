"""The bottom of a dishwasher's tub is in front of the robot with its two filters out: the flat lower filter lies on
the table beside it and the cylindrical upper filter assembly stands there. Put them back by reversing the maker's removal steps (the
upper filter assembly comes out with a quarter turn counterclockwise and a lift, then the lower filter lifts slightly and
pulls forward). In this tub two locating tabs at the back of the floor hold the lower filter: push it back under them, so
its round opening lines up with the round opening in the bottom of the tub. Insert the upper filter assembly into that
opening, turning it until it drops in, then turn it clockwise (seen from above) until it stops, about a quarter turn, so it is
locked and no longer turns freely. Then move both grippers at least 10 mm from both filters.
"""
import functools
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from mechanism import Joint, Link, Shape, body, hull, mechanism, on_reset, on_step  # noqa: E402
from yam_arena import station  # noqa: E402,F401
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
D2R = math.pi / 180
IN = 0.0254

# -- the filters: Whirlpool's two-part filter (upper filter assembly W10872845 + lower filter). Steps and the
#    quarter turn: producthelp.whirlpool.com, src/whirlpool_producthelp_clean_filters.html. The cup: 3-1/4 in diameter
#    x 5 in high [SOURCE: parts listing, src/thrifty_W10872845.html; Whirlpool publishes no drawing]. Every other size is
#    [EST] or [CHOICE], stated below.
CUP_R = 3.25 * IN / 2             # 41.3 mm [SOURCE]
CUP_H = 5.0 * IN                  # 127 mm [SOURCE]
CUP_WALL = 0.002
CUP_KG = 0.10                     # [EST]: no listing states a weight
HANDLE = (0.030, 0.006, 0.015)    # the grip bar across the cup's top: half length, half width, height [EST]
LUG_DEG = (0.0, 120.0, 240.0)     # the cup's three bayonet lugs at its foot (own frame) [CHOICE]
LUG_W = 20.0
NECK_R = 0.0285                   # the cup's neck under its shoulder, which goes into the sump [EST]
LUG_R = (NECK_R - 0.001, NECK_R + 0.005)   # the lugs stay inside the published 3-1/4 in (test)
LUG_Z = (0.001, 0.005)
FLANGE_T = 0.002                  # the shoulder's thickness. WHY a shoulder: the full-width body sits 1 mm over the
#                                   lower filter's ring when seated, so a locked cup blocks the lower filter from going in
#                                   after it and keeps it under its tabs (test_a_locked_cup_keeps_the_lower_filter_in_and_out)

PLATE = (0.150, 0.130, 0.003)     # the lower filter: front-back, side-side, thickness [EST]
OPEN_R = 0.0345                   # the well's bore: the lugs pass at any angle with 1 mm play [CHOICE]
MOUTH_R = 0.038                   # the well's mouth and the lower filter's opening, under the shoulder's 41.3 mm. WHY a funnel: the cup hangs from
#                                   its bar and swings a few degrees, so it met the bore 4.8 mm off-centre (p9)
FUNNEL_Z = 0.015                  # the bore runs straight this far over the well's floor, then widens to the mouth
COLLAR = (0.006, 0.008)           # a raised ring round the opening: wall, height [EST]
PLATE_KG = 0.06                   # [EST]

# -- the tub floor (fixed), the sump well under the round opening and its bayonet collar ---------------------------
SX, SY = 0.240, -0.160            # the sump's axis [CHOICE of layout: inside the right arm's reach, see G12]
FLOOR = TOP + 0.067               # the tub floor's top [EST]. WHY so low: the cup must be carried over the lower filter
#                                   and the arm reaches TOP + 0.223 m at most here (ArmPlan bisection, 2026-11-13)
WELL_BOT = TOP + 0.012            # the well's floor, where the cup's foot rests: the cup stands 72 mm proud [EST]
FLOOR_X = (-0.100, 0.090)         # the floor block about the sump's axis
FLOOR_Y = (-0.090, 0.090)
RIB_H = 0.004                     # side rails that guide the lower filter
TAB = (0.014, 0.040, 0.0055, 0.0085)   # each locating tab: overhang, |y| of its centre, bottom and top over the floor
TAB_W = 0.016
SLIDE = 0.018                     # the lower filter goes down this far in front of its home, then slides back under the tabs
COLLAR_R = (NECK_R + 0.0025, OPEN_R)
LIP_Z = (0.006, 0.009)            # over the well's floor
GAP_HALF = 15.0
TURN = 90.0                       # from the drop to the lock, clockwise: Whirlpool's quarter turn [SOURCE]
LIP_END = TURN + LUG_W / 2        # deg clockwise past a gap's centre: the lip, then a stop block to SOLID_END.
#                                   WHY: a lug turned a quarter turn from a gap's centre then meets the stop, so a cup
#                                   that dropped anywhere in its gap (+-5 deg) and is turned until it stops is locked
SOLID_END = 120.0 - GAP_HALF      # the stop block ends at the next gap
GAP0 = 75.0                       # the first gap's centre (deg about the sump, 0 = +x); the others every 120
LOCK = GAP0 - TURN                # -15: the cup's lug 0 at the lock (mod 120)
DET_R = (NECK_R + 0.001, LUG_R[1] - 0.001)          # the click: a spring detent pin in the well's floor under the lugs' path, at 0 deg
DET_T = 0.0015                    # its half width across the path
DET_UP = DET_T                    # its ridge stands this far over a lug's underside, its 45 deg slopes reaching down
#                                   to the underside, so a lug rides up them with no step [CHOICE]
DET_LEN = 0.010
K_DET = 150.0                     # N/m, preloaded DET_PRE up against its stop; while a lug rides over it it pushes
#                                   0.5 N; even at its travel limit it pushes less than the cup's weight, so it cannot lift the cup off the well's floor [CHOICE]
DET_PRE = 0.002
DET_KG = 0.01                     # WHY 10 g: a 2 g pin on its spring rings at 44 Hz, near the physics step [CHOICE]
RAIL_GAP = 0.0015                 # the lower filter's side play between its rails [EST]
BEVEL = 0.0025                    # 45 deg lead-in on each rail's inner top edge [CHOICE]. WHY: the lower filter turned
#                                   1.8 deg in the grip while carried, so its corners stuck out 2.6 mm, landed on a
#                                   plain rail's top and jammed it tilted (p11); a bevel slides it in square
RUNNER = 0.001                    # two runners under its side edges that it slides on [CHOICE]. WHY: a part sliding
#                                   across the floor's seams (it is many convex pieces) snagged on them and stalled
#                                   at one x every time (probes p2, p6, p7); each runner is a single box
PLATE_Z = FLOOR + RUNNER          # the lower filter's underside when home
FLANGE_Z = PLATE_Z + PLATE[2] + COLLAR[1] + 0.001 - WELL_BOT   # the shoulder's underside over the cup's foot

# -- start layout -------------------------------------------------------------------------------------------------------
PLATE_START = (0.240, 0.000, TOP)
CUP_START = (0.170, -0.360, TOP)
CUP_YAW0 = 0.0

WHITE, GREY, DARK, STEEL = (0.90, 0.90, 0.88), (0.55, 0.56, 0.58), (0.22, 0.23, 0.25), (0.70, 0.71, 0.73)
MATERIALS = {"default": (1.2, 1.0),   # the scene's friction on its own assets (rules.md 3)
             "plastic": (0.40, 0.35), # filter frames and tub: hard plastic on hard plastic [EST]
             "slick": (0.10, 0.10)}   # the detent pin


def _box(lo, hi, color, material="plastic", mass=0.0):
    c = tuple(float(v) for v in (np.asarray(lo, float) + np.asarray(hi, float)) / 2)
    s = tuple(float(v) for v in np.asarray(hi, float) - np.asarray(lo, float))
    return Shape("box", c, s, color, material=material, mass=mass)


def _sector(r0, r1, a0, a1, z0, z1, color, material="plastic", mass=0.0, at=(0.0, 0.0, 0.0)):
    """A convex piece of a ring: inner chord, outer arc (3 points), from a0 to a1 deg, z0..z1, centred at ``at``."""
    a0, a1 = a0 * D2R, a1 * D2R
    am = (a0 + a1) / 2
    xy = [(r0 * math.cos(a0), r0 * math.sin(a0)), (r1 * math.cos(a0), r1 * math.sin(a0)),
          (r1 * math.cos(am), r1 * math.sin(am)), (r1 * math.cos(a1), r1 * math.sin(a1)),
          (r0 * math.cos(a1), r0 * math.sin(a1))]
    ring = lambda z: [(at[0] + x, at[1] + y, at[2] + z) for x, y in xy]  # noqa: E731
    return hull(ring(z0) + ring(z1), color, material=material, mass=mass)


def _ring(r0, r1, z0, z1, n, color, at=(0.0, 0.0, 0.0)):
    return [_sector(r0, r1, 360.0 * k / n, 360.0 * (k + 1) / n, z0, z1, color, at=at) for k in range(n)]


def _hole_in_square(r, half, z0, z1, color, n=36, at=(0.0, 0.0, 0.0), r_top=None):
    """Convex pieces filling a square of half side ``half`` less a round hole of radius ``r`` (chords inside it);
    with ``r_top`` the hole widens to that radius at z1 (a funnel)."""
    out = []
    rt = r if r_top is None else r_top
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        pts = [(r * math.cos(a0), r * math.sin(a0)), (r * math.cos(a1), r * math.sin(a1))]
        top = [(rt * math.cos(a0), rt * math.sin(a0)), (rt * math.cos(a1), rt * math.sin(a1))]
        for a in (a1, a0):
            c, s = math.cos(a), math.sin(a)
            t = half / max(abs(c), abs(s))
            pts.append((t * c, t * s))
        corners = [(half * sx, half * sy) for sx in (1, -1) for sy in (1, -1)]
        for cx, cy in corners:     # a square corner inside this sector's span joins its outer edge
            ac = math.atan2(cy, cx) % (2 * math.pi)
            if a0 < ac < a1:
                pts.insert(3, (cx, cy))
        # WHY: hull() wants two rings of the same points; pad a ring with its last point to keep the count even
        ring = lambda z, q: [(at[0] + x, at[1] + y, at[2] + z) for x, y in q]  # noqa: E731
        out.append(hull(ring(z0, pts) + ring(z1, top + pts[2:]), color))
    return out


def _split(a0, a1, step=9.0):
    n = max(1, int(round(abs(a1 - a0) / step)))
    lo, hi = min(a0, a1), max(a0, a1)
    return [(lo + (hi - lo) * k / n, lo + (hi - lo) * (k + 1) / n) for k in range(n)]


def gap_centres():
    return tuple(GAP0 + d for d in LUG_DEG)


HOLE_HALF = 0.060                 # the square round the sump that the floor's ring pieces fill


def floor_parts():
    """(base shapes, links, joints) of the tub floor, in the world frame."""
    w = lambda x, y, z: (SX + x, SY + y, z)  # noqa: E731
    at_floor = (SX, SY, WELL_BOT)
    x0, x1 = FLOOR_X
    y0, y1 = FLOOR_Y
    h = HOLE_HALF
    sh = [_box(w(x0, y0, TOP), w(-h, y1, FLOOR), WHITE), _box(w(h, y0, TOP), w(x1, y1, FLOOR), WHITE),
          _box(w(-h, y0, TOP), w(h, -h, FLOOR), WHITE), _box(w(-h, h, TOP), w(h, y1, FLOOR), WHITE)]
    sh += _hole_in_square(OPEN_R, h, WELL_BOT, WELL_BOT + FUNNEL_Z, WHITE, at=(SX, SY, 0.0))
    sh += _hole_in_square(OPEN_R, h, WELL_BOT + FUNNEL_Z, FLOOR, WHITE, at=(SX, SY, 0.0), r_top=MOUTH_R)
    # the well's floor, with a channel for the detent pin at 0 deg
    cx0, cx1, cy = DET_R[0] - 0.0003, DET_R[1] + 0.0003, DET_T + 0.0003
    sh += [_box(w(-h, -h, TOP), w(cx0, h, WELL_BOT), GREY), _box(w(cx1, -h, TOP), w(h, h, WELL_BOT), GREY),
           _box(w(cx0, -h, TOP), w(cx1, -cy, WELL_BOT), GREY), _box(w(cx0, cy, TOP), w(cx1, h, WELL_BOT), GREY)]
    for g in gap_centres():
        for a0, a1 in _split(g - LIP_END, g - GAP_HALF, 15.0):
            sh.append(_sector(*COLLAR_R, a0, a1, *LIP_Z, GREY, at=at_floor))
        for a0, a1 in _split(g - SOLID_END, g - LIP_END):
            sh.append(_sector(*COLLAR_R, a0, a1, 0.0, LIP_Z[1], GREY, at=at_floor))
    # the lower filter's home: side rails, the rear wall and its two locating tabs
    px0, px1 = -PLATE[0] / 2, PLATE[0] / 2
    hy = PLATE[1] / 2 + RAIL_GAP
    for sgn in (1.0, -1.0):
        yz = [(hy, FLOOR), (hy + 0.006, FLOOR), (hy + 0.006, FLOOR + RIB_H), (hy + BEVEL, FLOOR + RIB_H),
              (hy, FLOOR + RIB_H - BEVEL)]
        sh.append(hull([w(x, sgn * y, z) for x in (px0 - SLIDE - 0.005, px1) for y, z in yz], GREY, material="plastic"))
    sh += [
           _box(w(px1 + 0.0005, -hy - 0.006, FLOOR), w(px1 + 0.0065, hy + 0.006, FLOOR + TAB[3]), GREY),
           _box(w(px0 - SLIDE - 0.005, MOUTH_R + 0.001, FLOOR), w(px1, hy, PLATE_Z), GREY),
           _box(w(px0 - SLIDE - 0.005, -hy, FLOOR), w(px1, -MOUTH_R - 0.001, PLATE_Z), GREY)]
    for sgn in (1.0, -1.0):
        yc = sgn * TAB[1]
        sh.append(_box(w(px1 + 0.0005 - TAB[0], yc - TAB_W / 2, FLOOR + TAB[2]),
                       w(px1 + 0.0005, yc + TAB_W / 2, FLOOR + TAB[3]), GREY))
    # the detent pin: a ridge across the lugs' path, sloped both ways so a lug rides over it and it clicks up behind
    zt = WELL_BOT + LUG_Z[0] + DET_UP
    zb = zt - DET_LEN
    r0, r1, t = DET_R[0], DET_R[1], DET_T
    xy = [(r0, -t), (r1, -t), (r1, 0.0), (r1, t), (r0, t), (r0, 0.0)]
    zs = [zt - t, zt - t, zt, zt - t, zt - t, zt]
    pin = hull([w(x, y, zb) for x, y in xy] + [w(x, y, z) for (x, y), z in zip(xy, zs)], STEEL, material="slick",
               mass=DET_KG)
    links = [Link("detent", DET_KG, (pin,))]
    joints = [Joint("detent", "prismatic", "detent", w(sum(DET_R) / 2, 0.0, zt), "Z", -(DET_UP + 0.002), 0.0,
                    stiffness=K_DET, damping=0.3, target=DET_PRE)]
    return sh, links, joints


def _massed(shapes, kg):
    vol = [float(np.prod(np.subtract(*s.bounds()[::-1]))) for s in shapes]   # bounding-box volumes [CHOICE]
    tot = sum(vol)
    return [Shape(s.kind, s.center, s.size, s.color, s.axis, s.material, s.contact_offset, s.points, kg * v / tot)
            for s, v in zip(shapes, vol)]


def cup_shapes():
    """The upper filter assembly in its own frame: origin at its foot's centre, the grip bar along +x."""
    sh = _ring(CUP_R - CUP_WALL, CUP_R, FLANGE_Z, CUP_H - HANDLE[2], 36, GREY)
    sh += _ring(NECK_R - CUP_WALL, NECK_R, 0.002, FLANGE_Z, 36, GREY)
    sh.append(Shape("cylinder", (0.0, 0.0, 0.001), (NECK_R, 0.002), GREY, material="plastic"))
    sh.append(Shape("cylinder", (0.0, 0.0, CUP_H - HANDLE[2] - 0.001), (CUP_R, 0.002), GREY, material="plastic"))
    sh.append(_box((-HANDLE[0], -HANDLE[1], CUP_H - HANDLE[2]), (HANDLE[0], HANDLE[1], CUP_H), DARK))
    sh += _ring(NECK_R, CUP_R - CUP_WALL, FLANGE_Z, FLANGE_Z + FLANGE_T, 36, GREY)
    sh += [_sector(*LUG_R, a0, a1, *LUG_Z, GREY) for d in LUG_DEG
           for a0, a1 in _split(d - LUG_W / 2, d + LUG_W / 2, 10.0)]
    return _massed(sh, CUP_KG)


def plate_shapes():
    """The lower filter in its own frame: origin at the centre of its underside (and of its round opening)."""
    hx, hy, t = PLATE[0] / 2, PLATE[1] / 2, PLATE[2]
    h = HOLE_HALF
    sh = [_box((-hx, -hy, 0.0), (-h, hy, t), WHITE), _box((h, -hy, 0.0), (hx, hy, t), WHITE),
          _box((-h, -hy, 0.0), (h, -h, t), WHITE), _box((-h, h, 0.0), (h, hy, t), WHITE)]
    sh += _hole_in_square(MOUTH_R, h, 0.0, t, WHITE)
    sh += _ring(MOUTH_R, MOUTH_R + COLLAR[0], t, t + COLLAR[1], 36, WHITE)
    return _massed(sh, PLATE_KG)


def _base():
    sh, links, joints = floor_parts()
    return mechanism("tub", (0.0, 0.0, 0.0), Link("tub", 10.0, tuple(sh)), links, joints, MATERIALS)


TUB = _base()
PLATE_BODY = body("lower_filter", PLATE_START, plate_shapes(), MATERIALS, color=WHITE)
CUP = body("upper_filter", CUP_START, cup_shapes(), MATERIALS, yaw=CUP_YAW0 * D2R, color=GREY)
OBJECTS = [TUB, PLATE_BODY, CUP]
PARTS = ("lower_filter", "upper_filter")

# -- scoring --------------------------------------------------------------------------------------------------------------
HOME_TOL = 0.002       # m: the lower filter within 2 mm of home in x and y (its back then under both tabs)
SEAT_TOL = 0.0015      # m: a part within 1.5 mm of its seated height
YAW_TOL = 3.0          # deg: the lower filter square to its rails
CENTRE_TOL = 0.003     # m: the cup's axis within 3 mm of the sump's
TILT_DEG = 3.0
LOCK_WIN = (-2.5, 2.0)     # deg about LOCK (mod 120): every lug under its lip and past the pin (the stop is at 0)
CLEAR_MIN = 0.010
STILL_V, STILL_W = 0.005, 0.05
LOST_Z = TOP - 0.030


def _rot3(q):
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _wrap(a, period=360.0):
    return (a + period / 2) % period - period / 2


def yaw_deg(state):
    r = _rot3(state.quat_xyzw)
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def tilt_deg(state):
    return math.degrees(math.acos(float(np.clip(_rot3(state.quat_xyzw)[2, 2], -1.0, 1.0))))


def lower_home(objects):
    """The lower filter flat on the floor, back under both tabs, its opening over the sump."""
    s = objects["lower_filter"]
    p = np.asarray(s.pos, float)
    return bool(abs(p[0] - SX) <= HOME_TOL and abs(p[1] - SY) <= HOME_TOL and abs(p[2] - PLATE_Z) <= SEAT_TOL
                and abs(_wrap(yaw_deg(s), 180.0)) <= YAW_TOL and tilt_deg(s) <= TILT_DEG)


def cup_dropped(objects):
    """The cup's foot on the well's floor: its lugs went down through the gaps."""
    s = objects["upper_filter"]
    p = np.asarray(s.pos, float)
    return bool(math.hypot(p[0] - SX, p[1] - SY) <= CENTRE_TOL and abs(p[2] - WELL_BOT) <= SEAT_TOL
                and tilt_deg(s) <= TILT_DEG)


def detent_depth(objects):
    return -float(objects["tub"].joints["detent"])


def cup_angle(objects):
    """The cup's turn past the lock (deg, mod 120): negative is further clockwise."""
    return _wrap(yaw_deg(objects["upper_filter"]) - LOCK, 120.0)


def cup_locked(objects):
    """Dropped and turned into the lock window. WHY no pin test here: the pin stands in the lugs' path between the drop
    and the window, so a cup in the window got there past it and does not turn freely back; inside the window the pin
    is always clear of the lugs (test_pin_clicks_behind_a_lug_only_at_the_lock)."""
    return bool(cup_dropped(objects) and LOCK_WIN[0] <= cup_angle(objects) <= LOCK_WIN[1])


def pose(objects):
    return {k: (np.asarray(objects[k].pos, float), _rot3(objects[k].quat_xyzw)) for k in PARTS}


def rates(prev, now, dt):
    v = w = 0.0
    for k in PARTS:
        (p0, r0), (p1, r1) = prev[k], now[k]
        v = max(v, float(np.linalg.norm(p1 - p0)) / dt)
        c = (float(np.trace(r0.T @ r1)) - 1.0) / 2.0
        w = max(w, math.acos(float(np.clip(c, -1.0, 1.0))) / dt)
    return v, w


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


def _prism(r, z0, z1):
    """A 16-sided prism round radius r (so it contains the circle), z0..z1: (vertices, normals) in a part's frame."""
    rr = r / math.cos(math.pi / _PRISM)
    ang = [2 * math.pi * (k + 0.5) / _PRISM for k in range(_PRISM)]
    v = np.array([(rr * math.cos(u), rr * math.sin(u), z) for z in (z0, z1) for u in ang])
    n = np.array([(math.cos(2 * math.pi * k / _PRISM), math.sin(2 * math.pi * k / _PRISM), 0.0)
                  for k in range(_PRISM)] + [(0.0, 0.0, 1.0)])
    return v, n


def _cuboid(lo, hi):
    v = np.array([(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    return v, np.eye(3)


@functools.lru_cache(maxsize=1)
def part_convexes():
    """Convex sets that contain each loose part (its frame): the gap from a hand to them is a lower bound on the gap to
    the part."""
    hx, hy, t = PLATE[0] / 2, PLATE[1] / 2, PLATE[2]
    return {"lower_filter": (_cuboid((-hx, -hy, 0.0), (hx, hy, t)), _prism(MOUTH_R + COLLAR[0], 0.0, t + COLLAR[1])),
            "upper_filter": (_prism(LUG_R[1], 0.0, FLANGE_Z), _prism(CUP_R, FLANGE_Z, CUP_H - HANDLE[2]),
                             _cuboid((-HANDLE[0], -HANDLE[1], CUP_H - HANDLE[2]), (HANDLE[0], HANDLE[1], CUP_H)))}


def _sat_gap(hv, hn, kv, kn):
    """A lower bound on the distance between two convex sets: the largest gap of their projections on either set's
    face normals; negative when they overlap on every one of those axes."""
    ax = np.vstack([hn, kn])
    ph, pk = hv @ ax.T, kv @ ax.T
    return float(np.max(np.maximum(pk.min(axis=0) - ph.max(axis=0), ph.min(axis=0) - pk.max(axis=0))))


def world_convexes(objects):
    out = []
    for k in PARTS:
        rot, p = _rot3(objects[k].quat_xyzw), np.asarray(objects[k].pos, float)
        for v, n in part_convexes()[k]:
            w = v @ rot.T + p
            c = (w.min(axis=0) + w.max(axis=0)) / 2
            out.append((w, n @ rot.T, c, float(np.linalg.norm(w - c, axis=1).max())))
    return out


def hand_gap(objects, hands=None, need=None):
    """Smallest distance (m) from the hands to both filters (a lower bound), -inf when the hands were not read. With
    ``need``, the search stops at the first pair under it."""
    hs = _HANDS["hands"] if hands is None else hands
    if hs is None or len(hs) == 0:
        return -math.inf
    best = math.inf
    for kv, kn, kc, kr in world_convexes(objects):
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
    """Latched by update(): a filter off the table is lost."""
    for k in PARTS:
        if float(objects[k].pos[2]) < LOST_Z:
            return f"lost {k}"
    return None


def stage(objects):
    """0 nothing; 1 lower filter home under its tabs; 2 the cup dropped through it into the well; 3 the cup locked
    (turned into the lock window, which it can reach only past the detent pin). Each needs the ones before it."""
    if not lower_home(objects):
        return 0
    if not cup_dropped(objects):
        return 1
    return 3 if cup_locked(objects) else 2


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
    _MEM["now"] = {"stage": stage(objects), "clear": clear(objects), "still": still(_MEM["rates"])}
    return _MEM


def success(objects):
    """Both filters in, the cup locked, the hands clear, both filters still, no latched failure (held 1 s by
    yam_arena)."""
    m = update(objects)
    n = m["now"]
    return bool(not m["fail"] and n["stage"] == 3 and n["clear"] and n["still"])


LADDER = {0: 0.0, 1: 0.25, 2: 0.5}


def score(objects):
    """Partial credit at the last step (success held 1 s scores 1.0 and ends the trial). 0 after a latched failure.
    Locked, hands clear and still: 0.8; locked and clear: 0.7; locked, a hand within 10 mm of a filter (or the hands
    unread): 0.6. Otherwise: the cup dropped into the well but not locked 0.5; the lower filter home 0.25; else 0."""
    m = update(objects)
    if m["fail"]:
        return 0.0
    n = m["now"]
    if n["stage"] == 3:
        if n["clear"]:
            return 0.8 if n["still"] else 0.7
        return 0.6
    return LADDER[n["stage"]]


# -- demo -----------------------------------------------------------------------------------------------------------------
TIP = 0.013               # PAD_TIP_PAST_TCP: the pad tips sit 13 mm under the TCP
TIP_UP = 0.002            # the pad tips 2 mm over a grip's underside
GRIP_PLATE = 0.0          # finger targets. WHY so tight: a grip turns or carries a part only through the pads'
GRIP_CUP = -0.001         # friction, so each squeezes 3-5 mm (15-25 N at 5000 N/m); 2 mm let parts swing (newtask5 p1)
OPEN_F = -0.030
RELEASE = -0.018
YAW_X = math.pi / 2       # fingers closing along x (probes p1-p4: yaw 0 grips the cup's bar across y)
BAR_YAW = 0.0             # the cup's bar (along its own x) is gripped across its width at the cup's own yaw
PRESS = 0.003             # the TCP aims this far under a seat, so the arm keeps a part down
HUNT = 20.0               # the cup goes in this far counterclockwise of a gap, its lugs resting on the next stop
#                           block and lip; it is turned clockwise until it drops
STEP = 5.0
OVER = 1.5                # the hand turns this far past the lock, into the stop block. WHY: the cup lags the
#                           wrist about 1.2 deg (p10), so this brings it onto the stop with a light press
PARK = (0.18, -0.38, TOP + 0.20)


def _cy(a):
    """The wrist yaw (rad) that grips the cup's bar across its width with the cup at ``a`` deg."""
    return (a + BAR_YAW) * D2R


def _w(t, p, f, label, yaw):
    return Waypoint(float(t), tuple(float(v) for v in p), float(f), label, float(yaw))


def grip_x():
    """The lower filter's grip, its frame: the front of the collar round the opening."""
    return -(MOUTH_R + COLLAR[0] / 2)


def plan(delay=0.0):
    """(left, right, info): the right hand slides the lower filter home, then puts the cup in and turns it clockwise
    until it drops and then until it locks. The left hand stays parked."""
    pz = PLATE[2] + TIP_UP + TIP
    gx = grip_x()
    p0 = np.array([PLATE_START[0] + gx, PLATE_START[1], PLATE_START[2] + pz])
    carry_p = FLOOR + RIB_H + 0.010 + pz
    front = np.array([SX + gx - SLIDE, SY, carry_p])
    down = PLATE_Z + pz + 0.0008   # WHY held 0.8 mm up: any press into the floor (the arm sags about 1 mm) gave
#                                  floor friction that beat the grip and stalled the slide (probes p1, p2, p5)
    home = np.array([SX + gx + 0.002, SY, down])
    right = [_w(2.0 + delay, p0 + [0, 0, 0.05], OPEN_F, "over the lower filter", YAW_X),
             _w(0.5, p0 + [0, 0, 0.05], OPEN_F, "settle over", YAW_X),
             _w(1.0, p0, OPEN_F, "descend to its collar", YAW_X),
             _w(0.8, p0, GRIP_PLATE, "close on the collar", YAW_X),
             _w(0.4, p0, GRIP_PLATE, "settle grip on the collar", YAW_X),
             _w(1.0, [p0[0], p0[1], carry_p], GRIP_PLATE, "lift the lower filter", YAW_X),
             _w(2.0, front, GRIP_PLATE, "carry in front of the tabs", YAW_X),
             _w(0.4, front, GRIP_PLATE, "settle in front", YAW_X),
             _w(1.0, [front[0], front[1], down], GRIP_PLATE, "lower onto the floor", YAW_X),
             _w(0.4, [front[0], front[1], down], GRIP_PLATE, "settle on the floor", YAW_X),
             _w(1.5, home, GRIP_PLATE, "slide back under the tabs", YAW_X),
             _w(0.4, home, GRIP_PLATE, "hold home", YAW_X),
             _w(0.6, home, RELEASE, "open off the collar", YAW_X),
             _w(0.8, home + [0, 0, 0.04], RELEASE, "lift off the collar", YAW_X)]
    cz = CUP_H - HANDLE[2] + TIP_UP + TIP
    c0 = np.array([CUP_START[0], CUP_START[1], CUP_START[2] + cz])
    y0 = CUP_YAW0
    carry_c = PLATE_Z + PLATE[2] + COLLAR[1] + 0.005 + cz
    a_in = GAP0 + HUNT
    over = np.array([SX, SY, carry_c])
    rest = WELL_BOT + LIP_Z[1] - LUG_Z[0] + cz - PRESS
    sink = WELL_BOT + cz - PRESS
    right += [_w(1.8, c0 + [0, 0, 0.05], OPEN_F, "over the cup", _cy(y0)),
              _w(0.5, c0 + [0, 0, 0.05], OPEN_F, "settle over the cup", _cy(y0)),
              _w(1.0, c0, OPEN_F, "descend to the grip bar", _cy(y0)),
              _w(0.8, c0, GRIP_CUP, "close on the grip bar", _cy(y0)),
              _w(0.4, c0, GRIP_CUP, "settle grip on the bar", _cy(y0)),
              _w(1.2, [c0[0], c0[1], carry_c], GRIP_CUP, "lift the cup", _cy(y0)),
              _w(2.0, over, GRIP_CUP, "carry over the opening", _cy(a_in)),
              _w(1.2, over, GRIP_CUP, "settle over the opening", _cy(a_in)),
              _w(1.5, [SX, SY, rest], GRIP_CUP, "insert into the opening", _cy(a_in)),
              _w(0.5, [SX, SY, rest], GRIP_CUP, "rest on the stop blocks", _cy(a_in))]
    # WHY two turns: the lugs ride the stop blocks until they line up with the gaps and the cup drops (the hand
    # follows it down); then the same clockwise turn carries them under the lips and past the pin
    a, k = a_in, 0
    while a > GAP0 + 1e-9:
        a, k = max(GAP0, a - STEP), k + 1
        right.append(_w(0.5, [SX, SY, rest], GRIP_CUP, f"rotate clockwise {k}", _cy(a)))
    right += [_w(0.8, [SX, SY, sink], GRIP_CUP, "drop into place", _cy(GAP0)),
              _w(0.4, [SX, SY, sink], GRIP_CUP, "settle dropped", _cy(GAP0))]
    end = LOCK - OVER
    while a > end + 1e-9:
        a, k = max(end, a - STEP), k + 1
        right.append(_w(0.5, [SX, SY, sink], GRIP_CUP, f"rotate clockwise {k}", _cy(a)))
    right += [_w(0.5, [SX, SY, sink], GRIP_CUP, "hold at the lock", _cy(end)),
              _w(0.6, [SX, SY, sink], RELEASE, "open off the bar", _cy(end)),
              _w(0.8, [SX, SY, sink + 0.04], RELEASE, "lift off the bar", _cy(end)),
              _w(1.5, PARK, OPEN_F, "park", _cy(end))]
    left = [_w(2.0 + delay, (0.18, 0.38, TOP + 0.20), OPEN_F, "park left", 0.0),
            _w(sum(w.duration for w in right) - 2.0, (0.18, 0.38, TOP + 0.20), OPEN_F, "stay parked", 0.0)]
    times = {}
    for arm, ws in (("left", left), ("right", right)):
        t = 0.0
        for w in ws:
            times[(arm, w.label)] = (t, t + w.duration)
            t += w.duration
    return left, right, {"times": times}


def demo(delay=0.0):
    """The right hand carries the lower filter by its collar, sets it down in front of the tabs and slides it back
    under them; then it lowers the cup into the opening 20 deg counterclockwise of a gap, so its lugs rest on a stop
    block and a lip, turns it clockwise until it drops, then a quarter turn on, past the pin, to the lock; it lets go
    and parks."""
    left, right, _ = plan(delay)
    return {"left": left, "right": right}
