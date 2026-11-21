"""Set up a folded tabletop display easel and rest a canvas on it.

A U.S. Art Supply 10.5 in Small Wooden Tabletop Easel (USA E-003, beechwood) lies folded on the table, face up, its
feet toward the robot: two front legs that lean in toward a top crossbar, a canvas ledge low on the front, and a rear
leg that hangs from a brass hinge behind the crossbar and lies folded flat under the frame. The hinge lets the rear
leg swing out only to a stop (the maker's "locking leg"); open, the easel stands on three feet about 26 cm tall.
To the right of it a 5 x 7 in gallery-depth (1.5 in) stretched canvas stands on one long edge, its blank face toward
the robot. The robot unfolds the easel, stands it on its three feet with the rear leg at its stop, and rests the
canvas on the ledge, leaning back against the easel.

The easel (module section "the easel", reusable) is a free two-body mechanism: the front frame (legs, crossbar and
ledge) and the rear leg on one hinge, with a hard stop where the rear leg is 47 degrees open (estimate). Metres;
x forward from the arm bases, y left, z up.
"""
import math
import pathlib
import sys
from collections import deque

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, box  # noqa: E402
from yam_joints import Hinge, Link  # noqa: E402
from yam_mesh import Mesh, MeshMechanism  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
RELEASE_CLEAR = 0.030
STILL_S = 1.0
TEST_GRIPPERS = None
DIAG = False  # GPU diagnostic copies print EASEL_DIAG lines
PROBE = False  # probe copies: the easel alone, the right arm parked
DEMO_PACE = 1.0  # scales every demo duration except the fingers' close and open
DEMO_NOLIFT = False  # wrong-way copy: stand the folded easel up on the table and let go without opening the leg


# ---------------------------------------------------------------- run context (episode clock for the scorer)
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


def grippers():
    """{arm: (TCP position, opening 0..1)} from the robots' joints (TEST_GRIPPERS offline)."""
    if TEST_GRIPPERS is not None:
        return TEST_GRIPPERS
    env = _RUN["env"]
    if env is None:
        return None
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

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
        tcp = ik.tcp_pose(q[arm_ids], station.BASES[arm])[0]
        opening = float(np.clip(np.mean(q[finger_ids]) / FINGER_OPEN, 0.0, 1.0))
        out[arm] = (tcp, opening)
    return out


TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S, SCORE_OVER = 1.0, "end"
RUN_LIMIT_STEPS = 2400
MM = 0.001
DEG = math.pi / 180

BEECH = (0.80, 0.62, 0.42)
BEECH_DARK = (0.70, 0.52, 0.34)
CANVAS_WHITE = (0.96, 0.96, 0.94)


def _rot_y(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _T(R, p):
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = R, p
    return m


def _inv(m):
    R, p = m[:3, :3], m[:3, 3]
    return _T(R.T, -R.T @ p)


def _oriented(name, points, faces, color, proxy):
    points, faces = np.asarray(points, float), np.asarray(faces, int)
    tri = (points - points.mean(axis=0))[faces]
    if np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum() < 0:
        faces = faces[:, ::-1]
    return Mesh(name, points, faces, collision="convex", color=color, proxy=tuple(proxy))


_BOX_FACES = [(0, 2, 1), (1, 2, 3), (4, 5, 6), (5, 7, 6), (0, 1, 4), (1, 5, 4), (2, 6, 3), (3, 6, 7),
              (0, 4, 2), (2, 4, 6), (1, 3, 5), (3, 7, 5)]


def _slanted_bar(name, p0, p1, width, z0, z1, color, segments=6):
    """A straight bar of rectangular section lying in the part's x-y plane from p0 to p1 (x, y), `width` across, from
    z0 to z1, as one convex mesh; its offline stand-in is `segments` boxes along it."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = (p1 - p0) / np.linalg.norm(p1 - p0)
    n = np.array([-d[1], d[0]]) * width / 2
    corners = []
    for p in (p0, p1):
        for s in (-1, 1):
            for z in (z0, z1):
                q = p + s * n
                corners.append((q[0], q[1], z))
    pts = np.array(corners)
    # reorder to the box face list: index bit0 = z, bit1 = side, bit2 = end
    order = [0, 1, 2, 3, 4, 5, 6, 7]
    pts = pts[order]
    proxy = []
    for k in range(segments):
        a = p0 + (p1 - p0) * k / segments
        b = p0 + (p1 - p0) * (k + 1) / segments
        xs = [a[0] + n[0], a[0] - n[0], b[0] + n[0], b[0] - n[0]]
        ys = [a[1] + n[1], a[1] - n[1], b[1] + n[1], b[1] - n[1]]
        lo, hi = np.array([min(xs), min(ys), z0]), np.array([max(xs), max(ys), z1])
        shrink = np.array([0.0, (hi[1] - lo[1] - width) * 0.5, 0.0])
        lo2, hi2 = lo + shrink * 0.5, hi - shrink * 0.5
        proxy.append(box(tuple((lo2 + hi2) / 2), tuple(np.maximum(hi2 - lo2, 0.5 * MM))))
    return _oriented(name, pts, _BOX_FACES, color, proxy)


def _box_mesh(name, lo, hi, color):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    pts = np.array([(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    pts = pts[[0, 1, 2, 3, 4, 5, 6, 7]]
    return _oriented(name, pts, _BOX_FACES, color, (box(tuple((lo + hi) / 2), tuple(hi - lo)),))


# ---------------------------------------------------------------- the easel
# U.S. Art Supply USA E-003 "10.5 in Small Wooden Tabletop Easel" (usartsupply.com/products/usa-e-003): open
# 7.5 in wide x 8.5 in deep x 10.5 in set-up height, 0.75 in deep canvas ledge, folds to 11 in, under 0.5 lb,
# beechwood, rear leg on a brass hinge ("tilts with locking leg"). Not published, so estimates: bar sections, where
# the crossbar, ledge and hinge sit, the rear leg's length, the stop angle and the masses.
# The easel's own frame (the part frame, built lying face up as it starts): x up the frame from the feet to the top,
# y across (left), z out of the front face. Origin: the front face's foot end on the centre line.
LEG_L = 279.0 * MM  # front legs = the folded length, 11 in
LEG_W, LEG_T = 18.0 * MM, 13.0 * MM  # across the face, front to back (estimate)
LEG_FOOT_Y, LEG_TOP_Y = 86.0 * MM, 36.0 * MM  # leg centre lines at the feet and at the top (7.5 in over the feet)
XBAR_X = (231.0 * MM, 249.0 * MM)
XBAR_HALF = 40.0 * MM
LEDGE_X = (20.0 * MM, 32.0 * MM)  # the ledge's underside and top face along the frame
LEDGE_D = 19.0 * MM  # 0.75 in, in front of the legs
LEDGE_HALF = 95.0 * MM
LIP_X, LIP_D = 5.0 * MM, 5.0 * MM  # a low lip along the ledge's front edge (estimate)
PIN = np.array([240.0 * MM, 0.0, -LEG_T])  # hinge pin: the crossbar's back face, mid-height
REAR_L = 236.0 * MM  # pin to the rear leg's foot (its foot 4 mm short of the front feet when folded)
REAR_W, REAR_T = 18.0 * MM, 13.0 * MM
STOP_DEG = 47.0  # rear leg's opening at the hinge stop (estimate from the 8.5 in depth)
FRAME_MASS, REAR_MASS = 0.147, 0.043  # 0.19 kg in all (published: under 0.5 lb)
EASEL_FRICTION = (0.50, 0.40)  # bare beech on the table (estimate)
HINGE_FRICTION = 0.004  # N m, the brass hinge's static friction (estimate; the leg drops open under its own weight)
HINGE_DAMPING = 0.002  # N m s/rad


def leg_y(x):
    """Centre line of the left front leg across the face at distance x up the frame (the right leg is -leg_y)."""
    return LEG_FOOT_Y + (LEG_TOP_Y - LEG_FOOT_Y) * x / LEG_L


def make_easel(pos):
    leg_kw = dict(width=LEG_W, z0=-LEG_T, z1=0.0, color=BEECH)
    base = (
        _slanted_bar("leg_l", (0.0, LEG_FOOT_Y), (LEG_L, LEG_TOP_Y), **leg_kw),
        _slanted_bar("leg_r", (0.0, -LEG_FOOT_Y), (LEG_L, -LEG_TOP_Y), **leg_kw),
        _box_mesh("crossbar", (XBAR_X[0], -XBAR_HALF, -LEG_T), (XBAR_X[1], XBAR_HALF, 0.0), BEECH),
        _box_mesh("ledge", (LEDGE_X[0], -LEDGE_HALF, 0.0), (LEDGE_X[1], LEDGE_HALF, LEDGE_D), BEECH_DARK),
        _box_mesh("lip", (LEDGE_X[1], -LEDGE_HALF, LEDGE_D - LIP_D), (LEDGE_X[1] + LIP_X, LEDGE_HALF, LEDGE_D),
                  BEECH_DARK),
    )
    rear = _box_mesh("rear", (PIN[0] - REAR_L, -REAR_W / 2, -LEG_T - REAR_T), (PIN[0], REAR_W / 2, -LEG_T), BEECH)
    link = Link("rear_leg", [], Hinge("y", tuple(float(v) for v in PIN), -STOP_DEG, 0.0, damping=HINGE_DAMPING,
                                      static_friction=HINGE_FRICTION, dynamic_friction=0.75 * HINGE_FRICTION),
                REAR_MASS, color=BEECH, friction=EASEL_FRICTION)
    return MeshMechanism("easel", pos, (), (link,), mass=FRAME_MASS, color=BEECH, friction=EASEL_FRICTION,
                         grip=2, meshes={"base": base, "rear_leg": (rear,)})


# Feet and the rear leg's foot, in the part frame at q = 0 (corners of each foot end).
def _foot_corners(sign):
    p0 = np.array([0.0, sign * LEG_FOOT_Y])
    d = np.array([LEG_L, sign * (LEG_TOP_Y - LEG_FOOT_Y)])
    n = np.array([-d[1], d[0]]) / np.linalg.norm(d) * LEG_W / 2
    return [np.array([*(p0 + s * n), z]) for s in (-1, 1) for z in (0.0, -LEG_T)]


FRONT_FEET = {"left": _foot_corners(1.0), "right": _foot_corners(-1.0)}
REAR_FOOT = [np.array([PIN[0] - REAR_L, y, z]) for y in (-REAR_W / 2, REAR_W / 2) for z in (-LEG_T, -LEG_T - REAR_T)]


def easel_rot(lean_deg):
    """Part-to-world rotation of the easel standing face toward the robot, its frame leaning back lean_deg."""
    return _rot_y(-(90.0 - lean_deg))


def rear_leg_rot(q_rad):
    return _rot_y(math.degrees(q_rad))


def easel_points(R, o, q_rad):
    """World points of the two front feet's corners and the rear foot's corners for a pose (R, o) and hinge q."""
    Rl = R @ rear_leg_rot(q_rad)
    front = {k: [o + R @ c for c in v] for k, v in FRONT_FEET.items()}
    rear = [o + R @ PIN + Rl @ (c - PIN) for c in REAR_FOOT]
    return front, rear


def _standing_lean(q_rad):
    """Lean (deg) at which the front feet and the rear foot touch a flat table together, with the hinge at q."""
    def gap(lean):
        R = easel_rot(lean)
        front, rear = easel_points(R, np.zeros(3), q_rad)
        return min(p[2] for v in front.values() for p in v) - min(p[2] for p in rear)
    lo, hi = 2.0, 80.0
    for _ in range(60):
        mid = (lo + hi) / 2
        if (gap(lo) > 0) == (gap(mid) > 0):
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def easel_standing_pose(lean, q_rad, front_x, y):
    """(R, origin) with all feet on the table; the front feet's front-most bottom point at x = front_x."""
    R = easel_rot(lean)
    front, rear = easel_points(R, np.zeros(3), q_rad)
    zmin = min(p[2] for v in front.values() for p in v)
    xmin = min(p[0] for v in front.values() for p in v)
    return R, np.array([front_x - xmin, y, TOP - zmin])


STAND_LEAN = _standing_lean(-STOP_DEG * DEG)  # ~22 deg


def depth_of(R, o, q_rad):
    """Ruler depth: front of the front feet to the back of the rear foot, along the easel's facing (m)."""
    front, rear = easel_points(R, o, q_rad)
    back = -np.array([R[0, 2], R[1, 2], 0.0])
    back /= max(np.linalg.norm(back), 1e-9)
    return max(float(p @ back) for p in rear) - min(float(p @ back) for v in front.values() for p in v)


FULL_DEPTH = depth_of(easel_rot(STAND_LEAN), np.zeros(3), -STOP_DEG * DEG)

# ---------------------------------------------------------------- the canvas
# U.S. Art Supply 5 x 7 in Gallery Depth 1-1/2 in Profile Stretched Canvas (USA GSC05X07G): 178 x 127 mm, 38 mm deep,
# cotton over pine stretcher bars. Bar width and mass are estimates. Part frame: x into the canvas from its face,
# y along its long edge (left), z up its face; origin at the middle of its bottom front edge.
CAN_W, CAN_H, CAN_D = 177.8 * MM, 127.0 * MM, 38.1 * MM
CAN_BAR = 19.0 * MM
CAN_FACE = 2.0 * MM
CANVAS_MASS = 0.17
CANVAS_FRICTION = (0.55, 0.45)


def make_canvas(pos):
    hw = CAN_W / 2
    shapes = [
        box((CAN_FACE / 2, 0.0, CAN_H / 2), (CAN_FACE, CAN_W, CAN_H), color=CANVAS_WHITE),
        box(((CAN_FACE + CAN_D) / 2, 0.0, CAN_H - CAN_BAR / 2), (CAN_D - CAN_FACE, CAN_W, CAN_BAR), color=CANVAS_WHITE),
        box(((CAN_FACE + CAN_D) / 2, 0.0, CAN_BAR / 2), (CAN_D - CAN_FACE, CAN_W, CAN_BAR), color=CANVAS_WHITE),
        box(((CAN_FACE + CAN_D) / 2, hw - CAN_BAR / 2, CAN_H / 2), (CAN_D - CAN_FACE, CAN_BAR, CAN_H - 2 * CAN_BAR),
            color=CANVAS_WHITE),
        box(((CAN_FACE + CAN_D) / 2, -hw + CAN_BAR / 2, CAN_H / 2), (CAN_D - CAN_FACE, CAN_BAR, CAN_H - 2 * CAN_BAR),
            color=CANVAS_WHITE),
    ]
    return Part("canvas", pos, shapes, mass=CANVAS_MASS, color=CANVAS_WHITE, friction=CANVAS_FRICTION, grip=1)


# ---------------------------------------------------------------- layout
EASEL_START = np.array([0.120, 0.030, TOP + LEG_T + REAR_T])  # lying on the rear leg's back face
CANVAS_START = np.array([0.250, -0.210, TOP])
STAND_FRONT_X, STAND_Y = 0.110, -0.015  # where the demo stands the easel (front of the front feet, centre line)
EASEL = make_easel(tuple(EASEL_START))
CANVAS = make_canvas(tuple(CANVAS_START))
OBJECTS = [EASEL] if PROBE else [EASEL, CANVAS]
PRESSED_PARTS = ()

# ---------------------------------------------------------------- scoring
FOOT_ON = 4.0 * MM  # a foot on the table: its lowest corner within this of the table top
LEAN_OK = (10.0, 35.0)  # a standing easel's frame leans back between these (deg)
STOP_SLACK = 10.0 * MM  # at its stop: the ruler depth within this of the full depth
SHORT_SLACK = 30.0 * MM
STILL_MOVE, STILL_DEG = 1.0 * MM, 2.0
SEAT_X = (LEDGE_X[1] - 3.0 * MM, LEDGE_X[1] + 12.0 * MM)  # canvas bottom-back corners: along the frame ...
SEAT_Z = (-3.0 * MM, LEDGE_D + 2.0 * MM)  # ... and out from the legs' front face (on the ledge)
SEAT_ALIGN_DEG = 10.0  # the canvas's face within this of the easel's face (leaning back on it)

_TIMERS = {}
_EVER = {}
_MEMORY = {"episode": None}
_WINDOWS = {}


def _episode_memory():
    if _MEMORY["episode"] != _RUN["episode"]:
        _TIMERS.clear()
        _EVER.clear()
        _WINDOWS.clear()
        _MEMORY["episode"] = _RUN["episode"]


def ever(key, cond):
    if _RUN["t"] is None:
        return bool(cond)
    _episode_memory()
    _EVER[key] = bool(_EVER.get(key, False) or cond)
    return _EVER[key]


def _rot_deg(a, b):
    return math.degrees(math.acos(float(np.clip((np.trace(a.T @ b) - 1) / 2, -1.0, 1.0))))


def still(key, state):
    """Over the last second the object moved under 1 mm and turned under 2 degrees (offline: its velocity)."""
    t = _RUN["t"]
    if t is None:
        return bool(np.linalg.norm(state.lin_vel) < 0.01)
    _episode_memory()
    hist = _WINDOWS.setdefault(key, deque())
    if hist and t < hist[-1][0]:
        hist.clear()
    now = (np.asarray(state.origin, float), state.rotation())
    if not hist or t > hist[-1][0] + 1e-9:
        hist.append((t, now))
    while len(hist) > 1 and hist[1][0] <= t - STILL_S + 1e-9:
        hist.popleft()
    if t - hist[0][0] < STILL_S - 0.051:
        return False
    return all(np.linalg.norm(now[0] - old[0]) <= STILL_MOVE and _rot_deg(now[1], old[1]) <= STILL_DEG
               for _, old in hist)


def _box_clear(state, clear):
    hands = grippers()
    if hands is None:
        return True
    low, high = np.asarray(state.low, float), np.asarray(state.high, float)
    return all(float(np.linalg.norm(np.maximum(np.maximum(low - tcp, tcp - high), 0.0))) >= clear
               for tcp, _ in hands.values())


def easel_terms(easel):
    R, o = easel.rotation(), np.asarray(easel.origin, float)
    q = float(easel.joints["rear_leg"])
    front, rear = easel_points(R, o, q)
    lows = [min(p[2] for p in v) - TOP for v in front.values()] + [min(p[2] for p in rear) - TOP]
    lean = math.degrees(math.acos(float(np.clip(R[2, 0], -1.0, 1.0))))
    face_up = R[2, 2] > 0.0  # front face toward the sky side: leaning back, not on its face
    standing = bool(all(abs(z) <= FOOT_ON for z in lows) and LEAN_OK[0] <= lean <= LEAN_OK[1] and face_up)
    depth = depth_of(R, o, q)
    return {"standing": standing, "lean": lean, "foot_z": lows, "depth": depth, "open_deg": -math.degrees(q),
            "short": FULL_DEPTH - depth}


def canvas_terms(canvas, easel):
    Re, oe = easel.rotation(), np.asarray(easel.origin, float)
    Rc, oc = canvas.rotation(), np.asarray(canvas.origin, float)
    corners = [oc + Rc @ np.array([CAN_D, y, 0.0]) for y in (-CAN_W / 2, CAN_W / 2)]
    local = [Re.T @ (c - oe) for c in corners]  # in the easel's frame: x up the frame, z out of its face
    on_ledge = all(SEAT_X[0] <= p[0] <= SEAT_X[1] and SEAT_Z[0] <= p[2] <= SEAT_Z[1] for p in local)
    face_c, face_e = -Rc[:, 0], Re[:, 2]  # outward normals of the canvas face and the easel's front
    align = math.degrees(math.acos(float(np.clip(face_c @ face_e, -1.0, 1.0))))
    return {"seated": bool(on_ledge and align <= SEAT_ALIGN_DEG), "on_ledge": on_ledge, "align": align,
            "corners_in_easel": local}


def score_terms(objects):
    easel = objects["easel"]
    e = easel_terms(easel)
    free = _box_clear(easel, RELEASE_CLEAR)
    rest = still("easel", easel)
    out = {**e, "released": free, "still": rest}
    if not PROBE:
        canvas = objects["canvas"]
        c = canvas_terms(canvas, easel)
        out.update(c)
        out["released"] = bool(free and _box_clear(canvas, RELEASE_CLEAR))
        out["still"] = bool(rest and still("canvas", canvas))
    if DIAG:
        _diag(out)
    return out


def score(objects):
    """0 if the easel is not standing on its three feet; otherwise the lowest of the rear-leg, canvas and let-go
    terms (README "Scoring")."""
    t = score_terms(objects)
    if not t["standing"]:
        return 0.0
    leg = 1.0 if t["short"] <= STOP_SLACK else (0.75 if t["short"] <= SHORT_SLACK else 0.5)
    canvas = 1.0 if PROBE or t["seated"] else 0.5
    done = 1.0 if (t["released"] and t["still"]) else 0.25
    return min(leg, canvas, done)


def success(objects):
    return score(objects) >= 1.0


_DIAG = {"step": -1}


def _diag(t):
    step = _RUN["step"]
    if step == _DIAG["step"] or step % 10:
        return
    _DIAG["step"] = step
    tt = _RUN["t"] if _RUN["t"] is not None else -1.0
    extra = ""
    if "align" in t:
        extra = f" seated={int(t['seated'])} on_ledge={int(t['on_ledge'])} align={t['align']:.1f}"
    print(f"EASEL_DIAG t={tt:.2f} stand={int(t['standing'])} lean={t['lean']:.1f} open={t['open_deg']:.1f} "
          f"depth_mm={t['depth'] * 1000:.1f} feet_mm={tuple(round(z * 1000, 1) for z in t['foot_z'])} "
          f"free={int(t['released'])} still={int(t['still'])}{extra}", flush=True)


# ---------------------------------------------------------------- the scripted demo
PAD_TIP = 0.0128  # the pad tips are this far past the TCP
GRIP_X = 120.0 * MM  # the left hand pinches the left front leg this far up from its foot (near the frame's CoM)
LEG_OPEN = -0.021  # pads 42 mm apart round the 18 mm leg
LEG_PINCH = -0.0020  # pads aimed 4 mm apart: ~35 N of squeeze on the 18 mm leg (2.5 N per mm of inset)
CAN_OPEN = -0.030  # pads 60 mm apart round the 38 mm canvas
CAN_PINCH = -0.0100  # pads aimed 20 mm apart: ~45 N on the 38 mm canvas
CAN_RELEASE = -0.026
CAN_BELOW = 20.0 * MM  # the pad tips this far under the canvas's top edge (the top bar is 19 mm)
LEFT_CLOSE_SIGN = -1.0  # which way round the pinch faces (the arm's wrist limits prefer one)
CAN_CLOSE_SIGN = 1.0
LEFT_CLEAR = (0.0, 0.08, 0.04)  # the left hand's last move, out of the canvas's way
CAN_UP = 0.030  # the fingers rise this far after letting go (tips 10 mm over the top edge)
CAN_CLEAR = (-0.05, -0.03, 0.0)  # the right hand's move after backing off the canvas ...
CAN_AWAY = (-0.05, -0.16, -0.03)  # ... and its last, out to the right of the easel (both 3 cm clear of their boxes)
CAN_HOVER = 0.012  # the upright canvas crosses in front of the easel with its bottom this far over the lip
CAN_BESIDE_Y = -0.225  # ... after coming forward this far to the right
CAN_TIP = 8.0  # degrees it is tipped back before the fingers open (it then falls back onto the legs)
RAISE_DEG = 30.0  # the frame is tipped up this far from lying before it is lifted (lean 60)
LIFT_Z = 0.125
TOUCH_LEAN = 60.0
FINAL_HOLD_S = 3.0
PROBE_PARK = (0.200, -0.200, TOP + 0.160)  # probe copies: the right arm waits here (straight down)
PARK = {"left": np.array([0.150, 0.300, TOP + 0.220]), "right": np.array([0.150, -0.300, TOP + 0.220])}


class Grip(float):
    def __new__(cls, yaw, matrix=None):
        if matrix is not None:  # the float is the yaw of the closing axis seen from above (for yaw-only readers)
            closing = np.asarray(matrix, float)[:, 0]
            yaw = math.atan2(-closing[0], closing[1])
        value = super().__new__(cls, yaw)
        value.grip_matrix = None if matrix is None else np.array(matrix, float)
        return value


def _install_grip_matrix():
    from yam_arena import demo as arena_demo
    original = arena_demo.grip_rotation
    if getattr(original, "_grip_matrix", False):
        return

    def grip_rotation(yaw=0.0):
        matrix = getattr(yaw, "grip_matrix", None)
        return matrix.copy() if matrix is not None else original(float(yaw))

    grip_rotation._grip_matrix = True
    grip_rotation.__wrapped__ = original
    arena_demo.grip_rotation = grip_rotation


_install_grip_matrix()


def _grip(close, approach):
    """TCP rotation: columns = closing axis, approach x closing, approach (the kit's convention)."""
    a = np.asarray(approach, float)
    a = a / np.linalg.norm(a)
    c = np.asarray(close, float)
    c = c - a * float(c @ a)
    c = c / np.linalg.norm(c)
    return np.column_stack((c, np.cross(a, c), a))


def easel_pose_start():
    return _T(np.eye(3), EASEL_START.copy())


def _hand_in_easel():
    """Left hand on the left front leg, GRIP_X up from its foot: approach into the front face (top-down while it
    lies), closing across the leg, the TCP mid-way through the leg's thickness."""
    d = np.array([LEG_L, LEG_TOP_Y - LEG_FOOT_Y, 0.0])
    d /= np.linalg.norm(d)
    close = LEFT_CLOSE_SIGN * np.cross([0.0, 0.0, 1.0], d)
    R = _grip(close, (0.0, 0.0, -1.0))
    p = np.array([GRIP_X, leg_y(GRIP_X), -LEG_T / 2])
    return _T(R, p)


H_EASEL = _hand_in_easel()


def _pivoted(pose, pivot_part, R_new):
    """The easel turned to rotation R_new about the part-frame point pivot_part (kept where it is in the world)."""
    piv_w = pose[:3, :3] @ pivot_part + pose[:3, 3]
    return _T(R_new, piv_w - R_new @ pivot_part)


REAR_PIVOT = np.array([PIN[0] - REAR_L, 0.0, -LEG_T - REAR_T])  # the folded rear foot's lower edge (lying)


def _touch_pose():
    """The open easel (rear leg at its stop) leaning TOUCH_LEAN, its rear foot 2 mm over the spot where it will
    stand at the end."""
    q = -STOP_DEG * DEG
    Rf, of = easel_standing_pose(STAND_LEAN, q, STAND_FRONT_X, STAND_Y)
    _, rear_f = easel_points(Rf, of, q)
    contact = min(rear_f, key=lambda p: p[2])  # the rear foot's front-lower edge
    contact_part = Rf.T @ (contact - of)  # hmm: rear foot point in the part frame at the stop
    R = easel_rot(TOUCH_LEAN)
    o = contact - R @ contact_part
    _, rear_t = easel_points(R, o, q)
    o = o + np.array([0.0, 0.0, TOP + 2.0 * MM - min(p[2] for p in rear_t)])
    return _T(R, o), contact_part


def _rolled(lean, contact_part, contact_world):
    R = easel_rot(lean)
    return _T(R, contact_world - R @ contact_part)


def easel_plan():
    """[(label, seconds, planned easel pose, finger, read key)]"""
    p0 = easel_pose_start()
    up = lambda pose, dz: _T(pose[:3, :3], pose[:3, 3] + np.array([0.0, 0.0, dz]))  # noqa: E731
    plan = [("over the left leg", 2.5, None, LEG_OPEN, "grasp"),
            ("fingers down round the leg", 1.5, p0, LEG_OPEN, None),
            ("pinch the leg", 1.0, p0, LEG_PINCH, None)]
    if DEMO_NOLIFT:
        # wrong way: tip the folded easel up on the table about its feet and let go, leg still folded
        for k, eps in enumerate((20.0, 40.0, 55.0, 68.0)):
            plan.append((f"tip it up folded ({k + 1})", 1.0, _pivoted(p0, REAR_PIVOT, _rot_y(-eps)), LEG_PINCH, None))
        last = plan[-1][2]
        plan += [("let go of the folded easel", 0.8, last, LEG_OPEN, None)]
        return plan, last
    for k, eps in enumerate((10.0, 20.0, RAISE_DEG)):
        plan.append((f"tip the frame up ({k + 1})", 0.9, _pivoted(p0, REAR_PIVOT, _rot_y(-eps)), LEG_PINCH, None))
    tipped = plan[-1][2]
    lifted = up(tipped, LIFT_Z)
    plan += [("lift it clear: the leg drops open", 1.4, lifted, LEG_PINCH, None),
             ("hold while the leg settles on its stop", 1.2, lifted, LEG_PINCH, None)]
    touch, contact_part = _touch_pose()
    plan += [("carry it over its spot", 2.0, up(touch, 0.030), LEG_PINCH, "carry"),
             ("rear foot down", 1.2, touch, LEG_PINCH, None)]
    contact_w = touch[:3, :3] @ contact_part + touch[:3, 3] - np.array([0.0, 0.0, 2.0 * MM])
    for k, lean in enumerate((52.0, 44.0, 36.0, 29.0, STAND_LEAN + 0.6)):
        plan.append((f"roll it onto its front feet ({k + 1})", 0.9, _rolled(lean, contact_part, contact_w),
                     LEG_PINCH, None))
    last = plan[-1][2]
    plan += [("draw the front feet out: the leg on its stop", 1.2, last, LEG_PINCH, "seat"),
             ("let go of the leg", 0.8, last, LEG_OPEN, None)]
    return plan, last


SEAT_PULL = 3.0 * MM  # the front feet are drawn this far past the stop's depth (the rear foot may drag that much)
SEAT_ABOVE = 1.0 * MM  # ... and set this far over the table


def stand_at_stop_from(R_a, o_a, q_a):
    """The standing pose with the rear leg at its stop that keeps the actual rear foot's lower edge where it is
    (drawn SEAT_PULL back) and the actual heading."""
    yaw = math.atan2(R_a[1, 0], R_a[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    q = -STOP_DEG * DEG
    R_t = Rz @ easel_rot(STAND_LEAN)
    low = lambda pts: np.mean(sorted(pts, key=lambda p: p[2])[:2], axis=0)  # noqa: E731  (the foot's lower edge)
    edge_a = low(easel_points(R_a, o_a, q_a)[1])
    edge_t = low(easel_points(R_t, np.zeros(3), q)[1])
    goal = edge_a + SEAT_PULL * (Rz @ np.array([1.0, 0.0, 0.0]))
    goal[2] = TOP + SEAT_ABOVE
    return _T(R_t, goal - edge_t)


# --- reads: the plan is corrected once from the easel's pose before the grasp ("grasp": hand follows the easel)
# and once after the lift ("carry": the held easel is brought to its planned pose)
_READS = {"episode": None, "values": {}}
READ_MAX_SHIFT, READ_MAX_DEG = 0.025, 12.0


def _state_now(name):
    env = _RUN["env"]
    if env is None:
        return None
    try:
        from yam_arena import scene as arena_scene
        return arena_scene.object_states(env, [name])[name]
    except Exception as e:  # noqa: BLE001
        print(f"EASEL_READ {name} failed: {e!r}", flush=True)
        return None


def _axis_angle(rot):
    c = (np.trace(rot) - 1.0) / 2.0
    a = math.acos(float(np.clip(c, -1.0, 1.0)))
    if a < 1e-9:
        return np.array([0.0, 0.0, 1.0]), 0.0
    axis = np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]]) / (2 * math.sin(a))
    return axis, a


def _limit_rotation(rot, max_deg):
    axis, ang = _axis_angle(rot)
    if ang <= math.radians(max_deg):
        return rot
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    a = math.radians(max_deg)
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * (k @ k)


def _read(key, planned, ref_part, follow, name="easel", prior=None):
    """A world correction (4x4) for poses planned from `planned` (the object's planned pose at this point)."""
    if _READS["episode"] != _RUN["episode"]:
        _READS["episode"], _READS["values"] = _RUN["episode"], {}
    values = _READS["values"]
    if key in values:
        return values[key]
    prior = np.eye(4) if prior is None else prior
    st = _state_now(name)
    if st is None:
        values[key] = prior
        return prior
    actual = _T(st.rotation(), np.asarray(st.origin, float))
    plan_w = prior @ planned
    pt_p = plan_w[:3, :3] @ ref_part + plan_w[:3, 3]
    pt_a = actual[:3, :3] @ ref_part + actual[:3, 3]
    if follow:  # the hand goes to where the object is: turn planned -> actual about the reference point
        rot = _limit_rotation(actual[:3, :3] @ plan_w[:3, :3].T, READ_MAX_DEG)
        src, dst = pt_p, pt_a
    else:  # the held object is brought to its plan: turn actual -> planned
        rot = _limit_rotation(plan_w[:3, :3] @ actual[:3, :3].T, READ_MAX_DEG)
        src, dst = pt_a, pt_p
    move = dst - src
    if follow:
        move[2] = 0.0
    n = float(np.linalg.norm(move))
    if n > READ_MAX_SHIFT:
        move *= READ_MAX_SHIFT / n
    d = _T(rot, src + move - rot @ src) @ prior
    values[key] = d
    err = math.degrees(_axis_angle(actual[:3, :3] @ plan_w[:3, :3].T)[1])
    print(f"EASEL_READ {key} t={_RUN['t']:.2f} ref_err_mm={tuple(round(float(v) * 1000, 2) for v in pt_p - pt_a)} "
          f"rot_err_deg={err:.2f}", flush=True)
    return d


def _seat_read(planned, prior):
    """Once the easel is on its rear foot just over its front feet: a world correction that brings the held easel
    from where it is to standing at the stop round its actual rear foot (the hand moves with it)."""
    key = "seat"
    if _READS["episode"] != _RUN["episode"]:
        _READS["episode"], _READS["values"] = _RUN["episode"], {}
    values = _READS["values"]
    if key in values:
        return values[key]
    plan_w = prior @ planned
    st = _state_now("easel")
    if st is None:
        actual, q_a = plan_w, -STOP_DEG * DEG
    else:
        actual, q_a = _T(st.rotation(), np.asarray(st.origin, float)), float(st.joints["rear_leg"])
    target = stand_at_stop_from(actual[:3, :3], actual[:3, 3], q_a)
    move = target @ _inv(actual)
    ang = math.degrees(_axis_angle(move[:3, :3])[1])
    shift = float(np.linalg.norm(target[:3, :3] @ GRIP_REF + target[:3, 3] - actual[:3, :3] @ GRIP_REF
                                 - actual[:3, 3]))
    d = (move if ang <= 10.0 and shift <= 0.030 else np.eye(4)) @ prior
    values[key] = d
    if st is not None:
        print(f"EASEL_READ seat t={_RUN['t']:.2f} open_deg={-math.degrees(q_a):.1f} grip_shift_mm={shift * 1000:.1f} "
              f"turn_deg={ang:.2f}{'' if ang <= 10.0 and shift <= 0.030 else ' (too far: kept the plan)'}",
              flush=True)
    return d


GRIP_REF = np.array([GRIP_X, leg_y(GRIP_X), -LEG_T / 2])


class PlanWaypoint(Waypoint):
    """A waypoint worked out when played: planned object pose @ hand-in-object, corrected by the reads so far."""

    def __init__(self, duration, pose, hand, finger, label, corr, offset=(0.0, 0.0, 0.0), keep=False):
        self.duration = duration * (1.0 if keep else DEMO_PACE)
        self.pose, self.hand, self.finger, self.label, self.corr = pose, hand, finger, label, corr
        self.offset = np.asarray(offset, float)

    def _hand(self):
        h = self.corr() @ self.pose @ self.hand
        h[:3, 3] += self.offset
        return h

    @property
    def pos(self):
        return tuple(float(v) for v in self._hand()[:3, 3])

    @property
    def yaw(self):
        return Grip(0.0, self._hand()[:3, :3])


def _left_waypoints():
    plan, last = easel_plan()
    p0 = easel_pose_start()
    keys = [p[4] for p in plan]
    carry_planned = plan[keys.index("carry") - 1][2] if "carry" in keys else None  # where the easel is at the read
    grasp_corr = lambda: _read("grasp", p0, GRIP_REF, True)  # noqa: E731
    carry_corr = lambda: _read("carry", carry_planned, GRIP_REF + np.array([0.0, 0.0, 0.0]), False,  # noqa: E731
                               prior=grasp_corr())
    seat_corr = lambda: _seat_read(last, carry_corr())  # noqa: E731
    wps, corr = [], (lambda: np.eye(4))
    for label, secs, pose, finger, key in plan:
        if key == "grasp":
            corr = grasp_corr
        elif key == "carry":
            corr = carry_corr
        elif key == "seat":
            corr = seat_corr
        if pose is None:  # approach: 60 mm back along the approach from the grasp
            wps.append(PlanWaypoint(secs, p0, H_EASEL, finger, label, corr, offset=(0.0, 0.0, 0.060)))
            continue
        keep = label.startswith(("pinch", "let go"))
        wps.append(PlanWaypoint(secs, pose, H_EASEL, finger, label, corr, keep=keep))
    # back out of the leg along the approach, then park
    back = last @ H_EASEL
    out_dir = -back[:3, 2]
    wps.append(PlanWaypoint(1.2, last, H_EASEL, LEG_OPEN, "fingers back off the leg", corr,
                            offset=tuple(0.060 * out_dir)))
    clear, away = ((0.0, 0.0, 0.05), (0.02, 0.03, 0.05)) if DEMO_NOLIFT else ((0.0, 0.05, 0.08), LEFT_CLEAR)
    wps.append(PlanWaypoint(1.5, last, H_EASEL, LEG_OPEN, "clear of the easel", corr,
                            offset=tuple(0.060 * out_dir + np.asarray(clear))))
    wps.append(PlanWaypoint(1.5, last, H_EASEL, LEG_OPEN, "out of the canvas's way", corr,
                            offset=tuple(0.060 * out_dir + np.asarray(away))))
    return wps


# --- the canvas: right hand at its top-right corner, from the upper right
def _hand_in_canvas():
    """Right hand straight down on the middle of the canvas's top edge, pads across its 38 mm depth (one on the face,
    one on the top stretcher bar's back), pad tips CAN_BELOW under the top edge."""
    R = _grip((CAN_CLOSE_SIGN, 0.0, 0.0), (0.0, 0.0, -1.0))
    tip = np.array([CAN_D / 2, 0.0, CAN_H - CAN_BELOW])
    return _T(R, tip + np.array([0.0, 0.0, PAD_TIP]))


H_CANVAS = _hand_in_canvas()


def canvas_pose_start():
    return _T(np.eye(3), CANVAS_START.copy())


def canvas_on_ledge(clear=0.0, lean_extra=0.0, open_deg=STOP_DEG):
    """The canvas seated on the standing easel's ledge, leaning back on its legs (clear: lifted this far along the
    frame; lean_extra: leaning that much less than the easel)."""
    q = -open_deg * DEG
    stand_lean = _standing_lean(q)
    Re, oe = easel_standing_pose(stand_lean, q, STAND_FRONT_X, STAND_Y)
    lean = stand_lean - lean_extra
    Rc = _rot_y(lean)  # canvas frame: x back into it, z up its face; leaning back by `lean`
    # bottom-back edge of the canvas on the ledge top, against the legs' front face (easel frame x = ledge top, z = 0)
    corner_e = oe + Re @ np.array([LEDGE_X[1] + LIP_X + 1.0 * MM, 0.0, 0.5 * MM])
    o = corner_e - Rc @ np.array([CAN_D, 0.0, 0.0]) + Re[:, 0] * clear + Re[:, 2] * clear * 0.5
    return _T(Rc, o)


def canvas_upright_at_ledge(rise=0.002, gap=0.003, y=None, tip_back=0.0):
    """The canvas upright in front of the standing easel: its bottom `rise` over the ledge lip's top edge, its back
    `gap` in front of the legs' front face at that height, centred at y (default the easel's centre line), turned back
    by tip_back degrees about its bottom-back edge."""
    q = -STOP_DEG * DEG
    Re, oe = easel_standing_pose(STAND_LEAN, q, STAND_FRONT_X, STAND_Y)
    lip = oe + Re @ np.array([LEDGE_X[1] + LIP_X, 0.0, LEDGE_D])
    zb = lip[2] + rise
    n = Re[:, 2]  # the legs' front face: points p with n . (p - oe) = 0
    x_face = oe[0] + (-(n[2] * (zb - oe[2]))) / n[0] if abs(n[0]) > 1e-9 else oe[0]
    y = oe[1] if y is None else y
    o = np.array([x_face - gap - CAN_D, y, zb])
    R = _rot_y(tip_back)
    edge = o + np.array([CAN_D, 0.0, 0.0])  # bottom-back edge stays put while it tips back
    return _T(R, edge - R @ np.array([CAN_D, 0.0, 0.0]))


def _right_waypoints(t_wait):
    c0 = canvas_pose_start()
    up = lambda pose, dz: _T(pose[:3, :3], pose[:3, 3] + np.array([0.0, 0.0, dz]))  # noqa: E731
    can_ref = np.array([CAN_D / 2, 0.0, CAN_H / 2])
    grab_corr = lambda: _read("canvas", c0, can_ref, True, name="canvas")  # noqa: E731
    q = -STOP_DEG * DEG
    stand = _T(*easel_standing_pose(STAND_LEAN, q, STAND_FRONT_X, STAND_Y))
    ledge_ref = np.array([LEDGE_X[1], 0.0, LEDGE_D / 2])
    place_corr = lambda: _read("ledge", stand, ledge_ref, True)  # noqa: E731  (the hand follows the standing easel)
    over = tuple(float(v) for v in (c0 @ H_CANVAS)[:3, 3] + np.array([0.0, 0.0, 0.050]))
    rot0 = Grip(0.0, H_CANVAS[:3, :3])
    beside = canvas_upright_at_ledge(rise=CAN_HOVER, gap=0.018, y=CAN_BESIDE_Y)
    front = canvas_upright_at_ledge(rise=CAN_HOVER, gap=0.018)
    wps = [Waypoint(2.5, over, CAN_OPEN, "over the canvas", rot0),
           Waypoint(max(t_wait - 2.5, 0.5), over, CAN_OPEN, "wait for the easel", rot0),
           PlanWaypoint(1.5, c0, H_CANVAS, CAN_OPEN, "fingers down over its top edge", grab_corr),
           PlanWaypoint(1.0, c0, H_CANVAS, CAN_PINCH, "pinch the canvas", grab_corr, keep=True),
           PlanWaypoint(1.2, up(c0, 0.045), H_CANVAS, CAN_PINCH, "lift it", grab_corr),
           PlanWaypoint(2.0, beside, H_CANVAS, CAN_PINCH, "beside the easel, in front of its legs", place_corr),
           PlanWaypoint(2.0, front, H_CANVAS, CAN_PINCH, "in front of the easel", place_corr),
           PlanWaypoint(1.5, canvas_upright_at_ledge(), H_CANVAS, CAN_PINCH, "down onto the ledge", place_corr),
           PlanWaypoint(1.0, canvas_upright_at_ledge(tip_back=CAN_TIP), H_CANVAS, CAN_PINCH, "tip it back a little",
                        place_corr),
           PlanWaypoint(0.8, canvas_upright_at_ledge(tip_back=CAN_TIP), H_CANVAS, CAN_RELEASE, "let go", place_corr,
                        keep=True)]
    placed = canvas_upright_at_ledge(tip_back=CAN_TIP)
    wps += [PlanWaypoint(1.2, placed, H_CANVAS, CAN_RELEASE, "fingers up off its top edge", place_corr,
                         offset=(0.0, 0.0, CAN_UP)),
            PlanWaypoint(1.5, placed, H_CANVAS, CAN_OPEN, "clear of the canvas", place_corr,
                         offset=tuple(np.array([0.0, 0.0, CAN_UP]) + np.asarray(CAN_CLEAR))),
            PlanWaypoint(1.5, placed, H_CANVAS, CAN_OPEN, "out to the right of the easel", place_corr,
                         offset=tuple(np.array([0.0, 0.0, CAN_UP]) + np.asarray(CAN_AWAY)))]
    return wps


def demo():
    _READS["episode"], _READS["values"] = None, {}
    left = _left_waypoints()
    t_left = sum(w.duration for w in left)
    if PROBE:
        right = [Waypoint(t_left, tuple(float(v) for v in PROBE_PARK), FINGER_OPEN, "park", 0.0)]
    else:
        right = _right_waypoints(t_left)
    total = max(t_left, sum(w.duration for w in right)) + FINAL_HOLD_S
    for arm, wps in (("left", left), ("right", right)):
        rest = total - sum(w.duration for w in wps)
        last = wps[-1]
        wps.append(Waypoint(max(rest, 0.5), last.pos, last.finger, "hold clear", last.yaw))
    return {"left": left, "right": right}


# ---------------------------------------------------------------- offline test layouts (states.yaml)
def _quat(rot):
    from yam_arena.ik import matrix_to_quat_xyzw
    return [float(v) for v in matrix_to_quat_xyzw(np.asarray(rot, float))]


def easel_layout(lean=None, open_deg=STOP_DEG, front_x=STAND_FRONT_X, y=STAND_Y, lift=0.0):
    """[origin, quat] of the easel standing with its rear leg open_deg, all feet on the table (lift: raised)."""
    q = -open_deg * DEG
    lean = _standing_lean(q) if lean is None else lean
    R, o = easel_standing_pose(lean, q, front_x, y)
    return [float(v) for v in o + np.array([0.0, 0.0, lift])], _quat(R)


def easel_fallen_back():
    """The folded easel lying face up where it was set down (it tipped back)."""
    return [float(v) for v in EASEL_START + np.array([0.0, -0.03, 0.0])], [0.0, 0.0, 0.0, 1.0]


def canvas_layout(clear=0.0, lean_extra=0.0, dx=0.0, open_deg=STOP_DEG):
    pose = canvas_on_ledge(clear, lean_extra, open_deg)
    return [float(v) for v in pose[:3, 3] + np.array([dx, 0.0, 0.0])], _quat(pose[:3, :3])


def canvas_standing_start(dx=0.0, dy=0.0):
    return [float(v) for v in CANVAS_START + np.array([dx, dy, 0.0])], [0.0, 0.0, 0.0, 1.0]


def canvas_upright_layout(tip_back=CAN_TIP):
    pose = canvas_upright_at_ledge(tip_back=tip_back)
    return [float(v) for v in pose[:3, 3]], _quat(pose[:3, :3])
