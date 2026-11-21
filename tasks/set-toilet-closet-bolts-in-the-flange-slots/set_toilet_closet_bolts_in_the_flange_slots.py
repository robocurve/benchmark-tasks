"""Set a toilet's two closet bolts in a PVC closet flange: T-heads down the slots, slid to the line, retainers down.

An Oatey 43495 3 x 4 in PVC closet flange with its stainless-steel ring is screwed to a square of 23/32 in plywood
subfloor laid on two 2x4s, like a floor over joists. Two 1/4-20 x 2-1/4 in zinc-plated steel closet bolts stand on
their heads on rubber mats, one each side of the flange, each with its white plastic retainer washer started 2 mm onto
its thread. Each arm sets the bolt on its own side: it takes the bolt by its retainer's rim and lifts it so it hangs
head-down, lowers the T-head through the wide end of the ring's slot onto the shelf under the ring, slides
it round the slot to the pencil line drawn on the floor through the flange's centre parallel to the wall behind the
toilet, lets go, and pushes the retainer about 51 mm down the thread until it bears on the ring. Metres; x forward
from the arm bases, y left, z up.

The bolt (module section "the closet bolt", reusable) is a free mechanism: the bolt's body (T-head and shank) and
the retainer on a slide along the shank that moves only past a set force (joint stiction): the thread grip of the
retainer's fingers.
"""
import math
import pathlib
import sys
from collections import deque

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, box, cyl  # noqa: E402
from yam_joints import Link, Slide  # noqa: E402
from yam_mesh import Mesh, MeshMechanism, MeshPart, revolve  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
STILL_S = 1.0
TEST_GRIPPERS = None
DIAG = False  # GPU diagnostic copies print BOLT_DIAG lines
DEMO_PACE = 1.0  # scales every demo duration except the fingers' close and open
DEMO_WRONG = ""  # wrong-way copies: "noretainer" = slide both bolts to the line and let go, never push a retainer


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
_EVER = {}


def grippers():
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


def _episode_memory():
    global _MEMORY_EPISODE
    if _MEMORY_EPISODE != _RUN["episode"]:
        _EVER.clear()
        _MEMORY_EPISODE = _RUN["episode"]


def ever(key, cond):
    """Remember a satisfied condition until the next episode; offline each call stands alone."""
    if _RUN["t"] is None:
        return bool(cond)
    _episode_memory()
    _EVER[key] = bool(_EVER.get(key, False) or cond)
    return _EVER[key]


TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S, SCORE_OVER = 1.0, "end"
RUN_LIMIT_STEPS = 2400  # 120 s at 20 Hz
MM = 0.001
TAU = 2 * math.pi
DEG = math.pi / 180

PLYWOOD = (0.78, 0.64, 0.42)
SPRUCE = (0.86, 0.76, 0.56)
PVC_WHITE = (0.93, 0.93, 0.91)
STAINLESS = (0.72, 0.73, 0.75)
ZINC_YELLOW = (0.80, 0.70, 0.36)
RETAINER_WHITE = (0.96, 0.96, 0.94)
RUBBER_BLACK = (0.08, 0.08, 0.09)
PENCIL = (0.96, 0.80, 0.08)  # the layout line, in yellow lumber crayon
SCREW_GREY = (0.55, 0.55, 0.56)


# ---------------------------------------------------------------- convex pieces
def _oriented(name, points, faces, color, proxy=(), collision="convex"):
    points, faces = np.asarray(points, float), np.asarray(faces, int)
    tri = (points - points.mean(axis=0))[faces]
    if np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum() < 0:
        faces = faces[:, ::-1]
    return Mesh(name, points, faces, collision=collision, color=color, proxy=tuple(proxy))


def _slab(name, quad_xy, z0, z1, color, proxy=()):
    """A convex prism: the quadrilateral (or any convex polygon, CCW or CW) quad_xy from z0 up to z1."""
    q = [tuple(map(float, p)) for p in quad_xy]
    n = len(q)
    points = [(x, y, z0) for x, y in q] + [(x, y, z1) for x, y in q]
    faces = []
    for i in range(n):
        k = (i + 1) % n
        faces += [(i, k, n + k), (i, n + k, n + i)]
    faces += [(0, j + 1, j) for j in range(1, n - 1)]
    faces += [(n, n + j, n + j + 1) for j in range(1, n - 1)]
    return _oriented(name, points, faces, color, proxy)


def _polar(r, a):
    return (r * math.cos(a), r * math.sin(a))


# ---------------------------------------------------------------- floor mock-up
# A 14 x 14 in square of 23/32 in (18.3 mm) plywood subfloor, nailed to two 2x4s laid flat (38.1 x 88.9 mm) at its
# left and right edges, like a floor over joists; the flange's hub goes down through a hole in it. Fixed to the bench.
PLY_T = 18.3 * MM
SLEEPER_T, SLEEPER_W = 38.1 * MM, 88.9 * MM
PANEL = 355.6 * MM
PANEL_XY = (0.178, 0.0)
FLOOR = TOP + SLEEPER_T + PLY_T  # floor surface, 56.4 mm above the bench
HOLE = 104.0 * MM  # square hole for the hub (OD 101.6), covered by the flange
FC = np.array([0.180, 0.0])  # the flange's centre


def make_floor():
    px, py = PANEL_XY
    h = PANEL / 2
    zc = TOP + SLEEPER_T + PLY_T / 2
    hx0, hx1 = FC[0] - HOLE / 2, FC[0] + HOLE / 2
    hy0, hy1 = FC[1] - HOLE / 2, FC[1] + HOLE / 2
    shapes = [
        box((px - h + (hx0 - (px - h)) / 2, py, zc), (hx0 - (px - h), PANEL, PLY_T)),  # behind the hole
        box((hx1 + (px + h - hx1) / 2, py, zc), (px + h - hx1, PANEL, PLY_T)),  # in front of it
        box((FC[0], hy1 + (py + h - hy1) / 2, zc), (HOLE, py + h - hy1, PLY_T)),  # left of it
        box((FC[0], py - h + (hy0 - (py - h)) / 2, zc), (HOLE, hy0 - (py - h), PLY_T)),  # right of it
    ]
    for side in (1, -1):
        shapes.append(box((px, side * (h - SLEEPER_W / 2), TOP + SLEEPER_T / 2), (PANEL, SLEEPER_W, SLEEPER_T),
                          color=SPRUCE))
    # the layout line: lumber crayon on the plywood, through the flange's centre, parallel to the wall behind
    for side in (1, -1):
        shapes.append(box((FC[0], side * (LINE_R0 + LINE_R1) / 2, FLOOR + 0.1 * MM), (LINE_W, LINE_R1 - LINE_R0,
                                                                                         0.2 * MM), color=PENCIL))
    return Part("subfloor", (0.0, 0.0, 0.0), shapes, mass=None, color=PLYWOOD, friction=(0.5, 0.45))


# ---------------------------------------------------------------- the closet flange (Oatey 43495)
# Published: 3 x 4 in PVC closet flange with stainless-steel ring, 7 in across, 2 5/16 in tall (retailer); the
# Oatey 42778 ring for 3 or 4 in flanges: ID 5.08 in, screw centre 6.00 in, OD 7.13 in (7.01 in on its product page),
# overall height 0.32 in, six countersunk holes, two closet-bolt slots (submittal drawing: the slots are 180 degrees
# apart, each with a wide end at one end). Estimates (scaled from the drawing or set to fit): ring sheet 16 gauge
# (1.52 mm); slot centre line 75 mm from the flange's centre, narrow part 7.5 mm wide, wide end 14 mm wide and 24 mm
# long, 53 mm end to end; the shelf under the ring 3.5 mm below its underside (Oatey: "bolt slot shelf eliminates
# bolt fall through"); PVC hub 4.0 in outside (fits inside 4 in pipe), 3.5 in inside (over 3 in pipe).
RING_R_IN, RING_R_OUT = 64.5 * MM, 89.0 * MM
RING_H, SHEET = 8.13 * MM, 1.52 * MM
RING_TOP = FLOOR + RING_H
RING_UNDER = RING_TOP - SHEET
CHANNEL = 3.5 * MM  # the head's space between the shelf and the ring's underside
SHELF = RING_UNDER - CHANNEL
SKIRT_T = 1.5 * MM
HUB_RI, HUB_RO = 44.45 * MM, 50.8 * MM
LIP_RO = 66.0 * MM
LIP_TOP = RING_TOP + 0.5 * MM
PLATE_RO = 87.0 * MM
R_SLOT = 75.0 * MM
W_NARROW, W_WIDE = 7.5 * MM, 14.0 * MM
# Along the slot's centre line from its mark (s, mm, positive toward the wide end): closed end, narrow part's end,
# taper's end, wide end's far end.
S_CLOSED, S_NARROW, S_TAPER, S_FAR = -12.0 * MM, 14.0 * MM, 17.0 * MM, 41.0 * MM
S_WIDE_MID = (S_TAPER + S_FAR) / 2
MARK_DEG = (-90.0, 90.0)  # slot 0 (right, the robot's right arm) and slot 1 (left): where the line crosses them
LINE_R0, LINE_R1, LINE_W = 93.0 * MM, 150.0 * MM, 3.0 * MM  # the floor line's two segments (from the centre)
SCREW_R, SCREW_HEAD_R = 76.2 * MM, 4.2 * MM


def slot_theta(k, s):
    """Angle (rad, about the flange's centre) of the point s along slot k's centre line from its mark."""
    return MARK_DEG[k] * DEG - s / R_SLOT


def slot_point(k, s, r=R_SLOT):
    th = slot_theta(k, s)
    return np.array([FC[0] + r * math.cos(th), FC[1] + r * math.sin(th)])


def slot_tangent(k, s):
    """Unit tangent at s pointing from the wide end toward the mark (counter-clockwise from above)."""
    th = slot_theta(k, s)
    return np.array([-math.sin(th), math.cos(th)])


def slot_half_width(s):
    if s < S_CLOSED or s > S_FAR:
        return 0.0
    if s <= S_NARROW:
        return W_NARROW / 2
    if s <= S_TAPER:
        return W_NARROW / 2 + (W_WIDE - W_NARROW) / 2 * (s - S_NARROW) / (S_TAPER - S_NARROW)
    return W_WIDE / 2


def _ring_pieces():
    """The ring's top sheet as convex pieces: plain sectors, and inner/outer bands either side of each slot."""
    pieces = []
    z0, z1 = RING_UNDER, RING_TOP
    edges = [S_CLOSED, S_NARROW, S_TAPER, S_FAR]
    s_steps = sorted(set([float(v) for v in np.arange(S_CLOSED, S_FAR + 1e-9, 2.5 * MM)] + edges))
    slot_spans = []
    for k in (0, 1):
        for i in range(len(s_steps) - 1):
            sa, sb = s_steps[i], s_steps[i + 1]
            wa = slot_half_width(sa + 1e-9) if sa < sb else 0.0
            wb = slot_half_width(sb - 1e-9)
            ta, tb = slot_theta(k, sa), slot_theta(k, sb)
            for band, (ra0, ra1, rb0, rb1) in (("in", (RING_R_IN, R_SLOT - wa, RING_R_IN, R_SLOT - wb)),
                                              ("out", (R_SLOT + wa, RING_R_OUT, R_SLOT + wb, RING_R_OUT))):
                quad = [_polar(ra0, ta), _polar(ra1, ta), _polar(rb1, tb), _polar(rb0, tb)]
                pieces.append(_slab(f"ring_s{k}_{band}_{i}", [(FC[0] + x, FC[1] + y) for x, y in quad], z0, z1,
                                    STAINLESS))
        # angular span the slot occupies (theta decreases with s)
        slot_spans.append((slot_theta(k, S_FAR), slot_theta(k, S_CLOSED)))
    # plain sectors between the two slots
    spans = sorted(((a % TAU), (b % TAU)) for a, b in slot_spans)
    gaps = [(spans[0][1], spans[1][0]), (spans[1][1], spans[0][0] + TAU)]
    j = 0
    for g0, g1 in gaps:
        n = max(1, int(math.ceil((g1 - g0) / (6.0 * DEG))))
        for i in range(n):
            ta, tb = g0 + (g1 - g0) * i / n, g0 + (g1 - g0) * (i + 1) / n
            quad = [_polar(RING_R_IN, ta), _polar(RING_R_OUT, ta), _polar(RING_R_OUT, tb), _polar(RING_R_IN, tb)]
            pieces.append(_slab(f"ring_p{j}", [(FC[0] + x, FC[1] + y) for x, y in quad], z0, z1, STAINLESS))
            j += 1
    return pieces


def _annulus_proxy(r0, r1, z0, z1, n=12):
    """Offline stand-in for an annulus: n boxes round it (no box covers the middle)."""
    out = []
    rm, w = (r0 + r1) / 2, r1 - r0
    for i in range(n):
        a = (i + 0.5) * TAU / n
        cx, cy = FC[0] + rm * math.cos(a), FC[1] + rm * math.sin(a)
        size = 2 * rm * math.sin(math.pi / n) * 0.9
        # axis-aligned boxes: the larger of the rotated extents, shrunk to stay inside the annulus
        ex = abs(size * math.sin(a)) + abs(w * math.cos(a))
        ey = abs(size * math.cos(a)) + abs(w * math.sin(a))
        out.append(box((cx, cy, (z0 + z1) / 2), (0.6 * ex, 0.6 * ey, z1 - z0)))
    return tuple(out)


def _with_proxy(mesh, extra):
    """The mesh with offline stand-ins added (the shelf under each slot's line, where a released bolt stands)."""
    return Mesh(mesh.name, mesh.points, mesh.faces, collision=mesh.collision, color=mesh.color,
                proxy=tuple(mesh.proxy) + tuple(extra))


def make_flange():
    """Fixed: the PVC body (hub, shelf plate, lip) and the stainless ring with its slots, screwed to the floor."""
    c = (FC[0], FC[1])

    def rev(name, profile, color, samples=72):
        m = revolve(profile, samples=samples, name=name, collision="exact", color=color, proxy=())
        return Mesh(m.name, m.points + np.array([c[0], c[1], 0.0]), m.faces, collision="exact", color=color,
                    proxy=_annulus_proxy(min(r for r, _ in profile), max(r for r, _ in profile),
                                         min(z for _, z in profile), max(z for _, z in profile)))

    meshes = [
        rev("hub", [(HUB_RI, FLOOR - 0.035), (HUB_RO, FLOOR - 0.035), (HUB_RO, SHELF), (HUB_RI, SHELF)], PVC_WHITE),
        _with_proxy(rev("plate", [(HUB_RO, FLOOR), (PLATE_RO, FLOOR), (PLATE_RO, SHELF), (HUB_RO, SHELF)], PVC_WHITE),
                    tuple(box((*slot_point(k, 0.0), (FLOOR + SHELF) / 2), (0.030, 0.016, SHELF - FLOOR))
                          for k in (0, 1))),
        rev("lip", [(HUB_RI, SHELF), (LIP_RO, SHELF), (LIP_RO, LIP_TOP), (HUB_RI, LIP_TOP)], PVC_WHITE),
        rev("skirt", [(RING_R_OUT - SKIRT_T, FLOOR), (RING_R_OUT, FLOOR), (RING_R_OUT, RING_UNDER),
                      (RING_R_OUT - SKIRT_T, RING_UNDER)], STAINLESS),
    ] + _ring_pieces()
    # four flat-head wood screws in the ring's countersunk holes (six holes on a 6.00 in circle; 30 and 90 degrees
    # from the slots' middles)
    shapes = []
    mid = [slot_theta(k, (S_CLOSED + S_FAR) / 2) for k in (0, 1)]
    for a in (mid[0] + 90 * DEG, mid[0] - 90 * DEG, mid[0] + 30 * DEG, mid[1] + 30 * DEG):
        x, y = _polar(SCREW_R, a)
        shapes.append(cyl((FC[0] + x, FC[1] + y, RING_TOP + 0.15 * MM), SCREW_HEAD_R, 0.3 * MM, color=SCREW_GREY))
    return MeshPart("flange", (0.0, 0.0, 0.0), tuple(shapes), mass=None, color=PVC_WHITE, friction=(0.30, 0.25),
                    contact_offset=0.001, meshes=tuple(meshes))


# ---------------------------------------------------------------- the closet bolt (reusable: copy this section)
# A CLOSET BOLT WITH A PUSH-ON RETAINER. A 1/4-20 x 2-1/4 in zinc-plated steel closet (toilet floor) bolt with a flat
# T-head, and its white plastic retainer washer, whose fingers grip the thread. Frame: z along the bolt from the head's
# outer face (z = 0) to the tip, x along the head's length, y across it (the head's width). Rigid body: the head (a
# stadium 19 x 9.5 mm, 2.5 mm thick; estimate) and the shank (6.35 mm, 57.15 mm from under the head; the thread is a
# plain cylinder), 16 g (estimate). The retainer is a link: a disc 22 mm across and 2 mm thick (estimate), on a slide
# along z whose stiction stands for the fingers' grip on the thread: it moves only under RET_STATIC N (joint static
# friction; sliding RET_DYNAMIC N). It starts with its top face RET_START_BELOW_TIP below the tip and can slide down to
# RET_TRAVEL. Tuned: 1 g with a 0.005 kg slide armature (numerical inertia for a light link). Built standing on its
# head; the pads can take it by the retainer's rim (the closing axis along x, the head's length), and it then hangs
# head-down from the pinch. Read the slide with the joint reading, never the link's position.
HEAD_L, HEAD_W, HEAD_T = 19.0 * MM, 9.5 * MM, 2.5 * MM
SHANK_R, SHANK_L = 3.175 * MM, 57.15 * MM
BOLT_L = HEAD_T + SHANK_L  # 59.65 mm
BOLT_MASS = 16.0e-3
RET_R, RET_T = 11.0 * MM, 2.0 * MM
RET_START_BELOW_TIP = 2.0 * MM
RET_Z0 = BOLT_L - RET_START_BELOW_TIP - RET_T / 2  # the retainer's centre at the start
RET_TRAVEL = 53.5 * MM
RET_STATIC, RET_DYNAMIC = 3.5, 2.5  # N, estimates
RET_MASS, RET_ARMATURE = 1.0e-3, 0.005
BOLT_FRICTION = (0.35, 0.30)  # zinc-plated steel (estimate)
RET_FRICTION = (0.50, 0.40)  # plastic (estimate)


def _head_mesh():
    n = 10
    pts = []
    for side, (c0, a0) in ((1, (HEAD_L / 2 - HEAD_W / 2, -90.0)), (-1, (-(HEAD_L / 2 - HEAD_W / 2), 90.0))):
        for i in range(n + 1):
            a = math.radians(a0 + 180.0 * i / n)
            pts.append((c0 + HEAD_W / 2 * math.cos(a), HEAD_W / 2 * math.sin(a)))
    m = len(pts)
    points = [(x, y, 0.0) for x, y in pts] + [(x, y, HEAD_T) for x, y in pts]
    faces = []
    for i in range(m):
        k = (i + 1) % m
        faces += [(i, k, m + k), (i, m + k, m + i)]
    faces += [(0, j + 1, j) for j in range(1, m - 1)]
    faces += [(m, m + j, m + j + 1) for j in range(1, m - 1)]
    return _oriented("head", points, faces, ZINC_YELLOW, (box((0, 0, HEAD_T / 2), (HEAD_L, HEAD_W, HEAD_T)),))


def make_bolt(name, pos, yaw):
    """A closet bolt standing on its head (turned yaw about z), with its retainer started on the tip. `pos` is the
    centre of the head's outer (bottom) face."""
    shank = cyl((0.0, 0.0, HEAD_T + SHANK_L / 2), SHANK_R, SHANK_L)
    retainer = Link("retainer", [cyl((0.0, 0.0, RET_Z0), RET_R, RET_T)],
                    Slide("z", (0.0, 0.0, RET_Z0), -RET_TRAVEL, 0.0, damping=0.5, armature=RET_ARMATURE,
                          static_friction=RET_STATIC, dynamic_friction=RET_DYNAMIC),
                    RET_MASS, color=RETAINER_WHITE, friction=RET_FRICTION)
    return MeshMechanism(name, pos, (shank,), (retainer,), mass=BOLT_MASS, color=ZINC_YELLOW, yaw=yaw,
                         friction=BOLT_FRICTION, contact_offset=0.001, grip=None, meshes={"base": (_head_mesh(),)})


def retainer_slid(bolt):
    """How far the retainer has been pushed down the thread from its start (m, >= 0)."""
    return max(0.0, -float(bolt.joints["retainer"]))


def bolt_frame(bolt):
    return np.asarray(bolt.origin, float), bolt.rotation()


# ---------------------------------------------------------------- mats and the layout
MAT_L, MAT_W, MAT_T = 70.0 * MM, 70.0 * MM, 3.0 * MM
MAT_XY = {"right": (0.110, -0.130), "left": (0.110, 0.130)}
MAT_TOP = FLOOR + MAT_T
SLOT_OF = {"right": 0, "left": 1}
# Each bolt stands on its head in the middle of its mat, the head's length along the tangent of its slot's wide end
# (so the hand that takes it by the retainer's rim need not turn on the way).
HEAD_YAW = {arm: math.atan2(*reversed(tuple(slot_tangent(SLOT_OF[arm], (S_TAPER + S_FAR) / 2))))
            for arm in ("right", "left")}


def _bolt_start(arm):
    x, y = MAT_XY[arm]
    return np.array([x, y, MAT_TOP + 0.5 * MM]), HEAD_YAW[arm]


def make_mat(arm):
    x, y = MAT_XY[arm]
    return Part("mat_" + arm, (x, y, FLOOR + MAT_T / 2), [box((0, 0, 0), (MAT_L, MAT_W, MAT_T))], mass=None,
                color=RUBBER_BLACK, friction=(0.9, 0.8))


FLOOR_PART = make_floor()
FLANGE = make_flange()
BOLTS = {arm: make_bolt("bolt_" + arm, *_bolt_start(arm)) for arm in ("right", "left")}
OBJECTS = [FLOOR_PART, FLANGE, make_mat("right"), make_mat("left"), BOLTS["right"], BOLTS["left"]]
PRESSED_PARTS = ("flange",)  # the retainers are pushed down onto the ring


# ---------------------------------------------------------------- scoring
MARK_TOL = 3.0 * MM  # the shank's centre at the ring's top within this of the line's crossing with the slot
RET_SEATED = 2.0 * MM  # the retainer's underside within this of the ring's top (a 2 mm card stops)
UPRIGHT_DEG = 5.0
CLEAR = 0.030  # every gripper's TCP at least this far from the bolt's bounding box
STILL_MOVE = 1.0 * MM
SPEED = 0.01
MARK_XY = [slot_point(k, 0.0) for k in (0, 1)]


def bolt_terms(bolt):
    """Where a bolt is against the flange (metres, degrees)."""
    o, r = bolt_frame(bolt)
    axis = r[:, 2]
    tilt = math.degrees(math.acos(float(np.clip(axis[2], -1.0, 1.0))))
    head = o + r @ np.array([0.0, 0.0, HEAD_T / 2])
    d = head[:2] - FC
    rad = float(np.hypot(*d))
    th = math.atan2(d[1], d[0])
    slot, s_head = None, None
    for k in (0, 1):
        s = ((MARK_DEG[k] * DEG - th + math.pi) % TAU - math.pi) * R_SLOT
        if (S_CLOSED - 6 * MM <= s <= S_FAR + 2 * MM and abs(rad - R_SLOT) <= 9 * MM
                and SHELF - 1.5 * MM <= head[2] <= RING_UNDER + 0.5 * MM):
            slot, s_head = k, s
    at_top = None
    if axis[2] > 0.3:
        at_top = o + axis * ((RING_TOP - o[2]) / axis[2])
    mark_err = (float(np.linalg.norm(at_top[:2] - MARK_XY[slot])) if slot is not None and at_top is not None
                else math.inf)
    ret_bottom = o + axis * (RET_Z0 - RET_T / 2 - retainer_slid(bolt))
    ret_gap = float(ret_bottom[2] - RING_TOP)
    seated = bool(abs(ret_gap) <= RET_SEATED and axis[2] > 0.9)
    return {"slot": slot, "s": s_head, "rad": rad, "tilt": tilt, "mark_err": mark_err, "ret_gap": ret_gap,
            "seated": seated, "head": head, "tip": o + axis * BOLT_L}


def _box_clear(state, clear):
    hands = grippers()
    if hands is None:
        return True
    low, high = np.asarray(state.low, float), np.asarray(state.high, float)
    return all(float(np.linalg.norm(np.maximum(np.maximum(low - tcp, tcp - high), 0.0))) >= clear
               for tcp, _ in hands.values())


def _rot_deg(a, b):
    return math.degrees(math.acos(float(np.clip((np.trace(a.T @ b) - 1) / 2, -1.0, 1.0))))


_WINDOWS = {}


def still(bolt, key):
    """Over the last second the bolt moved under 1 mm and turned under 2 degrees (offline: its speed)."""
    t = _RUN["t"]
    if t is None:
        return bool(np.linalg.norm(bolt.lin_vel) < SPEED)
    w = _WINDOWS.setdefault(key, {"episode": None, "hist": deque()})
    hist = w["hist"]
    if w["episode"] != _RUN["episode"] or (hist and t < hist[-1][0]):
        hist.clear()
        w["episode"] = _RUN["episode"]
    now = (np.asarray(bolt.origin, float), bolt.rotation())
    if not hist or t > hist[-1][0] + 1e-9:
        hist.append((t, now))
    while len(hist) > 1 and hist[1][0] <= t - STILL_S + 1e-9:
        hist.popleft()
    if t - hist[0][0] < STILL_S - 0.051:
        return False
    return all(np.linalg.norm(now[0] - old[0]) <= STILL_MOVE and _rot_deg(now[1], old[1]) <= 2.0
               for _, old in hist)


def score_terms(objects):
    out = {}
    lost = False
    for arm in ("right", "left"):
        b = objects["bolt_" + arm]
        t = bolt_terms(b)
        t["released"] = _box_clear(b, CLEAR)
        t["still"] = still(b, arm)
        t["set"] = bool(t["slot"] is not None and t["mark_err"] <= MARK_TOL and t["seated"]
                        and t["tilt"] <= UPRIGHT_DEG and t["released"] and t["still"])
        lost = lost or float(np.asarray(b.origin, float)[2]) < TOP - 0.02
        out[arm] = t
    out["lost"] = ever("a bolt left the bench", lost)
    slots_in = {out[a]["slot"] for a in ("right", "left") if out[a]["slot"] is not None}
    slots_set = {out[a]["slot"] for a in ("right", "left") if out[a]["set"]}
    out["n_in"], out["n_set"] = len(slots_in), len(slots_set)
    if DIAG:
        _diag(objects, out)
    return out


def score(objects):
    t = score_terms(objects)
    if t["lost"] or t["n_in"] == 0:
        return 0.0
    if t["n_set"] == 2:
        return 1.0
    if t["n_set"] == 1:
        return 0.75 if t["n_in"] == 2 else 0.5
    return 0.25


def success(objects):
    return score(objects) >= 1.0


# ---------------------------------------------------------------- diagnostics (GPU copies only)
_DIAG = {"step": -1}


def _diag(objects, out):
    step = _RUN["step"]
    if _RUN["t"] is None or step == _DIAG["step"] or step % 5:
        return
    _DIAG["step"] = step
    hands = grippers() or {}
    tcp = " ".join(f"{a[0]}=({','.join(f'{v * 1000:.1f}' for v in hands[a][0])};{hands[a][1]:.2f})" for a in hands)
    parts = []
    for arm in ("right", "left"):
        t, b = out[arm], objects["bolt_" + arm]
        h = t["head"]
        parts.append(f"{arm[0]}:slot={t['slot']} s={(t['s'] or 0) * 1000:.1f} rad={t['rad'] * 1000:.1f} "
                     f"head=({h[0] * 1000:.1f},{h[1] * 1000:.1f},{(h[2] - FLOOR) * 1000:.2f}) tilt={t['tilt']:.1f} "
                     f"mark={min(t['mark_err'], 9.99) * 1000:.1f} ret={retainer_slid(b) * 1000:.1f} "
                     f"gap={t['ret_gap'] * 1000:.1f} set={int(t['set'])} rel={int(t['released'])} "
                     f"st={int(t['still'])} v={np.linalg.norm(b.lin_vel) * 1000:.0f}")
    print(f"BOLT_DIAG t={_RUN['t']:.2f} " + " | ".join(parts) + f" n_in={out['n_in']} n_set={out['n_set']} {tcp}",
          flush=True)


# ---------------------------------------------------------------- demo
# Each arm sets the bolt on its own side. Both pick their bolts at once; then the right arm sets its bolt while the
# left holds its bolt up out of the way, and the left goes on as the right parks (see demo()). The hand, pointing straight down with its fingers
# closing along the head's length, pinches the retainer by its rim, lifts the bolt (it hangs head-down), carries it
# over the slot's wide end, lowers the head through it onto the shelf, slides it round the slot to the line (the hand
# turns with the slot), lets go (the bolt stands on its head, held under the ring), and then, fingers 15 mm apart
# either side of the bolt's tip (across the slot), pushes the retainer down the thread with the fingertips until it
# bears on the ring (the grip on the thread is about 3.5 N), eases off and lifts clear. The demo reads the bolts' poses (scripted demos may read simulator state);
# it never moves an object.
PAD_TIP = 0.013  # the pad tips are 13 mm past the TCP
PINCH_DZ = 6.0 * MM  # the TCP this far above the retainer's centre when it is pinched (pads span -3..13 mm)
HANG = RET_Z0 + PINCH_DZ  # TCP over the head's outer face when the bolt hangs straight down from the pinch
OPEN_PICK = -0.020  # pads 40 mm apart round the 22 mm retainer
PINCH = -(RET_R - 1.25 * MM)  # pads aimed 19.5 mm apart: 2.5 mm inside the rim, about 6 N of squeeze
OPEN_REL = -0.014  # pads 28 mm apart
PRESS_OPEN = -0.0075  # pads 15 mm apart, either side of the 6.35 mm shank (across the slot, clear of the head)
PRESS_OUT = 0.0  # the hand this far outward of the bolt (radially) while pushing
PUSH_LEAD = 8.0 * MM  # the push aims the fingertips this far below the seated retainer's top (more and the palm
# would reach the bolt's tip; the ring stops the fingertips first)
PUSH_STEPS, PUSH_STEP_S = 8, 0.6
SLIDE_S = (23.0, 20.0, 16.0, 12.0, 8.0, 4.0, 0.0)  # mm along the slot, from the wide end's middle (29) to the line
SLIDE_STEP_S = 1.0
CARRY_Z = RING_TOP + 12.0 * MM  # head's outer face this high over the wide end before going in
HEAD_FLOAT = 0.3 * MM  # while it slides, the head's underside is aimed this far over the shelf (the channel leaves
# 1 mm over the 2.5 mm head): pressed on the shelf, the head drags, the bolt leans in the pinch and the head wedges
SET_DOWN = 0.5 * MM  # at the line, the head is aimed this far into the shelf before the fingers let go
READ_MAX = 0.015  # a read's correction is clamped to this (jitter is up to 10 mm)
PARK = {"left": np.array([0.140, 0.240, TOP + 0.200]), "right": np.array([0.140, -0.240, TOP + 0.200])}
FINAL_HOLD_S = 3.0
WAIT_AT = {"left": np.array([0.090, 0.200, FLOOR + 0.140]), "right": np.array([0.090, -0.200, FLOOR + 0.140])}


def _T(R, p):
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = R, p
    return m


def _axis_rot(u, a):
    u = np.asarray(u, float) / np.linalg.norm(u)
    k = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * (k @ k)


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def bolt_pose_start(arm):
    p, yaw = _bolt_start(arm)
    return _T(_rot_z(yaw), p - np.array([0.0, 0.0, 0.5 * MM]))  # standing on its head on the mat


def _grip_matrix(c):
    c = np.array([c[0], c[1], 0.0], float)
    c /= np.linalg.norm(c)
    a = np.array([0.0, 0.0, -1.0])
    return np.column_stack((c, np.cross(a, c), a))


class Grip(float):
    def __new__(cls, yaw, matrix=None):
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


def _state_now(name):
    env = _RUN["env"]
    if env is None:
        return None
    try:
        from yam_arena import scene as arena_scene
        return arena_scene.object_states(env, [name])[name]
    except Exception as e:  # noqa: BLE001
        print(f"BOLT_READ {name} failed: {e!r}", flush=True)
        return None


_ARMS = {}


def _arm_state(arm):
    """Per-arm, per-episode demo memory: the bolt's read start pose, the hand-in-bolt correction, cached poses."""
    s = _ARMS.get(arm)
    if s is None or s["episode"] != _RUN["episode"]:
        s = {"episode": _RUN["episode"], "B0": None, "corr": np.zeros(3), "yaw": 0.0, "press": None, "cache": {}}
        _ARMS[arm] = s
    return s


def _bolt_now(arm):
    st = _state_now("bolt_" + arm)
    if st is None:
        return None, None
    o, r = bolt_frame(st)
    return _T(r, o), retainer_slid(st)


def _clamp(v, m):
    n = float(np.linalg.norm(v))
    return v * (m / n) if n > m else v


def _read_start(arm, S):
    planned = bolt_pose_start(arm)
    actual, _ = _bolt_now(arm)
    if actual is None:
        S["B0"] = planned
        return
    shift = _clamp(actual[:3, 3] - planned[:3, 3], 0.025)
    S["B0"] = _T(actual[:3, :3], planned[:3, 3] + shift)
    print(f"BOLT_READ {arm} start t={_RUN['t']:.2f} shift_mm=({shift[0] * 1000:.1f},{shift[1] * 1000:.1f},"
          f"{shift[2] * 1000:.1f}) tilt={math.degrees(math.acos(min(1.0, abs(actual[2, 2])))):.1f}", flush=True)


def _read_hang(arm, S, prev_tcp, use_z, key, prev_c=None):
    """The hanging bolt against the hand: correct so its head goes where it is planned."""
    actual, _ = _bolt_now(arm)
    if actual is None or prev_tcp is None:
        return
    expect = prev_tcp - np.array([0.0, 0.0, HANG]) - S["corr"]  # where the head's outer face was planned
    err = expect - actual[:3, 3]
    if not use_z:
        err[2] = 0.0
    S["corr"] = _clamp(S["corr"] + err, READ_MAX)
    if use_z:  # the head's length against the hand's closing axis (the wrist sags in yaw under load)
        y = actual[:3, 0]  # the head's length
        c = _grip_matrix(_turn(prev_c, 0.0))[:, 0] if prev_c is not None else None
        if c is not None:
            e = math.atan2(float(c[0] * y[1] - c[1] * y[0]), float(c[0] * y[0] + c[1] * y[1]))
            e = (e + math.pi / 2) % math.pi - math.pi / 2  # the head's length either way along the axis
            S["yaw"] = float(np.clip(S["yaw"] - e, -12 * DEG, 12 * DEG))
    tilt = math.degrees(math.acos(float(np.clip(actual[2, 2], -1, 1))))
    print(f"BOLT_READ {arm} {key} t={_RUN['t']:.2f} err_mm=({err[0] * 1000:.2f},{err[1] * 1000:.2f},"
          f"{err[2] * 1000:.2f}) corr_mm=({S['corr'][0] * 1000:.1f},{S['corr'][1] * 1000:.1f},"
          f"{S['corr'][2] * 1000:.1f}) yaw={math.degrees(S['yaw']):.1f} tilt={tilt:.1f}", flush=True)


def _read_press(arm, S):
    actual, slid = _bolt_now(arm)
    if actual is None:
        k = SLOT_OF[arm]
        xy = slot_point(k, 0.0)
        S["press"] = np.array([xy[0], xy[1], SHELF + BOLT_L - RET_START_BELOW_TIP])
        return
    axis = actual[:3, 2]
    S["press"] = actual[:3, 3] + axis * (RET_Z0 + RET_T / 2 - slid)
    p = S["press"]
    print(f"BOLT_READ {arm} press t={_RUN['t']:.2f} ret_top=({p[0] * 1000:.1f},{p[1] * 1000:.1f},"
          f"{(p[2] - FLOOR) * 1000:.1f}) slid={slid * 1000:.1f} tilt="
          f"{math.degrees(math.acos(float(np.clip(axis[2], -1, 1)))):.1f}", flush=True)


def _turn(c, a):
    c = np.asarray(c, float)[:2]
    return np.array([c[0] * math.cos(a) - c[1] * math.sin(a), c[0] * math.sin(a) + c[1] * math.cos(a)])


def _hang_tcp(S, head_xyz):
    return np.asarray(head_xyz, float) + np.array([0.0, 0.0, HANG]) + S["corr"]


class BoltWaypoint(Waypoint):
    """One arm's waypoint worked out when it is first played: (TCP position, closing direction) from fn(S, prev)."""

    def __init__(self, arm, index, duration, finger, label, fn, keep=False, turn=False):
        self.arm, self.index, self.fn, self.turn = arm, index, fn, turn
        self.duration = duration * (1.0 if keep else DEMO_PACE)
        self.finger, self.label = finger, label

    def _pose(self):
        S = _arm_state(self.arm)
        if self.index not in S["cache"]:
            prev = S["cache"].get(self.index - 1)
            S["prev_c"] = None if prev is None else prev[1]
            pos, c = self.fn(S, None if prev is None else prev[0])
            c = np.asarray(c, float)
            if self.turn:
                c = _turn(c, S["yaw"])
            S["cache"][self.index] = (np.asarray(pos, float), c)
        return S["cache"][self.index]

    @property
    def pos(self):
        return tuple(float(v) for v in self._pose()[0])

    @property
    def yaw(self):
        c = self._pose()[1]  # the float is the closing axis's yaw (offline checks); the matrix is what is played
        return Grip(math.atan2(-float(c[0]), float(c[1])), _grip_matrix(c))


def _plan(arm, wait_s=0.0):
    """[(label, seconds, finger, fn, keep)] for one arm; wait_s: hold the lifted bolt this long before carrying it."""
    k = SLOT_OF[arm]
    c_wide = slot_tangent(k, S_WIDE_MID)

    def pick_c(S):
        y = S["B0"][:3, 0]  # the head's length
        c = np.array([y[0], y[1]])
        return c if c @ c_wide >= 0 else -c

    def disc(S, B=None):
        B = S["B0"] if B is None else B
        return (B @ np.array([0.0, 0.0, RET_Z0, 1.0]))[:3]

    def first(S, prev):
        _read_start(arm, S)
        return disc(S) + np.array([0.0, 0.0, PINCH_DZ + 0.040]), pick_c(S)

    plan = [("reach over the retainer", 2.0, OPEN_PICK, lambda S, p: (disc(S, bolt_pose_start(arm)) + np.array(
                [0.0, 0.0, PINCH_DZ + 0.060]), c_wide), False),
            ("over the retainer", 1.2, OPEN_PICK, first, False),
            ("fingers down beside the retainer", 1.5, OPEN_PICK,
             lambda S, p: (disc(S) + np.array([0.0, 0.0, PINCH_DZ]), pick_c(S)), False),
            ("pinch the retainer by its rim", 1.0, PINCH, lambda S, p: (p, pick_c(S)), True)]

    plan.append(("lift it off the mat: it hangs head-down", 1.5, PINCH,
                 lambda S, p: (np.array([p[0], p[1], FLOOR + 0.140]), pick_c(S)), False))
    if wait_s > 0:  # out of the other arm's way (its links pass within ~35 mm of a hand held over the mat)
        plan += [("move it out of the other arm's way", 1.5, PINCH, lambda S, p: (WAIT_AT[arm], pick_c(S)), False),
                 ("hold it there while the other arm works", wait_s - 1.5, PINCH, lambda S, p: (p, pick_c(S)), True)]
    wide = slot_point(k, S_WIDE_MID)
    plan += [
        ("carry it over the slot's wide end", 2.5, PINCH,
         lambda S, p: (_hang_tcp(S, (wide[0], wide[1], CARRY_Z)), c_wide), False),
        ("let it hang still", 1.0, PINCH, lambda S, p: (p, c_wide), False),
    ]

    def over(S, p):
        _read_hang(arm, S, p, True, "carry", S.get("prev_c"))
        return _hang_tcp(S, (wide[0], wide[1], CARRY_Z)), c_wide

    def just_over(S, p):
        return _hang_tcp(S, (wide[0], wide[1], RING_TOP + 3.0 * MM)), c_wide

    def into(S, p):
        _read_hang(arm, S, p, True, "over")  # hanging free just over the opening: height read too
        return _hang_tcp(S, (wide[0], wide[1], SHELF + HEAD_FLOAT)), c_wide

    plan += [("line it up over the wide end", 1.0, PINCH, over, False),
             ("head just over the opening", 1.2, PINCH, just_over, False),
             ("head down through the wide end", 1.2, PINCH, into, False),
             ("head just over the shelf", 0.5, PINCH, lambda S, p: (p, c_wide), False)]

    def slide_to(s_mm, read=None, dz=HEAD_FLOAT):
        def fn(S, p):
            if read:
                _read_hang(arm, S, p, False, read)  # where the head is along the slot: the hand leads it there
            xy = slot_point(k, s_mm * MM)
            return _hang_tcp(S, (xy[0], xy[1], SHELF + dz)), slot_tangent(k, s_mm * MM)
        return fn

    for s in SLIDE_S:
        plan.append((f"slide it round the slot ({s:.0f} mm to go)", SLIDE_STEP_S, PINCH,
                     slide_to(s, None if s == SLIDE_S[0] else f"slide{s:.0f}"), False))
    c_mark = slot_tangent(k, 0.0)
    c_out = (slot_point(k, 0.0) - FC) / R_SLOT  # pushing: fingers either side of the shank across the slot
    c_out = c_out if c_out[1] <= 0 else -c_out  # either way along it; this one keeps both wrists in range
    plan += [("hold at the line", 0.6, PINCH, lambda S, p: (p, c_mark), False),
             ("onto the line", 0.8, PINCH, slide_to(0.0, "mark"), False),
             ("set the head down on the shelf", 0.5, PINCH, slide_to(0.0, None, -SET_DOWN), False),
             ("let go of the retainer", 0.8, OPEN_REL, lambda S, p: (p, c_mark), True),
             ("fingers up off the retainer", 0.8, OPEN_REL, lambda S, p: (p + np.array([0.0, 0.0, 0.010]), c_out),
              False)]
    out = (slot_point(k, 0.0) - FC) / R_SLOT

    def press_tcp(S, dz):
        if S["press"] is None:
            _read_press(arm, S)
        q = S["press"]
        return np.array([q[0] + out[0] * PRESS_OUT, q[1] + out[1] * PRESS_OUT, q[2] + dz + PAD_TIP])

    if DEMO_WRONG == "noretainer":
        plan.append(("lift clear (no push)", 1.5, OPEN_REL,
                     lambda S, p: (np.array([p[0], p[1], FLOOR + 0.120]), c_out), False))
    else:
        seated_top = RING_TOP + RET_T
        plan += [("fingers either side of the bolt's tip", 1.0, PRESS_OPEN,
                  lambda S, p: (press_tcp(S, 0.008), c_out), False),
                 ("fingertips on the retainer", 1.0, PRESS_OPEN, lambda S, p: (press_tcp(S, 0.0005), c_out), False)]

        def push(i):
            def fn(S, p):
                top = S["press"][2]
                z = top + (seated_top - PUSH_LEAD - top) * (i + 1) / PUSH_STEPS
                q = press_tcp(S, 0.0)
                return np.array([q[0], q[1], z + PAD_TIP]), c_out
            return fn

        for i in range(PUSH_STEPS):
            plan.append((f"push the retainer down the thread ({i + 1})", PUSH_STEP_S, PRESS_OPEN, push(i), False))
        plan += [("hold it down on the ring", 0.6, PRESS_OPEN, lambda S, p: (p, c_out), False),
                 ("ease off", 0.6, PRESS_OPEN,
                  lambda S, p: (np.array([p[0], p[1], seated_top + 1.0 * MM + PAD_TIP]), c_out), False),
                 ("lift straight up", 1.5, PRESS_OPEN,
                  lambda S, p: (np.array([p[0], p[1], FLOOR + 0.120]), c_out), False)]
    plan += [("park", 2.0, OPEN_REL, lambda S, p: (PARK[arm], c_out), False),
             ("hold clear", FINAL_HOLD_S, OPEN_REL, lambda S, p: (PARK[arm], c_out), True)]
    return plan


def _waypoints(arm, wait_s=0.0):
    _ARMS.pop(arm, None)
    plan = _plan(arm, wait_s)
    labels = [p[0] for p in plan]
    t0, t1 = labels.index("line it up over the wide end"), labels.index("let go of the retainer")
    return [BoltWaypoint(arm, i, secs, finger, label, fn, keep, t0 <= i <= t1)
            for i, (label, secs, finger, fn, keep) in enumerate(plan)]


def demo():
    """Both arms pick their bolts at once; then one works at a time: the slots are 150 mm apart and, with both
    hands at them, the two arms' links touch (each pushes the other outward). The left arm holds its bolt up near its
    mat while the right arm sets its bolt, and carries it to its slot once the right arm starts to park."""
    right = _waypoints("right")
    labels = [w.label for w in right]
    park_at = sum(w.duration for w in right[:labels.index("park")])
    left_lifted = _waypoints("left")
    lift_end = sum(w.duration for w in left_lifted[:[w.label for w in left_lifted].index(
        "lift it off the mat: it hangs head-down") + 1])
    return {"right": right, "left": _waypoints("left", park_at - lift_end)}


# ---------------------------------------------------------------- offline test layouts (states.yaml)
SEATED_SLID = RET_Z0 - RET_T / 2 - (RING_TOP - SHELF)  # 50.6 mm: the retainer on the ring, the head on the shelf


def _quat(rot):
    from yam_arena.ik import matrix_to_quat_xyzw
    return [float(v) for v in matrix_to_quat_xyzw(np.asarray(rot, float))]


def bolt_in_slot(k, s_mm, tilt_deg=0.0, lift_mm=0.0):
    """[origin, quat] of a bolt standing in slot k, s_mm along it from the line, its head on the shelf (raised
    lift_mm), its head's length along the slot, leaning tilt_deg across the slot."""
    t = slot_tangent(k, s_mm * MM)
    x = np.array([t[0], t[1], 0.0])
    z = np.array([0.0, 0.0, 1.0])
    r = np.column_stack((x, np.cross(z, x), z))
    r = _axis_rot(x, tilt_deg * DEG) @ r
    xy = slot_point(k, s_mm * MM)
    o = np.array([xy[0], xy[1], SHELF + lift_mm * MM])
    return [float(v) for v in o], _quat(r)


def bolt_standing(arm, dx=0.0, dy=0.0, dz=0.0):
    """[origin, quat] of a bolt standing on its mat as at the start, shifted (metres)."""
    B = bolt_pose_start(arm)
    return [float(v) for v in B[:3, 3] + np.array([dx, dy, dz])], _quat(B[:3, :3])


def bolt_upright_at(x, y, z_bottom):
    """[origin, quat] of a bolt standing head-down with its head's outer face at height z_bottom at (x, y)."""
    return [float(x), float(y), float(z_bottom)], _quat(np.eye(3))
