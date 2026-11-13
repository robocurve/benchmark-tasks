"""Mount a turned shaft between bench centres for a runout check: hold the sprung tailstock centre back with its lever,
seat one centre hole on the fixed point, swing the shaft into line, then let the sprung centre into the other hole.

The bench centre is the compact cast-iron kind (a 350 x 110 mm bench, 75 mm centre height, 200 mm between centres): a
left tailstock with a fixed hardened 60-degree point and a right tailstock whose 60-degree point is on a sprung sleeve
(free travel about 10 mm). A hand lever with a black ball knob leaves the back of the right tailstock; pressing the
knob down draws the sleeve back (a finger on the lever bears on a lug on the sleeve, so the lever only pulls; the work
can still push the sleeve back on its own). The bench lies along y in front of the robot, the fixed point on the left.

The shaft is a ground C45 steel test shaft, 16 mm across and 160 mm long, both ends centre-drilled to DIN 332-1 form B
3.15 x 6.7 (a 60-degree seat inside a 120-degree protecting chamfer 10 mm across). It starts in two V-blocks in front
of the bench. Mounted, each point bears on its seat 5.9 mm inside an end face and the sprung sleeve sits 1.5 mm back
from full extension, pressing the shaft with about 4.6 N.

Metres; x forward from the arm bases, y left, z up; zero jitter for demos.
"""
import math
import pathlib
import sys
from collections import deque
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import ball, box, cyl  # noqa: E402
from yam_joints import Hinge, Link, Slide  # noqa: E402
from yam_mesh import Mesh, MeshMechanism, MeshPart  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
# Release-and-stillness block, verbatim from hang_holder_on_keyhole_studs.py (see release-stillness.md).
# Released means every gripper TCP is at least RELEASE_CLEAR from the object's box.
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
_WINDOW = deque()


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


def _box_distance(low, high, p):
    return float(np.linalg.norm(np.maximum(np.maximum(np.asarray(low) - p, p - np.asarray(high)), 0.0)))


def released(state, clear=RELEASE_CLEAR):
    hands = grippers()
    if hands is None:
        return True  # Offline arm poses are unknown, so release is assumed.
    return all(_box_distance(state.low, state.high, tcp) >= clear for tcp, _ in hands.values())


def _episode_memory():
    global _MEMORY_EPISODE
    if _MEMORY_EPISODE != _RUN["episode"]:
        _TIMERS.clear()
        _EVER.clear()
        _WINDOW.clear()
        _SLEEVE.clear()
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


TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S, SCORE_OVER = 1.0, "end"
RUN_LIMIT_STEPS = 2400  # 120 s at 20 Hz
DEMO_PACE = 1.0  # scales every demo duration (0.8 for the timing-margin copy)

CAST_WHITE, BENCH_GREY, BLACK = (0.86, 0.87, 0.85), (0.74, 0.75, 0.74), (0.07, 0.07, 0.08)
HARD_STEEL, SHAFT_STEEL, VBLOCK_GREY = (0.78, 0.79, 0.81), (0.62, 0.63, 0.66), (0.22, 0.23, 0.24)

# ---------------------------------------------------------------- the shaft: C45 test shaft, 16 x 160, DIN 332 B3.15
SHAFT_R, SHAFT_L = 0.008, 0.160
SHAFT_MASS = 0.252  # pi * 8^2 * 160 mm^3 * 7.85 g/cm^3
SHAFT_FRICTION = (0.20, 0.15)  # ground steel (estimate)
SHAFT_CONTACT_OFFSET = 0.0008
# DIN 332-1 form B 3.15 x 6.7: pilot 3.15, 60-degree seat to 6.7, 120-degree protecting chamfer to 10.0; t >= 6.8.
HOLE_D1, HOLE_D2, HOLE_D3, HOLE_T = 0.00315, 0.0067, 0.0100, 0.0070
CHAMFER_DEPTH = (HOLE_D3 - HOLE_D2) / 2 / math.tan(math.radians(60.0))  # 0.95 mm
SEAT_DEPTH = CHAMFER_DEPTH + (HOLE_D2 - HOLE_D1) / 2 / math.tan(math.radians(30.0))  # 4.03 mm: the seat's inner edge
SEAT_APEX_DEPTH = CHAMFER_DEPTH + HOLE_D2 / 2 / math.tan(math.radians(30.0))  # 6.76 mm: the seat cone's apex
N_SECTORS = 16  # each end is 16 convex sectors around the hole


def _hole_profile():
    """(radius, depth) of the centre hole's wall, from the end face inward, then the outside of the end piece."""
    return [(HOLE_D3 / 2, 0.0), (HOLE_D2 / 2, CHAMFER_DEPTH), (HOLE_D1 / 2, SEAT_DEPTH), (HOLE_D1 / 2, HOLE_T),
            (SHAFT_R, HOLE_T), (SHAFT_R, 0.0)]


def _prism_mesh(name, ring0, ring1, color):
    """Closed convex mesh between two matching convex polygons (lists of 3-D points in the same order)."""
    n = len(ring0)
    pts = np.array(list(ring0) + list(ring1), float)
    faces = []
    for i in range(1, n - 1):
        faces.append((0, i + 1, i))
        faces.append((n, n + i, n + i + 1))
    for i in range(n):
        j = (i + 1) % n
        faces += [(i, j, n + j), (i, n + j, n + i)]
    faces = np.array(faces, int)
    p = pts[faces]
    vol = float(np.einsum("ij,ij->i", p[:, 0], np.cross(p[:, 1], p[:, 2])).sum()) / 6.0
    if vol < 0:
        faces = faces[:, ::-1]
    return Mesh(name, pts, faces, collision="convex", color=color)


def _end_sectors(end):
    """The shaft's `end` (+1 left, -1 right; local +y / -y) as convex sectors around its centre hole."""
    half = math.pi / N_SECTORS
    out = []
    for k in range(N_SECTORS):
        rings = []
        for a in (2 * k * half - half, 2 * k * half + half):
            ring = []
            for r, depth in _hole_profile():
                rr = r / math.cos(half) if r == SHAFT_R else r  # outer facets circumscribe the bar; hole facets inscribe
                ring.append((rr * math.cos(a), end * (SHAFT_L / 2 - depth), rr * math.sin(a)))
            rings.append(ring)
        out.append(_prism_mesh(f"{'left' if end > 0 else 'right'}_end_{k:02d}", rings[0], rings[1], SHAFT_STEEL))
    return out


SHAFT_SHAPES = [cyl((0.0, 0.0, 0.0), SHAFT_R, SHAFT_L - 2 * HOLE_T, axis="y")]  # the solid body between the end pieces

# ---------------------------------------------------------------- the bench centre (fixed)
BX = 0.180  # the centre line, x
ZC = TOP + 0.115  # the centre line, z: 40 mm bench + 75 mm centre height
BENCH_TOP = TOP + 0.040
BENCH_Y = (-0.135, 0.215)  # 350 mm bench along y, 110 mm wide (x 0.125 .. 0.235)
BENCH_W = 0.110
# Mounted shaft: middle at y 0.052, so its left end face is at y 0.132 and its right end face at y -0.028.
MOUNT_MID_Y = 0.052
LEFT_FACE_Y, RIGHT_FACE_Y = MOUNT_MID_Y + SHAFT_L / 2, MOUNT_MID_Y - SHAFT_L / 2
# Points: 60-degree cones 14 mm across at the base, tip relieved to 1 mm across (it clears the pilot hole's bottom).
POINT_R, POINT_TIP_R = 0.007, 0.0005
POINT_CONE_L = (POINT_R - POINT_TIP_R) / math.tan(math.radians(30.0))  # 11.26 mm
TIP_DEPTH = SEAT_APEX_DEPTH - POINT_TIP_R / math.tan(math.radians(30.0))  # 5.89 mm: a seated tip inside its end face
LEFT_TIP_Y = LEFT_FACE_Y - TIP_DEPTH
# The sprung sleeve: q = 0 at full extension, negative drawn back; seated it sits SEAT_BACK behind full extension.
SLEEVE_TRAVEL = 0.0105  # stop; the lever draws it 10.0 mm
SEAT_BACK = 0.0015
RIGHT_TIP_Y0 = RIGHT_FACE_Y + TIP_DEPTH + SEAT_BACK  # the sprung point's tip at full extension
SLEEVE_PROTRUDES = 0.025  # tip to the tailstock's front face at full extension
SLEEVE_K, SLEEVE_REST, SLEEVE_DAMP, SLEEVE_MASS = 400.0, 0.010, 4.0, 0.08  # 4.0 N at full extension, 0.4 N/mm
# Tailstock castings: head 64 long (y) x 46 wide (x), z from ZC-42 to ZC+28; foot 60 wide down to the bench.
HEAD_L, HEAD_HALF_W, HEAD_Z0, HEAD_Z1 = 0.064, 0.023, ZC - 0.042, ZC + 0.028
FOOT_HALF_W = 0.030
PRISM_Z1 = HEAD_Z1 + 0.010  # the V-prism on top of each head (for work without centres)
LEFT_HEAD_Y = (LEFT_TIP_Y + POINT_CONE_L + 0.008, LEFT_TIP_Y + POINT_CONE_L + 0.008 + HEAD_L)
RIGHT_HEAD_Y = (RIGHT_TIP_Y0 - SLEEVE_PROTRUDES - HEAD_L, RIGHT_TIP_Y0 - SLEEVE_PROTRUDES)
# Inside the right head: a cavity for the sleeve's back end, its lug and the lever's finger, open to the lever's slot.
CAV_X = (BX - 0.021, BX + 0.010)
CAV_Z1 = ZC + 0.011
WALL = 0.006  # front wall
BACK_WALL = 0.005
BORE_HALF = 0.009  # the sleeve (14 mm) runs in an 18 mm opening (no contact: the slide joint guides it)
SLOT_X, SLOT_Z1 = (BX - 0.020, BX - 0.008), ZC - 0.014
# Sleeve lug (on the robot's side of the sleeve's back end) and the lever.
LEVER_X = BX - 0.014
LUG_X = (BX - 0.019, BX - 0.006)
LUG_HALF_Z = 0.006
LUG_LEN = 0.006
PIVOT_Y, PIVOT_Z = -0.081, ZC - 0.022  # lever pivot (pin across x, in the head's near wall)
FINGER_R = 0.003  # the finger's ball end; its stem is thinner and stops inside the ball, so only the ball meets the lug
STEM_R, STEM_TOP = 0.0025, ZC - 0.0015
FOOT_INSET = 0.010
FINGER_H = ZC - PIVOT_Z  # the finger's ball end sits on the centre line at rest
FINGER_GAP = 0.0005  # rest gap between the finger and the lug
LUG_FRONT_Y = PIVOT_Y - FINGER_R - FINGER_GAP
SLEEVE_BACK_Y = LUG_FRONT_Y - LUG_LEN
KNOB_R = 0.010  # 20 mm black ball knob
KNOB_Y, KNOB_Z = RIGHT_HEAD_Y[0] - 0.045, ZC - 0.012  # 45 mm behind the head's back face, 12 mm below the line
ARM_R, ARM_BEND_Y = 0.004, -0.128
LEVER_K, LEVER_REST_DEG, LEVER_DAMP, LEVER_MASS = 0.04, -45.0, 0.002, 0.040  # return spring 0.031 N m at rest
LEVER_MAX = math.asin((0.010 + FINGER_GAP) / FINGER_H)  # the lever's stop: the finger has drawn the sleeve 10.0 mm
BENCH_FRICTION = (0.15, 0.12)  # hardened steel points (estimate)
BENCH_CONTACT_OFFSET = 0.0005


def _b(x0, x1, y0, y1, z0, z1, color=None):
    return box(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color=color)


def _frustum_y(name, y0, r0, y1, r1, x, z, color, n=24):
    """A convex cone frustum along y (circle r0 at y0, circle r1 at y1)."""
    a = np.arange(n) * 2 * math.pi / n
    ring0 = [(x + r0 * math.cos(t), y0, z + r0 * math.sin(t)) for t in a]
    ring1 = [(x + r1 * math.cos(t), y1, z + r1 * math.sin(t)) for t in a]
    return _prism_mesh(name, ring0, ring1, color)


def _rod(name, p0, p1, r, color, n=12):
    """A straight round rod from p0 to p1 as one convex mesh."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = (p1 - p0) / np.linalg.norm(p1 - p0)
    u = np.cross(d, [1.0, 0.0, 0.0])
    if np.linalg.norm(u) < 1e-6:
        u = np.cross(d, [0.0, 1.0, 0.0])
    u /= np.linalg.norm(u)
    v = np.cross(d, u)
    a = np.arange(n) * 2 * math.pi / n
    ring = [r * (math.cos(t) * u + math.sin(t) * v) for t in a]
    return _prism_mesh(name, [p0 + o for o in ring], [p1 + o for o in ring], color)


def _casting(y0, y1, foot_inset=0.0, color=CAST_WHITE):
    """A tailstock's foot, V-prism on top and eccentric clamp lever (the head itself is added separately). The right
    tailstock's foot is set in at the back under the head, where the lever comes out."""
    ym = (y0 + y1) / 2
    return [
        _b(BX - FOOT_HALF_W, BX + FOOT_HALF_W, y0 + foot_inset, y1, BENCH_TOP, HEAD_Z0, color),
        _b(BX - 0.020, BX - 0.004, ym - 0.010, ym + 0.010, HEAD_Z1, PRISM_Z1, HARD_STEEL),
        _b(BX + 0.004, BX + 0.020, ym - 0.010, ym + 0.010, HEAD_Z1, PRISM_Z1, HARD_STEEL),
        cyl((BX - FOOT_HALF_W - 0.005, ym, BENCH_TOP + 0.012), 0.005, 0.010, axis="x", color=BLACK),
        cyl((BX - FOOT_HALF_W - 0.013, ym, BENCH_TOP + 0.027), 0.0045, 0.040, color=BLACK),
    ]


def _bench_base():
    x0, x1 = BX - HEAD_HALF_W, BX + HEAD_HALF_W
    ly0, ly1 = LEFT_HEAD_Y
    ry0, ry1 = RIGHT_HEAD_Y
    shapes = [_b(BX - BENCH_W / 2, BX + BENCH_W / 2, BENCH_Y[0], BENCH_Y[1], TOP, BENCH_TOP, BENCH_GREY)]
    # left tailstock: solid head, fixed point (shank; the cone is a mesh)
    shapes += _casting(ly0, ly1)
    shapes.append(_b(x0, x1, ly0, ly1, HEAD_Z0, HEAD_Z1, CAST_WHITE))
    shapes.append(cyl((BX, (LEFT_TIP_Y + POINT_CONE_L + ly0) / 2, ZC), POINT_R, ly0 - LEFT_TIP_Y - POINT_CONE_L,
                      axis="y", color=HARD_STEEL))
    # right tailstock: hollow head around the sleeve's back end and the lever's finger
    cx0, cx1 = CAV_X
    shapes += _casting(ry0, ry1, foot_inset=FOOT_INSET)
    shapes += [
        _b(x0, x1, ry0, ry1, CAV_Z1, HEAD_Z1, CAST_WHITE),  # top
        _b(x0, cx0, ry0, ry1, HEAD_Z0, CAV_Z1, CAST_WHITE),  # near side wall (lever pivot pin)
        _b(cx1, x1, ry0, ry1, HEAD_Z0, CAV_Z1, CAST_WHITE),  # far side wall
        # front wall with the sleeve's opening
        _b(cx0, cx1, ry1 - WALL, ry1, HEAD_Z0, ZC - BORE_HALF, CAST_WHITE),
        _b(cx0, cx1, ry1 - WALL, ry1, ZC + BORE_HALF, CAV_Z1, CAST_WHITE),
        _b(cx0, BX - BORE_HALF, ry1 - WALL, ry1, ZC - BORE_HALF, ZC + BORE_HALF, CAST_WHITE),
        _b(BX + BORE_HALF, cx1, ry1 - WALL, ry1, ZC - BORE_HALF, ZC + BORE_HALF, CAST_WHITE),
        # back wall with the lever's slot
        _b(SLOT_X[1], cx1, ry0, ry0 + BACK_WALL, HEAD_Z0, CAV_Z1, CAST_WHITE),
        _b(SLOT_X[0], SLOT_X[1], ry0, ry0 + BACK_WALL, SLOT_Z1, CAV_Z1, CAST_WHITE),
        # knurled cap on the back of the head, on the centre line
        cyl((BX, ry0 - 0.005, ZC), 0.0075, 0.010, axis="y", color=BLACK),
    ]
    return shapes


SLEEVE = Link(
    "sleeve",
    [cyl((BX, (RIGHT_TIP_Y0 - POINT_CONE_L + SLEEVE_BACK_Y) / 2, ZC), POINT_R,
         RIGHT_TIP_Y0 - POINT_CONE_L - SLEEVE_BACK_Y, axis="y", color=HARD_STEEL),
     _b(LUG_X[0], LUG_X[1], SLEEVE_BACK_Y, LUG_FRONT_Y, ZC - LUG_HALF_Z, ZC + LUG_HALF_Z, HARD_STEEL)],
    Slide("y", (BX, RIGHT_TIP_Y0, ZC), -SLEEVE_TRAVEL, 0.0, damping=SLEEVE_DAMP, spring=SLEEVE_K,
          rest=SLEEVE_REST), SLEEVE_MASS, color=HARD_STEEL)
LEVER = Link(
    "lever",
    [cyl((LEVER_X, PIVOT_Y, PIVOT_Z), 0.005, 0.010, axis="x", color=BLACK),  # hub on the pivot pin
     cyl((LEVER_X, PIVOT_Y, (PIVOT_Z + STEM_TOP) / 2), STEM_R, STEM_TOP - PIVOT_Z, color=BLACK),  # finger stem
     ball((LEVER_X, PIVOT_Y, ZC), FINGER_R, color=BLACK),
     cyl((LEVER_X, (PIVOT_Y + ARM_BEND_Y) / 2, PIVOT_Z), ARM_R, PIVOT_Y - ARM_BEND_Y, axis="y", color=BLACK),
     ball((LEVER_X, KNOB_Y, KNOB_Z), KNOB_R, color=BLACK)],
    Hinge("x", (LEVER_X, PIVOT_Y, PIVOT_Z), 0.0, math.degrees(LEVER_MAX), damping=LEVER_DAMP, spring=LEVER_K,
          rest=LEVER_REST_DEG), LEVER_MASS, color=BLACK)
BENCH = MeshMechanism(
    "bench", (0.0, 0.0, 0.0), _bench_base(), [SLEEVE, LEVER], color=CAST_WHITE, friction=BENCH_FRICTION,
    contact_offset=BENCH_CONTACT_OFFSET, self_collide=True,
    meshes={"base": (_frustum_y("fixed_point", LEFT_TIP_Y, POINT_TIP_R, LEFT_TIP_Y + POINT_CONE_L, POINT_R, BX, ZC,
                                HARD_STEEL),),
            "sleeve": (_frustum_y("sprung_point", RIGHT_TIP_Y0, POINT_TIP_R, RIGHT_TIP_Y0 - POINT_CONE_L, POINT_R,
                                  BX, ZC, HARD_STEEL),),
            "lever": (_rod("lever_arm", (LEVER_X, ARM_BEND_Y, PIVOT_Z), (LEVER_X, KNOB_Y, KNOB_Z), ARM_R, BLACK),)})

# ---------------------------------------------------------------- V-blocks and the shaft's start
VB_X, VB_W, VB_L, VB_H, VB_MOUTH = 0.080, 0.050, 0.040, 0.040, 0.040  # 90-degree V, 40 mm across, 20 mm deep
VB_APEX_Z = VB_H - VB_MOUTH / 2
VB_Y = (MOUNT_MID_Y - 0.050, MOUNT_MID_Y + 0.050)
START_AXIS_Z = TOP + VB_APEX_Z + SHAFT_R * math.sqrt(2.0)  # 31.3 mm up: the bar on both flanks of the V


def _vblock_meshes():
    out = []
    for i, yc in enumerate(VB_Y):
        y0, y1 = yc - VB_L / 2, yc + VB_L / 2
        for side in (-1, 1):
            poly = [(side * VB_W / 2, VB_APEX_Z), (0.0, VB_APEX_Z), (side * VB_MOUTH / 2, VB_H), (side * VB_W / 2, VB_H)]
            r0 = [(VB_X + px, y0, TOP + pz) for px, pz in poly]
            r1 = [(VB_X + px, y1, TOP + pz) for px, pz in poly]
            out.append(_prism_mesh(f"vblock{i}_{'near' if side < 0 else 'far'}", r0, r1, VBLOCK_GREY))
    return out


VBLOCKS = MeshPart(
    "vblocks", (0.0, 0.0, 0.0),
    shapes=[_b(VB_X - VB_W / 2, VB_X + VB_W / 2, yc - VB_L / 2, yc + VB_L / 2, TOP, TOP + VB_APEX_Z, VBLOCK_GREY)
            for yc in VB_Y],
    mass=None, color=VBLOCK_GREY, friction=(0.20, 0.15), contact_offset=0.001, meshes=_vblock_meshes())

SHAFT_START = (VB_X, MOUNT_MID_Y, START_AXIS_Z)
SHAFT = MeshPart("shaft", SHAFT_START, shapes=SHAFT_SHAPES, mass=SHAFT_MASS, color=SHAFT_STEEL,
                 friction=SHAFT_FRICTION, contact_offset=SHAFT_CONTACT_OFFSET, grip=0,
                 meshes=_end_sectors(+1) + _end_sectors(-1))
OBJECTS = [BENCH, VBLOCKS, SHAFT]
PRESSED_PARTS = ("bench",)  # the right hand presses the lever's knob; replay cannot turn the lever


# ---------------------------------------------------------------- geometry the scorer and the demo share
def lever_draw(a):
    """How far (m) the lever at angle a (rad) holds the sleeve back from full extension."""
    return max(0.0, FINGER_H * math.sin(a) - FINGER_GAP)


def knob_centre(a):
    """The knob's centre (world) at lever angle a (rad): a turn about the pivot's x axis."""
    dy, dz = KNOB_Y - PIVOT_Y, KNOB_Z - PIVOT_Z
    c, s = math.cos(a), math.sin(a)
    return np.array([LEVER_X, PIVOT_Y + dy * c - dz * s, PIVOT_Z + dy * s + dz * c])


def knob_drop(a):
    return float(knob_centre(0.0)[2] - knob_centre(a)[2])


def left_tip():
    return np.array([BX, LEFT_TIP_Y, ZC])


def right_tip(q_sleeve):
    return np.array([BX, RIGHT_TIP_Y0 + q_sleeve, ZC])


def shaft_axis(shaft):
    """(centre, unit axis toward the shaft's local +y end) of a shaft state."""
    R = np.asarray(shaft.rotation(), float)
    return np.asarray(shaft.origin, float), R[:, 1]


def tip_in_end(shaft, tip):
    """For each end (+1, -1): (radial offset of `tip` from the shaft's axis, depth of `tip` inside that end face)."""
    c, u = shaft_axis(shaft)
    rel = np.asarray(tip, float) - c
    along = float(rel @ u)
    radial = float(np.linalg.norm(rel - along * u))
    return {+1: (radial, SHAFT_L / 2 - along), -1: (radial, SHAFT_L / 2 + along)}


SEAT_RADIAL = 0.0010  # in its seat: the tip within 1 mm of the hole's axis (on the chamfer it is at least 3.3 mm off)
SEAT_DEPTH_MIN, SEAT_DEPTH_MAX = 0.0050, 0.0068  # ... and 5.0-6.8 mm inside the end face (seated: 5.9)
BEAR_RADIAL, BEAR_DEPTH = SHAFT_R, -0.0005  # bearing on the end: inside the face's circle, touching it or deeper


def point_state(shaft, tip):
    """'seated' (in a seat; the end it is in), 'bearing' (on a chamfer or an end face) or None."""
    for end, (radial, depth) in tip_in_end(shaft, tip).items():
        if radial <= SEAT_RADIAL and SEAT_DEPTH_MIN <= depth <= SEAT_DEPTH_MAX:
            return "seated", end
    for end, (radial, depth) in tip_in_end(shaft, tip).items():
        if radial <= BEAR_RADIAL and BEAR_DEPTH <= depth < SEAT_DEPTH_MIN:
            return "bearing", end
    return None, None


def shaft_bounds(shaft):
    """The shaft's world box from its true shape (the offline state's box covers only its middle body)."""
    c, u = shaft_axis(shaft)
    ends = np.array([c + u * SHAFT_L / 2, c - u * SHAFT_L / 2])
    ext = SHAFT_R * np.sqrt(np.clip(1.0 - u ** 2, 0.0, 1.0))
    return SimpleNamespace(low=ends.min(0) - ext, high=ends.max(0) + ext)


def shaft_lowest_z(shaft):
    c, u = shaft_axis(shaft)
    return float(c[2] - abs(u[2]) * SHAFT_L / 2 - SHAFT_R * math.sqrt(max(0.0, 1.0 - u[2] ** 2)))


TABLE_X, TABLE_Y = (-0.19, 0.91), (-0.65, 0.65)


def dropped_now(shaft):
    """The shaft lying on the table or on the bench's top, or off the table."""
    c, _ = shaft_axis(shaft)
    low = shaft_lowest_z(shaft)
    on_bench = (BX - BENCH_W / 2 <= c[0] <= BX + BENCH_W / 2 and BENCH_Y[0] <= c[1] <= BENCH_Y[1]
                and low <= BENCH_TOP + 0.002)
    return bool(low <= TOP + 0.002 or on_bench or not TABLE_X[0] <= c[0] <= TABLE_X[1]
                or not TABLE_Y[0] <= c[1] <= TABLE_Y[1])


FORCE_PUSH = 0.003  # forcing: the work pushes the sleeve back more than 3 mm while the lever is not holding it
_SLEEVE = {}


def pushed_back(q_sleeve, a):
    """How far (m) the work has pushed the sleeve back since the lever last held it: the sleeve's most forward
    position since then, minus where it is now. A point stopped short on a chamfer has not been pushed back; a shaft
    rammed against the point has. Offline each state stands alone (0)."""
    if _RUN["t"] is None:
        return 0.0
    _episode_memory()
    holding = lever_draw(a) >= -q_sleeve - 0.0005  # the finger is on the lug: the lever sets the sleeve
    front = q_sleeve if holding or "front" not in _SLEEVE else max(_SLEEVE["front"], q_sleeve)
    _SLEEVE["front"] = front
    return front - q_sleeve
LEVER_REST_DROP = 0.002  # lever at rest: knob within 2 mm of its rest height
STILL_MOVE, STILL_TURN_DEG = 0.0005, 0.5  # still: over 1 s the shaft moves < 0.5 mm and its axis turns < 0.5 deg
STILL_SPEED, STILL_RATE = 0.005, 0.05  # offline (one state): speeds below 5 mm/s and 0.05 rad/s (spin ignored)


def _still(shaft):
    c, u = shaft_axis(shaft)
    t = _RUN["t"]
    if t is None:
        w = np.asarray(shaft.ang_vel, float)
        w_perp = w - (w @ u) * u  # spin about its own axis is free between centres and does not count
        return bool(np.linalg.norm(shaft.lin_vel) < STILL_SPEED and np.linalg.norm(w_perp) < STILL_RATE)
    _episode_memory()
    _WINDOW.append((t, c, u))
    while _WINDOW and _WINDOW[0][0] < t - STILL_S - 1e-9:
        _WINDOW.popleft()
    if t - _WINDOW[0][0] < STILL_S - 0.06:
        return False
    return all(np.linalg.norm(cc - c) < STILL_MOVE
               and math.degrees(math.acos(float(np.clip(abs(uu @ u), -1.0, 1.0)))) < STILL_TURN_DEG
               for _, cc, uu in _WINDOW)


def score_terms(objects):
    shaft, bench = objects["shaft"], objects["bench"]
    q_sleeve, a = float(bench.joints["sleeve"]), float(bench.joints["lever"])
    left, left_end = point_state(shaft, left_tip())
    right, right_end = point_state(shaft, right_tip(q_sleeve))
    excess = pushed_back(q_sleeve, a)
    forced = ever("sleeve pushed back by the work, not drawn by the lever", excess > FORCE_PUSH)
    dropped = ever("shaft dropped onto the table or the bench", dropped_now(shaft))
    still = _still(shaft)
    lever_rest = knob_drop(a) <= LEVER_REST_DROP
    hands_off = released(shaft_bounds(shaft)) and released(
        SimpleNamespace(low=knob_centre(a) - KNOB_R, high=knob_centre(a) + KNOB_R))
    both = left == "seated" and right == "seated" and left_end != right_end
    one_bearing = ((left == "seated" and right == "bearing") or (right == "seated" and left == "bearing")) \
        and left_end != right_end
    # the last second: held in place with hands off for 1 s, and still over that same second (the window)
    done = held_for("both seated, lever at rest, hands off", both and lever_rest and hands_off)
    return {"left": left, "right": right, "q_sleeve": q_sleeve, "lever_deg": math.degrees(a), "excess": excess,
            "forced": forced, "dropped": dropped, "still": still, "lever_rest": lever_rest, "released": hands_off,
            "both": both, "one_bearing": one_bearing, "done_s": done}


def score(objects):
    t = score_terms(objects)
    if t["dropped"]:
        return 0.0
    if t["both"] and t["lever_rest"] and t["released"] and t["still"] and t["done_s"] + 1e-10 >= STILL_S:
        value = 1.0
    elif t["both"] and t["still"]:
        value = 0.75
    elif t["one_bearing"] and t["released"] and t["still"]:
        value = 0.5
    elif t["left"] == "seated" or t["right"] == "seated":
        value = 0.25
    else:
        value = 0.0
    return min(value, 0.5) if t["forced"] else value


def success(objects):
    return score(objects) >= 1.0


# ---------------------------------------------------------------- demo: open loop from the known start layout
GRIP_DZ = 0.003  # the TCP 3 mm above the bar's axis: pad tips 10 mm below it
# A gripped part rides 3-4 mm low in the hand, so carried poses are commanded higher. 4.5 mm (1 mm over the expected
# sag) centres the tolerance: in a convex-hull contact stand-in the sprung point still finds its seat with the hand
# 3 mm off in any direction (the free end is then 6 mm off: the hand holds the middle, the pivot is the far end).
CARRY_DZ = 0.0045
PAD_OPEN = -0.020  # fingers 40 mm apart for the bar (the V-blocks and the bench stay clear of the fingers)
SWING_DEG = -12.0  # the shaft starts its swing turned 12 deg about the fixed point, its free end toward the robot
STANDOFF = 0.012  # the left centre hole starts 12 mm short of its seat, on the point's line
PUSH = 0.002  # held 2 mm into the fixed point while swinging, so the arm's give keeps the hole on its seat
LIFT_Z = TOP + 0.170
PARK_L, PARK_R = (0.180, 0.250, TOP + 0.180), (0.180, -0.280, TOP + 0.180)  # behind the tailstocks, above their tops
PRESS_OVER = 0.012  # the closed fingertips are commanded 12 mm past the knob once the lever is on its stop
RIGHT_YAW = math.pi / 2  # fingers close across the bench, so the palm's long side stays clear of the head


def _rz(psi):
    c, s = math.cos(psi), math.sin(psi)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def shaft_mid_for(psi_deg, standoff):
    """The shaft's middle when its left hole lies on the fixed point's line turned psi about z through the tip,
    `standoff` short of its seat."""
    u = _rz(math.radians(psi_deg)) @ np.array([0.0, 1.0, 0.0])  # toward the shaft's left end
    return left_tip() - u * (SHAFT_L / 2 - TIP_DEPTH + standoff)


def left_tcp(mid, psi_deg, carried=True):
    return tuple(float(v) for v in mid + np.array([0.0, 0.0, GRIP_DZ + (CARRY_DZ if carried else 0.0)]))


def left_yaw(psi_deg):
    return -math.pi / 2 + math.radians(psi_deg)


def knob_tcp(a, over=0.0):
    p = knob_centre(a) + np.array([0.0, 0.0, KNOB_R + 0.013 - over])
    return tuple(float(v) for v in p)


def demo_plan():
    """{'left': [...], 'right': [...]} of (label, seconds, TCP, finger, yaw) at pace 1.0."""
    L, R = [], []
    x0, y0, z0 = SHAFT_START
    grip = (x0, y0, z0 + GRIP_DZ)
    L += [("open hand high over the shaft", 1.6, (x0, y0, TOP + 0.180), PAD_OPEN, -math.pi / 2),
          ("pads down round the shaft", 1.6, grip, PAD_OPEN, -math.pi / 2),
          ("close on the shaft", 0.8, grip, FINGER_CLOSED, -math.pi / 2),
          ("settle the grip", 0.3, grip, FINGER_CLOSED, -math.pi / 2),
          ("lift the shaft out of the V-blocks", 1.2, (x0, y0, LIFT_Z), FINGER_CLOSED, -math.pi / 2)]
    above = shaft_mid_for(SWING_DEG, STANDOFF) + np.array([0.0, 0.0, 0.030])
    L += [("carry it over the bench, turned", 2.0, left_tcp(above, SWING_DEG), FINGER_CLOSED, left_yaw(SWING_DEG)),
          ("lower it to the centre line", 1.2, left_tcp(shaft_mid_for(SWING_DEG, STANDOFF), SWING_DEG),
           FINGER_CLOSED, left_yaw(SWING_DEG)),
          ("slide the left centre hole onto the fixed point", 1.4,
           left_tcp(shaft_mid_for(SWING_DEG, -PUSH), SWING_DEG), FINGER_CLOSED, left_yaw(SWING_DEG))]
    steps = int(round(abs(SWING_DEG)))
    for k in range(1, steps + 1):
        psi = SWING_DEG * (1.0 - k / steps)
        L.append(("swing the free end into line", 0.30, left_tcp(shaft_mid_for(psi, -PUSH), psi), FINGER_CLOSED,
                  left_yaw(psi)))
    L.append(("hold it in line", 0.6, left_tcp(shaft_mid_for(0.0, -PUSH), 0.0), FINGER_CLOSED, left_yaw(0.0)))
    t_in_line = sum(w[1] for w in L)

    R += [("closed fingers over the knob", 2.0, knob_tcp(0.0, -0.030), FINGER_CLOSED, RIGHT_YAW),
          ("fingertips onto the knob", 0.8, knob_tcp(0.0, -0.001), FINGER_CLOSED, RIGHT_YAW)]
    n = 6
    for k in range(1, n + 1):
        R.append(("press the lever down", 0.25, knob_tcp(LEVER_MAX * k / n), FINGER_CLOSED, RIGHT_YAW))
    R.append(("hold the lever on its stop", 0.5, knob_tcp(LEVER_MAX, PRESS_OVER), FINGER_CLOSED, RIGHT_YAW))
    t_pressed = sum(w[1] for w in R)
    R.append(("keep the lever down", max(0.5, t_in_line - t_pressed), knob_tcp(LEVER_MAX, PRESS_OVER),
              FINGER_CLOSED, RIGHT_YAW))
    R.append(("take up the knob", 0.4, knob_tcp(LEVER_MAX, 0.0), FINGER_CLOSED, RIGHT_YAW))
    n = 10
    for k in range(1, n + 1):
        R.append(("ease the lever up", 0.35, knob_tcp(LEVER_MAX * (1.0 - k / n), 0.0), FINGER_CLOSED, RIGHT_YAW))
    R += [("lift off the knob", 0.8, knob_tcp(0.0, -0.040), FINGER_CLOSED, RIGHT_YAW)]
    t_eased = sum(w[1] for w in R)
    R += [("park", 1.6, PARK_R, FINGER_OPEN, 0.0)]

    L.append(("hold while the sprung centre comes in", max(0.5, t_eased - sum(w[1] for w in L) + 0.8),
              left_tcp(shaft_mid_for(0.0, -PUSH), 0.0), FINGER_CLOSED, left_yaw(0.0)))
    # let go where the hand holds it (backing the push off first would drag a firmly held shaft off its seat)
    seated = shaft_mid_for(0.0, -PUSH)
    L += [("let go", 0.8, left_tcp(seated, 0.0), PAD_OPEN, left_yaw(0.0)),
          ("lift the open hand clear", 1.4, (seated[0], seated[1], TOP + 0.180), PAD_OPEN, left_yaw(0.0)),
          ("park", 1.6, PARK_L, FINGER_OPEN, 0.0)]
    end = max(sum(w[1] for w in L), sum(w[1] for w in R)) + 3.0
    L.append(("hold clear", end - sum(w[1] for w in L), PARK_L, FINGER_OPEN, 0.0))
    R.append(("hold clear", end - sum(w[1] for w in R), PARK_R, FINGER_OPEN, 0.0))
    return {"left": L, "right": R}


def demo():
    # Known start layout, the bench's drawing and time only: no sensing, no object or scorer reads.
    return {arm: [Waypoint(secs * DEMO_PACE, tuple(float(v) for v in p), finger, label, float(yaw))
                  for label, secs, p, finger, yaw in plan]
            for arm, plan in demo_plan().items()}
