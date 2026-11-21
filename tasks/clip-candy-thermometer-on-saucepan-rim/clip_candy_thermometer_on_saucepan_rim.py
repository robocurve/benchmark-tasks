"""Clip a candy / deep-fry dial thermometer onto a saucepan's rim so its probe hangs clear of the pan's floor.

An empty, cold Vollrath 682115 Wear-Ever 1.5 qt tapered aluminium sauce pan stands on the bench with its silicone
handle pointing to the robot's right. A Taylor 3505 TruTemp candy / deep-fry thermometer (2 in dial, 6 in stem) lies
on the bench to the left of the pan, its tip toward the robot, its stainless spring pan clip hanging under the stem;
the clip has been slid along the stem to this pan's setting (with the clip seated on the rim the tip hangs 20 mm off
the floor). The right hand holds the pan steady by its handle. The left hand takes the thermometer by its dial, turns
it in the air so it hangs stem-down with the clip toward the pan's far-right rim, lifts it over the rim, lowers it so
the clip's spring U straddles the rim (one leg inside, one outside) and pushes until the clip's top bend sits on the
rim, then lets go: the thermometer hangs from the rim by its clip, dial up, tip off the floor. Pushed harder than the
clip's grip on the stem, the stem slides down through the clip and the tip lands on the floor. Metres; x forward
from the arm bases' midpoint, y left, z up.
"""
import math
import pathlib
import sys
from collections import deque

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import box, cyl  # noqa: E402
from yam_joints import Hinge, Link, Slide  # noqa: E402
from yam_mesh import Mesh, MeshMechanism, MeshPart  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
STILL_S = 1.0
TEST_GRIPPERS = None
DIAG = False  # GPU diagnostic copies print POT_DIAG lines
DEMO_PACE = 1.0  # scales every demo duration except the fingers' close and open
DEMO_WRONG = ""  # wrong-way copies: "halfpush" = stop pushing with the bend 12 mm over the rim and let go


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
DEG = math.pi / 180
TAU = 2 * math.pi

ALUMINIUM = (0.62, 0.63, 0.65)  # natural-finish aluminium
SILICONE_BLACK = (0.07, 0.07, 0.08)
RIVET = (0.62, 0.63, 0.64)
STAINLESS = (0.66, 0.67, 0.70)
DIAL_WHITE = (0.95, 0.95, 0.93)
NEEDLE_RED = (0.80, 0.08, 0.08)
INDICATOR_BLUE = (0.12, 0.20, 0.55)


# ---------------------------------------------------------------- convex pieces
def _oriented(name, points, faces, color, proxy=(), collision="convex"):
    points, faces = np.asarray(points, float), np.asarray(faces, int)
    tri = (points - points.mean(axis=0))[faces]
    if np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum() < 0:
        faces = faces[:, ::-1]
    return Mesh(name, points, faces, collision=collision, color=color, proxy=tuple(proxy))


def _hexa(name, bottom, top, color, proxy=()):
    """A convex six-faced piece from a bottom quadrilateral and a top quadrilateral (corners in the same order)."""
    points = [tuple(map(float, p)) for p in bottom] + [tuple(map(float, p)) for p in top]
    n = 4
    faces = []
    for i in range(n):
        k = (i + 1) % n
        faces += [(i, k, n + k), (i, n + k, n + i)]
    faces += [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7)]
    return _oriented(name, points, faces, color, proxy)


def _cuboid(name, center, R, size, color, proxy=()):
    """A box of `size` (along R's columns) centred at `center`, turned by R: as a convex mesh."""
    c, R, h = np.asarray(center, float), np.asarray(R, float), np.asarray(size, float) / 2
    corner = [c + R @ (h * np.array(s)) for s in ((-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
                                                   (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1))]
    return _hexa(name, corner[:4], corner[4:], color, proxy)


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _T(R, p):
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = R, p
    return m


# ---------------------------------------------------------------- the saucepan (Vollrath 682115)
# Maker page: diameter 6.219 in, usable diameter 6.018 in, base diameter 5.594 in, height 4.21 in (to the handle's
# top), usable height 3.625 in, 13.16 in long with the handle, 11 gauge 3004 aluminium, double-thick rim and bottom,
# beadless, stay-cool silicone handle on rivets. Derived: rim wall (6.219 - 6.018) / 2 = 2.55 mm. Estimates: the
# bottom 4.6 mm (two 11-gauge sheets), the wall 2.55 mm all the way down, the handle's section 24 x 16 mm and its
# rise, the riveted bracket, the mass 0.55 kg (a case of six weighs 8.1 lb).
PAN_XY = np.array([0.120, 0.060])
R_OUT_TOP, R_IN_TOP = 78.99 * MM, 76.43 * MM
WALL = R_OUT_TOP - R_IN_TOP
R_OUT_BASE = 71.04 * MM
DEPTH_IN = 92.08 * MM
FLOOR_T = 4.6 * MM
PAN_H = DEPTH_IN + FLOOR_T  # 96.7 mm: the rim's top above the bench
PAN_MASS = 0.55
HANDLE_ANGLE = -90.0 * DEG  # the handle points to the robot's right
HANDLE_END_R = 255.3 * MM  # 13.16 in overall less the 79 mm on the far side
HANDLE_W, HANDLE_T = 24.0 * MM, 16.0 * MM
HANDLE_Z0, HANDLE_Z1 = 84.0 * MM, 99.0 * MM  # the handle's mid-height at its root and at its end (top 107 mm)
HANDLE_R0 = 81.0 * MM
N_WALL = 64
RIM_MID_R = (R_OUT_TOP + R_IN_TOP) / 2


def r_out(z):
    return R_OUT_BASE + (R_OUT_TOP - R_OUT_BASE) * z / PAN_H


def r_in(z):
    return r_out(z) - WALL


def _wall_pieces():
    out = []
    for i in range(N_WALL):
        a0, a1 = TAU * i / N_WALL, TAU * (i + 1) / N_WALL
        corners = []
        for z in (0.0, PAN_H):
            ri, ro = r_out(z) - WALL, r_out(z)
            corners.append([(ri * math.cos(a0), ri * math.sin(a0), z), (ro * math.cos(a0), ro * math.sin(a0), z),
                            (ro * math.cos(a1), ro * math.sin(a1), z), (ri * math.cos(a1), ri * math.sin(a1), z)])
        out.append(_hexa(f"wall_{i}", corners[0], corners[1], ALUMINIUM))
    return out


def _wall_proxy(n=24):
    """Offline stand-in for the wall: n axis-aligned boxes round it, inside the wall's band at mid height."""
    out = []
    rm, w = (r_out(PAN_H / 2) - WALL / 2), WALL
    for i in range(n):
        a = (i + 0.5) * TAU / n
        cx, cy = rm * math.cos(a), rm * math.sin(a)
        size = 2 * rm * math.sin(math.pi / n) * 0.9
        ex = abs(size * math.sin(a)) + abs(w * math.cos(a))
        ey = abs(size * math.cos(a)) + abs(w * math.sin(a))
        out.append(box((cx, cy, PAN_H / 2), (0.6 * ex, 0.6 * ey, PAN_H * 0.9)))
    return tuple(out)


def _handle_pieces():
    u = np.array([math.cos(HANDLE_ANGLE), math.sin(HANDLE_ANGLE), 0.0])
    t = np.array([-u[1], u[0], 0.0])
    slope = math.atan2(HANDLE_Z1 - HANDLE_Z0, HANDLE_END_R - HANDLE_R0)
    length = math.hypot(HANDLE_END_R - HANDLE_R0, HANDLE_Z1 - HANDLE_Z0)
    along = u * math.cos(slope) + np.array([0, 0, math.sin(slope)])
    up = np.cross(along, t)
    R = np.column_stack((along, t, up))
    mid = u * (HANDLE_R0 + HANDLE_END_R) / 2 + np.array([0, 0, (HANDLE_Z0 + HANDLE_Z1) / 2])
    proxy = tuple(box(tuple(u * r + np.array([0, 0, HANDLE_Z0 + (HANDLE_Z1 - HANDLE_Z0) * (r - HANDLE_R0)
                                                    / (HANDLE_END_R - HANDLE_R0)])),
                      (abs(u[0]) * 0.02 + abs(t[0]) * HANDLE_W * 0.8, abs(u[1]) * 0.02 + abs(t[1]) * HANDLE_W * 0.8,
                       HANDLE_T * 0.8)) for r in np.linspace(HANDLE_R0 + 0.015, HANDLE_END_R - 0.015, 8))
    bar = _cuboid("handle", mid, R, (length, HANDLE_W, HANDLE_T), SILICONE_BLACK, proxy)
    # the riveted aluminium bracket on the wall under the rim, carrying the handle's root (estimate)
    bz = 78.0 * MM
    br0 = r_out(bz)
    bmid = u * (br0 + 2.5 * MM + 0.004) + np.array([0, 0, bz])
    bracket = _cuboid("bracket", bmid, np.column_stack((u, t, (0, 0, 1))), (13.0 * MM, 26.0 * MM, 24.0 * MM),
                      ALUMINIUM, (box(tuple(bmid), (abs(u[0]) * 0.01 + abs(t[0]) * 0.02,
                                                     abs(u[1]) * 0.01 + abs(t[1]) * 0.02, 0.02)),))
    return [bar, bracket]


def make_pan():
    shapes = [cyl((0.0, 0.0, FLOOR_T / 2), r_in(FLOOR_T) + 0.5 * MM, FLOOR_T, color=ALUMINIUM)]
    u = np.array([math.cos(HANDLE_ANGLE), math.sin(HANDLE_ANGLE), 0.0])
    for dz in (-6.0 * MM, 6.0 * MM):  # two rivet heads inside the wall (visual and real bumps, 0.8 mm proud)
        c = u * (r_in(78.0 * MM + dz) - 0.4 * MM) + np.array([0, 0, 78.0 * MM + dz])
        shapes.append(box(tuple(c), (abs(u[0]) * 0.8 * MM + abs(u[1]) * 6.0 * MM,
                                     abs(u[1]) * 0.8 * MM + abs(u[0]) * 6.0 * MM, 6.0 * MM), color=RIVET))
    meshes = tuple(_wall_pieces()) + tuple(_handle_pieces())
    meshes = (Mesh(meshes[0].name, meshes[0].points, meshes[0].faces, collision="convex", color=ALUMINIUM,
                   proxy=_wall_proxy()),) + meshes[1:]
    return MeshPart("saucepan", (PAN_XY[0], PAN_XY[1], TOP + 0.3 * MM), tuple(shapes), mass=PAN_MASS,
                    color=ALUMINIUM, friction=(0.40, 0.35), contact_offset=0.001, meshes=meshes)


# ---------------------------------------------------------------- the thermometer (Taylor 3505)
# Published: 2 in dial, 6 in stainless stem, adjustable stainless pan clip, 100-380 F, NSF. From the maker's and
# retailers' photos: the dial's face is square to the stem (the stem leaves the middle of its back), so the dial faces
# up when the thermometer hangs on a pan; the clip is a bent stainless spring strip on a sleeve that slides along the
# stem, folded into an inverted U that goes over the rim. Estimates (ruler and kitchen scale): stem 4.8 mm across,
# dial case 14 mm deep, 40 g with the dial; clip 12 mm wide, its top bend 14 mm out from the stem's axis, inner leg
# 16 mm long with a 5 mm lead-in bent toward the stem, outer leg 14 mm long with an 8 mm flare bent outward, legs 4.5
# mm apart just under the bend (a loop wider than the rim) closing to 2.2 mm at the outer leg's end, 4 g; the clip's
# grip on the rim about 1 N; the stem slides through the clip's sleeve only past about 20 N (16 N once moving).
#
# Part frame (as it lies at the start): origin at the tip's centre, the stem along +x up to the dial's back, the clip
# hanging toward -z, the clip's width along y. Upright on a pan (R_UP below) +x points up and the clip outward.
STEM_R, STEM_L = 2.4 * MM, 152.4 * MM
DIAL_R, CASE_T = 25.4 * MM, 13.0 * MM  # the case; the face (0.6 mm) and the needle (0.4 mm) stand on its front
DIAL_T = CASE_T + 1.0 * MM  # 14 mm across the case, the face and the needle: what the pads take
DIAL_X = STEM_L + CASE_T / 2  # the case's middle along the stem from the tip
DIAL_MID = STEM_L + DIAL_T / 2  # the middle of what the pads take (case, face and needle)
THERMO_MASS = 0.040
CLIP_W, STRIP = 12.0 * MM, 1.0 * MM  # collider strip 1 mm (the real strip is thinner: estimate 0.6 mm)
CLIP_X = DEPTH_IN - 20.0 * MM  # the top bend's underside, this far up the stem from the tip: the tip hangs 20 mm off
IN_LEG_R = 14.0 * MM  # the inner leg's outer face (the face that meets the pan's inner wall), out from the stem
GAP = 2.2 * MM  # between the legs at rest, at the outer leg's lower end (where the spring bites)
TOP_GAP = 4.5 * MM  # between the legs just under the top bend: the rounded loop the rim seats up into
OUT_LEG_R = IN_LEG_R + GAP  # the outer leg's inner face at its lower end
OUT_TOP_R = IN_LEG_R + TOP_GAP  # and just under the bend
CROWN_R = OUT_TOP_R + STRIP  # the bend's outer end
IN_LEG_L, IN_LEAD_L, IN_LEAD_DEG = 16.0 * MM, 5.0 * MM, 35.0
OUT_LEG_L, FLARE_L, FLARE_DEG = 14.0 * MM, 8.0 * MM, 40.0
CLIP_MASS, LEAF_MASS = 4.0e-3, 1.5e-3
CLIP_STATIC, CLIP_DYNAMIC = 20.0, 16.0  # N: the sleeve's grip on the stem (estimate; a 2 kg weight on the dial)
CLIP_ARMATURE = 0.005
LEAF_SPRING = 0.11  # N m/rad; preloaded closed by LEAF_PRELOAD_DEG: about 1 N on the rim's outer face when seated
LEAF_PRELOAD_DEG = 6.0
LEAF_ARMATURE = 2.0e-6
HINGE = (CLIP_X + STRIP / 2, 0.0, -(OUT_TOP_R + STRIP / 2))  # the outer leg's hinge: the top bend's outer corner
TIP_X, TIP_Y = 0.045, 0.200  # where the tip lies at the start
THERMO_FRICTION = (0.35, 0.30)  # stainless (estimate)


def _strip(name, a, b, side, color, proxy=True):
    """A 1 mm strip (clip-wide) in the part frame's x-z plane whose face on `side` (+1: toward the stem, +z; -1: away)
    runs from point a to point b: legs and their bent ends meet face corner to face corner, so the rim never meets a
    ledge where a leg's flat end pokes out past the bent end."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = (b - a) / np.linalg.norm(b - a)
    n = np.array([-d[2], 0.0, d[0]])  # in-plane normal
    if n[2] * side < 0:
        n = -n
    c = (a + b) / 2 - n * STRIP / 2  # the strip lies behind its face
    R = np.column_stack((d, (0.0, 1.0, 0.0), np.cross(d, (0.0, 1.0, 0.0))))
    length = float(np.linalg.norm(b - a))
    return _cuboid(name, c, R, (length, CLIP_W, STRIP), color,
                   (box(tuple(c), (length * 0.6, CLIP_W * 0.8, STRIP)),) if proxy else ())


def _hull2d(points):
    """Convex hull (counter-clockwise, no collinear points) of 2-D points."""
    P = sorted({(round(float(x), 12), round(float(z), 12)) for x, z in points})

    def turn(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in P:
        while len(lower) >= 2 and turn(lower[-2], lower[-1], p) <= 1e-14:
            lower.pop()
        lower.append(p)
    for p in reversed(P):
        while len(upper) >= 2 and turn(upper[-2], upper[-1], p) <= 1e-14:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _one_collider(name, strips, color):
    """Clip-wide strips (a leg and its bent end) as ONE convex collider and render mesh: the convex hull of their
    corners, a prism across the clip's width. A leg and its bent end are convex on the side the rim meets, so the hull
    adds nothing there, and the rim's edge never meets the flat end of a separate leg collider (whose contact normal
    would point along the leg and stop the clip instead of opening it)."""
    poly = _hull2d([(p[0], p[2]) for m in strips for p in m.points])
    n = len(poly)
    pts = [(x, -CLIP_W / 2, z) for x, z in poly] + [(x, CLIP_W / 2, z) for x, z in poly]
    faces = []
    for i in range(n):
        k = (i + 1) % n
        faces += [(i, n + i, n + k), (i, n + k, k)]
    for i in range(1, n - 1):
        faces += [(0, i, i + 1), (n, n + i + 1, n + i)]
    return _oriented(name, pts, faces, color, tuple(b for m in strips for b in m.proxy))


def _clip_meshes():
    """The clip's sleeve, top bend and inner leg (the mechanism's base), and its outer leg (link "leaf"): part frame at
    q=0."""
    x0 = CLIP_X
    sleeve = box((x0 + 5.0 * MM, 0.0, 0.0), (10.0 * MM, 7.0 * MM, 7.0 * MM), color=STAINLESS)
    bend = box((x0 + STRIP / 2, 0.0, -(3.0 * MM + CROWN_R) / 2), (STRIP, CLIP_W, CROWN_R - 3.0 * MM),
               color=STAINLESS)
    # the inner leg and its lead-in (bent toward the stem), one convex collider
    a = IN_LEAD_DEG * DEG
    A = np.array([x0 - IN_LEG_L, 0.0, -IN_LEG_R])
    inner = _one_collider("inner_leg", (
        _strip("inner_strip", np.array([x0, 0.0, -IN_LEG_R]), A, -1, STAINLESS),
        _strip("lead_in", A, A + IN_LEAD_L * np.array([-math.cos(a), 0.0, math.sin(a)]), -1, STAINLESS)), STAINLESS)
    # the outer leg: its inner face runs from 4.5 mm off the inner leg just under the bend to 2.2 mm at its lower end
    top = np.array([x0, 0.0, -OUT_TOP_R])
    C = np.array([x0 - OUT_LEG_L, 0.0, -OUT_LEG_R])
    outer = box((x0 - 1.5 * MM, 0.0, -(OUT_TOP_R + STRIP / 2)), (3.0 * MM, CLIP_W, STRIP), color=STAINLESS)
    # the outer leg and its flare (bent outward from the leg's lower end), one convex collider
    b = FLARE_DEG * DEG
    leaf = _one_collider("outer_leg", (
        _strip("leg_strip", top, C, +1, STAINLESS),
        _strip("flare", C, C + FLARE_L * np.array([-math.cos(b), 0.0, -math.sin(b)]), +1, STAINLESS)), STAINLESS)
    return (sleeve, bend), inner, (outer,), leaf


def make_thermometer():
    """The Taylor 3505 lying on the bench: tip toward the robot, clip under the stem. The mechanism's base is the
    clip (its sleeve, top bend and inner leg), so its two joints hang off one body and PhysX keeps it as the root:
    link `stem` (stem, dial, face, needle, marker) slides along the clip's sleeve; link `leaf` (outer leg and flare)
    swings on the top bend. The base frame is the part frame: the tip sits at its origin while the slide reads 0."""
    stem = cyl((STEM_L / 2, 0.0, 0.0), STEM_R, STEM_L, axis="x", color=STAINLESS)
    dial = cyl((DIAL_X, 0.0, 0.0), DIAL_R, CASE_T, axis="x", color=STAINLESS)
    face = cyl((STEM_L + CASE_T + 0.3 * MM, 0.0, 0.0), DIAL_R - 2.0 * MM, 0.6 * MM, axis="x", color=DIAL_WHITE)
    needle = box((STEM_L + CASE_T + 0.8 * MM, 0.0, 7.0 * MM), (0.4 * MM, 2.0 * MM, 14.0 * MM), color=NEEDLE_RED)
    tab = box((STEM_L + CASE_T - 2.5 * MM, DIAL_R + 1.0 * MM, 0.0), (4.0 * MM, 2.0 * MM, 5.0 * MM),
              color=INDICATOR_BLUE)  # the blue target-temperature marker on the bezel
    clip_shapes, inner, leaf_shapes, outer_leg = _clip_meshes()
    stem_link = Link("stem", (stem, dial, face, needle, tab),
                     Slide("x", (0.0, 0.0, 0.0), -0.060, 0.040, damping=0.5, armature=CLIP_ARMATURE,
                           static_friction=CLIP_STATIC, dynamic_friction=CLIP_DYNAMIC),
                     THERMO_MASS, color=STAINLESS, friction=THERMO_FRICTION)
    leaf = Link("leaf", leaf_shapes,
                Hinge("y", HINGE, -25.0, 0.0, damping=0.0005, spring=LEAF_SPRING, rest=LEAF_PRELOAD_DEG,
                      armature=LEAF_ARMATURE),
                LEAF_MASS, color=STAINLESS, friction=THERMO_FRICTION)
    return MeshMechanism("thermometer", (TIP_X, TIP_Y, TOP + DIAL_R + 0.5 * MM), clip_shapes, (stem_link, leaf),
                         mass=CLIP_MASS, color=STAINLESS, friction=THERMO_FRICTION, contact_offset=0.001,
                         grip=None, meshes={"base": (inner,), "leaf": (outer_leg,)})


def clip_slid(th):
    """How far the stem has been pushed down through the clip from its setting (m; positive = pushed down)."""
    return -float(th.joints["stem"])


def stem_shift(th):
    """The stem's offset along its own axis from the base frame (m): the tip sits at origin + R @ (shift, 0, 0)."""
    return float(th.joints.get("stem", 0.0)) if hasattr(th, "joints") else 0.0


PAN = make_pan()
THERMO = make_thermometer()
OBJECTS = [PAN, THERMO]

# ---------------------------------------------------------------- upright on the pan
U_DEG = -30.0  # the clip goes on the rim at this bearing from the pan's centre (the pan's far-right rim)
U = np.array([math.cos(U_DEG * DEG), math.sin(U_DEG * DEG), 0.0])
TANGENT = np.array([-U[1], U[0], 0.0])
TAPER = math.atan2(R_OUT_TOP - R_OUT_BASE, PAN_H)  # 4.7 deg: the wall leans out; the seated clip and stem follow it
R_UP = _rot_z(U_DEG * DEG) @ _rot_y(-90.0 * DEG)  # part frame -> world, stem vertical, dial up, clip along U


def _axis_rot(u, a):
    u = np.asarray(u, float) / np.linalg.norm(u)
    k = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * (k @ k)


R_HANG = _axis_rot(TANGENT, TAPER) @ R_UP  # as it hangs on the rim: legs along the wall, tip 4.7 deg in
CROWN_PT = np.array([CLIP_X, 0.0, -(IN_LEG_R + WALL / 2)])  # under the top bend, over the rim's middle when seated
MOUTH_DOWN = max(IN_LEG_L + IN_LEAD_L * math.cos(IN_LEAD_DEG * DEG), OUT_LEG_L + FLARE_L * math.cos(FLARE_DEG * DEG))


def rim_point(pan_xy=None):
    p = PAN_XY if pan_xy is None else np.asarray(pan_xy, float)
    return p + RIM_MID_R * U[:2]


def hang_pose(crown_xy, crown_z, R=None):
    """4x4 pose of the thermometer hanging as on the rim (R_HANG) with its top bend's middle at (crown_xy, crown_z)."""
    R = R_HANG if R is None else R
    c = R @ CROWN_PT
    return _T(R, np.array([crown_xy[0] - c[0], crown_xy[1] - c[1], crown_z - c[2]]))


def rim_top_z():
    return TOP + PAN_H


AXIS = R_HANG[:, 0]  # up the stem as it hangs: along the wall's 4.7 deg lean
AIM_IN = 0.3 * MM  # aim the clip this far in from the rim's middle: its flat 12 mm inner leg's edges would otherwise
# meet the curved inner wall 0.23 mm before its middle does


def seat_line(pan_xy, corr_xy, h):
    """The hanging thermometer's pose with its top bend h above the rim's top, on the line along which the seated clip
    slides onto the rim (the wall's lean): the legs stay parallel to the wall all the way down."""
    xy = rim_point(pan_xy) - U[:2] * AIM_IN + np.asarray(corr_xy, float)[:2] + AXIS[:2] * (h / AXIS[2])
    return hang_pose(xy, rim_top_z() + h)


# ---------------------------------------------------------------- scoring
SEAT_MM = (3.0, 10.0)  # top bend's underside over the rim: 1.0 within 3 mm, 0.5 within 10 mm, else 0.25
GAP_MM = (10.0, 5.0, 1.0)  # tip over the floor: 1.0 from 10 mm, 0.5 from 5 mm, 0.25 from 1 mm; under 1 mm touches
SHIFT_MM = (10.0, 20.0, 30.0)  # pan's shift from its marks: 1.0 / 0.75 / 0.5; past 30 mm scores 0
TILT_DEG = (10.0, 20.0)  # stem from vertical: 1.0 / 0.5 / 0.25
HOOK_R_MM = 4.0  # the rim lies between the clip's legs: the top bend's middle within this of the rim's middle
HOOK_UP_MM = 16.0  # and the bend no more than this over the rim (the inner leg's straight length)
PAN_TIP_DEG = 20.0
CLEAR = 0.030  # every gripper's TCP at least this far from the thermometer's bounding box
STILL_MOVE = 1.0 * MM
SPEED = 0.01


def _pan_frame(pan):
    return np.asarray(pan.origin, float), pan.rotation()


def thermo_terms(th, pan):
    base, R = np.asarray(th.origin, float), th.rotation()
    o = base + R[:, 0] * stem_shift(th)  # the tip
    po, pR = _pan_frame(pan)
    axis = R[:, 0]  # tip -> dial
    tilt = math.degrees(math.acos(float(np.clip(axis[2], -1.0, 1.0))))
    tip_l = pR.T @ (o - po)
    tip_gap = float(tip_l[2] - FLOOR_T - STEM_R * math.sin(min(tilt, 90.0) * DEG))
    tip_r = float(np.hypot(tip_l[0], tip_l[1]))
    inside = bool(tip_r < r_in(max(tip_l[2], FLOOR_T)) + 0.5 * MM and FLOOR_T - 0.003 < tip_l[2] < PAN_H)  # may rest
    # against the inner wall, as a real one does
    crown = base + R @ CROWN_PT  # the clip is the mechanism's base: read from its frame, never a link's pose
    crown_l = pR.T @ (crown - po)
    crown_r = float(np.hypot(crown_l[0], crown_l[1]))
    crown_gap = float(crown_l[2] - PAN_H)
    hooked = bool(abs(crown_r - RIM_MID_R) <= HOOK_R_MM * MM and -1.0 * MM <= crown_gap <= HOOK_UP_MM * MM)
    return {"tilt": tilt, "tip_gap": tip_gap, "tip_r": tip_r, "inside": inside, "crown_r": crown_r,
            "crown_gap": crown_gap, "hooked": hooked, "slid": clip_slid(th), "tip": o}


def pan_terms(pan):
    po, pR = _pan_frame(pan)
    shift = float(np.hypot(po[0] - PAN_XY[0], po[1] - PAN_XY[1]))
    tip = math.degrees(math.acos(float(np.clip(pR[2, 2], -1.0, 1.0))))
    return {"shift": shift, "tipped": tip}


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


def still(obj, key):
    """Over the last second the object moved under 1 mm and turned under 2 degrees (offline: its speed)."""
    t = _RUN["t"]
    if t is None:
        return bool(np.linalg.norm(obj.lin_vel) < SPEED)
    w = _WINDOWS.setdefault(key, {"episode": None, "hist": deque()})
    hist = w["hist"]
    if w["episode"] != _RUN["episode"] or (hist and t < hist[-1][0]):
        hist.clear()
        w["episode"] = _RUN["episode"]
    now = (np.asarray(obj.origin, float), obj.rotation())
    if not hist or t > hist[-1][0] + 1e-9:
        hist.append((t, now))
    while len(hist) > 1 and hist[1][0] <= t - STILL_S + 1e-9:
        hist.popleft()
    if t - hist[0][0] < STILL_S - 0.051:
        return False
    return all(np.linalg.norm(now[0] - old[0]) <= STILL_MOVE and _rot_deg(now[1], old[1]) <= 2.0
               for _, old in hist)


def score_terms(objects):
    th, pan = objects["thermometer"], objects["saucepan"]
    t = thermo_terms(th, pan)
    t.update(pan_terms(pan))
    t["released"] = _box_clear(th, CLEAR)
    t["still"] = still(th, "thermometer")
    lost = (float(np.asarray(th.origin, float)[2]) < TOP - 0.02 or float(np.asarray(pan.origin, float)[2]) < TOP - 0.02)
    t["lost"] = ever("an object left the bench", lost)
    if DIAG:
        _diag(objects, t)
    return t


def score(objects):
    t = score_terms(objects)
    if t["lost"] or t["tipped"] > PAN_TIP_DEG or t["shift"] > SHIFT_MM[2] * MM:
        return 0.0
    if not (t["hooked"] and t["released"] and t["inside"]) or t["tip_gap"] < GAP_MM[2] * MM:
        return 0.0
    g, sh, cg = t["tip_gap"] / MM, t["shift"] / MM, t["crown_gap"] / MM
    seat = 1.0 if cg <= SEAT_MM[0] else 0.5 if cg <= SEAT_MM[1] else 0.25
    gap = 1.0 if g >= GAP_MM[0] else 0.5 if g >= GAP_MM[1] else 0.25
    shift = 1.0 if sh <= SHIFT_MM[0] else 0.75 if sh <= SHIFT_MM[1] else 0.5
    tilt = 1.0 if t["tilt"] <= TILT_DEG[0] else 0.5 if t["tilt"] <= TILT_DEG[1] else 0.25
    s = min(seat, gap, shift, tilt)
    return s if t["still"] else min(s, 0.75)


def success(objects):
    return score(objects) >= 1.0


# ---------------------------------------------------------------- diagnostics (GPU copies only)
_DIAG = {"step": -1}


def _diag(objects, t):
    step = _RUN["step"]
    if _RUN["t"] is None or step == _DIAG["step"] or step % 5:
        return
    _DIAG["step"] = step
    th = objects["thermometer"]
    hands = grippers() or {}
    tcp = " ".join(f"{a[0]}=({','.join(f'{v * 1000:.1f}' for v in hands[a][0])};{hands[a][1]:.2f})" for a in hands)
    o = t["tip"]
    print(f"POT_DIAG t={_RUN['t']:.2f} tip=({o[0] * 1000:.1f},{o[1] * 1000:.1f},{(o[2] - TOP) * 1000:.1f}) "
          f"gap={t['tip_gap'] * 1000:.1f} tip_r={t['tip_r'] * 1000:.1f} in={int(t['inside'])} "
          f"crown_r={t['crown_r'] * 1000:.1f} crown_gap={t['crown_gap'] * 1000:.1f} hook={int(t['hooked'])} "
          f"slid={t['slid'] * 1000:.1f} leaf={math.degrees(th.joints['leaf']):.1f} tilt={t['tilt']:.1f} "
          f"pan_shift={t['shift'] * 1000:.1f} pan_tip={t['tipped']:.1f} rel={int(t['released'])} "
          f"st={int(t['still'])} {tcp}", flush=True)


# ---------------------------------------------------------------- demo
# Right hand: pinches the silicone handle top-down across its width and holds the pan until the left hand has let
# go. Left hand: pinches the lying thermometer's dial top-down across its depth (pads on the lens and the back,
# closing along the stem), lifts it, reads how it sits in the pinch, turns it in the air (the fingers end level,
# pointing along U, one pad above and one below the dial's edge) so it hangs stem-down with the clip toward the
# rim, rises so the tip clears the rim, carries it over the rim, lowers it so the clip's U straddles the rim, pushes
# the bend onto the rim, lets go and backs out. Every left-hand pose after the lift is worked out from the
# thermometer's wanted pose and the grasp read from the simulator (scripted demos may read poses); it never moves an
# object.
OPEN_PICK = -0.020  # pads 40 mm apart
PINCH_DIAL = -(DIAL_T / 2 - 1.5 * MM)  # pads aimed 11 mm apart on the 14 mm dial
OPEN_REL = -0.015
PINCH_HANDLE = -(HANDLE_W / 2 - 1.5 * MM)  # pads aimed 21 mm apart on the 24 mm handle
PICK_DOWN = 2.0 * MM  # the TCP this far below the dial's top: pads from 1 mm above it to 15 mm below
HIGH_TIP = 18.0 * MM  # carried over the rim with the tip this far above the rim's top
MOUTH_UP = 5.0 * MM  # lined up with the clip's mouth this far over the rim
PUSH_LEAD = 15.0 * MM  # the push aims the bend this far below the rim's top (the arm is a soft spring)
WRONG_STOP = 12.0 * MM  # the wrong way stops with the bend this far over the rim
READ_MAX = 0.015
BESIDE = np.array([0.200, 0.190])  # the top bend's place beside the pan (left of it) once the thermometer hangs
PARK = {"left": np.array([0.140, 0.240, TOP + 0.200]), "right": np.array([0.140, -0.240, TOP + 0.200])}
FINAL_HOLD_S = 3.0
HANDLE_GRIP_R = 175.0 * MM


def R_grip(close, approach):
    a = np.asarray(approach, float)
    a = a / np.linalg.norm(a)
    c = np.asarray(close, float)
    c = c - a * (c @ a)
    c = c / np.linalg.norm(c)
    return np.column_stack((c, np.cross(a, c), a))


R_PICK = R_grip((-1.0, 0.0, 0.0), (0.0, 0.0, -1.0))


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


def _float_yaw(R):
    """The offline checker's top-down yaw for a hand rotation: its closing axis's heading if it is level, else the
    pick's (the checker models every grip as top-down)."""
    c = R[:, 0]
    if abs(c[2]) > 0.7:
        return math.atan2(-float(R_PICK[0, 0]), float(R_PICK[1, 0]))
    return math.atan2(-float(c[0]), float(c[1]))


def _state_now(name):
    env = _RUN["env"]
    if env is None:
        return None
    try:
        from yam_arena import scene as arena_scene
        return arena_scene.object_states(env, [name])[name]
    except Exception as e:  # noqa: BLE001
        print(f"POT_READ {name} failed: {e!r}", flush=True)
        return None


_ARMS = {}


def _arm_state(arm):
    s = _ARMS.get(arm)
    if s is None or s["episode"] != _RUN["episode"]:
        s = {"episode": _RUN["episode"], "O0": None, "Tho": None, "corr": np.zeros(3), "pan": None, "cache": {}}
        _ARMS[arm] = s
    return s


def thermo_start_pose():
    """The thermometer's planned start pose (4x4), resting on its dial's rim and the clip's flare (settled)."""
    p = np.array([TIP_X, TIP_Y, TOP + DIAL_R])
    return _T(np.eye(3), p)


def _thermo_now():
    st = _state_now("thermometer")
    if st is None:
        return None
    return _T(st.rotation(), np.asarray(st.origin, float))


def _pan_now(S):
    if S["pan"] is None:
        st = _state_now("saucepan")
        S["pan"] = PAN_XY.copy() if st is None else np.asarray(st.origin, float)[:2].copy()
        if st is not None:
            print(f"POT_READ pan t={_RUN['t']:.2f} shift_mm=({(S['pan'][0] - PAN_XY[0]) * 1000:.1f},"
                  f"{(S['pan'][1] - PAN_XY[1]) * 1000:.1f})", flush=True)
    return S["pan"]


def _clamp(v, m):
    n = float(np.linalg.norm(v))
    return v * (m / n) if n > m else v


def _read_start(S):
    planned = thermo_start_pose()
    actual = _thermo_now()
    if actual is None:
        S["O0"] = planned
        return
    shift = _clamp(actual[:3, 3] - planned[:3, 3], 0.025)
    S["O0"] = _T(actual[:3, :3], planned[:3, 3] + shift)
    st = _state_now("thermometer")
    S["shift"] = stem_shift(st) if st is not None else 0.0
    tilt = math.degrees(math.acos(float(np.clip(actual[2, 2], -1, 1))))
    print(f"POT_READ start t={_RUN['t']:.2f} shift_mm=({shift[0] * 1000:.1f},{shift[1] * 1000:.1f},"
          f"{shift[2] * 1000:.1f}) roll_tilt={tilt:.1f}", flush=True)


def _dial_pick(S):
    """TCP and hand rotation for the pick: over the dial's top, closing along the stem as it lies."""
    O = S["O0"]
    ax = O[:3, 0]
    yaw_ax = np.array([ax[0], ax[1], 0.0])
    yaw_ax /= np.linalg.norm(yaw_ax)
    R = R_grip(-yaw_ax, (0.0, 0.0, -1.0))
    dial_c = O[:3, 3] + O[:3, :3] @ np.array([DIAL_MID + S.get("shift", 0.0), 0.0, 0.0])
    top = dial_c[2] + DIAL_R * math.sqrt(max(0.0, 1.0 - ax[2] ** 2))
    return np.array([dial_c[0], dial_c[1], top - PICK_DOWN]), R


def _grasp_from(S, hand):
    """Read how the thermometer sits in the hand (commanded hand pose `hand`, 4x4)."""
    actual = _thermo_now()
    if actual is None:  # offline: the planned grasp
        p, R = _dial_pick(S)
        actual = S["O0"] if S["O0"] is not None else thermo_start_pose()
        hand = _T(R, p)
    S["Tho"] = np.linalg.inv(hand) @ actual


def _hand_for(S, obj):
    """The hand pose (4x4) that puts the thermometer at `obj` (4x4) with the grasp last read."""
    return obj @ np.linalg.inv(S["Tho"])


def _read_hang(S, hand, want, key):
    """The hanging thermometer's clip (the point under its top bend) against where it was meant to be: correct the
    hand's aim (xy only)."""
    actual = _thermo_now()
    if actual is None:
        return
    err = (want[:3, :3] @ CROWN_PT + want[:3, 3]) - (actual[:3, :3] @ CROWN_PT + actual[:3, 3])  # the clip's bend
    err[2] = 0.0
    S["corr"] = _clamp(S["corr"] + err, READ_MAX)
    tilt = math.degrees(math.acos(float(np.clip(actual[2, 0], -1, 1))))
    print(f"POT_READ {key} t={_RUN['t']:.2f} err_mm=({err[0] * 1000:.2f},{err[1] * 1000:.2f}) "
          f"corr_mm=({S['corr'][0] * 1000:.1f},{S['corr'][1] * 1000:.1f}) tilt={tilt:.1f} "
          f"tip_z={(actual[2, 3] - TOP) * 1000:.1f}", flush=True)


class PotWaypoint(Waypoint):
    """One arm's waypoint worked out when it is first played: fn(S, prev_hand_4x4) -> hand 4x4."""

    def __init__(self, arm, index, duration, finger, label, fn, keep=False):
        self.arm, self.index, self.fn = arm, index, fn
        self.duration = duration * (1.0 if keep else DEMO_PACE)
        self.finger, self.label = finger, label

    def _pose(self):
        S = _arm_state(self.arm)
        if self.index not in S["cache"]:
            prev = S["cache"].get(self.index - 1)
            S["cache"][self.index] = np.asarray(self.fn(S, prev), float)
        return S["cache"][self.index]

    @property
    def pos(self):
        return tuple(float(v) for v in self._pose()[:3, 3])

    @property
    def yaw(self):
        R = self._pose()[:3, :3]
        return Grip(_float_yaw(R), R)


def _left_plan():
    def first(S, prev):
        _read_start(S)
        p, R = _dial_pick(S)
        return _T(R, p + np.array([0.0, 0.0, 0.040]))

    def plan_start(S, prev):
        p, R = _dial_pick(_arm_state_planned())
        return _T(R, p + np.array([0.0, 0.0, 0.060]))

    def down(S, prev):
        p, R = _dial_pick(S)
        return _T(R, p)

    def lift(S, prev):
        h = prev.copy()
        h[2, 3] = TOP + 0.120
        return h

    def read_grasp(S, prev):
        _grasp_from(S, prev)
        return prev

    def hang_beside(S, prev):
        return _hand_for(S, hang_pose(BESIDE, rim_top_z() + HIGH_TIP + CLIP_X))

    def regrasp_read(S, prev):
        _grasp_from(S, prev)  # the turn may have shifted it in the pinch
        return prev

    high = HIGH_TIP + CLIP_X  # the top bend's height over the rim's top
    mouth = MOUTH_DOWN + MOUTH_UP
    zero = np.zeros(2)

    def over_rim(S, prev):
        return _hand_for(S, seat_line(_pan_now(S), zero, high))

    def line_up(S, prev):
        _read_hang(S, prev, seat_line(_pan_now(S), zero, high), "over")
        return _hand_for(S, seat_line(_pan_now(S), S["corr"], mouth))

    def mouth_read(S, prev):
        _read_hang(S, prev, seat_line(_pan_now(S), zero, mouth), "mouth")
        return _hand_for(S, seat_line(_pan_now(S), S["corr"], mouth))

    def push(S, prev):  # along the wall's lean, not straight down: the legs slide along the rim's faces
        lead = -WRONG_STOP if DEMO_WRONG == "halfpush" else PUSH_LEAD
        return _hand_for(S, seat_line(_pan_now(S), S["corr"], -lead))

    def back_out(S, prev):
        h = prev.copy()
        h[:3, 3] = h[:3, 3] - U * 0.055 + np.array([0.0, 0.0, 0.003])
        return h

    def up(S, prev):
        h = prev.copy()
        h[2, 3] = TOP + 0.230
        return h

    def park(S, prev):  # back to a straight-down hand, out of the top camera's view of the pan
        return _T(R_PICK, PARK["left"])

    plan = [("reach over the dial", 2.0, OPEN_PICK, plan_start, False),
            ("over the dial", 1.0, OPEN_PICK, first, False),
            ("fingers down either side of the dial", 1.5, OPEN_PICK, down, False),
            ("pinch the dial", 1.0, PINCH_DIAL, lambda S, p: p, True),
            ("lift it: it hangs from the dial, stem level", 1.5, PINCH_DIAL, lift, False),
            ("hold still and read the grasp", 0.5, PINCH_DIAL, read_grasp, False),
            ("turn it to hang stem-down, rising beside the pan", 3.5, PINCH_DIAL, hang_beside, False),
            ("let it hang still", 0.8, PINCH_DIAL, regrasp_read, False),
            ("carry it over the rim, tip clear of the rim", 2.5, PINCH_DIAL, over_rim, False),
            ("let it hang still over the rim", 0.8, PINCH_DIAL, lambda S, p: p, False),
            ("lower it: clip's mouth just over the rim", 2.0, PINCH_DIAL, line_up, False),
            ("line the mouth up on the rim", 0.8, PINCH_DIAL, mouth_read, False),
            ("push the clip down over the rim", 2.5, PINCH_DIAL, push, False),
            ("hold it on the rim", 1.0, PINCH_DIAL, lambda S, p: p, False),
            ("let go of the dial", 0.8, OPEN_REL, lambda S, p: p, True),
            ("back the fingers out", 1.2, OPEN_REL, back_out, False),
            ("up", 1.0, OPEN_REL, up, False),
            ("park", 2.5, OPEN_REL, park, False),
            ("hold clear", FINAL_HOLD_S, OPEN_REL, lambda S, p: p, True)]
    return plan


def _arm_state_planned():
    """A throwaway state with the planned start, for the first waypoint (before any read)."""
    return {"O0": thermo_start_pose()}


def handle_point(pan_xy=None, r=HANDLE_GRIP_R):
    p = PAN_XY if pan_xy is None else np.asarray(pan_xy, float)
    u = np.array([math.cos(HANDLE_ANGLE), math.sin(HANDLE_ANGLE)])
    z = HANDLE_Z0 + (HANDLE_Z1 - HANDLE_Z0) * (r - HANDLE_R0) / (HANDLE_END_R - HANDLE_R0)
    return np.array([p[0] + u[0] * r, p[1] + u[1] * r, TOP + 0.3 * MM + z + HANDLE_T / 2])


R_HANDLE = R_grip((math.cos(HANDLE_ANGLE + math.pi / 2), math.sin(HANDLE_ANGLE + math.pi / 2), 0.0),
                  (0.0, 0.0, -1.0))


def _right_plan(hold_s):
    def over_planned(S, prev):
        return _T(R_HANDLE, handle_point() + np.array([0.0, 0.0, 0.040]))

    def over(S, prev):  # read where the pan stands (scripted demos may read poses)
        return _T(R_HANDLE, handle_point(_pan_now(S)) + np.array([0.0, 0.0, 0.040]))

    def down(S, prev):
        return _T(R_HANDLE, handle_point(_pan_now(S)) - np.array([0.0, 0.0, 2.0 * MM]))

    def up(S, prev):
        h = prev.copy()
        h[2, 3] += 0.060
        return h

    return [("reach over the handle", 2.0, OPEN_PICK, over_planned, False),
            ("over the handle", 0.6, OPEN_PICK, over, False),
            ("fingers down either side of the handle", 1.5, OPEN_PICK, down, False),
            ("pinch the handle", 1.0, PINCH_HANDLE, lambda S, p: p, True),
            ("hold the pan steady by its handle", hold_s, PINCH_HANDLE, lambda S, p: p, True),
            ("let go of the handle", 0.8, OPEN_PICK, lambda S, p: p, True),
            ("lift off the handle", 1.0, OPEN_PICK, up, False),
            ("park", 2.0, OPEN_PICK, lambda S, p: _T(R_HANDLE, PARK["right"]), False),
            ("hold clear", FINAL_HOLD_S, OPEN_PICK, lambda S, p: p, True)]


def _waypoints(arm, plan):
    _ARMS.pop(arm, None)
    return [PotWaypoint(arm, i, secs, finger, label, fn, keep) for i, (label, secs, finger, fn, keep)
            in enumerate(plan)]


def demo():
    """The right hand takes the handle first and holds the pan until the left hand has let go of the thermometer."""
    left = _waypoints("left", _left_plan())
    labels = [w.label for w in left]
    let_go = sum(w.duration for w in left[:labels.index("let go of the dial") + 1])
    right_before = 2.0 + 0.6 + 1.5 + 1.0
    return {"left": left, "right": _waypoints("right", _right_plan(let_go + 0.5 - right_before))}


# ---------------------------------------------------------------- offline test layouts (states.yaml)
def _quat(rot):
    from yam_arena.ik import matrix_to_quat_xyzw
    return [float(v) for v in matrix_to_quat_xyzw(np.asarray(rot, float))]


def thermo_on_rim(crown_up_mm=0.0, slid_mm=0.0, extra_tilt_deg=0.0, dr_mm=0.0, pan_dx=0.0, pan_dy=0.0):
    """[origin, quat] of the thermometer hanging on the rim: its top bend's middle crown_up_mm over the rim's top and
    dr_mm outward of the rim's middle (base frame; give the stem's joint -slid_mm/1000 for a stem pushed down),
    leaning extra_tilt_deg more toward the pan's centre than the wall's 4.7 degrees. The pan as at pan_at(dx, dy)."""
    R = _axis_rot(TANGENT, extra_tilt_deg * DEG) @ R_HANG
    xy = rim_point(PAN_XY + np.array([pan_dx, pan_dy])) + U[:2] * dr_mm * MM
    c = R @ CROWN_PT  # the clip (the base) sits on the rim; slid_mm is the stem's slide, set as its joint
    z = TOP + 0.3 * MM + PAN_H + crown_up_mm * MM
    o = np.array([xy[0] - c[0], xy[1] - c[1], z - c[2]])
    return [float(v) for v in o], _quat(R)


def thermo_lying(dx=0.0, dy=0.0):
    p = thermo_start_pose()[:3, 3] + np.array([dx, dy, 0.0])
    return [float(v) for v in p], _quat(np.eye(3))


def thermo_upright_at(x, y, tip_z):
    """Upright (as if held) with its tip at (x, y) and height tip_z above the bench, clip along U."""
    return [float(x), float(y), float(TOP + tip_z)], _quat(R_HANG)


def pan_at(dx=0.0, dy=0.0, tip_deg=0.0):
    p = np.array([PAN_XY[0] + dx, PAN_XY[1] + dy, TOP + 0.3 * MM])
    return [float(v) for v in p], _quat(_rot_x(tip_deg * DEG))
