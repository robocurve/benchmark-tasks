"""Slide a Shure SM58 into its A25D clip on a light tripod desk stand, holding the stand, and aim it at the speaker's mark.

An On-Stage DS7425 tripod desktop mic stand (363 g) stands on the table with a Shure A25D mic clip screwed on its
5/8 in thread, the clip's swivel tightened level. A Shure SM58 lies on the table to the right. A red circle on
the table marks where the speaker sits. The left hand takes the stand by its tube, lifts it a few millimetres, turns it
so the clip points at the circle and sets it down; it keeps holding the stand while the right hand picks up the mic
near its balance point, puts its tail end into the clip's open end and slides it tail-first through the clip until
the handle's taper wedges snug in the clip's sprung jaws. Then both let go. Metres; x forward from the arm bases, y
left, z up.

The A25D (module section "the A25D clip", reusable) is part of the stand mechanism: its body rides on the stand's
inner shaft (link `head`, on the height clutch's slide), and two sprung jaws form its C-shaped cradle; the handle
spreads the jaws. PhysX makes the body with the most joints the articulation root, and reset puts the root at the
object's spawn pose, so the stand's base (legs, hub, tube, clutch) carries all three joints: the head's slide and the
two jaw hinges (the jaws' hinge lines stay on the clip body while the clutch holds, which it does under these loads).
"""
import math
import pathlib
import sys
from collections import deque

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, ball, box, cyl  # noqa: E402
from yam_joints import Hinge, Link, Slide  # noqa: E402
from yam_mesh import Mesh, MeshMechanism, MeshPart, revolve  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
STILL_S = 1.0
TEST_GRIPPERS = None
DIAG = False  # GPU diagnostic copies print MIC_DIAG lines
DEMO_PACE = 1.0  # scales every demo duration except the fingers' close and open
DEMO_WRONG = ""  # wrong-way copies: "onehand" = the left hand turns the stand and lets go; the mic goes in unheld


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
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, FINGER_OPEN as FOPEN

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
        opening = float(np.clip(np.mean(q[finger_ids]) / FOPEN, 0.0, 1.0))
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

ENAMEL = (0.20, 0.21, 0.23)  # the SM58's dark grey enamelled die-cast handle
GRILLE = (0.74, 0.75, 0.77)  # matte silver steel-mesh grille
CLIP_BLACK = (0.07, 0.07, 0.08)
STAND_BLACK = (0.10, 0.10, 0.11)
CHROME = (0.78, 0.79, 0.81)
RUBBER = (0.05, 0.05, 0.05)
TAPE_RED = (0.85, 0.10, 0.10)


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _grip_matrix(a, c):
    a = np.asarray(a, float) / np.linalg.norm(a)
    c = np.asarray(c, float)
    c = c - a * float(c @ a)
    c = c / np.linalg.norm(c)
    return np.column_stack((c, np.cross(a, c), a))


def _hull8(name, pts, color, proxy):
    """A convex hexahedron from 8 points ordered as a box (0-3 one face ring, 4-7 the opposite ring, same order)."""
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7),
             (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    pts = np.asarray(pts, float)
    m = Mesh(name, pts, faces, collision="convex", color=color, proxy=proxy) if _vol(pts, faces) > 0 else \
        Mesh(name, pts, [f[::-1] for f in faces], collision="convex", color=color, proxy=proxy)
    return m


def _vol(p, faces):
    return sum(float(np.dot(p[a], np.cross(p[b], p[c]))) for a, b, c in faces) / 6.0


# ---------------------------------------------------------------- the Shure SM58
# Mic frame: x along the axis from the centre of the tail (XLR) end toward the grille. Published: 162 mm long, grille
# 51 mm across, shank 23 mm across at the tail end, 298 g. Estimates (ruler check): the handle widens to 26.4 mm at
# 60 mm and 34 mm at 112 mm from the tail, where the grille's base cup (to 44 mm across at 122 mm) begins.
MIC_L = 162.0 * MM
GRILLE_R = 25.5 * MM
GRILLE_C = MIC_L - GRILLE_R  # 136.5 mm from the tail
MIC_MASS = 0.298
MIC_FRICTION = (0.40, 0.35)  # enamel paint on plastic / rubber (estimate)
HANDLE = ((0.0, 10.2), (1.5, 11.5), (60.0, 13.2), (112.0, 17.0))  # (s mm, r mm): tail chamfer, two tapers
CUP = ((112.0, 17.5), (122.0, 22.0))


def handle_r(s):
    """Handle radius (m) at s metres from the tail end (np.interp on the profile)."""
    return float(np.interp(s / MM, [p[0] for p in HANDLE], [p[1] for p in HANDLE])) * MM


def _lying_tilt():
    """The angle the mic's axis rises toward the grille when it lies on a table: the line touching the grille ball
    and the tail rim."""
    s1, r1 = HANDLE[1][0] * MM, HANDLE[1][1] * MM * math.cos(math.pi / 20)  # 20-sided handle: lowest flat
    d = GRILLE_C - s1
    return math.asin((GRILLE_R - r1) / d)


MIC_TILT = _lying_tilt()  # about 5.9 degrees
R_TILT = _rot_y(-MIC_TILT)  # mic frame -> part frame (x rises toward the grille)


def _mic_mesh(name, profile, samples, color):
    prof = [(0.0, profile[0][0] * MM)] + [(r * MM, s * MM) for s, r in profile] + [(0.0, profile[-1][0] * MM)]
    m = revolve([(r, z) for r, z in prof], samples=samples, name=name)
    pts = (R_TILT @ (_rot_y(math.pi / 2) @ m.points.T)).T  # revolve's z -> the mic's x, then the lying tilt
    s0, s1 = profile[0][0] * MM, profile[-1][0] * MM
    rmax = max(r for _, r in profile) * MM
    centre = R_TILT @ np.array([(s0 + s1) / 2, 0.0, 0.0])
    proxy = (cyl(tuple(centre), rmax, s1 - s0, axis="x"),)
    return Mesh(name, pts, m.faces, collision="convex", color=color, proxy=proxy)


def mic_point(s):
    """A point on the mic's axis s metres from the tail end, in the part frame."""
    return R_TILT @ np.array([s, 0.0, 0.0])


def make_mic(pos, yaw):
    meshes = (_mic_mesh("handle_a", HANDLE[:3], 20, ENAMEL), _mic_mesh("handle_b", HANDLE[2:], 20, ENAMEL),
              _mic_mesh("cup", CUP, 24, GRILLE))
    grille = ball(tuple(mic_point(GRILLE_C)), GRILLE_R, color=GRILLE)
    return MeshPart("mic", pos, (grille,), mass=MIC_MASS, color=GRILLE, yaw=yaw, friction=MIC_FRICTION,
                    contact_offset=0.0015, grip=2, meshes=meshes)


def _mic_low_z():
    lows = [GRILLE_C * math.sin(MIC_TILT) - GRILLE_R]
    for prof, n in ((HANDLE[:3], 20), (HANDLE[2:], 20), (CUP, 24)):
        m = _mic_mesh("t", prof, n, ENAMEL)
        lows.append(float(m.points[:, 2].min()))
    return min(lows)


MIC_LOW = _mic_low_z()  # the part origin (tail centre) this far above the table when the mic lies on it


def _mic_com():
    """Uniform-density centre of mass along the mic's axis (s, metres), as PhysX derives it from the colliders."""
    tot, mom = 0.0, 0.0
    for prof in (HANDLE[:3], HANDLE[2:], CUP):
        for (s0, r0), (s1, r1) in zip(prof, prof[1:]):
            s0, r0, s1, r1 = s0 * MM, r0 * MM, s1 * MM, r1 * MM
            v = math.pi * (s1 - s0) / 3 * (r0 * r0 + r0 * r1 + r1 * r1)
            c = s0 + (s1 - s0) * (r0 * r0 + 2 * r0 * r1 + 3 * r1 * r1) / (4 * (r0 * r0 + r0 * r1 + r1 * r1))
            tot, mom = tot + v, mom + v * c
    vb = 4 / 3 * math.pi * GRILLE_R ** 3
    return (mom + vb * GRILLE_C) / (tot + vb)


MIC_COM = _mic_com()


def mic_frame(st):
    """(tail-end centre, unit axis toward the grille) in world from a mic state."""
    o = np.asarray(st.origin, float)
    r = st.rotation()
    return o, r @ R_TILT @ np.array([1.0, 0.0, 0.0])


# ---------------------------------------------------------------- the stand and the A25D clip
# Stand frame: origin on the table under the stand's shaft, x = the clip's cradle axis toward the clip's open front
# end (where the grille will point), z up. On-Stage DS7425 (published: height 4.3-6.7 in, 7.5 in tripod base, rubber
# feet, 12.8 oz listed); set at 6.0 in (152 mm) to the top of the shaft. Tube, clutch and leg sizes are estimates.
STAND_MASS = 0.328  # the stand's legs, hub, tube and clutch (363 g listed, less its inner shaft)
HEAD_MASS = 0.035 + 0.006 + 0.014  # the inner shaft, the A25D's threaded boss and the A25D's body
CLUTCH_STATIC, CLUTCH_DYNAMIC = 40.0, 30.0  # N: the locked height clutch (estimate)
STAND_FRICTION = (0.80, 0.70)  # rubber feet on the table; the chrome tube shares it (estimate)
FOOT_R = 0.090  # the three rubber feet on a 180 mm circle (7.5 in = 190 mm across the legs' tips)
LEG_DEG = (90.0, 210.0, 330.0)
HUB_R, HUB_Z = 0.017, (0.028, 0.058)
TUBE_R, TUBE_Z = 0.0095, (0.058, 0.125)
CLUTCH_R, CLUTCH_Z = 0.014, (0.125, 0.139)
SHAFT_R, SHAFT_Z = 0.008, (0.139, 0.152)
BOSS_R, BOSS_Z = 0.011, (0.152, 0.166)
# The A25D (estimates, ruler check): swivel axis across the cradle 20 mm above the shaft top (its thumb nut tightened
# with the clip level); cradle axis 21 mm above the swivel axis; bore 25.4 mm across at rest (it grips 25-30 mm
# barrels, published), wall 3.2 mm, 36 mm long, open over the top 116 degrees; the front 6 mm flares to 33 mm across
# (lead-in). The SM58's taper meets the bore 43 mm from its tail end, so the jaws start to spread when the cradle's
# centre is 25 mm along the handle (7 mm of handle out behind the clip); the push rises from about 3 N there by about
# 0.7 N per mm further in.
SWIVEL_Z = 0.172
CRADLE_Z = SWIVEL_Z + 0.021
BORE_R, WALL = 0.0127, 0.0032
CRADLE_HALF = 0.018
FLARE_L, FLARE_R = 0.006, 0.0165
JAW_HINGE_Y, JAW_HINGE_Z = 0.005, CRADLE_Z - 0.014
# Probes 2-4: with 0.6 and then 3 N m/rad the clip let the head-heavy mic (0.2 N m about the clip) lever out of its
# open top, then droop 13-19 degrees once let go. The A25D's C is stiff glass-filled plastic: 6 N m/rad at each jaw
# (about 12 N per mm at its tips), 0.5 degree of preload; 36 mm long.
JAW_SPRING = 6.0  # N m/rad (estimate: the slide-in push about 3-6 N; kitchen-scale check)
JAW_PRELOAD_DEG = 0.5  # the jaws rest on their closed stop with this much preload
JAW_OPEN_DEG = 15.0
JAW_MASS, JAW_ARMATURE, JAW_DAMPING = 0.006, 1.0e-4, 0.02
CLIP_FRICTION = (0.35, 0.30)  # glass-filled plastic (estimate)
JAW_ARC = (-70.0, 32.0)  # degrees from the horizontal (y) toward +z, for the +y jaw; mirrored for the -y jaw
SPINE_ARC = (-110.0, -70.0)


def _arc_pieces(prefix, a0, a1, n, side, color):
    """Convex pieces of the cradle wall from angle a0 to a1 (deg, +y side; side=-1 mirrors), stand frame at q=0."""
    out = []
    edges = np.linspace(a0, a1, n + 1)
    for i in range(n):
        th0, th1 = math.radians(edges[i]), math.radians(edges[i + 1])

        def ring(x, rin, rout):
            return [(x, side * rin * math.cos(th0), CRADLE_Z + rin * math.sin(th0)),
                    (x, side * rin * math.cos(th1), CRADLE_Z + rin * math.sin(th1)),
                    (x, side * rout * math.cos(th1), CRADLE_Z + rout * math.sin(th1)),
                    (x, side * rout * math.cos(th0), CRADLE_Z + rout * math.sin(th0))]

        xb, xf = -CRADLE_HALF, CRADLE_HALF - FLARE_L
        main = ring(xb, BORE_R, BORE_R + WALL) + ring(xf, BORE_R, BORE_R + WALL)
        flare = ring(xf, BORE_R, BORE_R + WALL) + ring(CRADLE_HALF, FLARE_R, FLARE_R + WALL)
        th = (th0 + th1) / 2
        rm = BORE_R + WALL / 2
        pc = (0.0, side * rm * math.cos(th), CRADLE_Z + rm * math.sin(th))
        px = (box((pc[0], pc[1], pc[2]), (2 * CRADLE_HALF, WALL, WALL)),)
        out.append(_hull8(f"{prefix}{i}", main, color, px))
        out.append(_hull8(f"{prefix}{i}f", flare, color, ()))
    return out


def _leg(i, deg):
    a = math.radians(deg)
    u, v = np.array([math.cos(a), math.sin(a), 0.0]), np.array([-math.sin(a), math.cos(a), 0.0])
    w0, w1 = 0.0065, 0.005
    r0, r1 = HUB_R - 0.004, FOOT_R
    z0b, z0t, z1b, z1t = 0.030, 0.046, 0.006, 0.014
    pts = [r0 * u - w0 * v + (0, 0, z0b), r0 * u + w0 * v + (0, 0, z0b), r1 * u + w1 * v + (0, 0, z1b),
           r1 * u - w1 * v + (0, 0, z1b), r0 * u - w0 * v + (0, 0, z0t), r0 * u + w0 * v + (0, 0, z0t),
           r1 * u + w1 * v + (0, 0, z1t), r1 * u - w1 * v + (0, 0, z1t)]
    pts = [np.asarray(p, float) for p in pts]
    mid = (r0 + r1) / 2 * u
    return _hull8(f"leg{i}", pts, STAND_BLACK, (box((mid[0], mid[1], 0.026), (0.01, 0.01, 0.02)),))


def make_stand(pos, yaw):
    """The stand mechanism: base = legs, feet, hub, tube and clutch (the articulation root: it carries all three
    joints); link `head` = the inner shaft, the clip's threaded boss, swivel ears and body (spine) on the clutch's
    locked slide; links `jaw_l`, `jaw_r` = the clip's sprung jaws, hinged along the cradle's bottom."""
    base = [
        cyl((0.0, 0.0, sum(HUB_Z) / 2), HUB_R, HUB_Z[1] - HUB_Z[0], color=STAND_BLACK),
        cyl((0.0, 0.0, sum(TUBE_Z) / 2), TUBE_R, TUBE_Z[1] - TUBE_Z[0], color=CHROME),
        cyl((0.0, 0.0, sum(CLUTCH_Z) / 2), CLUTCH_R, CLUTCH_Z[1] - CLUTCH_Z[0], color=STAND_BLACK),
    ]
    for deg in LEG_DEG:
        a = math.radians(deg)
        base.append(cyl((FOOT_R * math.cos(a), FOOT_R * math.sin(a), 0.003), 0.007, 0.006, color=RUBBER))
    legs = tuple(_leg(i, d) for i, d in enumerate(LEG_DEG))
    spine_z0 = CRADLE_Z - BORE_R - WALL
    head = Link("head", (cyl((0.0, 0.0, sum(SHAFT_Z) / 2), SHAFT_R, SHAFT_Z[1] - SHAFT_Z[0], color=CHROME),
                         cyl((0.0, 0.0, sum(BOSS_Z) / 2), BOSS_R, BOSS_Z[1] - BOSS_Z[0], color=CLIP_BLACK),
                         box((0.0, 0.0105, SWIVEL_Z), (0.014, 0.004, 0.016), color=CLIP_BLACK),  # swivel ears
                         box((0.0, -0.0105, SWIVEL_Z), (0.014, 0.004, 0.016), color=CLIP_BLACK),
                         box((0.0, 0.0, (SWIVEL_Z - 0.006 + spine_z0 + 0.002) / 2),
                             (0.012, 0.016, spine_z0 + 0.002 - SWIVEL_Z + 0.006), color=CLIP_BLACK)),
                Slide("z", (0.0, 0.0, SHAFT_Z[0]), -0.040, 0.015, damping=2.0, static_friction=CLUTCH_STATIC,
                      dynamic_friction=CLUTCH_DYNAMIC),
                HEAD_MASS, color=CLIP_BLACK, friction=CLIP_FRICTION)
    jaws = []
    for name, side in (("jaw_l", 1), ("jaw_r", -1)):
        rib = box((0.0, side * (BORE_R + WALL / 2), CRADLE_Z), (2 * CRADLE_HALF - 2 * MM, WALL - 0.4 * MM, 0.004))
        lo, hi = (-JAW_OPEN_DEG, 0.0) if side > 0 else (0.0, JAW_OPEN_DEG)
        jaws.append(Link(name, (rib,), Hinge("x", (0.0, side * JAW_HINGE_Y, JAW_HINGE_Z), lo, hi,
                                             damping=JAW_DAMPING, spring=JAW_SPRING,
                                             rest=side * JAW_PRELOAD_DEG, armature=JAW_ARMATURE),
                         JAW_MASS, color=CLIP_BLACK, friction=CLIP_FRICTION))
    meshes = {"base": legs,
              "head": tuple(_arc_pieces("spine", SPINE_ARC[0], SPINE_ARC[1], 2, 1, CLIP_BLACK)),
              "jaw_l": tuple(_arc_pieces("jl", JAW_ARC[0], JAW_ARC[1], 5, 1, CLIP_BLACK)),
              "jaw_r": tuple(_arc_pieces("jr", JAW_ARC[0], JAW_ARC[1], 5, -1, CLIP_BLACK))}
    return MeshMechanism("stand", pos, tuple(base), (head, *jaws), mass=STAND_MASS, color=STAND_BLACK, yaw=yaw,
                         friction=STAND_FRICTION, contact_offset=0.0015, grip=1, meshes=meshes)


def stand_frame(st):
    """The stand's frame (origin on the table under the tube's axis, z up the tube): the base, the root."""
    return np.asarray(st.origin, float), st.rotation()


def cradle(st):
    """(cradle centre, cradle axis toward the open front end) in world, from the head link's reported pose."""
    rot = st.link_rotation("head")
    centre = np.asarray(st.links["head"][0], float) + rot @ np.array([0.0, 0.0, CRADLE_Z - SHAFT_Z[0]])
    return centre, rot @ np.array([1.0, 0.0, 0.0])


# ---------------------------------------------------------------- layout
STAND_XY = np.array([0.165, 0.000])
AIM_DEG = -60.0  # the clip's axis after the turn (world yaw): toward the speaker's mark
TURN_DEG = -40.0  # the left hand turns the stand by this much (the stand starts at AIM_DEG - TURN_DEG)
STAND_YAW0 = (AIM_DEG - TURN_DEG) * DEG
MARK_D = 0.300
MARK_XY = STAND_XY + MARK_D * np.array([math.cos(AIM_DEG * DEG), math.sin(AIM_DEG * DEG)])
MARK_R = 0.030
MIC_START_DEG = -90.0  # the lying mic's axis (tail -> grille), world yaw: grille to the robot's right
MIC_COM_XY = np.array([0.210, -0.210])


def _mic_start():
    u = np.array([math.cos(MIC_START_DEG * DEG), math.sin(MIC_START_DEG * DEG)])
    com = mic_point(MIC_COM)
    tail_xy = MIC_COM_XY - u * float(com[0])
    return np.array([tail_xy[0], tail_xy[1], TOP - MIC_LOW + 0.3 * MM]), MIC_START_DEG * DEG


def make_mark():
    return Part("speaker_mark", (MARK_XY[0], MARK_XY[1], TOP + 0.0005), [cyl((0, 0, 0), MARK_R, 0.001)], mass=None,
                color=TAPE_RED, friction=(0.6, 0.5))


STAND = make_stand((STAND_XY[0], STAND_XY[1], TOP + 0.2 * MM), STAND_YAW0)
MIC = make_mic(*_mic_start())
OBJECTS = [make_mark(), STAND, MIC]


# ---------------------------------------------------------------- scoring
IN_AXIS = 5.0 * MM  # the cradle's centre within this of the mic's axis: the handle is in the clip
IN_ANGLE = 18.0  # degrees between the mic's axis and the cradle's axis (beyond: levered out of the C)
SEAT_TAIL = 8.0 * MM  # this much handle out past the clip's back face: snug (the jaws spread on the taper)
AIM_TIERS = ((0.030, 1.0), (0.060, 0.75), (0.100, 0.5))
SHIFT_TIERS = ((0.020, 1.0), (0.050, 0.75))
PITCH_OK = (-12.0, 12.0)  # degrees the mic's axis may rise toward the grille (the clip is set level)
TIP_DEG = 20.0
CLEAR = 0.030
STILL_MOVE = 1.0 * MM
SPEED = 0.01


def mic_terms(objects):
    mic, stand = objects["mic"], objects["stand"]
    tail, a = mic_frame(mic)
    c, u = cradle(stand)
    so, sr = stand_frame(stand)
    s_c = float((c - tail) @ a)  # the cradle's centre along the mic from its tail end
    off = float(np.linalg.norm((c - tail) - a * s_c))
    ang = math.degrees(math.acos(float(np.clip(abs(a @ u), -1.0, 1.0))))
    in_clip = bool(off <= IN_AXIS and ang <= IN_ANGLE and CRADLE_HALF <= s_c <= HANDLE[-1][0] * MM - CRADLE_HALF)
    tail_out = s_c - CRADLE_HALF
    a2 = np.array([a[0], a[1]])
    n = float(np.linalg.norm(a2))
    if n < 1e-6:
        miss = math.inf
    else:
        a2 = a2 / n
        d = MARK_XY - tail[:2]
        t = float(d @ a2)
        miss = float(np.linalg.norm(d - a2 * t)) if t > 0 else float(np.linalg.norm(d))
    pitch = math.degrees(math.asin(float(np.clip(a[2], -1.0, 1.0))))
    tilt = math.degrees(math.acos(float(np.clip(sr[2, 2], -1.0, 1.0))))
    shift = float(np.linalg.norm(so[:2] - STAND_XY))
    low = float(np.asarray(mic.low, float)[2])
    return {"tail": tail, "axis": a, "cradle": c, "s_c": s_c, "off": off, "ang": ang, "in_clip": in_clip,
            "tail_out": tail_out, "miss": miss, "pitch": pitch, "tilt": tilt, "shift": shift, "low": low}


def _box_clear(state, clear):
    hands = grippers()
    if hands is None:
        return True
    low, high = np.asarray(state.low, float), np.asarray(state.high, float)
    return all(float(np.linalg.norm(np.maximum(np.maximum(low - tcp, tcp - high), 0.0))) >= clear
               for tcp, _ in hands.values())


def _tube_clear(stand, clear):
    hands = grippers()
    if hands is None:
        return True
    so, sr = stand_frame(stand)
    ax = sr[:, 2]
    for tcp, _ in hands.values():
        d = np.asarray(tcp, float) - so
        h = float(d @ ax)
        if -0.01 <= h <= BOSS_Z[1] + 0.01 and float(np.linalg.norm(d - ax * h)) < clear + 0.015:
            return False
    return True


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
    return all(np.linalg.norm(now[0] - old[0]) <= STILL_MOVE and _rot_deg(now[1], old[1]) <= 2.0 for _, old in hist)


def score_terms(objects):
    t = mic_terms(objects)
    t["released"] = _box_clear(objects["mic"], CLEAR) and _tube_clear(objects["stand"], CLEAR)
    t["still"] = still(objects["mic"], "mic")
    t["tipped"] = ever("the stand tipped over", t["tilt"] > TIP_DEG)
    t["dropped"] = ever("the mic left the table", t["low"] < TOP - 0.02)
    t["held"] = bool(t["in_clip"] and t["low"] > TOP + 0.003)  # in the clip, not resting on the table
    if DIAG:
        _diag(objects, t)
    return t


def score(objects):
    t = score_terms(objects)
    if t["tipped"] or t["dropped"] or not t["held"] or not t["released"] or not t["still"]:
        return 0.0
    seat = 1.0 if t["tail_out"] >= SEAT_TAIL else 0.5
    aim = next((v for lim, v in AIM_TIERS if t["miss"] <= lim), 0.25)
    shift = next((v for lim, v in SHIFT_TIERS if t["shift"] <= lim), 0.5)
    pitch = 1.0 if PITCH_OK[0] <= t["pitch"] <= PITCH_OK[1] else 0.5
    return min(seat, aim, shift, pitch)


def success(objects):
    return score(objects) >= 1.0


# ---------------------------------------------------------------- diagnostics (GPU copies only)
_DIAG = {"step": -1}


def _diag(objects, t):
    step = _RUN["step"]
    if _RUN["t"] is None or step == _DIAG["step"] or step % 5:
        return
    _DIAG["step"] = step
    hands = grippers() or {}
    tcp = " ".join(f"{a[0]}=({','.join(f'{v * 1000:.1f}' for v in hands[a][0])};{hands[a][1]:.2f})" for a in hands)
    st = objects["stand"]
    j = st.joints
    so, sr = stand_frame(st)
    yaw = math.degrees(math.atan2(sr[1, 0], sr[0, 0]))
    tl = t["tail"]
    print(f"MIC_DIAG t={(_RUN['t'] or 0.0):.2f} tail=({tl[0] * 1000:.1f},{tl[1] * 1000:.1f},{(tl[2] - TOP) * 1000:.1f}) "
          f"s_c={t['s_c'] * 1000:.1f} off={t['off'] * 1000:.1f} ang={t['ang']:.1f} in={int(t['in_clip'])} "
          f"miss={min(t['miss'], 9.99) * 1000:.0f} pitch={t['pitch']:.1f} stand=({so[0] * 1000:.1f},{so[1] * 1000:.1f}"
          f",{(so[2] - TOP) * 1000:.1f}) yaw={yaw:.1f} tilt={t['tilt']:.1f} hd={j['head'] * 1000:.1f} "
          f"jl={math.degrees(j['jaw_l']):.1f} jr={math.degrees(j['jaw_r']):.1f} rel={int(t['released'])} "
          f"st={int(t['still'])} {tcp}", flush=True)


# ---------------------------------------------------------------- demo
# The left hand, level and pointing at the stand's tube, pinches the tube under the clutch, lifts the stand 6 mm,
# turns it about the tube until the clip points at the mark, sets it down and holds it. Meanwhile the right hand,
# pointing straight down with its fingers closing across the lying mic, pinches the handle just behind the balance
# point (the pads' front edges at the balance point, where the taper is widest under them), lifts the mic and waits.
# When the stand is set down, the right hand carries the mic so its axis is on the clip's axis, its tail end in front
# of the clip's flared front end, puts the tail into the flare and slides the mic tail-first through the clip until the
# handle's taper wedges in the jaws; it lets go and lifts away, then the left hand lets go. The demo reads poses
# (scripted demos may read simulator state); it never moves an object.
L_GRIP_Z = TOP + 0.090  # the left pinch on the tube (pads about 73-107 mm up: under the clutch, over the hub)
L_PITCH = 15.0 * DEG  # the left hand points forward over the bench, dipped this far below level
L_BACK = 0.070  # the left hand's stand-off before it goes in round the tube
L_LIFT = 0.018  # commanded: probe 2's 6 mm never lifted the stand (the arm sags about 6 mm under its 0.4 kg)
TUBE_PINCH = -(TUBE_R - 3.0 * MM)  # 3 mm inside per finger: about 15 N of squeeze on the rigid tube
L_OPEN = FINGER_OPEN  # fully open (94 mm between the pads) round the tube
R_OPEN = -0.030
R_TILT_DEG = 20.0  # the right hand leans this far from straight down, fingertips toward the mic's tail (reach)
R_TCP_DZ = 0.005  # the TCP this far above the mic's axis at the grip (the pads' low corner clears the table)


def _taper_deg():
    (s0, r0), (s1, r1) = HANDLE[2], HANDLE[3]
    return math.degrees(math.atan((r1 - r0) / (s1 - s0)))


TAPER_DEG = _taper_deg()  # the front taper's half-angle: one pad is turned this much to lie along it (line contact)


def r_grip_matrix(axis):
    """The right hand's grip on the mic whose axis (unit, toward the grille) is `axis`: closing across it (turned by
    the taper's half-angle so one pad lies along the handle), approach leaning R_TILT_DEG toward the tail."""
    ah = np.array([axis[0], axis[1], 0.0])
    ah /= np.linalg.norm(ah)
    c0 = np.array([-ah[1], ah[0], 0.0])
    if c0[0] < 0:  # this side keeps the wrist in range for the mic headings used here
        c0 = -c0
    t = math.radians(TAPER_DEG)
    c = c0 * math.cos(t) - ah * math.sin(t) * (1.0 if c0 @ np.cross([0, 0, 1.0], ah) > 0 else -1.0)
    d = math.radians(R_TILT_DEG)
    a = np.array([0.0, 0.0, -math.cos(d)]) - ah * math.sin(d)
    return _grip_matrix(a, c)


# The TCP's foot this far along the mic. Probe 1 (pads' front ends at the balance point, 2 mm squeeze): the fingers
# stopped on the handle where it is 31 mm across, about 10 mm in front of the TCP's foot, so the head-heavy mic turned
# grille-down about the pinch and slipped out. Now the foot is 14 mm behind the balance point (the pads reach 20 mm
# in front of it) and the squeeze is 5 mm per finger (about 25 N): more friction to hold the mic level.
R_GRIP_S = MIC_COM - 0.014
R_PINCH = -(handle_r(MIC_COM - 0.004) - 5.0 * MM)
SLIDE_START = 0.025  # the tail end this far in front of the cradle's centre before going in (front face at 18)
SLIDE_IN = 0.008  # first step: the tail end this far in front of the centre (inside the flare)
SEAT_S = 0.032  # target: the cradle's centre this far along the mic from the tail (14 mm out past the back face)
SLIDE_LEAD = 0.015  # the hand is led this much further than the target (probe 2: the arm lagged 14 mm in the slide)
SLIDE_STEPS, SLIDE_STEP_S = 6, 0.9
READ_MAX = 0.015
CARRY_Z = TOP + 0.220
R_WAIT = np.array([0.235, -0.150, TOP + 0.215])
PARK = {"left": np.array([0.140, 0.240, TOP + 0.200]), "right": np.array([0.140, -0.240, TOP + 0.200])}
FINAL_HOLD_S = 3.0


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
PARK_M = {"left": _grip_matrix((0.0, 0.0, -1.0), (0.71, -0.71, 0.0)),  # straight down; wrists in range
          "right": _grip_matrix((0.0, 0.0, -1.0), (0.95, 0.31, 0.0))}


def _state_now(name):
    env = _RUN["env"]
    if env is None:
        return None
    try:
        from yam_arena import scene as arena_scene
        return arena_scene.object_states(env, [name])[name]
    except Exception as e:  # noqa: BLE001
        print(f"MIC_READ {name} failed: {e!r}", flush=True)
        return None


_ARMS = {}


def _arm_state(arm):
    s = _ARMS.get(arm)
    if s is None or s["episode"] != _RUN["episode"]:
        s = {"episode": _RUN["episode"], "cache": {}}
        _ARMS[arm] = s
    return s


def _rot_between(a, b):
    """The smallest rotation taking unit vector a onto unit vector b."""
    a, b = np.asarray(a, float) / np.linalg.norm(a), np.asarray(b, float) / np.linalg.norm(b)
    v, c = np.cross(a, b), float(a @ b)
    if np.linalg.norm(v) < 1e-12:
        return np.eye(3)
    k = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return np.eye(3) + k + k @ k / (1.0 + c)


def _clamp(v, m):
    n = float(np.linalg.norm(v))
    return v * (m / n) if n > m else v


def _planned_stand(turned):
    """Planned stand origin and world rotation (before or after the turn)."""
    yaw = AIM_DEG * DEG if turned else STAND_YAW0
    return np.array([STAND_XY[0], STAND_XY[1], TOP]), _rot_z(yaw)


def _planned_cradle(turned):
    o, r = _planned_stand(turned)
    return o + r @ np.array([0.0, 0.0, CRADLE_Z]), r[:, 0]


def _read_stand():
    st = _state_now("stand")
    if st is None:
        return None
    o, r = stand_frame(st)
    c, u = cradle(st)
    return o, r, c, u


def _left_a(yaw_world):
    """The left hand's approach (level, dipped by L_PITCH) and closing directions for a grip turned to yaw_world."""
    a = np.array([math.cos(yaw_world) * math.cos(L_PITCH), math.sin(yaw_world) * math.cos(L_PITCH), -math.sin(L_PITCH)])
    c = np.array([math.sin(yaw_world), -math.cos(yaw_world), 0.0])  # this sign keeps the wrist in range
    return a, c


L_AZ_MID = -32.0 * DEG  # the left hand points forward-right and turns from about -12 to -52 degrees (world yaw)


class Live(Waypoint):
    """A waypoint worked out when first played: fn(S, prev) -> (TCP position, grip matrix)."""

    def __init__(self, arm, index, duration, finger, label, fn, keep=False):
        self.arm, self.index, self.fn = arm, index, fn
        self.duration = duration * (1.0 if keep else DEMO_PACE)
        self.finger, self.label = finger, label

    def _pose(self):
        S = _arm_state(self.arm)
        if self.index not in S["cache"]:
            prev = S["cache"].get(self.index - 1)
            pos, m = self.fn(S, None if prev is None else prev)
            S["cache"][self.index] = (np.asarray(pos, float), np.asarray(m, float))
        return S["cache"][self.index]

    @property
    def pos(self):
        return tuple(float(v) for v in self._pose()[0])

    @property
    def yaw(self):
        m = self._pose()[1]
        c = m[:, 0]
        return Grip(math.atan2(-float(c[0]), float(c[1])), m)


def _left_plan(hold_s):
    def grip_at(S, turn):
        a, c = _left_a(L_AZ0 + turn)
        return a, c

    def tube_tcp(S, dz=0.0):
        return np.array([S["xy"][0], S["xy"][1], L_GRIP_Z + dz])

    def first(S, p):
        rd = _read_stand()
        if rd is None:
            S["xy"], S["turn"] = STAND_XY.copy(), TURN_DEG * DEG
        else:
            o, r, _, _ = rd
            S["xy"] = _clamp(o[:2] - STAND_XY, 0.02) + STAND_XY
            yaw = math.atan2(r[1, 0], r[0, 0])
            S["turn"] = (AIM_DEG * DEG - yaw + math.pi) % (2 * math.pi) - math.pi
            print(f"MIC_READ stand start t={(_RUN['t'] or 0.0):.2f} xy=({o[0] * 1000:.1f},{o[1] * 1000:.1f}) "
                  f"yaw={math.degrees(yaw):.1f} turn={math.degrees(S['turn']):.1f}", flush=True)
        S["az0"] = L_AZ_MID - S["turn"] / 2
        a, c = _left_a(S["az0"])
        return tube_tcp(S) - a * (L_BACK + 0.030) + np.array([0.0, 0.0, 0.030]), _grip_matrix(a, c)

    def at(dz, frac, back=0.0):
        def fn(S, p):
            a, c = _left_a(S["az0"] + frac * S["turn"])
            return tube_tcp(S, dz) - a * back, _grip_matrix(a, c)
        return fn

    def check(S, p):
        rd = _read_stand()
        if rd is not None:
            o, r, _, _ = rd
            yaw = math.atan2(r[1, 0], r[0, 0])
            err = (AIM_DEG * DEG - yaw + math.pi) % (2 * math.pi) - math.pi
            print(f"MIC_READ stand turned t={(_RUN['t'] or 0.0):.2f} xy=({o[0] * 1000:.1f},{o[1] * 1000:.1f}) "
                  f"yaw={math.degrees(yaw):.1f} err={math.degrees(err):.1f}", flush=True)
        return at(-0.001, 1.0)(S, p)

    plan = [("reach toward the stand's tube", 2.5, L_OPEN, first, False),
            ("level with the tube, behind it", 1.0, L_OPEN, at(0.0, 0.0, L_BACK), False),
            ("fingers round the tube", 1.2, L_OPEN, at(0.0, 0.0), False),
            ("pinch the tube", 1.0, TUBE_PINCH, at(0.0, 0.0), True),
            ("lift the stand 6 mm", 1.0, TUBE_PINCH, at(L_LIFT, 0.0), False)]
    for k in range(4):
        plan.append((f"turn the stand toward the mark ({k + 1}/4)", 0.9, TUBE_PINCH, at(L_LIFT, (k + 1) / 4), False))
    plan += [("set it down", 1.0, TUBE_PINCH, at(-0.001, 1.0), False),
             ("hold the stand: read its turn", 0.5, TUBE_PINCH, check, False)]
    if DEMO_WRONG == "onehand":
        plan += [("let go of the stand", 0.8, L_OPEN, at(-0.001, 1.0), True),
                 ("back off", 1.2, L_OPEN, at(-0.001, 1.0, L_BACK), False),
                 ("park", 2.0, L_OPEN, lambda S, p: (PARK["left"], PARK_M["left"]), False),
                 ("stay parked", hold_s, L_OPEN, lambda S, p: (PARK["left"], PARK_M["left"]), True)]
    else:
        plan += [("hold the stand while the mic goes in", hold_s, TUBE_PINCH, at(-0.001, 1.0), True),
                 ("let go of the stand", 0.8, L_OPEN, at(-0.001, 1.0), True),
                 ("back off", 1.2, L_OPEN, at(-0.001, 1.0, L_BACK), False),
                 ("park", 2.0, L_OPEN, lambda S, p: (PARK["left"], PARK_M["left"]), False)]
    plan.append(("hold clear", FINAL_HOLD_S, L_OPEN, lambda S, p: (PARK["left"], PARK_M["left"]), True))
    return plan


def _mic_read():
    st = _state_now("mic")
    if st is None:
        return None
    return mic_frame(st)


def _right_plan(wait_s):

    def start_frame():
        p, yaw = _mic_start()
        r = _rot_z(yaw)
        return p, r @ R_TILT @ np.array([1.0, 0.0, 0.0])

    def pick_point(S):
        tail, a = S["mic0"]
        return tail + a * R_GRIP_S + np.array([0.0, 0.0, R_TCP_DZ])

    def first(S, p):
        tail, a = start_frame()
        S["mic0"] = (tail, a)
        q = pick_point(S)
        return q + np.array([0.0, 0.0, 0.060]), r_grip_matrix(a)

    def over(S, p):
        rd = _mic_read()
        if rd is not None:
            tail, a = rd
            tail0, _ = start_frame()
            S["mic0"] = (tail0 + _clamp(tail - tail0, 0.025), a)
            print(f"MIC_READ mic start t={(_RUN['t'] or 0.0):.2f} shift_mm=({(tail - tail0)[0] * 1000:.1f},"
                  f"{(tail - tail0)[1] * 1000:.1f},{(tail - tail0)[2] * 1000:.1f}) yaw="
                  f"{math.degrees(math.atan2(a[1], a[0])):.1f} pitch={math.degrees(math.asin(a[2])):.1f}", flush=True)
        q = pick_point(S)
        return q + np.array([0.0, 0.0, 0.040]), r_grip_matrix(S["mic0"][1])

    def at_pick(S, p):
        return pick_point(S), r_grip_matrix(S["mic0"][1])

    def lift(S, p):
        q = pick_point(S)
        return np.array([q[0], q[1], CARRY_Z]), r_grip_matrix(S["mic0"][1])

    def wait(S, p):  # turn the mic to the clip's planned heading while waiting
        return R_WAIT, r_grip_matrix(_planned_cradle(True)[1])

    def hand_in_mic(S, tcp):
        """Where the TCP is in the mic's frame (s along, d across) from a read: the grip offset."""
        rd = _mic_read()
        if rd is None:
            return None
        tail, a = rd
        return tail, a

    def slide_tcp(S, tail_u):
        """TCP for the mic's tail end at tail_u along the cradle axis from its centre (the read grip offset, held in
        the hand's frame, applied)."""
        c, u = S["cr"]
        m = slide_m(S)
        return c + u * tail_u - m @ S["off_h"], m

    def slide_m(S):
        """The hand's frame for the slide: the grip matrix for the clip's axis, turned so the mic's axis as held (read
        in the hand frame) lies along the clip's axis (the mic is held tilted as it lay, 6 degrees grille-up)."""
        u = S["cr"][1]
        m0 = r_grip_matrix(u)
        return _rot_between(m0 @ S["ax_h"], u) @ m0

    def to_front(S, p):
        rd = _read_stand()
        if rd is None:
            S["cr"] = _planned_cradle(True)
        else:
            S["cr"] = (rd[2], rd[3])
        # the mic in the hand: offset from the tail end to the TCP, from a read now (the hand is still at R_WAIT)
        m = _mic_read()
        tcp_now, m_now = p[0], p[1]
        if m is None:  # offline: the tail end's offset in the hand's frame as it was gripped
            tail0, a0 = S["mic0"]
            S["off_h"] = r_grip_matrix(a0).T @ (-(a0 * R_GRIP_S + np.array([0.0, 0.0, R_TCP_DZ])))
            S["ax_h"] = r_grip_matrix(a0).T @ a0
        else:
            tail, a = m
            S["off_h"] = m_now.T @ (tail - tcp_now)
            S["ax_h"] = m_now.T @ a
        S["corr"] = np.zeros(3)
        c, u = S["cr"]
        print(f"MIC_READ carry t={(_RUN['t'] or 0.0):.2f} cradle=({c[0] * 1000:.1f},{c[1] * 1000:.1f},{(c[2] - TOP) * 1000:.1f}"
              f") axis_yaw={math.degrees(math.atan2(u[1], u[0])):.1f} pitch={math.degrees(math.asin(u[2])):.1f} "
              f"off_h=({S['off_h'][0] * 1000:.1f},{S['off_h'][1] * 1000:.1f},{S['off_h'][2] * 1000:.1f})", flush=True)
        q, m2 = slide_tcp(S, SLIDE_START)
        return q + np.array([0.0, 0.0, 0.018]), m2

    def correct(tag, tail_u, use_axial=False, up=0.0):
        def fn(S, p):
            m = _mic_read()
            c, u = S["cr"]
            if m is not None:
                tail, a = m
                goal_prev = p[0] + p[1] @ S["off_h"] - S["corr"]  # where the tail was meant to be before
                if float((c - tail) @ a) > 0.0:
                    # the tail is past the cradle's centre, held by the clip: line up the point under the hand
                    # instead (probe 3: pushing, the arm sagged 7 mm and the mic pivoted grille-down in the clip,
                    # while the tail read pulled the hand further down)
                    goal_prev = goal_prev + (p[1] @ S["ax_h"]) * R_GRIP_S + np.array([0.0, 0.0, up])
                    tail = tail + a * R_GRIP_S
                err = goal_prev - tail
                if not use_axial:
                    err = err - u * float(err @ u)
                S["corr"] = _clamp(S["corr"] + err, READ_MAX)
                print(f"MIC_READ {tag} t={(_RUN['t'] or 0.0):.2f} err_mm=({err[0] * 1000:.2f},{err[1] * 1000:.2f},"
                      f"{err[2] * 1000:.2f}) corr_mm=({S['corr'][0] * 1000:.1f},{S['corr'][1] * 1000:.1f},"
                      f"{S['corr'][2] * 1000:.1f}) s_c={float((c - tail) @ a) * 1000:.1f} "
                      f"ang={math.degrees(math.acos(min(1.0, abs(float(a @ u))))):.1f}", flush=True)
            q, mm = slide_tcp(S, tail_u)
            return q + S["corr"], mm
        return fn

    def away(S, p):  # sideways off the mic's line (toward the right arm's side), keeping the height
        c, u = S["cr"]
        n = np.array([u[1], -u[0], 0.0])
        n /= np.linalg.norm(n)
        return p[0] + n * 0.070, p[1]

    plan = [("reach over the mic", 2.5, R_OPEN, first, False),
            ("over the handle: read the mic", 1.2, R_OPEN, over, False),
            ("fingers down beside the handle", 1.5, R_OPEN, at_pick, False),
            ("pinch the handle at its balance point", 1.0, R_PINCH, at_pick, True),
            ("lift the mic", 1.5, R_PINCH, lift, False),
            ("carry it clear of the turning stand", 1.5, R_PINCH, wait, False),
            ("wait while the stand is turned", wait_s, R_PINCH, wait, True),
            ("carry it in front of the clip", 2.5, R_PINCH, to_front, False),
            ("down onto the clip's axis", 1.2, R_PINCH,
             lambda S, p: (slide_tcp(S, SLIDE_START)[0], slide_tcp(S, SLIDE_START)[1]), False),
            ("line the tail up with the clip", 0.8, R_PINCH, correct("line", SLIDE_START), False),
            ("tail into the clip's flared end", 1.0, R_PINCH, correct("flare", SLIDE_IN), False)]
    for k in range(SLIDE_STEPS):
        tail_u = SLIDE_IN + (-(SEAT_S + SLIDE_LEAD) - SLIDE_IN) * (k + 1) / SLIDE_STEPS
        plan.append((f"slide it tail-first through the clip ({k + 1}/{SLIDE_STEPS})", SLIDE_STEP_S, R_PINCH,
                     correct(f"slide{k + 1}", tail_u), False))
    plan += [("hold it in the jaws", 1.5, R_PINCH, correct("hold", -(SEAT_S + SLIDE_LEAD), up=0.003), False),
             ("let go of the mic", 0.8, R_OPEN, lambda S, p: (p[0], p[1]), True),
             ("lift the hand away", 1.2, R_OPEN, lambda S, p: (p[0] + np.array([0.0, 0.0, 0.040]), p[1]), False),
             ("move out to the side", 1.2, R_OPEN, away, False),
             ("park", 2.0, R_OPEN, lambda S, p: (PARK["right"], PARK_M["right"]), False),
             ("hold clear", FINAL_HOLD_S, R_OPEN, lambda S, p: (PARK["right"], PARK_M["right"]), True)]
    return plan


def _waypoints(arm, plan):
    _ARMS.pop(arm, None)
    return [Live(arm, i, secs, finger, label, fn, keep) for i, (label, secs, finger, fn, keep) in enumerate(plan)]


def demo():
    """Left: take the stand by its tube, lift, turn, set down, hold. Right: pick the mic and wait clear of the
    turning stand, then slide it into the clip while the left holds; the left lets go after the right has left."""
    left_probe = _left_plan(1.0)
    labels = [p[0] for p in left_probe]
    t_set = sum(p[1] for p in left_probe[:labels.index("hold the stand: read its turn") + 1])
    right_probe = _right_plan(1.0)
    rl = [p[0] for p in right_probe]
    t_wait0 = sum(p[1] for p in right_probe[:rl.index("wait while the stand is turned")])
    wait_s = max(0.5, t_set + 0.3 - t_wait0)
    right = _right_plan(wait_s)
    rl = [p[0] for p in right]
    t_away = sum(p[1] for p in right[:rl.index("lift the hand away") + 1])
    hold_s = max(1.0, t_away + 0.5 - t_set)
    return {"right": _waypoints("right", right), "left": _waypoints("left", _left_plan(hold_s))}


# ---------------------------------------------------------------- offline test layouts (states.yaml)
def _quat(rot):
    from yam_arena.ik import matrix_to_quat_xyzw
    return [float(v) for v in matrix_to_quat_xyzw(np.asarray(rot, float))]


def stand_pose(yaw_deg=AIM_DEG, dx=0.0, dy=0.0, tilt_deg=0.0):
    """[origin, quat] of the stand standing at its spot (shifted dx, dy), turned to yaw_deg, leaning tilt_deg."""
    r = _rot_z(yaw_deg * DEG) @ _rot_x(tilt_deg * DEG)
    o = np.array([STAND_XY[0] + dx, STAND_XY[1] + dy, TOP + 0.2 * MM])
    return [float(v) for v in o], _quat(r)


def jaw_open_deg(s_c):
    """Each jaw's opening (degrees) with the handle's cradle centre s_c metres from the tail."""
    r = handle_r(s_c + CRADLE_HALF)
    return float(np.clip((r - BORE_R) / (CRADLE_Z - JAW_HINGE_Z) / DEG, 0.0, JAW_OPEN_DEG)) if r > BORE_R else 0.0


def mic_seated(s_c, yaw_deg=AIM_DEG, dx=0.0, dy=0.0, droop_deg=0.0, reverse=False, lift_mm=0.0):
    """[origin, quat] of the mic in the clip of a stand at stand_pose(yaw_deg, dx, dy), the cradle's centre s_c metres
    from the tail; reverse = grille toward the clip's back."""
    o = np.array([STAND_XY[0] + dx, STAND_XY[1] + dy, TOP + 0.2 * MM])
    r = _rot_z(yaw_deg * DEG)
    c = o + r @ np.array([0.0, 0.0, CRADLE_Z])
    rd = r @ _rot_y(droop_deg * DEG)  # the mic drooping in the jaws, front end down, about the cradle's centre
    a = -rd[:, 0] if reverse else rd[:, 0]
    tail = c - a * s_c + np.array([0.0, 0.0, lift_mm * MM])
    yaw, pitch = math.atan2(a[1], a[0]), math.asin(float(np.clip(a[2], -1, 1)))
    rot = _rot_z(yaw) @ _rot_y(-pitch) @ R_TILT.T
    return [float(v) for v in tail], _quat(rot)


def mic_lying(dx=0.0, dy=0.0):
    p, yaw = _mic_start()
    return [float(p[0] + dx), float(p[1] + dy), float(p[2])], _quat(_rot_z(yaw))
