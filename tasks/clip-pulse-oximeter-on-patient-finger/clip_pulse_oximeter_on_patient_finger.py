"""A patient's hand at the bedside: an adult-size task-trainer right hand lies palm down on a 20-degree foam
arm-elevation wedge, fingers pointing up the slope toward the right arm and slightly spread, the index,
middle and ring fingertips reaching past the wedge's high edge. A fingertip pulse oximeter (58 x 32 x 37 mm,
50 g) lies display up on the slope beside the little finger, its back end overhanging the high edge. The
oximeter is a spring clip: squeezing its back end (the ridged top tab against the underside) opens the jaws;
released, the spring closes them. The trainer hand is light and loose on the wedge. Zero object jitter; 20 Hz.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, ball, box
from yam_joints import Hinge, Link
from yam_mesh import Mesh, MeshMechanism, MeshPart
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
def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array(((1, 0, 0), (0, c, -s), (0, s, c)))


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array(((c, 0, s), (0, 1, 0), (-s, 0, c)))


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array(((c, -s, 0), (s, c, 0), (0, 0, 1)))


def _prism_mesh(name, profile_yz, x0, x1, R=np.eye(3), offset=(0, 0, 0), color=None, proxy=None):
    """Closed convex prism: a convex (y, z) polygon extruded from x0 to x1, then rotated by R and shifted."""
    prof = [tuple(map(float, p)) for p in profile_yz]
    n = len(prof)
    pts = np.array([(x, y, z) for x in (x0, x1) for (y, z) in prof], float)
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
    pts = pts @ np.asarray(R, float).T + np.asarray(offset, float)
    if proxy is None:
        proxy = (box((pts.min(0) + pts.max(0)) / 2, np.ptp(pts, axis=0)),)
    return Mesh(name, pts, faces, collision="convex", color=color, proxy=proxy)


def _ball_tiling(lo, hi, R):
    """Offline proxy: spheres inscribed in a box (exact under any rotation, unlike a rotated bounding box)."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    r = float(np.min(hi - lo)) / 2
    axes = [np.linspace(lo[i] + r, hi[i] - r, max(1, int(math.ceil((hi[i] - lo[i] - 2 * r) / (1.6 * r))) + 1))
            for i in range(3)]
    return tuple(ball(R @ np.array((x, y, z)), r) for x in axes[0] for y in axes[1] for z in axes[2])


def _box_mesh(name, lo, hi, R, color=None, proxy=()):
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    return _prism_mesh(name, ((y0, z0), (y1, z0), (y1, z1), (y0, z1)), x0, x1, R, color=color,
                       proxy=_ball_tiling(lo, hi, R) if proxy is None else proxy)


# ------------------------------------------------------------------ layout (metres, world frame)
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

SLOPE_DEG = 20.0                                   # arm-elevation wedge: rises toward -y (toward the fingertips)
_A = math.radians(SLOPE_DEG)
R_SLOPE = _rx(-_A)                                 # hand/wedge-surface frame -> world
U_TIP = R_SLOPE @ np.array([0, -1.0, 0])           # along the fingers, toward the tips (up the slope)
N_UP = R_SLOPE @ np.array([0, 0, 1.0])             # wedge surface normal

# Hand frame: x = world x, y along the fingers toward the wrist (down the slope), z = surface normal; origin
# on the (extended) wedge surface plane under the index fingertip apex.
HAND_X, HAND_Y = 0.270, -0.075
OVERHANG = 0.050                                   # fingertips reach 50 mm past the wedge's high edge
WEDGE_LEN = 0.180                                  # sloped length (hand-frame y from 0.050 to 0.230)
WEDGE_LOW_Z = 0.005                                # thickness at the low end
WEDGE_X = (0.085, 0.335)
_EDGE_LOCAL_Y, _LOW_LOCAL_Y = OVERHANG, OVERHANG + WEDGE_LEN
HAND_Z = TOP + WEDGE_LOW_Z + _LOW_LOCAL_Y * math.sin(_A)
HAND_POS = (HAND_X, HAND_Y, HAND_Z)
EDGE = np.array(HAND_POS) + R_SLOPE @ np.array([0, _EDGE_LOCAL_Y, 0])
LOW = np.array(HAND_POS) + R_SLOPE @ np.array([0, _LOW_LOCAL_Y, 0])


def hand_to_world(p_local):
    return np.array(HAND_POS) + R_SLOPE @ np.asarray(p_local, float)


# Wedge (fixed): convex prism; profile in world (y, z) relative to its origin on the table under the low end.
WEDGE_POS = (sum(WEDGE_X) / 2, float(LOW[1]), TOP)
_DY, _H = float(EDGE[1] - LOW[1]), float(EDGE[2] - TOP)
# Offline proxy only: an inscribed staircase 7 mm under the slope, because the checker bounds each sphere of
# the hand by an axis-aligned box whose lower corner dips up to 6 mm below a 20-degree surface.
_WEDGE_STEPS, _PROXY_DROP = 12, 0.007
_wedge_proxy = tuple(
    box((0.0, _DY * (k + 0.5) / _WEDGE_STEPS, h / 2), (WEDGE_X[1] - WEDGE_X[0], abs(_DY) / _WEDGE_STEPS, h))
    for k in range(_WEDGE_STEPS)
    for h in [max(0.002, WEDGE_LOW_Z + (_H - WEDGE_LOW_Z) * k / _WEDGE_STEPS - _PROXY_DROP)])
WEDGE = MeshPart("wedge", WEDGE_POS, (), color=(0.55, 0.70, 0.82), friction=(0.6, 0.5), contact_offset=0.001,
                 meshes=(_prism_mesh("foam", ((0.0, 0.0), (0.0, WEDGE_LOW_Z), (_DY, _H), (_DY, 0.0)),
                                     -(WEDGE_X[1] - WEDGE_X[0]) / 2, (WEDGE_X[1] - WEDGE_X[0]) / 2,
                                     proxy=_wedge_proxy),))

# Trainer hand (free): spheres along each finger and over the palm, so it lies exactly on the slope.
HAND_MASS = 0.300
SKIN = (0.86, 0.66, 0.52)
# per finger: tip (x, y), knuckle (x, y), radii distal / middle / proximal, phalanx boundary fractions
FINGERS = {
    "middle": ((-0.036, -0.010), (-0.024, 0.093), (0.0078, 0.0083, 0.0093)),
    "ring": ((-0.068, -0.002), (-0.045, 0.097), (0.0073, 0.0078, 0.0086)),
    "little": ((-0.098, 0.022), (-0.064, 0.102), (0.0065, 0.0068, 0.0076)),
}
PHALANX = (0.28, 0.58)                             # distal ends at 28 %, middle at 58 % of tip-to-knuckle
INDEX = ((0.0, 0.0), (0.0, 0.095))
INDEX_W = (0.018, 0.019, 0.021)                    # index widths, distal / middle / proximal
INDEX_T = (0.015, 0.016, 0.018)                    # index thicknesses


def _hand_balls():
    out = []
    # Index: two rows of spheres per phalanx give an 18 x 15 mm oval fingertip.
    (tx, ty), (kx, ky) = INDEX
    for y in np.arange(ty + INDEX_T[0] / 2, ky + 0.0001, 0.004):
        f = (y - ty) / (ky - ty)
        i = 0 if f < PHALANX[0] else 1 if f < PHALANX[1] else 2
        r = INDEX_T[i] / 2
        for s in (-1, 1):
            out.append(ball(hand_to_world((tx + s * (INDEX_W[i] / 2 - r), y, r)) - HAND_POS, r))
    for name, ((tx, ty), (kx, ky), radii) in FINGERS.items():
        for y in np.arange(ty + radii[0], ky + 0.0001, 0.005):
            f = (y - ty) / (ky - ty)
            r = radii[0 if f < PHALANX[0] else 1 if f < PHALANX[1] else 2]
            out.append(ball(hand_to_world((tx + f * (kx - tx), y, r)) - HAND_POS, r))
    # Thumb, beside the index finger, slightly abducted.
    for f in np.linspace(0.0, 1.0, 14):
        x, y, r = 0.042 + f * (0.016 - 0.042), 0.058 + f * (0.165 - 0.058), 0.0095 + f * 0.0025
        out.append(ball(hand_to_world((x, y, r)) - HAND_POS, r))
    # Palm: 86 mm across the knuckles, 26 mm thick.
    for x in np.linspace(-0.060, 0.000, 5):
        for y in np.linspace(0.097, 0.176, 7):
            out.append(ball(hand_to_world((x, y, 0.013)) - HAND_POS, 0.013))
    out.append(ball(hand_to_world((0.010, 0.150, 0.015)) - HAND_POS, 0.015))      # thenar eminence
    # Wrist and forearm stub (the trainer's cut end): 60 mm wide, 40 mm thick.
    for x in (-0.045, -0.025):
        for y in np.linspace(0.183, 0.225, 6):
            out.append(ball(hand_to_world((x, y, 0.020)) - HAND_POS, 0.020))
    return out


HAND_SHAPES = _hand_balls()
STUB_L = np.array([-0.035, 0.2082, 0.020])         # where the steadying hand pinches the forearm stub
STUB_W = 0.060
_STUB_GRIP = int(np.argmin([np.linalg.norm(np.asarray(b.center) - (hand_to_world(STUB_L + [-0.010, 0, 0]) - HAND_POS))
                            for b in HAND_SHAPES]))
HAND = Part("hand", HAND_POS, HAND_SHAPES, mass=HAND_MASS, color=SKIN, friction=(0.5, 0.4),
            contact_offset=0.001, grip=_STUB_GRIP)
INDEX_TIP_L = np.array([0.0, 0.0, 0.0075])          # fingertip apex on the distal axis (hand frame)
INDEX_BACK_L = np.array([0.0, 0.020, 0.0075])       # 20 mm back along the same axis
INDEX_TIP = R_SLOPE @ INDEX_TIP_L                   # the same points in the hand Part's (yaw-only) frame
INDEX_BACK = R_SLOPE @ INDEX_BACK_L

# Pulse oximeter (free MeshMechanism). Clip frame K: origin on the bottom face in the hinge plane; +x toward
# the back end (squeeze tabs), the mouth at -x; +z out of the display. K is the part frame pitched by the
# wedge angle about the hinge axis (y), so the clip lies flat on the slope; the hinge stays on y.
OXI_YAW = -math.pi / 2                             # K -x (mouth) points toward world +y (down the slope)
R_PK = _ry(-_A)                                    # K -> part frame
R_WK = _rz(OXI_YAW) @ R_PK                         # K -> world
L_BACK, L_FRONT, OXI_W = 0.020, 0.038, 0.032        # 58 mm long, 32 mm wide
HINGE_Z = 0.027                                    # top-jaw underside, jaws parallel at q = 0
FLOOR_Z = 0.010                                    # bottom-jaw finger pad
STOP_X = -0.004                                    # finger-stop face
MOUTH_X = -L_FRONT
CRADLE_HALF = 0.012                                # finger axis within 12 mm of the clip's centre line
TROUGH_FLAT = 0.002
TROUGH_RISE = (0.016 - TROUGH_FLAT) * math.tan(math.radians(20.0))
JAW_CLOSED, JAW_OPEN = -20.0, 25.0                 # degrees; + opens the mouth
SPRING_K = 0.104                                   # N m / rad
SPRING_REST = -45.0                                # degrees (preload against the closed stop)
BLUE = (0.10, 0.32, 0.78)
SHELL = (0.52, 0.55, 0.60)                         # grey lower shell
SCREEN = (0.04, 0.05, 0.07)
PAD_GREY = (0.45, 0.47, 0.50)


def _k(p):
    return R_PK @ np.asarray(p, float)


OXI_BASE_MESHES = (
    _box_mesh("lower_shell", (-L_FRONT, -OXI_W / 2, 0.0), (L_BACK, OXI_W / 2, FLOOR_Z), R_PK, SHELL, proxy=None),
    # Silicone finger pad: a shallow trough (flat 4 mm centre strip, sides rising at 20 deg) that centres the finger.
    _prism_mesh("trough_left", ((TROUGH_FLAT, FLOOR_Z), (OXI_W / 2, FLOOR_Z + TROUGH_RISE), (OXI_W / 2, FLOOR_Z)),
                MOUTH_X, STOP_X, R_PK, color=PAD_GREY, proxy=()),
    _prism_mesh("trough_right", ((-TROUGH_FLAT, FLOOR_Z), (-OXI_W / 2, FLOOR_Z), (-OXI_W / 2, FLOOR_Z + TROUGH_RISE)),
                MOUTH_X, STOP_X, R_PK, color=PAD_GREY, proxy=()),
    _box_mesh("finger_stop", (STOP_X, -CRADLE_HALF, FLOOR_Z), (STOP_X + 0.004, CRADLE_HALF, 0.022), R_PK, PAD_GREY),
    _box_mesh("hinge_cheek_l", (-0.004, OXI_W / 2 - 0.003, FLOOR_Z), (0.004, OXI_W / 2, HINGE_Z), R_PK, SHELL),
    _box_mesh("hinge_cheek_r", (-0.004, -OXI_W / 2, FLOOR_Z), (0.004, -OXI_W / 2 + 0.003, HINGE_Z), R_PK, SHELL),
)
OXI_TOP_MESHES = (
    _box_mesh("upper_shell", (-L_FRONT, -OXI_W / 2, HINGE_Z), (L_BACK, OXI_W / 2, 0.0355), R_PK, BLUE),
    _box_mesh("squeeze_ridge", (0.012, -OXI_W / 2, 0.0355), (L_BACK, OXI_W / 2, 0.037), R_PK, BLUE),
    _box_mesh("display", (-0.032, -0.011, 0.0355), (-0.004, 0.011, 0.0365), R_PK, SCREEN),
)
# On the slope beside the little finger; the back end overhangs the high edge by 16 mm (x_K = +0.004 at the edge).
OXI_X = 0.130
_oxi_origin = EDGE - R_WK @ np.array([0.004, 0.0, 0.0])
OXI_START = (OXI_X, float(_oxi_origin[1]), float(_oxi_origin[2]))
OXIMETER = MeshMechanism(
    "oximeter", OXI_START, [ball(_k((0.015, 0, 0.005)), 0.005)],
    [Link("top_jaw", [ball(_k((-0.009, 0, 0.03125)), 0.00425)],
          Hinge("y", tuple(_k((0, 0, HINGE_Z))), JAW_CLOSED, JAW_OPEN, damping=2e-4, spring=SPRING_K,
                rest=SPRING_REST, initial=JAW_CLOSED, armature=1e-5),
          mass=0.018, color=BLUE, friction=(1.0, 0.9))],
    mass=0.032, color=SHELL, yaw=OXI_YAW, friction=(1.0, 0.9), contact_offset=0.001,
    grip=0, meshes={"base": OXI_BASE_MESHES, "top_jaw": OXI_TOP_MESHES})
# Offline proxies: the lower shell is tiled with inscribed spheres; the top jaw carries only one sphere inside
# its shell, because kinematic replay cannot open the jaws (the GPU simulates every mesh collider).
# The offline replay models every grip as a straight-down one; finger contact with these bodies is advisory.
PRESSED_PARTS = ("oximeter", "hand")
OBJECTS = [WEDGE, HAND, OXIMETER]

# Reference layouts for the scorer tests (states.yaml): the clip seated on a fingertip, and flipped over.
OXI_AXIS = R_WK[:, 0]                               # world direction of the clip's back end (+x in K)


def _seated_origin(tip_world, axis_z, R=R_WK):
    return np.asarray(tip_world, float) - R @ np.array([STOP_X, 0.0, FLOOR_Z + axis_z])


GOAL_OXI = _seated_origin(hand_to_world((0.0, 0.0, 0.0075)), 0.0075)
MIDDLE_OXI = _seated_origin(hand_to_world((-0.036, -0.010, 0.0078)), 0.0078)
_R_FLIP = R_WK @ _rx(math.pi)
FLIP_OXI = _seated_origin(hand_to_world((0.0, 0.0, 0.0075)), 0.0075, _R_FLIP)
FLIP_QUAT = tuple(float(v) for v in matrix_to_quat_xyzw(_R_FLIP @ R_PK.T))

# ------------------------------------------------------------------ scoring
SEAT_TOL = 0.005          # fingertip within 5 mm of the finger stop = seated
TIP_PEN = 0.003           # allowed apparent penetration of the stop face
MIN_DEPTH = 0.015         # fingertip at least 15 mm inside the mouth = on the finger
AXIS_Z = (0.012, 0.024)   # finger axis height inside the channel (clip frame K)
JAW_REST = (-10.0, 6.0)   # top-jaw angle resting on the fingertip (fully shut -20, squeezed open above 6)
DISPLAY_TILT = 45.0       # display normal within 45 degrees of vertical (20 nominal on the wedge)
HAND_MARK = 0.010         # hand on its mark: within 10 mm and 5 degrees of where it lay
HAND_MARK_DEG = 5.0
HAND_YANK = 0.025         # sticky zero: hand ever dragged 25 mm, lifted 15 mm or turned 15 degrees
HAND_LIFT = 0.015
HAND_TURN_DEG = 15.0
DROP_Z = TOP - 0.020      # sticky zero: oximeter below the table top (off the table)
STEP_M, STEP_DEG = 0.0005, 0.25
TEST_STICKY = ()          # offline tests: sticky keys that already fired earlier in the episode


def sticky(key, cond):
    return ever(key, bool(cond) or key in TEST_STICKY)


def _angle_deg(r0, r1):
    c = (np.trace(np.asarray(r0).T @ np.asarray(r1)) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(c, -1.0, 1.0))))


def hand_offset(hand):
    d = np.asarray(hand.origin, float) - np.asarray(HAND_POS, float)
    return float(np.linalg.norm(d[:2])), float(d[2]), _angle_deg(np.eye(3), hand.rotation())


def to_clip(oxi, p_world):
    return R_PK.T @ oxi.to_local(p_world)


def fingertip_in_clip(oxi, hand):
    """(depth inside the mouth, distance short of the stop, in channel) for the index finger, in frame K."""
    tip = to_clip(oxi, hand.to_world(INDEX_TIP))
    back = to_clip(oxi, hand.to_world(INDEX_BACK))
    in_channel = all(abs(p[1]) <= CRADLE_HALF and AXIS_Z[0] <= p[2] <= AXIS_Z[1] for p in (tip, back))
    pointing_in = back[0] < tip[0]
    short = float(STOP_X - tip[0])
    return float(tip[0] - MOUTH_X), short, bool(in_channel and pointing_in and short >= -TIP_PEN)


def display_tilt(oxi):
    n = oxi.rotation() @ R_PK[:, 2]
    return math.degrees(math.acos(float(np.clip(n[2], -1.0, 1.0))))


def clip_state(objects):
    oxi, hand = objects["oximeter"], objects["hand"]
    depth, short, in_channel = fingertip_in_clip(oxi, hand)
    jaw = oxi.angle_deg("top_jaw")
    on = (in_channel and depth >= MIN_DEPTH and JAW_REST[0] <= jaw <= JAW_REST[1]
          and display_tilt(oxi) <= DISPLAY_TILT and oxi.low[2] >= TOP + 0.005)
    return bool(on), bool(on and short <= SEAT_TOL), dict(depth=depth, short=short, jaw=jaw)


_POSE_EPISODE = None
_POSE_TOKEN = None
_PREVIOUS = None
_QUIET = False


def at_rest(objects):
    """Pose differences between consecutive control steps (contact chatter makes joint speeds unreliable)."""
    global _POSE_EPISODE, _POSE_TOKEN, _PREVIOUS, _QUIET
    oxi, hand = objects["oximeter"], objects["hand"]
    if _RUN["t"] is None:
        return oxi.resting() and hand.resting() and abs(oxi.joint_vel["top_jaw"]) < 0.05
    if _POSE_EPISODE != _RUN["episode"]:
        _PREVIOUS, _POSE_TOKEN, _QUIET = None, None, False
        _POSE_EPISODE = _RUN["episode"]
    token = (_RUN["step"], float(_RUN["t"]))
    if token != _POSE_TOKEN:
        pose = (np.r_[oxi.to_world(_k((L_BACK, 0, 0))), oxi.to_world(_k((MOUTH_X, 0, 0))),
                      hand.to_world((0, 0, 0)), hand.to_world((0, 0.2, 0))], oxi.angle_deg("top_jaw"))
        _QUIET = False
        if _PREVIOUS is not None:
            step, old = _PREVIOUS
            _QUIET = bool(token[0] == step + 1
                          and np.all(np.abs(pose[0] - old[0]) < STEP_M) and abs(pose[1] - old[1]) < STEP_DEG)
        _PREVIOUS = (token[0], pose)
        _POSE_TOKEN = token
    return _QUIET


def score(objects):
    oxi, hand = objects["oximeter"], objects["hand"]
    shift, dz, turn = hand_offset(hand)
    yanked = sticky("hand yanked", shift > HAND_YANK or abs(dz) > HAND_LIFT or turn > HAND_TURN_DEG)
    dropped = sticky("oximeter off the table", oxi.origin[2] < DROP_Z)
    on, seated, _ = clip_state(objects)
    hand_ok = shift <= HAND_MARK and turn <= HAND_MARK_DEG
    quiet = at_rest(objects)
    clear = released(oxi) and released(hand)
    done = on and clear and quiet
    seconds = held_for("on the finger, released, still", done)
    if yanked or dropped:
        return 0.0
    if not (done and seconds + 1e-9 >= STILL_S):
        return 0.0
    return 0.4 + 0.3 * float(seated) + 0.3 * float(hand_ok)


def success(objects):
    return score(objects) >= 1.0


# ------------------------------------------------------------------ scripted demo
# Tilted-wrist waypoints: a float yaw (for the offline checker's coarse replay) carrying the full grip
# rotation for the arm's IK, the same pattern the tilted-loop and hoist demos used.
class GripOrientation(float):
    def __new__(cls, matrix):
        obj = super().__new__(cls, math.atan2(-matrix[0, 0], matrix[1, 0]))
        obj.matrix = np.asarray(matrix, float).copy()
        return obj


_original_grip_rotation = getattr(_motion.grip_rotation, "_oxi_original", _motion.grip_rotation)


def _grip_rotation(yaw=0.0):
    matrix = getattr(yaw, "matrix", None)
    if isinstance(matrix, np.ndarray) and matrix.shape == (3, 3):
        return matrix
    return _original_grip_rotation(yaw)


_grip_rotation._oxi_original = _original_grip_rotation
_motion.grip_rotation = _grip_rotation

APPROACH = -(R_WK[:, 0])                           # the right gripper points from the back end to the mouth
_closing = -(R_WK[:, 2])
R_SQUEEZE = np.column_stack([_closing, np.cross(APPROACH, _closing), APPROACH])
R_DOWN_X = _original_grip_rotation(-math.pi / 2)   # left hand: straight down, pads closing along x

# Pads (13 mm past the TCP) pinch the back 10 mm of the clip: ridge above, underside below. Squeezed to its
# open stop the back end is 31 mm thick; each finger is commanded 4 mm past contact (about 20 N).
TCP_K = np.array([0.010 + 0.013, 0.0, 0.0155])     # TCP in frame K while squeezing
SQUEEZE_FINGER = -(0.031 - 0.008) / 2
SEAT_DROP = 0.0025                                 # bottom jaw carried 2.5 mm below the finger
TIP_WORLD = hand_to_world(INDEX_TIP_L)
# Carried clip frame at the seat: the fingertip axis sits SEAT_DROP above the floor pad, tip on the stop.
_K_SEAT_ORIGIN = TIP_WORLD - R_WK @ np.array([STOP_X, 0.0, FLOOR_Z + 0.0075 + SEAT_DROP])
SEAT = _K_SEAT_ORIGIN + R_WK @ TCP_K
OVERDRIVE = 0.0                                    # no push past the stop: it would shove the hand
LINEUP = SEAT - 0.065 * APPROACH
GRASP = np.array(OXI_START) + R_WK @ TCP_K
STEADY = hand_to_world(STUB_L) + np.array([0.0, 0.0, 0.006])
STEADY_FINGER = -(STUB_W - 0.008) / 2
DEMO_PACE = 1.0


def demo():
    G = GripOrientation
    plan = {"left": [], "right": []}
    st = {}

    def add(dt, label, right=None, left=None, rf=None, lf=None, rr=None, lr=None):
        r = st["r"] if right is None else np.asarray(right, float)
        l = st["l"] if left is None else np.asarray(left, float)
        rf = st["rf"] if rf is None else rf
        lf = st["lf"] if lf is None else lf
        rr = st["rr"] if rr is None else rr
        lr = st["lr"] if lr is None else lr
        st.update(r=r, l=l, rf=rf, lf=lf, rr=rr, lr=lr)
        plan["right"].append(Waypoint(dt * DEMO_PACE, tuple(float(v) for v in r), rf, label, G(rr)))
        plan["left"].append(Waypoint(dt * DEMO_PACE, tuple(float(v) for v in l), lf, label, G(lr)))

    behind = GRASP - 0.070 * APPROACH
    st.update(r=behind + [0, 0, 0.060], l=STEADY + [0, 0, 0.090],
              rf=FINGER_OPEN, lf=FINGER_OPEN, rr=R_SQUEEZE, lr=R_DOWN_X)
    add(3.0, "approach")
    add(1.5, "clip: in line behind it; wrist: descend", right=behind, left=STEADY)
    add(1.5, "clip: pads around its back end", right=GRASP)
    add(1.0, "squeeze the clip open; hold the wrist", rf=SQUEEZE_FINGER, lf=STEADY_FINGER)
    add(0.5, "squeeze hold")
    add(0.8, "lift off the wedge", right=GRASP + 0.006 * N_UP)
    add(1.5, "draw back off the wedge", right=GRASP + 0.006 * N_UP - 0.075 * APPROACH)
    add(1.5, "back past the fingertips", right=[GRASP[0], LINEUP[1], LINEUP[2]])
    add(2.0, "across to the index finger", right=LINEUP)
    add(0.5, "settle")
    add(3.0, "slide onto the finger to the stop", right=SEAT + OVERDRIVE * APPROACH)
    add(0.5, "hold at the stop")
    add(0.8, "release the squeeze", rf=FINGER_OPEN)
    add(1.0, "let the jaws close")
    add(1.5, "back away", right=SEAT - 0.070 * APPROACH)
    add(1.2, "right clear up", right=SEAT - 0.080 * APPROACH + [-0.020, 0, 0.060])
    add(0.8, "let go of the wrist", lf=FINGER_OPEN)
    add(1.0, "left clear up", left=STEADY + [0, 0, 0.090])
    add(1.0, "left back off", left=STEADY + [-0.040, 0.040, 0.110])
    add(3.5, "stillness tail")
    return plan


def demo_duration():
    return sum(w.duration for w in demo()["right"])
