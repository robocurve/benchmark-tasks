"""A patient with a mallet finger: an adult-size task-trainer right hand lies palm down on a 20-degree foam
arm-elevation wedge, fingers pointing up the slope and slightly spread, the fingertips reaching 50 mm past the
wedge's high edge. The index finger's last joint (DIP) is hinged and, as in a mallet finger, cannot hold itself
straight: the fingertip droops 40 degrees toward the palm. It lifts easily (about 0.5 N at the tip to hold it
straight) and sags back when let go. A rigid Stack-type mallet-finger splint (a light-blue moulded plastic shell, closed at the
fingertip end, open at the other) lies on the table in front of the fingertips, its open end toward the hand.
The hand is light and loose on the wedge. Zero object jitter; 20 Hz.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, ball, box  # noqa: E402
from yam_joints import Hinge, Link, Mechanism  # noqa: E402
from yam_mesh import Mesh, MeshPart  # noqa: E402
from yam_arena import demo as _motion  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"

# Release-and-stillness block, as in the accepted clip-pulse-oximeter-on-patient-finger task.
# Released means every gripper TCP is at least RELEASE_CLEAR from the object's box.
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
        out[arm] = (tcp, float(np.clip(np.mean(q[finger_ids]) / FINGER_OPEN, 0.0, 1.0)))
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


_POSE_PREV = {}


def pose_quiet(key, points, tol=0.0005):
    """Still: every given point moved under 0.5 mm since the previous control step (pose differences, because
    contact chatter makes the simulator's reported velocities unreliable). Offline it is always true."""
    if _RUN["t"] is None:
        return True
    _episode_memory()
    step = _RUN["step"]
    pts = np.asarray(points, float)
    prev = _POSE_PREV.get(key)
    if prev is not None and prev[0] == step:
        return prev[2]
    quiet = bool(prev is not None and prev[0] == step - 1 and np.all(np.abs(pts - prev[1]) < tol))
    _POSE_PREV[key] = (step, pts, quiet)
    return quiet


def ever(key, cond):
    """Remember a satisfied condition until the next episode; offline each call stands alone."""
    if _RUN["t"] is None:
        return bool(cond)
    _episode_memory()
    _EVER[key] = bool(_EVER.get(key, False) or cond)
    return _EVER[key]


# ------------------------------------------------------------------ geometry helpers
def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array(((1, 0, 0), (0, c, -s), (0, s, c)))


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

SLOPE_DEG = 20.0                                   # arm-elevation wedge: rises toward -y (toward the fingertips)
_A = math.radians(SLOPE_DEG)
R_SLOPE = _rx(-_A)                                 # hand/wedge-surface frame -> world
N_UP = R_SLOPE @ np.array([0, 0, 1.0])             # wedge surface normal

# Hand frame: x = world x, y along the fingers toward the wrist (down the slope), z = surface normal (the back of
# the hand); origin on the wedge surface plane under the index fingertip apex (finger straight).
HAND_X, HAND_Y = 0.270, -0.075
OVERHANG = 0.050                                   # fingertips reach 50 mm past the wedge's high edge
WEDGE_LEN = 0.180
WEDGE_LOW_Z = 0.005
WEDGE_X = (0.085, 0.335)
_EDGE_LOCAL_Y, _LOW_LOCAL_Y = OVERHANG, OVERHANG + WEDGE_LEN
HAND_Z = TOP + WEDGE_LOW_Z + _LOW_LOCAL_Y * math.sin(_A)
HAND_POS = (HAND_X, HAND_Y, HAND_Z)
EDGE = np.array(HAND_POS) + R_SLOPE @ np.array([0, _EDGE_LOCAL_Y, 0])
LOW = np.array(HAND_POS) + R_SLOPE @ np.array([0, _LOW_LOCAL_Y, 0])


def hand_to_world(p_local):
    return np.array(HAND_POS) + R_SLOPE @ np.asarray(p_local, float)


def hand_to_part(p_local):
    """Hand frame -> the hand Part's frame (world axes, origin HAND_POS)."""
    return R_SLOPE @ np.asarray(p_local, float)


# Wedge (fixed): convex prism; profile in world (y, z) relative to its origin on the table under the low end.
WEDGE_POS = (sum(WEDGE_X) / 2, float(LOW[1]), TOP)
_DY, _H = float(EDGE[1] - LOW[1]), float(EDGE[2] - TOP)
# Wedge geometry and offline proxy as in the accepted oximeter task.
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

# Trainer hand: spheres along each finger and over the palm (as the accepted oximeter task's hand), with the
# index finger's distal segment on its own hinge at the DIP joint.
HAND_MASS = 0.290
TIP_MASS = 0.008
SKIN = (0.86, 0.66, 0.52)
FINGERS = {
    "middle": ((-0.036, -0.010), (-0.024, 0.093), (0.0078, 0.0083, 0.0093)),
    "ring": ((-0.068, -0.002), (-0.045, 0.097), (0.0073, 0.0078, 0.0086)),
    "little": ((-0.098, 0.022), (-0.064, 0.102), (0.0065, 0.0068, 0.0076)),
}
PHALANX = (0.28, 0.58)                             # distal ends at 28 %, middle at 58 % of tip-to-knuckle
INDEX = ((0.0, 0.0), (0.0, 0.095))
INDEX_W = (0.018, 0.019, 0.021)                    # index widths, distal / middle / proximal
INDEX_T = (0.015, 0.016, 0.018)                    # index thicknesses
DIP_Y = PHALANX[0] * INDEX[1][1]                   # 26.6 mm from the fingertip
DIP_L = np.array([0.0, DIP_Y, INDEX_T[0] / 2])     # DIP axis (hand frame), across the finger
DROOP = 40.0                                       # degrees of fingertip droop (flexion) at rest
DIP_SPRING = 0.02                                  # N m/rad toward DIP_REST: passive tissue, no active extension
DIP_REST = 37.0
DIP_LIMITS = (-5.0, 70.0)


def _hand_balls():
    base, tip = [], []
    (tx, ty), (kx, ky) = INDEX
    for y in np.arange(ty + INDEX_T[0] / 2, ky + 0.0001, 0.004):
        f = (y - ty) / (ky - ty)
        i = 0 if f < PHALANX[0] else 1 if f < PHALANX[1] else 2
        r = INDEX_T[i] / 2
        for s in (-1, 1):
            (tip if i == 0 else base).append(ball(hand_to_part((tx + s * (INDEX_W[i] / 2 - r), y, r)), r))
    for name, ((tx, ty), (kx, ky), radii) in FINGERS.items():
        for y in np.arange(ty + radii[0], ky + 0.0001, 0.005):
            f = (y - ty) / (ky - ty)
            r = radii[0 if f < PHALANX[0] else 1 if f < PHALANX[1] else 2]
            base.append(ball(hand_to_part((tx + f * (kx - tx), y, r)), r))
    for f in np.linspace(0.0, 1.0, 14):            # thumb, beside the index finger
        x, y, r = 0.042 + f * (0.016 - 0.042), 0.058 + f * (0.165 - 0.058), 0.0095 + f * 0.0025
        base.append(ball(hand_to_part((x, y, r)), r))
    for x in np.linspace(-0.060, 0.000, 5):        # palm: 86 mm across the knuckles, 26 mm thick
        for y in np.linspace(0.097, 0.176, 7):
            base.append(ball(hand_to_part((x, y, 0.013)), 0.013))
    base.append(ball(hand_to_part((0.010, 0.150, 0.015)), 0.015))   # thenar eminence
    for x in (-0.045, -0.025):                     # wrist and forearm stub: 60 mm wide, 40 mm thick
        for y in np.linspace(0.183, 0.225, 6):
            base.append(ball(hand_to_part((x, y, 0.020)), 0.020))
    return base, tip


HAND_BASE_SHAPES, TIP_SHAPES = _hand_balls()
STUB_L = np.array([-0.035, 0.2082, 0.020])         # where the steadying gripper pinches the forearm stub
STUB_W = 0.060
_STUB_GRIP = int(np.argmin([np.linalg.norm(np.asarray(b.center) - hand_to_part(STUB_L + [-0.010, 0, 0]))
                            for b in HAND_BASE_SHAPES]))
HAND = Mechanism(
    "hand", HAND_POS, HAND_BASE_SHAPES,
    [Link("index_tip", TIP_SHAPES,
          Hinge("x", tuple(hand_to_part(DIP_L)), DIP_LIMITS[0], DIP_LIMITS[1], damping=5e-4, spring=DIP_SPRING,
                rest=DIP_REST, initial=DROOP, armature=1e-6),
          mass=TIP_MASS, color=SKIN, friction=(0.5, 0.4))],
    mass=HAND_MASS, color=SKIN, friction=(0.5, 0.4), contact_offset=0.001, grip=_STUB_GRIP)
TIP_APEX_L = np.array([0.0, 0.0, INDEX_T[0] / 2])  # fingertip apex on the distal axis, finger straight
TIP_BACK_L = np.array([0.0, 0.020, INDEX_T[0] / 2])

# Stack-type splint. Splint frame S: origin where the fingertip apex sits when fully seated (on the finger's axis),
# +y from the closed end toward the open end (toward the wrist), +z out of the hood (the nail side).
SPLINT_COLOR = (0.45, 0.70, 0.95)
W_IN = 0.0205                    # inside width (distal segment 18 mm, middle 19 mm)
WALL = 0.0015                    # shell thickness
Z_FLOOR, Z_ROOF = -0.0085, 0.0085  # inside floor and hood (distal segment 15 mm thick, middle 16 mm)
CAP_IN = -0.001                  # inside face of the closed end
HOOD_END = 0.030                 # the hood covers the fingertip and the DIP joint (26.6 mm from the tip)
L_END = 0.045                    # the floor runs on 18 mm under the middle segment: the support that holds the DIP
_XO = W_IN / 2 + WALL


def _bx(x0, x1, y0, y1, z0, z1):
    return box(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0))


SPLINT_SHAPES = [
    _bx(-_XO, _XO, CAP_IN - WALL, L_END, Z_FLOOR - WALL, Z_FLOOR),                 # floor (under the pad)
    _bx(-_XO, _XO, CAP_IN - WALL, CAP_IN, Z_FLOOR, Z_ROOF),                        # closed end
    _bx(-_XO, _XO, CAP_IN - WALL, HOOD_END, Z_ROOF, Z_ROOF + WALL),                # hood (over the nail and DIP)
    _bx(-_XO, -W_IN / 2, CAP_IN, HOOD_END, Z_FLOOR, Z_ROOF),                       # side walls under the hood
    _bx(W_IN / 2, _XO, CAP_IN, HOOD_END, Z_FLOOR, Z_ROOF),
    _bx(-_XO, -W_IN / 2, HOOD_END, L_END, Z_FLOOR, 0.0),                           # low side walls, open part
    _bx(W_IN / 2, _XO, HOOD_END, L_END, Z_FLOOR, 0.0),
]
SPLINT_MASS = 0.006
# On the table in front of the fingertips, hood up, open end toward the hand (+y).
SPLINT_START = (0.300, -0.200, TOP + (WALL - Z_FLOOR) + 0.0005)
SPLINT = Part("splint", SPLINT_START, SPLINT_SHAPES, mass=SPLINT_MASS, color=SPLINT_COLOR, friction=(0.35, 0.3),
              contact_offset=0.001)

OBJECTS = [WEDGE, HAND, SPLINT]

# ------------------------------------------------------------------ scoring
ON_DEPTH = 0.020          # on the fingertip: apex within 20 mm of the closed end (at least 10 mm under the hood)
SEAT_TOL = 0.004          # seated: apex within 4 mm of the closed end
AXIS_TOL = 0.005          # apex and a point 20 mm back both within 5 mm of the splint's centre line
HOOD_UP_DEG = 30.0        # the hood on the nail side: its normal within 30 degrees of the fingernail's
STRAIGHT_DEG = 8.0        # DIP held straight: within 8 degrees of full extension
FORCED_DEG = 10.0         # sticky zero: DIP ever bent more than 10 degrees past its starting droop
HAND_MARK, HAND_MARK_DEG = 0.010, 5.0
HAND_YANK, HAND_LIFT, HAND_TURN_DEG = 0.025, 0.015, 15.0
DROP_Z = TOP - 0.020
TEST_STICKY = ()


def sticky(key, cond):
    return ever(key, bool(cond) or key in TEST_STICKY)


def _angle_deg(r0, r1):
    c = (np.trace(np.asarray(r0).T @ np.asarray(r1)) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(c, -1.0, 1.0))))


def hand_offset(hand):
    d = np.asarray(hand.origin, float) - np.asarray(HAND_POS, float)
    return float(np.linalg.norm(d[:2])), float(d[2]), _angle_deg(np.eye(3), hand.rotation())


def dip_deg(hand):
    """DIP flexion in degrees: 0 straight, positive with the fingertip bent toward the palm."""
    return float(hand.angle_deg("index_tip"))


def tip_in_splint(objects):
    """(distance of the fingertip apex from the closed end, on the centre line, hood on the nail side)."""
    hand, sp = objects["hand"], objects["splint"]
    apex = sp.to_local(hand.link_to_world("index_tip", hand_to_part(TIP_APEX_L)))
    back = sp.to_local(hand.link_to_world("index_tip", hand_to_part(TIP_BACK_L - [0, 0.006, 0])))
    centred = all(abs(p[0]) <= AXIS_TOL and abs(p[2]) <= AXIS_TOL for p in (apex, back))
    nail = hand.link_rotation("index_tip") @ R_SLOPE[:, 2]
    hood = sp.rotation()[:, 2]
    hood_up = math.degrees(math.acos(float(np.clip(nail @ hood, -1, 1)))) <= HOOD_UP_DEG
    return float(apex[1] - CAP_IN), bool(centred and back[1] > apex[1]), hood_up


def score(objects):
    """0.3 with the splint on the fingertip; 0.6 seated against its closed end; 1.0 seated with the DIP held
    straight and the hand on its mark. Only once both grippers are 3 cm clear and all is still for 1 s."""
    hand, sp = objects["hand"], objects["splint"]
    shift, dz, turn = hand_offset(hand)
    dip = dip_deg(hand)
    yanked = sticky("hand yanked", shift > HAND_YANK or abs(dz) > HAND_LIFT or turn > HAND_TURN_DEG)
    forced = sticky("DIP forced into flexion", dip > DROOP + FORCED_DEG)
    dropped = sticky("splint off the table", sp.origin[2] < DROP_Z)
    short, centred, hood_up = tip_in_splint(objects)
    on = centred and hood_up and -0.002 <= short <= ON_DEPTH
    seated = on and short <= SEAT_TOL
    straight = abs(dip) <= STRAIGHT_DEG
    hand_ok = shift <= HAND_MARK and turn <= HAND_MARK_DEG
    clear = released(sp) and released(hand)
    still = pose_quiet("splint and hand", [sp.origin, sp.to_world((0, L_END, 0)), hand.origin,
                                           hand.link_to_world("index_tip", hand_to_part(TIP_APEX_L))])
    seconds = held_for("on, released, still", on and clear and still)
    if yanked or forced or dropped or seconds + 1e-9 < STILL_S:
        return 0.0
    if seated and straight and hand_ok:
        return 1.0
    return 0.6 if seated else 0.3


def success(objects):
    return score(objects) >= 1.0


# ------------------------------------------------------------------ closed-loop scripted demo (cabledemo.py)
class GripOrientation(float):
    """A float yaw carrying a full grip rotation for the arm's IK (the accepted oximeter task's pattern)."""
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

R_DOWN_X = _original_grip_rotation(-math.pi / 2)   # straight down, pads closing along world x
STEADY = hand_to_world(STUB_L) + np.array([0.0, 0.0, 0.006])
STEADY_FINGER = -(STUB_W - 0.008) / 2
PAD = 0.013                                        # pad centres 13 mm past the TCP
GRIP_S = np.array([0.0, 0.012, 0.0])               # pinch the side walls over the fingertip part of the hood
SPLINT_FINGER = -(2 * _XO - 0.006) / 2             # 3 mm past contact on each side
IN_DROOPED = 0.021                                 # apex this far from the closed end while it droops
SWING_TO = 0.0                                     # swing to straight
SERVO_GAIN = 0.5                                   # fraction of the splint's position error corrected per step
SEAT_PUSH = -0.0015                                # end of the slide: the closed end just against the apex


def tip_frame(hand, flex_deg=None):
    """World pose (origin, R) the splint frame S takes when seated on the fingertip at a DIP angle."""
    a = math.radians(dip_deg(hand) if flex_deg is None else flex_deg)
    R_hand = hand.rotation() @ R_SLOPE
    dip = np.asarray(hand.origin) + hand.rotation() @ hand_to_part(DIP_L)
    R = R_hand @ _rx(a)
    return dip + R @ (TIP_APEX_L - DIP_L), R


def demo_script(ctx):
    L, R = ctx.left, ctx.right
    st = ctx.states()
    sp0 = st["splint"]
    R_s0 = sp0.rotation()
    grip_w = np.asarray(sp0.origin) + R_s0 @ GRIP_S
    # Pads close across the splint (along x). The grip is tilted 20 degrees so that once the splint lies along the
    # straightened finger (sloping 20 degrees) the gripper stands straight up, clear of the hand steadying the forearm.
    R_g0 = R_s0 @ R_SLOPE.T @ _original_grip_rotation(math.pi / 2)
    approach = R_g0[:, 2]                           # from the TCP toward the pads
    def steady():
        yield from L.move(STEADY + [0, 0, 0.090], yaw=GripOrientation(R_DOWN_X), finger=FINGER_OPEN, duration=2.5)
        yield from L.move(STEADY, yaw=GripOrientation(R_DOWN_X), duration=1.5)
        yield from L.grip(STEADY_FINGER, duration=1.0)
        while not flags["placed"]:                    # hold the forearm until the splint is on and let go
            yield
        yield from L.wait(0.3)
        yield from L.grip(FINGER_OPEN, duration=1.0)    # open fully before lifting off the forearm
        yield from L.wait(0.3)
        yield from L.move(STEADY + [0, 0, 0.030], yaw=GripOrientation(R_DOWN_X), duration=1.2)
        yield from L.move(STEADY + [0, 0, 0.090], yaw=GripOrientation(R_DOWN_X), duration=1.0)
        yield from L.move(STEADY + [-0.06, 0.08, 0.12], yaw=GripOrientation(R_DOWN_X), duration=1.2)
        flags["left_clear"] = True

    def splint():
        pick = grip_w - (PAD - 0.002) * approach
        G0 = GripOrientation(R_g0)
        yield from R.move(pick - 0.06 * approach, yaw=G0, finger=FINGER_OPEN, duration=2.5)
        yield from R.move(pick, yaw=G0, duration=1.2)
        yield from R.grip(SPLINT_FINGER, duration=0.8)
        yield from R.move(pick + [0, 0, 0.05], yaw=G0, duration=1.2)
        # The splint's pose in the gripper, as measured once it is held.
        s = ctx.states()["splint"]
        tcp = np.asarray(R.tcp())
        G = s.rotation().T @ R_g0                    # gripper rotation in S
        T = s.rotation().T @ (tcp - np.asarray(s.origin))   # TCP in S

        bias = np.zeros(3)                           # correction for the arm's sag: measured splint error, integrated

        tilt = [0.0]                                 # the same for the splint's pitch about the finger's x axis

        def carry(o_S, R_S, duration, servo=True):
            nonlocal bias
            Rc = R_S @ _rx(tilt[0])                  # command the splint pitched by the correction
            yield from R.move(o_S + Rc @ T + bias, yaw=GripOrientation(Rc @ G), duration=duration)
            if servo:
                sp = ctx.states()["splint"]
                err = o_S - np.asarray(sp.origin)
                bias = np.clip(bias + SERVO_GAIN * err, -0.020, 0.020)
                Re = R_S.T @ sp.rotation()           # actual splint relative to the target
                tilt[0] = float(np.clip(tilt[0] - SERVO_GAIN * math.atan2(Re[2, 1], Re[1, 1]), -0.35, 0.35))

        hand = ctx.states()["hand"]
        apex, R_t = tip_frame(hand)                  # the drooping fingertip as it lies now
        y_ax = R_t[:, 1]
        # 1. In line with the drooping fingertip, its open end 15 mm short of the apex; slide on until the apex is
        #    21 mm from the closed end (further in, the floor would meet the middle segment while the tip droops).
        yield from carry(apex - 0.060 * y_ax + 0.020 * R_t[:, 2], R_t, 2.5)
        yield from carry(apex - 0.060 * y_ax, R_t, 1.2)
        for k in range(1, 13):
            apex, R_t = tip_frame(ctx.states()["hand"])
            yield from carry(apex - (0.060 - (0.060 - IN_DROOPED) * k / 12) * R_t[:, 1], R_t, 0.2)
        ref = {}                                      # the hand as it lies once the finger is straight

        def _hold():
            a_, r_ = tip_frame(ref["hand"], SWING_TO)
            return a_ - IN_DROOPED * r_[:, 1], r_

        # 2. Swing the splint (and the fingertip in it) up about the DIP to straight, keeping the apex in place.
        a0 = dip_deg(ctx.states()["hand"])
        n = 20
        for k in range(1, n + 1):
            a = a0 + (SWING_TO - a0) * k / n
            apex, R_t = tip_frame(ctx.states()["hand"], a)
            yield from carry(apex - IN_DROOPED * R_t[:, 1], R_t, 0.15)
        ref["hand"] = ctx.states()["hand"]            # from here on, aim at a fixed target: following the live hand
                                                      # would chase it if it were nudged
        # 3. Slide it home along the straight finger until the closed end meets the fingertip.
        d0 = tip_in_splint(ctx.states())[0] + CAP_IN   # where the apex is in the splint now
        n = max(6, int(round((d0 - SEAT_PUSH) / 0.002)))  # 2 mm per step
        for k in range(1, n + 1):
            apex, R_t = tip_frame(ref["hand"], SWING_TO)
            yield from carry(apex - (d0 - (d0 - SEAT_PUSH) * k / n) * R_t[:, 1], R_t, 0.2)
        yield from R.wait(0.5)
        # 4. Let go and back off along the hood's normal, then away.
        yield from R.grip(FINGER_OPEN, duration=0.8)
        yield from R.wait(0.3)
        up = R_t[:, 2]
        Rf = GripOrientation(R_t @ _rx(tilt[0]) @ G)
        yield from R.move(np.asarray(R.tcp()) + 0.05 * up, yaw=Rf, duration=1.5)
        yield from R.move(np.asarray(R.tcp()) + [0, -0.06, 0.06], yaw=Rf, duration=1.2)
        flags["placed"] = True
        yield from R.wait(4.0)

    flags = {"placed": False, "left_clear": False}
    yield from ctx.both(splint(), steady())
