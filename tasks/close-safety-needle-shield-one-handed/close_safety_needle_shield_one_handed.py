"""After an injection into a practice pad: a 3 mL safety syringe (10.5 mm barrel, plunger pushed in) lies on its
side beside a silicone injection pad, its 25 G x 1 in needle bare and pointing right (-y). The orange hinged
safety sheath on the needle hub stands open, sticking out sideways toward the arms (-x). The sheath is a
U-channel that swings on a hinge at the hub and closes around the needle; a sprung catch inside the channel
snaps past the needle in the last 8 degrees and then holds the sheath shut for good. A red open-top sharps
tray stands to the right. The table top is the firm surface. Zero object jitter; 20 Hz.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, ball, box, cyl
from yam_joints import Hinge, Link, Mechanism, Slide
from yam_arena import demo as _motion
from yam_arena import ik as _ik
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY
from yam_arena.demo import Waypoint
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


# ------------------------------------------------------------------ layout (metres, world frame)
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# Syringe frame: origin on the barrel axis; +y toward the plunger, the needle points to -y; the open sheath
# sticks out to -x; z is the hinge axis. Lying on the table the part frame is the world frame.
BARREL_R = 0.00525                                 # 10.5 mm outside diameter
BARREL_Y = (-0.030, 0.036)                         # barrel shoulder .. finger flange
FLANGE_W = 0.022
THUMB_Y = 0.048                                    # plunger thumb rest (plunger fully in)
HUB_Y = -0.050                                     # front of the needle hub
NEEDLE_D = 0.0005                                  # 25 G cannula, 0.51 mm
NEEDLE_L = 0.025
NEEDLE_TIP = np.array([0.0, HUB_Y - NEEDLE_L, 0.0])
HINGE = np.array([-0.004, -0.044, 0.0])            # hinge pin on the hub collar, 4 mm off the axis
SHEATH_L = 0.042                                   # hinge pin to the sheath's far end
SHEATH_END_Y = float(HINGE[1] - SHEATH_L)
SHEATH_W = 0.009                                   # across the channel (along the hinge axis)
WALL = 0.001
CH_BACK, CH_MOUTH = -0.0045, 0.004                 # channel back (outside face) and mouth edge, x at q = 0
HINGE_STICTION = (0.012, 0.010)                    # N m: the hinge holds the sheath where it is put

# Catch: a sprung plunger in the sheath's side wall, 25 mm from the hinge pin. A 1.5 mm nose with a 45-degree
# lead-in faces the mouth; a square heel faces the back. The needle lifts it 0.44 mm, passes, and it drops
# behind the needle.
CATCH_Y = -0.069
NOSE_R = 0.0015
NOSE_X = 0.0021                                    # nose centre; the heel face is at x = 0.6 mm
NOSE_Z = 0.00131
HEEL_X = NOSE_X - NOSE_R
CATCH_TRAVEL = 0.0012
CATCH_K = 600.0                                    # N/m
CATCH_PRELOAD = 3.0                                # N at rest (3.26 N at the crest)
CATCH_LIFT = NEEDLE_D / 2 - (NOSE_Z - NOSE_R)      # 0.44 mm

WHITE = (0.55, 0.78, 0.92)                         # clear barrel, tinted so the cameras see it
ORANGE = (0.95, 0.45, 0.08)                        # ISO 6009 colour for 25 G
STEEL = (0.62, 0.64, 0.66)
RED = (0.78, 0.10, 0.10)
SKIN = (0.90, 0.72, 0.60)

GRIP_Y = -0.018                                    # where the barrel is held: its front 24 mm
_by = 2 * GRIP_Y - BARREL_Y[0]
SYRINGE_BASE = [
    cyl((0, GRIP_Y, 0), BARREL_R, 2 * (GRIP_Y - BARREL_Y[0]), axis="y", color=WHITE),          # 0 barrel, front
    cyl((0, (_by + BARREL_Y[1]) / 2, 0), BARREL_R, BARREL_Y[1] - _by, axis="y", color=WHITE),  # barrel, rear
    box((0, BARREL_Y[1] + 0.001, 0), (FLANGE_W, 0.002, 2 * BARREL_R), color=WHITE),            # finger flange
    cyl((0, 0.042, 0), 0.003, 0.008, axis="y", color=WHITE),                                   # plunger rod
    cyl((0, 0.047, 0), BARREL_R, 0.002, axis="y", color=WHITE),                                # thumb rest
    cyl((0, -0.036, 0), 0.0032, 0.012, axis="y", color=ORANGE),                                # hub collar
    cyl((0, -0.046, 0), 0.0022, 0.008, axis="y", color=ORANGE),                                # hub nose
    box((0, HUB_Y - NEEDLE_L / 2, 0), (NEEDLE_D, NEEDLE_L, NEEDLE_D), color=STEEL),            # cannula
]
_sy = (HINGE[1] + SHEATH_END_Y) / 2
_cx, _cw = (CH_BACK + CH_MOUTH) / 2, CH_MOUTH - CH_BACK
SHEATH_SHAPES = [
    box((CH_BACK + WALL / 2, _sy, 0), (WALL, SHEATH_L, SHEATH_W)),                             # channel back
    box((_cx, _sy, SHEATH_W / 2 - WALL / 2), (_cw, SHEATH_L, WALL)),                           # side wall +z
    box((_cx, _sy, -SHEATH_W / 2 + WALL / 2), (_cw, SHEATH_L, WALL)),                          # side wall -z
]
CATCH_SHAPES = [
    ball((NOSE_X, CATCH_Y, NOSE_Z), NOSE_R),                                                   # lead-in nose
    box(((HEEL_X + NOSE_X) / 2, CATCH_Y, (NOSE_Z - NOSE_R + 0.0025) / 2),
        (NOSE_X - HEEL_X, 0.0012, 0.0025 - (NOSE_Z - NOSE_R))),                                # square heel
    box((0.0012, CATCH_Y, 0.0030), (0.0012, 0.0012, 0.0008)),                                  # stem
]
SYRINGE_POS = (0.230, -0.040, TOP + BARREL_R)       # the syringe body's origin at the start
# The articulation's root is the sheath (the middle of the chain: body - sheath - catch), so the part frame
# is the sheath's frame: at joint angle 0 the sheath lies closed along the needle and the two frames agree.
# "body" is the syringe turning about the hinge pin: +100 degrees = sheath open in its ready position.
OPEN_DEG = 100.0
OVER_DEG = 3.0                                     # hard stop 3 degrees past closed


def part_pose(body_origin, open_deg=OPEN_DEG, body_yaw=0.0):
    """Part (sheath) origin and yaw that put the syringe body at ``body_origin`` with yaw ``body_yaw``."""
    yaw = body_yaw - math.radians(open_deg)
    pos = np.asarray(body_origin, float) + _rz(body_yaw) @ HINGE - _rz(yaw) @ HINGE
    return tuple(float(v) for v in pos), yaw


START_POS, START_YAW = part_pose(SYRINGE_POS)
SYRINGE = Mechanism(
    "syringe", START_POS, SHEATH_SHAPES,
    [Link("body", SYRINGE_BASE,
          Hinge("z", tuple(HINGE), -OVER_DEG, OPEN_DEG, damping=2e-5, initial=OPEN_DEG, armature=5e-7,
                static_friction=HINGE_STICTION[0], dynamic_friction=HINGE_STICTION[1]),
          mass=0.0064, color=WHITE, friction=(0.6, 0.5)),
     Link("catch", CATCH_SHAPES,
          Slide("z", (0.0012, CATCH_Y, 0.0034), 0.0, CATCH_TRAVEL, damping=2.0, spring=CATCH_K,
                rest=-CATCH_PRELOAD / CATCH_K, armature=0.01),
          mass=0.00005, color=ORANGE, friction=(0.15, 0.12))],
    mass=0.0006, color=ORANGE, yaw=START_YAW, friction=(0.4, 0.3), contact_offset=0.0005,
    grip=len(SHEATH_SHAPES), self_collide=True)

# Practice injection pad (fixed) and sharps tray (free).
PAD_SIZE = (0.100, 0.100, 0.020)
PAD_POS = (0.230, 0.128, TOP)
PAD = Part("pad", PAD_POS, [box((0, 0, PAD_SIZE[2] / 2), PAD_SIZE)], color=SKIN, friction=(0.8, 0.7))
TRAY_OUT = (0.110, 0.160, 0.060)                   # x, y, height
TRAY_WALL = 0.003
TRAY_POS = (0.130, -0.225, TOP)
_tx, _ty, _th = TRAY_OUT
TRAY_SHAPES = [
    box((0, 0, TRAY_WALL / 2), (_tx, _ty, TRAY_WALL)),
    box((_tx / 2 - TRAY_WALL / 2, 0, _th / 2), (TRAY_WALL, _ty, _th)),
    box((-_tx / 2 + TRAY_WALL / 2, 0, _th / 2), (TRAY_WALL, _ty, _th)),
    box((0, _ty / 2 - TRAY_WALL / 2, _th / 2), (_tx - 2 * TRAY_WALL, TRAY_WALL, _th)),
    box((0, -_ty / 2 + TRAY_WALL / 2, _th / 2), (_tx - 2 * TRAY_WALL, TRAY_WALL, _th)),
]
TRAY = Part("tray", TRAY_POS, TRAY_SHAPES, mass=0.35, color=RED, friction=(0.6, 0.5))
PRESSED_PARTS = ("syringe",)
OBJECTS = [SYRINGE, PAD, TRAY]

# ------------------------------------------------------------------ scoring
LOCK_DEG = 1.5            # sheath within 1.5 degrees of closed (the catch can only drop behind the needle there)
CATCH_HOME = 0.0003       # catch back within 0.3 mm of fully out
COVER = 0.003             # needle tip at least 3 mm inside the sheath's far end
PART_DEG = 45.0           # partly closed: within 45 degrees of closed
FREE_HAND = 0.100         # second gripper's centre point to the bare needle tip
TOUCH = 0.0015            # needle tip inside a body grown by 1.5 mm = touching it
ON_TABLE = 0.003          # sheath's lowest point within 3 mm of the table top when it locks
FINGER_REACH = 0.015      # a finger can touch things 15 mm + half its opening from the centre point
HALF_OPEN = 0.047
DROP_Z = TOP - 0.020
BARREL_GRIP = (BARREL_Y[0] - 0.005, BARREL_Y[1] + 0.005)
STEP_M, STEP_DEG = 0.0005, 0.25
TEST_STICKY = ()          # offline tests: sticky keys that already fired earlier in the episode


def sticky(key, cond):
    return ever(key, bool(cond) or key in TEST_STICKY)


def _box_dist(p, low, high):
    p = np.asarray(p, float)
    return float(np.linalg.norm(np.maximum(np.maximum(np.asarray(low) - p, p - np.asarray(high)), 0.0)))


def frames(syr):
    """(sheath rotation, sheath origin, body rotation, body origin) in the world, from the root pose and the
    hinge angle alone."""
    R0, o0 = syr.rotation(), np.asarray(syr.origin, float)
    R = R0 @ _rz(syr.joints["body"])
    return R0, o0, R, o0 + R0 @ HINGE - R @ HINGE


def sheath_state(syr):
    """(angle from closed in degrees, locked, needle tip covered)."""
    q = syr.joints["body"]
    angle = math.degrees(q)
    locked = angle <= LOCK_DEG and syr.joints["catch"] <= CATCH_HOME
    p = HINGE + _rz(q) @ (NEEDLE_TIP - HINGE)                    # needle tip in the sheath's frame
    covered = (CH_BACK + WALL <= p[0] <= CH_MOUTH and abs(p[2]) <= SHEATH_W / 2 - WALL
               and p[1] - SHEATH_END_Y >= COVER)
    return float(angle), bool(locked), bool(covered)


def _corners(lo, hi):
    return np.array([(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])


_SHEATH_BOX = _corners((CH_BACK, SHEATH_END_Y, -SHEATH_W / 2), (CH_MOUTH, HINGE[1], SHEATH_W / 2))
_BODY_BOX = np.vstack([_corners((-BARREL_R, NEEDLE_TIP[1], -BARREL_R), (BARREL_R, THUMB_Y, BARREL_R)),
                       _corners((-FLANGE_W / 2, BARREL_Y[1], -BARREL_R), (FLANGE_W / 2, BARREL_Y[1] + 0.002, BARREL_R))])


class _Bounds:
    def __init__(self, points):
        self.low, self.high = points.min(axis=0), points.max(axis=0)


def boxes(syr):
    """World bounding boxes of the sheath and of the whole syringe."""
    R0, o0, R, o = frames(syr)
    sheath = o0 + _SHEATH_BOX @ R0.T
    return _Bounds(sheath), _Bounds(np.vstack([sheath, o + _BODY_BOX @ R.T]))


def _tray_walls(tray):
    """World boxes of the tray's four walls."""
    out = []
    for s in TRAY_SHAPES[1:]:
        c = tray.to_world(s.center)
        r = np.abs(tray.rotation()) @ (np.asarray(s.size) / 2)
        out.append((c - r, c + r))
    return out


def in_tray(syr, tray):
    """The whole syringe inside the tray's inner outline (2 mm tolerance), below the rim, tray upright."""
    a, b = tray.to_local(syr.low), tray.to_local(syr.high)        # syr: the whole syringe's bounds
    lo, hi = np.minimum(a, b), np.maximum(a, b)
    inner = np.array([_tx / 2 - TRAY_WALL, _ty / 2 - TRAY_WALL])
    return bool(np.all(lo[:2] >= -inner - 0.002) and np.all(hi[:2] <= inner + 0.002)
                and syr.low[2] < tray.high[2] and abs(tray.low[2] - TOP) < 0.005)


_POSE_EPISODE = None
_POSE_TOKEN = None
_PREVIOUS = None
_QUIET = False


def at_rest(objects):
    """Pose differences between consecutive control steps (under 0.5 mm and 0.25 degrees per 50 ms)."""
    global _POSE_EPISODE, _POSE_TOKEN, _PREVIOUS, _QUIET
    syr, tray = objects["syringe"], objects["tray"]
    if _RUN["t"] is None:
        return syr.resting() and tray.resting() and abs(syr.joint_vel["body"]) < 0.05
    if _POSE_EPISODE != _RUN["episode"]:
        _PREVIOUS, _POSE_TOKEN, _QUIET = None, None, False
        _POSE_EPISODE = _RUN["episode"]
    token = (_RUN["step"], float(_RUN["t"]))
    if token != _POSE_TOKEN:
        _, _, R, o = frames(syr)
        pose = (np.r_[o + R @ np.array((0, THUMB_Y, 0)), o + R @ NEEDLE_TIP,
                      tray.to_world((0, 0, 0)), tray.to_world((0, 0.08, 0))], syr.angle_deg("body"))
        _QUIET = False
        if _PREVIOUS is not None:
            step, old = _PREVIOUS
            _QUIET = bool(token[0] == step + 1
                          and np.all(np.abs(pose[0] - old[0]) < STEP_M) and abs(pose[1] - old[1]) < STEP_DEG)
        _PREVIOUS = (token[0], pose)
        _POSE_TOKEN = token
    return _QUIET


def score(objects):
    syr, pad, tray = objects["syringe"], objects["pad"], objects["tray"]
    angle, locked, covered = sheath_state(syr)
    _, _, R, o = frames(syr)
    tip = o + R @ NEEDLE_TIP
    sheath_box, whole = boxes(syr)
    bare = not locked
    hands = grippers()
    second_hand, wrong_end, finger_on_sheath = False, False, False
    if hands is not None:
        # The holding hand is the gripper nearest the middle of the barrel; every other gripper is a free hand.
        mid = o + R @ np.array((0.0, (BARREL_Y[0] + BARREL_Y[1]) / 2, 0.0))
        order = sorted(hands.values(), key=lambda h: float(np.linalg.norm(h[0] - mid)))
        second_hand = any(float(np.linalg.norm(tcp - tip)) < FREE_HAND for tcp, _ in order[1:])
        along = (R.T @ (order[0][0] - o))[1]
        wrong_end = (float(np.linalg.norm(order[0][0] - tip)) < FREE_HAND
                     and not BARREL_GRIP[0] <= along <= BARREL_GRIP[1])
        finger_on_sheath = any(_box_dist(tcp, sheath_box.low, sheath_box.high) < FINGER_REACH + HALF_OPEN * opening
                               for tcp, opening in hands.values())
    # Sticky zeros, evaluated on every call.
    free_hand = sticky("free hand near the bare needle", bare and second_hand)
    by_needle = sticky("holding hand at the needle end", bare and wrong_end)
    on_pad = sticky("bare needle touched the pad", bare and _box_dist(tip, pad.low, pad.high) < TOUCH)
    on_tray = sticky("bare needle touched the tray wall",
                     bare and any(_box_dist(tip, lo, hi) < TOUCH for lo, hi in _tray_walls(tray)))
    dropped = sticky("syringe off the table", o[2] < DROP_Z)
    if _RUN["t"] is None:
        bad_lock = sticky("sheath not locked on the table", False)
    else:
        _episode_memory()
        was_locked = _EVER.get("locked", False)
        ever("locked", locked)
        sheath_low = sheath_box.low[2]
        over_other = (_box_dist(tip, pad.low, pad.high) < 0.010
                      or _box_dist(tip[:2], tray.low[:2], tray.high[:2]) < 0.010)
        legit = sheath_low <= TOP + ON_TABLE and not finger_on_sheath and not over_other
        bad_lock = sticky("sheath not locked on the table", locked and not was_locked and not legit)
    quiet = at_rest(objects)
    clear = released(whole)
    done = clear and quiet
    seconds = held_for("released and still", done)
    if free_hand or on_pad or on_tray or dropped or by_needle or bad_lock:
        return 0.0
    if not (done and seconds + 1e-9 >= STILL_S):
        return 0.0
    disposed = in_tray(whole, tray)
    if locked and covered:
        return 1.0 if disposed else 0.4
    if disposed:
        return 0.15
    return 0.10 if angle <= PART_DEG else 0.0


# Reference layouts for the scorer tests (states.yaml).
_IN_TRAY = (TRAY_POS[0], TRAY_POS[1] + 0.018, TOP + TRAY_WALL + BARREL_R)
BARE_IN_TRAY, _ = part_pose((_IN_TRAY[0] + 0.020, _IN_TRAY[1], _IN_TRAY[2]))
NEEDLE_IN_PAD, _ = part_pose((PAD_POS[0], PAD_POS[1] + 0.120, TOP + BARREL_R))


def success(objects):
    return score(objects) >= 1.0


# ------------------------------------------------------------------ scripted demo
# Tilted-wrist waypoints: a float yaw (for the offline checker's coarse replay) carrying the full grip
# rotation for the arm's IK.
class GripOrientation(float):
    def __new__(cls, matrix):
        obj = super().__new__(cls, math.atan2(-matrix[0, 0], matrix[1, 0]))
        obj.matrix = np.asarray(matrix, float).copy()
        return obj


_original_grip_rotation = getattr(_motion.grip_rotation, "_syringe_original", _motion.grip_rotation)


def _grip_rotation(yaw=0.0):
    matrix = getattr(yaw, "matrix", None)
    if isinstance(matrix, np.ndarray) and matrix.shape == (3, 3):
        return matrix
    return _original_grip_rotation(yaw)


_grip_rotation._syringe_original = _original_grip_rotation
_motion.grip_rotation = _grip_rotation

R_DOWN = _original_grip_rotation(math.pi / 2)      # right hand straight down, pads closing along x
GRIP_L = np.array([0.0, GRIP_Y, 0.015 - BARREL_R])  # centre point between the pads, in the syringe frame
SHEATH_TIP_L = np.array([CH_BACK, SHEATH_END_Y, -SHEATH_W / 2])   # the sheath corner that meets the table
ROLL_DEG = 55.0                                    # wrist rolled about the barrel so the sheath points down
PRESS_XY = (0.225, -0.100)                         # table point the sheath end is set on
ARC_STEPS = 10
ARC_START = (100.0, 95.0)                          # sheath angle from closed, sheath angle to the table
ARC_END = (0.0, 25.0)
ARC_INTO_TABLE = 0.005                             # the arc is commanded 5 mm into the table
PRESS_EXTRA = 0.010                                # final press: 10 mm further down
GRASP = np.array(SYRINGE_POS) + GRIP_L
TRAY_DROP = np.array([TRAY_POS[0], TRAY_POS[1] - 0.001, TOP + 0.042])
HALF_FINGER = -0.020
LEFT_BASE = (0.0, 0.305, 0.75)
DEMO_PACE = 1.0


def press_pose(sheath_deg, table_deg, into=0.0, tip_xy=PRESS_XY):
    """Grip centre point and rotation that put the sheath end on the table point with the sheath at the given
    angle from closed and at the given angle to the table (the barrel pitched to suit)."""
    pitch = math.radians(table_deg - sheath_deg)
    R = _rx(pitch) @ _ry(-math.radians(ROLL_DEG))
    tip_l = HINGE + _rz(-math.radians(sheath_deg)) @ (SHEATH_TIP_L - HINGE)
    origin = np.array([tip_xy[0], tip_xy[1], TOP - into]) - R @ tip_l
    return origin + R @ GRIP_L, R @ R_DOWN


def demo():
    G = GripOrientation
    left_pos, left_rot = _ik.tcp_pose(np.array(READY, float), LEFT_BASE)
    plan = {"left": [], "right": []}
    st = {}

    def add(dt, label, pos=None, finger=None, rot=None):
        st["p"] = st["p"] if pos is None else np.asarray(pos, float)
        st["f"] = st["f"] if finger is None else finger
        st["r"] = st["r"] if rot is None else rot
        plan["right"].append(Waypoint(dt * DEMO_PACE, tuple(float(v) for v in st["p"]), st["f"], label, G(st["r"])))
        plan["left"].append(Waypoint(dt * DEMO_PACE, tuple(float(v) for v in left_pos), FINGER_OPEN, label,
                                     G(left_rot)))

    st.update(p=GRASP + [0, 0, 0.080], f=HALF_FINGER, r=R_DOWN)
    add(2.5, "over the barrel")
    add(1.5, "down around the barrel", pos=GRASP)
    add(1.0, "grip the barrel", finger=FINGER_CLOSED)
    add(1.2, "lift", pos=GRASP + [0, 0, 0.070])
    p0, r0 = press_pose(*ARC_START, into=-0.020)
    add(2.5, "roll the sheath downward", pos=p0, rot=r0)
    p1, r1 = press_pose(*ARC_START, into=ARC_INTO_TABLE)
    add(1.2, "set the sheath end on the table", pos=p1, rot=r1)
    for k in range(1, ARC_STEPS + 1):
        s = k / ARC_STEPS
        p, r = press_pose(ARC_START[0] + s * (ARC_END[0] - ARC_START[0]),
                          ARC_START[1] + s * (ARC_END[1] - ARC_START[1]), into=ARC_INTO_TABLE)
        add(0.5, f"rock the sheath shut {k}", pos=p, rot=r)
    add(1.0, "press until it clicks", pos=st["p"] - [0, 0, PRESS_EXTRA])
    add(0.7, "hold")
    add(1.2, "lift off the table", pos=st["p"] + [0, 0, 0.060])
    add(2.5, "over the sharps tray", pos=[TRAY_DROP[0], TRAY_DROP[1], TOP + 0.120], rot=R_DOWN)
    add(1.5, "down into the tray", pos=TRAY_DROP)
    add(0.8, "let go", finger=HALF_FINGER)
    add(1.2, "rise", pos=[TRAY_DROP[0], TRAY_DROP[1], TOP + 0.150])
    add(1.0, "back off", pos=[TRAY_DROP[0] + 0.030, TRAY_DROP[1] + 0.060, TOP + 0.150])
    add(3.5, "stillness tail")
    return plan


def demo_duration():
    return sum(w.duration for w in demo()["right"])
