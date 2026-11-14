"""Swipe a true-size magnetic-stripe card through a desktop swipe reader, then stand it back in its card stand.

The reader follows a real USB swipe reader (99.5 x 31.6 x 30.4 mm, slot floor 7.76 mm up, head at mid-length) and
the reader maker's design rules for swipe slots (0.864-1.067 mm slot, head gap on track 2, sprung head resting on
the far wall). The card is an ISO/IEC 7810 ID-1 card, 85.60 x 53.98 x 0.76 mm, with its stripe where ISO/IEC 7811
puts it.
Nothing is read by software: the score judges the swipe from the card's pose and velocity and the head's
deflection, the physical conditions under which a real head reads the stripe. The status lamp shows the same
judgement from its own copy of that physical state and is never read by the scorer.
"""
import math
import pathlib
import sys
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import yam_parts as parts  # noqa: E402
import yam_joints as joints  # noqa: E402
from yam_parts import Part, PartState, box, cyl  # noqa: E402
from yam_joints import Lamp, Link, Mechanism, Slide  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
import yam_arena.demo as _motion  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"

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


TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S, SCORE_OVER = 1.0, "end"
RUN_LIMIT_STEPS = 2400  # 120 s at 20 Hz
CONTROL_DT = 0.05
DEMO_PACE = 1.0  # scales every demo duration (0.8 for the timing-margin copy)
TEST_HISTORY = None  # offline only: recorded samples before the scored layout (see history rows below)

# ---------------------------------------------------------------- card: ISO/IEC 7810 ID-1, stripe per ISO/IEC 7811
CARD_L, CARD_W, CARD_T = 0.08560, 0.05398, 0.00076
CARD_MASS = 0.0049  # rigid PVC, 1.39 g/cm3 x 3.51 cm3
CARD_FRICTION = (0.40, 0.35)
CARD_CONTACT_OFFSET = 0.0005
# Stripe area 5.54 mm (max) to 15.95 mm (min, three tracks) from the top reference edge (ISO/IEC 7811-2/-4).
STRIPE_FROM, STRIPE_TO = 0.00554, 0.01595
TRACK2 = (0.00897, 0.01176)  # ISO/IEC 7811-4 track 2, from the reference edge
DATA_FROM, DATA_TO = 0.00744 - 0.00050, CARD_L - 0.00693  # first data bit 7.44 +- 0.50 mm; last >= 6.93 mm from the far edge
# Card part frame: x = across the width (the stripe, or reference, edge is at x = +W/2), y = along the length,
# z = out of the stripe face. At the start the card lies flat on the counter, stripe face up, stripe edge toward +x.

# ---------------------------------------------------------------- reader: desktop USB swipe reader, fixed with tape
# Turned -90 deg so the slot lies on the line straight out from the right arm's base (reader +y = world +x).
READER_YAW = -math.pi / 2
READER_R = np.array([[math.cos(READER_YAW), -math.sin(READER_YAW), 0.0],
                     [math.sin(READER_YAW), math.cos(READER_YAW), 0.0], [0.0, 0.0, 1.0]])
R_W, R_L, R_H = 0.0316, 0.0995, 0.0304  # x (across), y (along the slot), z
R_FLOOR = 0.00776  # slot floor above the counter (0.305 in)
SLOT_F, SLOT_B = -0.00276, -0.0017  # slot faces (front = head side, world +y): 1.06 mm, inside the maker's 0.864-1.067
SLOT_XC = (SLOT_F + SLOT_B) / 2
XR, YR = 0.316, -0.305 + SLOT_XC  # slot centre line passes through the right arm's base (m)
Y_STRAIGHT, Y_END = 0.03475, R_L / 2  # straight track |y| <= 34.75 mm; flared mouths to the ends
FLARE_OPEN = 0.002  # each mouth wall steps back 2.0 mm over the last 15 mm (slot 1.06 -> 5.06 mm)
FLARE_T = 0.002
SLOT_ROUND = 0.0008  # radius on the slot's top inner edges
JUNCTION_GAP = 0.0003  # straight walls stop 0.3 mm short of the mouth parts (no static overlap)
HEAD_R = 0.0045
HEAD_Z0, HEAD_Z1 = R_FLOOR + 0.0046, R_FLOOR + 0.0169
GAP_Z = 0.0104  # head gap above the slot floor: track 2's centreline, 0.408 in from the card's edge
HEAD_TRAVEL = 0.0015
HEAD_K, HEAD_REST, HEAD_DAMP, HEAD_ARMATURE, HEAD_MASS = 1700.0, 0.0001, 2.0, 0.01, 0.003
WIN_HALF, WIN_Z0, WIN_Z1 = 0.0052, HEAD_Z0 - 0.0004, HEAD_Z1 + 0.0004
WIN_BACK_X = -0.0126  # front-wall material behind the head window (head reaches -12.2 mm at full travel)
READER_FRICTION, HEAD_FRICTION = (0.30, 0.25), (0.20, 0.15)
READER_CONTACT_OFFSET = 0.0015
LED_CENTER, LED_R = (0.0080, 0.0312, R_H + 0.0008), 0.0028

# ---------------------------------------------------------------- card stand (desk stand, weighted, does not move)
XS, YS = 0.235, -0.410
S_W, S_L, S_H = 0.024, 0.108, 0.018
S_FLOOR = 0.004
S_GAP = 0.0016  # slot width between the straight walls
S_LIP_R, S_LIP_Z = 0.003, 0.015  # rounded lips (axis along y) form the slot's funnel mouth
S_SLOT_LEN = 0.100  # 7.2 mm longer than the card at each end: a card can be dropped in without lining up its ends
STAND_FRICTION = (0.40, 0.35)

BLACK, GREY, STEEL, SILVER = (0.05, 0.05, 0.06), (0.26, 0.27, 0.29), (0.55, 0.57, 0.60), (0.78, 0.79, 0.80)
CARD_WHITE, CARD_BLUE, STRIPE_BLACK = (0.93, 0.93, 0.91), (0.07, 0.30, 0.72), (0.03, 0.03, 0.03)
GREEN, RED = (0.10, 0.90, 0.20), (0.95, 0.08, 0.06)

# ---------------------------------------------------------------- scoring constants (see README for sources)
V_MIN, V_MAX = 6 * 0.0254, 60 * 0.0254  # rated card speed 6 to 60 in/s
CONTACT_Q = CARD_T / 2  # head pushed back by at least half a card thickness = a card is against it
LIFT_MAX = 0.0010  # card edge more than 1.0 mm off the floor: the 1.5 mm read core slides off track 2
SINK_MAX = 0.0020  # solver penetration allowance below the floor (one-sided gate)
FACE_COS = EDGE_COS = 0.5
REVERSE_TOL = 0.0001
LED_RED_S = 2.0
HOME_X_TOL, HOME_Z_TOL = 0.0005, 0.002
HOME_ALONG_DEG, HOME_LEAN_DEG = 10.0, 15.0
LINEAR_SPEED, ANGULAR_SPEED = 0.010, 0.5
FALL_DEPTH = 0.020  # let go with its centre this far below the counter's surface: the card fell off the counter

def reader_to_world(p_local):
    """Reader (u across, v along, absolute z) to world; the reader's planar origin is (XR, YR)."""
    return np.array([XR, YR, 0.0]) + READER_R @ np.asarray(p_local, float)


GAP_WORLD = reader_to_world((SLOT_B - CARD_T / 2, 0.0, TOP + R_FLOOR + GAP_Z))
FRONT_WORLD = READER_R @ np.array([-1.0, 0.0, 0.0])


class CardPart(Part):
    @property
    def free(self):
        # Dynamic at runtime; the task has one fixed starting layout, so the card is never jittered.
        return False


def _b(x0, x1, y0, y1, z0, z1, color=None):
    return box(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color=color)


def _reader_base():
    fx0, fx1, ys = -R_W / 2, SLOT_F, Y_STRAIGHT - JUNCTION_GAP
    bx0, bx1, r = SLOT_B, R_W / 2, SLOT_ROUND
    front_end = SLOT_F - FLARE_OPEN - FLARE_T - 0.0002
    back_end = SLOT_B + FLARE_OPEN + FLARE_T + 0.0002
    shapes = [_b(-R_W / 2, R_W / 2, -R_L / 2, R_L / 2, 0.0, R_FLOOR)]  # base and slot floor
    for y0, y1 in ((-ys, -WIN_HALF), (WIN_HALF, ys)):  # front (head-side) wall beside the head window
        shapes += [_b(fx0, fx1 - r, y0, y1, R_FLOOR, R_H), _b(fx1 - r, fx1, y0, y1, R_FLOOR, R_H - r)]
    shapes += [_b(fx0, fx1, -WIN_HALF, WIN_HALF, R_FLOOR, WIN_Z0),  # below the head
               _b(fx0, fx1 - r, -WIN_HALF, WIN_HALF, WIN_Z1, R_H),  # above the head
               _b(fx1 - r, fx1, -WIN_HALF, WIN_HALF, WIN_Z1, R_H - r),
               _b(fx0, WIN_BACK_X, -WIN_HALF, WIN_HALF, WIN_Z0, WIN_Z1),  # closes the window's outer face
               cyl((fx1 - r, 0.0, R_H - r), r, 2 * ys, axis="y"),
               _b(bx0 + r, bx1, -ys, ys, R_FLOOR, R_H), _b(bx0, bx0 + r, -ys, ys, R_FLOOR, R_H - r),
               cyl((bx0 + r, 0.0, R_H - r), r, 2 * ys, axis="y")]
    for s in (-1.0, 1.0):  # end blocks behind the flared mouths
        y0, y1 = sorted((s * Y_STRAIGHT, s * Y_END))
        shapes += [_b(fx0, front_end, y0, y1, R_FLOOR, R_H), _b(back_end, bx1, y0, y1, R_FLOOR, R_H)]
    shapes.append(_b(-0.0120, -0.0050, -0.030, 0.030, R_H, R_H + 0.0002, color=SILVER))  # front marking strip
    return shapes


def _lamp_logic(state):
    return _led_state(state)


HEAD = Link("head", [cyl((SLOT_B - HEAD_R, 0.0, (HEAD_Z0 + HEAD_Z1) / 2), HEAD_R, HEAD_Z1 - HEAD_Z0)],
            Slide("x", (SLOT_B - HEAD_R, 0.0, (HEAD_Z0 + HEAD_Z1) / 2), -HEAD_TRAVEL, 0.0, damping=HEAD_DAMP,
                  spring=HEAD_K, rest=HEAD_REST, armature=HEAD_ARMATURE),
            HEAD_MASS, color=STEEL, friction=HEAD_FRICTION)
READER = Mechanism("reader", (XR, YR, TOP), _reader_base(), [HEAD], color=BLACK, friction=READER_FRICTION,
                   yaw=READER_YAW, contact_offset=READER_CONTACT_OFFSET,
                   lamps=(Lamp("led", LED_CENTER, LED_R, on=RED, off=GREEN),), lamp_logic=_lamp_logic)


def _flares():
    """The four mouth walls: yawed fixed parts whose inner faces run from the slot faces at |y| = 34.75 mm."""
    out = []
    for end, s in (("left", 1.0), ("right", -1.0)):
        for side, x0, sx in (("front", SLOT_F, -1.0), ("back", SLOT_B, 1.0)):
            p0 = np.array((x0, s * Y_STRAIGHT))
            p1 = np.array((x0 + sx * FLARE_OPEN, s * Y_END))
            d = p1 - p0
            length = float(np.linalg.norm(d))
            n = np.array((-d[1], d[0])) / length
            if n[0] * sx < 0:
                n = -n  # outward, away from the slot
            c = (p0 + p1) / 2 + n * FLARE_T / 2
            yaw = math.atan2(-d[0], d[1])
            out.append(Part(f"reader_mouth_{side}_{end}", tuple(reader_to_world((c[0], c[1], TOP))),
                            [box((0.0, 0.0, (R_FLOOR + R_H) / 2), (FLARE_T, length, R_H - R_FLOOR))],
                            color=BLACK, yaw=yaw + READER_YAW, friction=READER_FRICTION,
                            contact_offset=READER_CONTACT_OFFSET))
    return out


MOUTHS = _flares()


def _stand_shapes():
    g, r = S_GAP / 2, S_LIP_R
    shapes = [_b(-S_W / 2, S_W / 2, -S_L / 2, S_L / 2, 0.0, S_FLOOR)]
    for sx in (-1.0, 1.0):
        lo, hi = sorted((sx * g, sx * S_W / 2))
        shapes.append(_b(lo, hi, -S_L / 2, S_L / 2, S_FLOOR, S_LIP_Z))
        lo, hi = sorted((sx * (g + r), sx * S_W / 2))
        shapes.append(_b(lo, hi, -S_L / 2, S_L / 2, S_LIP_Z, S_H))
        shapes.append(cyl((sx * (g + r), 0.0, S_LIP_Z), r, S_L, axis="y"))
    for sy in (-1.0, 1.0):
        y0, y1 = sorted((sy * S_SLOT_LEN / 2, sy * S_L / 2))
        shapes.append(_b(-(g + r), g + r, y0, y1, S_FLOOR, S_H))
    return shapes


STAND = Part("card_stand", (XS, YS, TOP), _stand_shapes(), color=GREY, friction=STAND_FRICTION,
             contact_offset=CARD_CONTACT_OFFSET)
STAND_FLOOR_Z = TOP + S_FLOOR
# The card lies flat on the counter in front of the stand (toward the robot), long side along y, its -y end 202 mm
# short of the counter's right edge (y = -0.65), stripe face up and stripe edge toward +x (away from the robot); here
# the top camera sees it past the right arm's home pose.
CARD_X0, CARD_Y0 = 0.165, -0.405
CARD_START = (CARD_X0, CARD_Y0, TOP + CARD_T / 2 + 0.0001)
CARD = CardPart("card", CARD_START, [box((0.0, 0.0, 0.0), (CARD_W, CARD_L, CARD_T))], CARD_MASS, CARD_WHITE,
                friction=CARD_FRICTION, contact_offset=CARD_CONTACT_OFFSET)
# The closing pads intentionally pinch the card; kinematic replay cannot carry a non-jittered dynamic part.
PRESSED_PARTS = ("card",)
OBJECTS = [STAND, READER, *MOUTHS, CARD]

# ---------------------------------------------------------------- runtime adapter (Isaac/USD imported lazily)
_CARD_CFG = None
_CARD_SPAWN = None
_CARD_TOKEN = object()
_LAMP = {"card": None, "tracker": None, "episode": None}


def _material(look_path, color):
    import isaaclab.sim as sim
    from yam_arena.station import linear_rgb
    look = sim.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.4)
    look.func(look_path, look)
    return look_path


def _visual_box(stage, path, center, size, look):
    import isaaclab.sim as sim
    from pxr import Gf, UsdGeom
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*(float(v) for v in center)))
    cube.AddScaleOp().Set(Gf.Vec3f(*(float(v) for v in size)))
    sim.bind_visual_material(path, look, stage=stage)


def _spawn_card(prim_path, cfg, *args, **kwargs):
    import isaaclab.sim as sim
    from pxr import PhysxSchema
    # A plain Part with a mass: the kit authors a dynamic rigid body with the card's single box collider.
    proxy = Part(CARD.name, CARD.pos, CARD.shapes, CARD.mass, CARD.color,
                 friction=CARD.friction, contact_offset=CARD.contact_offset)
    key = str(prim_path)
    parts._SPECS[key] = proxy
    try:
        body = parts._spawn_part(prim_path, SimpleNamespace(part_name=key), *args, **kwargs)
    finally:
        parts._SPECS.pop(key, None)
    rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(body)
    rigid.CreateSolverPositionIterationCountAttr().Set(32)
    rigid.CreateSolverVelocityIterationCountAttr().Set(4)
    rigid.CreateMaxDepenetrationVelocityAttr().Set(0.5)
    rigid.CreateLinearDampingAttr().Set(0.05)
    rigid.CreateAngularDampingAttr().Set(0.05)
    stage = kwargs.get("stage") or sim.get_current_stage()
    # Visual only (no collider, no mass): the black stripe on the back face and the printed blue front.
    stripe_x = CARD_W / 2 - (STRIPE_FROM + STRIPE_TO) / 2
    _visual_box(stage, f"{prim_path}/stripe", (stripe_x, 0.0, CARD_T / 2 + 0.00001),
                (STRIPE_TO - STRIPE_FROM, CARD_L - 0.001, 0.00002), _material(f"{prim_path}/stripe_look", STRIPE_BLACK))
    _visual_box(stage, f"{prim_path}/front_print", (0.0, 0.0, -CARD_T / 2 - 0.00001),
                (CARD_W - 0.0004, CARD_L - 0.0004, 0.00002), _material(f"{prim_path}/print_look", CARD_BLUE))
    _visual_box(stage, f"{prim_path}/front_panel", (-0.008, -0.020, -CARD_T / 2 - 0.00002),
                (0.016, 0.026, 0.00002), _material(f"{prim_path}/panel_look", CARD_WHITE))
    return body


def _card_object(spec):
    global _CARD_CFG, _CARD_SPAWN
    from isaaclab.sim.utils import clone
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    if _CARD_CFG is None:
        from swipe_config import swipe_config
        _CARD_CFG, _CARD_SPAWN = swipe_config(), clone(_spawn_card)
    # Python floats only: NumPy floats crash Arena's Pose.
    return Object(name=spec.name, object_type=ObjectType.RIGID,
                  spawner_cfg=_CARD_CFG(func=_CARD_SPAWN, card_name=spec.name),
                  initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


def _array(value):
    value = getattr(value, "torch", value)
    return value.detach().cpu().numpy().astype(float)


def _card_state(env):
    scene = env.unwrapped.scene
    origin = _array(scene.env_origins)[0]
    data = scene.rigid_objects[CARD.name].data
    pose = _array(data.root_pose_w)[0]
    p, q = pose[:3] - origin, pose[3:]
    lv, av = _array(data.root_lin_vel_w)[0], _array(data.root_ang_vel_w)[0]
    lo, hi = CARD.aabb(p, PartState(p, q, lv, av, p, p, p).rotation())
    return PartState((lo + hi) / 2, q, lv, av, lo, hi, p)


def _patch_card_scene(scene):
    import functools
    joints._patch_scene(scene)  # compound parts + passive joints (idempotent)
    if getattr(scene, "_card_token", None) is _CARD_TOKEN:
        return
    old_object, old_states = scene.task_object, scene.object_states

    @functools.wraps(old_object)
    def task_object(spec):
        return _card_object(spec) if isinstance(spec, CardPart) else old_object(spec)

    @functools.wraps(old_states)
    def object_states(env, names):
        names = tuple(names)
        out = {}
        if CARD.name in names:
            # Read first, so the reader's lamp sees the card's state from this same instant.
            out[CARD.name] = _LAMP["card"] = _card_state(env)
        out.update(old_states(env, tuple(n for n in names if n != CARD.name)))
        return out

    scene.task_object, scene.object_states = task_object, object_states
    scene._card_token = _CARD_TOKEN


def _install_card_hooks():
    import importlib.machinery

    class Finder(importlib.machinery.PathFinder):
        _card_scene_finder = True

        def find_spec(self, fullname, path=None, target=None):
            if fullname != "yam_arena.scene":
                return None
            spec = importlib.machinery.PathFinder.find_spec(fullname, path)
            if spec is not None and spec.loader is not None:
                spec.loader = parts._SceneLoader(spec.loader, _patch_card_scene)
            return spec

    sys.meta_path[:] = [f for f in sys.meta_path if not getattr(f, "_card_scene_finder", False)]
    sys.meta_path.insert(0, Finder())
    if "yam_arena.scene" in sys.modules:
        _patch_card_scene(sys.modules["yam_arena.scene"])


_install_card_hooks()


# ---------------------------------------------------------------- the read: physical conditions at the head
def card_axes(R):
    """(along the length, across the width from the stripe edge, out of the stripe face) as world vectors."""
    R = np.asarray(R, float)
    return R[:, 1], -R[:, 0], R[:, 2]


def measure(origin, R, lin_vel, ang_vel, head_q):
    """Where the head gap is on the card, how fast the card passes it, and how the card is held there."""
    e_l, e_w, e_n = card_axes(R)
    r = GAP_WORLD - np.asarray(origin, float)
    a = float(e_l @ r) + CARD_L / 2  # head position along the card, from its -e_l end
    d = float(e_w @ r) + CARD_W / 2  # head gap's distance from the stripe (reference) edge
    vp = np.asarray(lin_vel, float) + np.cross(np.asarray(ang_vel, float), r)  # card material at the gap
    return {"a": a, "d": d, "va": -float(e_l @ vp), "lift": GAP_Z - d,
            "face": float(e_n @ FRONT_WORLD),  # stripe face toward the reader's head side
            "edge": float(e_w[2]),  # stripe edge down
            "contact": bool(head_q <= -CONTACT_Q), "head_q": float(head_q)}


def sample_reasons(s):
    """Why a head-contact sample cannot give a clean read (empty when it can)."""
    out = []
    if s["face"] < FACE_COS:
        out.append("stripe not facing the head")
    if s["edge"] < EDGE_COS:
        out.append("stripe edge not down")
    if not -SINK_MAX <= s["lift"] <= LIFT_MAX:
        out.append("track off the head gap (card not riding the floor)")
    return out


class ReadTracker:
    """Splits a run into head-contact episodes (swipe attempts) and judges each one from physical samples.

    An attempt is a run of samples with the head pushed back by the card, plus the samples just before and after.
    An attempt whose span never reaches the encoded region is not counted (nothing was read)."""

    def __init__(self):
        self.token = None
        self.last = None
        self.current = None
        self.attempts = []

    def advance(self, token, sample):
        if token == self.token:
            return
        sample = dict(sample, step=int(token[0]), t=float(token[1]))
        if sample["contact"]:
            if self.current is None:
                self.current = {"before": self.last, "samples": []}
            self.current["samples"].append(sample)
        elif self.current is not None:
            self._finish(self.current, sample)
            self.current = None
        self.last, self.token = sample, token

    def open_attempt(self):
        """A swipe still touching the head when scored: judged on its samples so far if they already span the data;
        otherwise a failed swipe. None when no swipe is open or it has not reached the data."""
        if self.current is None:
            return None
        seq = _sequence(self.current, None)
        if not _reaches_data(seq):
            return None
        if _spans_data(seq):
            reasons = judge(self.current["before"], self.current["samples"], None, open_end=True)
        else:
            reasons = ["swipe still on the head at the end of the run"]
        return {"good": not reasons, "reasons": reasons}

    def _finish(self, att, after):
        seq = _sequence(att, after)
        if not _reaches_data(seq):
            return
        reasons = judge(att["before"], att["samples"], after)
        self.attempts.append({"good": not reasons, "reasons": reasons, "t_end": after["t"]})


def _sequence(att, after):
    return [s for s in [att["before"], *att["samples"], after] if s is not None]


def _reaches_data(seq):
    a = [s["a"] for s in seq]
    return bool(a) and max(a) >= DATA_FROM and min(a) <= DATA_TO


def _spans_data(seq):
    first, last = seq[0]["a"], seq[-1]["a"]
    return (first <= DATA_FROM and last >= DATA_TO) or (first >= DATA_TO and last <= DATA_FROM)


def judge(before, samples, after, open_end=False):
    """Reasons an attempt is not a good read (empty list: a good read). open_end: still on the head at the end."""
    seq = [s for s in [before, *samples, after] if s is not None]
    reasons = []
    if before is None or (after is None and not open_end):
        reasons.append("attempt not bracketed by samples off the head")
    for p, q in zip(seq, seq[1:]):
        if not (q["step"] == p["step"] + 1 and 0.0 < q["t"] - p["t"] <= CONTROL_DT + 1e-6):
            reasons.append("samples not contiguous")
            break
    sign = math.copysign(1.0, seq[-1]["a"] - seq[0]["a"]) if seq[-1]["a"] != seq[0]["a"] else 0.0
    if sign == 0.0 or any((q["a"] - p["a"]) * sign < -REVERSE_TOL for p, q in zip(seq, seq[1:])
                          if max(p["a"], q["a"]) >= DATA_FROM and min(p["a"], q["a"]) <= DATA_TO):
        reasons.append("card reversed on the head")
    if not _spans_data(seq):
        reasons.append("stripe did not pass the head end to end")
    # Pair-average speed: only between consecutive samples both on the span, or on opposite sides of all of it. A pair
    # with one sample on the span is covered by that sample's own speed (the motion past the span's end is not data).
    for p, q in zip(seq, seq[1:]):
        both_on = _on_span(p) and _on_span(q)
        across = min(p["a"], q["a"]) < DATA_FROM and max(p["a"], q["a"]) > DATA_TO
        if (both_on or across) and q["t"] > p["t"]:
            speed = abs(q["a"] - p["a"]) / (q["t"] - p["t"])
            if speed < V_MIN:
                reasons.append("too slow (or stopped) over the stripe")
            if speed > V_MAX:
                reasons.append("too fast over the stripe")
    for s in seq:
        if DATA_FROM <= s["a"] <= DATA_TO:
            if abs(s["va"]) < V_MIN:
                reasons.append("too slow (or stopped) over the stripe")
            if abs(s["va"]) > V_MAX:
                reasons.append("too fast over the stripe")
    # Stripe side, edge and height count while the encoded data passes the head (a reader decodes nothing before the
    # first data bit or after the last): at every sample on the span (the untouched ones before and after included),
    # and where two consecutive samples cross an end of the span, at that end, interpolated linearly between them. So
    # the card reaches and leaves the head along the slot, not over it, whatever the 20 Hz sampling phase.
    for s in seq:
        if _on_span(s):
            reasons += sample_reasons(s)
    for p, q in zip(seq, seq[1:]):
        lo, hi = min(p["a"], q["a"]), max(p["a"], q["a"])
        for end in (DATA_FROM, DATA_TO):
            if lo < end < hi and not (_on_span(p) and _on_span(q)):
                w = (end - p["a"]) / (q["a"] - p["a"])
                reasons += sample_reasons({k: p[k] + w * (q[k] - p[k]) for k in ("face", "edge", "lift")})
    return sorted(set(reasons))


def _on_span(s):
    return DATA_FROM <= s["a"] <= DATA_TO


def _reader_q(reader):
    return float(reader.joints["head"])


def _sample_from_state(card, reader):
    return measure(card.origin, card.rotation(), card.lin_vel, card.ang_vel, _reader_q(reader))


def _sample_from_row(row):
    step, t, p, q, lv, av, head_q = row
    R = PartState.rotation(SimpleNamespace(quat_xyzw=q))
    return (int(step), float(t)), measure(np.asarray(p, float), R, lv, av, head_q)


_TRACK = {"episode": None, "tracker": None}


def read_tracker(objects):
    """The scorer's own tracker (online: per episode; offline: TEST_HISTORY rows, then the current layout)."""
    if _RUN["t"] is None:
        tracker = ReadTracker()
        token = (0, 0.0)
        for row in TEST_HISTORY or ():
            token, sample = _sample_from_row(row)
            tracker.advance(token, sample)
        token = (token[0] + 1, token[1] + CONTROL_DT) if TEST_HISTORY else (0, 0.0)
        tracker.advance(token, _sample_from_state(objects["card"], objects["reader"]))
        return tracker
    if _TRACK["episode"] != _RUN["episode"]:
        _TRACK["episode"], _TRACK["tracker"] = _RUN["episode"], ReadTracker()
    _TRACK["tracker"].advance((_RUN["step"], float(_RUN["t"])), _sample_from_state(objects["card"], objects["reader"]))
    return _TRACK["tracker"]


def read_result(tracker):
    """'clean' (one swipe, a good read), 'retry' (a good read, but not on a lone first swipe) or 'none'."""
    counted = list(tracker.attempts)
    open_attempt = tracker.open_attempt()
    if open_attempt is not None:
        counted.append(open_attempt)
    if len(counted) == 1 and counted[0]["good"]:
        return "clean"
    if any(a["good"] for a in counted):
        return "retry"
    return "none"


def _led_state(reader_state):
    """Status lamp: red for 2 s after a swipe that is not a good read; green otherwise (and before any swipe).

    Its own tracker, fed from the card's simulated pose and velocity (read in the same object-state call) and the
    head's joint; it never touches the scorer's tracker, and the scorer never reads the lamp."""
    try:
        t, card = _RUN["t"], _LAMP["card"]
        if t is None or card is None:
            return {"led": False}
        if _LAMP["episode"] != _RUN["episode"]:
            _LAMP["episode"], _LAMP["tracker"] = _RUN["episode"], ReadTracker()
        tracker = _LAMP["tracker"]
        tracker.advance((_RUN["step"], float(t)), _sample_from_state(card, reader_state))
        last = tracker.attempts[-1] if tracker.attempts else None
        return {"led": bool(last is not None and not last["good"] and t - last["t_end"] < LED_RED_S)}
    except Exception:  # noqa: BLE001 - a lamp must never stop a run
        return {"led": False}


# ---------------------------------------------------------------- end state and violations
def home_terms(card):
    """The card standing in its stand: bottom edge between the slot walls on the slot floor, along the slot."""
    R = card.rotation()
    e_l, e_w, _ = card_axes(R)
    up = e_w if e_w[2] >= 0 else -e_w
    bottom = card.origin - CARD_W / 2 * up
    ends = [bottom + s * CARD_L / 2 * e_l for s in (-1.0, 1.0)]
    in_x = all(abs(p[0] - XS) <= S_GAP / 2 + HOME_X_TOL for p in ends)
    in_y = all(abs(p[1] - YS) <= S_SLOT_LEN / 2 + HOME_X_TOL for p in ends)
    on_floor = all(-HOME_Z_TOL <= p[2] - STAND_FLOOR_Z <= HOME_Z_TOL for p in ends)
    along = abs(e_l[1]) >= math.cos(math.radians(HOME_ALONG_DEG))
    upright = up[2] >= math.cos(math.radians(HOME_LEAN_DEG))
    return {"in_x": bool(in_x), "in_y": bool(in_y), "on_floor": bool(on_floor), "along": bool(along),
            "upright": bool(upright), "home": bool(in_x and in_y and on_floor and along and upright)}


def dropped_now(card):
    """Fallen off the counter: let go with its centre more than 20 mm below the counter's surface. A card lying
    on the counter is where the task starts, and a card held past the counter's edge is still in hand."""
    return bool(card.origin[2] < TOP - FALL_DEPTH and released(card))


def score_terms(objects):
    card = objects["card"]
    tracker = read_tracker(objects)
    dropped = ever("card dropped", dropped_now(card))
    home = home_terms(card)
    still = bool(np.linalg.norm(card.lin_vel) < LINEAR_SPEED and np.linalg.norm(card.ang_vel) < ANGULAR_SPEED)
    done = bool(home["home"] and released(card) and still)
    held = held_for("card home, released and still", done)
    return {"read": read_result(tracker), "dropped": dropped, "done": done, "still_s": held, **home,
            "attempts": [(a["good"], a["reasons"]) for a in tracker.attempts]}


def score(objects):
    terms = score_terms(objects)
    if terms["dropped"] or terms["read"] == "none":
        return 0.0
    finished = terms["done"] and terms["still_s"] + 1e-10 >= STILL_S
    if terms["read"] == "clean":
        return 1.0 if finished else 0.5
    return 0.5 if finished else 0.25


def success(objects):
    return score(objects) >= 1.0


# ---------------------------------------------------------------- history rows for offline layouts and tests
# A row is (step, t, card origin xyz, card quat xyzw, lin vel, ang vel, head joint q); 20 Hz, contiguous steps.
# Fixture quaternions are reader-frame and describe an upright card (identity: stripe toward reader +x, stripe edge
# down, length along reader y); _rows rotates poses and velocities into world coordinates and Q_UPRIGHT turns the
# card's own flat frame up onto its edge.
Q_UPRIGHT = (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))  # flat card frame -> standing on its stripe edge
Q_STAND = Q_UPRIGHT  # world: standing in the stand, stripe toward +x, stripe edge down
Q_TURNED = (0.0, 0.0, 1.0, 0.0)  # card turned 180 deg about vertical: stripe toward the head, stripe edge down
Q_WRONG_SIDE = (0.0, 0.0, 0.0, 1.0)  # stripe toward the back wall
Q_UPSIDE_DOWN = (0.0, 1.0, 0.0, 0.0)  # stripe toward the head but its edge up (half turn about the long axis)
SLOT_CARD_U = SLOT_B - CARD_T / 2  # reader-frame u: card pressed against the far wall by the head (m)
FLOOR_CARD_Z = TOP + R_FLOOR + CARD_W / 2
SWIPE_FROM = -(Y_END - 0.003) - CARD_L / 2  # reader-frame v: leading end 3 mm inside the near mouth (m)
SWIPE_TO = 0.010 + CARD_L / 2  # reader-frame v: trailing end 10 mm past the head (m)
LEFT_IN_READER = reader_to_world((SLOT_CARD_U, SWIPE_TO, FLOOR_CARD_Z))
HEAD_TOUCH = 0.0026  # the card's end meets the head's rounded crown 2.6 mm before its apex


def _smooth(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * (3.0 - 2.0 * u)


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


Q_READER = (0.0, 0.0, math.sin(READER_YAW / 2), math.cos(READER_YAW / 2))


def _world_q(q):
    """A reader-frame fixture quaternion of an upright card as the card's world orientation."""
    return _quat_mul(_quat_mul(Q_READER, q), Q_UPRIGHT)


Q_TURNED_W = _world_q(Q_TURNED)


def _pitched(q, deg):
    """q turned by `deg` about reader u (the card's normal in the slot): an along-track tilt."""
    h = math.radians(deg) / 2
    return _quat_mul((math.sin(h), 0.0, 0.0, math.cos(h)), q)


def _rows(path, q, t0=0.0, step0=1, in_slot=True, u=SLOT_CARD_U):
    """Sample reader-frame v(t), z(t) at 20 Hz into world-frame history rows, until the path returns None."""
    rows, k, h = [], 0, 1e-4
    q_world = _world_q(q)
    while True:
        t = k * CONTROL_DT
        p = path(t)
        if p is None:
            return rows
        v, z = p
        p_lo, p_hi = path(max(t - h, 0.0)) or p, path(t + h) or p
        dt = (t + h) - max(t - h, 0.0)
        vel = (0.0, (p_hi[0] - p_lo[0]) / dt, (p_hi[1] - p_lo[1]) / dt)
        overlap = (v - CARD_L / 2 <= HEAD_TOUCH) and (v + CARD_L / 2 >= -HEAD_TOUCH)
        head_q = -CARD_T if (in_slot and overlap) else 0.0
        rows.append((step0 + k, t0 + t, tuple(reader_to_world((u, v, z))), q_world,
                     tuple(READER_R @ vel), (0.0, 0.0, 0.0), head_q))
        k += 1


def _profile(segments, lift=0.0, z_of_y=None):
    """Reader-frame path: (seconds, v_from, v_to, 'smooth'|'linear'|'hold'); then 0.3 s still."""
    total = sum(s[0] for s in segments) + 0.3

    def path(t):
        if t > total + 1e-9:
            return None
        y = segments[-1][2]
        for dur, y0, y1, kind in segments:
            if t <= dur:
                u = t / dur if dur > 0 else 1.0
                y = y0 + (y1 - y0) * (_smooth(u) if kind == "smooth" else u if kind == "linear" else 0.0)
                break
            t -= dur
        z = FLOOR_CARD_Z + lift if z_of_y is None else z_of_y(y)
        return y, z
    return path


def _chain(*parts_):
    """Concatenate row lists into one contiguous 20 Hz history."""
    out = []
    for rows in parts_:
        for r in rows:
            step = len(out) + 1
            out.append((step, (step - 1) * CONTROL_DT, *r[2:]))
    return out


def swipe_history(kind="clean"):
    """Recorded samples of one or more swipes, for offline layouts (the scored layout is the sample after them)."""
    pre = [(0.3, SWIPE_FROM, SWIPE_FROM, "hold")]
    clean = pre + [(0.55, SWIPE_FROM, SWIPE_TO, "smooth")]
    mid = -CARD_L / 2 + 0.040  # head 40 mm into the card from its leading end
    if kind == "clean":
        return _chain(_rows(_profile(clean), Q_TURNED))
    if kind == "reverse":  # from the far mouth, swiping toward -v: readers read both ways
        back_from, back_to = (Y_END - 0.003) + CARD_L / 2, -0.010 - CARD_L / 2
        return _chain(_rows(_profile([(0.3, back_from, back_from, "hold"), (0.55, back_from, back_to, "smooth")]),
                            Q_TURNED))
    if kind == "wrong_side":
        return _chain(_rows(_profile(clean), Q_WRONG_SIDE))
    if kind == "upside_down":
        return _chain(_rows(_profile(clean), Q_UPSIDE_DOWN))
    if kind in ("high_1p5", "high_0p8"):
        return _chain(_rows(_profile(clean, lift=0.0015 if kind == "high_1p5" else 0.0008), Q_TURNED))
    if kind in ("tilted", "tilted_small"):
        # Nose-up along-track tilt (3 deg, or 0.4 deg), riding on its lowest bottom corner on the floor.
        q = _pitched(Q_TURNED, 3.0 if kind == "tilted" else 0.4)
        R = PartState.rotation(SimpleNamespace(quat_xyzw=_quat_mul(q, Q_UPRIGHT)))
        e_l, e_w, _ = card_axes(R)
        drop = max(abs(float((-CARD_W / 2 * e_w + s * CARD_L / 2 * e_l)[2])) for s in (-1.0, 1.0))
        return _chain(_rows(_profile(clean, z_of_y=lambda y: TOP + R_FLOOR + drop), q))
    if kind == "slow":  # 0.12 m/s, under the rated 6 in/s
        return _chain(_rows(_profile(pre + [((SWIPE_TO - SWIPE_FROM) / 0.12, SWIPE_FROM, SWIPE_TO, "linear")]),
                            Q_TURNED))
    if kind == "fast":  # 1.70 m/s, over the rated 60 in/s (the phase puts one sample on the stripe)
        y_touch = -HEAD_TOUCH - CARD_L / 2
        return _chain(_rows(_profile([(0.3, y_touch - 0.068, y_touch - 0.068, "hold"),
                                      (0.21 / 1.70, y_touch - 0.068, y_touch - 0.068 + 0.21, "linear")]), Q_TURNED))
    if kind == "stop":  # swipes, stops 0.4 s with the head on the stripe, then carries on
        return _chain(_rows(_profile(pre + [(0.35, SWIPE_FROM, mid, "smooth"), (0.4, mid, mid, "hold"),
                                            (0.35, mid, SWIPE_TO, "smooth")]), Q_TURNED))
    if kind == "back":  # pushed until the head is 40 mm in, then pulled back out of the mouth
        return _chain(_rows(_profile(pre + [(0.35, SWIPE_FROM, mid, "smooth"), (0.35, mid, SWIPE_FROM, "smooth")]),
                            Q_TURNED))
    if kind == "nudge":  # the leading edge only reaches the head's lead-in (no data), then one clean swipe
        y_nudge = -CARD_L / 2 + 0.004
        return _chain(_rows(_profile(pre + [(0.3, SWIPE_FROM, y_nudge, "smooth"), (0.3, y_nudge, SWIPE_FROM, "smooth")]
                                     + clean[1:]), Q_TURNED))
    if kind == "retry":  # a pull-back, then a clean swipe
        return _chain(swipe_history("back"), swipe_history("clean"))
    if kind == "double":  # two clean swipes
        return _chain(swipe_history("clean"), swipe_history("reverse"))
    if kind in ("lift_out", "drop_in"):
        # 0.8 m/s along the slot (one sample every 40 mm), the card 30 mm up, clear of the head, where the head is past
        # 55 mm along it (lift_out: it leaves over the head mid-stripe) or short of 25 mm (drop_in: it comes down
        # onto the head mid-stripe). The head is touched only while the card rides the floor.
        v0 = -0.040 - CARD_L / 2
        high = (lambda a: a > 0.055) if kind == "lift_out" else (lambda a: a < 0.025)
        rows = _rows(_profile([(0.3, v0, v0, "hold"), (0.2, v0, v0 + 0.160, "linear")],
                              z_of_y=lambda y: FLOOR_CARD_Z + (0.030 if high(y + CARD_L / 2) else 0.0)), Q_TURNED)
        return _chain([r[:6] + ((r[6] if r[2][2] < FLOOR_CARD_Z + 0.005 else 0.0),) for r in rows])
    if kind == "stuck_mid":  # pushed until the head is 40 mm in, and left there (the head still touched at the end)
        return _chain(_rows(_profile(pre + [(0.35, SWIPE_FROM, mid, "smooth")]), Q_TURNED))
    if kind == "beside":  # the same motion 30 mm in front of the reader: nothing touches the head
        return _chain(_rows(_profile(clean), Q_TURNED, in_slot=False, u=-0.030))
    raise ValueError(f"unknown swipe history {kind!r}")


# ---------------------------------------------------------------- demo: open loop from the known start layout
H_TIP = 0.030  # pad tips 30 mm above the card's bottom edge (about 7 mm above the reader's top while swiping)
TCP_ABOVE_EDGE = H_TIP + PAD_TIP_PAST_TCP
PRESS = 0.0010  # the stroke ends with the edge commanded 1.0 mm into the track floor, so the soft arm keeps it down
SET_PRESS = 0.0006  # the edge is set 0.6 mm into the floor before the stroke, so the soft arm rests it on the floor
LEAD_IN = 0.003
TRAIL_PAST = 0.015  # the stroke ends with the trailing end 15 mm past the head
# The pads pinch the card 26 mm from its centre toward its trailing end, as a hand holds the trailing end. The
# head, the back wall and the floor drag the card below the pads, and a 1.9 N pinch has little grip against turning,
# so the card tips nose-down unless the floor under its edge reaches well ahead of the pads; held this way the floor
# reaches about 40 mm ahead of them until the encoded data has passed the head (14 mm if held at its centre).
GRIP_ALONG = 0.026
# The soft arm strays up to ~4 mm sideways while it moves vertically, so the card goes down beside the reader's end,
# waits, and enters the mouth level with the floor (as a hand does).
ENTRY_GAP = 0.008  # leading end 8 mm short of the reader's near end while the card is lowered and settles
ENTRY_ABOVE = 0.0008  # bottom edge 0.8 mm above the track floor while it slides into the mouth
STAND_HOVER = 0.006  # bottom edge 6 mm above the stand's lips: settle there, then lower slowly into the slot
SWIPE_S = 0.75  # one smoothstep stroke: about 220-300 mm/s commanded over the stripe's data
TRAVEL_Z = TOP + 0.090
PICK_YAW, SWIPE_YAW = -math.pi / 2, 0.0  # pads close along world y at the reader
# A rigid card yawed by e in the 1.06 mm slot wedges once (1.06 - 0.76) mm / tan(e) of it is in the straight track,
# so 0.25 deg over the 69.5 mm track. YAW_TRIM cancels the wrist's steady yaw offset measured in simulation
# (joint 6 friction: 0.06 N m / 10 N m/rad = 0.34 deg). YAW_SEAT approaches rest from the same side, keeping
# that offset the same every run and at 0.8x pace.
YAW_TRIM = math.radians(-0.38)
YAW_SEAT = math.radians(1.0)
PARK_L = (0.100, 0.270, TOP + 0.160)
FINAL_HOLD_S = 4.0
GRIP_S = 1.9  # a smoothstep grip ramp peaks at 1.5/GRIP_S per s; the action guard allows 0.05 per 50 ms tick (0.8x too)


# ---------------------------------------------------------------- pickup from flat
# A 0.76 mm card lying flat cannot be pinched from above. As people do at a counter, the demo slides it with a
# fingertip until one end overhangs the counter's edge, pinches the overhanging end between the pads (one above, one
# below), and stands the card in its stand; the stand pick and the swipe below then follow unchanged.
EDGE_Y = -0.65  # the counter's right edge
OVERHANG = 0.036  # the card's -y end is slid 36 mm past the edge; its centre stays 6.8 mm over the counter
PUSH_YAW = 0.0  # pads open along world y; the +y pad's flat inner face pushes the card's +y end
PUSH_TCP_Z = TOP + 0.014  # TCP height while sliding the card
PUSH_PRESS = 0.0015  # the hand is rolled so the pushing pad's tip is commanded 1.5 mm into the counter and rides on
# it, as a fingertip does, while the far pad stays clear above the card
PUSH_ROLL = math.asin((PUSH_TCP_Z - TOP + PUSH_PRESS - PAD_TIP_PAST_TCP) / -FINGER_OPEN)  # about 3 deg
PUSH_CLEAR = 0.006  # the pushing pad goes down 6 mm beyond the card's +y end
PINCH_PITCH = math.radians(19.0)  # the pinch points along +x, tipped 19 deg down; the pinched card tilts with the pads
PINCH_DEPTH = 0.024  # pad tips 24 mm in from the card's -x long edge, so 30 mm from its stripe edge, as at the stand
PINCH_Y = EDGE_Y - 0.040  # pads centred 40 mm past the edge: the lower finger and its base stay clear of the counter
PINCH_BACK = 0.060  # the open pads start 60 mm back along the approach, then slide over and under the card's end
PINCH_LIFT = 0.060
# Measured in simulation: the slide stops about 2 mm short, so the end pinch holds the card about 2.3 mm deeper than
# planned; the arm stops about 0.6 mm short of its target over the stand, and the card slips about 1 mm in the pads as
# it lands; uncorrected, it seats about 4 mm toward +y of the stand's centre. The stand pick below must find it centred
# (its pinch 26 mm toward the trailing end: 2.6 mm less doubled the card's slip in the pads during the stroke and lost
# the read), so the set-down aims 4.0 mm toward -y. A fixed number in the script, like YAW_TRIM, not feedback.
SEAT_TRIM = 0.0040
PICKUP_Z = TOP + 0.090  # the wrist turns level, and the card upright, with the TCP at least this high


class GripOrientation(float):
    """A waypoint's grip as a full rotation (the level pinch at the counter's edge). Its float value is the closure's
    azimuth, read as a plain yaw by tools that only know straight-down grips."""

    def __new__(cls, matrix):
        m = np.asarray(matrix, float)
        c = m[:, 0]
        azimuth = math.atan2(-c[0], c[1]) if math.hypot(c[0], c[1]) > 0.5 else -math.pi / 2
        obj = super().__new__(cls, azimuth)
        obj.matrix = m.copy()
        return obj


# The scripted demo's waypoints may carry a GripOrientation; plain float yaws keep the straight-down grip.
_ORIGINAL_GRIP_ROTATION = getattr(_motion.grip_rotation, "_original", _motion.grip_rotation)


def _grip_rotation(yaw=0.0):
    matrix = getattr(yaw, "matrix", None)
    if isinstance(matrix, np.ndarray) and matrix.shape == (3, 3):
        return matrix
    return _ORIGINAL_GRIP_ROTATION(yaw)


_grip_rotation._original = _ORIGINAL_GRIP_ROTATION
_motion.grip_rotation = _grip_rotation


def pinch_rotation():
    """Approach along +x tipped PINCH_PITCH down, pads closing nearly vertically: the stand grip turned onto its side."""
    turn = axis_angle_matrix(np.array([0.0, 1.0, 0.0]), PINCH_PITCH - math.pi / 2)
    return turn @ _ORIGINAL_GRIP_ROTATION(PICK_YAW)


def push_rotation():
    """The straight-down grip, pads open along y, rolled PUSH_ROLL about x so the +y pad's tip is the lowest point."""
    return axis_angle_matrix(np.array([1.0, 0.0, 0.0]), -PUSH_ROLL) @ _ORIGINAL_GRIP_ROTATION(PUSH_YAW)


def pickup_targets():
    # The +y pad's inner face at its tip, across y from the TCP, with the fingers open and the hand rolled.
    open_face = -FINGER_OPEN * math.cos(PUSH_ROLL) - PAD_TIP_PAST_TCP * math.sin(PUSH_ROLL)
    end_y = EDGE_Y - OVERHANG + CARD_L  # the card's +y end once its -y end overhangs the edge
    push0 = (CARD_X0, CARD_Y0 + CARD_L / 2 + PUSH_CLEAR - open_face, PUSH_TCP_Z)
    push1 = (CARD_X0, end_y - open_face, PUSH_TCP_Z)
    # Pinched, the card tilts with the pads about its stripe edge, which stays on the counter; the TCP is on the
    # card's mid-plane, its pad tips PINCH_DEPTH in from the card's -x edge.
    b = PINCH_PITCH
    s = CARD_W - PINCH_DEPTH + PAD_TIP_PAST_TCP
    pinch = np.array((CARD_X0 + CARD_W / 2 - s * math.cos(b) + CARD_T / 2 * math.sin(b), PINCH_Y,
                      TOP + s * math.sin(b) + CARD_T / 2 * math.cos(b)))
    back = pinch - PINCH_BACK * pinch_rotation()[:, 2]
    beyond = (EDGE_Y - OVERHANG) - PINCH_Y  # TCP past the card's -y end, along the card
    stand_y = YS - CARD_L / 2 - beyond - SEAT_TRIM  # TCP over the stand with the card centred in it
    return {
        "above_push": (push0[0], push0[1], TOP + 0.060), "push_down": push0, "push_end": push1,
        "push_up": (push1[0], push1[1], TOP + 0.060), "past_edge": (pinch[0], PINCH_Y, TOP + 0.070),
        "pinch_high": (pinch[0], PINCH_Y, PICKUP_Z), "pinch_back": tuple(back), "pinch": tuple(pinch),
        "pinch_lift": tuple(pinch + (0.0, 0.0, PINCH_LIFT)), "upright": (XS, stand_y, PICKUP_Z),
        "set_hover": (XS, stand_y, TOP + S_H + STAND_HOVER + TCP_ABOVE_EDGE),
        "set_place": (XS, stand_y, STAND_FLOOR_Z + 0.001 + TCP_ABOVE_EDGE), "set_rise": (XS, stand_y, TRAVEL_Z),
    }


def demo_targets():
    edge_floor = TOP + R_FLOOR + TCP_ABOVE_EDGE
    entry_v = -Y_END - ENTRY_GAP - CARD_L / 2
    start_v = -(Y_END - LEAD_IN) - CARD_L / 2
    end_v = TRAIL_PAST + CARD_L / 2
    ys = YS + GRIP_ALONG  # in the stand the trailing end (after the quarter turn) is the +y end
    # A radial outward stroke keeps base and wrist yaw still; wrist-pitch lag pushes the supported edge down.
    def point(v, z):  # TCP for the card's centre at reader v
        return tuple(reader_to_world((SLOT_XC, v - GRIP_ALONG, z)))

    return {
        "pick": (XS, ys, STAND_FLOOR_Z + TCP_ABOVE_EDGE), "above_stand": (XS, ys, TRAVEL_Z),
        "above_entry": point(entry_v, TRAVEL_Z), "beside": point(entry_v, edge_floor + ENTRY_ABOVE),
        "entered": point(start_v, edge_floor + ENTRY_ABOVE), "touch": point(start_v, edge_floor - SET_PRESS),
        "swiped": point(end_v, edge_floor - PRESS), "lifted": point(end_v, TRAVEL_Z),
        "hover": (XS, ys, TOP + S_H + STAND_HOVER + TCP_ABOVE_EDGE),
        "place": (XS, ys, STAND_FLOOR_Z + 0.001 + TCP_ABOVE_EDGE),
        "rise": (XS, ys, STAND_FLOOR_Z + CARD_W + 0.050), "back_off": (XS - 0.060, ys, TOP + 0.120),
    }


def pickup(w):
    """Right arm: slide the card until one end overhangs the counter's edge, pinch that end, stand it in the stand."""
    C, O = FINGER_CLOSED, FINGER_OPEN
    P, G, H = pickup_targets(), GripOrientation(pinch_rotation()), GripOrientation(push_rotation())
    return [w(2.0, P["above_push"], O, "over the card's +y end, pads open along y", H),
            w(1.4, P["push_down"], O, "pushing fingertip down on the counter beyond the card's end", H),
            w(0.5, P["push_down"], O, "settle on the counter", H),
            w(6.0, P["push_end"], O, "slide the card until its end overhangs the edge", H),
            w(0.4, P["push_end"], O, "settle at the edge", H),
            w(1.0, P["push_up"], O, "lift the fingertip", H),
            w(1.2, P["past_edge"], O, "move past the counter's edge", PUSH_YAW),
            w(2.0, P["pinch_high"], O, "turn the wrist level, pads above and below", G),
            w(1.0, P["pinch_back"], O, "lower the open pads beside the overhanging end", G),
            w(1.2, P["pinch"], O, "slide the pads over and under the overhanging end", G),
            w(GRIP_S, P["pinch"], C, "pinch the overhanging end", G),
            w(0.3, P["pinch"], C, "settle the end pinch", G),
            w(1.0, P["pinch_lift"], C, "lift the card off the counter", G),
            w(2.6, P["upright"], C, "turn the card upright over the stand", PICK_YAW),
            w(1.0, P["set_hover"], C, "lower the card to just above the stand", PICK_YAW),
            w(0.8, P["set_hover"], C, "settle above the stand", PICK_YAW),
            w(1.6, P["set_place"], C, "set the card into the stand", PICK_YAW),
            w(0.3, P["set_place"], C, "settle in the stand", PICK_YAW),
            w(GRIP_S, P["set_place"], O, "let go in the stand", PICK_YAW),
            w(1.0, P["set_rise"], O, "rise clear of the card", PICK_YAW)]


def demo():
    # Known start layout and time only: no sensing, no object or scorer reads.
    C, O = FINGER_CLOSED, FINGER_OPEN
    T = demo_targets()

    def w(seconds, xyz, finger, label, yaw):
        return Waypoint(seconds * DEMO_PACE, tuple(float(v) for v in xyz), finger, label, yaw)

    right = [*pickup(w),
             w(1.5, T["above_stand"], O, "over the card stand, pads across the card", PICK_YAW),
             w(1.2, T["pick"], O, "lower the open pads beside the card", PICK_YAW),
             w(GRIP_S, T["pick"], C, "pinch the card", PICK_YAW),
             w(0.3, T["pick"], C, "settle the pinch", PICK_YAW),
             w(1.2, T["above_stand"], C, "lift the card out of the stand", PICK_YAW),
             w(1.2, T["above_entry"], C, "carry toward the reader's near end", PICK_YAW),
             w(1.2, T["above_entry"], C, "turn the stripe toward the head side", SWIPE_YAW + YAW_TRIM),
             w(1.6, T["beside"], C, "lower the card level with the track floor", SWIPE_YAW + YAW_TRIM),
             w(0.6, T["beside"], C, "settle short of the mouth", SWIPE_YAW + YAW_TRIM),
             w(0.6, T["beside"], C, "turn the wrist past its mark", SWIPE_YAW + YAW_TRIM + YAW_SEAT),
             w(1.0, T["beside"], C, "seat the wrist on its mark", SWIPE_YAW + YAW_TRIM),
             w(1.0, T["entered"], C, "slide the leading end into the mouth", SWIPE_YAW + YAW_TRIM),
             w(0.6, T["touch"], C, "set the edge on the track floor", SWIPE_YAW + YAW_TRIM),
             w(0.4, T["touch"], C, "settle on the floor", SWIPE_YAW + YAW_TRIM),
             w(SWIPE_S, T["swiped"], C, "swipe through the head in one pass", SWIPE_YAW + YAW_TRIM),
             w(0.4, T["swiped"], C, "stop past the head", SWIPE_YAW + YAW_TRIM),
             w(1.2, T["lifted"], C, "lift the card out of the slot", SWIPE_YAW + YAW_TRIM),
             w(1.4, T["above_stand"], C, "carry back", SWIPE_YAW + YAW_TRIM),
             w(1.2, T["above_stand"], C, "turn the card back", PICK_YAW),
             w(1.0, T["hover"], C, "lower the card to just above the stand", PICK_YAW),
             w(0.8, T["hover"], C, "settle above the stand", PICK_YAW),
             w(1.6, T["place"], C, "lower the card into the stand", PICK_YAW),
             w(0.3, T["place"], C, "settle in the stand", PICK_YAW),
             w(GRIP_S, T["place"], O, "let go", PICK_YAW),
             w(1.2, T["rise"], O, "rise clear of the card", PICK_YAW),
             w(1.0, T["back_off"], O, "back off", PICK_YAW),
             w(FINAL_HOLD_S, T["back_off"], O, "hold clear", PICK_YAW)]
    left = [w(1.5, PARK_L, O, "park", 0.0)]
    end = sum(a.duration for a in right)
    left.append(Waypoint(end - left[0].duration, tuple(float(v) for v in PARK_L), O, "hold clear", 0.0))
    return {"left": left, "right": right}
