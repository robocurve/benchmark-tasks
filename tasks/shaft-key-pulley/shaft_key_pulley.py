"""On the station table: a 25 mm steel shaft in a pillow-block bearing on a pedestal, screwed to the table, its axis
across the table (along y) 65 mm up. The shaft turns freely in the bearing; a hand crank on its left end hangs down, so
at rest the keyway milled in its right end faces the robot. Beside it lie an 8 x 7 x 40 mm parallel key (yellow) and,
standing in a grey cradle on the right, an 80 mm aluminium V-belt pulley whose bore has a keyway, turned about a third
of a right angle away from the top. The left arm turns the crank up so the keyway faces up and holds the shaft there;
the right arm lays the key in the keyway, picks the pulley up by the flats on its hub, turns it so its keyway lines up
with the key, slides it onto the shaft, lets go and pushes it home against the shoulder; the left arm lets the crank
down.

The shaft and its stand are one articulation from ``assets/`` (the stand fixed to the table, the shaft on a lightly
damped revolute joint, the crank handle's sleeve spinning on its pin); the key and the pulley are free rigid bodies;
the cradle is fixed. Every collider is a primitive or a small convex mesh.

Run it:  inspect-robots run --embodiment yam_arena -E task=shaft_key_pulley.py --scorer task_score ...

Like the other task files of this benchmark, it extends the stock task-file format without changing yam_arena:
``UsdPart`` objects are spawned from their own USD files at a full pose with the physics authored there
(``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim starts), and
``Waypoint.rot`` gives the scripted demo full gripper rotations (``yam_arena.demo.ArmPlan.target``).
"""

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from yam_arena import demo as yam_demo
from yam_arena import ik
from yam_arena.constants import FINGER_JOINTS, FINGER_OPEN, GRIPPER, PAD_TIP_PAST_TCP
from yam_arena.station import BASES
from yam_arena.task_file import TABLE_TOP, Asset, ObjectState
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # success() itself needs every success condition held for STILL_S: the run ends when it holds
ASSETS = Path(__file__).resolve().parent / "assets"
MM = 0.001
TAG = "shaft_key_pulley"

# -- the parts (mm in the asset build; m here) -------------------------------------------------------------------------
# Stand frame: origin at the base plate's underside centre, +x along the shaft toward its keyed end. Shaft frame (the
# "Shaft" link): origin on the axis in the bearing's middle, +x along the shaft; its keyway's outward direction is
# KEYWAY_DIR and the keyway's width runs along KEYWAY_SIDE (both in the shaft frame).
SHAFT_R = 12.5 * MM
AXIS_H = 65.0 * MM  # the shaft's axis above the stand's underside
SHOULDER_X = 34.0 * MM  # the collar's face the pulley goes home against
SHAFT_END_X = 80.0 * MM
SLOT_X0 = 38.0 * MM  # the keyway runs from here to the shaft's end
SLOT_FLOOR = 8.5 * MM  # its floor above the axis: DIN 6885 t1 = 4.0 mm deep
SLOT_HALF = 4.1 * MM  # half its width (8.2 mm: 0.1 mm a side around the 8 mm key)
KEYWAY_DIR = np.array([0.0, -1.0, 0.0])
KEYWAY_SIDE = np.array([0.0, 0.0, 1.0])
CRANK_R = 45.0 * MM  # the crank handle's axis from the shaft's
SLEEVE_X = -113.0 * MM  # the handle's sleeve (18 mm square, broken corners, 30 mm long): its middle along the shaft
CRANK_HALF = 15.0 * MM  # the sleeve's half-length, the reach used for the clearance check
SLEEVE_W = 18.0 * MM
# Key frame: origin at its centre, +x along its length, +z up through its height.
KEY_L, KEY_W, KEY_H = 40.0 * MM, 8.0 * MM, 7.0 * MM  # DIN 6885 8 x 7 x 40 for a 25 mm shaft
# Pulley frame: origin at the centre of the face that goes against the shoulder, +z along the bore toward the hub's end,
# +x toward its keyway. Its hub has two flats (normals +-y) 42 mm apart.
BORE_R = 12.75 * MM  # 25.5 mm bore: 0.25 mm a side around the 25 mm shaft
PULLEY_R, PULLEY_LEN = 40.0 * MM, 50.0 * MM
RIM_W = 20.0 * MM
CRADLE_AXIS_H = 50.0 * MM  # the cradled pulley's axis above the cradle's underside
CRADLE_RIM_MID = 10.0 * MM  # its rim's middle (pulley z) over the cradle's centre

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
STAND_XY, STAND_YAW = (0.32, 0.06), -math.pi / 2  # the shaft's +x points to the robot's right (-y)
KEY_XY, KEY_YAW = (0.28, -0.10), -math.pi / 2  # the key lies on the table along y
CRADLE_XY = (0.30, -0.18)
PULLEY_TURN = math.radians(35)  # the cradled pulley's keyway this far from the top (about its axis, toward the robot)


def _rz(a: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), a)


def _rx(a: float) -> np.ndarray:
    return axis_angle_matrix(np.array([1.0, 0.0, 0.0]), a)


def _yaw_xyzw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _matrix_xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    return tuple(float(v) for v in ik.matrix_to_quat_xyzw(np.asarray(r, float)))


STAND_POS = np.array([*STAND_XY, TOP])
STAND_R = _rz(STAND_YAW)
AXIS_W = STAND_POS + STAND_R @ np.array([0.0, 0.0, AXIS_H])  # the shaft's origin (world)
UP_ANGLE = -math.pi / 2  # the shaft's joint angle with the keyway facing straight up


def shaft_rot(angle: float) -> np.ndarray:
    """The shaft link's rotation (world) at joint angle ``angle`` (rad; 0 at rest, crank hanging down)."""
    return STAND_R @ _rx(angle)


def shaft_point(p, angle: float = UP_ANGLE) -> np.ndarray:
    """A point given in the shaft's frame, in the world, the shaft turned to ``angle``."""
    return AXIS_W + shaft_rot(angle) @ np.asarray(p, float)


# The seated key and the home pulley, in the shaft's frame.
KEY_SEAT_R = np.column_stack([[1.0, 0.0, 0.0], np.cross(KEYWAY_DIR, [1.0, 0.0, 0.0]), KEYWAY_DIR])
KEY_SEAT_P = np.array([(SLOT_X0 + SHAFT_END_X) / 2, 0.0, 0.0]) + (SLOT_FLOOR + KEY_H / 2) * KEYWAY_DIR
PULLEY_HOME_R = np.column_stack([KEYWAY_DIR, np.cross([1.0, 0.0, 0.0], KEYWAY_DIR), [1.0, 0.0, 0.0]])
PULLEY_HOME_P = np.array([SHOULDER_X, 0.0, 0.0])

# Start poses (world).
KEY_POS = np.array([*KEY_XY, TOP + KEY_H / 2 + 0.0002])
KEY_R = _rz(KEY_YAW)
CRADLE_POS = np.array([*CRADLE_XY, TOP + 0.0002])
PULLEY_UPRIGHT_R = shaft_rot(UP_ANGLE) @ PULLEY_HOME_R  # bore along -y, keyway up: as it goes on
PULLEY_START_R = axis_angle_matrix(PULLEY_UPRIGHT_R[:, 2], PULLEY_TURN) @ PULLEY_UPRIGHT_R
PULLEY_START_POS = (CRADLE_POS + np.array([0.0, 0.0, CRADLE_AXIS_H + 0.0003])
                    - CRADLE_RIM_MID * PULLEY_UPRIGHT_R[:, 2])


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it, as one rigid body or as an articulation
    (``fixed_base`` pins its root link where it spawns). ``Asset``'s fields only satisfy the task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    fixed_base: bool = False
    kinematic: bool = False

    @property
    def free(self) -> bool:
        return not (self.kinematic or self.fixed_base)


def _tup(v) -> tuple[float, float, float]:
    return tuple(float(x) for x in v)


# Arena places an articulation by its root link: the stand's, whose origin is the stand's.
STAND = UsdPart("shaft_stand", asset="shaft_stand", xy=STAND_XY, extent=0.24,
                usd_path=str(ASSETS / "shaft_stand.usda"), pos=(*STAND_XY, TOP + 0.0002),
                rot_xyzw=_yaw_xyzw(STAND_YAW), articulation=True, fixed_base=True)
KEY = UsdPart("key", asset="key", xy=KEY_XY, extent=KEY_L, mass=0.0174, usd_path=str(ASSETS / "shaft_key.usda"),
              pos=_tup(KEY_POS), rot_xyzw=_matrix_xyzw(KEY_R))
PULLEY = UsdPart("pulley", asset="pulley", xy=tuple(PULLEY_START_POS[:2]), extent=2 * PULLEY_R, mass=0.217,
                 usd_path=str(ASSETS / "pulley.usda"), pos=_tup(PULLEY_START_POS),
                 rot_xyzw=_matrix_xyzw(PULLEY_START_R))
CRADLE = UsdPart("cradle", asset="cradle", xy=CRADLE_XY, extent=0.076, usd_path=str(ASSETS / "pulley_cradle.usda"),
                 pos=_tup(CRADLE_POS), kinematic=True)
OBJECTS = [STAND, KEY, PULLEY, CRADLE]

SOLVER_ITERATIONS = 32
MAX_DEPENETRATION = 0.5  # m/s


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD."""
    import isaaclab.sim as sim_utils

    if spec.kinematic:
        return scene.Object(
            name=spec.name, usd_path=spec.usd_path, object_type=scene.ObjectType.RIGID,
            initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
            spawn_cfg_addon={"func": scene._spawn_fixture,
                             "rigid_props": scene.PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)})
    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION)}
    if spec.articulation:
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            fix_root_link=spec.fixed_base or None, enabled_self_collisions=False,
            solver_position_iteration_count=SOLVER_ITERATIONS, solver_velocity_iteration_count=1)
    return scene.Object(
        name=spec.name,
        usd_path=spec.usd_path,
        object_type=scene.ObjectType.ARTICULATION if spec.articulation else scene.ObjectType.RIGID,
        initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
        spawn_cfg_addon=addon,
    )


@dataclass(frozen=True)
class GripperState:
    """One gripper after a control step (world frame, m): the span of its finger pads along its pointing axis (``back``
    to the pad ``tips``), how far apart the pads are, and the smaller of the two fingers' grip forces (N): each
    finger's drive stiffness times how far something holds it back from its commanded position."""

    back: np.ndarray
    tips: np.ndarray
    opening: float
    contact: float


@dataclass(frozen=True)
class PartPose:
    """One link of an articulation after a control step (world frame, m)."""

    pos: np.ndarray
    quat_xyzw: np.ndarray


PAD_BACK = 0.025  # m: the pads reach this far behind the TCP


def _gripper_states(env) -> dict:
    """Each gripper's ``GripperState`` and the episode's time (s), keyed ``left_gripper``, ``right_gripper`` and
    ``episode_time``."""
    from yam_arena import runtime

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    out = {"episode_time": float(base.episode_length_buf[0]) * float(base.step_dt)}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        axis = runtime.link_poses(robot)["gripper"][:3, 2]
        tcp = runtime.tcp_world(robot) - origin
        q = robot.data.joint_pos.torch[0, robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]].cpu().numpy()
        target = float(base.action_manager.get_term(f"{side}_gripper").processed_actions[0, 0]) * FINGER_OPEN
        held_back = np.clip(target - q, 0.0, None)  # fingers open to -q each; a closing finger is held back
        out[f"{side}_gripper"] = GripperState(
            back=tcp - PAD_BACK * axis,
            tips=tcp + PAD_TIP_PAST_TCP * axis,
            opening=-float(q.sum()),
            contact=min(float(GRIPPER["stiffness"]) * float(held_back.min()), float(GRIPPER["effort"])),
        )
    return out


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        return _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)

    def object_states(env, names):
        """Each object's root, plus the grippers, the episode time, the shaft link's pose and its joint angle."""
        base = env.unwrapped
        origin = base.scene.env_origins[0].detach().cpu().numpy()

        def arr(t):
            return t.torch[0].detach().cpu().numpy().astype(np.float64)

        out = {}
        for name in names:
            data = base.scene[name].data
            pose = arr(data.root_link_pose_w)
            pos = pose[:3] - origin
            out[name] = ObjectState(pos=pos, quat_xyzw=pose[3:], lin_vel=arr(data.root_lin_vel_w),
                                    ang_vel=arr(data.root_ang_vel_w), low=pos.copy(), high=pos.copy())
        out.update(_gripper_states(env))
        stand = base.scene["shaft_stand"]
        i = stand.find_bodies(["Shaft"])[0][0]
        pose = stand.data.body_link_pose_w.torch[0, i].detach().cpu().numpy().astype(np.float64)
        out["shaft"] = PartPose(pos=pose[:3] - origin, quat_xyzw=pose[3:])
        j = stand.find_joints(["ShaftBearing"])[0][0]
        out["shaft_angle"] = float(stand.data.joint_pos.torch[0, j].detach().cpu().numpy())
        return out

    task_object._usd_parts = True
    scene.task_object = task_object
    scene.object_states = object_states


class _AfterSceneImport(importlib.abc.MetaPathFinder):
    """Patches ``yam_arena.scene`` right after it is first imported (it needs Isaac Sim, which starts later)."""

    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        spec = importlib.util.find_spec(fullname)
        run = spec.loader.exec_module

        def exec_module(module):
            run(module)
            _patch_scene(module)

        spec.loader.exec_module = exec_module
        return spec


if "yam_arena.scene" in sys.modules:
    _patch_scene(sys.modules["yam_arena.scene"])
elif not any(isinstance(f, _AfterSceneImport) for f in sys.meta_path):
    sys.meta_path.insert(0, _AfterSceneImport())


# -- scoring ----------------------------------------------------------------------------------------------------------
# The key is in the shaft's keyway: its underside no more than KEY_DEPTH off the keyway's floor (a seated key rests on
# the floor; with the pulley on, the key can rise up to 0.3 mm toward the pulley's keyway roof), within KEY_SIDE of the
# keyway's middle line (the walls hold it to 0.1 mm), along the shaft with its ends inside the keyway (KEY_OVERHANG past
# the shaft's end at most), and lined up with the shaft within KEY_ALIGN.
KEY_DEPTH = 0.0005
KEY_SIDE = 0.0005
KEY_OVERHANG = 0.004
KEY_ALIGN = math.radians(10)
# The key is in the pulley's keyway: its centre out from the bore's axis toward the pulley's keyway by at least
# KEY_IN_HUB_OUT (the key's centre is 12 mm from the axis; anywhere else in the bore it is nearer the axis), within
# KEY_IN_HUB_SIDE of the keyway's middle line, and inside the pulley's length.
KEY_IN_HUB_OUT = 0.009
KEY_IN_HUB_SIDE = 0.0015
# The pulley is on the shaft: its bore's axis within COAXIAL of the shaft's and within COAXIAL_TILT of parallel, and
# its face past STARTED of the shaft's end (started) or within HOME of the shoulder (home). The bore has 0.25 mm a side
# of clearance, so a pulley resting on the shaft is up to 0.25 mm off its axis.
COAXIAL = 0.001
COAXIAL_TILT = math.radians(3)
STARTED = 0.010
HOME = 0.002
FALLEN_Z = -0.02  # m above the table: a part whose centre is lower has fallen off the table
CLEAR = 0.03  # m: when let go, each gripper is at least this far from the key, the pulley and the crank handle
GRIP_SLACK = 0.005
GRIP_FORCE = 0.5
GRIP_OPENING = 0.003
KEY_BOX = (np.array([-KEY_L / 2, -KEY_W / 2, -KEY_H / 2]), np.array([KEY_L / 2, KEY_W / 2, KEY_H / 2]))
PULLEY_BOX = (np.array([-PULLEY_R, -PULLEY_R, 0.0]), np.array([PULLEY_R, PULLEY_R, PULLEY_LEN]))
STILL_S = 1.0  # s: success needs every full-success condition held without a break this long, among them
STILL_MOVE, STILL_TURN = 0.002, math.radians(2.0)  # ... the key, the pulley and the shaft within 2 mm and 2 degrees
# of where they were when the conditions began to hold (judged from the poses: contact jitter in the speeds does not
# count)
STILL_PARTS = ("key", "pulley", "shaft")
TRACE_S = 1.0
STEPS = ("key in the shaft's keyway", "pulley started on the shaft", "pulley home over the key", "released")
POINTS = 0.2  # per step; only success scores 1.0


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))


def _box(pose, box) -> tuple[np.ndarray, np.ndarray]:
    """The world-aligned bounds of ``box`` (low, high corners in the part's frame) at ``pose``."""
    lo, hi = box
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    w = corners @ _rot(pose).T + pose.pos
    return w.min(0), w.max(0)


def _held(bounds, gripper) -> bool:
    """``gripper`` holds the part with world ``bounds``: its pads reach them and both fingers press."""
    if gripper is None or gripper.contact < GRIP_FORCE or gripper.opening < GRIP_OPENING:
        return False
    low, high = bounds[0] - GRIP_SLACK, bounds[1] + GRIP_SLACK
    span = (gripper.back + f * (gripper.tips - gripper.back) for f in np.linspace(0.0, 1.0, 11))
    return any(bool(np.all(q >= low) and np.all(q <= high)) for q in span)


def _held_by_any(objects, bounds) -> bool:
    return any(_held(bounds, objects.get(g)) for g in ("left_gripper", "right_gripper"))


def key_held(objects) -> bool:
    return _held_by_any(objects, _box(objects["key"], KEY_BOX))


def pulley_held(objects) -> bool:
    return _held_by_any(objects, _box(objects["pulley"], PULLEY_BOX))


def in_shaft(objects, point) -> np.ndarray:
    """A world point in the shaft's frame, as the shaft is now."""
    s = objects["shaft"]
    return _rot(s).T @ (np.asarray(point, float) - s.pos)


def key_in_shaft(objects) -> dict:
    """Where the key's centre is in the shaft's keyway (m): ``depth`` above resting on its floor, ``side`` off its
    middle line, ``along`` the shaft; and ``tilt`` (rad) of its length from the shaft's axis."""
    k = objects["key"]
    c = in_shaft(objects, k.pos)
    length = _rot(objects["shaft"]).T @ _rot(k)[:, 0]
    return {"depth": float(c @ KEYWAY_DIR - (SLOT_FLOOR + KEY_H / 2)), "side": float(c @ KEYWAY_SIDE),
            "along": float(c[0]), "tilt": math.acos(min(1.0, abs(float(length[0]))))}


def key_seated(objects) -> bool:
    """The key lies in the shaft's keyway (see KEY_DEPTH and the rest) and no gripper holds it."""
    k = key_in_shaft(objects)
    ends_in = SLOT_X0 + KEY_L / 2 - 0.001 <= k["along"] <= SHAFT_END_X - KEY_L / 2 + KEY_OVERHANG
    return bool(-0.001 < k["depth"] < KEY_DEPTH and abs(k["side"]) < KEY_SIDE and ends_in and k["tilt"] < KEY_ALIGN
                and not key_held(objects))


def pulley_on_shaft(objects) -> dict:
    """The pulley against the shaft: ``face`` (its shoulder face's place along the shaft, m), ``off`` (its bore's
    axis from the shaft's, m), ``tilt`` (rad between the two axes)."""
    p = objects["pulley"]
    o = in_shaft(objects, p.pos)
    z = _rot(objects["shaft"]).T @ _rot(p)[:, 2]
    return {"face": float(o[0]), "off": float(np.hypot(o[1], o[2])), "tilt": math.acos(float(np.clip(z[0], -1, 1)))}


def _coaxial(on) -> bool:
    return bool(on["off"] < COAXIAL and on["tilt"] < COAXIAL_TILT)


def pulley_started(objects) -> bool:
    on = pulley_on_shaft(objects)
    return bool(_coaxial(on) and on["face"] < SHAFT_END_X - STARTED)


def pulley_home(objects) -> bool:
    on = pulley_on_shaft(objects)
    return bool(_coaxial(on) and on["face"] < SHOULDER_X + HOME)


def key_in_hub(objects) -> bool:
    """The key's centre lies in the pulley's keyway (see KEY_IN_HUB_OUT and the rest)."""
    p = objects["pulley"]
    c = _rot(p).T @ (objects["key"].pos - p.pos)
    return bool(c[0] > KEY_IN_HUB_OUT and abs(c[1]) < KEY_IN_HUB_SIDE and 0.0 < c[2] < PULLEY_LEN)


def _dist_box(q, bounds) -> float:
    lo, hi = bounds
    return float(np.linalg.norm(np.maximum(np.maximum(lo - q, 0.0), q - hi)))


def crank_world(objects) -> np.ndarray:
    """The crank handle's sleeve centre (world), the shaft turned as it is now."""
    s = objects["shaft"]
    return np.asarray(s.pos, float) + _rot(s) @ np.array([SLEEVE_X, 0.0, -CRANK_R])


def _clear_of_parts(objects, side: str) -> bool:
    """``side``'s gripper pads are at least CLEAR from the key, the pulley and the crank handle (none touched)."""
    g = objects.get(f"{side}_gripper")
    if g is None:
        return True
    key = _box(objects["key"], KEY_BOX)
    pulley = _box(objects["pulley"], PULLEY_BOX)
    crank = crank_world(objects)
    for f in np.linspace(0.0, 1.0, 11):
        q = g.back + f * (g.tips - g.back)
        gap = min(_dist_box(q, key), _dist_box(q, pulley), float(np.linalg.norm(q - crank)) - CRANK_HALF)
        if gap < CLEAR:
            return False
    return True


def _steps(objects) -> list[bool]:
    """Each of ``STEPS`` as things stand (see ``steps_done``)."""
    key = key_seated(objects)
    started = bool(key and pulley_started(objects))
    home = bool(started and pulley_home(objects) and key_in_hub(objects))
    released = bool(home and not pulley_held(objects) and _clear_of_parts(objects, "left")
                    and _clear_of_parts(objects, "right"))
    return [key, started, home, released]


def _poses(objects) -> dict:
    return {n: (np.array(objects[n].pos, float), np.array(objects[n].quat_xyzw, float)) for n in STILL_PARTS}


def parts_still(objects, anchor) -> bool:
    """The key, the pulley and the shaft are each within STILL_MOVE and STILL_TURN of their poses in ``anchor``."""
    for name, (pos, quat) in _poses(objects).items():
        pos0, quat0 = anchor[name]
        if np.linalg.norm(pos - pos0) > STILL_MOVE:
            return False
        if 2 * math.acos(min(1.0, abs(float(quat0 @ quat)))) > STILL_TURN:
            return False
    return True


class _Progress:
    """The episode's success timing and the trace's state, updated once per control step: since when every
    full-success condition has held without a break (``met_since``, ``None`` while one fails) and the parts' poses when
    they began to hold (``anchor``). Any lapse (a step undone, a violation, or the key, the pulley or the shaft moving
    more than STILL_MOVE or turning more than STILL_TURN from the anchor) restarts the count. Starts over when the
    episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.met_since = None
        self.anchor = None
        self.reported = [False] * len(STEPS)
        self.traced = -1.0

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is None or t == self.time:  # no clock (nothing can be timed) or this step already counted
            return
        if t < self.time:
            self.__init__()
        self.time = t
        met = bool(all(_steps(objects)) and not _violation(objects))
        if met and self.met_since is not None and parts_still(objects, self.anchor):
            return
        if self.met_since is not None:
            print(f"[{TAG}] t={t:.2f}s  success conditions lapsed ("
                  f"{'parts moved' if met else 'a step undone'}): timing restarts", flush=True)
        self.met_since, self.anchor = (t, _poses(objects)) if met else (None, None)
        if met:
            print(f"[{TAG}] t={t:.2f}s  success conditions met: timing {STILL_S:g} s", flush=True)

    def held_s(self) -> float:
        """How long every full-success condition has held without a break (s), 0 while one fails."""
        return 0.0 if self.met_since is None else self.time - self.met_since


_PROGRESS = _Progress()


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``, as things stand: (1) the key in the shaft's keyway, let go; (2) with it, the pulley's bore
    around the shaft; (3) with it, the pulley home against the shoulder with the key inside its keyway too (a pulley
    home without the key earns nothing here); (4) with it, both grippers let go and at least CLEAR from the key, the
    pulley and the crank handle."""
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects)
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):  # in a run: the step timeline, a trace
        k, on = key_in_shaft(objects), pulley_on_shaft(objects)
        kc = _rot(objects["pulley"]).T @ (objects["key"].pos - objects["pulley"].pos)
        line = (f"(shaft {math.degrees(objects['shaft_angle']):.1f} deg; key depth {k['depth'] * 1000:.1f} side "
                f"{k['side'] * 1000:.1f} along {k['along'] * 1000:.1f} mm tilt {math.degrees(k['tilt']):.1f} deg; "
                f"pulley face {on['face'] * 1000:.1f} off {on['off'] * 1000:.1f} mm tilt {math.degrees(on['tilt']):.1f}"
                f" deg; key in hub frame {np.array2string(kc * 1000, precision=1, separator=',')} mm)")
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok in zip(STEPS, done, strict=True) if ok) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  {line}", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {line}", flush=True)
        p.traced = t
    return done


def _violation(objects) -> bool:
    """Scores 0: the key or the pulley off the table, or a state that is not finite."""
    for name in ("key", "pulley"):
        pos = objects[name].pos
        if not np.all(np.isfinite(pos)) or pos[2] < TOP + FALLEN_Z:
            return True
    return not np.all(np.isfinite(objects["shaft"].pos))


def success(objects) -> bool:
    """Every full-success condition has held without a break for STILL_S, counted from the first control step they all
    held: all four steps done (the last one: both grippers let go and clear), no violation, and the key, the pulley and
    the shaft within STILL_MOVE and STILL_TURN of where they were when that second began."""
    p = _PROGRESS
    p.update(objects)
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6)


def score(objects) -> float:
    """1.0 on success (every full-success condition held for STILL_S); otherwise 0.2 per step done (``STEPS``, at most
    0.8, also while the second is being timed); 0 for any violation."""
    done = steps_done(objects)
    if _violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: pointing down, leaning LEFT_LEAN outward (its wide body clear of the crank arm), fingers closing along x on
# the crank handle's sleeve (it spins on its pin, so the gripper keeps one rotation); swing the crank up toward the
# robot until the keyway faces up; hold it there while the right arm works; let it back down; go home.
# Right arm: the key from above, fingers across it, carried over the keyway and let go KEY_DROP above its floor; then
# the pulley by the two flats on its hub (the gripper tilted with the cradled pulley, PULLEY_TURN), lifted out of the
# cradle, turned upright (keyway up, lined up with the key), held coaxial with the shaft past its end and slid on until
# its face is at START_FACE (30 mm on: the key well into the hub's keyway, the pulley's weight over the shaft); let go,
# the fingers set either side of the bore behind the hub's end, and the pulley pushed home against the shoulder (a
# straight push behind the hub lets the pulley follow the shaft; carried on in the hand, the arm's soft wrist tips it
# under the push and it binds in the bore 15 mm short); back off; go home.
SQUEEZE = 0.008  # grips close the fingers this far inside the part's width (~20 N a finger)
LEFT_LEAN = math.radians(-25)  # about world x: the left gripper's body leans toward +y (away from the crank)
SLEEVE_GRIP_OUT = 0.005  # the left pads this far outboard of the sleeve's middle
SLEEVE_TIPS_PAST = 0.004  # ... their tips this far past the sleeve's axis
SLEEVE_HOLD = -(SLEEVE_W - SQUEEZE) / 2
CRANK_STEPS = 6  # waypoints on the crank's quarter turn
CRANK_PAST = math.radians(2.0)  # the crank is turned this far past a quarter turn: the hand holding it gives that much
KEY_TIPS_ABOVE = 0.0015  # the right pad tips this far above the key's underside
KEY_HOLD = -(KEY_W - SQUEEZE) / 2
KEY_RELEASE = -0.005  # 10 mm apart: 1 mm clear of the 8 mm key each side
# The key is 2 mm shorter than its keyway: laid off line toward the shoulder it would land on the keyway's closed end,
# toward the open end it may overhang. It is aimed KEY_AIM_OUT toward the shaft's end from the keyway's middle.
KEY_AIM_OUT = 0.002
KEY_SIDE_IN = 0.006  # the key comes in sideways over this: the arm's joint friction leaves it short on that side
KEY_PRESS = 0.004  # then the closed fingertips press its top this far down (~2 N), seating a key that landed canted
KEY_DROP = 0.0025  # the key's underside this far above the keyway's floor when let go (pad tips 0.7 mm over its edges)
PULLEY_LIFT = 0.04
PRE_SLIDE = 0.010  # the pulley's face this far past the shaft's end before it slides on
PRESS = 0.020  # the push is commanded this far past the shoulder: the arm (about 200 N/m there) presses with ~4 N
START_FACE = 0.050  # the hand slides the pulley on until its face is here (30 mm on), lets go and pushes it home
PUSH_OPEN = -0.016  # 32 mm apart: the fingers push on the hub's end beside the bore
PUSH_REACH = 0.0185  # the fingers' fronts this far ahead of the TCP where they meet the hub's end (they widen upward)
PUSH_TCP_UP = 0.002  # the TCP this far above the shaft's axis while pushing
# Measured: holding the pulley out over the shaft, the right arm's drives give ~6 mm under it (they are tuned for the
# arm's own weight). The pulley is aimed up by SLIDE_AIM_UP to meet the shaft on its axis (it then rides 0.3 mm low).
SLIDE_AIM_UP = 0.006
# Measured on the station: in the hand, the pulley's face sits ~3.8 mm to one side of where the plan puts it and its
# axis is turned ~1.8 degrees about the vertical (the pads let it pivot a little in the lift and turn). The slide is
# aimed across by SLIDE_AIM_SIDE and turned back by SLIDE_YAW so the pulley meets the shaft on its axis.
SLIDE_AIM_SIDE = 0.0038
SLIDE_YAW = math.radians(-1.8)


@dataclass
class Waypoint(yam_demo.Waypoint):
    rot: np.ndarray | None = None  # a full gripper (TCP) rotation; replaces ``yaw`` when set


def _target(self, t: float):
    """``ArmPlan.target`` that also follows a waypoint's ``rot``."""
    prev_pos, prev_rot, prev_finger = self.start_pos, self.start_rot, self.finger_start
    for w in self.waypoints:
        goal = np.asarray(w.pos, dtype=float)
        rot = w.rot if getattr(w, "rot", None) is not None else yam_demo.grip_rotation(w.yaw)
        if t < w.duration:
            s = ik.smoothstep(t / w.duration)
            return (
                prev_pos + s * (goal - prev_pos),
                ik.slerp_matrix(prev_rot, rot, s),
                prev_finger + s * (w.finger - prev_finger),
                w.label,
            )
        t -= w.duration
        prev_pos, prev_rot, prev_finger = goal, rot, w.finger
    return prev_pos, prev_rot, prev_finger, "done"


yam_demo.ArmPlan.target = _target

_Z = np.array([0.0, 0.0, 1.0])


def _tcp_with_tips(point, past: float, rot: np.ndarray) -> np.ndarray:
    """The TCP that puts the pad tips ``past`` beyond ``point`` along the gripper's pointing axis."""
    return np.asarray(point, float) + (past - PAD_TIP_PAST_TCP) * (rot @ _Z)


# Left: the crank.
LEFT_ROT = _rx(LEFT_LEAN) @ yam_demo.grip_rotation(math.pi / 2)  # closing along x, pointing down, leaning


def _sleeve_tcp(angle: float) -> np.ndarray:
    """The left TCP holding the sleeve, the shaft turned to ``angle``."""
    crank = np.array([SLEEVE_X - SLEEVE_GRIP_OUT, 0.0, 0.0]) + _rx(angle) @ np.array([0.0, 0.0, -CRANK_R])
    return _tcp_with_tips(STAND_POS + STAND_R @ (crank + np.array([0.0, 0.0, AXIS_H])), SLEEVE_TIPS_PAST, LEFT_ROT)


# Right: the key.
KEY_ROT = yam_demo.grip_rotation(math.pi / 2)  # closing along x, across the key
KEY_PICK_TCP = _tcp_with_tips(KEY_POS - np.array([0.0, 0.0, KEY_H / 2 - KEY_TIPS_ABOVE]), 0.0, KEY_ROT)
KEY_IN_HAND = KEY_ROT.T @ (KEY_POS - KEY_PICK_TCP)  # the key's centre in the gripper frame
KEY_DROP_POS = shaft_point(KEY_SEAT_P + KEY_AIM_OUT * KEY_SEAT_R[:, 0] + KEY_DROP * KEY_SEAT_R[:, 2])
assert np.allclose(shaft_rot(UP_ANGLE) @ KEY_SEAT_R, KEY_R, atol=1e-9)  # carried without turning it
KEY_DROP_TCP = KEY_DROP_POS - KEY_ROT @ KEY_IN_HAND
KEY_TOP = shaft_point(KEY_SEAT_P + KEY_AIM_OUT * KEY_SEAT_R[:, 0] + KEY_H / 2 * KEY_SEAT_R[:, 2])  # a seated key's top
KEY_PRESS_TCP = _tcp_with_tips(KEY_TOP, KEY_PRESS, KEY_ROT)

# Right: the pulley, by the two flats on its hub: the pads press on them, so the pulley cannot turn about its bore in
# the grip and its keyway stays where the gripper puts it. The gripper points down onto the flats (tilted with the
# pulley while it is cradled, PULLEY_TURN), its pads' middle HUB_GRIP_Z along the bore: the fingers widen above the
# pads, and there they must stay clear of the rim's face (at 20 mm).
FLATS_W = 42.0 * MM
HUB_GRIP_Z = 0.048  # the pads' middle along the bore (pulley z; the hub runs 20..50 mm): the fingers 4 mm off the rim
HUB_TIPS_BELOW = 0.011  # the pad tips this far below the bore's axis (the flats reach 13.6 mm either side of it)
HUB_SQUEEZE = 0.014  # ~35 N a finger: the pulley hangs 25 mm beside the pads and must not pivot in them
HUB_HOLD = -(FLATS_W - HUB_SQUEEZE) / 2
HUB_RELEASE = -0.035  # 70 mm apart: 14 mm clear of each flat
GRIP_ROT_UPRIGHT = yam_demo.grip_rotation(math.pi / 2)  # pointing down, closing along x: across the upright flats
GRIP_ROT_PICK = axis_angle_matrix(PULLEY_UPRIGHT_R[:, 2], PULLEY_TURN) @ GRIP_ROT_UPRIGHT
_hub_tips = np.array([-HUB_TIPS_BELOW, 0.0, HUB_GRIP_Z])  # pulley frame (+x toward the keyway: up when upright)
PULLEY_PICK_TCP = _tcp_with_tips(PULLEY_START_POS + PULLEY_START_R @ _hub_tips, 0.0, GRIP_ROT_PICK)
PH_R = GRIP_ROT_PICK.T @ PULLEY_START_R  # the pulley in the gripper frame
PH_P = GRIP_ROT_PICK.T @ (PULLEY_START_POS - PULLEY_PICK_TCP)


def _holding_pulley(pos, r) -> tuple[np.ndarray, np.ndarray]:
    """The right TCP and rotation that hold the pulley at ``pos`` turned ``r`` (world)."""
    g = np.asarray(r, float) @ PH_R.T
    return np.asarray(pos, float) - g @ PH_P, g


def right_plan(wait_s: float) -> list[Waypoint]:
    """Lay the key in the keyway, then slide the pulley on over it; go home."""
    up = np.array([0.0, 0.0, 1.0])
    lifted = PULLEY_START_POS + PULLEY_LIFT * up
    t_lift, r_lift = _holding_pulley(lifted, PULLEY_START_R)
    t_upright, r_up = _holding_pulley(lifted + 0.02 * up, PULLEY_UPRIGHT_R)
    along = shaft_rot(UP_ANGLE)[:, 0]  # the shaft's axis, toward its end
    side = shaft_rot(UP_ANGLE) @ KEYWAY_SIDE  # across the keyway
    aim = SLIDE_AIM_UP * up + SLIDE_AIM_SIDE * side
    r_slide = axis_angle_matrix(shaft_rot(UP_ANGLE) @ KEYWAY_DIR, SLIDE_YAW) @ PULLEY_UPRIGHT_R
    pre = shaft_point(PULLEY_HOME_P + np.array([SHAFT_END_X - SHOULDER_X + PRE_SLIDE, 0.0, 0.0]))
    t_pre, r_pre = _holding_pulley(pre + aim, r_slide)
    t_start, _ = _holding_pulley(shaft_point(np.array([START_FACE, 0.0, 0.0])) + aim, r_slide)

    def push(face):  # the right TCP with the fingers' fronts against the hub's end, the pulley's face at ``face``
        return shaft_point(np.array([face + PULLEY_LEN + PUSH_REACH, 0.0, 0.0])) + PUSH_TCP_UP * up


    wps = [
        Waypoint(2.5, KEY_PICK_TCP + 0.08 * up, FINGER_OPEN, "above the key", rot=KEY_ROT),
        Waypoint(1.5, KEY_PICK_TCP, FINGER_OPEN, "around the key", rot=KEY_ROT),
        Waypoint(0.8, KEY_PICK_TCP, KEY_HOLD, "grip the key", rot=KEY_ROT),
        Waypoint(1.0, KEY_PICK_TCP + 0.06 * up, KEY_HOLD, "lift it", rot=KEY_ROT),
    ]
    used = sum(w.duration for w in wps)
    if wait_s > used:
        wps.append(Waypoint(wait_s - used, KEY_PICK_TCP + 0.06 * up, KEY_HOLD, "wait for the keyway", rot=KEY_ROT))
    wps += [
        Waypoint(2.0, KEY_DROP_TCP + 0.04 * up + KEY_SIDE_IN * side, KEY_HOLD, "over the keyway", rot=KEY_ROT),
        Waypoint(1.5, KEY_DROP_TCP + 0.006 * up, KEY_HOLD, "in line with the keyway", rot=KEY_ROT),
        Waypoint(1.0, KEY_DROP_TCP, KEY_HOLD, "into the keyway", rot=KEY_ROT),
        Waypoint(0.6, KEY_DROP_TCP, KEY_RELEASE, "let the key go", rot=KEY_ROT),
        Waypoint(0.6, KEY_DROP_TCP + 0.015 * up, KEY_RELEASE, "up from the key", rot=KEY_ROT),
        Waypoint(0.5, KEY_DROP_TCP + 0.015 * up, 0.0, "close the fingers", rot=KEY_ROT),
        Waypoint(1.0, KEY_PRESS_TCP, 0.0, "press the key down", rot=KEY_ROT),
        Waypoint(0.5, KEY_PRESS_TCP, 0.0, "hold it down", rot=KEY_ROT),
        Waypoint(1.0, KEY_PRESS_TCP + 0.05 * up, 0.0, "up", rot=KEY_ROT),
        Waypoint(2.5, PULLEY_PICK_TCP + 0.07 * GRIP_ROT_PICK @ -_Z, FINGER_OPEN, "above the hub", rot=GRIP_ROT_PICK),
        Waypoint(1.5, PULLEY_PICK_TCP, FINGER_OPEN, "around the hub", rot=GRIP_ROT_PICK),
        Waypoint(0.8, PULLEY_PICK_TCP, HUB_HOLD, "grip the flats", rot=GRIP_ROT_PICK),
        Waypoint(1.5, t_lift, HUB_HOLD, "lift it out", rot=r_lift),
        Waypoint(1.5, t_upright, HUB_HOLD, "turn it upright", rot=r_up),
        Waypoint(4.0, t_pre + 0.05 * up, HUB_HOLD, "over the shaft's end", rot=r_pre),
        Waypoint(2.5, t_pre, HUB_HOLD, "in line with the shaft", rot=r_pre),
        Waypoint(1.5, t_pre, HUB_HOLD, "steady", rot=r_pre),
        Waypoint(9.0, t_start, HUB_HOLD, "slide it on", rot=r_pre),
        Waypoint(0.8, t_start, HUB_RELEASE, "let go", rot=r_pre),
        Waypoint(1.2, t_start + 0.035 * along, HUB_RELEASE, "off the hub's end", rot=r_up),
        Waypoint(0.6, t_start + 0.035 * along, PUSH_OPEN, "close to push", rot=r_up),
        Waypoint(1.0, push(START_FACE + 0.004), PUSH_OPEN, "behind the hub", rot=r_up),
        Waypoint(4.0, push(SHOULDER_X - PRESS), PUSH_OPEN, "push it home", rot=r_up),
        Waypoint(0.5, push(SHOULDER_X - PRESS), PUSH_OPEN, "press", rot=r_up),
        Waypoint(1.0, push(SHOULDER_X + 0.03), PUSH_OPEN, "back off", rot=r_up),
        Waypoint(1.0, push(SHOULDER_X + 0.03) + 0.06 * up, PUSH_OPEN, "up", rot=r_up),
        Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]),
    ]
    return wps


def left_plan(release_s: float) -> list[Waypoint]:
    """Take the crank's handle, turn the keyway up and hold it until ``release_s``, let it down, go home."""
    up = np.array([0.0, 0.0, 1.0])
    rest = _sleeve_tcp(0.0)
    arc = [_sleeve_tcp((UP_ANGLE - CRANK_PAST) * k / CRANK_STEPS) for k in range(1, CRANK_STEPS + 1)]
    wps = [
        Waypoint(2.5, rest + 0.06 * up, FINGER_OPEN, "above the crank handle", rot=LEFT_ROT),
        Waypoint(1.5, rest, FINGER_OPEN, "around the handle", rot=LEFT_ROT),
        Waypoint(0.8, rest, SLEEVE_HOLD, "grip the handle", rot=LEFT_ROT),
    ]
    wps += [Waypoint(0.5, p, SLEEVE_HOLD, "turn the keyway up", rot=LEFT_ROT) for p in arc]
    used = sum(w.duration for w in wps)
    wps.append(Waypoint(max(0.1, release_s - used), arc[-1], SLEEVE_HOLD, "hold the shaft", rot=LEFT_ROT))
    wps += [Waypoint(0.5, p, SLEEVE_HOLD, "let the crank down", rot=LEFT_ROT) for p in [*arc[-2::-1], rest]]
    wps += [
        Waypoint(0.8, rest, FINGER_OPEN, "let go", rot=LEFT_ROT),
        Waypoint(1.0, rest + 0.06 * up, FINGER_OPEN, "up", rot=LEFT_ROT),
        Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"]),
    ]
    return wps


def demo():
    """The left arm turns the keyway up and holds the shaft; the right lays the key in and slides the pulley on; the
    left lets the crank down once the right has let go of the pulley. Both go home."""
    left_turned = 2.5 + 1.5 + 0.8 + 0.5 * CRANK_STEPS
    right = right_plan(wait_s=left_turned + 0.3)
    labels = [w.label for w in right]
    let_go = sum(w.duration for w in right[: len(labels) - labels[::-1].index("up")])  # the right hand is done
    return {"left": left_plan(release_s=let_go), "right": right}


# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
