"""On the station table: an IV/venipuncture training arm lying across the table, palm up, its hand on the robot's
right. A needle holder (a 52.3 mm single-use holder, 17 mm bore, 30 mm flange) sits on a 21 G multi-sample needle
whose patient end is already in the arm's vein at the elbow crease, the holder raised 20 degrees off the skin, its
open end toward the hand. Inside the holder the needle's back end (16 mm, under its rubber sleeve) points at the
opening. Two empty 13 x 100 mm evacuated tubes (clear PET, red rubber stoppers, 7 g) lie in a blue tube tray on the
right.

Each tube is filled the way a phlebotomist does it: its stopper end goes into the holder and is pushed onto the back
needle until the needle pierces the stopper's diaphragm and the tube is fully on; it stays on while it fills; then it
is pulled off and laid back in the tray. Pushed on, the stopper folds back the needle's rubber sleeve, a spring of
1.2 to 2.4 N that pushes the tube back off the needle unless it is held on; the needle only slides and levers in the
tissue (the tissue's drag and springs), so pushing the tube on and holding it on while it fills drive the holder,
and the needle in the vein, deeper unless a hand holds the holder steady, braced against the arm. The needle pushed
in or pulled out 5 mm, or levered 5 degrees, would go through or out of the vein: the run scores 0.

The holder is an articulation from ``assets/needle_holder.usda`` (Entry, fixed at the skin entry point; Pivot, the
tissue's spring against levering; Track; Slide, the tissue's drag along the needle; Holder; Sleeve, the needle's
rubber sleeve on a sprung slide). The tubes are rigid
bodies from ``assets/blood_tube.usda``; the arm (``assets/training_arm.usda``) and the tray (``assets/tube_tray.usda``)
are fixtures. Piercing and filling are read from the tube's pose on the needle (declared task code: nothing is moved
or held by it). The drawn blood is shown, not simulated: while a tube is pierced, a dark-red column grows inside it from
the stopper at a measured draw rate and stays when the tube comes off. The column is made of five visual-only pieces
per tube (``assets/blood_column_*.usda``: no collider, no gravity) that wait out of sight under the table; a step
event moves them into the tube, with the tube, as the blood is drawn. Nothing touches them.

Run it:  inspect-robots run --embodiment yam_arena -E task=vacutainer_tube_push.py --scorer task_score ...

The stock task-file format spawns primitives and Arena library models, upright and gripped straight down. This file
extends it without changing yam_arena: ``UsdPart`` objects are spawned from their own USD files at a full pose with
the physics authored there (``yam_arena.scene.task_object`` is wrapped once that module is imported, after Isaac Sim
starts), and ``Waypoint.rot`` gives the scripted demo full gripper rotations (``yam_arena.demo.ArmPlan.target``).
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
from yam_arena.constants import FINGER_JOINTS, FINGER_OPEN, GRIPPER, PAD_TIP_PAST_TCP, READY, SHOULDER, WRIST
from yam_arena.station import BASES
from yam_arena.task_file import TABLE_TOP, Asset, ObjectState
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # success() itself needs every full-success condition held for STILL_S: the run ends when it holds
ASSETS = Path(__file__).resolve().parent / "assets"
MM = 0.001
TAG = "vacutainer_tube_push"

_X, _Y, _Z = np.eye(3)

# -- the tube (its frame: origin at the middle of the stopper's top face, +Z out of the stopper) ----------------------
TUBE_R = 6.5 * MM
TUBE_L = 100.0 * MM
TUBE_MASS = 0.007
TUBE_COM = np.array([0.0, 0.0, -0.048])  # its centre of mass (its frame)
TUBE_IN_R = 5.6 * MM  # its inside radius (0.9 mm PET wall)

# -- the holder (its frame: origin at the needle's skin entry point, +Y along the needle into the arm; a point a
# distance s back from the entry point is at y = -s) -------------------------------------------------------------------
S_BACK = 82.3 * MM  # the holder's open end
S_NEEDLE_TIP = 56.0 * MM  # the back needle's tip
S_DIAPHRAGM = (7.1 * MM, 9.5 * MM)  # the stopper's diaphragm, this far under its top face
S_SEAT = 44.0 * MM  # the stopper's top face when the tube is fully on
HOLDER_R = (10.0 * MM, 15.0 * MM, S_BACK)  # body and flange radius, length (for the release check)
ANGLE = math.radians(20.0)  # the needle's angle to the skin
PIVOT, SLIDE, SLEEVE = "Pivot", "Slide", "Sleeve"
HOLDER_LINK = "Holder"

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
# The training arm lies along y, its hand on the robot's right; the needle points into the arm toward the shoulder
# (+y), so the holder's open end faces the right arm. The tray is on the right, nearer the robot.
ENTRY = np.array([0.30, 0.17, TOP + 0.075])  # the needle's skin entry point (the arm's skin is 75 mm up there)
ARM_POS = np.array([ENTRY[0], ENTRY[1], TOP])
HOLDER_R0 = axis_angle_matrix(_X, -ANGLE)  # the holder's +Y (the needle) points into the arm and down 20 degrees
NEEDLE_DIR = HOLDER_R0 @ _Y
TRAY_POS = np.array([0.19, -0.23, TOP])
TRAY_HALF = (70.0 * MM, 80.0 * MM)  # the tray's outside half-sizes (x, y)
REST_AXIS = 0.016 + TUBE_R / math.cos(math.radians(45.0))  # a tube lying on a lane's V-rests: its axis this high
LANE_X = {1: TRAY_POS[0] + 0.040, 2: TRAY_POS[0]}  # the lanes the tubes start in (the third is empty)
TUBE_LYING_R = axis_angle_matrix(_X, -math.pi / 2)  # a tube lying along +y, its stopper toward the holder


def holder_point(s: float, slide: float = 0.0) -> np.ndarray:
    """The point on the holder's axis a distance ``s`` back from the needle's entry point (the holder as inserted,
    pushed ``slide`` deeper)."""
    return ENTRY + (slide - s) * NEEDLE_DIR


def tube_start(k: int) -> np.ndarray:
    """Tube ``k``'s origin (its stopper's top face) lying on its lane's rests, its middle over the tray's middle."""
    z = TOP + REST_AXIS + 0.0003
    return np.array([LANE_X[k], TRAY_POS[1] + TUBE_L / 2, z])


def _xyzw(r: np.ndarray) -> tuple[float, float, float, float]:
    return tuple(float(v) for v in ik.matrix_to_quat_xyzw(r))


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it: one rigid body, a fixture (``kinematic``),
    or an articulation (``fixed_base`` pins its root link where it spawns). ``Asset``'s fields only satisfy the
    task-file checks."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    fixed_base: bool = False
    kinematic: bool = False
    no_gravity: bool = False


def _part(name, usd, pos, rot=np.eye(3), extent=0.1, mass=None, **kw) -> UsdPart:
    return UsdPart(name, asset=name, xy=(float(pos[0]), float(pos[1])), extent=extent, mass=mass,
                   usd_path=str(ASSETS / usd), pos=tuple(float(v) for v in pos), rot_xyzw=_xyzw(rot), **kw)


# Arena places an articulation by its root link: the holder's is its Entry, at the needle's entry point.
ARM = _part("training_arm", "training_arm.usda", ARM_POS, extent=0.58, kinematic=True)
TRAY = _part("tube_tray", "tube_tray.usda", TRAY_POS, extent=0.16, kinematic=True)
HOLDER = _part("holder", "needle_holder.usda", ENTRY, HOLDER_R0, extent=0.09, articulation=True, fixed_base=True)
TUBES = ("tube_1", "tube_2")
TUBE_1 = _part("tube_1", "blood_tube.usda", tube_start(1), TUBE_LYING_R, extent=TUBE_L, mass=TUBE_MASS)
TUBE_2 = _part("tube_2", "blood_tube.usda", tube_start(2), TUBE_LYING_R, extent=TUBE_L, mass=TUBE_MASS)

# -- the drawn blood (declared task code: shown, not simulated; its mass is not added) ----------------------------------
DRAW_ML = 6.0  # mL: each tube's draw (a 13 x 100 mm, 6 mL evacuated tube)
DRAW_RATE = 5.0 / 7.15  # mL/s: 5 mL evacuated tubes filled in 7.15 s on average through 21 G needles (Fujii 2013)
BLOOD_FROM = 13.0 * MM  # the column starts at the stopper's plug, this far under the stopper's top face ...
BLOOD_PER_ML = 1e-6 / (math.pi * TUBE_IN_R ** 2)  # ... and is this long per mL (10.2 mm; 6 mL: 61 mm)
BLOOD_PIECES = (48, 24, 12, 6, 6)  # mm: each tube's column pieces (assets/blood_column_<mm>.usda), stacked from the
# stopper to show any length (the pieces overlap; a column shorter than 6 mm hides the rest of its piece in the plug)
BLOOD_HIDDEN_Z = TOP - 0.25  # the pieces wait here, under the table, out of every camera's sight


def blood_name(tube: str, i: int) -> str:
    return f"{tube}_blood_{i + 1}"


def blood_hidden(tube: str, i: int) -> np.ndarray:
    return np.array([TRAY_POS[0] - 0.06 + 0.03 * i, TRAY_POS[1] + (0.05 if tube == TUBES[0] else -0.05),
                     BLOOD_HIDDEN_Z])


BLOOD = [_part(blood_name(tube, i), f"blood_column_{mm:02d}.usda", blood_hidden(tube, i), extent=mm * MM,
               no_gravity=True) for tube in TUBES for i, mm in enumerate(BLOOD_PIECES)]
OBJECTS = [ARM, TRAY, HOLDER, TUBE_1, TUBE_2, *BLOOD]

SOLVER_ITERATIONS = 128  # the holder's articulation (and what touches it): at 32 the 0.3 g sleeve, pushed by its
# 1.2-2 N spring, sank 3 mm into the stopper's diaphragm hole and caught there, so pulling the tube off dragged the
# needle out; at 128 it rests on the stopper's cone
MAX_DEPENETRATION = 0.2  # m/s


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD."""
    import isaaclab.sim as sim_utils

    if spec.kinematic:
        return scene.Object(
            name=spec.name, usd_path=spec.usd_path, object_type=scene.ObjectType.RIGID,
            initial_pose=scene.Pose(position_xyz=spec.pos, rotation_xyzw=spec.rot_xyzw),
            spawn_cfg_addon={"func": scene._spawn_fixture,
                             "rigid_props": scene.PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)})
    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION,
                                                             disable_gravity=True if spec.no_gravity else None)}
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


def _arr(t) -> np.ndarray:
    return t.torch[0].detach().cpu().numpy().astype(np.float64)


def object_states(env, names):
    """Each object's root pose (the holder's is its fixed Entry; bounds are left at the root: Arena's geometry queries
    take rigid objects only), the holder link's pose as ``holder_link``, the needle's ``pivot`` (rad) and ``slide`` (m)
    in the tissue, how far its sleeve is folded back (``sleeve``, m; traced only), the grippers and the episode time."""
    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    out = {}
    for name in names:
        data = base.scene[name].data
        pose = _arr(data.root_link_pose_w)
        pos = pose[:3] - origin
        out[name] = ObjectState(pos=pos, quat_xyzw=pose[3:], lin_vel=_arr(data.root_lin_vel_w),
                                ang_vel=_arr(data.root_ang_vel_w), low=pos.copy(), high=pos.copy())
    art = base.scene["holder"]
    i = art.find_bodies([HOLDER_LINK])[0][0]
    pose = art.data.body_link_pose_w.torch[0, i].detach().cpu().numpy().astype(np.float64)
    pos = pose[:3] - origin
    out["holder_link"] = ObjectState(pos=pos, quat_xyzw=pose[3:], lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                                     low=pos.copy(), high=pos.copy())
    ids = art.find_joints([PIVOT, SLIDE, SLEEVE], preserve_order=True)[0]
    q = art.data.joint_pos.torch[0, ids].detach().cpu().numpy().astype(np.float64)
    out["pivot"], out["slide"], out["sleeve"] = float(q[0]), float(q[1]), float(q[2])
    out.update(_gripper_states(env))
    return out


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        return _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)

    stock_cfg_callback = scene.env_cfg_callback

    def env_cfg_callback(control_hz=None):
        """yam_arena's environment settings, plus a reset event that puts the needle's joints at 0 (as inserted) and a
        step event (every control step) that shows the drawn blood in the tubes."""
        configure = stock_cfg_callback(control_hz)

        def with_start(env_cfg):
            from isaaclab.managers import EventTermCfg, SceneEntityCfg

            env_cfg = configure(env_cfg)
            env_cfg.events.holder_start = EventTermCfg(func=_joints_to_zero, mode="reset",
                                                       params={"asset_cfg": SceneEntityCfg("holder")})
            env_cfg.events.blood_step = EventTermCfg(func=_blood_step, mode="interval", interval_range_s=(0.0, 0.0),
                                                     is_global_time=False)
            return env_cfg

        return with_start

    task_object._usd_parts = True
    scene.task_object = task_object
    scene.object_states = object_states
    scene.env_cfg_callback = env_cfg_callback


def _joints_to_zero(env, env_ids, asset_cfg) -> None:
    """Reset event: the needle's joints at 0 and still. The tissue springs need no writing: Arena gives task objects no
    actuators, so the drives keep the targets in the USD (0)."""
    import torch

    if env_ids is None:
        return
    art = env.scene[asset_cfg.name]
    names = list(art.joint_names)
    zero = torch.zeros((len(env_ids), len(names)), device=art.device)
    ids = list(range(len(names)))
    art.write_joint_position_to_sim_index(position=zero, joint_ids=ids, env_ids=env_ids)
    art.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(zero), joint_ids=ids, env_ids=env_ids)


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
PIERCED_S = S_NEEDLE_TIP - S_DIAPHRAGM[1] - 0.5 * MM  # the stopper's top face at most this far back: the needle's
# tip is through the diaphragm (46 mm)
ON_AXIS = 1.0 * MM  # the tube's axis within this of the needle's (it is on the needle)
ON_TILT = math.radians(6.0)
FILL_S = DRAW_ML / DRAW_RATE  # s pierced without a break: the tube has drawn its 6 mL and is filled (8.6 s)
SLIDE_MAX = 5.0 * MM  # the needle pushed in or pulled out this far in the tissue: through or out of the vein (0)
PIVOT_MAX = math.radians(5.0)  # the needle levered this far in the tissue (0)
TRAY_IN = (TRAY_HALF[0] - 3.0 * MM, TRAY_HALF[1] - 3.0 * MM)  # a tube's middle inside the tray's walls
TRAY_LYING = math.radians(15.0)  # its axis within this of level
TRAY_RESTING = REST_AXIS + 4.0 * MM  # its axis at most this high: resting on the rests, not held
CLEAR = 0.03  # m: both grippers' pads at least this far from the tubes and the holder: nothing holds them
FALLEN_Z = -0.02  # m above the table: a part whose origin is lower has fallen off the table
STILL_S, STILL_MOVE, STILL_TURN = 1.0, 0.002, math.radians(2.0)  # success: every condition held 1 s, the parts
# within 2 mm and 2 degrees of where they were when that second began
TRACE_S = 1.0
STEPS = ("a tube filled", "a filled tube back in the tray", "both tubes filled", "both filled tubes back in the tray")
POINTS = 0.2  # per step; only success scores 1.0


def _rot(state) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(state.quat_xyzw, float))


def _pads(gripper) -> list[np.ndarray]:
    return [gripper.back + f * (gripper.tips - gripper.back) for f in np.linspace(0.0, 1.0, 11)]


def _segment_distance(p, a, b) -> float:
    ab = b - a
    t = min(1.0, max(0.0, float((p - a) @ ab) / float(ab @ ab)))
    return float(np.linalg.norm(p - (a + t * ab)))


def on_needle(objects, tube: str) -> dict:
    """Where tube ``tube`` is on the holder's axis: ``s`` how far its stopper's top face is back from the needle's
    entry point (m; 44 mm is fully on, 56 mm touches the needle's tip), ``off`` its axis from the holder's (m) and
    ``tilt`` its axis from the holder's (rad)."""
    h, t = objects["holder_link"], objects[tube]
    rh = _rot(h)
    q = rh.T @ (t.pos - h.pos)
    axis = rh.T @ _rot(t)[:, 2]
    if not (np.all(np.isfinite(q)) and np.all(np.isfinite(axis))):
        return {"s": math.inf, "off": math.inf, "tilt": math.pi}
    return {"s": float(-q[1]), "off": float(math.hypot(q[0], q[2])),
            "tilt": float(math.acos(min(1.0, max(-1.0, axis[1]))))}


def pierced(objects, tube: str) -> bool:
    """The tube is on the needle (coaxial) and pushed on far enough that the needle's tip is through its stopper's
    diaphragm: blood flows into it."""
    n = on_needle(objects, tube)
    return bool(n["s"] <= PIERCED_S and n["off"] <= ON_AXIS and n["tilt"] <= ON_TILT)


def _tube_ends(t) -> tuple[np.ndarray, np.ndarray]:
    axis = _rot(t)[:, 2]
    return t.pos - (TUBE_L - TUBE_R) * axis, t.pos - TUBE_R * axis


def in_tray(objects, tube: str) -> bool:
    """The tube lies in the tray: its middle inside the tray's walls, level (within 15 degrees) and resting on the
    tray's troughs (not held up)."""
    t = objects[tube]
    axis = _rot(t)[:, 2]
    mid = t.pos - (TUBE_L / 2) * axis
    d = mid - TRAY_POS
    return bool(abs(d[0]) <= TRAY_IN[0] and abs(d[1]) <= TRAY_IN[1] and abs(float(axis[2])) <= math.sin(TRAY_LYING)
                and d[2] <= TRAY_RESTING)


def tube_free(objects, tube: str) -> bool:
    """No gripper's pads within CLEAR of the tube's surface."""
    a, b = _tube_ends(objects[tube])
    for side in ("left_gripper", "right_gripper"):
        g = objects.get(side)
        if g is not None and any(_segment_distance(q, a, b) - TUBE_R < CLEAR for q in _pads(g)):
            return False
    return True


def holder_free(objects) -> bool:
    """No gripper's pads within CLEAR of the holder (its axis from the needle's hub to its open end, as a 15 mm
    radius)."""
    h = objects["holder_link"]
    rh = _rot(h)
    a = h.pos + rh @ np.array([0.0, -0.020, 0.0])
    b = h.pos + rh @ np.array([0.0, -S_BACK, 0.0])
    for side in ("left_gripper", "right_gripper"):
        g = objects.get(side)
        if g is not None and any(_segment_distance(q, a, b) - HOLDER_R[1] < CLEAR for q in _pads(g)):
            return False
    return True


class _Progress:
    """What has happened so far this episode: each tube's unbroken time pierced, its blood and whether it has filled,
    whether the needle has been moved too far in the tissue, and since when every full-success condition has held
    without a break (``met_since``, with the parts' poses then). Starts over when the episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.pierced_since = {t: None for t in TUBES}
        self.filled = {t: False for t in TUBES}
        self.drawn = {t: 0.0 for t in TUBES}  # mL of blood in each tube (shown; it stays when the tube comes off)
        self.shown = set()  # the blood pieces in a tube (the others wait under the table)
        self.moved = False
        self.met_since = None  # when every full-success condition last started holding (episode time, s) ...
        self.anchor = None  # ... and the tubes' and the holder's poses then
        self.reported = [False] * len(STEPS)
        self.traced = -1.0

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        last = self.time
        if t is not None:
            if t == self.time:
                return
            if t < self.time:
                self.__init__()
                last = None
            self.time = t
        for tube in TUBES:
            if pierced(objects, tube):
                if self.pierced_since[tube] is None:
                    self.pierced_since[tube] = t if t is not None else 0.0
                elif t is not None and last is not None:  # still pierced: blood flows in at the draw rate
                    self.drawn[tube] = min(DRAW_ML, self.drawn[tube] + DRAW_RATE * (t - last))
                if t is not None and t - self.pierced_since[tube] >= FILL_S - 1e-6:
                    self.filled[tube] = True
            else:
                self.pierced_since[tube] = None
        self.moved |= bool(abs(objects["slide"]) > SLIDE_MAX or abs(objects["pivot"]) > PIVOT_MAX
                           or not np.isfinite(objects["slide"]) or not np.isfinite(objects["pivot"]))
        if t is not None:
            if not _conditions_met(objects, self):
                self.met_since = self.anchor = None
            elif self.met_since is None or not _near(objects, self.anchor):
                self.met_since = t  # (re)start the second, from these poses
                self.anchor = {n: (np.array(objects[n].pos, float), np.array(objects[n].quat_xyzw, float))
                               for n in (*TUBES, "holder_link")}

    def held_s(self) -> float:
        return -1.0 if self.met_since is None else self.time - self.met_since


_PROGRESS = _Progress()


def _back_in_tray(objects, tube: str) -> bool:
    return bool(_PROGRESS.filled[tube] and in_tray(objects, tube) and tube_free(objects, tube)
                and not pierced(objects, tube))


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``: the filled steps once reached (a tube pierced for FILL_S without a break), the tray steps as
    things stand (a filled tube lying in the tray, no gripper within 3 cm of it)."""
    p = _PROGRESS
    p.update(objects)
    filled = sum(p.filled.values())
    back = sum(_back_in_tray(objects, tube) for tube in TUBES)
    done = [filled >= 1, back >= 1, filled >= 2, back >= 2]
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):  # in a run: the step timeline, a trace
        parts = []
        for tube in TUBES:
            n = on_needle(objects, tube)
            parts.append(f"{tube}: s {n['s'] * 1000:.1f} mm off {n['off'] * 1000:.2f} mm tilt "
                         f"{math.degrees(n['tilt']):.1f} deg pierced {pierced(objects, tube)} blood "
                         f"{p.drawn[tube]:.1f} mL filled {p.filled[tube]} in tray {in_tray(objects, tube)}")
        sleeve = f", sleeve folded {objects['sleeve'] * 1000:.1f} mm" if "sleeve" in objects else ""
        line = (f"(needle: slide {objects['slide'] * 1000:+.2f} mm, pivot {math.degrees(objects['pivot']):+.2f} deg"
                f"{sleeve}; " + "; ".join(parts) + ")")
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok in zip(STEPS, done, strict=True) if ok) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  {line}", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {line}", flush=True)
        p.traced = t
    return done


def _near(objects, anchor) -> bool:
    """The tubes and the holder within ``STILL_MOVE`` and ``STILL_TURN`` of the poses in ``anchor``."""
    for name, (pos, quat) in anchor.items():
        s = objects[name]
        if np.linalg.norm(np.asarray(s.pos, float) - pos) > STILL_MOVE:
            return False
        if 2 * math.acos(min(1.0, abs(float(np.dot(quat, np.asarray(s.quat_xyzw, float)))))) > STILL_TURN:
            return False
    return True


def _broken(objects) -> bool:
    """A tube or the holder off the table, or a state that is not finite."""
    for name in (*TUBES, "holder_link"):
        s = objects[name]
        if not (np.all(np.isfinite(s.pos)) and np.all(np.isfinite(s.quat_xyzw))) or s.pos[2] < TOP + FALLEN_Z:
            return True
    return False


def _conditions_met(objects, p) -> bool:
    """Every full-success condition at this step: both tubes filled and back in the tray (off the needle, no gripper
    within 3 cm), no gripper within 3 cm of the holder, and no violation."""
    return bool(all(p.filled.values()) and all(_back_in_tray(objects, tube) for tube in TUBES)
                and holder_free(objects) and not p.moved and not _broken(objects))


def _violation(objects) -> bool:
    """Scores 0: the needle pushed in or pulled out more than 5 mm, or levered more than 5 degrees, at any time; a tube
    off the table; a state that is not finite."""
    _PROGRESS.update(objects)
    return bool(_PROGRESS.moved or _broken(objects))


def success(objects) -> bool:
    """Every full-success condition has held without a break for ``STILL_S``, counted from the first control step they
    all held: both tubes filled and back in the tray, no gripper within 3 cm of either tube or the holder, no violation,
    and the tubes and the holder within ``STILL_MOVE`` and ``STILL_TURN`` of where they were when that second began."""
    p = _PROGRESS
    p.update(objects)
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6)


def score(objects) -> float:
    """1.0 on success (every full-success condition held for ``STILL_S``); otherwise 0.2 per step done (``STEPS``, at
    most 0.8, also while that second is being timed); 0 for any violation."""
    done = steps_done(objects)
    if _violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    return round(POINTS * sum(done), 6)


# -- the blood shown in the tubes (declared task code) -------------------------------------------------------------------


def blood_pieces(length: float) -> list:
    """Where each of ``BLOOD_PIECES`` goes for a column ``length`` (m) long from the stopper's plug: how far its near
    end is from the plug (m), or None (it waits under the table). Longest first, each ending where the last began
    (or at the plug, overlapping), so together they cover exactly 0..length; a column shorter than the shortest piece
    puts that piece partly inside the stopper's plug, which hides it."""
    out = [None] * len(BLOOD_PIECES)
    if length <= 1e-5:
        return out
    sizes = [mm * MM for mm in BLOOD_PIECES]
    order = sorted(range(len(sizes)), key=lambda i: -sizes[i])
    end = length  # the column is covered from here to its far end
    for i in order:
        if end <= 1e-9:
            break
        if sizes[i] <= length:
            out[i] = max(0.0, end - sizes[i])
            end = out[i]
    if end > 1e-9:
        out[order[-1]] = length - sizes[order[-1]]
    return out


def blood_poses(objects, tube: str) -> list:
    """Each of ``tube``'s blood pieces' pose and velocity, (position, quat_xyzw, linear and angular velocity), or
    None when it waits under the table: moving with the tube (the velocity of the tube's point it sits at)."""
    t = objects[tube]
    r = _rot(t)
    com = t.pos + r @ TUBE_COM
    out = []
    for mm, start in zip(BLOOD_PIECES, blood_pieces(_PROGRESS.drawn[tube] * BLOOD_PER_ML), strict=True):
        if start is None:
            out.append(None)
            continue
        p = t.pos - (BLOOD_FROM + start + mm * MM / 2) * r[:, 2]
        w = np.asarray(t.ang_vel, float)
        out.append((p, np.asarray(t.quat_xyzw, float), np.asarray(t.lin_vel, float) + np.cross(w, p - com), w))
    return out


def _put(base, name: str, pos, quat_xyzw, lin_vel, ang_vel) -> None:
    """Moves a blood piece (no collider, no gravity) to a pose with a velocity, in the simulator."""
    import torch

    origin = base.scene.env_origins[0].detach().cpu().numpy()
    asset = base.scene[name]
    pose = asset.data.root_link_pose_w.torch.clone()
    pose[:, :3] = torch.as_tensor(np.asarray(pos, float) + origin, dtype=pose.dtype, device=pose.device)
    pose[:, 3:] = torch.as_tensor(np.asarray(quat_xyzw, float), dtype=pose.dtype, device=pose.device)
    asset.write_root_pose_to_sim_index(root_pose=pose)
    vel = torch.zeros_like(asset.data.default_root_vel.torch)
    vel[:, :3] = torch.as_tensor(np.asarray(lin_vel, float), dtype=vel.dtype, device=vel.device)
    vel[:, 3:] = torch.as_tensor(np.asarray(ang_vel, float), dtype=vel.dtype, device=vel.device)
    asset.write_root_velocity_to_sim_index(root_velocity=vel)


def _blood_step(env, env_ids) -> None:
    """Step event (every control step, after the physics): the fill (``_Progress``), then each tube's blood column:
    its pieces moved into the tube with the tube's velocity (so they ride along with it until the next step), the
    rest kept under the table."""
    base = env.unwrapped
    objects = object_states(env, (*TUBES, "holder"))
    _PROGRESS.update(objects)
    for tube in TUBES:
        for i, want in enumerate(blood_poses(objects, tube)):
            name = blood_name(tube, i)
            if want is not None:
                _put(base, name, *want)
                _PROGRESS.shown.add(name)
            elif name in _PROGRESS.shown:
                _put(base, name, blood_hidden(tube, i), (0.0, 0.0, 0.0, 1.0), np.zeros(3), np.zeros(3))
                _PROGRESS.shown.discard(name)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: comes down beside the holder from above, leaned 55 degrees toward the hand (its wrist over the upper arm,
# clear of the right arm), its fingers open across the holder's body; rests its fingertips on the skin either side of
# the holder, pressing lightly (the anchor a phlebotomist's hand takes on the patient's arm), then closes on the
# holder's body and holds it there until both tubes are done; lets go, lifts off, goes home.
# Right arm, for each tube: from above, fingers across the tube 30 mm from its closed end, lifts it off its lane's
# rests, turns it (pointing in from the near side, 50 degrees forward) in line with the holder, slides its stopper end
# into the holder's bore up to the needle, pushes it on until it is fully on and holds it there while it fills (both
# commanded as the pose that presses with a set force, from the arm's joint stiffness: the arm is soft), pulls it off,
# lays it back on its lane's rests, lets go, lifts off.
SQUEEZE = 0.008  # grips close the fingers this far inside the part's width (about 20 N a finger)
TUBE_HOLD = -(2 * TUBE_R - SQUEEZE) / 2
TUBE_OPEN = -(2 * TUBE_R + 0.024) / 2  # around the tube before gripping and after letting go
HOLDER_HOLD = -(2 * HOLDER_R[0] - SQUEEZE) / 2
HOLDER_OPEN = -(2 * HOLDER_R[0] + 0.016) / 2
GRIP_FROM_BOTTOM = 0.030  # m: the right pads across the tube this far from its closed end
TUBE_IN_PADS = 0.0  # m: the tube's axis this far past the TCP toward the pad tips (on the TCP: further out, the tube
# slid 5 mm in the pads under the push)
TRAY_PHI = 0.0  # the right gripper straight down over the tray
HOLDER_PHI = math.radians(50.0)  # ... and pointing in from the near side at the holder, 50 degrees forward
RIGHT_FLIP = -1  # which way round the right gripper's pads lie along the tube (its wrist's range)
LEFT_LEAN = math.radians(55.0)  # the left gripper leaned toward the hand
LEFT_FLIP = 1
LEFT_S = 0.050  # m: the left TCP over the holder's axis this far back from the entry point ...
LEFT_IN = 0.0066  # ... and this far along its pointing axis (its fingertips' corners on the skin)
LEFT_TOUCH = 0.015  # m: then commanded this far lower, open: its fingertips press lightly on the skin (the arm is
# soft: about 4 N)
LIFT = 0.12  # m: a tube carried this high over the tray
PRE_S = S_BACK + 0.020  # the stopper's face 20 mm behind the holder's open end, in line with it
TOUCH_S = S_NEEDLE_TIP + 0.002  # ... slid into the bore to just short of the needle's tip
PUSH_FORCE = 4.0  # N: the push is commanded as the pose that, with the arm's joint stiffness, presses this hard
# along the needle at the seat (and keeps the tube in line: the soft wrist would tilt it)
HOLD_FORCE = 3.0  # N: ... and while it fills (more than the sleeve's push-back, about 2 N there)
SEAT_AIM = 0.001  # m: the push aims this far past the seat
PULL_TO = S_BACK + 0.045  # m: the pull is commanded to here (the stopper's face 45 mm clear of the open end)
PUSH_S, PULL_S = 2.5, 2.0


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


def tube_grip_rot(axis, phi: float) -> np.ndarray:
    """The right gripper's rotation holding a tube whose axis (toward its stopper) is ``axis``: the pads along the
    tube, pointing down in the vertical plane through the tube, then turned ``phi`` toward +x about the tube."""
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    down = np.cross(axis, _X)
    down /= np.linalg.norm(down)
    if down[2] > 0:
        down = -down
    d = math.cos(phi) * down + math.sin(phi) * _X
    d -= (d @ axis) * axis
    d /= np.linalg.norm(d)
    y = RIGHT_FLIP * axis
    return np.column_stack([np.cross(y, d), y, d])


def tube_tcp(origin, axis, rot) -> np.ndarray:
    """The right TCP holding a tube whose stopper face is at ``origin`` and axis ``axis``, the gripper at ``rot``."""
    axis = np.asarray(axis, float)
    grip = np.asarray(origin, float) - (TUBE_L - GRIP_FROM_BOTTOM) * axis
    return grip - TUBE_IN_PADS * rot[:, 2]


_KQ = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)  # N m/rad: the arm joints' PD stiffness


def _ik(side: str, pos, rot) -> np.ndarray:
    q4 = ik.matrix_to_quat_xyzw(rot)
    best = None
    for seed in (np.array(READY, float), np.array([0.6, 1.2, 1.0, -0.6, 0.8, -0.6]),
                 np.array([0.4, 1.8, 1.6, -1.0, 0.6, -0.6])):
        q, pe, re = ik.solve(pos, q4, seed, base_pos=BASES[side], iters=200)
        if best is None or pe + 0.1 * re < best[1]:
            best = (q, pe + 0.1 * re)
    return best[0]


def braced(side: str, pos, rot, force, point):
    """The TCP command (position, rotation) that ends at ``pos``, ``rot`` while the held part pushes back on the
    gripper with ``-force`` (N, world) at ``point``: the arm's joints are springs (their PD stiffness), so under that
    load the gripper gives by the compliance J K^-1 J^T times the wrench; the command is moved the other way. From
    the robot's model only (no simulator state)."""
    pos, rot = np.asarray(pos, float), np.asarray(rot, float)
    q = _ik(side, pos, rot)
    p0, r0 = ik.tcp_pose(q, BASES[side])
    jac = np.zeros((6, 6))
    for i in range(6):
        dq = q.copy()
        dq[i] += 1e-6
        p1, r1 = ik.tcp_pose(dq, BASES[side])
        jac[:3, i] = (p1 - p0) / 1e-6
        jac[3:, i] = ik._rot_error(r1, r0) / 1e-6
    force = np.asarray(force, float)
    wrench = np.concatenate([force, np.cross(np.asarray(point, float) - pos, force)])
    d = jac @ ((jac.T @ wrench) / _KQ)
    ang = float(np.linalg.norm(d[3:]))
    turn = axis_angle_matrix(d[3:] / ang, ang) if ang > 1e-12 else np.eye(3)
    return pos + d[:3], turn @ rot


def left_rot() -> np.ndarray:
    d = np.array([0.0, -math.sin(LEFT_LEAN), -math.cos(LEFT_LEAN)])
    c = LEFT_FLIP * _X
    return np.column_stack([c, np.cross(d, c), d])


def left_plan(t_release: float) -> tuple[list[Waypoint], float]:
    """Anchor on the arm beside the holder and hold the holder until ``t_release``; let go, lift off, go home. Also
    returns when the holder is held."""
    rot = left_rot()
    at = holder_point(LEFT_S) + LEFT_IN * rot[:, 2]
    wps = [
        Waypoint(2.5, at - 0.06 * rot[:, 2], HOLDER_OPEN, "over the holder", rot=rot),
        Waypoint(1.5, at, HOLDER_OPEN, "fingertips to the skin", rot=rot),
        Waypoint(0.8, at - LEFT_TOUCH * _Z, HOLDER_OPEN, "fingertips on the skin", rot=rot),
        Waypoint(0.8, at - LEFT_TOUCH * _Z, HOLDER_HOLD, "close on the holder", rot=rot),
    ]
    t_held = sum(w.duration for w in wps)
    wps += [
        Waypoint(max(0.2, t_release - t_held), at - LEFT_TOUCH * _Z, HOLDER_HOLD, "keep it steady", rot=rot),
        Waypoint(0.8, at - LEFT_TOUCH * _Z, HOLDER_OPEN, "let go", rot=rot),
        Waypoint(1.2, at - 0.06 * rot[:, 2], HOLDER_OPEN, "lift off", rot=rot),
        Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"]),
    ]
    return wps, t_held


def _tube_cycle(k: int, t_wait: float) -> list[Waypoint]:
    """Tube ``k``: pick it from its lane, line it up with the holder (waiting until ``t_wait`` into the cycle if the
    holder is not held yet), push it on, hold it while it fills, pull it off, lay it back in its lane, let go."""
    start = tube_start(k)
    lying = TUBE_LYING_R[:, 2]
    r_tray = tube_grip_rot(lying, TRAY_PHI)
    r_hold = tube_grip_rot(NEEDLE_DIR, HOLDER_PHI)
    pick = tube_tcp(start, lying, r_tray)
    up = pick + np.array([0.0, 0.0, LIFT])

    def at(s):
        return tube_tcp(holder_point(s), NEEDLE_DIR, r_hold)

    def pressed(s, force):
        return braced("right", at(s), r_hold, force * NEEDLE_DIR, holder_point(s))

    push_p, push_r = pressed(S_SEAT - SEAT_AIM, PUSH_FORCE)
    hold_p, hold_r = pressed(S_SEAT - SEAT_AIM, HOLD_FORCE)

    wps = [
        Waypoint(2.0, pick + np.array([0.0, 0.0, 0.07]), TUBE_OPEN, f"over tube {k}", rot=r_tray),
        Waypoint(1.2, pick, TUBE_OPEN, f"around tube {k}", rot=r_tray),
        Waypoint(0.7, pick, TUBE_HOLD, f"grip tube {k}", rot=r_tray),
        Waypoint(1.2, up, TUBE_HOLD, f"lift tube {k}", rot=r_tray),
        Waypoint(2.5, at(PRE_S) - 0.04 * NEEDLE_DIR + np.array([0, 0, 0.03]), TUBE_HOLD, "turn it toward the holder",
                 rot=r_hold),
        Waypoint(1.2, at(PRE_S), TUBE_HOLD, "in line with the holder", rot=r_hold),
    ]
    t_lined = sum(w.duration for w in wps)
    wps += [
        Waypoint(max(0.2, t_wait - t_lined), at(PRE_S), TUBE_HOLD, "wait for the holder to be held", rot=r_hold),
        Waypoint(2.0, at(TOUCH_S), TUBE_HOLD, "into the holder", rot=r_hold),
        Waypoint(PUSH_S, push_p, TUBE_HOLD, "push it onto the needle", rot=push_r),
        Waypoint(0.8, hold_p, TUBE_HOLD, "ease off", rot=hold_r),
        Waypoint(FILL_S + 0.6, hold_p, TUBE_HOLD, "hold it while it fills", rot=hold_r),
        Waypoint(PULL_S, at(PULL_TO), TUBE_HOLD, "pull it off", rot=r_hold),
        Waypoint(2.5, up, TUBE_HOLD, "back over its lane", rot=r_tray),
        Waypoint(1.5, pick + np.array([0.0, 0.0, 0.002]), TUBE_HOLD, "lay it in its lane", rot=r_tray),
        Waypoint(0.7, pick + np.array([0.0, 0.0, 0.002]), TUBE_OPEN, "let go", rot=r_tray),
        Waypoint(1.0, pick + np.array([0.0, 0.0, 0.07]), TUBE_OPEN, "lift off", rot=r_tray),
    ]
    return wps


def right_plan(t_held: float) -> list[Waypoint]:
    """Tube 1, then tube 2, then home."""
    wps = _tube_cycle(1, t_held + 0.3) + _tube_cycle(2, 0.0)
    return wps + [Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"])]


def demo():
    """The left arm anchors on the arm and holds the holder; the right fills tube 1 and tube 2 in turn once the holder
    is held; after the right gripper has let go of the second tube and lifted off, the left lets go. Both go home."""
    _, t_held = left_plan(0.0)
    right = right_plan(t_held)
    t_done = sum(w.duration for w in right[:-1])
    left, _ = left_plan(t_done)
    return {"left": left, "right": right}


# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
