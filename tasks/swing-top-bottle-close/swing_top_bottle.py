"""On the station table: an open swing-top (flip-top) glass bottle, 190 mm tall and 65 mm across, 300 g. A wire yoke
round its neck carries the bail lever's pivot; the lever (two 6 mm wire arms, a 12 mm crank and a red thumb paddle
70 mm out) rests down along the bottle's right side, and the porcelain stopper hangs from the wire link beside the
shoulder, under the lever's arms. The bottle stands free on the table: it tips over or slides if the lever is pushed
without it being held.

The left arm holds the bottle by its body. The right arm picks the stopper up by its knob, carries it up and over and
sets it on the mouth (the lever follows it up through the link), lets go, then takes the lever's paddle and pushes the
lever down: the crank swings under the pivot, stretching the link (the gasket and wire give 1.3 mm), past centre, and
the stretched link then holds the lever on its stop and the stopper on the mouth. Both hands let go and go back.

The bottle is one articulation from ``assets/swing_top_bottle.usda``, not fixed to the world: Bottle (root),
BailPivot - Lever, LinkPivot - Link, SealSlide (the link's sprung give) - Carrier, StopperHinge - Stopper.

Run it:  inspect-robots run --embodiment yam_arena -E task=swing_top_bottle.py --scorer task_score ...

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
from yam_arena.constants import FINGER_JOINTS, FINGER_OPEN, GRIPPER, PAD_TIP_PAST_TCP
from yam_arena.station import BASES
from yam_arena.task_file import TABLE_TOP, Asset, ObjectState
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # success() itself needs every success condition held for STILL_S: the run ends when it holds
ASSETS = Path(__file__).resolve().parent / "assets"
MM = 0.001
TAG = "swing_top_bottle"

# -- the bottle (its frame: origin at the middle of the base, z up the bottle's axis, the lever closing down its -x
# side, the lever's pivot axis along y) -------------------------------------------------------------------------------
BODY_R, BODY_TOP, SHOULDER_TOP = 32.5 * MM, 112.0 * MM, 150.0 * MM  # body, then the shoulder curving in to the neck
NECK_R, LIP_R, LIP_TOP = 13.5 * MM, 15.0 * MM, 190.0 * MM  # the neck, the mouth's bead; the bottle is 190 mm tall
PIVOT = np.array([0.0, 0.0, 162.0]) * MM  # the lever's pivot axis (along y)
CRANK, LEVER = 12.0 * MM, 70.0 * MM  # pivot to the link's hook; pivot to the paddle's middle
BETA = math.radians(55.0)  # the lever's arm leads its crank by this much
PADDLE_ALONG, PADDLE_ACROSS, PADDLE_THICK = 20.0 * MM, 58.0 * MM, 12.0 * MM
LINK_LEN = 50.6845 * MM  # the link, hook to hinge, unstretched (from the asset's dimensions)
HINGE_SEATED = np.array([0.0, 0.0, 202.0]) * MM  # the stopper's hinge (its frame's origin) when seated
KNOB_LO, KNOB_HI = 2.0 * MM, 22.0 * MM  # the knob along the stopper's axis from its hinge; 16 mm across
KNOB_WIDTH = 16.0 * MM
PSI_L0 = math.radians(-149.23697)  # the crank's direction at joint 0 (from +z toward +x), stopper seated, link taut
PSI_U0 = math.radians(6.95556)  # the link's direction at joint 0
CENTRE = math.radians(30.76303)  # BailPivot with the crank straight under the pivot (the link stretched 1.32 mm)
CLOSED = math.radians(45.76303)  # BailPivot on its stop, 15 degrees past centre
JOINTS = ("BailPivot", "LinkPivot", "SealSlide", "StopperHinge")
# The open bottle (written by a reset event; Isaac Lab starts joints at 0, which is the stopper seated): the lever down
# on its stop, the stopper hanging from the link beside the shoulder, under the lever's arms.
START_JOINTS = {"BailPivot": CLOSED, "LinkPivot": math.radians(65.21687), "SealSlide": 0.0,
                "StopperHinge": math.radians(-110.97988)}

# -- layout (m; x forward, y to the robot's left, from the midpoint between the arm bases) ----------------------------
# The bottle's -x side (where the lever and the stopper hang) faces the robot's right arm.
BOTTLE_XY = (0.42, 0.06)
BOTTLE_YAW = math.pi / 2
BOTTLE_POS = np.array([*BOTTLE_XY, TOP])
BOTTLE_ROT = axis_angle_matrix(np.array([0.0, 0.0, 1.0]), BOTTLE_YAW)

_X, _Y, _Z = np.eye(3)


def _ry(a: float) -> np.ndarray:
    return axis_angle_matrix(_Y, a)


def _dir(psi: float) -> np.ndarray:
    """The unit vector at ``psi`` from +z toward +x (bottle frame)."""
    return np.array([math.sin(psi), 0.0, math.cos(psi)])


def linkage(bail: float, link: float, seal: float, hinge: float) -> dict:
    """The closure's points in the bottle frame for joint values (rad, rad, m, rad): the link's hook, the stopper's
    hinge and axis, the lever's arm direction and the paddle's middle."""
    psi_l = PSI_L0 - bail
    hook = PIVOT + CRANK * _dir(psi_l)
    hinge_pt = hook + (LINK_LEN + seal) * _dir(PSI_U0 - bail - link)
    arm = _dir(psi_l + BETA)
    return {"hook": hook, "hinge": hinge_pt, "axis": _dir(-(bail + link + hinge)), "arm": arm,
            "paddle": PIVOT + LEVER * arm, "push": np.array([-arm[2], 0.0, arm[0]])}


def solve_linkage(hinge_pt, axis_psi: float = 0.0) -> tuple[float, float, float]:
    """(BailPivot, LinkPivot, StopperHinge) that put the stopper's hinge at ``hinge_pt`` (bottle frame, link
    unstretched) with its axis at ``axis_psi``: the branch on which the lever closes down the -x side."""
    v = np.asarray(hinge_pt, float) - PIVOT
    dist = math.hypot(v[0], v[2])
    c = (dist * dist + CRANK * CRANK - LINK_LEN * LINK_LEN) / (2 * dist * CRANK)
    psi_l = math.atan2(v[0], v[2]) - math.acos(max(-1.0, min(1.0, c)))
    hook = PIVOT + CRANK * _dir(psi_l)
    u = np.asarray(hinge_pt, float) - hook
    bail = PSI_L0 - psi_l
    bail = (bail + math.pi) % (2 * math.pi) - math.pi
    link = (PSI_U0 - math.atan2(u[0], u[2])) - bail
    hinge = -axis_psi - bail - link
    wrap = lambda a: (a + math.pi) % (2 * math.pi) - math.pi  # noqa: E731
    return bail, wrap(link), wrap(hinge)


def bottle_radius(z: float) -> float:
    """The glass's radius at height ``z`` (bottle frame)."""
    if z < 0.0 or z > LIP_TOP:
        return 0.0
    if z <= BODY_TOP:
        return BODY_R
    if z <= SHOULDER_TOP:
        return NECK_R + (BODY_R - NECK_R) * math.cos(math.pi / 2 * (z - BODY_TOP) / (SHOULDER_TOP - BODY_TOP))
    return NECK_R if z < 182.0 * MM else LIP_R


def to_world(p, pos=BOTTLE_POS, rot=BOTTLE_ROT) -> np.ndarray:
    return np.asarray(pos, float) + rot @ np.asarray(p, float)


# -- objects ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class UsdPart(Asset):
    """A USD file spawned at a full pose with the physics authored in it, as one rigid body or as an articulation.
    ``Asset``'s fields only satisfy the task-file checks. ``fixed``: an articulation whose root link is fixed to the
    world (it then has no mass for the task-file checks, like a fixed object)."""

    usd_path: str = ""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    fixed: bool = False

    @property
    def free(self) -> bool:
        return not self.fixed


# Arena places an articulation by its root link: the bottle's is its glass body, whose origin is the base's middle.
BOTTLE = UsdPart("bottle", asset="bottle", xy=BOTTLE_XY, extent=0.19, mass=0.30,
                 usd_path=str(ASSETS / "swing_top_bottle.usda"), pos=(*BOTTLE_XY, TOP),
                 rot_xyzw=(0.0, 0.0, math.sin(BOTTLE_YAW / 2), math.cos(BOTTLE_YAW / 2)), articulation=True)
OBJECTS = [BOTTLE]

SOLVER_ITERATIONS = 128  # position iterations per step for the articulation: with 32, PhysX let the 100 N/mm seal spring
# tip the seated stopper off the mouth instead of stretching past ~0.4 mm; at 128 the closure stretches as designed
MAX_DEPENETRATION = 0.2  # m/s


def _spawn(scene, spec: UsdPart):
    """An Arena object for ``spec``, with its physics as authored in the USD."""
    import isaaclab.sim as sim_utils

    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=MAX_DEPENETRATION)}
    if spec.articulation:
        addon["articulation_props"] = sim_utils.ArticulationRootPropertiesCfg(
            solver_position_iteration_count=SOLVER_ITERATIONS, solver_velocity_iteration_count=1,
            enabled_self_collisions=True, fix_root_link=True if spec.fixed else None)
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


def _patch_scene(scene) -> None:
    if getattr(scene.task_object, "_usd_parts", False):
        return
    stock = scene.task_object

    def task_object(spec):
        return _spawn(scene, spec) if isinstance(spec, UsdPart) else stock(spec)

    def object_states(env, names):
        """The bottle's root (its glass body), the grippers, the episode time, the closure's four joints and the
        stopper's pose."""
        base = env.unwrapped
        origin = base.scene.env_origins[0].detach().cpu().numpy()
        out = {}

        def arr(t):
            return t.torch[0].detach().cpu().numpy().astype(np.float64)

        for name in names:
            data = base.scene[name].data
            pose = arr(data.root_link_pose_w)
            pos = pose[:3] - origin
            out[name] = ObjectState(pos=pos, quat_xyzw=pose[3:], lin_vel=arr(data.root_lin_vel_w),
                                    ang_vel=arr(data.root_ang_vel_w), low=pos.copy(), high=pos.copy())
        out.update(_gripper_states(env))
        bottle = base.scene["bottle"]
        ids = bottle.find_joints(list(JOINTS), preserve_order=True)[0]
        q = bottle.data.joint_pos.torch[0, ids].detach().cpu().numpy().astype(np.float64)
        out["bail"], out["link"], out["seal"], out["hinge"] = (float(v) for v in q)  # rad, rad, m, rad
        i = bottle.find_bodies(["Stopper"])[0][0]
        pose = bottle.data.body_link_pose_w.torch[0, i].detach().cpu().numpy().astype(np.float64)
        out["stopper_pos"], out["stopper_quat"] = pose[:3] - origin, pose[3:]
        return out

    stock_cfg_callback = scene.env_cfg_callback

    def env_cfg_callback(control_hz=None):
        """yam_arena's environment settings, plus a reset event that opens the bottle: Isaac Lab starts an
        articulation's joints at 0 (here: the stopper on the mouth, the lever up)."""
        configure = stock_cfg_callback(control_hz)

        def with_start(env_cfg):
            from isaaclab.managers import EventTermCfg, SceneEntityCfg

            env_cfg = configure(env_cfg)
            env_cfg.events.bottle_open = EventTermCfg(func=_start_joints, mode="reset",
                                                      params={"asset_cfg": SceneEntityCfg("bottle")})
            return env_cfg

        return with_start

    task_object._usd_parts = True
    scene.task_object = task_object
    scene.object_states = object_states
    scene.env_cfg_callback = env_cfg_callback


def _start_joints(env, env_ids, asset_cfg) -> None:
    """Reset event: the lever down on its stop, the stopper hanging from its link beside the shoulder."""
    import torch

    if env_ids is None:
        return
    bottle = env.scene[asset_cfg.name]
    ids = bottle.find_joints(list(START_JOINTS), preserve_order=True)[0]
    start = torch.tensor([[*START_JOINTS.values()]], device=bottle.device).repeat(len(env_ids), 1)
    bottle.write_joint_position_to_sim_index(position=start, joint_ids=ids, env_ids=env_ids)
    bottle.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(start), joint_ids=ids, env_ids=env_ids)


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
UPRIGHT = math.radians(5.0)  # the bottle within this of upright
KNOCKED = math.radians(45.0)  # tilted this far, the bottle is knocked over (scores 0)
PLACE = 0.02  # m: the bottle's base within this of where it started is at its place
SEAT_RADIAL, SEAT_DROP, SEAT_RISE = 2.5 * MM, 2.0 * MM, 1.5 * MM  # the stopper's hinge off the seated point
SEAT_TILT = math.radians(6.0)  # ... and its axis off the bottle's
PAST = CENTRE + math.radians(5.0)  # the lever this far down is past centre
STRETCHED = 0.5 * MM  # the link stretched at least this much (50 N of the 100 N closed clamp): the stopper is pulled down
BACK_UP = CENTRE - math.radians(10.0)  # the lever back up this far undoes a crossing
CLEAR = 0.03  # m: both grippers at least this far from the bottle and its closure (3 cm: a clear release)
REACH_LO = np.array([-0.095, -0.040, -0.005])  # the bottle with its closure (bottle frame), for CLEAR
REACH_HI = np.array([0.040, 0.040, 0.235])
# Held: a gripper's pads pass through the glass (a point between its pad's back and tip inside the bottle's outline),
# both fingers press with at least GRIP_FORCE, and the pads are apart by at least GRIP_OPENING.
GRIP_FORCE = 0.5
GRIP_OPENING = 0.020
GLASS_SLACK = 0.003
STILL_S, STILL_TURN, STILL_MOVE = 1.0, math.radians(2.0), 0.002  # success: all held 1 s, within 2 degrees and 2 mm
TRACE_S = 1.0  # s between trace lines in a run
STEPS = ("bottle held", "stopper on the mouth", "lever pushed down past centre",
         "let go, stopper still on, bottle upright at its place")
POINTS = 0.2  # per step; only success scores 1.0


def _rot(quat_xyzw) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(quat_xyzw, float))


def _bottle_frame(objects) -> tuple[np.ndarray, np.ndarray]:
    b = objects["bottle"]
    return np.asarray(b.pos, float), _rot(b.quat_xyzw)


def tilt(objects) -> float:
    """The bottle's axis from vertical (rad)."""
    _, r = _bottle_frame(objects)
    return math.acos(max(-1.0, min(1.0, float(r[2, 2]))))


def upright_in_place(objects) -> bool:
    pos, _ = _bottle_frame(objects)
    return bool(tilt(objects) <= UPRIGHT and np.linalg.norm(pos[:2] - BOTTLE_POS[:2]) <= PLACE)


def seated(objects) -> bool:
    """The stopper sits on the mouth: its hinge over the bore at the seated height, its axis along the bottle's."""
    pos, r = _bottle_frame(objects)
    h = r.T @ (np.asarray(objects["stopper_pos"], float) - pos)
    axis = r.T @ _rot(objects["stopper_quat"])[:, 2]
    off = h - HINGE_SEATED
    return bool(math.hypot(off[0], off[1]) <= SEAT_RADIAL and -SEAT_DROP <= off[2] <= SEAT_RISE
                and math.acos(max(-1.0, min(1.0, float(axis[2])))) <= SEAT_TILT)


def lever_closed(objects) -> bool:
    """The lever is down past centre and the stretched link pulls the stopper down."""
    return bool(objects["bail"] >= PAST and objects["seal"] >= STRETCHED)


def _pads(gripper) -> list[np.ndarray]:
    return [gripper.back + f * (gripper.tips - gripper.back) for f in np.linspace(0.0, 1.0, 11)]


def bottle_held(objects) -> bool:
    """A gripper holds the bottle by its glass."""
    pos, r = _bottle_frame(objects)
    for g in ("left_gripper", "right_gripper"):
        gripper = objects.get(g)
        if gripper is None or gripper.contact < GRIP_FORCE or gripper.opening < GRIP_OPENING:
            continue
        for p in _pads(gripper):
            q = r.T @ (p - pos)
            if 0.0 <= q[2] <= LIP_TOP and math.hypot(q[0], q[1]) <= bottle_radius(q[2]) + GLASS_SLACK:
                return True
    return False


def let_go(objects) -> bool:
    """No gripper is within CLEAR of the bottle or its closure."""
    pos, r = _bottle_frame(objects)
    for g in ("left_gripper", "right_gripper"):
        gripper = objects.get(g)
        if gripper is None:
            continue
        for p in _pads(gripper):
            q = r.T @ (p - pos)
            if np.all(q >= REACH_LO - CLEAR) and np.all(q <= REACH_HI + CLEAR):
                return False
    return True


def _poses(objects) -> tuple:
    """The poses whose stillness success needs: the bottle (position, quaternion), the lever's angle, the stopper's
    position."""
    b = objects["bottle"]
    return (np.array(b.pos, float), np.array(b.quat_xyzw, float), float(objects["bail"]),
            np.array(objects["stopper_pos"], float))


def still(objects, anchor) -> bool:
    """The bottle within STILL_MOVE and STILL_TURN, the lever within STILL_TURN and the stopper within STILL_MOVE of
    their poses in ``anchor``. Judged from the poses, not the simulator's instantaneous speeds."""
    pos, quat, bail, stopper = _poses(objects)
    pos0, quat0, bail0, stopper0 = anchor
    turn = 2 * math.acos(min(1.0, abs(float(np.dot(quat0, quat)))))
    return bool(np.linalg.norm(pos - pos0) <= STILL_MOVE and turn <= STILL_TURN and abs(bail - bail0) <= STILL_TURN
                and np.linalg.norm(stopper - stopper0) <= STILL_MOVE)


class _Progress:
    """What has happened so far this episode, updated once per control step: whether a gripper has held the bottle,
    whether the lever went over centre with the stopper on the mouth, whether the bottle was knocked over, and since
    when every full-success condition has held without a break (``met_since``, ``None`` while one fails) with the
    poses when they began to hold (``anchor``). Any lapse (a step undone, a violation, or the bottle, the lever or the
    stopper moving more than STILL_MOVE or turning more than STILL_TURN from the anchor) restarts the count. Starts
    over when the episode time goes back."""

    def __init__(self):
        self.time = -1.0
        self.held = False
        self.crossed = False
        self.knocked = False
        self.bail = None
        self.met_since = None
        self.anchor = None
        self.reported = [False] * len(STEPS)
        self.traced = -1.0

    def update(self, objects) -> None:
        t = objects.get("episode_time")
        if t is not None:
            if t == self.time:  # this control step already counted
                return
            if t < self.time:
                self.__init__()
            self.time = t
        held = bottle_held(objects)
        self.held |= held and upright_in_place(objects)
        self.knocked |= tilt(objects) > KNOCKED
        bail, on = objects["bail"], seated(objects)
        if bail < BACK_UP or not on:
            self.crossed = False
        elif self.bail is not None and self.bail < CENTRE <= bail:  # over centre just now, with the stopper on
            self.crossed = True
        self.bail = bail
        if t is None:  # no clock: nothing can be timed
            return
        met = bool(all(_steps(objects)) and not _violation(objects))
        if met and self.met_since is not None and still(objects, self.anchor):
            return
        if self.met_since is not None:
            print(f"[{TAG}] t={t:.2f}s  success conditions lapsed ({'parts moved' if met else 'a step undone'}): "
                  f"timing restarts", flush=True)
        self.met_since, self.anchor = (t, _poses(objects)) if met else (None, None)
        if met:
            print(f"[{TAG}] t={t:.2f}s  success conditions met: timing {STILL_S:g} s", flush=True)

    def held_s(self) -> float:
        """How long every full-success condition has held without a break (s), 0 while one fails."""
        return 0.0 if self.met_since is None else self.time - self.met_since


_PROGRESS = _Progress()


def _steps(objects) -> list[bool]:
    """Each of ``STEPS`` as things stand, from the episode's progress so far (see ``steps_done``)."""
    p = _PROGRESS
    ok = upright_in_place(objects)
    on = bool(ok and seated(objects))
    closed = bool(on and p.crossed and lever_closed(objects))
    return [bool(ok and p.held), on, closed, bool(closed and let_go(objects))]


def steps_done(objects) -> list[bool]:
    """Each of ``STEPS``, as things stand (the bottle upright at its place for all of them). Step 1 counts once a
    gripper has held the bottle by its glass; step 3 only if the lever went over centre with the stopper on the mouth,
    and only with step 2; step 4 only with step 3."""
    p = _PROGRESS
    p.update(objects)
    done = _steps(objects)
    t = objects.get("episode_time")
    if t is not None and (done != p.reported or t - p.traced >= TRACE_S):  # in a run: the step timeline, a trace
        pos, r = _bottle_frame(objects)
        h = r.T @ (np.asarray(objects["stopper_pos"], float) - pos) - HINGE_SEATED
        line = (f"(bottle tilt {math.degrees(tilt(objects)):.1f} deg, moved {np.linalg.norm(pos[:2] - BOTTLE_POS[:2]) * 1000:.1f} mm, "
                f"{'held' if bottle_held(objects) else 'free'}; stopper {h[0] * 1000:+.1f} {h[1] * 1000:+.1f} "
                f"{h[2] * 1000:+.1f} mm from seated; lever {math.degrees(objects['bail']):.1f} deg "
                f"(centre {math.degrees(CENTRE):.1f}), link stretched {objects['seal'] * 1000:.2f} mm)")
        if done != p.reported:
            p.reported = done
            got = ", ".join(name for name, ok_ in zip(STEPS, done, strict=True) if ok_) or "none"
            print(f"[{TAG}] t={t:.2f}s  steps done: {got}  {line}", flush=True)
        else:
            print(f"[{TAG}] t={t:.2f}s  {line}", flush=True)
        p.traced = t
    return done


def _violation(objects) -> bool:
    """Scores 0: the bottle knocked over (now or earlier this episode) or off the table, or a state that is not
    finite. Reads the episode's progress without updating it."""
    pos, _ = _bottle_frame(objects)
    values = [objects["bail"], objects["link"], objects["seal"], objects["hinge"], *pos, *objects["stopper_pos"]]
    if not np.all(np.isfinite(values)):
        return True
    return bool(_PROGRESS.knocked or tilt(objects) > KNOCKED or pos[2] < TOP - 0.03)


def violation(objects) -> bool:
    """``_violation`` after counting this control step."""
    _PROGRESS.update(objects)
    return _violation(objects)


def success(objects) -> bool:
    """Every full-success condition has held without a break for STILL_S, counted from the first control step they all
    held: all four steps done (the last one: both grippers clear), no violation, and the bottle, the lever and the
    stopper within STILL_MOVE and STILL_TURN of where they were when that second began. The arms' return is not
    scored."""
    p = _PROGRESS
    p.update(objects)
    return bool(p.met_since is not None and p.held_s() >= STILL_S - 1e-6)


def score(objects) -> float:
    """1.0 on success (every full-success condition held for STILL_S); otherwise 0.2 per step done (``STEPS``, at most
    0.8, also while the second is being timed); 0 for any violation."""
    done = steps_done(objects)
    if violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    return round(POINTS * sum(done), 6)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Left arm: from the bottle's left, pointing across it a little downward, its fingers closing on the body's front and
# back; it holds the bottle until the right arm has let go of the lever, then lets go and goes home.
# Right arm: straight down onto the hanging stopper's knob, its fingers closing across the knob (they square it up);
# carries the stopper up beside the neck, over the mouth and down onto it, lets go and lifts away. Then it takes the
# paddle between its fingers, pointing at the paddle's face, and pushes the lever down along its arc onto its stop,
# turning with it; lets go, backs off the way it points and goes home.
SQUEEZE = 0.008  # grips close the fingers this far inside the part's width (about 20 N a finger)
BOTTLE_HOLD = -(2 * BODY_R - SQUEEZE) / 2
BOTTLE_OPEN = -(2 * BODY_R + 0.022) / 2
HOLD_Z = 0.070  # the left TCP's height on the bottle (bottle frame)
LEFT_TILT = math.radians(15.0)  # the left gripper points this far below horizontal
KNOB_HOLD = -(KNOB_WIDTH - SQUEEZE) / 2
KNOB_OPEN = -(KNOB_WIDTH + 0.012) / 2
KNOB_GRIP = 0.018  # the right TCP this far up the knob from the stopper's hinge
OVER_KNOB = 0.045  # the right gripper comes down on the knob from this far above
PADDLE_HOLD = -(PADDLE_ALONG - SQUEEZE) / 2
PADDLE_OPEN = -(PADDLE_ALONG + 0.024) / 2
PADDLE_OUT = 0.003  # the right TCP this far out from the paddle's middle (toward its pushed face)
OVER_PADDLE = 0.045
SEAT_PRESS = 0.0015  # the stopper is set this far below its seated height
PUSH_OVER = math.radians(2.0)  # the lever is pushed this far past its stop
PUSH_STEPS = 6
KNOB_TILT = math.radians(45.0)  # the right gripper leans this far from straight down on the knob, toward the front
PADDLE_TILT = math.radians(45.0)  # ... and this far from the paddle's push, toward the front
RIGHT_FLIP = 1  # which way round the right gripper is about its pointing axis on the knob (0 or 1)
PADDLE_FLIP = 0  # ... and on the paddle
BACK_OFF = 0.05
# The stopper's hinge on its way from where it hangs to the mouth (bottle frame, m), in order.
CARRY = [np.array(p) * MM for p in ((-46.0, 0.0, 165.0), (-42.0, 0.0, 190.0), (-36.0, 0.0, 203.0),
                                    (-24.0, 0.0, 211.0), (-10.0, 0.0, 213.0), (0.0, 0.0, 210.0))]
CARRY_AIM = 0.014  # m: the knob is aimed this far above the carry path (the wrist gives under the lever's pull)


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


def _frame(x_b, z_b) -> np.ndarray:
    """A gripper rotation (world) from its closing axis ``x_b`` and pointing axis ``z_b`` in the bottle frame."""
    x, z = BOTTLE_ROT @ np.asarray(x_b, float), BOTTLE_ROT @ np.asarray(z_b, float)
    return np.column_stack([x, np.cross(z, x), z])


def _left_rot() -> np.ndarray:
    """Pointing across the bottle from its +x side (the robot's left), a little downward; closing along its y."""
    point = np.array([-math.cos(LEFT_TILT), 0.0, -math.sin(LEFT_TILT)])
    return _frame(_Y, point)


def _knob_rot() -> np.ndarray:
    """Down, leaning KNOB_TILT toward the robot's front (pointing away from the robot), the fingers closing along the
    bottle's x (across the knob, square to the stopper's hinge)."""
    point = np.array([0.0, -math.sin(KNOB_TILT), -math.cos(KNOB_TILT)])
    return _frame(_X if RIGHT_FLIP == 0 else -_X, point)


def _paddle_rot(bail: float) -> np.ndarray:
    """Pointing at the paddle's pushed face, the fingers closing along the lever's arm, the lever at ``bail``."""
    k = linkage(bail, 0.0, 0.0, 0.0)
    point = math.cos(PADDLE_TILT) * k["push"] - math.sin(PADDLE_TILT) * _Y
    return _frame(k["arm"] if PADDLE_FLIP == 0 else -k["arm"], point)


def _paddle_tcp(bail: float, back: float = 0.0) -> np.ndarray:
    k = linkage(bail, 0.0, 0.0, 0.0)
    point = math.cos(PADDLE_TILT) * k["push"] - math.sin(PADDLE_TILT) * _Y
    return to_world(k["paddle"] - PADDLE_OUT * k["push"] - back * point)


def _knob_tcp(hinge_pt, back: float = 0.0) -> np.ndarray:
    """The right TCP on the upright stopper's knob, its hinge at ``hinge_pt`` (bottle frame)."""
    point = np.array([0.0, -math.sin(KNOB_TILT), -math.cos(KNOB_TILT)])
    return to_world(np.asarray(hinge_pt, float) + KNOB_GRIP * _Z - back * point)


def left_plan(hold_s: float) -> list[Waypoint]:
    """Grip the bottle's body, hold it for ``hold_s``, let go, go home."""
    rot = _left_rot()
    point = rot[:, 2]
    grip = to_world(np.array([0.0, 0.0, HOLD_Z]))
    return [
        Waypoint(2.5, grip - 0.07 * point, BOTTLE_OPEN, "beside the bottle", rot=rot),
        Waypoint(1.5, grip, BOTTLE_OPEN, "around the bottle", rot=rot),
        Waypoint(0.8, grip, BOTTLE_HOLD, "grip the bottle", rot=rot),
        Waypoint(hold_s, grip, BOTTLE_HOLD, "hold the bottle", rot=rot),
        Waypoint(0.6, grip, BOTTLE_OPEN, "let go", rot=rot),
        Waypoint(1.2, grip - 0.07 * point, BOTTLE_OPEN, "back off", rot=rot),
        Waypoint(2.5, _START_TCP["left"], FINGER_OPEN, "home", rot=_START_ROT["left"]),
    ]


def right_plan(wait_s: float) -> tuple[list[Waypoint], float]:
    """Wait ``wait_s``, put the stopper on the mouth, push the lever down past centre onto its stop, go home. Also
    returns when the right gripper has let go of the paddle and backed off."""
    hang = linkage(*START_JOINTS.values())["hinge"]
    down = _knob_rot()
    seat = HINGE_SEATED - SEAT_PRESS * _Z
    wps = [
        Waypoint(wait_s, _START_TCP["right"], FINGER_OPEN, "wait", rot=_START_ROT["right"]),
        Waypoint(2.5, _knob_tcp(hang, OVER_KNOB), KNOB_OPEN, "over the knob", rot=down),
        Waypoint(1.2, _knob_tcp(hang), KNOB_OPEN, "around the knob", rot=down),
        Waypoint(0.8, _knob_tcp(hang), KNOB_HOLD, "grip the knob", rot=down),
    ]
    wps += [Waypoint(1.0, _knob_tcp(p + CARRY_AIM * _Z), KNOB_HOLD, "carry the stopper", rot=down) for p in CARRY]
    wps += [
        Waypoint(1.0, _knob_tcp(seat), KNOB_HOLD, "set it on the mouth", rot=down),
        Waypoint(0.4, _knob_tcp(seat), KNOB_HOLD, "hold", rot=down),
        Waypoint(0.6, _knob_tcp(seat), KNOB_OPEN, "let go of the stopper", rot=down),
        Waypoint(1.0, _knob_tcp(seat, OVER_KNOB), KNOB_OPEN, "lift away", rot=down),
    ]
    start = 0.0  # the lever's angle once the stopper is on (it rests just past the link going taut)
    end = CLOSED + PUSH_OVER
    r0 = _paddle_rot(start)
    wps += [
        Waypoint(2.0, _paddle_tcp(start, OVER_PADDLE), PADDLE_OPEN, "over the paddle", rot=r0),
        Waypoint(1.2, _paddle_tcp(start), PADDLE_OPEN, "around the paddle", rot=r0),
        Waypoint(0.8, _paddle_tcp(start), PADDLE_HOLD, "grip the paddle", rot=r0),
    ]
    angles = [start + (end - start) * k / PUSH_STEPS for k in range(1, PUSH_STEPS + 1)]
    wps += [Waypoint(0.6, _paddle_tcp(a), PADDLE_HOLD, "push the lever down", rot=_paddle_rot(a)) for a in angles]
    wps += [
        Waypoint(0.4, _paddle_tcp(end), PADDLE_HOLD, "hold", rot=_paddle_rot(end)),
        Waypoint(0.6, _paddle_tcp(end), PADDLE_OPEN, "let go of the paddle", rot=_paddle_rot(end)),
        Waypoint(1.0, _paddle_tcp(end, BACK_OFF), PADDLE_OPEN, "back off", rot=_paddle_rot(end)),
    ]
    done = sum(w.duration for w in wps)
    wps.append(Waypoint(2.5, _START_TCP["right"], FINGER_OPEN, "home", rot=_START_ROT["right"]))
    return wps, done


def demo():
    """The left arm grips the bottle; the right puts the stopper on and pushes the lever down; once the right gripper
    has let go of the paddle and backed off, the left lets go. Both go home."""
    held_at = 2.5 + 1.5 + 0.8
    right, backed_off = right_plan(wait_s=held_at - 2.5 + 0.3)
    return {"left": left_plan(hold_s=backed_off + 0.3 - held_at), "right": right}


# Where each arm's TCP starts (its ready pose), to come back to at the end.
_START_TCP = {side: yam_demo.ArmPlan([], BASES[side]).start_pos for side in ("left", "right")}
_START_ROT = {side: yam_demo.ArmPlan([], BASES[side]).start_rot for side in ("left", "right")}
