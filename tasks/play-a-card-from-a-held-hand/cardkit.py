"""Parts for the play-a-card task: playing cards, flat tape marks, and a start that puts a hand of cards in the left
gripper.

What a task file can list in ``OBJECTS`` (both are Box subclasses so the loader accepts them):

``Card``   One poker-size playing card, a free rigid plate 63.5 x 88.9 mm and 0.29 mm thick. Its ``pos`` is its centre;
           its own x runs across its width, its y along its length, and its z out of its BACK: a card lying face up
           has its z down.
``Mark``   A flat mark on the table (a strip of tape). It is drawn only: nothing collides with it.

Every shape is built from numbers in NumPy when the task file is read, so the same numbers serve the scene, the
scoring and the offline checks. Isaac Sim is reached only when the scene is built.

The card, its friction and its solver settings are those of the accepted card-dealing task (TK-ZPAQZ4) in this
repository; the structure follows the kits of tasks already here (Box subclasses so the loader accepts the parts,
custom spawners, hooks installed through ``sys.meta_path``). New here: a card's face carries its rank as pips in the
middle and again as a small index of pips in two corners, as printed cards do, so the rank shows on the strip of a
card left in view in a fan; and ``on_start`` hooks, which run at the end of every reset to set the trial's starting
state (here: the left arm holding a fanned hand of cards) before the policy acts.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

# -- a card (m, kg) ------------------------------------------------------------------------------------------------------
CARD_W, CARD_L = 0.0635, 0.0889   # poker size: "Dimensions: 63.5mm x 88.9mm, 2.5" x 3.5"" (a card printer's page)
CARD_T = 0.00029                  # "The thickness of all 52 playing cards was 15.15mm" (a measured deck): 0.291 mm a card
CARD_MASS = 0.00155               # "Ten cards weighed 15.5 grams" (the same measured deck)
# Friction (static, dynamic), all estimates, as in the card-dealing task. The engine gives a pair of bodies the average
# of their two coefficients, and the robot's fingertips have its default (OTHER):
#   card on card                 CARD                 = 0.26, 0.20   coated card stock
#   fingertip on a card          (OTHER + CARD) / 2   = 0.38, 0.35   a pad on a coated card
#   card on the table            (OTHER + CARD) / 2   = 0.38, 0.35   (the table has the default too)
OTHER = (0.5, 0.5)
CARD_FRICTION = (0.26, 0.20)
PAD_ON_CARD = tuple((o + c) / 2 for o, c in zip(OTHER, CARD_FRICTION))
# Solver settings for a 1.55 g plate 0.29 mm thick (as in the card-dealing task).
CARD_ITERS = int(os.environ.get("CARD_ITERS", "255"))
CARD_OFFSET = float(os.environ.get("CARD_OFFSET", "0.0008"))  # contact offset (m)
CARD_DEPEN = float(os.environ.get("CARD_DEPEN", "0.5"))       # m/s, the most a pressed-in card is pushed back out at

BACK_RED = (0.62, 0.10, 0.12)
FACE_WHITE = (0.97, 0.97, 0.95)
PIP_BLACK = (0.08, 0.08, 0.09)
TAPE = (0.15, 0.35, 0.75)
MARK_T = 0.00008                  # how thick a mark is drawn

PALM_BACK = 0.065
FINGER_THICK = 0.012


# -- geometry (pure) -------------------------------------------------------------------------------------------------
def _rotation(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _quat(rot) -> tuple:
    m = np.asarray(rot, float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    else:
        i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(max(1e-12, 1.0 + m[i, i] - m[j, j] - m[k, k])) * 2
        q = [0.0, 0.0, 0.0, (m[k, j] - m[j, k]) / s]
        q[i], q[j], q[k] = 0.25 * s, (m[j, i] + m[i, j]) / s, (m[k, i] + m[i, k]) / s
    return tuple(float(v) for v in q)


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def corners(pos, quat) -> np.ndarray:
    """The four corners of a card (world), from its centre and quaternion."""
    rot = _rotation(quat)
    loc = np.array([[sx * CARD_W / 2, sy * CARD_L / 2, 0.0] for sx in (-1, 1) for sy in (-1, 1)])
    return loc @ rot.T + np.asarray(pos, float)


def back_up(quat) -> float:
    """How far a card's back faces up: 1 lying face down, 0 on edge, -1 lying face up."""
    return float(_rotation(quat)[2, 2])


def _obb(points, centre, rot, size) -> np.ndarray:
    """Signed distance from each point to a box of ``size`` at ``centre`` with its edges along the columns of ``rot``."""
    q = np.abs((np.asarray(points, float).reshape(-1, 3) - centre) @ rot) - np.asarray(size, float) / 2
    return np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)


def card_distance(points, pos, quat) -> float:
    """The least distance from any of ``points`` (world) to a card."""
    return float(_obb(points, np.asarray(pos, float), _rotation(quat), (CARD_W, CARD_L, CARD_T)).min())


# -- the parts -----------------------------------------------------------------------------------------------------------

class _Part(Box):
    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")


@dataclass(frozen=True)
class Card(_Part):
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    pips: int = 0                  # 1 to 6: an ace to a six, drawn as that many pips; 0: a plain panel


@dataclass(frozen=True)
class Mark(_Part):
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    strips: tuple = ()             # ((centre x, centre y), (size x, size y)) in the mark's own frame


@dataclass(frozen=True)
class KitState(ObjectState):
    """A kit object as the simulator has it; ``pos`` is its own origin (see each class)."""

    joints: dict = field(default_factory=dict)


# -- authoring helpers -----------------------------------------------------------------------------------------------

class _Author:
    def __init__(self, prim_path, translation, orientation):
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, UsdGeom

        self.sim_utils, self.Gf, self.UsdGeom = sim_utils, Gf, UsdGeom
        self.stage = get_current_stage()
        self.path = prim_path
        double = UsdGeom.XformOp.PrecisionDouble
        root = UsdGeom.Xform.Define(self.stage, prim_path)
        root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
        x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
        root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))
        self.root = root.GetPrim()
        self.looks, self.mats = {}, {}

    def look(self, color, rough=0.6, metal=0.0) -> str:
        from yam_arena.station import linear_rgb

        key = (tuple(color), rough, metal)
        if key not in self.looks:
            path = f"{self.path}/Looks/v{len(self.looks)}"
            vis = self.sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough, metallic=metal)
            vis.func(path, vis)
            self.looks[key] = path
        return self.looks[key]

    def mat(self, key, sf, df, bounce=0.0) -> str:
        if key not in self.mats:
            path = f"{self.path}/Looks/phys_{key}"
            try:
                pm = self.sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df, restitution=bounce,
                                                         friction_combine_mode="average", restitution_combine_mode="average")
            except TypeError:
                pm = self.sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df, restitution=bounce)
            pm.func(path, pm)
            self.mats[key] = path
        return self.mats[key]

    def body(self, path, mass, lin_damp=0.05, ang_damp=0.1, iterations=None, depen=0.3, kinematic=False):
        from pxr import PhysxSchema, UsdPhysics

        prim = self.UsdGeom.Xform.Define(self.stage, path).GetPrim()
        rb_api = UsdPhysics.RigidBodyAPI.Apply(prim)
        if kinematic:
            rb_api.CreateKinematicEnabledAttr(True)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(float(depen))
        rb.CreateLinearDampingAttr(lin_damp)
        rb.CreateAngularDampingAttr(ang_damp)
        rb.CreateSleepThresholdAttr(0.0)
        if iterations:
            rb.CreateSolverPositionIterationCountAttr(int(iterations))
            rb.CreateSolverVelocityIterationCountAttr(4)
        return prim

    def bar(self, path, centre, size, look, mat=None, offset=0.0008):
        """A box of ``size`` at ``centre``: a box collider (thin plates cannot be cooked as convex hulls for the GPU
        pipeline, and then collide with nothing). Without ``mat`` it is drawn only."""
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from pxr import PhysxSchema, UsdPhysics

        cube = self.UsdGeom.Cube.Define(self.stage, path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(self.Gf.Vec3d(*(float(v) for v in centre)))
        cube.AddScaleOp().Set(self.Gf.Vec3f(*(float(v) for v in size)))
        prim = cube.GetPrim()
        if look:
            bind_visual_material(str(prim.GetPath()), look, stage=self.stage)
        if mat:
            UsdPhysics.CollisionAPI.Apply(prim)
            cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            cp.CreateContactOffsetAttr(float(offset))
            cp.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(prim.GetPath()), mat, stage=self.stage)
        return prim


# -- what a card's face shows ------------------------------------------------------------------------------------------
PIP = 0.011                       # a pip in the middle of the face
INDEX_PIP = 0.0045                # a pip of the corner index
INDEX_IN = (0.0065, 0.0075)       # the corner index starts this far in from the card's side and end


def face_pips(pips: int) -> list:
    """(x, y) of each pip in the middle of a card's face, in the card's own frame (x across, y along), for an ace to a
    six: up to three in each of two columns."""
    if pips == 1:
        return [(0.0, 0.0)]
    if pips <= 3:
        return [(0.0, (k - (pips - 1) / 2) * 0.026) for k in range(pips)]
    rows = {4: (-0.026, 0.026), 5: (-0.026, 0.026), 6: (-0.026, 0.0, 0.026)}[pips]
    out = [(sx * 0.013, y) for sx in (-1, 1) for y in rows]
    return out + ([(0.0, 0.0)] if pips == 5 else [])


def index_pips(pips: int) -> list:
    """(x, y) of each pip of the two corner indices, in the card's own frame: a short column of ``pips`` small pips
    running in from the end of the card, near the top right and the bottom left corner of its face as it is held
    (face toward the viewer, the card's own +y up): in the fan of this task each card's right-hand strip is the part
    left in view. A card's own x points to the viewer's left when its face is toward the viewer, so the top right
    corner is at -x, +y."""
    out = []
    step = INDEX_PIP * 1.35
    for sx, sy in ((-1, 1), (1, -1)):
        x0 = sx * (CARD_W / 2 - INDEX_IN[0] - INDEX_PIP / 2)
        y0 = sy * (CARD_L / 2 - INDEX_IN[1] - INDEX_PIP / 2)
        out += [(x0, y0 - sy * k * step) for k in range(pips)]
    return out


# -- spawning ----------------------------------------------------------------------------------------------------------

_SPECS: dict = {}
_PATHS: dict = {}                      # where each kit object's prim was spawned


def _spawn_card(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec = _SPECS[cfg.kit_name]
    _PATHS[spec.name] = prim_path
    a = _Author(prim_path, translation, orientation)
    a.body(prim_path, CARD_MASS, lin_damp=0.02, ang_damp=0.05, iterations=CARD_ITERS, depen=CARD_DEPEN)
    a.bar(f"{prim_path}/plate", (0.0, 0.0, 0.0), (CARD_W, CARD_L, CARD_T), a.look(FACE_WHITE, rough=0.45),
          a.mat("card", *CARD_FRICTION), offset=CARD_OFFSET)
    # Drawn only: the red back (a border left white) and the pips on the face.
    eps = 0.00002
    a.bar(f"{prim_path}/back", (0.0, 0.0, CARD_T / 2 + eps / 2), (CARD_W - 0.007, CARD_L - 0.007, eps), a.look(BACK_RED, rough=0.45))
    pips = int(spec.pips)
    black = a.look(PIP_BLACK, rough=0.5)
    z = -CARD_T / 2 - eps / 2
    if 1 <= pips <= 6:
        for k, (x, y) in enumerate(face_pips(pips)):
            a.bar(f"{prim_path}/pip{k}", (x, y, z), (PIP, PIP, eps), black)
        for k, (x, y) in enumerate(index_pips(pips)):
            a.bar(f"{prim_path}/ix{k}", (x, y, z), (INDEX_PIP, INDEX_PIP, eps), black)
    else:
        a.bar(f"{prim_path}/panel", (0.0, 0.0, z), (0.034, 0.056, eps), a.look((0.25, 0.30, 0.55), rough=0.5))
    return a.root


def _spawn_mark(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec = _SPECS[cfg.kit_name]
    _PATHS[spec.name] = prim_path
    a = _Author(prim_path, translation, orientation)
    a.body(prim_path, 0.01, kinematic=True)
    look = a.look(tuple(spec.color), rough=0.7)
    for k, (centre, size) in enumerate(spec.strips):
        a.bar(f"{prim_path}/s{k}", (centre[0], centre[1], 0.0), (size[0], size[1], MARK_T), look)
    return a.root


def _rigid_object(spec, spawn):
    from isaaclab.sim.spawners.spawner_cfg import RigidObjectSpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class KitSpawnerCfg(RigidObjectSpawnerCfg):
        func: object = clone(spawn)
        kit_name: str = ""

    _SPECS[spec.name] = spec
    _PATHS.pop(spec.name, None)
    quat = tuple(float(v) for v in getattr(spec, "quat", (0.0, 0.0, 0.0, 1.0)))
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=KitSpawnerCfg(kit_name=spec.name),
                  initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=quat))


_SPAWNERS = {Card: _spawn_card, Mark: _spawn_mark}


# -- run time ----------------------------------------------------------------------------------------------------------

def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    if hasattr(x, "numpy") and not hasattr(x, "detach"):
        return np.asarray(x.numpy(), dtype=np.float64)
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _origin(environment) -> np.ndarray:
    return _np(environment.unwrapped.scene.env_origins[0])


def _kit_state(environment, spec) -> KitState:
    asset = environment.unwrapped.scene[spec.name]
    pose = _np(asset.data.root_link_pose_w)[0]
    pos, quat = pose[:3] - _origin(environment), pose[3:7]
    vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    pts = corners(pos, quat) if isinstance(spec, Card) else np.array([pos, pos])
    return KitState(pos=pos, quat_xyzw=quat, lin_vel=vel[:3], ang_vel=vel[3:6], low=pts.min(0), high=pts.max(0))


def finger_points(environment) -> list:
    """The grasp point of each gripper, left then right (world, relative to the env origin)."""
    from yam_arena.constants import TCP_OFFSET

    base = environment.unwrapped
    origin = _origin(environment)
    pts = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = _np(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
        pts.append(pose[:3] + _rotation(pose[3:7]) @ np.array(TCP_OFFSET, float) - origin)
    return pts


def finger_frames(environment) -> list:
    """Each gripper's own rotation as the robot has it, left then right."""
    base = environment.unwrapped
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = _np(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
        out.append(_rotation(pose[3:7]))
    return out


def finger_gaps(environment) -> list:
    """How far apart each gripper's pads are (m), left then right."""
    base = environment.unwrapped
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        q, jn = _np(robot.data.joint_pos)[0], list(robot.joint_names)
        out.append(sum(abs(float(q[jn.index(j)])) for j in ("joint7", "joint8")))
    return out


def gripper_points(environment) -> list:
    """Points on each gripper that could touch a card, left gripper then right (world, relative to the env origin):
    the pad tips, the tool point and the palm, at the fingers' current opening."""
    from yam_arena.constants import PAD_TIP_PAST_TCP, TCP_OFFSET

    base = environment.unwrapped
    origin = _origin(environment)
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        names = list(robot.body_names)
        pose = _np(robot.data.body_link_pose_w)[0][names.index("gripper")]
        rot = _rotation(pose[3:7])
        tcp = pose[:3] + rot @ np.array(TCP_OFFSET, float) - origin
        q, jn = _np(robot.data.joint_pos)[0], list(robot.joint_names)
        half = sum(abs(float(q[jn.index(j)])) for j in ("joint7", "joint8")) / 2 + FINGER_THICK
        pts = [tcp, tcp - rot[:, 2] * PALM_BACK, tcp + rot[:, 2] * PAD_TIP_PAST_TCP]
        for s in (-1.0, 1.0):
            for along in (PAD_TIP_PAST_TCP, 0.0, -0.03, -PALM_BACK):
                pts.append(tcp + rot[:, 0] * s * half + rot[:, 2] * along)
        out.append(np.array(pts))
    return out


# -- setting the starting state ----------------------------------------------------------------------------------------

def place_arm(environment, arm: str, q_arm, finger: float) -> None:
    """Put one arm's six joints at ``q_arm`` (rad) and both its fingers at ``finger`` (m, the finger joints' own value:
    0 closed, -0.04695 open), at rest, with its joint targets there too. The same writes the embodiment's own reset
    makes (``yam_arena.actions.reset_arms_to_default``)."""
    import torch
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    robot = environment.unwrapped.scene[arm]
    q = robot.data.joint_pos.torch.clone()
    arm_ids = robot.find_joints(list(ARM_JOINTS), preserve_order=True)[0]
    finger_ids = robot.find_joints(list(FINGER_JOINTS), preserve_order=True)[0]
    q[:, arm_ids] = torch.tensor([list(q_arm)], dtype=q.dtype, device=q.device)
    q[:, finger_ids] = float(finger)
    v = torch.zeros_like(q)
    robot.write_joint_position_to_sim_index(position=q)
    robot.write_joint_velocity_to_sim_index(velocity=v)
    robot.actuators.target_command.set_position_index(value=q)
    robot.actuators.target_command.set_velocity_index(value=v)


def place_object(environment, name: str, pos, quat) -> None:
    """Put a free object at ``pos`` (world, relative to the env origin) and ``quat`` (xyzw), at rest. The same writes
    the embodiment's own seeded reset makes."""
    import torch

    obj = environment.unwrapped.scene[name]
    pose = obj.data.default_root_pose.torch.clone()
    origin = environment.unwrapped.scene.env_origins
    pose[:, :3] = torch.tensor([list(pos)], dtype=pose.dtype, device=pose.device) + origin
    pose[:, 3:7] = torch.tensor([list(quat)], dtype=pose.dtype, device=pose.device)
    obj.write_root_pose_to_sim_index(root_pose=pose)
    obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))


# -- hooks ---------------------------------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []
_START: list = []
_ENV: dict = {"env": None}


def on_step(fn):
    _STEP.append(fn)
    return fn


def on_reset(fn):
    _RESET.append(fn)
    return fn


def on_start(fn):
    """``fn(embodiment)`` runs at the end of every reset, after the embodiment has put everything back and settled,
    and before the first observation is returned: it sets the trial's starting state. It may step the simulation
    (``embodiment._env.step``) to let that state settle; those steps are not the policy's."""
    _START.append(fn)
    return fn


def env():
    return _ENV["env"]


def _patch_scene(scene) -> None:
    if getattr(scene, "_cardkit_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        for kind, spawn in _SPAWNERS.items():
            if isinstance(spec, kind):
                return _rigid_object(spec, spawn)
        return stock_object(spec)

    def object_states(environment, names):
        mine = {n for n in names if isinstance(_SPECS.get(n), tuple(_SPAWNERS))}
        plain = [n for n in names if n not in mine]
        out = stock_states(environment, plain) if plain else {}
        for n in mine:
            out[n] = _kit_state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._cardkit_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_cardkit_patched", False):
        return
    stock_step, stock_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        return stock_step(self, *a, **k)

    def reset(self, *a, **k):
        obs = stock_reset(self, *a, **k)
        _ENV["env"] = self._env
        if _START:
            for fn in _START:
                fn(self)
            for _ in range(4):                       # the RTX renderer needs a few frames to converge
                self._env.unwrapped.sim.render()
            self._t = 0.0
            obs = self._observation()
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._cardkit_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    targets = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}

    def find_spec(self, fullname, path, target=None):
        if fullname not in self.targets:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        loader, patch = spec.loader, self.targets[fullname]
        stock_exec = loader.exec_module

        def exec_module(module):
            stock_exec(module)
            patch(module)

        loader.exec_module = exec_module
        return spec


def install() -> None:
    """Hook yam_arena (now for modules already imported, otherwise when they are). Idempotent."""
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
