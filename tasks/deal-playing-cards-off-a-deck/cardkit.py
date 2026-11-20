"""Parts for the card-dealing task: playing cards and flat tape marks.

What a task file can list in ``OBJECTS`` (both are Box subclasses so the loader accepts them):

``Card``   One poker-size playing card, a free rigid plate 63.5 x 88.9 mm and 0.29 mm thick. Its ``pos`` is its centre;
           its own x runs across its width, its y along its length, and its z out of its BACK: a card lying face down
           has its z up.
``Board``  The dealing board: a smooth laminate-faced board fixed to the table, which the cards lie and slide on. Its
           ``pos`` is its centre and ``size`` its length, width and thickness.
``Mark``   A flat mark on the board (a printed or painted line). It is drawn only: nothing collides with it, so a
           card slides over it as it would over a printed line.

Every shape is built from numbers in NumPy when the task file is read, so the same numbers serve the scene, the
scoring and the offline checks. Isaac Sim is reached only when the scene is built.

The structure follows the kits used by tasks already in this repository (Box subclasses so the loader accepts the
parts, custom spawners, hooks installed through ``sys.meta_path``).
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
# Friction (static, dynamic), all estimates. The engine gives a pair of bodies the average of their two coefficients,
# and the robot's fingertips have its default (OTHER); the card's and the board's own coefficients are chosen to give
# these three pairs:
#   card on card                 CARD                 = 0.26, 0.20   coated card stock
#   fingertip on a card's back   (OTHER + CARD) / 2   = 0.38, 0.35   a rubber pad on a coated card
#   card on the dealing board    (CARD + BOARD) / 2   = 0.18, 0.15   a coated card on smooth laminate
OTHER = (0.5, 0.5)
CARD_FRICTION = (0.26, 0.20)
BOARD_FRICTION = (0.10, 0.10)
PAD_ON_CARD = tuple((o + c) / 2 for o, c in zip(OTHER, CARD_FRICTION))
CARD_ON_BOARD = tuple((c + m) / 2 for c, m in zip(CARD_FRICTION, BOARD_FRICTION))
# -- the dealing board: a smooth laminate-faced board fixed to the table, on which the cards lie and slide ----------------
BOARD_T = 0.003
BOARD_GREEN = (0.16, 0.36, 0.26)
# Solver settings for a 1.55 g plate 0.29 mm thick (see README, "What the simulation simplifies").
CARD_INERTIA = float(os.environ.get("CARD_INERTIA", "0"))   # kg; 0: the card's body has its true mass
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


def yaw_quat(yaw: float) -> tuple:
    """A card lying face down, its length turned ``yaw`` radians anticlockwise (seen from above) from pointing along
    the world's y."""
    return _quat(_rz(yaw))


def face_up_quat(yaw: float = 0.0) -> tuple:
    """A card lying face up (turned over about its own length)."""
    return _quat(_rz(yaw) @ np.diag([-1.0, 1.0, -1.0]))


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


def outline_excess(pos, quat, centre_xy, yaw: float) -> float:
    """How far (m) the farthest corner of a card lies outside a card-sized outline at ``centre_xy`` whose length is
    turned ``yaw`` from the world's y; 0 if the card is within the outline."""
    c = corners(pos, quat)[:, :2] - np.asarray(centre_xy, float)
    loc = c @ _rz(yaw)[:2, :2]
    return float(max(0.0, (np.abs(loc[:, 0]) - CARD_W / 2).max(), (np.abs(loc[:, 1]) - CARD_L / 2).max()))


# -- the parts -----------------------------------------------------------------------------------------------------------

class _Part(Box):
    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")


@dataclass(frozen=True)
class Card(_Part):
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    pips: int = 0                  # 1 to 6: an ace to a six, drawn as that many pips; 0: any other card


@dataclass(frozen=True)
class Board(_Part):
    quat: tuple = (0.0, 0.0, 0.0, 1.0)


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


# -- spawning ----------------------------------------------------------------------------------------------------------

_SPECS: dict = {}
_PATHS: dict = {}                      # where each kit object's prim was spawned


def _spawn_card(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec = _SPECS[cfg.kit_name]
    _PATHS[spec.name] = prim_path
    a = _Author(prim_path, translation, orientation)
    a.body(prim_path, CARD_INERTIA or CARD_MASS, lin_damp=0.02, ang_damp=0.05, iterations=CARD_ITERS, depen=CARD_DEPEN)
    a.bar(f"{prim_path}/plate", (0.0, 0.0, 0.0), (CARD_W, CARD_L, CARD_T), a.look(FACE_WHITE, rough=0.45),
          a.mat("card", *CARD_FRICTION), offset=CARD_OFFSET)
    # Drawn only: the red back (a border left white) and the pips on the face.
    eps = 0.00002
    a.bar(f"{prim_path}/back", (0.0, 0.0, CARD_T / 2 + eps / 2), (CARD_W - 0.007, CARD_L - 0.007, eps), a.look(BACK_RED, rough=0.45))
    pips = int(spec.pips)
    black = a.look(PIP_BLACK, rough=0.5)
    if 1 <= pips <= 6:                # ace to six: the pips in two columns of up to three
        spots = [(0.0, 0.0)] if pips == 1 else [(sx * 0.013 * (pips > 3 or 0), y) for sx, y in _pip_grid(pips)]
        for k, (x, y) in enumerate(spots):
            a.bar(f"{prim_path}/pip{k}", (x, y, -CARD_T / 2 - eps / 2), (0.011, 0.011, eps), black)
    else:                             # any other card: a plain panel
        a.bar(f"{prim_path}/panel", (0.0, 0.0, -CARD_T / 2 - eps / 2), (0.034, 0.056, eps), a.look((0.25, 0.30, 0.55), rough=0.5))
    return a.root


def _pip_grid(pips: int) -> list:
    """(column sign, y) of each pip of a two to six."""
    if pips <= 3:
        return [(0, (k - (pips - 1) / 2) * 0.026) for k in range(pips)]
    rows = {4: (-0.026, 0.026), 5: (-0.026, 0.026), 6: (-0.026, 0.0, 0.026)}[pips]
    out = [(sx, y) for sx in (-1, 1) for y in rows]
    return out + ([(0, 0.0)] if pips == 5 else [])


def _spawn_board(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec = _SPECS[cfg.kit_name]
    _PATHS[spec.name] = prim_path
    a = _Author(prim_path, translation, orientation)
    a.body(prim_path, 1.0, kinematic=True)
    a.bar(f"{prim_path}/slab", (0.0, 0.0, 0.0), tuple(spec.size), a.look(tuple(spec.color), rough=0.35),
          a.mat("board", *BOARD_FRICTION), offset=0.001)
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


_SPAWNERS = {Card: _spawn_card, Mark: _spawn_mark, Board: _spawn_board}


# -- run time ----------------------------------------------------------------------------------------------------------

def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    if hasattr(x, "numpy") and not hasattr(x, "detach"):
        return np.asarray(x.numpy(), dtype=np.float64)
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _origin(environment) -> np.ndarray:
    return _np(environment.unwrapped.scene.env_origins[0])


def _lift(environment) -> None:
    """With ``CARD_INERTIA`` set, carry the extra weight of every card's heavier body, so that it still weighs what
    a card weighs."""
    if not CARD_INERTIA:
        return
    import torch

    f = (CARD_INERTIA - CARD_MASS) * 9.81
    for spec in _SPECS.values():
        if isinstance(spec, Card):
            asset = environment.unwrapped.scene[spec.name]
            force = torch.tensor([[[0.0, 0.0, float(f)]]], dtype=torch.float32, device=asset.device)
            asset.permanent_wrench_composer.set_forces_and_torques_index(forces=force, torques=torch.zeros_like(force),
                                                                         is_global=True)


def _kit_state(environment, spec) -> KitState:
    asset = environment.unwrapped.scene[spec.name]
    pose = _np(asset.data.root_link_pose_w)[0]
    pos, quat = pose[:3] - _origin(environment), pose[3:7]
    vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    if isinstance(spec, Card):
        pts = corners(pos, quat)
    elif isinstance(spec, Board):
        half = np.asarray(spec.size, float) / 2
        pts = np.array([pos - half, pos + half])
    else:
        pts = np.array([pos, pos])
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


# -- hooks ---------------------------------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []
_ENV: dict = {"env": None}


def on_step(fn):
    _STEP.append(fn)
    return fn


def on_reset(fn):
    _RESET.append(fn)
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
        _lift(self._env)
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
