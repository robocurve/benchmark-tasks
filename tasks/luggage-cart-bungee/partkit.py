"""partkit: rigid parts built from boxes for yam_arena task files, plus a per-physics-step callback.

Keep this file next to the task file. yam_arena's task-file objects are single primitives; a ``Part`` is one rigid
body made of several box colliders, with any start orientation. A part with ``ghost=True`` is a kinematic body with
no collider: the task moves it each control step (here, the segments that draw the elastic cord).

Importing this module hooks ``yam_arena.scene`` and the embodiment without modifying yam_arena:
``success()/score()`` receive an ``ObjectState`` whose ``pos`` is the part's body origin. ``on_substep(fn)``
registers ``fn(env)`` to run before every physics step (240 Hz), ``on_step(fn)`` after every control step and
``on_reset(fn)`` after every reset. Pure Python at import time; Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena import task_file
from yam_arena.task_file import Box, ObjectState

__all__ = ["Part", "part", "on_reset", "on_step", "on_substep", "set_pose", "pose", "add_force", "add_forces",
           "gripper_points"]


@dataclass(frozen=True)
class Part(Box):
    """A rigid part. It subclasses Box only so yam_arena's task-file checks accept it; ``size`` is a bounding box and
    ``pos`` the body origin. ``boxes`` are (centre, size, colour[, quat xyzw]) in the part frame. ``quat`` (x, y, z, w) is its
    start orientation."""

    boxes: tuple = ()
    cylinders: tuple = ()      # (centre, radius, length, colour, axis "X" | "Y" | "Z")
    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    friction: float = 1.0
    ghost: bool = False        # kinematic, no collider (moved by the task)
    fixed: bool = False        # kinematic, with colliders (stays where it is put)
    damping: float = 0.5       # angular damping (1/s)


def part(name, pos, boxes=(), mass=1.0, quat=(0.0, 0.0, 0.0, 1.0), color=(0.5, 0.5, 0.5), friction=1.0, ghost=False,
         cylinders=(), fixed=False, damping=0.5):
    half = [np.array(b[1]) / 2 * (1.0 if len(b) < 4 else 1.42) for b in boxes]
    for _, r, h, _, ax in cylinders:
        half.append(np.array([h / 2 if ax == "X" else r, h / 2 if ax == "Y" else r, h / 2 if ax == "Z" else r]))
    centres = np.array([b[0] for b in boxes] + [c[0] for c in cylinders], float)
    ext = np.max(np.abs(centres) + np.array(half), axis=0) * 2
    return Part(name=name, pos=tuple(float(v) for v in pos), size=tuple(ext.tolist()), mass=mass, color=color,
                boxes=tuple(boxes), cylinders=tuple(cylinders), quat=tuple(float(v) for v in quat),
                friction=friction, ghost=ghost, fixed=fixed, damping=damping)


_SPECS: dict[str, Part] = {}


def _spawn_part(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec: Part = _SPECS[cfg.part_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
    phys = f"{prim_path}/Looks/physics"
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction)
    pm.func(phys, pm)
    looks: dict = {}

    def look(color):
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.45)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    main = f"{prim_path}/body"
    xform = UsdGeom.Xform.Define(stage, main)
    xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0, 0, 0))
    xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
    prim = xform.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    if spec.ghost or spec.fixed:
        UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(spec.mass)
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateMaxDepenetrationVelocityAttr(1.0)
    rb.CreateSolverPositionIterationCountAttr(32)
    rb.CreateAngularDampingAttr(spec.damping)
    rb.CreateLinearDampingAttr(1.0 if spec.damping > 0.5 else 0.0)
    rb.CreateEnableCCDAttr(not (spec.ghost or spec.fixed))
    shapes = []
    for i, entry in enumerate(spec.boxes):
        centre, size, color = entry[:3]
        cube = UsdGeom.Cube.Define(stage, f"{main}/box_{i}")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
        if len(entry) > 3:                         # optional orientation (x, y, z, w) of this box in the part
            qx, qy, qz, qw = entry[3]
            cube.AddOrientOp().Set(Gf.Quatf(qw, qx, qy, qz))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        shapes.append((cube.GetPrim(), color))
    for i, (centre, radius, length, color, axis) in enumerate(spec.cylinders):
        cyl = UsdGeom.Cylinder.Define(stage, f"{main}/cyl_{i}")
        cyl.CreateAxisAttr(axis)
        cyl.CreateRadiusAttr(radius)
        cyl.CreateHeightAttr(length)
        cyl.AddTranslateOp().Set(Gf.Vec3d(*centre))
        shapes.append((cyl.GetPrim(), color))
    for p, color in shapes:
        if not spec.ghost:
            UsdPhysics.CollisionAPI.Apply(p)
            cp = PhysxSchema.PhysxCollisionAPI.Apply(p)
            cp.CreateContactOffsetAttr(0.002)
            cp.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(p.GetPath()), phys, stage=stage)
        bind_visual_material(str(p.GetPath()), look(color), stage=stage)
    return root.GetPrim()


def _arena_object(spec: Part):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class PartSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_part)
        part_name: str = ""

    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=PartSpawnerCfg(part_name=spec.name),
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=tuple(spec.quat)))


def _t(x):
    return getattr(x, "torch", x)


def pose(environment, name: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Body origin (env frame), quaternion xyzw and linear velocity of a part (env 0)."""
    base = environment.unwrapped
    asset = base.scene[name]
    origin = _t(base.scene.env_origins)[0].detach().cpu().numpy().astype(float)
    p = _t(asset.data.root_link_pose_w)[0].detach().cpu().numpy().astype(float)
    v = _t(asset.data.root_lin_vel_w)[0].detach().cpu().numpy().astype(float)
    return p[:3] - origin, p[3:7], v


def set_pose(environment, name: str, pos, quat_xyzw) -> None:
    import torch

    base = environment.unwrapped
    asset = base.scene[name]
    origin = _t(base.scene.env_origins)[0]
    t = torch.tensor([[*pos, *quat_xyzw]], dtype=torch.float32, device=origin.device)
    t[:, :3] += origin
    asset.write_root_pose_to_sim_index(root_pose=t)


def add_force(environment, name: str, force, point) -> None:
    """Apply ``force`` (N, world) at ``point`` (env frame) on a part for the next physics step: the force at the
    body's centre of mass plus the matching torque, both in the world frame."""
    import torch

    base = environment.unwrapped
    asset = base.scene[name]
    origin = _t(base.scene.env_origins)[0].detach().cpu().numpy().astype(float)
    com = _t(asset.data.root_com_pos_w)[0].detach().cpu().numpy().astype(float) - origin
    f = np.asarray(force, float)
    tq = np.cross(np.asarray(point, float) - com, f)
    dev = asset.device
    asset.permanent_wrench_composer.set_forces_and_torques_index(
        forces=torch.tensor([[f.tolist()]], dtype=torch.float32, device=dev),
        torques=torch.tensor([[tq.tolist()]], dtype=torch.float32, device=dev), is_global=True)


def add_forces(environment, name: str, pairs) -> None:
    """Apply several (force, point) pairs (N, world; env frame) on a part for the next physics step, summed."""
    import torch

    base = environment.unwrapped
    asset = base.scene[name]
    origin = _t(base.scene.env_origins)[0].detach().cpu().numpy().astype(float)
    com = _t(asset.data.root_com_pos_w)[0].detach().cpu().numpy().astype(float) - origin
    f, tq = np.zeros(3), np.zeros(3)
    for force, point in pairs:
        force = np.asarray(force, float)
        f += force
        tq += np.cross(np.asarray(point, float) - com, force)
    dev = asset.device
    asset.permanent_wrench_composer.set_forces_and_torques_index(
        forces=torch.tensor([[f.tolist()]], dtype=torch.float32, device=dev),
        torques=torch.tensor([[tq.tolist()]], dtype=torch.float32, device=dev), is_global=True)


def _rotm(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def gripper_points(environment) -> dict:
    """Points (env frame) on each gripper: the palm, the centre between the finger pads and each pad (tip and
    middle), keyed by arm; and each gripper's opening (m between the pads)."""
    base = environment.unwrapped
    origin = _t(base.scene.env_origins)[0].detach().cpu().numpy().astype(float)
    out = {}
    for key in ("left_robot", "right_robot"):
        robot = base.scene[key]
        names = list(robot.body_names)
        g = _t(robot.data.body_link_pose_w)[0].detach().cpu().numpy().astype(float)[names.index("gripper")]
        R, p = _rotm(g[3:7]), g[:3] - origin
        jn = list(robot.joint_names)
        half = abs(float(_t(robot.data.joint_pos)[0].detach().cpu().numpy()[jn.index("joint7")])) + 0.008
        pts = [p + R @ np.array(local, float) for local in ((0, 0, 0.06), (0, 0, 0.134), (half, 0, 0.147),
                                                              (-half, 0, 0.147), (half, 0, 0.12), (-half, 0, 0.12))]
        out[key.split("_")[0]] = (pts, 2 * half - 0.016)
    return out


def _state(environment, spec: Part) -> ObjectState:
    p, q, v = pose(environment, spec.name)
    x, y, z, w = q
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * np.array(spec.size) / 2
    pts = p + corners @ r.T
    return ObjectState(pos=p, quat_xyzw=q, lin_vel=v, ang_vel=np.zeros(3), low=pts.min(0), high=pts.max(0))


# -- hooking yam_arena -------------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []
_SUB: list = []


def on_step(fn):
    _STEP.append(fn)
    return fn


def on_reset(fn):
    _RESET.append(fn)
    return fn


def on_substep(fn):
    _SUB.append(fn)
    return fn


def _patch_scene(scene) -> None:
    if getattr(scene, "_partkit_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Part):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = orig_states(environment, plain) if plain else {}
        for n in names:
            if n in _SPECS:
                out[n] = _state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._partkit_patched = True


def _wrap_scene_writes(environment) -> None:
    """Run the substep callbacks just before the scene writes its data to the simulator (every physics step)."""
    sc = environment.unwrapped.scene
    if getattr(sc, "_partkit_wrapped", False):
        return
    orig = sc.write_data_to_sim

    def write_data_to_sim(*a, **k):
        for fn in _SUB:
            fn(environment)
        return orig(*a, **k)

    sc.write_data_to_sim = write_data_to_sim
    sc._partkit_wrapped = True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_partkit_patched", False):
        return
    base_task_step, base_reset, base_ensure = cls._task_step, cls.reset, cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        out = base_ensure(self, *a, **k)
        if self._env is not None:
            for fn in _RESET:
                fn(self._env)
            _wrap_scene_writes(self._env)
        return out

    def _task_step(self, *a, **k):
        for fn in _STEP:
            fn(self._env)
        return base_task_step(self, *a, **k)

    def reset(self, *a, **k):
        if self._env is not None:   # clear the task's state before the reset's settling steps
            for fn in _RESET:
                fn(self._env)
        obs = base_reset(self, *a, **k)
        for fn in _STEP:
            fn(self._env)
        return obs

    cls._ensure_sim, cls._task_step, cls.reset = _ensure_sim, _task_step, reset
    cls._partkit_patched = True


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
        orig_exec = loader.exec_module

        def exec_module(module):
            orig_exec(module)
            patch(module)

        loader.exec_module = exec_module
        return spec


def install() -> None:
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
assert task_file
