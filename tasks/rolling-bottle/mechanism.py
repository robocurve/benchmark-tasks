# This file is copied unchanged from robocurve/benchmark-tasks pull request #223 ("Open a panel held shut by a
# spring plunger", tasks/spring-plunger-panel/mechanism.py), which is published under the MIT License:
#
#   MIT License
#
#   Copyright (c) 2026 the authors of this task
#
#   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
#   documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
#   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
#   permit persons to whom the Software is furnished to do so, subject to the following conditions:
#
#   The above copyright notice and this permission notice shall be included in all copies or substantial portions of
#   the Software.
#
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
#   WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
#   OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
#   OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

"""A fixed mechanism with joints (a cabinet with a hinged panel and a spring plunger) for a yam_arena task file.

yam_arena's task files list single primitives (``Box``, ``Cylinder``, ``Sphere``) and library assets. This module adds
one more kind of object, :class:`Mechanism`: a fixed base and moving links, each built from boxes, cylinders and
capsules, joined by revolute or prismatic joints with springs, damping and friction. It becomes one fixed-base
Isaac Lab articulation. ``success()`` and ``score()`` receive a :class:`MechanismState` for it, with every joint's
position.

Importing this module hooks yam_arena without modifying it:

- ``yam_arena.scene.task_object`` builds a ``Mechanism`` as an articulation (other objects as before);
- ``yam_arena.scene.object_states`` reports a ``MechanismState`` for it;
- after ``YamArenaEmbodiment.reset`` returns, every mechanism's joints are put back to 0 at rest, its spring
  targets are set, the scene settles again as the reset does, and functions registered with :func:`on_reset` run;
- with the environment variable ``TOP_CAM_VIDEO=path.mp4`` set, the top camera of each episode is recorded there
  at its full render size (see :class:`_TopCamRecorder`).

The modules are patched when they are imported (or at once, if they already are). Nothing here imports Isaac Sim
at import time, so a task file that uses it still loads before the simulator starts.
"""

from __future__ import annotations

import atexit
import importlib.abc
import importlib.util
import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["Shape", "Link", "Joint", "Mechanism", "MechanismState", "on_reset", "install", "top_cam_frame"]

Color = tuple[float, float, float]
AXES = ("X", "Y", "Z")


@dataclass(frozen=True)
class Shape:
    """One collider, which is also what is drawn. ``center`` is in the mechanism's frame (m).

    ``kind`` "box": ``size`` is its x, y, z edge lengths. "cylinder" and "capsule": ``size`` is (radius, length)
    along ``axis`` (a capsule's length is that of its straight part, without the two caps)."""

    kind: str
    center: tuple[float, float, float]
    size: tuple[float, ...]
    color: Color = (0.7, 0.7, 0.7)
    axis: str = "Z"
    material: str = "default"
    contact_offset: float = 0.002

    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        c = np.asarray(self.center, float)
        if self.kind == "box":
            half = np.asarray(self.size, float) / 2
        else:
            radius, length = self.size
            half = np.full(3, radius)
            half[AXES.index(self.axis)] = length / 2 + (radius if self.kind == "capsule" else 0.0)
        return c - half, c + half


@dataclass(frozen=True)
class Link:
    """A rigid body: ``mass`` (kg) spread over its shapes."""

    name: str
    mass: float
    shapes: tuple[Shape, ...]


@dataclass(frozen=True)
class Joint:
    """A joint from the base to the link ``child``, through ``anchor`` (m, mechanism frame) along ``axis``.

    ``kind`` "revolute" (positions in rad) or "prismatic" (m); positive positions turn or slide the child about or
    along the axis by the right-hand rule. ``lower`` and ``upper`` are hard stops. A spring of ``stiffness``
    (N/m or N m/rad) pulls the joint toward ``target``, which may lie beyond a stop to preload it there. ``damping``
    is viscous (N s/m or N m s/rad); ``friction`` is the joint's friction as Isaac Lab's actuator takes it."""

    name: str
    kind: str
    child: str
    anchor: tuple[float, float, float]
    axis: str
    lower: float
    upper: float
    stiffness: float = 0.0
    damping: float = 0.0
    friction: float = 0.0
    target: float = 0.0


@dataclass(frozen=True)
class Mechanism(Box):
    """A fixed mechanism, its frame at ``pos`` (world, m) turned by ``yaw``. It subclasses ``Box`` only so that
    yam_arena's task-file checks accept it: ``size`` is the bounding box of its shapes at rest, and it has no
    ``mass`` (yam_arena treats it as fixed and never jitters it).

    ``materials`` maps each shape's ``material`` name to its (static, dynamic) friction."""

    base: Link | None = None
    links: tuple[Link, ...] = ()
    joints: tuple[Joint, ...] = ()
    materials: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        super()._check()
        if self.base is None or self.mass is not None:
            raise ValueError(f"{self.name}: a Mechanism needs a base link and no mass (it is fixed)")
        names = [self.base.name] + [link.name for link in self.links]
        if len(set(names)) != len(names):
            raise ValueError(f"{self.name}: link names must be unique, got {names}")
        for link in (self.base, *self.links):
            if not link.shapes or not link.mass > 0:
                raise ValueError(f"{self.name}.{link.name}: a link needs shapes and a positive mass")
            for shape in link.shapes:
                if shape.kind not in ("box", "cylinder", "capsule") or shape.axis not in AXES:
                    raise ValueError(f"{self.name}.{link.name}: unknown shape {shape.kind!r} or axis {shape.axis!r}")
                if shape.material not in self.materials:
                    raise ValueError(f"{self.name}.{link.name}: no friction given for material {shape.material!r}")
        for joint in self.joints:
            if joint.child not in names[1:] or joint.kind not in ("revolute", "prismatic") or joint.axis not in AXES:
                raise ValueError(f"{self.name}: bad joint {joint.name!r}")
            if not joint.lower < joint.upper:
                raise ValueError(f"{self.name}.{joint.name}: lower must be below upper")
        if sorted(j.child for j in self.joints) != sorted(names[1:]):
            raise ValueError(f"{self.name}: every moving link needs exactly one joint to the base")

    def all_links(self) -> tuple[Link, ...]:
        return (self.base, *self.links)


def mechanism(name: str, pos, base: Link, links, joints, materials: dict, yaw: float = 0.0,
              color: Color = (0.7, 0.7, 0.7)) -> Mechanism:
    """Build a Mechanism, its ``size`` filled in from its shapes' extent at rest."""
    lows, highs = zip(*(s.bounds() for link in (base, *links) for s in link.shapes))
    size = tuple((np.max(highs, axis=0) - np.min(lows, axis=0)).tolist())
    return Mechanism(name=name, pos=tuple(pos), size=size, yaw=yaw, color=color, base=base, links=tuple(links),
                     joints=tuple(joints), materials=dict(materials))


@dataclass(frozen=True)
class MechanismState(ObjectState):
    """The base's pose (``pos`` is its frame's origin), zero velocities (it is fixed), the world-aligned bounds of
    its shapes at rest, and each joint's position (rad or m, 0 at rest)."""

    joints: dict = field(default_factory=dict)


# -- spawning --------------------------------------------------------------------------------------------------------

_SPECS: dict[str, Mechanism] = {}


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the mechanism under ``prim_path``: an Xform (the articulation root) holding one rigid body per link,
    a fixed joint from the world to the base, and the joints. Bodies and shapes are in the mechanism's frame."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec: Mechanism = _SPECS[cfg.mechanism_name]
    stage = get_current_stage()
    double = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation if translation is not None else (0, 0, 0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))

    physics: dict[str, str] = {}
    for name, (static, dynamic) in spec.materials.items():
        path = f"{prim_path}/Looks/physics_{name}"
        material = sim_utils.RigidBodyMaterialCfg(static_friction=static, dynamic_friction=dynamic)
        material.func(path, material)
        physics[name] = path
    looks: dict[Color, str] = {}

    def look(color: Color) -> str:
        if color not in looks:
            path = f"{prim_path}/Looks/color_{len(looks)}"
            surface = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
            surface.func(path, surface)
            looks[color] = path
        return looks[color]

    def collider(prim, shape: Shape) -> None:
        UsdPhysics.CollisionAPI.Apply(prim)
        physx = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        physx.CreateContactOffsetAttr(shape.contact_offset)
        physx.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), physics[shape.material], stage=stage)
        bind_visual_material(str(prim.GetPath()), look(shape.color), stage=stage)

    def author_shape(path: str, shape: Shape) -> None:
        if shape.kind == "box":
            geom = UsdGeom.Cube.Define(stage, path)
            geom.CreateSizeAttr(1.0)
            geom.AddTranslateOp().Set(Gf.Vec3d(*shape.center))
            geom.AddScaleOp().Set(Gf.Vec3f(*shape.size))
        else:
            geom = (UsdGeom.Cylinder if shape.kind == "cylinder" else UsdGeom.Capsule).Define(stage, path)
            geom.CreateAxisAttr(shape.axis)
            geom.CreateRadiusAttr(shape.size[0])
            geom.CreateHeightAttr(shape.size[1])
            geom.AddTranslateOp().Set(Gf.Vec3d(*shape.center))
        collider(geom.GetPrim(), shape)

    bodies = {}
    for link in spec.all_links():
        path = f"{prim_path}/{link.name}"
        body = UsdGeom.Xform.Define(stage, path)
        body.AddTranslateOp(precision=double).Set(Gf.Vec3d(0.0, 0.0, 0.0))
        body.AddOrientOp(precision=double).Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
        prim = body.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(link.mass)
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateMaxDepenetrationVelocityAttr(1.0)
        for i, shape in enumerate(link.shapes):
            author_shape(f"{path}/{shape.kind}_{i}", shape)
        bodies[link.name] = Sdf.Path(path)

    # The base is fixed to the world where it stands. With the articulation root on the parent Xform (not on the
    # base body), PhysX makes this a fixed-base articulation rather than a body held by a joint.
    base_prim = stage.GetPrimAtPath(bodies[spec.base.name])
    to_world = UsdGeom.Xformable(base_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    fixed = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/{spec.base.name}/fixed_to_world")
    fixed.CreateBody1Rel().SetTargets([bodies[spec.base.name]])
    fixed.CreateLocalPos0Attr(Gf.Vec3f(to_world.ExtractTranslation()))
    fixed.CreateLocalRot0Attr(Gf.Quatf(to_world.ExtractRotationQuat()))
    fixed.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    fixed.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    for joint in spec.joints:
        path = f"{prim_path}/{joint.child}/{joint.name}"
        revolute = joint.kind == "revolute"
        usd_joint = (UsdPhysics.RevoluteJoint if revolute else UsdPhysics.PrismaticJoint).Define(stage, path)
        usd_joint.CreateAxisAttr(joint.axis)
        usd_joint.CreateBody0Rel().SetTargets([bodies[spec.base.name]])
        usd_joint.CreateBody1Rel().SetTargets([bodies[joint.child]])
        # Both bodies' frames are the mechanism's frame at rest, so the anchor is the same point in each.
        usd_joint.CreateLocalPos0Attr(Gf.Vec3f(*joint.anchor))
        usd_joint.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        usd_joint.CreateLocalPos1Attr(Gf.Vec3f(*joint.anchor))
        usd_joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        unit = math.degrees(1.0) if revolute else 1.0  # USD gives revolute limits and targets in degrees
        usd_joint.CreateLowerLimitAttr(joint.lower * unit)
        usd_joint.CreateUpperLimitAttr(joint.upper * unit)
        drive = UsdPhysics.DriveAPI.Apply(usd_joint.GetPrim(), "angular" if revolute else "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(joint.stiffness / unit)
        drive.CreateDampingAttr(joint.damping / unit)
        drive.CreateTargetPositionAttr(joint.target * unit)

    # The moving links collide with one another (the plunger's pin with the panel's strike) but not with the base,
    # whose parts they pass through or rest against by design (the pin runs inside its guide housing).
    UsdPhysics.FilteredPairsAPI.Apply(base_prim).CreateFilteredPairsRel().SetTargets(
        [bodies[link.name] for link in spec.links])
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    articulation = PhysxSchema.PhysxArticulationAPI.Apply(root.GetPrim())
    articulation.CreateEnabledSelfCollisionsAttr(True)
    articulation.CreateSolverPositionIterationCountAttr(32)
    articulation.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


def _arena_object(spec: Mechanism):
    """The mechanism as an Isaac Lab-Arena object: an articulation whose joints are implicit actuators carrying
    each joint's spring, damping and friction."""
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    _SPECS[spec.name] = spec

    @configclass
    class MechanismSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn)
        mechanism_name: str = ""

    class MechanismObject(Object):
        def _generate_articulation_cfg(self):
            actuators = {
                joint.name: ImplicitActuatorCfg(joint_names_expr=[joint.name], stiffness=joint.stiffness,
                                                damping=joint.damping, friction=joint.friction)
                for joint in spec.joints
            }
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    return MechanismObject(
        name=spec.name,
        object_type=ObjectType.ARTICULATION,
        spawner_cfg=MechanismSpawnerCfg(mechanism_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos),
                          rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))),
    )


# -- state, reset ----------------------------------------------------------------------------------------------------


def _torch(x):
    return getattr(x, "torch", x)  # Isaac Lab 3.0 wraps some buffers


def _numpy(x) -> np.ndarray:
    return _torch(x).detach().cpu().numpy().astype(np.float64)


def _joint_ids(asset, spec: Mechanism) -> list[int]:
    names = list(asset.joint_names)
    return [names.index(joint.name) for joint in spec.joints]


def _state(environment, spec: Mechanism) -> MechanismState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _numpy(base.scene.env_origins[0])
    pose = _numpy(asset.data.root_link_pose_w)[0]
    q = _numpy(asset.data.joint_pos)[0]
    joints = {joint.name: float(q[i]) for joint, i in zip(spec.joints, _joint_ids(asset, spec), strict=True)}
    half = np.asarray(spec.size) / 2
    lows, highs = zip(*(s.bounds() for link in spec.all_links() for s in link.shapes))
    centre = (np.min(lows, axis=0) + np.max(highs, axis=0)) / 2
    c, s = math.cos(spec.yaw), math.sin(spec.yaw)
    middle = pose[:3] - origin + np.array([c * centre[0] - s * centre[1], s * centre[0] + c * centre[1], centre[2]])
    reach = np.array([abs(c) * half[0] + abs(s) * half[1], abs(s) * half[0] + abs(c) * half[1], half[2]])
    return MechanismState(pos=pose[:3] - origin, quat_xyzw=pose[3:7], lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                          low=middle - reach, high=middle + reach, joints=joints)


def _rest(environment, spec: Mechanism) -> None:
    """Every joint back to 0 and still, and each spring's target set (a target beyond a stop preloads it)."""
    import torch

    base = environment.unwrapped
    asset = base.scene[spec.name]
    ids = _joint_ids(asset, spec)
    device = _torch(asset.data.joint_pos).device
    zeros = torch.zeros((1, len(ids)), dtype=torch.float32, device=device)
    targets = torch.tensor([[joint.target for joint in spec.joints]], dtype=torch.float32, device=device)
    if hasattr(asset, "write_joint_state_to_sim_index"):  # Isaac Lab 3.0: keyword-only writers
        asset.write_joint_state_to_sim_index(position=zeros, velocity=zeros, joint_ids=ids)
        asset.set_joint_position_target_index(target=targets, joint_ids=ids)
        asset.set_joint_velocity_target_index(target=zeros, joint_ids=ids)
    else:  # Isaac Lab 2.x
        asset.write_joint_state_to_sim(zeros, zeros, joint_ids=ids)
        asset.set_joint_position_target(targets, joint_ids=ids)
        asset.set_joint_velocity_target(zeros, joint_ids=ids)
    asset.write_data_to_sim()


_RESET: list = []


def on_reset(fn):
    """Register ``fn()`` to run at the end of every reset, once the mechanisms are back at rest."""
    _RESET.append(fn)
    return fn


# -- hooking yam_arena -----------------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_mechanism_patched", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def patched_task_object(spec):
        return _arena_object(spec) if isinstance(spec, Mechanism) else task_object(spec)

    def patched_object_states(environment, names):
        others = [n for n in names if n not in _SPECS]
        out = object_states(environment, others) if others else {}
        out.update({n: _state(environment, _SPECS[n]) for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states = patched_task_object, patched_object_states
    scene._mechanism_patched = True


TOP_CAM_VIDEO = "TOP_CAM_VIDEO"  # environment variable: record the top camera to this MP4


def top_cam_frame(embodiment) -> np.ndarray:
    """The station top camera's latest render at its full size (640 x 360), before yam_arena resizes it for the
    policy's observation."""
    out = embodiment._base.scene["top_cam"].data.output["rgb"]
    out = out.torch if hasattr(out, "torch") else out
    return np.ascontiguousarray(out[0, ..., :3].cpu().numpy(), dtype=np.uint8)


class _TopCamRecorder:
    """With ``TOP_CAM_VIDEO=path.mp4`` set, each episode's top camera is written to that H.264 MP4 at the control rate:
    the frame at the reset, then one per control step, at the camera's full render size whatever the policy's
    ``cam_width`` / ``cam_height``. The first frame is also saved beside it as ``path.png`` (the setup screenshot).
    The variable is read at each reset, so one process can record several episodes to different files."""

    def __init__(self) -> None:
        self.writer, self._exit_hook = None, False

    def start(self, embodiment) -> None:
        self.close()
        path = os.environ.get(TOP_CAM_VIDEO)
        if not path:
            return
        import imageio.v2 as imageio
        import imageio_ffmpeg

        path = os.path.abspath(os.path.expanduser(path))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        frame = top_cam_frame(embodiment)
        imageio.imwrite(os.path.splitext(path)[0] + ".png", frame)
        h, w = frame.shape[:2]
        hz = round(1.0 / embodiment._base.step_dt, 6)
        self.writer = imageio_ffmpeg.write_frames(
            path, (w, h), pix_fmt_in="rgb24", pix_fmt_out="yuv420p", fps=hz, quality=None, codec="libx264",
            macro_block_size=8, output_params=["-crf", "20", "-movflags", "+faststart"])
        self.writer.send(None)
        self.writer.send(frame)
        if not self._exit_hook:  # registered after yam_arena's (at the simulator's start), so it runs before Kit closes
            atexit.register(self.close)
            self._exit_hook = True

    def add(self, embodiment) -> None:
        if self.writer is not None:
            self.writer.send(top_cam_frame(embodiment))

    def close(self) -> None:
        if self.writer is not None:
            writer, self.writer = self.writer, None
            writer.close()


_RECORDER = _TopCamRecorder()


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_mechanism_patched", False):
        return
    reset, step, close = cls.reset, cls.step, cls.close

    def patched_reset(self, scene, *, seed=None):
        observation = reset(self, scene, seed=seed)
        specs = [o for o in (self.task.objects if self.task is not None else ()) if isinstance(o, Mechanism)]
        if specs:
            for spec in specs:
                _rest(self._env, _SPECS.get(spec.name, spec))
            base = self._base
            for _ in range(max(1, round(module.SETTLE_S / base.step_dt))):  # settle again, as the reset does
                self._env.step(self._ready)
            for _ in range(module.RESET_RENDERS):
                base.sim.render()
            self._t = 0.0
            for fn in _RESET:
                fn()
            observation = self._observation()
        _RECORDER.start(self)
        return observation

    def patched_step(self, action):
        result = step(self, action)
        _RECORDER.add(self)
        return result

    def patched_close(self):
        _RECORDER.close()
        return close(self)

    cls.reset, cls.step, cls.close = patched_reset, patched_step, patched_close
    cls._mechanism_patched = True


_PATCHES = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}


class _PatchOnImport(importlib.abc.MetaPathFinder):
    """Finds the modules in ``_PATCHES`` with the other finders, and patches each right after it executes."""

    def find_spec(self, fullname, path, target=None):
        if fullname not in _PATCHES:
            return None
        for finder in sys.meta_path:
            if finder is self or not hasattr(finder, "find_spec"):
                continue
            spec = finder.find_spec(fullname, path, target)
            if spec is not None:
                break
        else:
            return None
        if spec.loader is None:
            return None
        execute, patch = spec.loader.exec_module, _PATCHES[fullname]

        def exec_module(module):
            execute(module)
            patch(module)

        spec.loader.exec_module = exec_module
        return spec


def install() -> None:
    """Patch the yam_arena modules already imported, and the others when they are. Safe to call again."""
    for name, patch in _PATCHES.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _PatchOnImport) for f in sys.meta_path):
        sys.meta_path.insert(0, _PatchOnImport())


install()
