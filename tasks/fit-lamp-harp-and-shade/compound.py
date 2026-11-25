"""Compound rigid bodies and passive articulated legs for the YAM task."""

import importlib.abc
import importlib.util
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState
from yam_arena.urdf import quat_xyzw_to_matrix

from geometry import FACES

_SPECS = {}


@dataclass(frozen=True)
class Link:
    name: str
    parent: str
    origin: tuple
    axis: str
    limits: tuple
    mass: float
    color: tuple
    boxes: tuple = ()
    cylinders: tuple = ()
    wedges: tuple = field(default=(), compare=False, hash=False)
    stiffness: float = 0.
    damping: float = 0.
    friction: float = .40


@dataclass(frozen=True)
class Part(Box):
    boxes: tuple = ()
    cylinders: tuple = ()
    wedges: tuple = field(default=(), compare=False, hash=False)
    friction: float = .4
    links: tuple = ()


@dataclass(frozen=True)
class PartState(ObjectState):
    links: dict = field(default_factory=dict)

    def rotation(self):
        return quat_xyzw_to_matrix(self.quat_xyzw)


def _spawn_part(prim_path, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    spec = _SPECS[cfg.part_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp().Set(Gf.Vec3d(*(translation or (0., 0., 0.))))
    x, y, z, w = orientation or (0., 0., 0., 1.)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
    visual_paths = {}
    locations = {"body": np.zeros(3)}

    def author_body(name, component, position):
        local = f"{prim_path}/{name}"
        node = UsdGeom.Xform.Define(stage, local)
        node.AddTranslateOp().Set(Gf.Vec3d(*position))
        body = node.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(body)
        UsdPhysics.MassAPI.Apply(body).CreateMassAttr(component.mass)
        PhysxSchema.PhysxContactReportAPI.Apply(body).CreateThresholdAttr(0.)
        rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(body)
        rigid.CreateSolverPositionIterationCountAttr(48)
        rigid.CreateSolverVelocityIterationCountAttr(8)
        rigid.CreateMaxDepenetrationVelocityAttr(.45)
        rigid.CreateAngularDampingAttr(.20)
        material_path = f"{prim_path}/Looks/physics_{name}"
        material = sim.RigidBodyMaterialCfg(static_friction=component.friction,
                                             dynamic_friction=component.friction)
        material.func(material_path, material)

        def collide(shape, color, offset):
            prim = shape.GetPrim()
            UsdPhysics.CollisionAPI.Apply(prim)
            api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            api.CreateContactOffsetAttr(offset)
            api.CreateRestOffsetAttr(0.)
            bind_physics_material(str(prim.GetPath()), material_path, stage=stage)
            if color not in visual_paths:
                path = f"{prim_path}/Looks/color_{len(visual_paths)}"
                surface = sim.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=.5)
                surface.func(path, surface)
                visual_paths[color] = path
            bind_visual_material(str(prim.GetPath()), visual_paths[color], stage=stage)

        for i, (center, size, color) in enumerate(component.boxes):
            box = UsdGeom.Cube.Define(stage, f"{local}/box_{i}")
            box.CreateSizeAttr(1.)
            box.AddTranslateOp().Set(Gf.Vec3d(*center))
            box.AddScaleOp().Set(Gf.Vec3f(*size))
            collide(box, color, .00012)
        for i, (center, radius, length, axis, color) in enumerate(component.cylinders):
            cyl = UsdGeom.Cylinder.Define(stage, f"{local}/cylinder_{i}")
            cyl.CreateAxisAttr(axis)
            cyl.CreateRadiusAttr(radius)
            cyl.CreateHeightAttr(length)
            cyl.AddTranslateOp().Set(Gf.Vec3d(*center))
            collide(cyl, color, .00011)
        for i, points in enumerate(component.wedges):
            mesh = UsdGeom.Mesh.Define(stage, f"{local}/wedge_{i}")
            mesh.CreateSubdivisionSchemeAttr("none")
            mesh.CreatePointsAttr([Gf.Vec3f(*v) for v in points])
            mesh.CreateFaceVertexCountsAttr([4]*len(FACES))
            mesh.CreateFaceVertexIndicesAttr([i for face in FACES for i in face])
            collide(mesh, component.color, .000035)
            UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
        return body

    body = author_body("body", spec, locations["body"])
    for link in spec.links:
        locations[link.name] = locations[link.parent]+link.origin
        author_body(link.name, link, locations[link.name])
        path = f"{prim_path}/{link.name}/joint_{link.name}"
        joint = UsdPhysics.RevoluteJoint.Define(stage,path)
        limits = np.degrees(link.limits)
        joint.CreateAxisAttr(link.axis)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(f"{prim_path}/{link.parent}")])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/{link.name}")])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*link.origin))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0.,0.,0.))
        joint.CreateLocalRot0Attr(Gf.Quatf(1.,0.,0.,0.))
        joint.CreateLocalRot1Attr(Gf.Quatf(1.,0.,0.,0.))
        joint.CreateLowerLimitAttr(float(limits[0]))
        joint.CreateUpperLimitAttr(float(limits[1]))
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(),"angular")
        factor = np.pi/180
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(float(link.stiffness*factor))
        drive.CreateDampingAttr(float(link.damping*factor))
        drive.CreateTargetPositionAttr(0.)
    if spec.links:
        UsdPhysics.ArticulationRootAPI.Apply(body)
        art = PhysxSchema.PhysxArticulationAPI.Apply(body)
        art.CreateEnabledSelfCollisionsAttr(False)
        art.CreateSolverPositionIterationCountAttr(48)
        art.CreateSolverVelocityIterationCountAttr(8)
    return root.GetPrim()


def _arena_object(spec):
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

    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg

    class CompoundObject(Object):
        def _generate_articulation_cfg(self):
            drives = {link.name: ImplicitActuatorCfg(joint_names_expr=[f"joint_{link.name}"],
                      stiffness=link.stiffness, damping=link.damping) for link in spec.links}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=drives)
            cfg = self._add_initial_pose_to_cfg(cfg)
            cfg.init_state.joint_pos = {f"joint_{link.name}": 0. for link in spec.links}
            return cfg

    return CompoundObject(name=spec.name, object_type=ObjectType.ARTICULATION if spec.links else ObjectType.RIGID,
                  spawner_cfg=PartSpawnerCfg(part_name=spec.name),
                  initial_pose=Pose(position_xyz=spec.pos, rotation_xyzw=(0.,0.,0.,1.)))

def _part_state(environment, spec):
    base = environment.unwrapped
    asset = base.scene[spec.name]

    def as_numpy(tensor):
        return getattr(tensor, "torch", tensor).detach().cpu().numpy().astype(float)

    origin = as_numpy(base.scene.env_origins[0])
    pose = as_numpy(asset.data.root_link_pose_w)[0]
    velocity = as_numpy(asset.data.root_vel_w)[0]
    members = {}
    if spec.links:
        for name, state in zip(asset.body_names,as_numpy(asset.data.body_link_pose_w)[0]):
            members[name] = (state[:3]-origin, quat_xyzw_to_matrix(state[3:7]))
    return PartState(pos=pose[:3]-origin, quat_xyzw=pose[3:7], lin_vel=velocity[:3], ang_vel=velocity[3:],
                     low=np.zeros(3), high=np.zeros(3), links=members)



# -- hooking yam_arena -------------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []
_ENV: dict = {"env": None}


def on_step(fn):
    """Register ``fn(env)`` to run after every control step, before the task file is scored."""
    _STEP.append(fn)
    return fn


def on_reset(fn):
    """Register ``fn(env)`` to run after every reset, once objects have settled."""
    _RESET.append(fn)
    return fn


def env():
    return _ENV["env"]


def _patch_scene(scene) -> None:
    if getattr(scene, "_a1_lamp_patched", False):
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
                out[n] = _part_state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._a1_lamp_patched = True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_a1_lamp_patched", False):
        return
    base_task_step, base_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        return base_task_step(self, *a, **k)

    def reset(self, *a, **k):
        if self._env is not None:
            import torch
            for spec in _SPECS.values():
                if spec.links:
                    asset = self._env.unwrapped.scene[spec.name]
                    zeros = torch.zeros_like(asset.data.joint_pos.torch)
                    asset.write_joint_state_to_sim_index(position=zeros, velocity=zeros)
                    asset.set_joint_position_target_index(target=zeros)
        obs = base_reset(self, *a, **k)
        _ENV["env"] = self._env
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._a1_lamp_patched = True


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
    """Hook yam_arena (now for modules already imported, otherwise when they are). Idempotent."""
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
