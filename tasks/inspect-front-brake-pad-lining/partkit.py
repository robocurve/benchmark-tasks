"""Rigid compound meshes for YAM task files, adapted from the tray part helper."""

import importlib.abc
import importlib.util
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState
from yam_arena.urdf import quat_xyzw_to_matrix

FACES = [(0, 1, 2, 3), (7, 6, 5, 4), (0, 4, 5, 1), (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)]

_SPECS = {}


@dataclass(frozen=True)
class Part(Box):
    boxes: tuple = ()
    cylinders: tuple = ()
    wedges: tuple = field(default=(), compare=False, hash=False)
    marks: tuple = ()
    friction: float = .4
    orientation: tuple = (0., 0., 0., 1.)
    mounted: bool = False


@dataclass(frozen=True)
class PartState(ObjectState):
    def rotation(self):
        return quat_xyzw_to_matrix(self.quat_xyzw)


def _spawn_part(prim_path, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    spec = _SPECS[cfg.part_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp().Set(Gf.Vec3d(*(translation or (0., 0., 0.))))
    x, y, z, w = orientation or (0., 0., 0., 1.)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
    body_path = f"{prim_path}/body"
    body = UsdGeom.Xform.Define(stage, body_path).GetPrim()
    rigid = UsdPhysics.RigidBodyAPI.Apply(body)
    if spec.mounted:
        rigid.CreateKinematicEnabledAttr(True)
    PhysxSchema.PhysxContactReportAPI.Apply(body).CreateThresholdAttr(0.)
    UsdPhysics.MassAPI.Apply(body).CreateMassAttr(spec.mass)
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(body)
    rb.CreateSolverPositionIterationCountAttr(32)
    rb.CreateSolverVelocityIterationCountAttr(4)
    rb.CreateMaxDepenetrationVelocityAttr(.5)

    material_path = f"{prim_path}/Looks/physics"
    material = sim.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction)
    material.func(material_path, material)
    colors = {}

    def paint(shape, color):
        prim = shape.GetPrim()
        if color not in colors:
            path = f"{prim_path}/Looks/color_{len(colors)}"
            surface = sim.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=.5)
            surface.func(path, surface)
            colors[color] = path
        bind_visual_material(str(prim.GetPath()), colors[color], stage=stage)

    def collide(shape, color, offset):
        prim = shape.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        api.CreateContactOffsetAttr(offset)
        api.CreateRestOffsetAttr(0.)
        bind_physics_material(str(prim.GetPath()), material_path, stage=stage)
        paint(shape,color)

    for i, (center, size, color) in enumerate(spec.boxes):
        box = UsdGeom.Cube.Define(stage, f"{body_path}/box_{i}")
        box.CreateSizeAttr(1.)
        box.AddTranslateOp().Set(Gf.Vec3d(*center))
        box.AddScaleOp().Set(Gf.Vec3f(*size))
        collide(box, color, .0005)

    for i, (center, size, color) in enumerate(spec.marks):
        mark = UsdGeom.Cube.Define(stage, f"{body_path}/mark_{i}")
        mark.CreateSizeAttr(1.)
        mark.AddTranslateOp().Set(Gf.Vec3d(*center))
        mark.AddScaleOp().Set(Gf.Vec3f(*size))
        paint(mark,color)

    for i, (center, radius, length, axis, color) in enumerate(spec.cylinders):
        cyl = UsdGeom.Cylinder.Define(stage, f"{body_path}/cylinder_{i}")
        cyl.CreateAxisAttr(axis)
        cyl.CreateRadiusAttr(radius)
        cyl.CreateHeightAttr(length)
        cyl.AddTranslateOp().Set(Gf.Vec3d(*center))
        collide(cyl, color, .0005)

    for i, vertices in enumerate(spec.wedges):
        mesh = UsdGeom.Mesh.Define(stage, f"{body_path}/wedge_{i}")
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreatePointsAttr([Gf.Vec3f(*v) for v in vertices])
        mesh.CreateFaceVertexCountsAttr([4] * len(FACES))
        mesh.CreateFaceVertexIndicesAttr([i for face in FACES for i in face])
        collide(mesh, spec.color, .00005)
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")

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

    return Object(name=spec.name, object_type=ObjectType.RIGID,
                  spawner_cfg=PartSpawnerCfg(part_name=spec.name),
                  initial_pose=Pose(position_xyz=spec.pos, rotation_xyzw=spec.orientation))


def _part_state(environment, spec):
    base = environment.unwrapped
    asset = base.scene[spec.name]

    def as_numpy(tensor):
        return getattr(tensor, "torch", tensor).detach().cpu().numpy().astype(float)

    origin = as_numpy(base.scene.env_origins[0])
    pose = as_numpy(asset.data.root_link_pose_w)[0]
    velocity = as_numpy(asset.data.root_vel_w)[0]
    return PartState(pos=pose[:3] - origin, quat_xyzw=pose[3:7], lin_vel=velocity[:3], ang_vel=velocity[3:],
                     low=np.zeros(3), high=np.zeros(3))


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
    if getattr(scene, "_d4_compound_patched", False):
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
    scene._d4_compound_patched = True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_d4_compound_patched", False):
        return
    base_task_step, base_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        return base_task_step(self, *a, **k)

    def reset(self, *a, **k):
        obs = base_reset(self, *a, **k)
        _ENV["env"] = self._env
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._d4_compound_patched = True


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
