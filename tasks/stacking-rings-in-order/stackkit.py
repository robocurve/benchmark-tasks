"""stackkit: rigid bodies built from several shapes (boxes, cylinders, spheres and convex meshes) for yam_arena task
files. Adapted from the bodykit module used by other tasks in this repository (its Body, shapes and hooks), with two
additions: :class:`SphereShape`, :class:`HullShape` (a mesh given by its points and triangles), ``Body.weld_to`` (a
fixed joint to another Body) and ``Body.ignore_robots`` (no collisions with either arm).

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).parent))
    from stackkit import Body, BoxShape, CylShape, HullShape, SphereShape, on_reset, on_step

Importing it hooks ``yam_arena.scene`` without changing yam_arena: a Body is spawned as one rigid body, and
``success()``/``score()`` receive a :class:`BodyState` for it whose ``pos`` and ``quat_xyzw`` are the body origin's
pose. It also hooks the embodiment's ``reset()`` and step, so a task file can clear what it tracks over an episode
(:func:`on_reset`) and read the simulation after every control step (:func:`on_step`). Nothing here applies forces or
joints during an episode: every Body moves only by PhysX rigid-body contact, friction and gravity (and a fixed joint
welding two Bodies into one, ``weld_to``, authored once at spawn). Pure Python at import time:
everything that needs Isaac Sim is imported lazily, so the task file still loads without the simulator.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["Body", "BodyState", "BoxShape", "CylShape", "HullShape", "SphereShape", "as_numpy", "install", "on_reset",
           "on_step", "rotation", "state_from_pose"]

Color = tuple[float, float, float]
PHYSICS_DT = 1 / 240


@dataclass(frozen=True)
class BoxShape:
    """A box in the body frame: ``centre`` and ``size`` (m), turned ``yaw`` (rad) about the body's z axis.
    ``color=None``: collision only, not rendered. ``collide=False``: rendered only. """

    centre: tuple[float, float, float]
    size: tuple[float, float, float]
    color: Color | None = (0.6, 0.6, 0.6)
    yaw: float = 0.0
    collide: bool = True

    def quat(self) -> tuple[float, float, float, float]:
        return (0.0, 0.0, math.sin(self.yaw / 2), math.cos(self.yaw / 2))

    def half_extent(self) -> np.ndarray:
        return np.abs(rotation(self.quat())) @ (np.array(self.size) / 2)


@dataclass(frozen=True)
class CylShape:
    """A cylinder in the body frame, its axis along the body's ``axis`` ("X", "Y" or "Z")."""

    centre: tuple[float, float, float]
    radius: float
    height: float
    color: Color | None = (0.6, 0.6, 0.6)
    collide: bool = True
    axis: str = "Z"

    def half_extent(self) -> np.ndarray:
        half = np.full(3, self.radius)
        half["XYZ".index(self.axis)] = self.height / 2
        return half


@dataclass(frozen=True)
class SphereShape:
    """A sphere in the body frame."""

    centre: tuple[float, float, float]
    radius: float
    color: Color | None = (0.6, 0.6, 0.6)
    collide: bool = True

    def half_extent(self) -> np.ndarray:
        return np.full(3, self.radius)


@dataclass(frozen=True)
class HullShape:
    """A convex solid in the body frame: ``points`` (m, body frame) and the triangles of its surface (``faces``, index
    triples). PhysX collides with it as the convex hull of the points (a convex mesh, at most 255 points), or, with
    ``sdf=True``, as the triangle mesh itself through a signed distance field (fine enough to keep a curved surface
    curved); it is drawn as the triangles."""

    points: tuple
    faces: tuple
    color: Color | None = (0.6, 0.6, 0.6)
    collide: bool = True
    sdf: bool = False      # collide as a signed-distance-field triangle mesh (curved, not a convex hull)

    @property
    def centre(self) -> tuple[float, float, float]:
        p = np.asarray(self.points, float)
        return tuple(((p.min(0) + p.max(0)) / 2).tolist())

    def half_extent(self) -> np.ndarray:
        p = np.asarray(self.points, float)
        return (p.max(0) - p.min(0)) / 2


@dataclass(frozen=True)
class Body(Box):
    """A rigid body made of ``shapes``. It subclasses Box only so yam_arena's task-file checks accept it: ``pos`` is
    the body origin (world, m) and ``size`` its bounding box. ``friction`` is (static, dynamic); ``damping`` is
    (linear, angular). A ``kinematic`` body stays where it is put. ``com`` is the centre of mass in the body frame.
    ``ignore`` names bodies (earlier in OBJECTS) it never collides with."""

    shapes: tuple = ()
    kinematic: bool = False
    friction: tuple[float, float] = (0.6, 0.5)
    damping: tuple[float, float] = (0.0, 0.05)
    contact_offset: float = 0.002
    solver_iterations: int = 16
    com: tuple[float, float, float] | None = None
    ignore: tuple[str, ...] = ()
    ignore_robots: bool = False   # also never collide with either arm
    weld_to: str | None = None    # welded (a fixed joint) to this Body (earlier in OBJECTS) ...
    weld_at: tuple[float, float, float] = (0.0, 0.0, 0.0)   # ... with this body's origin at this point of its frame
    ccd: bool = False
    start_xyzw: tuple[float, float, float, float] | None = None  # the starting orientation, if not just ``yaw``

    @staticmethod
    def build(name: str, pos, mass: float, shapes, color: Color = (0.6, 0.6, 0.6), **kw) -> "Body":
        """A Body whose ``size`` is the extent of its shapes."""
        lo = np.min([np.array(s.centre) - s.half_extent() for s in shapes], axis=0)
        hi = np.max([np.array(s.centre) + s.half_extent() for s in shapes], axis=0)
        return Body(name=name, pos=tuple(float(v) for v in pos), size=tuple((hi - lo).tolist()), mass=mass,
                    color=color, shapes=tuple(shapes), **kw)

    def local_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        lo = np.min([np.array(s.centre) - s.half_extent() for s in self.shapes], axis=0)
        hi = np.max([np.array(s.centre) + s.half_extent() for s in self.shapes], axis=0)
        return lo, hi

    def _check(self) -> None:
        super()._check()
        if not self.shapes:
            raise ValueError(f"{self.name}: a Body needs at least one shape")
        if self.mass is None:
            raise ValueError(f"{self.name}: give a Body a mass (use kinematic=True for a fixed one)")


def rotation(quat_xyzw) -> np.ndarray:
    x, y, z, w = quat_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


@dataclass(frozen=True)
class BodyState(ObjectState):
    """``pos``/``quat_xyzw``: the body origin's pose (world, m); velocities; world-aligned bounds."""

    def rotation(self) -> np.ndarray:
        return rotation(self.quat_xyzw)


def state_from_pose(spec: Body, pos, quat_xyzw, lin_vel=(0, 0, 0), ang_vel=(0, 0, 0)) -> BodyState:
    lo, hi = spec.local_bounds()
    corners = np.array([[a, b, c] for a in (lo[0], hi[0]) for b in (lo[1], hi[1]) for c in (lo[2], hi[2])])
    pts = np.asarray(pos, float) + corners @ rotation(quat_xyzw).T
    return BodyState(pos=np.asarray(pos, float), quat_xyzw=np.asarray(quat_xyzw, float),
                     lin_vel=np.asarray(lin_vel, float), ang_vel=np.asarray(ang_vel, float),
                     low=pts.min(0), high=pts.max(0))


# -- spawning (needs Isaac Sim; imported lazily) ------------------------------------------------------------------

_SPECS: dict[str, Body] = {}


def _spawn_body(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the body under ``prim_path``: a plain Xform holding one rigid-body Xform with the shapes under it."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec = _SPECS[cfg.body_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))

    phys = f"{prim_path}/Looks/physics"
    material = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction[0], dynamic_friction=spec.friction[1])
    material.func(phys, material)
    looks: dict = {}

    def look(color) -> str:
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            surface = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
            surface.func(path, surface)
            looks[color] = path
        return looks[color]

    body_path = f"{prim_path}/body"
    xform = UsdGeom.Xform.Define(stage, body_path)
    xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0.0, 0.0, 0.0))
    xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
    prim = xform.GetPrim()
    rigid = UsdPhysics.RigidBodyAPI.Apply(prim)
    if spec.kinematic:
        rigid.CreateKinematicEnabledAttr(True)
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr(float(spec.mass))
    if spec.com is not None:
        mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*[float(v) for v in spec.com]))
    parent = prim_path.rsplit("/", 1)[0]
    if spec.ignore or spec.ignore_robots:
        pairs = UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel()
        for other in spec.ignore:
            pairs.AddTarget(f"{parent}/{other}/body")
        if spec.ignore_robots:
            from yam_arena.robot import asset_link_paths

            for arm in ("LeftArm", "RightArm"):    # yam_arena's arm prims; every rigid link of each
                for link in asset_link_paths().values():
                    pairs.AddTarget(f"{parent}/{arm}{link}")
    if spec.weld_to:
        joint = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/weld")
        joint.CreateBody0Rel().SetTargets([f"{parent}/{spec.weld_to}/body"])
        joint.CreateBody1Rel().SetTargets([body_path])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*[float(v) for v in spec.weld_at]))
        joint.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateCollisionEnabledAttr(False)
        joint.CreateExcludeFromArticulationAttr(True)
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateMaxDepenetrationVelocityAttr(1.0)
    rb.CreateSolverPositionIterationCountAttr(int(spec.solver_iterations))
    rb.CreateLinearDampingAttr(float(spec.damping[0]))
    rb.CreateAngularDampingAttr(float(spec.damping[1]))
    if spec.ccd:
        rb.CreateEnableCCDAttr(True)

    for i, shape in enumerate(spec.shapes):
        if isinstance(shape, BoxShape):
            geom = UsdGeom.Cube.Define(stage, f"{body_path}/s{i}")
            geom.CreateSizeAttr(1.0)
            geom.AddTranslateOp().Set(Gf.Vec3d(*shape.centre))
            qx, qy, qz, qw = shape.quat()
            if (qx, qy, qz) != (0.0, 0.0, 0.0):
                geom.AddOrientOp().Set(Gf.Quatf(qw, qx, qy, qz))
            geom.AddScaleOp().Set(Gf.Vec3f(*shape.size))
        elif isinstance(shape, CylShape):
            geom = UsdGeom.Cylinder.Define(stage, f"{body_path}/s{i}")
            geom.CreateAxisAttr(shape.axis)
            geom.CreateRadiusAttr(shape.radius)
            geom.CreateHeightAttr(shape.height)
            geom.AddTranslateOp().Set(Gf.Vec3d(*shape.centre))
        elif isinstance(shape, SphereShape):
            geom = UsdGeom.Sphere.Define(stage, f"{body_path}/s{i}")
            geom.CreateRadiusAttr(shape.radius)
            geom.AddTranslateOp().Set(Gf.Vec3d(*shape.centre))
        else:
            geom = UsdGeom.Mesh.Define(stage, f"{body_path}/s{i}")
            pts = np.asarray(shape.points, float)
            tris = np.asarray(shape.faces, int)
            geom.CreatePointsAttr([Gf.Vec3f(*map(float, q)) for q in pts])
            geom.CreateFaceVertexCountsAttr([3] * len(tris))
            geom.CreateFaceVertexIndicesAttr([int(v) for v in tris.reshape(-1)])
            geom.CreateSubdivisionSchemeAttr("none")
        path = str(geom.GetPrim().GetPath())
        if shape.color is None:
            UsdGeom.Imageable(geom.GetPrim()).MakeInvisible()
        else:
            bind_visual_material(path, look(tuple(shape.color)), stage=stage)
        if shape.collide:
            api = UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
            api.CreateCollisionEnabledAttr(True)
            collision = PhysxSchema.PhysxCollisionAPI.Apply(geom.GetPrim())
            collision.CreateContactOffsetAttr(float(spec.contact_offset))
            collision.CreateRestOffsetAttr(0.0)
            bind_physics_material(path, phys, stage=stage)
            if isinstance(shape, HullShape):
                if shape.sdf:
                    UsdPhysics.MeshCollisionAPI.Apply(geom.GetPrim()).CreateApproximationAttr("sdf")
                    PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(geom.GetPrim()).CreateSdfResolutionAttr(256)
                else:
                    UsdPhysics.MeshCollisionAPI.Apply(geom.GetPrim()).CreateApproximationAttr("convexHull")
                    PhysxSchema.PhysxConvexHullCollisionAPI.Apply(geom.GetPrim()).CreateHullVertexLimitAttr(255)
    return root.GetPrim()


def _arena_object(spec: Body):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class BodySpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_body)
        body_name: str = ""

    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=BodySpawnerCfg(body_name=spec.name),
                  initial_pose=Pose(position_xyz=tuple(spec.pos),
                                    rotation_xyzw=tuple(spec.start_xyzw) if spec.start_xyzw is not None else
                                    (0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))))


def as_numpy(x) -> np.ndarray:
    """An Isaac Lab array (a torch tensor, or a proxy with ``.torch``) as float64 NumPy."""
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def _body_state(environment, spec: Body) -> BodyState:
    base = environment.unwrapped
    data = base.scene[spec.name].data
    origin = as_numpy(base.scene.env_origins[0])
    pose = as_numpy(data.root_link_pose_w)[0]
    vel = as_numpy(data.root_com_vel_w)[0] if hasattr(data, "root_com_vel_w") else as_numpy(data.root_vel_w)[0]
    return state_from_pose(spec, pose[:3] - origin, pose[3:7], vel[:3], vel[3:6])


# -- hooking yam_arena.scene and the embodiment ----------------------------------------------------------------------

def _patch_scene(scene) -> None:
    if getattr(scene, "_stackkit_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Body):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = orig_states(environment, plain) if plain else {}
        for n in names:
            if n in _SPECS:
                out[n] = _body_state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._stackkit_patched = True


_RESET: list = []
_STEP: list = []


def on_reset(fn):
    """Register ``fn()`` to run after every episode reset."""
    _RESET.append(fn)
    return fn


def on_step(fn):
    """Register ``fn(env)`` to run after every control step, before the task file is scored, with the Isaac Lab env."""
    _STEP.append(fn)
    return fn


def _patch_embodiment(module) -> None:
    cls = getattr(module, "YamArenaEmbodiment", None)
    if cls is None or getattr(cls, "_stackkit_patched", False):
        return
    base_reset, base_task_step = cls.reset, cls._task_step

    def reset(self, *args, **kwargs):
        obs = base_reset(self, *args, **kwargs)
        for fn in _RESET:
            fn()
        return obs

    def _task_step(self, *args, **kwargs):
        for fn in _STEP:
            fn(self._env)
        return base_task_step(self, *args, **kwargs)

    cls.reset, cls._task_step = reset, _task_step
    cls._stackkit_patched = True


_TARGETS = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}


class _Finder(importlib.abc.MetaPathFinder):
    """Patches each target module right after it is first imported."""

    def find_spec(self, fullname, path, target=None):
        if fullname not in _TARGETS:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        orig_exec = spec.loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _TARGETS[fullname](module)

        spec.loader.exec_module = exec_module
        return spec


def install() -> None:
    """Hook yam_arena.scene and the embodiment (now if already imported, otherwise when they are). Idempotent."""
    for name, patch in _TARGETS.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
