"""binderkit: rigid bodies built from several shapes, joined by PhysX joints, for yam_arena task files.

Adapted from the bodykit module used by other tasks in this repository (its Body, shapes and hooks), with:

- shapes turned by any rotation (``quat``, x y z w), and :class:`CapShape`, a capsule between two points (a length of
  round wire);
- :class:`Hinge`: a PhysX revolute joint between two bodies about an axis, with optional limits (degrees);
- :class:`Slide`: a PhysX D6 joint that locks some of its six axes and drives others with a spring toward a target
  (the spring of a ring binder's cover, pushing the outer edge of a plate).

Joints are authored once, when the scene is built, and are always on. Bodies joined by a joint do not collide with each
other. The kinematic chain may form a closed loop (a four-bar or toggle): these are maximal-coordinate PhysX joints, not
an articulation.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).parent))
    from binderkit import Body, BoxShape, CapShape, Hinge, Slide, joint, on_reset, on_step

Importing it hooks ``yam_arena.scene`` without changing yam_arena: a Body is spawned as one rigid body, and
``success()``/``score()`` receive a :class:`BodyState` for it whose ``pos`` and ``quat_xyzw`` are the body origin's
pose. It also hooks the embodiment's ``reset()`` and step, so a task file can clear what it tracks over an episode
(:func:`on_reset`) and act after every control step (:func:`on_step`). Pure Python at import time: everything that
needs Isaac Sim is imported lazily, so the task file still loads without the simulator.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["Body", "BodyState", "BoxShape", "CapShape", "Hinge", "Slide", "as_numpy", "install", "joint", "on_reset",
           "on_step", "quat_x", "rotation", "state_from_pose"]

Color = tuple[float, float, float]
IDENT = (0.0, 0.0, 0.0, 1.0)


def rotation(quat_xyzw) -> np.ndarray:
    x, y, z, w = quat_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def quat_x(angle: float) -> tuple[float, float, float, float]:
    """(x, y, z, w) of a turn of ``angle`` rad about x."""
    return (math.sin(angle / 2), 0.0, 0.0, math.cos(angle / 2))


def quat_z(angle: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))


@dataclass(frozen=True)
class BoxShape:
    """A box in the body frame: ``centre`` and ``size`` (m), turned by ``quat`` (x y z w). ``color=None``: collision
    only, not rendered. ``collide=False``: rendered only."""

    centre: tuple[float, float, float]
    size: tuple[float, float, float]
    color: Color | None = (0.6, 0.6, 0.6)
    quat: tuple[float, float, float, float] = IDENT
    collide: bool = True

    def half_extent(self) -> np.ndarray:
        return np.abs(rotation(self.quat)) @ (np.array(self.size) / 2)


@dataclass(frozen=True)
class CapShape:
    """A capsule (round wire) of ``radius`` between points ``p0`` and ``p1`` of the body frame."""

    p0: tuple[float, float, float]
    p1: tuple[float, float, float]
    radius: float
    color: Color | None = (0.6, 0.6, 0.6)
    collide: bool = True

    @property
    def centre(self) -> tuple[float, float, float]:
        return tuple(((np.array(self.p0) + np.array(self.p1)) / 2).tolist())

    def length(self) -> float:
        return float(np.linalg.norm(np.array(self.p1) - np.array(self.p0)))

    def quat(self) -> tuple[float, float, float, float]:
        """The turn taking +z onto p0 -> p1 (x y z w)."""
        d = (np.array(self.p1, float) - np.array(self.p0, float)) / self.length()
        axis = np.cross([0.0, 0.0, 1.0], d)
        s = float(np.linalg.norm(axis))
        if s < 1e-12:
            return IDENT if d[2] > 0 else (1.0, 0.0, 0.0, 0.0)
        a = math.atan2(s, float(d[2]))
        axis = axis / s * math.sin(a / 2)
        return (float(axis[0]), float(axis[1]), float(axis[2]), math.cos(a / 2))

    def half_extent(self) -> np.ndarray:
        d = np.abs(np.array(self.p1) - np.array(self.p0)) / 2
        return d + self.radius


@dataclass(frozen=True)
class Body(Box):
    """A rigid body made of ``shapes``. It subclasses Box only so yam_arena's task-file checks accept it: ``pos`` is
    the body origin (world, m) and ``size`` its bounding box. ``friction`` is (static, dynamic); ``damping`` is
    (linear, angular). A ``kinematic`` body stays where it is put. ``com`` is the centre of mass in the body frame.
    ``ignore`` names bodies (earlier in OBJECTS) it never collides with."""

    shapes: tuple = ()
    kinematic: bool = False
    friction: tuple[float, float] = (0.6, 0.5)
    friction_combine: str = "average"   # PhysX friction combine mode: average, min, multiply or max
    damping: tuple[float, float] = (0.0, 0.05)
    contact_offset: float = 0.002
    solver_iterations: int = 16
    com: tuple[float, float, float] | None = None
    ignore: tuple[str, ...] = ()
    ccd: bool = False
    report: bool = False
    start_xyzw: tuple[float, float, float, float] | None = None

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


# -- joints ----------------------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Hinge:
    """A revolute joint about the x axis of both bodies' frames, through ``anchor0`` (in ``body0``'s frame) and
    ``anchor1`` (in ``body1``'s frame), which must be the same world point when the bodies start. ``limits``
    (degrees, low/high, measured from the starting pose; None: free). ``body0`` must come before ``body1`` in
    OBJECTS."""

    name: str
    body0: str
    body1: str
    anchor0: tuple[float, float, float]
    anchor1: tuple[float, float, float]
    limits: tuple[float, float] | None = None


@dataclass(frozen=True)
class Slide:
    """A D6 joint between ``anchor0`` (``body0``'s frame) and ``anchor1`` (``body1``'s frame), its axes those of the
    bodies' frames. ``locked`` axes ("transX", "transY", "transZ", "rotX", "rotY", "rotZ") are held; every other axis
    is free, except that ``spring`` = (axis, stiffness N/m, damping N s/m, target m) drives one translation toward a
    target, and ``span`` (m) bounds that axis's travel either side of the start."""

    name: str
    body0: str
    body1: str
    anchor0: tuple[float, float, float]
    anchor1: tuple[float, float, float]
    locked: tuple[str, ...] = ()
    spring: tuple[str, float, float, float] | None = None
    span: float = 0.005


_JOINTS: list = []


def joint(j):
    """Register a :class:`Hinge` or :class:`Slide` and return it."""
    _JOINTS.append(j)
    return j


def _author_joints(stage, prim_path: str, spec) -> None:
    from pxr import Gf, UsdPhysics

    parent = prim_path.rsplit("/", 1)[0]
    for j in _JOINTS:
        if j.body1 != spec.name:
            continue
        path = f"{prim_path}/{j.name}"
        if isinstance(j, Hinge):
            jt = UsdPhysics.RevoluteJoint.Define(stage, path)
            jt.CreateAxisAttr("X")
            if j.limits is not None:
                jt.CreateLowerLimitAttr(float(j.limits[0]))
                jt.CreateUpperLimitAttr(float(j.limits[1]))
        else:
            jt = UsdPhysics.Joint.Define(stage, path)   # a plain Joint is a D6: limits lock or bound each axis
        jt.CreateBody0Rel().SetTargets([f"{parent}/{j.body0}/body"])
        jt.CreateBody1Rel().SetTargets([f"{prim_path}/body"])
        jt.CreateLocalPos0Attr(Gf.Vec3f(*[float(v) for v in j.anchor0]))
        jt.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        jt.CreateLocalPos1Attr(Gf.Vec3f(*[float(v) for v in j.anchor1]))
        jt.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        jt.CreateCollisionEnabledAttr(False)
        jt.CreateExcludeFromArticulationAttr(True)
        if isinstance(j, Slide):
            prim = jt.GetPrim()
            for dof in j.locked:
                limit = UsdPhysics.LimitAPI.Apply(prim, dof)
                limit.CreateLowAttr(1.0)
                limit.CreateHighAttr(-1.0)
            if j.spring is not None:
                dof, k, d, target = j.spring
                limit = UsdPhysics.LimitAPI.Apply(prim, dof)
                limit.CreateLowAttr(-float(j.span))
                limit.CreateHighAttr(float(j.span))
                drive = UsdPhysics.DriveAPI.Apply(prim, dof)
                drive.CreateTypeAttr("force")
                drive.CreateStiffnessAttr(float(k))
                drive.CreateDampingAttr(float(d))
                drive.CreateTargetPositionAttr(float(target))
                drive.CreateMaxForceAttr(1.0e4)


# -- spawning (needs Isaac Sim; imported lazily) ------------------------------------------------------------------

_SPECS: dict[str, Body] = {}
_BODY_PATHS: dict[str, str] = {}


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
    material = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction[0], dynamic_friction=spec.friction[1],
                                              friction_combine_mode=spec.friction_combine)
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
    _BODY_PATHS[spec.name] = body_path
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
    if spec.ignore:
        parent = prim_path.rsplit("/", 1)[0]
        pairs = UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel()
        for other in spec.ignore:
            pairs.AddTarget(f"{parent}/{other}/body")
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateMaxDepenetrationVelocityAttr(1.0)
    rb.CreateSolverPositionIterationCountAttr(int(spec.solver_iterations))
    rb.CreateSolverVelocityIterationCountAttr(4)
    rb.CreateLinearDampingAttr(float(spec.damping[0]))
    rb.CreateAngularDampingAttr(float(spec.damping[1]))
    if spec.ccd:
        rb.CreateEnableCCDAttr(True)
    if spec.report:
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)

    for i, shape in enumerate(spec.shapes):
        if isinstance(shape, BoxShape):
            geom = UsdGeom.Cube.Define(stage, f"{body_path}/s{i}")
            geom.CreateSizeAttr(1.0)
            geom.AddTranslateOp().Set(Gf.Vec3d(*shape.centre))
            qx, qy, qz, qw = shape.quat
            if (qx, qy, qz) != (0.0, 0.0, 0.0):
                geom.AddOrientOp().Set(Gf.Quatf(qw, qx, qy, qz))
            geom.AddScaleOp().Set(Gf.Vec3f(*shape.size))
        else:
            geom = UsdGeom.Capsule.Define(stage, f"{body_path}/s{i}")
            geom.CreateAxisAttr("Z")
            geom.CreateRadiusAttr(shape.radius)
            geom.CreateHeightAttr(shape.length())
            geom.AddTranslateOp().Set(Gf.Vec3d(*shape.centre))
            qx, qy, qz, qw = shape.quat()
            geom.AddOrientOp().Set(Gf.Quatf(qw, qx, qy, qz))
        path = str(geom.GetPrim().GetPath())
        if shape.color is None:
            UsdGeom.Imageable(geom.GetPrim()).MakeInvisible()
        else:
            bind_visual_material(path, look(tuple(shape.color)), stage=stage)
        if shape.collide:
            UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
            collision = PhysxSchema.PhysxCollisionAPI.Apply(geom.GetPrim())
            collision.CreateContactOffsetAttr(float(spec.contact_offset))
            collision.CreateRestOffsetAttr(0.0)
            bind_physics_material(path, phys, stage=stage)
    _author_joints(stage, prim_path, spec)
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


_VIEWS: dict = {}


def net_contact_force(name: str) -> np.ndarray:
    """The net contact force (N, world) on Body ``name`` (spawned with ``report=True``) in the last physics step."""
    if name not in _VIEWS:
        from isaaclab_physx.physics import PhysxManager

        _VIEWS[name] = PhysxManager.get_physics_sim_view().create_rigid_body_view(_BODY_PATHS[name])
        _VIEWS[name + "/c"] = PhysxManager.get_physics_sim_view().create_rigid_contact_view(
            _BODY_PATHS[name], filter_patterns=[], max_contact_data_count=0)
    m = _VIEWS[name + "/c"].get_net_contact_forces(dt=1 / 240)
    m = m.numpy() if hasattr(m, "numpy") else as_numpy(m)
    return np.asarray(m, float).reshape(-1, 3).sum(0)


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
    if getattr(scene, "_binderkit_patched", False):
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
    scene._binderkit_patched = True


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
    if cls is None or getattr(cls, "_binderkit_patched", False):
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
    cls._binderkit_patched = True


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
