"""metkit: rigid bodies built from several shapes, and D6 joints between them, for yam_arena task files. Adapted from
the bodykit module used by other tasks in this repository (its Body, shapes and hooks), with two additions for a
mechanism made of jointed parts:

- :class:`Joint`: a PhysX D6 joint between two Bodies. Each of its six axes (``transX`` .. ``rotZ``, in the joint frame)
  is free, locked or bounded by limits, and can carry a spring/damper drive toward a target. ``collide=True`` keeps
  contacts between the two bodies (PhysX otherwise ignores contacts between jointed bodies).
- ``Body.inertia``: the principal moments of inertia (kg m^2, about ``com``, along the body axes), set explicitly instead
  of computed from the collision shapes (a part made of materials of different density).
- ``Body.report=True`` turns PhysX contact reports on for a body, and :func:`contact_force` reads the force on it from
  another body in the last physics step.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).parent))
    from metkit import Body, BoxShape, CylShape, joint, on_reset, on_step

Importing it hooks ``yam_arena.scene`` without changing yam_arena: a Body is spawned as one rigid body, and
``success()``/``score()`` receive a :class:`BodyState` for it whose ``pos`` and ``quat_xyzw`` are the body origin's
pose. It also hooks the embodiment's ``reset()`` and step, so a task file can clear what it tracks over an episode
(:func:`on_reset`) and act on the simulation after every control step (:func:`on_step`). Pure Python at import time:
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

__all__ = ["Body", "BodyState", "BoxShape", "CylShape", "Joint", "as_numpy", "contact_force", "install", "joint",
           "on_reset", "on_step", "rotation", "set_shape_collision", "state_from_pose"]

Color = tuple[float, float, float]
PHYSICS_DT = 1 / 240


@dataclass(frozen=True)
class BoxShape:
    """A box in the body frame: ``centre`` and ``size`` (m), turned ``pitch`` (rad) about the body's y axis and then
    ``yaw`` (rad) about its z axis.
    ``color=None``: collision only, not rendered. ``collide=False``: rendered only. ``tag`` names the shape so the
    task file can switch its collisions (:func:`set_shape_collision`)."""

    centre: tuple[float, float, float]
    size: tuple[float, float, float]
    color: Color | None = (0.6, 0.6, 0.6)
    yaw: float = 0.0
    collide: bool = True
    tag: str = ""
    pitch: float = 0.0

    def quat(self) -> tuple[float, float, float, float]:
        cz, sz = math.cos(self.yaw / 2), math.sin(self.yaw / 2)
        cy, sy = math.cos(self.pitch / 2), math.sin(self.pitch / 2)
        return (-sz * sy, cz * sy, sz * cy, cz * cy)   # yaw * pitch, (x, y, z, w)

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
    tag: str = ""

    def half_extent(self) -> np.ndarray:
        half = np.full(3, self.radius)
        half["XYZ".index(self.axis)] = self.height / 2
        return half


@dataclass(frozen=True)
class Body(Box):
    """A rigid body made of ``shapes``. It subclasses Box only so yam_arena's task-file checks accept it: ``pos`` is
    the body origin (world, m) and ``size`` its bounding box. ``friction`` is (static, dynamic); ``damping`` is
    (linear, angular). A ``kinematic`` body stays where it is put. ``com`` is the centre of mass in the body frame.
    ``ignore`` names bodies (earlier in OBJECTS) it never collides with; ``report`` turns PhysX contact reports on (for
    :func:`contact_force`)."""

    shapes: tuple = ()
    kinematic: bool = False
    friction: tuple[float, float] = (0.6, 0.5)
    damping: tuple[float, float] = (0.0, 0.05)
    contact_offset: float = 0.002
    solver_iterations: int = 16
    com: tuple[float, float, float] | None = None
    ignore: tuple[str, ...] = ()
    report: bool = False
    ccd: bool = False
    start_xyzw: tuple[float, float, float, float] | None = None  # the starting orientation, if not just ``yaw``
    off_tags: tuple[str, ...] = ()  # tagged shapes whose collisions start (and are reset) switched off
    inertia: tuple[float, float, float] | None = None  # principal moments about ``com`` (kg m^2), body axes

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


# -- joints ----------------------------------------------------------------------------------------------------------

DOFS = ("transX", "transY", "transZ", "rotX", "rotY", "rotZ")


@dataclass(frozen=True)
class Joint:
    """A PhysX D6 joint from ``body0`` to ``body1`` (``body0`` comes first in OBJECTS). Its frame sits at ``anchor0`` in
    ``body0``'s frame and at ``anchor1`` in ``body1``'s, both aligned with the body axes. ``free`` names the axes that
    move without limit; ``limits`` bounds axes ({dof: (low, high)}, m or degrees); every other axis is locked.
    ``drives`` adds a spring/damper drive to an axis ({dof: (stiffness, damping, target)}: N/m and N s/m, or N m per
    degree and N m s per degree, target in m or degrees). ``collide`` keeps contacts between the two bodies."""

    name: str
    body0: str
    body1: str
    anchor0: tuple[float, float, float]
    anchor1: tuple[float, float, float]
    free: tuple[str, ...] = ()
    limits: dict = None
    drives: dict = None
    collide: bool = False


_JOINTS: list[Joint] = []


def joint(*args, **kwargs) -> Joint:
    """Register a :class:`Joint` (same arguments) and return it."""
    j = Joint(*args, **kwargs)
    for dof in tuple(j.free) + tuple((j.limits or {}).keys()) + tuple((j.drives or {}).keys()):
        if dof not in DOFS:
            raise ValueError(f"{j.name}: unknown axis {dof!r}")
    _JOINTS.append(j)
    return j


def _author_joints(stage, prim_path: str, spec) -> None:
    from pxr import Gf, UsdPhysics

    parent = prim_path.rsplit("/", 1)[0]
    for j in _JOINTS:
        if j.body1 != spec.name:
            continue
        path = f"{prim_path}/{j.name}"
        d6 = UsdPhysics.Joint.Define(stage, path)  # a plain Joint is a D6: limits lock or bound each axis
        d6.CreateBody0Rel().SetTargets([f"{parent}/{j.body0}/body"])
        d6.CreateBody1Rel().SetTargets([f"{prim_path}/body"])
        d6.CreateLocalPos0Attr(Gf.Vec3f(*[float(v) for v in j.anchor0]))
        d6.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        d6.CreateLocalPos1Attr(Gf.Vec3f(*[float(v) for v in j.anchor1]))
        d6.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        d6.CreateJointEnabledAttr(True)
        d6.CreateCollisionEnabledAttr(bool(j.collide))
        d6.CreateExcludeFromArticulationAttr(True)
        prim = d6.GetPrim()
        limits = dict(j.limits or {})
        for dof in DOFS:
            if dof in j.free:
                continue
            lim = UsdPhysics.LimitAPI.Apply(prim, dof)
            low, high = limits.get(dof, (1.0, -1.0))  # low > high: locked
            lim.CreateLowAttr(float(low))
            lim.CreateHighAttr(float(high))
        for dof, (k, c, target) in (j.drives or {}).items():
            drive = UsdPhysics.DriveAPI.Apply(prim, dof)
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(float(k))
            drive.CreateDampingAttr(float(c))
            drive.CreateTargetPositionAttr(float(target))
            drive.CreateMaxForceAttr(1.0e3)

# -- shapes whose collisions the task file switches ---------------------------------------------------------------

_SHAPE_PATHS: dict[tuple[str, str], str] = {}
_SHAPE_ATTRS: dict[tuple[str, str], object] = {}


def set_shape_collision(body: str, tag: str, on: bool) -> None:
    """Switch the collisions of ``body``'s shape tagged ``tag`` off or on."""
    key = (body, tag)
    if key not in _SHAPE_ATTRS:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        api = UsdPhysics.CollisionAPI(get_current_stage().GetPrimAtPath(_SHAPE_PATHS[key]))
        _SHAPE_ATTRS[key] = api.GetCollisionEnabledAttr() or api.CreateCollisionEnabledAttr(True)
    _SHAPE_ATTRS[key].Set(bool(on))


def _reset_joints_and_shapes() -> None:
    """Every tagged shape's collisions back to how they start."""
    for body, tag in _SHAPE_PATHS:
        set_shape_collision(body, tag, tag not in _SPECS[body].off_tags)


# -- contact forces ---------------------------------------------------------------------------------------------

_BODY_PATHS: dict[str, str] = {}
_VIEWS: dict = {}


def contact_force(a: str, b: str) -> np.ndarray:
    """The contact force (N, world frame) on Body ``a`` (spawned with ``report=True``) from Body ``b`` in the last
    physics step (env 0), from PhysX's contact-report force matrix."""
    if (a, b) not in _VIEWS:
        from isaaclab_physx.physics import PhysxManager

        _VIEWS[(a, b)] = PhysxManager.get_physics_sim_view().create_rigid_contact_view(
            _BODY_PATHS[a], filter_patterns=[_BODY_PATHS[b]], max_contact_data_count=64)
    m = _VIEWS[(a, b)].get_contact_force_matrix(dt=PHYSICS_DT)
    m = m.numpy() if hasattr(m, "numpy") else as_numpy(m)
    return np.asarray(m, float).reshape(-1, 3).sum(0)


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
    if spec.inertia is not None:
        mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*[float(v) for v in spec.inertia]))
        mass_api.CreatePrincipalAxesAttr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    if spec.ignore:
        parent = prim_path.rsplit("/", 1)[0]
        pairs = UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel()
        for other in spec.ignore:
            pairs.AddTarget(f"{parent}/{other}/body")
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateMaxDepenetrationVelocityAttr(1.0)
    rb.CreateSolverPositionIterationCountAttr(int(spec.solver_iterations))
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
            qx, qy, qz, qw = shape.quat()
            if (qx, qy, qz) != (0.0, 0.0, 0.0):
                geom.AddOrientOp().Set(Gf.Quatf(qw, qx, qy, qz))
            geom.AddScaleOp().Set(Gf.Vec3f(*shape.size))
        else:
            geom = UsdGeom.Cylinder.Define(stage, f"{body_path}/s{i}")
            geom.CreateAxisAttr(shape.axis)
            geom.CreateRadiusAttr(shape.radius)
            geom.CreateHeightAttr(shape.height)
            geom.AddTranslateOp().Set(Gf.Vec3d(*shape.centre))
        path = str(geom.GetPrim().GetPath())
        if shape.color is None:
            UsdGeom.Imageable(geom.GetPrim()).MakeInvisible()
        else:
            bind_visual_material(path, look(tuple(shape.color)), stage=stage)
        if shape.collide:
            api = UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
            api.CreateCollisionEnabledAttr(shape.tag not in spec.off_tags)
            collision = PhysxSchema.PhysxCollisionAPI.Apply(geom.GetPrim())
            collision.CreateContactOffsetAttr(float(spec.contact_offset))
            collision.CreateRestOffsetAttr(0.0)
            bind_physics_material(path, phys, stage=stage)
            if shape.tag:
                _SHAPE_PATHS[(spec.name, shape.tag)] = path
                _SHAPE_ATTRS.pop((spec.name, shape.tag), None)
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
    if getattr(scene, "_metkit_patched", False):
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
    scene._metkit_patched = True


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
    if cls is None or getattr(cls, "_metkit_patched", False):
        return
    base_reset, base_task_step = cls.reset, cls._task_step

    def reset(self, *args, **kwargs):
        if _SHAPE_PATHS:
            _reset_joints_and_shapes()
        obs = base_reset(self, *args, **kwargs)
        for fn in _RESET:
            fn()
        return obs

    def _task_step(self, *args, **kwargs):
        for fn in _STEP:
            fn(self._env)
        return base_task_step(self, *args, **kwargs)

    cls.reset, cls._task_step = reset, _task_step
    cls._metkit_patched = True


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
