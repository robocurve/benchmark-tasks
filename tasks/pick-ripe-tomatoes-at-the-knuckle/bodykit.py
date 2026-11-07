"""bodykit: loose rigid bodies built from several boxes and cylinders, for yam_arena task files.

yam_arena's task-file objects are single primitives. A :class:`Body` is one rigid body made of several shapes (a pan
with a floor, walls and a handle, say), some of which may be visual only (markings that nothing collides with), with
its own friction. A body can also be fixed in place (``fixed=True``: kinematic, like a pipe clamped to the wall) and
start turned by a quaternion (``init_quat``), and be held to another body or the world by a joint (``joint=Joint(...)``:
a hinge, a ball joint or a fixed joint, which can snap like a stem when bent too far). Keep this file next to the task
file and import it from there:

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).parent))
    from bodykit import Body, BoxShape, CylShape

    PAN = Body.build("pan", pos=(0.27, -0.08, 0.75), mass=0.45, shapes=[CylShape((0, 0, 0.0025), 0.106, 0.005), ...])

Importing it hooks ``yam_arena.scene`` without changing yam_arena: such a body is spawned as one rigid body, and
``success()``/``score()`` receive a :class:`BodyState` for it whose ``pos`` and ``quat_xyzw`` are the body origin's
pose (yam_arena reports the bounding-box centre for primitives). It also hooks the embodiment's ``reset()`` and step,
so a task file can clear what it tracks over an episode (:func:`on_reset`) and look at the simulation after every
control step (:func:`on_step`), for instance at where the grippers are. Pure Python at import time: everything that needs
Isaac Sim is imported lazily, so the task file still loads without the simulator.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["Body", "BodyState", "BoxShape", "CylShape", "Joint", "SphereShape", "allow_matrix_grips", "as_numpy",
           "install",
           "on_reset", "on_step", "plate", "rotation", "segment"]

Color = tuple[float, float, float]


@dataclass(frozen=True)
class BoxShape:
    """A box in the body frame: ``centre`` and ``size`` (m), turned ``yaw`` (rad) about the body's z axis, or, when
    ``quat_xyzw`` is given, rotated by that quaternion instead."""

    centre: tuple[float, float, float]
    size: tuple[float, float, float]
    color: Color = (0.6, 0.6, 0.6)
    yaw: float = 0.0
    collide: bool = True
    quat_xyzw: tuple[float, float, float, float] | None = None

    def quat(self) -> tuple[float, float, float, float]:
        if self.quat_xyzw is not None:
            return tuple(self.quat_xyzw)
        return (0.0, 0.0, math.sin(self.yaw / 2), math.cos(self.yaw / 2))

    def half_extent(self) -> np.ndarray:
        return np.abs(rotation(self.quat())) @ (np.array(self.size) / 2)


def _quat_from_matrix(m: np.ndarray) -> tuple[float, float, float, float]:
    """(x, y, z, w) of a rotation matrix."""
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return (x / n, y / n, z / n, w / n)


def plate(p1, p2, width: float, thickness: float, normal, color: Color, collide: bool = True) -> BoxShape:
    """A flat plate whose middle line runs from ``p1`` to ``p2`` (body frame, m), ``width`` wide, ``thickness`` thick
    along ``normal`` (made perpendicular to the line): a wall panel of a cylinder or a cone, say."""
    a, b = np.asarray(p1, float), np.asarray(p2, float)
    d = b - a
    length = float(np.linalg.norm(d))
    d /= length
    n = np.asarray(normal, float) - np.dot(normal, d) * d
    n /= np.linalg.norm(n)
    t = np.cross(d, n)
    return BoxShape(tuple(((a + b) / 2).tolist()), (thickness, width, length), color, collide=collide,
                    quat_xyzw=_quat_from_matrix(np.column_stack([n, t, d])))


def segment(p1, p2, thickness: float, color: Color, collide: bool = True) -> BoxShape:
    """A thin square bar from ``p1`` to ``p2`` (body frame, m): a box whose long (z) axis runs along the segment."""
    a, b = np.asarray(p1, float), np.asarray(p2, float)
    d = b - a
    length = float(np.linalg.norm(d))
    d /= length
    axis = np.cross([0.0, 0.0, 1.0], d)
    s, c = float(np.linalg.norm(axis)), float(d[2])
    if s < 1e-9:
        q = (0.0, 0.0, 0.0, 1.0) if c > 0 else (1.0, 0.0, 0.0, 0.0)
    else:
        angle = math.atan2(s, c)
        axis /= s
        q = (*(axis * math.sin(angle / 2)).tolist(), math.cos(angle / 2))
    return BoxShape(tuple(((a + b) / 2).tolist()), (thickness, thickness, length), color, collide=collide, quat_xyzw=q)


@dataclass(frozen=True)
class CylShape:
    """A cylinder in the body frame, its axis along the body's ``axis`` ("X", "Y" or "Z")."""

    centre: tuple[float, float, float]
    radius: float
    height: float
    color: Color = (0.6, 0.6, 0.6)
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
    color: Color = (0.6, 0.6, 0.6)
    collide: bool = True

    def half_extent(self) -> np.ndarray:
        return np.full(3, self.radius)


@dataclass(frozen=True)
class Joint:
    """A joint that holds a body to ``parent`` (another Body's name, listed before it; None: the world) at ``anchor``
    (world, m). Angles are measured from the bodies' starting poses, in degrees.

    - ``kind="revolute"``: turns about the world axis ``axis`` ("X", "Y" or "Z") through the anchor, within ``limits``
      (lower, upper); ``damping`` (N m s/rad) resists turning and ``stiffness`` (N m/rad) springs it toward ``target``.
    - ``kind="ball"``: turns about all three axes but cannot move; ``limits`` = (a,) caps each turn at +-a degrees
      (about the world x, y and z axes through the anchor), like a stem that only bends so far.
    - ``kind="fixed"``: rigid.

    ``break_force`` (N) and ``break_torque`` (N m), when given, break the joint for good once the force or torque it
    carries exceeds them (a ball joint carries torque only once it is bent to its limit). The joint does not collide
    its two bodies with each other."""

    anchor: tuple[float, float, float]
    kind: str = "revolute"
    axis: str = "Y"
    parent: str | None = None
    limits: tuple | None = None
    damping: float = 0.0
    stiffness: float = 0.0
    target: float = 0.0
    break_force: float | None = None
    break_torque: float | None = None
    child_first: bool = False  # author this body as the joint's body0: PhysX measures the torque that breaks a joint
    # about body0's centre of mass, so for a fruit on a fixed plant the fruit must be body0
    axial: tuple | None = None  # ball joints: (stiffness N/m, damping N s/m, travel m) frees sliding along body0's z
    # axis (a stem that stretches when pulled) on a spring, within +-travel


@dataclass(frozen=True)
class Body(Box):
    """A loose rigid body made of ``shapes``. It subclasses Box only so yam_arena's task-file checks accept it:
    ``pos`` is the body origin (world, m) and ``size`` its bounding box. ``friction`` is (static, dynamic);
    ``damping`` is (linear, angular)."""

    shapes: tuple = ()
    fixed: bool = False  # kinematic: held where it is placed (a pipe fixed to the wall), still a body that others hit
    init_quat: tuple | None = None  # initial orientation (x, y, z, w); None: ``yaw`` about the vertical
    joint: Joint | None = None  # a joint to another body or the world (a lid on its hinge, a fruit on its stem)
    friction: tuple[float, float] = (0.6, 0.5)
    damping: tuple[float, float] = (0.0, 0.05)
    contact_offset: float = 0.002
    solver_iterations: int = 16

    @staticmethod
    def build(name: str, pos, mass: float, shapes, yaw: float = 0.0, color: Color = (0.6, 0.6, 0.6), **kw) -> "Body":
        """A Body whose ``size`` is the extent of its shapes."""
        lo = np.min([np.array(s.centre) - s.half_extent() for s in shapes], axis=0)
        hi = np.max([np.array(s.centre) + s.half_extent() for s in shapes], axis=0)
        return Body(name=name, pos=tuple(float(v) for v in pos), size=tuple((hi - lo).tolist()), mass=mass, yaw=yaw,
                    color=color, shapes=tuple(shapes), **kw)

    def start_quat(self) -> tuple[float, float, float, float]:
        if self.init_quat is not None:
            return tuple(float(v) for v in self.init_quat)
        return (0.0, 0.0, math.sin(self.yaw / 2), math.cos(self.yaw / 2))

    def local_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        lo = np.min([np.array(s.centre) - s.half_extent() for s in self.shapes], axis=0)
        hi = np.max([np.array(s.centre) + s.half_extent() for s in self.shapes], axis=0)
        return lo, hi

    def _check(self) -> None:
        super()._check()
        if not self.shapes:
            raise ValueError(f"{self.name}: a Body needs at least one shape")
        if self.mass is None:
            raise ValueError(f"{self.name}: a Body must be loose (give it a mass)")


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

    def to_local(self, world_point) -> np.ndarray:
        return self.rotation().T @ (np.asarray(world_point, float) - self.pos)

    def tilt(self) -> float:
        """Angle (rad) between the body's z axis and the vertical."""
        return float(math.acos(max(-1.0, min(1.0, self.rotation()[2, 2]))))


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
    if spec.fixed:
        rigid.CreateKinematicEnabledAttr(True)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(spec.mass))
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateMaxDepenetrationVelocityAttr(1.0)
    rb.CreateSolverPositionIterationCountAttr(int(spec.solver_iterations))
    rb.CreateLinearDampingAttr(float(spec.damping[0]))
    rb.CreateAngularDampingAttr(float(spec.damping[1]))

    for i, shape in enumerate(spec.shapes):
        if isinstance(shape, BoxShape):
            geom = UsdGeom.Cube.Define(stage, f"{body_path}/s{i}")
            geom.CreateSizeAttr(1.0)
            geom.AddTranslateOp().Set(Gf.Vec3d(*shape.centre))
            qx, qy, qz, qw = shape.quat()
            if (qx, qy, qz) != (0.0, 0.0, 0.0):
                geom.AddOrientOp().Set(Gf.Quatf(qw, qx, qy, qz))
            geom.AddScaleOp().Set(Gf.Vec3f(*shape.size))
        elif isinstance(shape, SphereShape):
            geom = UsdGeom.Sphere.Define(stage, f"{body_path}/s{i}")
            geom.CreateRadiusAttr(shape.radius)
            geom.AddTranslateOp().Set(Gf.Vec3d(*shape.centre))
        else:
            geom = UsdGeom.Cylinder.Define(stage, f"{body_path}/s{i}")
            geom.CreateAxisAttr(shape.axis)
            geom.CreateRadiusAttr(shape.radius)
            geom.CreateHeightAttr(shape.height)
            geom.AddTranslateOp().Set(Gf.Vec3d(*shape.centre))
        path = str(geom.GetPrim().GetPath())
        bind_visual_material(path, look(tuple(shape.color)), stage=stage)
        if shape.collide:
            UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
            collision = PhysxSchema.PhysxCollisionAPI.Apply(geom.GetPrim())
            collision.CreateContactOffsetAttr(float(spec.contact_offset))
            collision.CreateRestOffsetAttr(0.0)
            bind_physics_material(path, phys, stage=stage)
    if spec.joint is not None:
        _author_joint(stage, prim_path, body_path, spec)
    return root.GetPrim()


def _author_joint(stage, prim_path: str, body_path: str, spec: Body) -> None:
    """A UsdPhysics joint at ``<prim_path>/joint``. Both joint frames are world-aligned at the start (each is the
    body's start rotation undone), so the joint's axes are the world axes and its zero angles the start pose."""
    from pxr import Gf, UsdPhysics

    h = spec.joint
    anchor = np.asarray(h.anchor, float)

    def frame(other: Body | None):
        if other is None:
            return anchor, (0.0, 0.0, 0.0, 1.0)
        x, y, z, w = other.start_quat()
        r = rotation((x, y, z, w))
        return r.T @ (anchor - np.asarray(other.pos, float)), (-x, -y, -z, w)

    path = f"{prim_path}/joint"
    if h.kind == "revolute":
        joint = UsdPhysics.RevoluteJoint.Define(stage, path)
        joint.CreateAxisAttr(h.axis)
    elif h.kind == "fixed":
        joint = UsdPhysics.FixedJoint.Define(stage, path)
    elif h.kind == "ball":
        joint = UsdPhysics.Joint.Define(stage, path)  # a D6 joint: translations locked, rotations limited or free
        for dof in ("transX", "transY", "transZ"):
            lim = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), dof)
            if dof == "transZ" and h.axial is not None:
                lim.CreateLowAttr(-float(h.axial[2]))
                lim.CreateHighAttr(float(h.axial[2]))
                drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "transZ")
                drive.CreateTypeAttr("force")
                drive.CreateStiffnessAttr(float(h.axial[0]))
                drive.CreateDampingAttr(float(h.axial[1]))
                drive.CreateTargetPositionAttr(0.0)
                drive.CreateMaxForceAttr(1.0e6)
                continue
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)  # low > high: locked
        if h.limits is not None:
            for dof in ("rotX", "rotY", "rotZ"):
                lim = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), dof)
                lim.CreateLowAttr(-float(h.limits[0]))
                lim.CreateHighAttr(float(h.limits[0]))
    else:
        raise ValueError(f"{spec.name}: unknown joint kind {h.kind!r}")
    if h.parent is not None and h.parent not in _SPECS:
        raise ValueError(f"{spec.name}: joint parent {h.parent!r} must be a Body listed before it")
    parent_path = f"{prim_path.rsplit('/', 1)[0]}/{h.parent}/body" if h.parent is not None else None
    p0, q0 = frame(_SPECS[h.parent] if h.parent is not None else None)
    p1, q1 = frame(spec)
    if h.child_first:  # this body is body0, the parent body1
        if h.kind == "revolute":
            raise ValueError(f"{spec.name}: child_first is for ball and fixed joints")
        joint.CreateBody0Rel().SetTargets([body_path])
        if parent_path is not None:
            joint.CreateBody1Rel().SetTargets([parent_path])
        p0, q0, p1, q1 = p1, q1, p0, q0
    else:
        if parent_path is not None:
            joint.CreateBody0Rel().SetTargets([parent_path])
        joint.CreateBody1Rel().SetTargets([body_path])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*[float(v) for v in p0]))
    joint.CreateLocalRot0Attr(Gf.Quatf(q0[3], q0[0], q0[1], q0[2]))
    joint.CreateLocalPos1Attr(Gf.Vec3f(*[float(v) for v in p1]))
    joint.CreateLocalRot1Attr(Gf.Quatf(q1[3], q1[0], q1[1], q1[2]))
    if h.break_force is not None:
        joint.CreateBreakForceAttr(float(h.break_force))
    if h.break_torque is not None:
        joint.CreateBreakTorqueAttr(float(h.break_torque))
    if h.kind == "revolute":
        if h.limits is not None:
            joint.CreateLowerLimitAttr(float(h.limits[0]))
            joint.CreateUpperLimitAttr(float(h.limits[1]))
        if h.damping or h.stiffness:
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
            drive.CreateTypeAttr("force")
            per_deg = math.pi / 180.0  # USD angular drives take N m per degree (and N m s per degree)
            drive.CreateStiffnessAttr(float(h.stiffness) * per_deg)
            drive.CreateDampingAttr(float(h.damping) * per_deg)
            drive.CreateTargetPositionAttr(float(h.target))
            drive.CreateMaxForceAttr(1.0e6)


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

    return Object(
        name=spec.name,
        object_type=ObjectType.RIGID,
        spawner_cfg=BodySpawnerCfg(body_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=spec.start_quat()),
    )


def as_numpy(x) -> np.ndarray:
    """An Isaac Lab array (a torch tensor, or a proxy with ``.torch``) as float64 NumPy."""
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


_np = as_numpy


def _body_state(environment, spec: Body) -> BodyState:
    base = environment.unwrapped
    data = base.scene[spec.name].data
    origin = _np(base.scene.env_origins[0])
    pose = _np(data.root_link_pose_w)[0]
    vel = _np(data.root_com_vel_w)[0] if hasattr(data, "root_com_vel_w") else _np(data.root_vel_w)[0]
    return state_from_pose(spec, pose[:3] - origin, pose[3:7], vel[:3], vel[3:6])


# -- hooking yam_arena.scene -------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_bodykit_patched", False):
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
    scene._bodykit_patched = True


_RESET: list = []
_STEP: list = []


def on_reset(fn):
    """Register ``fn()`` to run after every episode reset (for state a task file keeps over an episode)."""
    _RESET.append(fn)
    return fn


def on_step(fn):
    """Register ``fn(env)`` to run after every control step, before the task file is scored, with the Isaac Lab env."""
    _STEP.append(fn)
    return fn


def _patch_embodiment(module) -> None:
    cls = getattr(module, "YamArenaEmbodiment", None)
    if cls is None or getattr(cls, "_bodykit_patched", False):
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
    cls._bodykit_patched = True


_TARGETS = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}


class _Finder(importlib.abc.MetaPathFinder):
    """Patches each target module right after it is first imported (yam_arena.scene needs Isaac Sim, so it loads after
    this file)."""

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


def allow_matrix_grips() -> None:
    """Let a demo Waypoint's ``yaw`` be a full 3x3 TCP rotation (world frame) instead of an angle, for grips other than
    straight down (yam_arena.demo builds the rotation with grip_rotation(yaw), which this wraps). Idempotent."""
    from yam_arena import demo

    if getattr(demo, "_bodykit_matrix_grips", False):
        return
    plain = demo.grip_rotation

    def grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else plain(yaw)

    demo.grip_rotation = grip_rotation
    demo._bodykit_matrix_grips = True


def install() -> None:
    """Hook yam_arena.scene and the embodiment (now if already imported, otherwise when they are). Idempotent."""
    for name, patch in _TARGETS.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
