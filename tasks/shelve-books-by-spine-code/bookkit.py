"""bookkit: parts built from several boxes and capsules, with printed marks, for yam_arena task files.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from bookkit import on_step, part

    BOOK = part("book_1", pos=(0.40, 0.2, 0.80), mass=0.24, boxes=[((0, 0, 0), (0.108, 0.175, 0.030), RED)],
                marks=[((-0.0541, 0.03, 0.0), (0.0002, 0.012, 0.004), BLACK), ...])    # printed on the spine
    SHELF = part("shelf", pos=(0.45, 0.07, 0.7475), boxes=[...])                        # no mass: fixed in place
    OBJECTS = [SHELF, BOOK]

yam_arena's task-file objects are single primitives. A part here is one body made of several shapes:

- boxes ``(centre, size, color)`` or ``(centre, size, color, quat_xyzw)`` for a box turned in the part's frame;
- capsules ``(centre, radius, height of the straight section, color[, axis])``;
- marks: boxes that are drawn but do not collide, for printing (letters on a label, a scale, a stripe).

A part is loose when it has a ``mass``, fixed in place when it has none, and with ``rail=(axis, lower, upper)`` it is
a loose body that can only slide along that axis of its own frame between the two stops (a prismatic joint to the
world), for drawers, plungers and lifters.

Importing this module hooks ``yam_arena.scene`` and the embodiment's step and reset without modifying yam_arena.
``success()`` and ``score()`` receive a :class:`PartState` for each part. ``on_step`` and ``on_reset`` register
callbacks ``fn(env)`` run after every control step (before the task file is scored) and after every reset.

The structure follows the kits used by tasks already in this repository (a Box subclass so the loader accepts the
part, a custom spawner, hooks installed through ``sys.meta_path``).

Pure Python at import time; Isaac Sim is reached only when the scene is built.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["Part", "PartState", "env", "finger_points", "install", "on_reset", "on_step", "part", "quat_about"]

COMBINE_MODES = ("average", "min", "multiply", "max")


def quat_about(axis: str, degrees: float) -> tuple:
    """Quaternion (x, y, z, w) of a turn of ``degrees`` about the X, Y or Z axis."""
    h = math.radians(degrees) / 2
    v = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}[axis]
    return (v[0] * math.sin(h), v[1] * math.sin(h), v[2] * math.sin(h), math.cos(h))


def _rotation(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


@dataclass(frozen=True)
class Part(Box):
    """One body made of ``boxes`` and ``capsules`` given in its own frame. ``pos`` is the frame's origin; ``lo`` and
    ``hi`` are the corners of its bounding box in that frame. Build one with :func:`part`."""

    boxes: tuple = ()
    capsules: tuple = ()
    marks: tuple = ()                    # boxes that are drawn but do not collide
    rail: tuple | None = None            # (axis, lower, upper) in metres: a slide along that axis between two stops
    rail_damping: float = 2.0            # N s/m along the rail
    lo: tuple = (0.0, 0.0, 0.0)
    hi: tuple = (0.0, 0.0, 0.0)
    static_friction: float = 0.5
    dynamic_friction: float = 0.4
    combine: str = "average"
    contact_offset: float = 0.002
    linear_damping: float = 0.05
    angular_damping: float = 0.5
    solver_iterations: int = 16
    gyroscopic: bool = True
    roughness: float = 0.6
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")
        if not (self.boxes or self.capsules):
            raise ValueError(f"{self.name}: a Part needs at least one shape")
        if self.mass is not None and self.mass <= 0:
            raise ValueError(f"{self.name}: mass must be positive (or None for a fixed part)")
        if self.static_friction < 0 or self.dynamic_friction < 0 or self.dynamic_friction > self.static_friction:
            raise ValueError(f"{self.name}: need 0 <= dynamic friction <= static friction")
        if self.combine not in COMBINE_MODES:
            raise ValueError(f"{self.name}.combine must be one of {COMBINE_MODES}, got {self.combine!r}")
        if self.rail is not None:
            axis, lower, upper = self.rail
            if self.mass is None or axis not in ("X", "Y", "Z") or not lower <= 0.0 <= upper:
                raise ValueError(f"{self.name}: a rail needs a mass, an axis X/Y/Z and lower <= 0 <= upper")


def _box_corners(spec) -> np.ndarray:
    centre, half = np.array(spec[0], float), np.array(spec[1], float) / 2
    corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * half
    if len(spec) > 3:
        corners = corners @ _rotation(spec[3]).T
    return corners + centre


def _capsule_corners(spec) -> np.ndarray:
    centre, r, h = np.array(spec[0], float), spec[1], spec[2]
    axis = spec[4] if len(spec) > 4 else "Z"
    half = np.array([r + (h / 2 if axis == a else 0.0) for a in ("X", "Y", "Z")])
    return np.array([centre - half, centre + half])


def part(name, pos, boxes=(), capsules=(), marks=(), mass=None, yaw=0.0, color=(0.6, 0.6, 0.6), **kw) -> Part:
    """Build a :class:`Part`; its bounds are filled in from its solid shapes."""
    pts = np.vstack([_box_corners(b) for b in boxes] + [_capsule_corners(c) for c in capsules])
    lo, hi = pts.min(0), pts.max(0)
    return Part(name=name, pos=tuple(pos), size=tuple((hi - lo).tolist()), mass=mass, color=color, yaw=yaw,
                boxes=tuple(boxes), capsules=tuple(capsules), marks=tuple(marks), lo=tuple(lo.tolist()),
                hi=tuple(hi.tolist()), **kw)


@dataclass(frozen=True)
class PartState(ObjectState):
    """A part as the simulator has it: ``pos`` and ``quat_xyzw`` are the pose of its frame."""

    def rotation(self) -> np.ndarray:
        return _rotation(self.quat_xyzw)

    def to_world(self, local_point) -> np.ndarray:
        return self.pos + self.rotation() @ np.asarray(local_point, float)

    def to_local(self, world_point) -> np.ndarray:
        return self.rotation().T @ (np.asarray(world_point, float) - self.pos)

    def tilt_deg(self) -> float:
        """Angle between the part's own z axis and the vertical."""
        return math.degrees(math.acos(max(-1.0, min(1.0, float(self.rotation()[2, 2])))))


def _yaw_xyzw(yaw: float) -> tuple:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


# -- spawning ---------------------------------------------------------------------------------------------------------

_PARTS: dict[str, Part] = {}


def _spawn_part(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the part under ``prim_path`` (pxr only): a plain Xform holding one body with its shapes."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec: Part = _PARTS[cfg.part_name]
    stage = get_current_stage()
    double = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))

    phys = f"{prim_path}/Looks/physics"
    try:
        pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.static_friction,
                                            dynamic_friction=spec.dynamic_friction,
                                            friction_combine_mode=spec.combine)
    except TypeError:  # a build without the combine-mode field: keep the coefficients
        pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.static_friction,
                                            dynamic_friction=spec.dynamic_friction)
    pm.func(phys, pm)
    looks: dict = {}

    def look(color) -> str:
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=spec.roughness)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    main = f"{prim_path}/body"
    xform = UsdGeom.Xform.Define(stage, main)
    xform.AddTranslateOp(precision=double).Set(Gf.Vec3d(0.0, 0.0, 0.0))
    xform.AddOrientOp(precision=double).Set(Gf.Quatd(1, 0, 0, 0))
    body = xform.GetPrim()
    if spec.free:
        UsdPhysics.RigidBodyAPI.Apply(body)
        UsdPhysics.MassAPI.Apply(body).CreateMassAttr(spec.mass)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(body)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(int(spec.solver_iterations))
        rb.CreateLinearDampingAttr(spec.linear_damping)
        rb.CreateAngularDampingAttr(spec.angular_damping)
        if not spec.gyroscopic:
            rb.CreateEnableGyroscopicForcesAttr(False)

    def collide(prim, color):
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(spec.contact_offset)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), phys, stage=stage)
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)

    for i, b in enumerate(spec.boxes):
        centre, size, color = b[:3]
        cube = UsdGeom.Cube.Define(stage, f"{main}/box_{i}")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
        if len(b) > 3:
            qx, qy, qz, qw = (float(v) for v in b[3])
            cube.AddOrientOp().Set(Gf.Quatf(qw, qx, qy, qz))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        collide(cube.GetPrim(), color)
    for i, m in enumerate(spec.marks):                     # drawn only: no collider
        centre, size, color = m[:3]
        cube = UsdGeom.Cube.Define(stage, f"{main}/mark_{i}")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
        if len(m) > 3:
            qx, qy, qz, qw = (float(v) for v in m[3])
            cube.AddOrientOp().Set(Gf.Quatf(qw, qx, qy, qz))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        bind_visual_material(str(cube.GetPrim().GetPath()), look(color), stage=stage)
    for i, c in enumerate(spec.capsules):
        centre, radius, height, color = c[:4]
        cap = UsdGeom.Capsule.Define(stage, f"{main}/capsule_{i}")
        cap.CreateAxisAttr(c[4] if len(c) > 4 else "Z")
        cap.CreateRadiusAttr(radius)
        cap.CreateHeightAttr(height)
        cap.AddTranslateOp().Set(Gf.Vec3d(*centre))
        collide(cap.GetPrim(), color)

    if spec.rail is not None:
        # A slide joint between the world and the body, placed at the body's own frame where it is spawned.
        axis, lower, upper = spec.rail
        world = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t, r = world.ExtractTranslation(), world.ExtractRotationQuat()
        joint = UsdPhysics.PrismaticJoint.Define(stage, f"{prim_path}/rail")
        joint.CreateAxisAttr(axis)
        joint.CreateBody1Rel().SetTargets([Sdf.Path(main)])
        joint.CreateLocalPos0Attr(Gf.Vec3f(float(t[0]), float(t[1]), float(t[2])))
        joint.CreateLocalRot0Attr(Gf.Quatf(float(r.GetReal()), *(float(v) for v in r.GetImaginary())))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        joint.CreateLowerLimitAttr(float(lower))
        joint.CreateUpperLimitAttr(float(upper))
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(float(spec.rail_damping))
        drive.CreateTargetVelocityAttr(0.0)
    return root.GetPrim()


def _part_object(spec: Part):
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

    return Object(name=spec.name, object_type=ObjectType.RIGID if spec.free else ObjectType.BASE,
                  spawner_cfg=PartSpawnerCfg(part_name=spec.name),
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=_yaw_xyzw(spec.yaw)))


def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _bounds(spec: Part, pos, quat) -> tuple:
    lo, hi = np.array(spec.lo), np.array(spec.hi)
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    pts = corners @ _rotation(quat).T + pos
    return pts.min(0), pts.max(0)


def _part_state(environment, spec: Part) -> PartState:
    if not spec.free:   # fixed in place: where the task file put it
        pos, quat = np.array(spec.pos, float), np.array(_yaw_xyzw(spec.yaw))
        low, high = _bounds(spec, pos, quat)
        return PartState(pos=pos, quat_xyzw=quat, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=low, high=high)
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.root_link_pose_w)[0]
    vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    pos, quat = pose[:3] - origin, pose[3:7]
    low, high = _bounds(spec, pos, quat)
    return PartState(pos=pos, quat_xyzw=quat, lin_vel=vel[:3], ang_vel=vel[3:6], low=low, high=high)


def finger_points(environment) -> list:
    """The grasp point of each gripper (world, relative to the env origin), for "has it let go" checks in an
    ``on_step`` hook; ``score()`` itself does not see the robot."""
    from yam_arena.constants import TCP_OFFSET

    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    pts = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = _np(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
        pts.append(pose[:3] + _rotation(pose[3:7]) @ np.array(TCP_OFFSET, float) - origin)
    return pts


# -- hooking yam_arena ------------------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []
_ENV: dict = {"env": None}


def on_step(fn):
    """Register ``fn(env)`` to run after every control step, before the task file is scored."""
    _STEP.append(fn)
    return fn


def on_reset(fn):
    """Register ``fn(env)`` to run after every reset of the embodiment."""
    _RESET.append(fn)
    return fn


def env():
    """The running environment, once the embodiment has been reset (else None)."""
    return _ENV["env"]


def _patch_scene(scene) -> None:
    if getattr(scene, "_bookkit_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Part):
            _PARTS[spec.name] = spec
            return _part_object(spec)
        return stock_object(spec)

    def object_states(environment, names):
        plain = [n for n in names if n not in _PARTS]
        out = stock_states(environment, plain) if plain else {}
        for n in names:
            if n in _PARTS:
                out[n] = _part_state(environment, _PARTS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._bookkit_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_bookkit_patched", False):
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
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._bookkit_patched = True


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
