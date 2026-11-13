"""eggkit: loose bodies built from several shapes, with hinged flaps, for yam_arena task files.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from eggkit import Flap, Slick, body, on_reset, on_step

    EGG = body("egg_1", pos=(0.25, 0.0, 0.78), mass=0.055, capsules=[((0, 0, 0), 0.021, 0.014, WHITE)])
    LID = Flap("lid", pivot=(0.053, 0.0, 0.042), axis="Y", upper=180.0, mass=0.012, hold=9.3, boxes=[...])
    CARTON = body("carton", pos=(0.25, 0.0, 0.7475), mass=0.045, boxes=[...], flaps=[LID])
    OBJECTS = [EGG, CARTON, Slick("mat", pos=..., size=..., static_friction=0.9, dynamic_friction=0.8)]

yam_arena's task-file objects are single primitives with one rubber-like material. This module adds:

- ``Slick``: a task-file ``Box`` with its own friction, spawned through yam_arena's own cuboid spawner.
- ``body(...)``: one loose rigid body made of boxes and capsules (an egg is one capsule; a carton base is a floor,
  walls and dividers). Give it ``flaps`` and it becomes an articulation whose root is the body and whose flaps are
  shapes on revolute joints (a carton's lid).

Importing this module hooks ``yam_arena.scene`` and the embodiment's step and reset without modifying yam_arena.
``success()`` and ``score()`` receive a :class:`BodyState` for each kit body: the pose of the body's origin, its
velocities, its bounds, and each flap's angle. ``on_step`` and ``on_reset`` register callbacks ``fn(env)`` that run
after every control step (before the task file is scored) and after every reset. ``set_flap_hold`` changes a flap's
hinge spring at run time, which is how a task file lets a snap closure go once it has been pulled open.

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

__all__ = ["Body", "BodyState", "Flap", "Slick", "body", "env", "finger_points", "install", "on_reset", "on_step",
           "set_flap_hold"]

COMBINE_MODES = ("average", "min", "multiply", "max")
Color = tuple


# -- specs ------------------------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Slick(Box):
    """A ``Box`` with its own friction. PhysX combines the two bodies' coefficients by ``combine``."""

    static_friction: float = 0.5
    dynamic_friction: float = 0.4
    combine: str = "average"
    roughness: float = 0.6             # look only

    def _check(self) -> None:
        super()._check()
        _check_friction(self)


@dataclass(frozen=True)
class Flap:
    """Shapes on a revolute joint about the body's ``axis`` ("X", "Y" or "Z") through ``pivot`` (body frame, m).
    ``boxes`` are given in the body frame with the flap at angle 0; ``lower``/``upper`` are the joint's limits in
    degrees. ``hold`` (N m/rad) is a spring pulling the flap to angle 0 and ``damping`` (N m s/rad) acts at the
    hinge."""

    name: str
    pivot: tuple
    boxes: tuple = ()
    axis: str = "X"
    lower: float = 0.0
    upper: float = 180.0
    mass: float = 0.02
    hold: float = 0.0
    damping: float = 0.0005


@dataclass(frozen=True)
class Body(Box):
    """A loose rigid body made of ``boxes`` ((centre, size, color)) and ``capsules`` ((centre, radius, height of the
    cylindrical part, color[, axis])), all in the body's own frame. ``pos`` is the body's origin; ``size`` its
    bounding box, whose corners in the body frame are ``lo`` and ``hi``. Build one with :func:`body`."""

    boxes: tuple = ()
    capsules: tuple = ()
    flaps: tuple = ()
    lo: tuple = (0.0, 0.0, 0.0)
    hi: tuple = (0.0, 0.0, 0.0)
    static_friction: float = 0.5
    dynamic_friction: float = 0.4
    combine: str = "average"
    contact_offset: float = 0.002
    linear_damping: float = 0.05
    angular_damping: float = 0.5
    solver_iterations: int = 32          # PhysX position iterations for this body
    gyroscopic: bool = True              # PhysX gyroscopic forces
    sleep_threshold: float | None = None  # J/kg below which PhysX lets the body fall asleep (None: PhysX default)
    roughness: float = 0.6
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")
        if not (self.boxes or self.capsules):
            raise ValueError(f"{self.name}: a Body needs at least one shape")
        if self.mass is None or self.mass <= 0:
            raise ValueError(f"{self.name}: a Body must be loose (give it a mass)")
        _check_friction(self)
        names = [f.name for f in self.flaps]
        if len(set(names)) != len(names) or any(not n.isidentifier() for n in names):
            raise ValueError(f"{self.name}: flap names must be unique identifiers")
        for f in self.flaps:
            if f.axis not in ("X", "Y", "Z") or not f.lower <= 0.0 <= f.upper or f.mass <= 0:
                raise ValueError(f"{self.name}.{f.name}: bad hinge (axis X/Y/Z, lower <= 0 <= upper, mass > 0)")


def _check_friction(spec) -> None:
    if spec.static_friction < 0 or spec.dynamic_friction < 0 or spec.dynamic_friction > spec.static_friction:
        raise ValueError(f"{spec.name}: need 0 <= dynamic friction <= static friction")
    if spec.combine not in COMBINE_MODES:
        raise ValueError(f"{spec.name}.combine must be one of {COMBINE_MODES}, got {spec.combine!r}")


def _capsule_half(spec) -> np.ndarray:
    r, h, axis = spec[1], spec[2], spec[4] if len(spec) > 4 else "Z"
    return np.array([r + (h / 2 if axis == a else 0.0) for a in ("X", "Y", "Z")])


def body(name, pos, mass, boxes=(), capsules=(), flaps=(), yaw=0.0, color=(0.6, 0.6, 0.6), **kw) -> Body:
    """Build a :class:`Body`; its bounds are filled in from its shapes (flaps at angle 0 included)."""
    all_boxes = list(boxes) + [b for f in flaps for b in f.boxes]
    centres = [np.array(c) for c, _, _ in all_boxes] + [np.array(c[0]) for c in capsules]
    halves = [np.array(s) / 2 for _, s, _ in all_boxes] + [_capsule_half(c) for c in capsules]
    lo = np.min([c - h for c, h in zip(centres, halves)], axis=0)
    hi = np.max([c + h for c, h in zip(centres, halves)], axis=0)
    return Body(name=name, pos=tuple(pos), size=tuple((hi - lo).tolist()), mass=mass, color=color, yaw=yaw,
                boxes=tuple(boxes), capsules=tuple(capsules), flaps=tuple(flaps), lo=tuple(lo.tolist()),
                hi=tuple(hi.tolist()), **kw)


@dataclass(frozen=True)
class BodyState(ObjectState):
    """A kit body as the simulator has it: ``pos`` and ``quat_xyzw`` are the body origin's pose; ``flaps`` maps each
    flap's name to its hinge angle in degrees."""

    flaps: dict = field(default_factory=dict)

    def rotation(self) -> np.ndarray:
        return _rotation(self.quat_xyzw)

    def to_world(self, local_point) -> np.ndarray:
        return self.pos + self.rotation() @ np.asarray(local_point, float)

    def to_local(self, world_point) -> np.ndarray:
        return self.rotation().T @ (np.asarray(world_point, float) - self.pos)

    def tilt_deg(self) -> float:
        """Angle between the body's own z axis and the vertical."""
        return math.degrees(math.acos(max(-1.0, min(1.0, float(self.rotation()[2, 2])))))


def _rotation(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _yaw_xyzw(yaw: float) -> tuple:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _material(sim_utils, spec):
    try:
        return sim_utils.RigidBodyMaterialCfg(static_friction=spec.static_friction,
                                              dynamic_friction=spec.dynamic_friction,
                                              friction_combine_mode=spec.combine)
    except TypeError:  # a build without the combine-mode field: keep the coefficients
        return sim_utils.RigidBodyMaterialCfg(static_friction=spec.static_friction,
                                              dynamic_friction=spec.dynamic_friction)


# -- spawning ---------------------------------------------------------------------------------------------------------

_BODIES: dict[str, Body] = {}


def _slick_object(scene, spec: Slick):
    """The Arena object for a Slick: yam_arena's ``task_object`` for a Box, with this material."""
    sim_utils = scene.sim_utils
    look = sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(spec.color), roughness=spec.roughness)
    if spec.free:
        parts = dict(rigid_props=scene.PhysxRigidBodyPropertiesCfg(),
                     mass_props=sim_utils.MassPropertiesCfg(mass=spec.mass),
                     collision_props=scene.PhysxCollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0))
    else:
        parts = dict(collision_props=scene.PhysxCollisionPropertiesCfg())
    cfg = sim_utils.CuboidCfg(size=tuple(spec.size), physics_material=_material(sim_utils, spec),
                              visual_material=look, **parts)
    return scene.Object(name=spec.name, object_type=scene.ObjectType.RIGID if spec.free else scene.ObjectType.BASE,
                        spawner_cfg=cfg,
                        initial_pose=scene.Pose(position_xyz=tuple(spec.pos), rotation_xyzw=_yaw_xyzw(spec.yaw)))


def _spawn_body(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the body under ``prim_path`` (pxr only): a plain Xform holding the body and its flaps as siblings."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec: Body = _BODIES[cfg.body_name]
    stage = get_current_stage()
    double = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))

    phys = f"{prim_path}/Looks/physics"
    pm = _material(sim_utils, spec)
    pm.func(phys, pm)
    looks: dict = {}

    def look(color) -> str:
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=spec.roughness)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    def rigid(path: str, pos, mass: float):
        xform = UsdGeom.Xform.Define(stage, path)
        xform.AddTranslateOp(precision=double).Set(Gf.Vec3d(*pos))
        xform.AddOrientOp(precision=double).Set(Gf.Quatd(1, 0, 0, 0))
        prim = xform.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(int(spec.solver_iterations))
        rb.CreateLinearDampingAttr(spec.linear_damping)
        rb.CreateAngularDampingAttr(spec.angular_damping)
        if not spec.gyroscopic:
            rb.CreateEnableGyroscopicForcesAttr(False)
        if spec.sleep_threshold is not None:
            rb.CreateSleepThresholdAttr(float(spec.sleep_threshold))
            rb.CreateStabilizationThresholdAttr(float(spec.sleep_threshold) / 5)
        return prim

    def collide(prim, color):
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(spec.contact_offset)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), phys, stage=stage)
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)

    def add_boxes(path: str, specs, offset=(0.0, 0.0, 0.0)):
        for i, (centre, size, color) in enumerate(specs):
            cube = UsdGeom.Cube.Define(stage, f"{path}/box_{i}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre, float) - np.array(offset, float))))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            collide(cube.GetPrim(), color)

    def add_capsules(path: str, specs):
        for i, c in enumerate(specs):
            centre, radius, height, color = c[:4]
            cap = UsdGeom.Capsule.Define(stage, f"{path}/capsule_{i}")
            cap.CreateAxisAttr(c[4] if len(c) > 4 else "Z")
            cap.CreateRadiusAttr(radius)
            cap.CreateHeightAttr(height)
            cap.AddTranslateOp().Set(Gf.Vec3d(*centre))
            collide(cap.GetPrim(), color)

    main = f"{prim_path}/body"
    rigid(main, (0.0, 0.0, 0.0), spec.mass)
    add_boxes(main, spec.boxes)
    add_capsules(main, spec.capsules)
    for f in spec.flaps:
        path = f"{prim_path}/{f.name}"
        rigid(path, f.pivot, f.mass)
        add_boxes(path, f.boxes, offset=f.pivot)
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{path}/{f.name}_hinge")
        j.CreateAxisAttr(f.axis)
        j.CreateBody0Rel().SetTargets([Sdf.Path(main)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*f.pivot))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLowerLimitAttr(float(f.lower))
        j.CreateUpperLimitAttr(float(f.upper))
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(math.radians(float(f.hold)))      # USD angular drives are per degree
        drive.CreateDampingAttr(math.radians(float(f.damping)))
        drive.CreateTargetPositionAttr(0.0)
    if spec.flaps:
        UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(main))
        art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(main))
        art.CreateEnabledSelfCollisionsAttr(False)
        art.CreateSolverPositionIterationCountAttr(32)
        art.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


def _body_object(spec: Body):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
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

    class KitObject(Object):
        def _generate_articulation_cfg(self):
            actuators = {f.name: ImplicitActuatorCfg(joint_names_expr=[f"{f.name}_hinge"], stiffness=f.hold,
                                                     damping=f.damping) for f in spec.flaps}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    return KitObject(
        name=spec.name,
        object_type=ObjectType.ARTICULATION if spec.flaps else ObjectType.RIGID,
        spawner_cfg=BodySpawnerCfg(body_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=_yaw_xyzw(spec.yaw)),
    )


def _np(x) -> np.ndarray:
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _body_state(environment, spec: Body) -> BodyState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.root_link_pose_w)[0]
    vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    pos, quat = pose[:3] - origin, pose[3:7]
    lo, hi = np.array(spec.lo), np.array(spec.hi)
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    pts = corners @ _rotation(quat).T + pos
    flaps = {}
    if spec.flaps:
        names = list(asset.joint_names)
        q = _np(asset.data.joint_pos)[0]
        flaps = {f.name: math.degrees(float(q[names.index(f"{f.name}_hinge")])) for f in spec.flaps}
    return BodyState(pos=pos, quat_xyzw=quat, lin_vel=vel[:3], ang_vel=vel[3:6], low=pts.min(0), high=pts.max(0),
                     flaps=flaps)


def _t(x):
    return getattr(x, "torch", x)


def set_flap_hold(environment, body_name: str, flap_name: str, hold: float) -> None:
    """Set a flap's hinge spring (N m/rad toward angle 0) in the running simulation."""
    import torch

    asset = environment.unwrapped.scene[body_name]
    j = list(asset.joint_names).index(f"{flap_name}_hinge")
    value = torch.full((1, 1), float(hold), device=_t(asset.data.joint_pos).device)
    write = getattr(asset, "write_joint_stiffness_to_sim_index", None)
    if write is not None:
        write(stiffness=value, joint_ids=[j], env_ids=[0])
    else:  # older Isaac Lab
        asset.write_joint_stiffness_to_sim(value, joint_ids=[j], env_ids=[0])


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
    if getattr(scene, "_eggkit_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Body):
            _BODIES[spec.name] = spec
            return _body_object(spec)
        if isinstance(spec, Slick):
            return _slick_object(scene, spec)
        return stock_object(spec)

    def object_states(environment, names):
        plain = [n for n in names if n not in _BODIES]
        out = stock_states(environment, plain) if plain else {}
        for n in names:
            if n in _BODIES:
                out[n] = _body_state(environment, _BODIES[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._eggkit_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_eggkit_patched", False):
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
    cls._eggkit_patched = True


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
