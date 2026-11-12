"""Passive articulated parts. NumPy offline; USD/Isaac imports stay lazy.

Shapes and anchors use the part frame at q=0. Runtime hinge positions are
radians; authored hinge limits, initial/rest angles and USD targets are degrees.
Supported readout lamps change colour or visibility from physical state only:
never collision geometry, never scored, and only where a real device has a readout.
"""
from __future__ import annotations

import functools
import importlib.abc
import importlib.machinery
import math
import sys
from dataclasses import dataclass, field, replace
from types import SimpleNamespace

import numpy as np
import yam_parts as parts
from yam_arena.task_file import GREY


@dataclass(frozen=True)
class Hinge:
    """Revolute joint; stiction efforts are N·m, without degree conversion.

    friction is PhysX jointFriction (unitless, weak dissipation, not stiction).
    Use static_friction and dynamic_friction for absolute friction efforts.
    """
    axis: str
    anchor: tuple[float, float, float]
    lower: float
    upper: float
    damping: float = 0.0
    spring: float = 0.0
    rest: float = 0.0
    friction: float = 0.0  # PhysX jointFriction: unitless weak dissipation; see static/dynamic_friction
    initial: float = 0.0  # degrees
    armature: float = 0.0  # kg m²
    static_friction: float = 0.0  # N·m
    dynamic_friction: float = 0.0  # N·m


@dataclass(frozen=True)
class Slide:
    """Prismatic joint; static_friction and dynamic_friction are efforts in N.

    friction is PhysX jointFriction (unitless, weak dissipation, not stiction).
    Use static_friction and dynamic_friction for absolute friction efforts.
    """
    axis: str
    anchor: tuple[float, float, float]
    lower: float
    upper: float
    damping: float = 0.0
    spring: float = 0.0
    rest: float = 0.0
    friction: float = 0.0  # PhysX jointFriction: unitless weak dissipation; see static/dynamic_friction
    initial: float = 0.0  # metres
    armature: float = 0.0  # kg
    static_friction: float = 0.0  # N
    dynamic_friction: float = 0.0  # N


@dataclass(frozen=True)
class Link:
    name: str
    shapes: tuple[parts.Shape, ...]
    joint: Hinge | Slide
    mass: float
    parent: str | None = None
    color: tuple[float, float, float] | None = None
    friction: tuple[float, float] | None = None

    def __post_init__(self):
        object.__setattr__(self, "shapes", tuple(self.shapes))


@dataclass(frozen=True)
class Lamp:
    """Supported visual readout in the part frame at q=0.

    Colour or visibility only, driven only by physical state; never collision
    geometry or scored, and only where the real device has a readout.
    """
    name: str
    center: tuple[float, float, float]
    radius: float
    link: str | None = None
    on: tuple[float, float, float] = (0.1, 0.9, 0.2)
    off: tuple[float, float, float] = (0.12, 0.12, 0.12)
    method: str = "color"


@dataclass(frozen=True)
class LinkView:
    """One link's reported bounds, suitable for a release-distance test."""
    pos: np.ndarray
    low: np.ndarray
    high: np.ndarray
    lin_vel: np.ndarray = field(default_factory=lambda: np.zeros(3))


def _rotation(axis, angle):
    v = np.eye(3)["xyz".index(axis)]
    cross = np.array(((0, -v[2], v[1]), (v[2], 0, -v[0]), (-v[1], v[0], 0)))
    return np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * (cross @ cross)


def _quat(R):
    # Eigenvector form remains stable at half turns, without scipy/Isaac.
    m = np.asarray(R)
    K = np.array(((m[0, 0]-m[1, 1]-m[2, 2], m[0, 1]+m[1, 0], m[0, 2]+m[2, 0], m[2, 1]-m[1, 2]),
                  (m[0, 1]+m[1, 0], m[1, 1]-m[0, 0]-m[2, 2], m[1, 2]+m[2, 1], m[0, 2]-m[2, 0]),
                  (m[0, 2]+m[2, 0], m[1, 2]+m[2, 1], m[2, 2]-m[0, 0]-m[1, 1], m[1, 0]-m[0, 1]),
                  (m[2, 1]-m[1, 2], m[0, 2]-m[2, 0], m[1, 0]-m[0, 1], np.trace(m)))) / 3
    q = np.linalg.eigh(K)[1][:, -1]
    return q if q[3] >= 0 else -q


def _rot(q):
    # Reuse the PartState quaternion convention without duplicating it.
    return parts.PartState.rotation(SimpleNamespace(quat_xyzw=q))


def _bounds(shapes, exact=False):
    points = []
    for kind, center, R, dims in shapes:
        half = (np.asarray(dims) / 2 if kind == "box" else
                np.array((dims[0], dims[0], dims[1] / 2)) if kind == "cyl" else
                np.full(3, dims[0]))
        extent = np.abs(R) @ half
        if exact and kind == "ball":
            extent = half
        elif exact and kind == "cyl":
            extent = dims[0] * np.linalg.norm(R[:, :2], axis=1) + dims[1] / 2 * np.abs(R[:, 2])
        points.extend((center - extent, center + extent))
    return np.min(points, axis=0), np.max(points, axis=0)


@dataclass(frozen=True, init=False)
class Mechanism(parts.Part):
    base_shapes: tuple[parts.Shape, ...]
    links: tuple[Link, ...]
    self_collide: bool = False
    lamps: tuple[Lamp, ...] = ()
    lamp_logic: object = field(default=None, repr=False, compare=False)

    def __init__(self, name, pos, shapes, links, mass=None, color=GREY, yaw=0.0,
                 friction=None, contact_offset=0.002, grip=None, self_collide=False,
                 lamps=(), lamp_logic=None):
        base_shapes, links = tuple(shapes), tuple(links)
        object.__setattr__(self, "base_shapes", base_shapes)
        object.__setattr__(self, "links", links)
        object.__setattr__(self, "self_collide", bool(self_collide))
        object.__setattr__(self, "lamps", tuple(lamps))
        object.__setattr__(self, "lamp_logic", lamp_logic)
        super().__init__(name, pos, base_shapes + tuple(s for link in links for s in link.shapes),
                         mass, color, yaw, friction, contact_offset, grip)

    def _check(self):
        super()._check()
        if self.mass is not None and not math.isfinite(self.mass):
            raise ValueError("base mass must be positive and finite")
        if not self.base_shapes or not self.links:
            raise ValueError("a mechanism needs base shapes and at least one link")
        seen = set()
        for link in self.links:
            if not isinstance(link.name, str) or not link.name.isidentifier() or link.name == "base" or link.name in seen:
                raise ValueError("link names must be unique identifiers other than base")
            if link.parent is not None and link.parent not in seen:
                raise ValueError(f"{link.name}: parent must name an earlier link")
            if not math.isfinite(link.mass) or link.mass <= 0:
                raise ValueError(f"{link.name}: mass must be positive and finite")
            parts.Part(link.name, (0, 0, 0), link.shapes, link.mass,
                       self.color if link.color is None else link.color,
                       friction=self.friction if link.friction is None else link.friction)._check()
            j = link.joint
            if not isinstance(j, (Hinge, Slide)):
                raise ValueError(f"{link.name}: joint must be Hinge or Slide")
            if j.axis not in ("x", "y", "z"):
                raise ValueError("joint axis must be x, y or z")
            a = np.asarray(j.anchor, dtype=float)
            if a.shape != (3,) or not np.isfinite(a).all():
                raise ValueError("joint anchor must contain three finite numbers")
            if not all(math.isfinite(x) for x in (j.lower, j.upper, j.rest)) or not j.lower < j.upper or not j.lower <= 0 <= j.upper:
                raise ValueError("joint limits must be finite, lower < upper, and contain 0; rest must be finite")
            if not all(math.isfinite(x) and x >= 0 for x in (j.spring, j.damping, j.friction)):
                raise ValueError("joint spring, damping and friction must be finite and non-negative")
            if not math.isfinite(j.initial) or not j.lower <= j.initial <= j.upper:
                raise ValueError("joint initial must be finite and within limits")
            if not math.isfinite(j.armature) or j.armature < 0:
                raise ValueError("joint armature must be finite and non-negative")
            if not all(math.isfinite(x) and x >= 0 for x in (j.static_friction, j.dynamic_friction)):
                raise ValueError("joint static_friction and dynamic_friction must be finite and non-negative")
            if j.dynamic_friction > j.static_friction:
                raise ValueError("joint dynamic_friction must not exceed static_friction")
            seen.add(link.name)
        if self.lamp_logic is not None and not callable(self.lamp_logic):
            raise ValueError("lamp_logic must be callable or None")
        lamp_names = set()
        for lamp in self.lamps:
            if not isinstance(lamp, Lamp):
                raise ValueError("lamps must contain Lamp objects")
            if not isinstance(lamp.name, str) or not lamp.name.isidentifier() or lamp.name in lamp_names:
                raise ValueError("lamp names must be unique identifiers")
            if lamp.link is not None and lamp.link not in seen:
                raise ValueError("lamp link must name a link or be None for the base")
            center = np.asarray(lamp.center, float)
            if center.shape != (3,) or not np.isfinite(center).all():
                raise ValueError("lamp center must contain three finite numbers")
            if not math.isfinite(lamp.radius) or lamp.radius <= 0:
                raise ValueError("lamp radius must be positive and finite")
            for color in (lamp.on, lamp.off):
                rgb = np.asarray(color, float)
                if rgb.shape != (3,) or not np.isfinite(rgb).all() or np.any((rgb < 0) | (rgb > 1)):
                    raise ValueError("lamp colors must contain three finite values in 0..1")
            if lamp.method not in ("color", "visibility"):
                raise ValueError("lamp method must be color or visibility")
            lamp_names.add(lamp.name)

    def _link(self, name):
        return next(link for link in self.links if link.name == name)

    def initial_q(self):
        """Initial joint positions by link, in radians for hinges and metres for slides."""
        return {l.name: float(l.joint.initial) * (math.pi / 180 if isinstance(l.joint, Hinge) else 1.0)
                for l in self.links}

    def _transforms(self, q):
        out = {None: (np.eye(3), np.zeros(3), np.zeros(3))}
        for link in self.links:
            Rp, tp, ap = out[link.parent]
            j = link.joint
            a = np.asarray(j.anchor, float)
            value = float(q.get(link.name, 0.0))
            R, t = Rp.copy(), tp + Rp @ (a - ap)
            if isinstance(j, Hinge):
                R = Rp @ _rotation(j.axis, value)
            else:
                t = t + Rp[:, "xyz".index(j.axis)] * value
            out[link.name] = R, t, a
        return out

    def link_transform(self, link, q):
        """(R, t) mapping the anchor-centred link frame into the part frame."""
        R, t, _ = self._transforms(q)[link]
        return R, t

    def _world_shapes(self, origin, R, poses):
        out = list(parts.Part.world_shapes(parts.Part(self.name, self.pos, self.base_shapes), origin, R))
        for link in self.links:
            p, rot = poses[link.name]
            shifted = tuple(replace(s, center=tuple(np.asarray(s.center) - link.joint.anchor)) for s in link.shapes)
            out.extend(parts.Part.world_shapes(parts.Part(link.name, p, shifted), p, rot))
        return tuple(out)

    def world_shapes_at(self, origin, R, q):
        origin, R = np.asarray(origin, float), np.asarray(R, float)
        poses = {n: (origin + R @ t, R @ r) for n, (r, t, _) in self._transforms(q).items() if n is not None}
        return self._world_shapes(origin, R, poses)

    def aabb_at(self, origin, R, q):
        return _bounds(self.world_shapes_at(origin, R, q))

    def _state(self, origin, quat, joints, joint_vel, links, lin_vel, ang_vel):
        poses = {n: (p, _rot(q)) for n, (p, q) in links.items()}
        low, high = _bounds(self._world_shapes(origin, _rot(quat), poses))
        return MechanismState((low + high) / 2, quat, lin_vel, ang_vel, low, high,
                              origin, joints, joint_vel, links, self)

    def offline_state(self, origin, yaw, q, qd=None, *, R=None):
        """Build a scorer state; missing joints use initial_q, full R overrides yaw."""
        q = {**self.initial_q(), **q}
        origin = np.asarray(origin, float)
        R = _rotation("z", yaw) if R is None else np.asarray(R, float)
        links = {n: (origin + R @ t, _quat(R @ r)) for n, (r, t, _) in self._transforms(q).items() if n is not None}
        return self._state(origin, _quat(R), {l.name: float(q.get(l.name, 0)) for l in self.links},
                           {l.name: float((qd or {}).get(l.name, 0)) for l in self.links}, links,
                           np.zeros(3), np.zeros(3))


@dataclass(frozen=True)
class MechanismState(parts.PartState):
    joints: dict[str, float]
    joint_vel: dict[str, float]
    links: dict[str, tuple[np.ndarray, np.ndarray]]
    _mechanism: Mechanism = field(repr=False, compare=False)

    def angle_deg(self, link):
        if not isinstance(self._mechanism._link(link).joint, Hinge):
            raise ValueError(f"{link} is not a hinge")
        return math.degrees(self.joints[link])

    def link_rotation(self, link):
        return _rot(self.links[link][1])

    def link_to_world(self, link, p_part):
        return self.links[link][0] + self.link_rotation(link) @ (np.asarray(p_part) - self._mechanism._link(link).joint.anchor)

    def still(self, max_speed=0.01, max_joint_speed=0.05):
        return bool(np.linalg.norm(self.lin_vel) < max_speed and np.linalg.norm(self.ang_vel) < max_joint_speed
                    and all(abs(v) < max_joint_speed for v in self.joint_vel.values()))

    def fk_error_mm(self):
        R = self.rotation()
        return 1000 * max((np.linalg.norm(self.links[l.name][0] -
                          (self.origin + R @ self._mechanism.link_transform(l.name, self.joints)[1]))
                          for l in self._mechanism.links), default=0.0)

    def fk_error_deg(self):
        errors = []
        for link in self._mechanism.links:
            expected = self.rotation() @ self._mechanism.link_transform(link.name, self.joints)[0]
            delta = expected.T @ self.link_rotation(link.name)
            sine = np.linalg.norm((delta - delta.T)[[2, 0, 1], [1, 2, 0]]) / 2
            errors.append(math.degrees(math.atan2(sine, np.clip((np.trace(delta) - 1) / 2, -1, 1))))
        return max(errors, default=0.0)

    def link_box(self, link):
        """Exact world AABB of this link's primitives at its reported pose."""
        spec = self._mechanism._link(link)
        shifted = tuple(replace(s, center=tuple(np.asarray(s.center) - spec.joint.anchor)) for s in spec.shapes)
        shapes = parts.Part(link, (0, 0, 0), shifted).world_shapes(self.links[link][0], self.link_rotation(link))
        return _bounds(shapes, exact=True)

    def link_view(self, link):
        low, high = self.link_box(link)
        return LinkView((low + high) / 2, low, high)


_SPECS: dict[str, Mechanism] = {}
_PRIM_PATHS: dict[str, set[str]] = {}
_LAMP_VALUES: dict[tuple[str, str], bool] = {}
_SPAWNER_CFG_TYPE = None
_CLONED_SPAWN = None
_PATCH_TOKEN = object()


def _mechanism_object(spec):
    global _SPAWNER_CFG_TYPE, _CLONED_SPAWN
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab.managers import EventTermCfg, SceneEntityCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.terms.events import reset_articulation_pose_and_joints
    from isaaclab_arena.utils.pose import Pose

    if _SPAWNER_CFG_TYPE is None:
        @configclass
        class MechanismSpawnerCfg(sim_utils.SpawnerCfg):
            mech_name: str = ""
        _SPAWNER_CFG_TYPE = MechanismSpawnerCfg
    if _CLONED_SPAWN is None:
        _CLONED_SPAWN = clone(_spawn_mechanism)

    class MechanismObject(Object):
        def _build_reset_event(self):
            if not self._requires_reset_pose_event():
                return None
            return EventTermCfg(func=reset_articulation_pose_and_joints, mode="reset",
                                params={"asset_cfg": SceneEntityCfg(self.name),
                                        "pose": self.get_initial_pose(), "velocity": None})

    _SPECS[spec.name] = spec
    _PRIM_PATHS[spec.name] = set()
    obj = MechanismObject(name=spec.name, object_type=ObjectType.ARTICULATION,
                           spawner_cfg=_SPAWNER_CFG_TYPE(func=_CLONED_SPAWN, mech_name=spec.name),
                           initial_pose=Pose(position_xyz=spec.pos,
                                             rotation_xyzw=(0, 0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))))
    obj.object_cfg.init_state.joint_pos = {f"{name}_joint": value for name, value in spec.initial_q().items()}
    return obj


def _spawn_mechanism(prim_path, cfg, *args, **kwargs):
    import isaaclab.sim as sim_utils
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    mech = _SPECS[cfg.mech_name]
    stage = kwargs.get("stage") or sim_utils.get_current_stage()
    _PRIM_PATHS.setdefault(mech.name, set()).add(str(prim_path))
    root = UsdGeom.Xform.Define(stage, prim_path)
    translation = kwargs.get("translation", args[0] if args else None)
    orientation = kwargs.get("orientation", args[1] if len(args) > 1 else None)
    if translation is not None:
        root.AddTranslateOp().Set(Gf.Vec3d(*map(float, translation)))
    if orientation is not None:
        x, y, z, w = map(float, orientation)
        root.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    art = PhysxSchema.PhysxArticulationAPI.Apply(root.GetPrim())
    art.CreateEnabledSelfCollisionsAttr().Set(mech.self_collide)
    art.CreateSolverPositionIterationCountAttr().Set(32)
    art.CreateSolverVelocityIterationCountAttr().Set(4)

    # Compose with the unchanged collider/material authoring code in yam_parts.
    bodies = [("base", np.zeros(3), mech.base_shapes, mech.mass or 1.0, mech.color, mech.friction)]
    bodies += [(l.name, np.asarray(l.joint.anchor), l.shapes, l.mass,
                mech.color if l.color is None else l.color,
                mech.friction if l.friction is None else l.friction) for l in mech.links]
    origins = {name: anchor for name, anchor, *_ in bodies}
    initial = mech.initial_q()
    poses = mech._transforms(initial) if any(initial.values()) else {}
    for name, anchor, shapes, mass, color, friction in bodies:
        key = f"{prim_path}/{name}"
        shifted = tuple(replace(s, center=tuple(np.asarray(s.center) - anchor)) for s in shapes)
        parts._SPECS[key] = parts.Part(name, (0, 0, 0), shifted, mass, color,
                                      friction=friction, contact_offset=mech.contact_offset)
        try:
            spawn_pose = {"translation": tuple(map(float, anchor))}
            if name in poses:
                R, t, _ = poses[name]
                spawn_pose = {"translation": tuple(map(float, t)), "orientation": tuple(map(float, _quat(R)))}
            parts._spawn_part(key, SimpleNamespace(part_name=key), **spawn_pose, stage=stage)
        finally:
            del parts._SPECS[key]

    for lamp in mech.lamps:
        anchor = origins[lamp.link or "base"]
        path = _lamp_path(prim_path, lamp)
        UsdGeom.Xform.Define(stage, f"{prim_path}/{lamp.link or 'base'}/lamps")
        UsdGeom.Xform.Define(stage, path)
        variants = [("sphere", lamp.off)] if lamp.method == "color" else [("on", lamp.on), ("off", lamp.off)]
        for variant, color in variants:
            _spawn_lamp_sphere(stage, f"{path}/{variant}", np.asarray(lamp.center) - anchor, lamp.radius, color)
        if lamp.method == "visibility":
            UsdGeom.Imageable(stage.GetPrimAtPath(f"{path}/on")).CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
        _LAMP_VALUES[str(prim_path), lamp.name] = False

    UsdGeom.Xform.Define(stage, f"{prim_path}/joints")
    for link in mech.links:
        j = link.joint
        hinge = isinstance(j, Hinge)
        cls = UsdPhysics.RevoluteJoint if hinge else UsdPhysics.PrismaticJoint
        joint = cls.Define(stage, f"{prim_path}/joints/{link.name}_joint")
        parent = link.parent or "base"
        joint.CreateBody0Rel().SetTargets([f"{prim_path}/{parent}"])
        joint.CreateBody1Rel().SetTargets([f"{prim_path}/{link.name}"])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*map(float, np.asarray(j.anchor) - origins[parent])))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1))
        joint.CreateAxisAttr().Set(j.axis.upper())
        joint.CreateLowerLimitAttr().Set(j.lower)
        joint.CreateUpperLimitAttr().Set(j.upper)
        if j.spring > 0 or j.damping > 0:
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular" if hinge else "linear")
            factor = math.pi / 180 if hinge else 1.0
            drive.CreateTypeAttr().Set("force")
            drive.CreateStiffnessAttr().Set(j.spring * factor)
            drive.CreateDampingAttr().Set(j.damping * factor)
            drive.CreateTargetPositionAttr().Set(j.rest)
            drive.CreateTargetVelocityAttr().Set(0)
            drive.CreateMaxForceAttr().Set(1e10)
        if j.friction > 0:
            PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim()).CreateJointFrictionAttr().Set(j.friction)
        if j.armature > 0:
            PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim()).CreateArmatureAttr().Set(j.armature)
        if j.static_friction > 0 or j.dynamic_friction > 0:
            axis = "angular" if hinge else "linear"
            prim = joint.GetPrim()
            prim.ApplyAPI("PhysxJointAxisAPI", axis)
            for effort, value in (("staticFrictionEffort", j.static_friction),
                                  ("dynamicFrictionEffort", j.dynamic_friction)):
                name = f"physxJointAxis:{axis}:{effort}"
                attr = prim.GetAttribute(name)
                if not attr:
                    from pxr import Sdf
                    attr = prim.CreateAttribute(name, Sdf.ValueTypeNames.Float)
                attr.Set(float(value))
    if mech.mass is None:
        base = stage.GetPrimAtPath(f"{prim_path}/base")
        pose = UsdGeom.XformCache().GetLocalToWorldTransform(base).RemoveScaleShear()
        fixed = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/joints/base_fixed")
        fixed.CreateBody1Rel().SetTargets([base.GetPath()])
        fixed.CreateLocalPos0Attr().Set(Gf.Vec3f(pose.ExtractTranslation()))
        fixed.CreateLocalRot0Attr().Set(Gf.Quatf(pose.ExtractRotationQuat()))
        fixed.CreateLocalPos1Attr().Set(Gf.Vec3f(0))
        fixed.CreateLocalRot1Attr().Set(Gf.Quatf(1))
    return root.GetPrim()


def _lamp_path(prim_path, lamp):
    return f"{prim_path}/{lamp.link or 'base'}/lamps/{lamp.name}"


def _spawn_lamp_sphere(stage, path, center, radius, color):
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    sphere = UsdGeom.Sphere.Define(stage, path)
    sphere.CreateRadiusAttr().Set(radius)
    sphere.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
    material = UsdShade.Material.Define(stage, f"{path}/Material")
    shader = UsdShade.Shader.Define(stage, f"{path}/Material/Shader")
    shader.CreateIdAttr().Set("UsdPreviewSurface")
    for name in ("diffuseColor", "emissiveColor"):
        shader.CreateInput(name, Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(sphere.GetPrim()).Bind(material)


def _update_lamps(mech, state):
    if not mech.lamps or mech.lamp_logic is None:
        return
    values = mech.lamp_logic(state)
    changes = [(path, lamp, bool(values[lamp.name]))
               for path in _PRIM_PATHS.get(mech.name, ()) for lamp in mech.lamps
               if _LAMP_VALUES.get((path, lamp.name)) != bool(values[lamp.name])]
    if not changes:
        return
    import isaaclab.sim as sim_utils
    from pxr import Gf, UsdGeom, UsdShade
    stage = sim_utils.get_current_stage()
    for prim_path, lamp, on in changes:
        path = _lamp_path(prim_path, lamp)
        if lamp.method == "color":
            shader = UsdShade.Shader.Get(stage, f"{path}/sphere/Material/Shader")
            for name in ("diffuseColor", "emissiveColor"):
                shader.GetInput(name).Set(Gf.Vec3f(*(lamp.on if on else lamp.off)))
        else:
            for variant, visible in (("on", on), ("off", not on)):
                UsdGeom.Imageable(stage.GetPrimAtPath(f"{path}/{variant}")).CreateVisibilityAttr().Set(
                    UsdGeom.Tokens.inherited if visible else UsdGeom.Tokens.invisible)
        _LAMP_VALUES[prim_path, lamp.name] = on


def _array(value):
    value = getattr(value, "torch", value)
    return value.detach().cpu().numpy().astype(np.float64)


def _patch_scene(scene):
    parts._patch_scene(scene)
    if getattr(scene, "_yam_joints_patch_token", None) is _PATCH_TOKEN:
        return
    inner_object = getattr(scene.task_object, "_yam_joints_original", scene.task_object)
    inner_states = getattr(scene.object_states, "_yam_joints_original", scene.object_states)

    @functools.wraps(inner_object)
    def task_object(spec):
        if isinstance(spec, Mechanism):
            return _mechanism_object(spec)
        _SPECS.pop(getattr(spec, "name", ""), None)
        return inner_object(spec)

    @functools.wraps(inner_states)
    def object_states(env, names):
        names = tuple(names)
        states = inner_states(env, tuple(n for n in names if n not in _SPECS))
        if not any(n in _SPECS for n in names):
            return states
        base = env.unwrapped
        for name in names:
            if name not in _SPECS:
                continue
            mech = _SPECS[name]
            asset = base.scene.articulations[name]
            data = asset.data
            env_origin = _array(base.scene.env_origins)[0]
            root = _array(data.root_pose_w)[0]
            jp, jv = _array(data.joint_pos)[0], _array(data.joint_vel)[0]
            ji = {n: i for i, n in enumerate(data.joint_names)}
            bi = {n: i for i, n in enumerate(data.body_names)}
            poses = _array(data.body_link_pose_w)[0]
            links = {l.name: (poses[bi[l.name], :3] - env_origin, poses[bi[l.name], 3:]) for l in mech.links}
            states[name] = mech._state(root[:3] - env_origin, root[3:],
                                      {l.name: float(jp[ji[l.name + "_joint"]]) for l in mech.links},
                                      {l.name: float(jv[ji[l.name + "_joint"]]) for l in mech.links}, links,
                                      _array(data.root_lin_vel_w)[0], _array(data.root_ang_vel_w)[0])
            _update_lamps(mech, states[name])
        return states

    task_object._yam_joints_original = inner_object
    object_states._yam_joints_original = inner_states
    scene.task_object, scene.object_states = task_object, object_states
    scene._yam_joints_patch_token = _PATCH_TOKEN


class _SceneFinder(importlib.abc.MetaPathFinder):
    _yam_joints_scene_finder = True

    def __init__(self, patch):
        self.patch = patch

    def find_spec(self, fullname, path=None, target=None):
        if fullname != "yam_arena.scene":
            return None
        spec = importlib.machinery.PathFinder.find_spec(fullname, path)
        if spec is not None and spec.loader is not None:
            spec.loader = parts._SceneLoader(spec.loader, self.patch)
        return spec


def install():
    finder = next((f for f in sys.meta_path if getattr(f, "_yam_joints_scene_finder", False)), None)
    if finder is None:
        sys.meta_path.insert(0, _SceneFinder(_patch_scene))
    else:
        finder.patch = _patch_scene
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])


install()
__all__ = ["Mechanism", "Link", "Hinge", "Slide", "Lamp", "LinkView", "MechanismState", "install"]
