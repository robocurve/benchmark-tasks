"""strapkit: one PhysX articulation built from rigid parts on revolute, prismatic and fixed joints, for yam_arena task
files. It adds what bodykit's Linkage lacks for a strap threaded through a buckle:

- prismatic joints (a part that slides along a world axis), with limits, Coulomb friction and a spring the task file
  may change while the run goes on (``set_friction``, ``set_drive``);
- loop-closing joints (``Loop``): a part also fixed to another part or to the world outside the joint tree (a strap
  sewn to the bag at both ends); this copy adds ``Loop(kind="spherical")``, a loop joint that holds the part at a
  point but lets it turn (a seat belt's webbing end on its floor anchor bolt);
- collision filtering between chosen pairs of parts (``filters``);
- per-joint drive (spring toward 0, damping), armature and friction.

Every part's frame is world-aligned at the start, with its origin at ``pos``; its shapes (bodykit BoxShape, CylShape,
SphereShape) are given in that frame. Each joint's axis is a world axis at the start, through ``anchor``. Positions
of revolute joints are radians, of prismatic joints metres (as Isaac Lab reports them). Import this file next to the
task file (it needs bodykit):

    from strapkit import Part, Loop, Strap
    STRAP = Strap.build("strap", parts=[...], loops=[...], filters=[...])

``score()`` receives a :class:`StrapState`: the root part's pose, every joint's position and every part's pose.
Pure Python at import time; everything that needs Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

from bodykit import BoxShape, CylShape, SphereShape

__all__ = ["Loop", "Part", "Strap", "StrapState", "ids", "install", "set_drive", "set_friction", "set_target"]


@dataclass(frozen=True)
class Part:
    """A rigid part. ``parent`` None: the articulation's root (exactly one part). Otherwise it hangs from ``parent`` (a
    part listed before it) by a ``joint``: "revolute" (about the world ``axis`` through ``anchor``), "prismatic"
    (sliding along the world ``axis``) or "fixed". ``limits``: (low, high) in degrees (revolute) or metres
    (prismatic). ``stiffness`` (N m/rad or N/m) springs it toward position 0 with ``damping``; ``friction`` (N m or N)
    is its Coulomb friction. ``inertia``: diagonal inertia (kg m^2) when the part has no colliding shapes (else PhysX
    computes it from the shapes and ``mass``)."""

    name: str
    shapes: tuple
    mass: float
    pos: tuple
    parent: str | None = None
    joint: str = "revolute"
    axis: str = "X"
    anchor: tuple | None = None
    limits: tuple | None = None
    stiffness: float = 0.0
    damping: float = 0.0
    friction: float = 0.0
    armature: float = 0.0
    inertia: tuple | None = None
    material: tuple = (0.6, 0.5)  # (static, dynamic) friction of its shapes
    linear_damping: float = 0.0
    angular_damping: float = 0.05


@dataclass(frozen=True)
class Loop:
    """A fixed joint closing a loop: ``part`` held to ``other`` (a part's name; None: the world) where they are at the
    start, outside the articulation's joint tree."""

    part: str
    other: str | None = None
    kind: str = "fixed"  # "fixed", or "spherical" (held at the point, free to turn: a webbing end on its anchor bolt)


@dataclass(frozen=True)
class Strap(Box):
    """The articulation. It subclasses Box only so yam_arena's task-file checks accept it: ``pos`` is the root part's
    origin, ``size`` the bounding box of every part, ``mass`` the root's mass. ``floating``: the root is a loose body
    (else fixed to the world)."""

    parts: tuple = ()
    loops: tuple = ()
    filters: tuple = ()  # (part name, part name) pairs that never collide (also: (part, "@<prim path>") for a body
    # outside the articulation)
    floating: bool = False
    self_collision: bool = True
    solver_iterations: int = 32
    velocity_iterations: int = 4
    contact_offset: float = 0.002

    @staticmethod
    def build(name: str, parts, loops=(), filters=(), color=(0.5, 0.5, 0.5), **kw) -> "Strap":
        root = [p for p in parts if p.parent is None]
        if len(root) != 1:
            raise ValueError(f"{name}: exactly one part must have no parent")
        lo, hi = _bounds(parts)
        return Strap(name=name, pos=tuple(float(v) for v in root[0].pos), size=tuple((hi - lo).tolist()),
                     mass=float(root[0].mass), color=color, parts=tuple(parts), loops=tuple(loops),
                     filters=tuple(filters), **kw)

    @property
    def free(self) -> bool:  # never jittered by yam_arena
        return False

    def part(self, name: str) -> Part:
        for p in self.parts:
            if p.name == name:
                return p
        raise KeyError(name)

    def local_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        lo, hi = _bounds(self.parts)
        return lo - np.asarray(self.pos), hi - np.asarray(self.pos)

    def _check(self) -> None:
        super()._check()
        seen = set()
        for p in self.parts:
            if p.parent is not None and p.parent not in seen:
                raise ValueError(f"{self.name}: part {p.name!r}'s parent {p.parent!r} must come before it")
            if p.joint not in ("revolute", "prismatic", "fixed"):
                raise ValueError(f"{self.name}: part {p.name!r}: unknown joint {p.joint!r}")
            seen.add(p.name)


def _half(shape) -> np.ndarray:
    return shape.half_extent()


def _bounds(parts) -> tuple[np.ndarray, np.ndarray]:
    pts_lo, pts_hi = [], []
    for p in parts:
        for s in p.shapes:
            c = np.asarray(p.pos, float) + np.asarray(s.centre, float)
            pts_lo.append(c - _half(s))
            pts_hi.append(c + _half(s))
        if not p.shapes:
            pts_lo.append(np.asarray(p.pos, float))
            pts_hi.append(np.asarray(p.pos, float))
    return np.min(pts_lo, axis=0), np.max(pts_hi, axis=0)


@dataclass(frozen=True)
class StrapState(ObjectState):
    """The root part's pose and bounds; ``joints``: each jointed part's joint position (rad or m) by part name;
    ``poses``: each part's (position, quaternion xyzw) in the world (env frame)."""

    joints: dict = field(default_factory=dict)
    poses: dict = field(default_factory=dict)


# -- spawning (needs Isaac Sim) -----------------------------------------------------------------------------------

_SPECS: dict = {}


def _spawn_strap(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.schemas.schemas import create_world_fixed_joint
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec: Strap = _SPECS[cfg.body_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
    origin = np.asarray(spec.pos, float)

    looks: dict = {}
    mats: dict = {}

    def look(color) -> str:
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            surface = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.6)
            surface.func(path, surface)
            looks[color] = path
        return looks[color]

    def mat(fr) -> str:
        fr = (float(fr[0]), float(fr[1]))
        if fr not in mats:
            path = f"{prim_path}/Looks/m{len(mats)}"
            m = sim_utils.RigidBodyMaterialCfg(static_friction=fr[0], dynamic_friction=fr[1])
            m.func(path, m)
            mats[fr] = path
        return mats[fr]

    paths: dict = {}
    for part in spec.parts:
        path = f"{prim_path}/{part.name}"
        paths[part.name] = path
        xform = UsdGeom.Xform.Define(stage, path)
        rel = np.asarray(part.pos, float) - origin
        xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*[float(v) for v in rel]))
        xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
        prim = xform.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr(float(part.mass))
        colliding = [s for s in part.shapes if s.collide]
        if part.inertia is not None or not colliding:
            inertia = part.inertia if part.inertia is not None else (1e-8, 1e-8, 1e-8)
            mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*[float(v) for v in inertia]))
            mass_api.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.0))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.5)
        rb.CreateLinearDampingAttr(float(part.linear_damping))
        rb.CreateAngularDampingAttr(float(part.angular_damping))
        for i, shape in enumerate(part.shapes):
            if isinstance(shape, BoxShape):
                geom = UsdGeom.Cube.Define(stage, f"{path}/s{i}")
                geom.CreateSizeAttr(1.0)
                geom.AddTranslateOp().Set(Gf.Vec3d(*shape.centre))
                qx, qy, qz, qw = shape.quat()
                if (qx, qy, qz) != (0.0, 0.0, 0.0):
                    geom.AddOrientOp().Set(Gf.Quatf(qw, qx, qy, qz))
                geom.AddScaleOp().Set(Gf.Vec3f(*shape.size))
            elif isinstance(shape, SphereShape):
                geom = UsdGeom.Sphere.Define(stage, f"{path}/s{i}")
                geom.CreateRadiusAttr(shape.radius)
                geom.AddTranslateOp().Set(Gf.Vec3d(*shape.centre))
            elif isinstance(shape, CylShape):
                geom = UsdGeom.Cylinder.Define(stage, f"{path}/s{i}")
                geom.CreateAxisAttr(shape.axis)
                geom.CreateRadiusAttr(shape.radius)
                geom.CreateHeightAttr(shape.height)
                geom.AddTranslateOp().Set(Gf.Vec3d(*shape.centre))
            else:
                raise TypeError(f"{part.name}: unknown shape {shape!r}")
            gpath = str(geom.GetPrim().GetPath())
            bind_visual_material(gpath, look(tuple(shape.color)), stage=stage)
            if shape.collide:
                UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
                col = PhysxSchema.PhysxCollisionAPI.Apply(geom.GetPrim())
                col.CreateContactOffsetAttr(float(spec.contact_offset))
                col.CreateRestOffsetAttr(0.0)
                bind_physics_material(gpath, mat(part.material), stage=stage)

    # joints of the tree
    jpaths: dict = {}
    for part in spec.parts:
        if part.parent is None:
            continue
        parent = spec.part(part.parent)
        anchor = np.asarray(part.anchor if part.anchor is not None else part.pos, float)
        jp = f"{paths[part.name]}/{part.name}_joint"
        if part.joint == "revolute":
            j = UsdPhysics.RevoluteJoint.Define(stage, jp)
            j.CreateAxisAttr(part.axis)
            if part.limits is not None:
                j.CreateLowerLimitAttr(float(part.limits[0]))
                j.CreateUpperLimitAttr(float(part.limits[1]))
        elif part.joint == "prismatic":
            j = UsdPhysics.PrismaticJoint.Define(stage, jp)
            j.CreateAxisAttr(part.axis)
            if part.limits is not None:
                j.CreateLowerLimitAttr(float(part.limits[0]))
                j.CreateUpperLimitAttr(float(part.limits[1]))
        else:
            j = UsdPhysics.FixedJoint.Define(stage, jp)
        j.CreateBody0Rel().SetTargets([Sdf.Path(paths[parent.name])])
        j.CreateBody1Rel().SetTargets([Sdf.Path(paths[part.name])])
        j.CreateLocalPos0Attr(Gf.Vec3f(*[float(v) for v in anchor - np.asarray(parent.pos, float)]))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(*[float(v) for v in anchor - np.asarray(part.pos, float)]))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        if part.joint != "fixed" and (part.stiffness or part.damping):
            kind = "angular" if part.joint == "revolute" else "linear"
            per = math.pi / 180.0 if kind == "angular" else 1.0  # USD angular drives take N m per degree
            drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), kind)
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(float(part.stiffness) * per)
            drive.CreateDampingAttr(float(part.damping) * per)
            drive.CreateTargetPositionAttr(0.0)
            drive.CreateMaxForceAttr(1.0e6)
        jpaths[part.name] = jp
    # loop-closing fixed joints (outside the tree)
    for k, loop in enumerate(spec.loops):
        part = spec.part(loop.part)
        lp = f"{prim_path}/loop_{k}"
        j = (UsdPhysics.SphericalJoint if loop.kind == "spherical" else UsdPhysics.FixedJoint).Define(stage, lp)
        j.CreateExcludeFromArticulationAttr(True)
        here = np.asarray(part.pos, float)
        if loop.other is None:
            world = here + np.asarray(translation or (0.0, 0.0, 0.0), float) - origin
            j.CreateLocalPos0Attr(Gf.Vec3f(*[float(v) for v in world]))
        else:
            other = spec.part(loop.other)
            j.CreateBody0Rel().SetTargets([Sdf.Path(paths[other.name])])
            j.CreateLocalPos0Attr(Gf.Vec3f(*[float(v) for v in here - np.asarray(other.pos, float)]))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateBody1Rel().SetTargets([Sdf.Path(paths[part.name])])
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))

    # collision filters
    pairs: dict = {}
    for a, b in spec.filters:
        pa = paths[a]
        pb = b[1:] if b.startswith("@") else paths[b]
        pairs.setdefault(pa, []).append(pb)
    for pa, targets in pairs.items():
        api = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(pa))
        rel = api.CreateFilteredPairsRel()
        for pb in targets:
            rel.AddTarget(Sdf.Path(pb))

    root_part = [p for p in spec.parts if p.parent is None][0]
    root_prim = stage.GetPrimAtPath(paths[root_part.name])
    UsdPhysics.ArticulationRootAPI.Apply(root_prim)
    art = PhysxSchema.PhysxArticulationAPI.Apply(root_prim)
    art.CreateEnabledSelfCollisionsAttr(bool(spec.self_collision))
    art.CreateSolverPositionIterationCountAttr(int(spec.solver_iterations))
    art.CreateSolverVelocityIterationCountAttr(int(spec.velocity_iterations))
    if not spec.floating:
        create_world_fixed_joint(root_prim, stage)
    return root.GetPrim()


def _strap_object(spec: Strap):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class StrapSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_strap)
        body_name: str = ""

    class StrapObject(Object):
        def _generate_articulation_cfg(self):
            actuators = {}
            for p in spec.parts:
                if p.parent is None or p.joint == "fixed":
                    continue
                actuators[p.name] = ImplicitActuatorCfg(
                    joint_names_expr=[f"{p.name}_joint"], stiffness=float(p.stiffness), damping=float(p.damping),
                    friction=float(p.friction), dynamic_friction=0.95 * float(p.friction), viscous_friction=0.0,
                    armature=float(p.armature))
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    return StrapObject(
        name=spec.name,
        object_type=ObjectType.ARTICULATION,
        spawner_cfg=StrapSpawnerCfg(body_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)),
    )


def as_numpy(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


_IDS: dict = {}


def ids(asset, spec: Strap) -> dict:
    """{"joint": {part: joint index}, "body": {part: body index}} for this articulation (cached)."""
    if spec.name not in _IDS:
        jn, bn = list(asset.joint_names), list(asset.body_names)
        _IDS[spec.name] = {"joint": {p.name: jn.index(f"{p.name}_joint") for p in spec.parts
                                     if p.parent is not None and p.joint != "fixed"},
                           "body": {p.name: bn.index(p.name) for p in spec.parts}}
    return _IDS[spec.name]


def _strap_state(environment, spec: Strap) -> StrapState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = as_numpy(base.scene.env_origins[0])
    idx = ids(asset, spec)
    q = as_numpy(asset.data.joint_pos)[0]
    bp = as_numpy(asset.data.body_pos_w)[0] - origin
    bq = as_numpy(asset.data.body_quat_w)[0]
    root = [p for p in spec.parts if p.parent is None][0]
    i0 = idx["body"][root.name]
    lo, hi = spec.local_bounds()
    pos = bp[i0]
    return StrapState(pos=pos, quat_xyzw=bq[i0], lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pos + lo,
                      high=pos + hi, joints={n: float(q[i]) for n, i in idx["joint"].items()},
                      poses={n: (bp[i], bq[i]) for n, i in idx["body"].items()})


def set_friction(env, spec: Strap, part: str, value: float) -> None:
    """Write the Coulomb friction (static and dynamic, N or N m) of ``part``'s joint into the simulation."""
    import torch

    base = getattr(env, "unwrapped", env)
    asset = base.scene[spec.name]
    j = ids(asset, spec)["joint"][part]
    val = torch.tensor([[float(value)]], dtype=torch.float32, device=asset.device)
    asset.write_joint_friction_coefficient_to_sim_index(joint_friction_coeff=val,
                                                        joint_dynamic_friction_coeff=0.95 * val, joint_ids=[j])


def set_drive(env, spec: Strap, part: str, stiffness: float, damping: float, target: float) -> None:
    """Spring ``part``'s joint toward ``target`` (m or rad) with ``stiffness`` (N/m or N m/rad) and ``damping``: the
    gains are written into the simulation, the target into the joint target buffer the environment writes every
    physics step."""
    import torch

    base = getattr(env, "unwrapped", env)
    asset = base.scene[spec.name]
    j = ids(asset, spec)["joint"][part]
    dev = asset.device
    asset.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([[float(stiffness)]], dtype=torch.float32,
                                                                    device=dev), joint_ids=[j])
    asset.write_joint_damping_to_sim_index(damping=torch.tensor([[float(damping)]], dtype=torch.float32, device=dev),
                                           joint_ids=[j])
    asset.set_joint_position_target_index(target=torch.tensor([[float(target)]], dtype=torch.float32, device=dev),
                                          joint_ids=[j])


def set_target(env, spec: Strap, part: str, target: float) -> None:
    """Set the rest point (m or rad) of ``part``'s joint spring (its ``stiffness``) for the coming control step."""
    import torch

    base = getattr(env, "unwrapped", env)
    asset = base.scene[spec.name]
    j = ids(asset, spec)["joint"][part]
    asset.set_joint_position_target_index(target=torch.tensor([[float(target)]], dtype=torch.float32,
                                                              device=asset.device), joint_ids=[j])


# -- hooking yam_arena.scene ----------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_strapkit_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Strap):
            _SPECS[spec.name] = spec
            return _strap_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = orig_states(environment, plain) if plain else {}
        for n in names:
            if n in _SPECS:
                out[n] = _strap_state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._strapkit_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
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
            _patch_scene(module)

        spec.loader.exec_module = exec_module
        return spec


def install() -> None:
    """Hook yam_arena.scene (now if already imported, else when it is). Idempotent."""
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    elif not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
