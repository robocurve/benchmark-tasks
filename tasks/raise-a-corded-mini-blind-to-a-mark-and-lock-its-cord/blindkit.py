"""blindkit: one PhysX articulation fixed to the world with prismatic, revolute and ball joints and mimic couplings,
for yam_arena task files (the corded mini blind: bottom rail and slats on vertical slides, a pull cord on a ball
joint at the cord lock with a pay-out slide, the pay-out coupled one to one to the rail's lift).

    import blindkit
    RIG = blindkit.Rig.build("blind", pos=..., size=..., shapes=[...], parts=[blindkit.Part(...), ...],
                             mimic=[("payout", "rail", 1.0)], unfiltered=..., filtered=[("cord0", None)])

Every body's frame is the rig's origin (``pos``), world-aligned, so shapes are given relative to ``pos`` and joint
anchors in world coordinates. The root body (``shapes``) is fixed to the world. A :class:`Part` hangs on its parent
(the root when ``parent`` is None) by a joint at ``anchor``:

- ``prismatic`` along ``axis`` (limits in m), ``revolute`` about ``axis`` (limits in degrees), ``ball`` (three
  rotations, each limited to +-``limits[0]`` degrees), or ``fixed``.
- ``frame``: the joint frame as a rotation (quaternion xyzw, world, at the start) when the axis is not a world axis.
- ``damping`` (viscous), ``friction`` (Coulomb) and ``stiffness`` (a spring toward the start) per joint.

``mimic``: (child, reference, gearing): PhysX keeps ``q_child + gearing * q_reference = 0`` (PhysxMimicJointAPI).
``filtered``: pairs of part names (None: the root) that never collide. Self-collision is on, so parts collide with
each other unless they are parent and child or filtered.

Isaac Lab writes the joints to 0 at every reset: the authored pose is the start. Pure Python at import time;
everything that needs Isaac Sim is imported lazily. No third-party assets.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["Part", "Rig", "RigState", "install"]


@dataclass(frozen=True)
class Part:
    name: str
    shapes: tuple
    mass: float
    kind: str
    anchor: tuple
    parent: str | None = None
    axis: str = "Z"
    frame: tuple | None = None
    limits: tuple | None = None
    damping: float = 0.0
    friction: float = 0.0
    stiffness: float = 0.0
    material: tuple = (0.6, 0.5)
    lin_damping: float = 0.05
    ang_damping: float = 0.05
    com: tuple | None = None  # world point (at the start): the centre of mass, when not the shapes' own
    inertia: tuple | None = None  # principal moments (kg m^2), with ``com``


@dataclass(frozen=True)
class Rig(Box):
    shapes: tuple = ()
    parts: tuple = ()
    mimic: tuple = ()
    filtered: tuple = ()
    root_material: tuple = (0.6, 0.5)
    contact_offset: float = 0.002
    solver_iterations: int = 32

    @property
    def free(self) -> bool:
        return False

    @staticmethod
    def build(name: str, pos, size, **kw) -> "Rig":
        return Rig(name=name, pos=tuple(float(v) for v in pos), size=tuple(float(v) for v in size), **kw)


@dataclass(frozen=True)
class RigState(ObjectState):
    joints: dict = field(default_factory=dict)  # joint name -> position (m or rad)
    bodies: dict = field(default_factory=dict)  # part name -> (pos (3,), rotation 3x3), world minus env origin


_SPECS: dict = {}


def _q(m: np.ndarray) -> tuple:
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return (x / n, y / n, z / n, w / n)


def _spawn_rig(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.schemas.schemas import create_world_fixed_joint
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    import bodykit
    from yam_arena.station import linear_rgb

    spec: Rig = _SPECS[cfg.rig_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
    looks: dict = {}
    mats: dict = {}
    origin = np.asarray(spec.pos, float)

    def look(color) -> str:
        color = tuple(float(c) for c in color)
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            s = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.45)
            s.func(path, s)
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

    def rigid(name: str, mass: float, shapes, material, damping=(0.05, 0.05), com=None, inertia=None) -> str:
        path = f"{prim_path}/{name}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0.0, 0.0, 0.0))
        xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mapi = UsdPhysics.MassAPI.Apply(prim)
        mapi.CreateMassAttr(float(mass))
        if com is not None:
            mapi.CreateCenterOfMassAttr(Gf.Vec3f(*[float(v) for v in np.asarray(com, float) - origin]))
            mapi.CreateDiagonalInertiaAttr(Gf.Vec3f(*[float(v) for v in inertia]))
            mapi.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateLinearDampingAttr(float(damping[0]))
        rb.CreateAngularDampingAttr(float(damping[1]))
        for i, shape in enumerate(shapes):
            geom = bodykit._define_shape(stage, f"{path}/s{i}", shape)
            sp = str(geom.GetPrim().GetPath())
            bind_visual_material(sp, look(shape.color), stage=stage)
            if shape.collide:
                UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
                c = PhysxSchema.PhysxCollisionAPI.Apply(geom.GetPrim())
                c.CreateContactOffsetAttr(float(spec.contact_offset))
                c.CreateRestOffsetAttr(0.0)
                bind_physics_material(sp, mat(material), stage=stage)
        return path

    base = rigid("base", 1.0, spec.shapes, spec.root_material)
    paths = {None: base}
    joints = {}
    for p in spec.parts:
        path = rigid(p.name, p.mass, p.shapes, p.material, (p.lin_damping, p.ang_damping), p.com, p.inertia)
        paths[p.name] = path
        jp = f"{path}/{p.name}_j"
        if p.kind == "prismatic":
            j = UsdPhysics.PrismaticJoint.Define(stage, jp)
            j.CreateAxisAttr(p.axis)
        elif p.kind == "revolute":
            j = UsdPhysics.RevoluteJoint.Define(stage, jp)
            j.CreateAxisAttr(p.axis)
        elif p.kind == "fixed":
            j = UsdPhysics.FixedJoint.Define(stage, jp)
        elif p.kind == "ball":
            j = UsdPhysics.Joint.Define(stage, jp)
            for dof in ("transX", "transY", "transZ"):
                lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), dof)
                lim.CreateLowAttr(1.0)
                lim.CreateHighAttr(-1.0)
            lims = p.limits if p.limits is not None else (179.0,)
            for k, dof in enumerate(("rotX", "rotY", "rotZ")):
                lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), dof)
                a = float(lims[min(k, len(lims) - 1)])
                lim.CreateLowAttr(-a)
                lim.CreateHighAttr(a)
                if p.damping or p.stiffness:
                    drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), dof)
                    drv.CreateTypeAttr("force")
                    drv.CreateStiffnessAttr(float(p.stiffness) * math.pi / 180.0)
                    drv.CreateDampingAttr(float(p.damping) * math.pi / 180.0)
                    drv.CreateTargetPositionAttr(0.0)
        else:
            raise ValueError(f"{p.name}: unknown joint kind {p.kind!r}")
        j.CreateBody0Rel().SetTargets([Sdf.Path(paths[p.parent])])
        j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        at = Gf.Vec3f(*[float(v) for v in np.asarray(p.anchor, float) - origin])
        q = p.frame if p.frame is not None else (0.0, 0.0, 0.0, 1.0)
        j.CreateLocalPos0Attr(at)
        j.CreateLocalRot0Attr(Gf.Quatf(q[3], q[0], q[1], q[2]))
        j.CreateLocalPos1Attr(at)
        j.CreateLocalRot1Attr(Gf.Quatf(q[3], q[0], q[1], q[2]))
        if p.kind in ("prismatic", "revolute") and p.limits is not None:
            j.CreateLowerLimitAttr(float(p.limits[0]))
            j.CreateUpperLimitAttr(float(p.limits[1]))
        joints[p.name] = j
    for child, ref, gearing in spec.mimic:
        m = PhysxSchema.PhysxMimicJointAPI.Apply(joints[child].GetPrim(), UsdPhysics.Tokens.rotX)
        m.GetReferenceJointRel().AddTarget(joints[ref].GetPath())
        m.GetGearingAttr().Set(float(gearing))
        m.GetOffsetAttr().Set(0.0)
    for a, b in spec.filtered:
        UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(paths[a])).GetFilteredPairsRel().AddTarget(
            Sdf.Path(paths[b]))
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    art = PhysxSchema.PhysxArticulationAPI.Apply(root.GetPrim())
    art.CreateEnabledSelfCollisionsAttr(True)
    art.CreateSolverPositionIterationCountAttr(int(spec.solver_iterations))
    art.CreateSolverVelocityIterationCountAttr(4)
    create_world_fixed_joint(stage.GetPrimAtPath(base), stage)
    return root.GetPrim()


def _rig_object(spec: Rig):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class RigSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_rig)
        rig_name: str = ""

    class RigObject(Object):
        def _generate_articulation_cfg(self):
            acts = {}
            for p in spec.parts:
                if p.kind == "fixed":
                    continue
                acts[p.name] = ImplicitActuatorCfg(joint_names_expr=[f"{p.name}_j.*"], stiffness=float(p.stiffness),
                                                   damping=float(p.damping), friction=float(p.friction),
                                                   dynamic_friction=float(p.friction), armature=1e-4)
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=acts)
            return self._add_initial_pose_to_cfg(cfg)

    return RigObject(
        name=spec.name,
        object_type=ObjectType.ARTICULATION,
        spawner_cfg=RigSpawnerCfg(rig_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)),
    )


def _np(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def _rot(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _rig_state(environment, spec: Rig) -> RigState:
    base = environment.unwrapped
    art = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    names = list(art.body_names)
    pose = _np(art.data.body_link_pose_w)[0]
    q = _np(art.data.joint_pos)[0]
    joints = {n: float(v) for n, v in zip(art.joint_names, q)}
    bodies = {n: (pose[i][:3] - origin, _rot(pose[i][3:7])) for i, n in enumerate(names)}
    pos = np.asarray(spec.pos, float)
    half = np.asarray(spec.size, float) / 2
    return RigState(pos=pos, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                    low=pos - half, high=pos + half, joints=joints, bodies=bodies)


def _patch_scene(scene) -> None:
    if getattr(scene, "_blindkit_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Rig):
            _SPECS[spec.name] = spec
            return _rig_object(spec)
        return orig_task_object(spec)

    def object_states(env, names):
        plain = [n for n in names if n not in _SPECS]
        out = orig_states(env, plain) if plain else {}
        for n in names:
            if n in _SPECS:
                out[n] = _rig_state(env, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._blindkit_patched = True


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
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    elif not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
