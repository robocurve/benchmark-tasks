"""syringekit: Luer-lock syringes with capped needles, and fixed stands with their own friction, for yam_arena task
files.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from syringekit import Fixed, Syringe, on_reset, on_step, write_joint

A :class:`Syringe` is one articulation whose root is the syringe (barrel, finger flange, Luer-lock collar and the
plunger drawn back with a dose in it). In its own frame the needle points along +z and the flange's outer face is at
z = 0. Its needle hangs from it on two joints and its cap from the needle on a third:

- ``lift`` (prismatic, along +z, syringe to needle carrier): the needle coming off the collar. The task file keeps it
  locked at 0 until the needle has been unscrewed.
- ``spin`` (revolute, about +z, carrier to needle): the needle turning on the collar's thread. Positive is
  anticlockwise seen from the needle's tip, the way a Luer-lock needle unscrews.
- ``cap`` (prismatic, along +z, needle to cap): the cap sliding off the needle's hub, held by a spring for its
  friction fit. The cap has no joint of its own about the axis (it is keyed to the hub by ribs), so turning the cap
  turns the needle.

``success()`` and ``score()`` receive a :class:`SyringeState` for each syringe: the syringe's pose and the three joint
positions. :func:`write_joint` changes a joint's limits, spring or damping, or its position, in the running
simulation. ``on_step`` and ``on_reset`` register callbacks ``fn(env)``.

The structure follows the kits used by tasks already in this repository (a Box subclass so the loader accepts the
part, a custom spawner, hooks installed through ``sys.meta_path``). Pure Python at import time; Isaac Sim is reached
only when the scene is built.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["Fixed", "FreeNeedle", "JOINTS", "Syringe", "SyringeState", "body_pose", "body_speed", "env",
           "finger_points", "hold_root", "set_part_masses", "install", "needle_pose", "on_reset", "on_step", "show_needle", "write_joint",
           "write_pose"]

JOINTS = ("lift", "spin", "cap")

# -- the syringe and needle, in the syringe's frame (m): a 3 mL Luer-lock syringe with a 21 G x 1 in needle ----------
FLANGE = (0.022, 0.009, 0.002)          # finger flange: across, thick, along the axis
BARREL_R, BARREL_Z = 0.00525, (0.002, 0.054)
COLLAR_R, COLLAR_Z = 0.0042, (0.054, 0.062)
HUB_R, HUB_Z = 0.0035, (0.058, 0.072)
NEEDLE_R, NEEDLE_Z = 0.0004, (0.072, 0.097)
CAP_R, CAP_Z = 0.0040, (0.066, 0.104)
PLUNGER_Z = (-0.032, 0.0)
THUMB_R = 0.0075
STOPPER_Z = (0.023, 0.028)              # the plunger's rubber stopper, 1.5 mL drawn
LIFT_PIVOT = 0.058                      # where the needle's joints sit on the axis (the hub's base)


@dataclass(frozen=True)
class Fixed(Box):
    """A fixed task-file ``Box`` with its own friction (PhysX combines the two bodies' values by ``combine``)."""

    static_friction: float = 0.5
    dynamic_friction: float = 0.4
    combine: str = "average"
    roughness: float = 0.6


@dataclass(frozen=True)
class Syringe(Box):
    """A syringe with its capped needle. ``pos`` is the flange's outer face centre and ``quat`` the syringe frame's
    orientation (xyzw) in the world. Masses in kg; ``spin_damping`` (N m s/rad) is the needle's drag on the thread,
    ``cap_spring`` (N/m) the cap's friction fit."""

    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    needle_mass: float = 0.0006
    cap_mass: float = 0.0008
    carrier_mass: float = 0.0004
    spin_damping: float = 0.02
    cap_spring: float = 2000.0
    cap_max: float = 0.05
    lift_max: float = 0.04
    barrel_friction: tuple = (0.5, 0.4)
    cap_friction: tuple = (0.9, 0.8)
    liquid: tuple = (0.75, 0.85, 0.95)
    hub_color: tuple = (0.20, 0.62, 0.30)       # 21 G is green
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")
        if self.mass is None or self.mass <= 0:
            raise ValueError(f"{self.name}: a Syringe must be loose (give it a mass)")


@dataclass(frozen=True)
class SyringeState(ObjectState):
    """A syringe as the simulator has it: ``pos``/``quat_xyzw`` the syringe frame's pose; ``joints`` maps lift (m),
    spin (rad) and cap (m) to their positions."""

    joints: dict = field(default_factory=dict)

    def rotation(self) -> np.ndarray:
        return _rotation(self.quat_xyzw)


@dataclass(frozen=True)
class FreeNeedle(Box):
    """A capped needle on its own: the hub, the needle and its cap as one loose body, laid out as they are on a
    syringe (its frame is the syringe's needle body frame: the axis along +z, the origin at the hub's base). The task
    file swaps one in for a syringe's needle once that needle has come off its collar."""

    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    hub_color: tuple = (0.20, 0.62, 0.30)
    cap_friction: tuple = (0.9, 0.8)
    meta: dict = field(default_factory=dict, compare=False, hash=False)


def _rotation(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _material(sim_utils, sf, df, combine):
    try:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df, friction_combine_mode=combine)
    except TypeError:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df)


# -- spawning ---------------------------------------------------------------------------------------------------------

_SYRINGES: dict[str, Syringe] = {}
_PRIMS: dict[str, str] = {}
_FREE: dict[str, FreeNeedle] = {}


def _fixed_object(scene, spec: Fixed):
    sim_utils = scene.sim_utils
    look = sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(spec.color), roughness=spec.roughness)
    cfg = sim_utils.CuboidCfg(size=tuple(spec.size),
                              physics_material=_material(sim_utils, spec.static_friction, spec.dynamic_friction,
                                                         spec.combine),
                              visual_material=look, collision_props=scene.PhysxCollisionPropertiesCfg())
    return scene.Object(name=spec.name, object_type=scene.ObjectType.BASE, spawner_cfg=cfg,
                        initial_pose=scene.Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


def _spawn_syringe(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the syringe under ``prim_path`` (pxr only): syringe, needle carrier, needle and cap bodies on three
    joints. Every body's origin is on the axis; the syringe's is its frame's origin."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec: Syringe = _SYRINGES[cfg.syringe_name]
    _PRIMS[cfg.syringe_name] = prim_path
    stage = get_current_stage()
    double = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))

    mats = {}
    for key, (sf, df) in (("barrel", spec.barrel_friction), ("cap", spec.cap_friction)):
        path = f"{prim_path}/Looks/phys_{key}"
        pm = _material(sim_utils, sf, df, "max" if key == "cap" else "average")
        pm.func(path, pm)
        mats[key] = path
    looks = {}

    def look(color, opacity=1.0, rough=0.4, metal=0.0) -> str:
        key = (color, opacity, rough, metal)
        if key not in looks:
            path = f"{prim_path}/Looks/v{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough, metallic=metal,
                                              opacity=opacity)
            vis.func(path, vis)
            looks[key] = path
        return looks[key]

    def body(path, z, mass):
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=double).Set(Gf.Vec3d(0.0, 0.0, z))
        xf.AddOrientOp(precision=double).Set(Gf.Quatd(1, 0, 0, 0))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.5)
        rb.CreateLinearDampingAttr(0.1)
        rb.CreateAngularDampingAttr(0.5)
        return prim

    def collide(prim, mat):
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(0.001)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), mats[mat], stage=stage)

    def cyl(path, r, z0, z1, color, body_z, opacity=1.0, collider=None, rough=0.4, metal=0.0):
        """A cylinder along the axis from z0 to z1 (syringe frame), drawn; with ``collider`` also a capsule (or a box
        when it is too short for one) of the same extent."""
        c = UsdGeom.Cylinder.Define(stage, path)
        c.CreateAxisAttr("Z")
        c.CreateRadiusAttr(r)
        c.CreateHeightAttr(z1 - z0)
        c.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, (z0 + z1) / 2 - body_z))
        bind_visual_material(str(c.GetPath()), look(color, opacity, rough, metal), stage=stage)
        if collider:
            h = (z1 - z0) - 2 * r
            if h > 0:
                cap = UsdGeom.Capsule.Define(stage, path + "_col")
                cap.CreateAxisAttr("Z")
                cap.CreateRadiusAttr(r)
                cap.CreateHeightAttr(h)
                cap.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, (z0 + z1) / 2 - body_z))
                prim = cap.GetPrim()
            else:
                cube = UsdGeom.Cube.Define(stage, path + "_col")
                cube.CreateSizeAttr(1.0)
                cube.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, (z0 + z1) / 2 - body_z))
                cube.AddScaleOp().Set(Gf.Vec3f(2 * r, 2 * r, z1 - z0))
                prim = cube.GetPrim()
            UsdGeom.Imageable(prim).MakeInvisible()
            collide(prim, collider)

    def box(path, centre, size, color, body_z, collider=None, opacity=1.0):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(centre[0], centre[1], centre[2] - body_z))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        bind_visual_material(str(cube.GetPath()), look(color, opacity), stage=stage)
        if collider:
            collide(cube.GetPrim(), collider)

    CLEAR, WHITE, GREY, STEEL = (0.92, 0.94, 0.96), (0.96, 0.96, 0.96), (0.22, 0.22, 0.24), (0.78, 0.79, 0.82)
    syr, carrier, needle, cap = (f"{prim_path}/{n}" for n in ("syringe", "carrier", "needle", "cap"))

    # the syringe: flange, clear barrel with the dose and the stopper inside, collar, plunger rod and thumb press
    body(syr, 0.0, spec.mass)
    box(f"{syr}/flange", (0.0, 0.0, FLANGE[2] / 2), FLANGE, CLEAR, 0.0, collider="barrel", opacity=0.85)
    cyl(f"{syr}/barrel", BARREL_R, *BARREL_Z, CLEAR, 0.0, opacity=0.35, collider="barrel", rough=0.1)
    cyl(f"{syr}/dose", BARREL_R - 0.0008, STOPPER_Z[1], BARREL_Z[1] - 0.0005, spec.liquid, 0.0, opacity=0.6)
    cyl(f"{syr}/stopper", BARREL_R - 0.0007, *STOPPER_Z, GREY, 0.0)
    cyl(f"{syr}/collar", COLLAR_R, *COLLAR_Z, CLEAR, 0.0, opacity=0.8, collider="barrel")
    box(f"{syr}/rod", (0.0, 0.0, (PLUNGER_Z[0] + STOPPER_Z[0]) / 2), (0.007, 0.007, STOPPER_Z[0] - PLUNGER_Z[0]),
        WHITE, 0.0)
    cyl(f"{syr}/thumb", THUMB_R, PLUNGER_Z[0] - 0.002, PLUNGER_Z[0], WHITE, 0.0)
    for i, zz in enumerate(np.arange(0.010, 0.0505, 0.004)):        # graduation marks along the top of the barrel
        box(f"{syr}/mark_{i}", (0.0, BARREL_R + 0.0001, float(zz)), (0.004 if i % 5 == 0 else 0.0025, 0.0002, 0.0004),
            GREY, 0.0)

    # the needle carrier (no shape), the needle (hub and shaft) and the cap
    body(carrier, LIFT_PIVOT, spec.carrier_mass)
    body(needle, LIFT_PIVOT, spec.needle_mass)
    cyl(f"{needle}/hub", HUB_R, *HUB_Z, spec.hub_color, LIFT_PIVOT, opacity=0.9, collider="barrel")
    cyl(f"{needle}/shaft", NEEDLE_R, *NEEDLE_Z, STEEL, LIFT_PIVOT, rough=0.2, metal=0.9)
    body(cap, LIFT_PIVOT, spec.cap_mass)
    cyl(f"{cap}/sheath", CAP_R, *CAP_Z, spec.hub_color, LIFT_PIVOT, opacity=0.75, collider="cap")
    for i in range(4):                                                 # the grip ribs
        a = i * math.pi / 2
        box(f"{cap}/rib_{i}", (CAP_R * math.cos(a), CAP_R * math.sin(a), CAP_Z[0] + 0.009), (0.0012, 0.0012, 0.014),
            spec.hub_color, LIFT_PIVOT)

    def joint(name, kind, b0, b1, pos0, lower, upper, stiffness=0.0, damping=0.0):
        j = (UsdPhysics.PrismaticJoint if kind == "linear" else UsdPhysics.RevoluteJoint).Define(stage, f"{b1}/{name}")
        j.CreateAxisAttr("Z")
        j.CreateBody0Rel().SetTargets([Sdf.Path(b0)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(b1)])
        j.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, pos0))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        if lower is not None:
            j.CreateLowerLimitAttr(float(lower))
            j.CreateUpperLimitAttr(float(upper))
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), kind)
        drive.CreateTypeAttr("force")
        per = 1.0 if kind == "linear" else math.radians(1.0)          # USD angular drives are per degree
        drive.CreateStiffnessAttr(float(stiffness) * per)
        drive.CreateDampingAttr(float(damping) * per)
        drive.CreateTargetPositionAttr(0.0)

    joint("lift", "linear", syr, carrier, LIFT_PIVOT, 0.0, 0.0)
    joint("spin", "angular", carrier, needle, 0.0, None, None, damping=spec.spin_damping)
    joint("cap", "linear", needle, cap, 0.0, 0.0, spec.cap_max, stiffness=spec.cap_spring, damping=0.5)
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(syr))
    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(syr))
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(8)
    return root.GetPrim()


def _spawn_free_needle(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """A loose capped needle (pxr only), shaped like a syringe's needle and cap at rest."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec: FreeNeedle = _FREE[cfg.needle_name]
    stage = get_current_stage()
    double = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))
    prim = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(spec.mass))
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateMaxDepenetrationVelocityAttr(0.5)
    rb.CreateAngularDampingAttr(0.5)
    mat = f"{prim_path}/Looks/phys_cap"
    pm = _material(sim_utils, *spec.cap_friction, "max")
    pm.func(mat, pm)
    looks = {}

    def look(color, opacity=1.0, rough=0.4, metal=0.0):
        key = (color, opacity, rough, metal)
        if key not in looks:
            path = f"{prim_path}/Looks/v{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough, metallic=metal,
                                              opacity=opacity)
            vis.func(path, vis)
            looks[key] = path
        return looks[key]

    def cyl(name, r, z0, z1, lk, collide=False):
        c = UsdGeom.Cylinder.Define(stage, f"{prim_path}/{name}")
        c.CreateAxisAttr("Z")
        c.CreateRadiusAttr(r)
        c.CreateHeightAttr(z1 - z0)
        c.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, (z0 + z1) / 2 - LIFT_PIVOT))
        bind_visual_material(str(c.GetPath()), lk, stage=stage)
        if collide:
            cap = UsdGeom.Capsule.Define(stage, f"{prim_path}/{name}_col")
            cap.CreateAxisAttr("Z")
            cap.CreateRadiusAttr(r)
            cap.CreateHeightAttr(max(0.0005, (z1 - z0) - 2 * r))
            cap.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, (z0 + z1) / 2 - LIFT_PIVOT))
            UsdGeom.Imageable(cap.GetPrim()).MakeInvisible()
            UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
            cp = PhysxSchema.PhysxCollisionAPI.Apply(cap.GetPrim())
            cp.CreateContactOffsetAttr(0.001)
            cp.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(cap.GetPath()), mat, stage=stage)

    cyl("hub", HUB_R, *HUB_Z, look(spec.hub_color, 0.9), collide=True)
    cyl("sheath", CAP_R, *CAP_Z, look(spec.hub_color, 0.75), collide=True)
    for i in range(4):                                                     # the cap's grip ribs, drawn
        a = i * math.pi / 2
        b = UsdGeom.Cube.Define(stage, f"{prim_path}/rib_{i}")
        b.CreateSizeAttr(1.0)
        b.AddTranslateOp().Set(Gf.Vec3d(CAP_R * math.cos(a), CAP_R * math.sin(a), CAP_Z[0] + 0.009 - LIFT_PIVOT))
        b.AddScaleOp().Set(Gf.Vec3f(0.0012, 0.0012, 0.014))
        bind_visual_material(str(b.GetPath()), look(spec.hub_color), stage=stage)
        UsdPhysics.CollisionAPI.Apply(b.GetPrim())                         # the ribs stop a dropped cap rolling
        cp = PhysxSchema.PhysxCollisionAPI.Apply(b.GetPrim())
        cp.CreateContactOffsetAttr(0.001)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(b.GetPath()), mat, stage=stage)
    return prim


def _free_needle_object(spec: FreeNeedle):
    from isaaclab.sim.spawners.spawner_cfg import RigidObjectSpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class FreeNeedleSpawnerCfg(RigidObjectSpawnerCfg):
        func: object = clone(_spawn_free_needle)
        needle_name: str = ""

    _FREE[spec.name] = spec
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=FreeNeedleSpawnerCfg(needle_name=spec.name),
                  initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                    rotation_xyzw=tuple(float(v) for v in spec.quat)))


def _syringe_object(spec: Syringe):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class SyringeSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_syringe)
        syringe_name: str = ""

    class KitObject(Object):
        def _generate_articulation_cfg(self):
            actuators = {"lift": ImplicitActuatorCfg(joint_names_expr=["lift"], stiffness=0.0, damping=0.0),
                         "spin": ImplicitActuatorCfg(joint_names_expr=["spin"], stiffness=0.0,
                                                     damping=spec.spin_damping),
                         "cap": ImplicitActuatorCfg(joint_names_expr=["cap"], stiffness=spec.cap_spring, damping=0.5)}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    return KitObject(name=spec.name, object_type=ObjectType.ARTICULATION,
                     spawner_cfg=SyringeSpawnerCfg(syringe_name=spec.name),
                     initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                       rotation_xyzw=tuple(float(v) for v in spec.quat)))


def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _syringe_state(environment, spec: Syringe) -> SyringeState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.root_link_pose_w)[0]
    vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    pos, quat = pose[:3] - origin, pose[3:7]
    names, q = list(asset.joint_names), _np(asset.data.joint_pos)[0]
    joints = {j: float(q[names.index(j)]) for j in JOINTS}
    half = np.array([FLANGE[0] / 2, FLANGE[0] / 2, 0.0])
    lo = np.array([-half[0], -half[1], PLUNGER_Z[0]])
    hi = np.array([half[0], half[1], CAP_Z[1]])
    corners = np.array([[a, b, c] for a in (lo[0], hi[0]) for b in (lo[1], hi[1]) for c in (lo[2], hi[2])])
    pts = corners @ _rotation(quat).T + pos
    return SyringeState(pos=pos, quat_xyzw=quat, lin_vel=vel[:3], ang_vel=vel[3:6], low=pts.min(0), high=pts.max(0),
                        joints=joints)


def write_joint(environment, name: str, joint: str, *, limits=None, stiffness=None, damping=None,
                position=None) -> None:
    """Change one joint of syringe ``name`` in the running simulation."""
    import torch

    asset = environment.unwrapped.scene[name]
    ids = [list(asset.joint_names).index(joint)]
    dev = _t(asset.data.joint_pos).device

    def call(stem, **kw):
        fn = getattr(asset, f"{stem}_index", None) or getattr(asset, stem)
        fn(**kw)

    if limits is not None:
        call("write_joint_position_limit_to_sim",
             limits=torch.tensor([[list(limits)]], dtype=torch.float32, device=dev), joint_ids=ids, env_ids=[0])
    if stiffness is not None:
        call("write_joint_stiffness_to_sim", stiffness=torch.full((1, 1), float(stiffness), device=dev),
             joint_ids=ids, env_ids=[0])
    if damping is not None:
        call("write_joint_damping_to_sim", damping=torch.full((1, 1), float(damping), device=dev),
             joint_ids=ids, env_ids=[0])
    if position is not None:
        pos = torch.full((1, 1), float(position), device=dev)
        call("write_joint_state_to_sim", position=pos, velocity=torch.zeros_like(pos), joint_ids=ids, env_ids=[0])


def needle_pose(environment, name: str):
    """The pose of syringe ``name``'s needle body (relative to the env origin): position and quaternion (xyzw)."""
    base = environment.unwrapped
    asset = base.scene[name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.body_link_pose_w)[0][list(asset.body_names).index("needle")]
    return pose[:3] - origin, pose[3:7]


def body_pose(environment, name: str):
    """A loose object's root pose: position (relative to the env origin) and quaternion (xyzw)."""
    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    pose = _np(base.scene[name].data.root_link_pose_w)[0]
    return pose[:3] - origin, pose[3:7]


def body_speed(environment, name: str) -> float:
    vel = _np(environment.unwrapped.scene[name].data.root_vel_w)[0]
    return float(np.linalg.norm(vel[:3]))


def write_pose(environment, name: str, pos, quat_xyzw) -> None:
    """Put a loose object at ``pos`` (relative to the env origin), ``quat_xyzw``, at rest."""
    import torch

    base = environment.unwrapped
    asset = base.scene[name]
    origin = _np(base.scene.env_origins[0])
    dev = _t(asset.data.root_link_pose_w).device
    pose = torch.tensor([[*(np.asarray(pos, float) + origin), *quat_xyzw]], dtype=torch.float32, device=dev)
    vel = torch.zeros((1, 6), dtype=torch.float32, device=dev)
    for stem, kw in (("write_root_pose_to_sim", {"root_pose": pose}),
                     ("write_root_velocity_to_sim", {"root_velocity": vel})):
        fn = getattr(asset, f"{stem}_index", None)
        if fn is not None:
            fn(env_ids=[0], **kw)
        else:
            getattr(asset, stem)(*kw.values())


def hold_root(environment, name: str):
    """Read syringe ``name``'s root pose; returns a function that writes it back (at rest). Moving a joint by hand
    shifts an articulation's root to keep its centre of mass; this undoes that."""
    import torch

    asset = environment.unwrapped.scene[name]
    pose = _t(asset.data.root_link_pose_w).clone()

    def restore() -> None:
        vel = torch.zeros((1, 6), dtype=pose.dtype, device=pose.device)
        for stem, kw in (("write_root_link_pose_to_sim", {"root_pose": pose}),
                         ("write_root_velocity_to_sim", {"root_velocity": vel})):
            fn = getattr(asset, f"{stem}_index", None)
            if fn is not None:
                fn(env_ids=[0], **kw)
            else:
                getattr(asset, stem)(*kw.values())

    return restore


_MASSES: dict = {}


def set_part_masses(environment, name: str, scale: float | None) -> None:
    """Scale the masses (and inertias) of syringe ``name``'s needle parts (carrier, needle, cap) by ``scale`` in the
    running simulation; ``None`` puts their own masses back. A needle part made almost massless can be moved inside
    the syringe without shifting it: an articulation keeps its centre of mass when a joint is moved by hand, so the
    shift scales with the moved mass."""
    import torch

    asset = environment.unwrapped.scene[name]
    if name not in _MASSES:
        _MASSES[name] = (_t(asset.data.body_mass).clone(), _t(asset.data.body_inertia).clone())
    m0, i0 = _MASSES[name]
    k = 1.0 if scale is None else float(scale)
    ids = [list(asset.body_names).index(part) for part in ("carrier", "needle", "cap")]
    body_ids = torch.tensor(ids, dtype=torch.int32, device=m0.device)
    env_ids = torch.tensor([0], dtype=torch.int32, device=m0.device)
    asset.set_masses_index(masses=m0[0:1, ids] * k, body_ids=body_ids, env_ids=env_ids)
    asset.set_inertias_index(inertias=i0[0:1, ids] * k, body_ids=body_ids, env_ids=env_ids)


def show_needle(name: str, visible: bool) -> None:
    """Show or hide syringe ``name``'s own needle and cap (hidden once a loose needle has taken their place)."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdGeom

    stage = get_current_stage()
    for part in ("needle", "cap"):
        prim = stage.GetPrimAtPath(f"{_PRIMS[name]}/{part}")
        if prim.IsValid():
            img = UsdGeom.Imageable(prim)
            img.MakeVisible() if visible else img.MakeInvisible()


def finger_points(environment) -> list:
    """The grasp point of each gripper (world, relative to the env origin)."""
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
    _STEP.append(fn)
    return fn


def on_reset(fn):
    _RESET.append(fn)
    return fn


def env():
    return _ENV["env"]


def _patch_scene(scene) -> None:
    if getattr(scene, "_syringekit_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Syringe):
            _SYRINGES[spec.name] = spec
            return _syringe_object(spec)
        if isinstance(spec, FreeNeedle):
            return _free_needle_object(spec)
        if isinstance(spec, Fixed):
            return _fixed_object(scene, spec)
        return stock_object(spec)

    def object_states(environment, names):
        plain = [n for n in names if n not in _SYRINGES]
        out = stock_states(environment, plain) if plain else {}
        for n in names:
            if n in _SYRINGES:
                out[n] = _syringe_state(environment, _SYRINGES[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._syringekit_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_syringekit_patched", False):
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
    cls._syringekit_patched = True


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
