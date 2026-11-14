"""bottlekit: a swing-top bottle (porcelain stopper on a wire bail, closed by an over-centre lever), and fixed parts
with their own friction, for yam_arena task files.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from bottlekit import Bottle, Fixed, on_reset, on_step

- :class:`Fixed` is a fixed box with its own friction and look.
- :class:`Bottle` is a loose 200 mL swing-top glass bottle: one articulation of the bottle (the root, free on the
  table), the stopper and the lever. In the bottle's frame z is up from the middle of its base, and both the stopper and
  the lever turn about the x axis through the wire's pivot on the neck.

  ``stopper`` joint: 0 seated on the mouth, positive swung over toward -y (it rests out beside the neck at STOPPER_OPEN).
  ``lever`` joint: 0 closed (lying down the neck's -y side), negative swung back over the top toward +y (LEVER_OPEN).

  The bail's over-centre action is modelled in :func:`_mechanism`, run every control step: with the stopper seated,
  closing the lever stretches the bail, so from LEVER_TIGHT the lever is sprung back open, harder the further it goes;
  past LEVER_CENTRE it snaps closed instead, and the bail clamps the stopper down (``sealed``). Let go short of the
  centre and it springs back open. With the stopper not on the mouth the lever swings freely and seals nothing.

``on_step`` and ``on_reset`` register callbacks ``fn(env)``.

The structure follows the kits used by tasks already in this repository (Box subclasses so the loader accepts the
parts, custom spawners, hooks installed through ``sys.meta_path``). Pure Python at import time; Isaac Sim is reached
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

__all__ = ["Bottle", "Fixed", "KitState", "body_pose", "env", "finger_points", "install", "on_reset", "on_step",
           "paddle_point", "sealed", "stopper_point"]

# -- a 200 mL swing-top bottle (m), in its own frame ---------------------------------------------------------------------
BODY_R, BODY_H = 0.028, 0.085                    # the cylindrical body: radius, height of the shoulder's start
SHOULDER = ((0.0255, 0.085, 0.093), (0.0210, 0.093, 0.101), (0.0165, 0.101, 0.108))   # (radius, z0, z1) steps
NECK_R, NECK_Z = 0.0125, (0.108, 0.143)
LIP_R, MOUTH_Z = 0.0140, 0.150                   # the lip round the mouth; the mouth's rim
PIVOT_Z = 0.125                                  # the bail wire's pivot on the neck (the joints' axis, along x)
PIVOT_X = 0.0155                                 # its two ends, either side of the neck
STOPPER_R, STOPPER_H = 0.0115, 0.014             # the porcelain stopper (its rubber washer under it)
STOPPER_L = MOUTH_Z + STOPPER_H / 2 - PIVOT_Z    # pivot to the seated stopper's centre
LEVER_DIR = math.radians(45.0)                   # the closed lever leans out from the neck by this much
LEVER_L = 0.050                                  # pivot to the paddle's centre
PADDLE = (0.018, 0.006, 0.022)                   # the lever's thumb paddle: across (x), thick, long
STOPPER_OPEN = math.radians(95.0)                # out beside the neck, against the bail's stop
LEVER_OPEN = math.radians(-212.0)                # flipped back over the top, lying out toward +y
LEVER_TIGHT = math.radians(-75.0)                # with the stopper seated, the bail starts to stretch here
LEVER_CENTRE = math.radians(-35.0)               # the over-centre point
SEATED = math.radians(5.0)                       # the stopper within this of its seat is on the mouth
BAIL_K = 0.30                                    # N m/rad: the stretched bail's spring (0.21 N m at the centre)


def lever_dir(phi: float) -> np.ndarray:
    """The lever's direction from the pivot (bottle frame) at joint angle ``phi``."""
    d0 = np.array([0.0, -math.sin(LEVER_DIR), -math.cos(LEVER_DIR)])
    c, s = math.cos(phi), math.sin(phi)
    return np.array([0.0, d0[1] * c - d0[2] * s, d0[1] * s + d0[2] * c])


def stopper_offset(theta: float) -> np.ndarray:
    """The stopper's centre from the pivot (bottle frame) at joint angle ``theta``."""
    return np.array([0.0, -STOPPER_L * math.sin(theta), STOPPER_L * math.cos(theta)])


@dataclass(frozen=True)
class Fixed(Box):
    """A fixed task-file ``Box`` with its own friction and look."""

    static_friction: float = 0.5
    dynamic_friction: float = 0.4
    roughness: float = 0.6
    metallic: float = 0.0
    quat: tuple = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class Bottle(Box):
    """A swing-top bottle. ``pos`` is the middle of its base, ``quat`` (xyzw) its orientation, ``mass`` the bottle
    with its contents (the stopper and lever are 18 g and 12 g more)."""

    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    glass: tuple = (0.20, 0.45, 0.28)
    friction: tuple = (0.6, 0.5)
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")
        if self.mass is None or self.mass <= 0:
            raise ValueError(f"{self.name}: a Bottle must be loose (give it a mass)")


@dataclass(frozen=True)
class KitState(ObjectState):
    """A kit object as the simulator has it; ``joints`` maps the bottle's joints to their angles (rad)."""

    joints: dict = field(default_factory=dict)


def _rotation(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _material(sim_utils, sf, df, combine="average"):
    try:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df, friction_combine_mode=combine)
    except TypeError:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df)


def _qx(angle):
    return (math.cos(angle / 2), math.sin(angle / 2), 0.0, 0.0)      # wxyz


# -- authoring helpers --------------------------------------------------------------------------------------------------

class _Author:
    def __init__(self, prim_path, translation, orientation):
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, UsdGeom

        self.sim_utils, self.Gf, self.UsdGeom = sim_utils, Gf, UsdGeom
        self.stage = get_current_stage()
        self.path = prim_path
        double = UsdGeom.XformOp.PrecisionDouble
        root = UsdGeom.Xform.Define(self.stage, prim_path)
        root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
        x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
        root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))
        self.root = root.GetPrim()
        self.looks, self.mats = {}, {}

    def look(self, color, rough=0.5, metal=0.0, opacity=1.0) -> str:
        from yam_arena.station import linear_rgb

        key = (tuple(color), rough, metal, opacity)
        if key not in self.looks:
            path = f"{self.path}/Looks/v{len(self.looks)}"
            vis = self.sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough, metallic=metal,
                                                   opacity=opacity)
            vis.func(path, vis)
            self.looks[key] = path
        return self.looks[key]

    def mat(self, key, sf, df) -> str:
        if key not in self.mats:
            path = f"{self.path}/Looks/phys_{key}"
            pm = _material(self.sim_utils, sf, df)
            pm.func(path, pm)
            self.mats[key] = path
        return self.mats[key]

    def body(self, path, pos, mass, lin_damp=0.05, ang_damp=0.1):
        from pxr import PhysxSchema, UsdPhysics

        xf = self.UsdGeom.Xform.Define(self.stage, path)
        if path != self.path and pos is not None:
            xf.AddTranslateOp(precision=self.UsdGeom.XformOp.PrecisionDouble).Set(self.Gf.Vec3d(*pos))
            xf.AddOrientOp(precision=self.UsdGeom.XformOp.PrecisionDouble).Set(self.Gf.Quatd(1, 0, 0, 0))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.3)
        rb.CreateLinearDampingAttr(lin_damp)
        rb.CreateAngularDampingAttr(ang_damp)
        return prim

    def collide(self, prim, mat, offset=0.0008):
        from isaaclab.sim.utils import bind_physics_material
        from pxr import PhysxSchema, UsdPhysics

        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(offset)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), mat, stage=self.stage)

    def paint(self, prim, look):
        from isaaclab.sim.utils import bind_visual_material

        bind_visual_material(str(prim.GetPath()), look, stage=self.stage)

    def box(self, path, centre, size, look, mat=None, quat_wxyz=None):
        cube = self.UsdGeom.Cube.Define(self.stage, path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(self.Gf.Vec3d(*(float(v) for v in centre)))
        if quat_wxyz is not None:
            cube.AddOrientOp().Set(self.Gf.Quatf(*(float(v) for v in quat_wxyz)))
        cube.AddScaleOp().Set(self.Gf.Vec3f(*(float(v) for v in size)))
        if look:
            self.paint(cube.GetPrim(), look)
        if mat:
            self.collide(cube.GetPrim(), mat)
        if not look:
            self.UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()
        return cube.GetPrim()

    def cyl_y(self, path, r, centre, look, length=0.0001):
        """A cylinder along y (drawn only)."""
        c = self.UsdGeom.Cylinder.Define(self.stage, path)
        c.CreateAxisAttr("Y")
        c.CreateRadiusAttr(float(r))
        c.CreateHeightAttr(float(length))
        c.AddTranslateOp().Set(self.Gf.Vec3d(*(float(v) for v in centre)))
        self.paint(c.GetPrim(), look)
        return c.GetPrim()

    def cyl(self, path, r, z0, z1, look, mat=None, centre=(0.0, 0.0)):
        c = self.UsdGeom.Cylinder.Define(self.stage, path)
        c.CreateAxisAttr("Z")
        c.CreateRadiusAttr(float(r))
        c.CreateHeightAttr(float(z1 - z0))
        c.AddTranslateOp().Set(self.Gf.Vec3d(float(centre[0]), float(centre[1]), float(z0 + z1) / 2))
        self.paint(c.GetPrim(), look)
        if mat:
            self.collide(c.GetPrim(), mat)
        return c.GetPrim()


# -- spawning ------------------------------------------------------------------------------------------------------------

_SPECS: dict = {}


def _spawn_bottle(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """The bottle (the articulation's root), and the stopper and lever, each on a revolute joint about x at the pivot.
    The stopper and lever are authored closed; their bodies' origins are at the pivot."""
    from pxr import Gf, PhysxSchema, Sdf, UsdPhysics

    spec: Bottle = _SPECS[cfg.kit_name]
    a = _Author(prim_path, translation, orientation)
    glass = a.look(spec.glass, rough=0.08, metal=0.0, opacity=0.85)
    drink = a.look((0.95, 0.80, 0.30), rough=0.3)
    label = a.look((0.96, 0.93, 0.84), rough=0.7)
    ink = a.look((0.75, 0.20, 0.15), rough=0.6)
    china = a.look((0.97, 0.97, 0.95), rough=0.25)
    rubber = a.look((0.85, 0.30, 0.10), rough=0.9)
    steel = a.look((0.78, 0.79, 0.81), rough=0.25, metal=0.9)
    m_glass = a.mat("glass", *spec.friction)
    m_china = a.mat("china", 0.8, 0.7)
    m_lever = a.mat("lever", 0.8, 0.7)

    bottle = f"{prim_path}/bottle"
    a.body(bottle, (0.0, 0.0, 0.0), spec.mass, ang_damp=0.05)
    a.cyl(f"{bottle}/body", BODY_R, 0.0, BODY_H, glass, m_glass)
    for i, (r, z0, z1) in enumerate(SHOULDER):
        a.cyl(f"{bottle}/shoulder_{i}", r, z0, z1, glass, m_glass)
    a.cyl(f"{bottle}/neck", NECK_R, NECK_Z[0], NECK_Z[1], glass, m_glass)
    a.cyl(f"{bottle}/lip", LIP_R, NECK_Z[1], MOUTH_Z, glass, m_glass)
    a.cyl(f"{bottle}/drink", BODY_R - 0.0025, 0.003, BODY_H - 0.004, drink)              # the lemonade inside
    a.cyl(f"{bottle}/label", BODY_R + 0.0004, 0.040, 0.090, label)
    a.cyl(f"{bottle}/label_band", BODY_R + 0.0006, 0.058, 0.072, ink)
    a.cyl(f"{bottle}/neck_wire", NECK_R + 0.0012, PIVOT_Z - 0.0012, PIVOT_Z + 0.0012, steel)
    for sx in (-1, 1):
        a.box(f"{bottle}/pivot_{'ab'[sx > 0]}", (sx * PIVOT_X, 0.0, PIVOT_Z), (0.004, 0.004, 0.004), steel)

    stopper = f"{prim_path}/stopper"
    a.body(stopper, (0.0, 0.0, PIVOT_Z), 0.018, ang_damp=0.5)
    s0 = MOUTH_Z - PIVOT_Z
    a.cyl(f"{stopper}/china", STOPPER_R, s0 + 0.002, s0 + STOPPER_H, china, m_china)
    a.cyl(f"{stopper}/washer", STOPPER_R - 0.0008, s0, s0 + 0.002, rubber)
    for sx in (-1, 1):                                                    # the bail: a wire loop over the stopper
        wz = (s0 + STOPPER_H * 0.6) / 2
        ang = math.atan2(PIVOT_X - STOPPER_R, s0 + STOPPER_H * 0.6)
        a.box(f"{stopper}/bail_{'ab'[sx > 0]}", (sx * (PIVOT_X + STOPPER_R) / 2, 0.0, wz),
              (0.0016, 0.0016, s0 + STOPPER_H * 0.6), steel, quat_wxyz=(math.cos(-sx * ang / 2), 0.0,
                                                                         math.sin(-sx * ang / 2), 0.0))
    a.box(f"{stopper}/bail_top", (0.0, 0.0, s0 + STOPPER_H + 0.0008), (2 * STOPPER_R, 0.0016, 0.0016), steel)

    lever = f"{prim_path}/lever"
    a.body(lever, (0.0, 0.0, PIVOT_Z), 0.012, ang_damp=0.5)
    d0 = lever_dir(0.0)
    tilt = math.pi - LEVER_DIR                                            # local z onto the lever's direction
    q = _qx(tilt)
    for sx in (-1, 1):
        c = d0 * (LEVER_L - PADDLE[2] / 2) / 2
        a.box(f"{lever}/arm_{'ab'[sx > 0]}", (sx * PIVOT_X, c[1], c[2]), (0.0018, 0.0018, LEVER_L - PADDLE[2] / 2),
              steel, quat_wxyz=q)
        a.box(f"{lever}/link_{'ab'[sx > 0]}", (sx * (PIVOT_X + PADDLE[0] / 2) / 2,) + tuple(d0[1:] * (LEVER_L - PADDLE[2] / 2)),
              (PIVOT_X - PADDLE[0] / 2 + 0.002, 0.0018, 0.0018), steel, quat_wxyz=q)
    pc = d0 * LEVER_L
    a.box(f"{lever}/paddle", (0.0, pc[1], pc[2]), PADDLE, steel, m_lever, quat_wxyz=q)

    stage = a.stage
    for name, lo, hi in (("stopper", -1.0, math.degrees(STOPPER_OPEN)),
                         ("lever", math.degrees(LEVER_OPEN) - 3.0, 0.0)):
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/{name}/{name}")
        j.CreateAxisAttr("X")
        j.CreateBody0Rel().SetTargets([Sdf.Path(bottle)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/{name}")])
        j.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, PIVOT_Z))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLowerLimitAttr(float(lo))
        j.CreateUpperLimitAttr(float(hi))
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(float(BAIL_K) * math.radians(1.0))   # per degree
        drive.CreateDampingAttr(0.002 * math.radians(1.0))
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(bottle))
    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(bottle))
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(8)
    return a.root


def _bottle_object(spec: Bottle):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class BottleSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_bottle)
        kit_name: str = ""

    class KitObject(Object):
        def _generate_articulation_cfg(self):
            actuators = {"bail": ImplicitActuatorCfg(joint_names_expr=["stopper", "lever"], stiffness=BAIL_K,
                                                     damping=0.002)}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            cfg = self._add_initial_pose_to_cfg(cfg)
            cfg.init_state.joint_pos = {"stopper": float(STOPPER_OPEN), "lever": float(LEVER_OPEN)}
            return cfg

    _SPECS[spec.name] = spec
    return KitObject(name=spec.name, object_type=ObjectType.ARTICULATION,
                     spawner_cfg=BottleSpawnerCfg(kit_name=spec.name),
                     initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                       rotation_xyzw=tuple(float(v) for v in spec.quat)))


def _fixed_object(scene, spec: Fixed):
    sim_utils = scene.sim_utils
    look = sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(spec.color), roughness=spec.roughness,
                                       metallic=spec.metallic)
    cfg = sim_utils.CuboidCfg(size=tuple(spec.size),
                              physics_material=_material(sim_utils, spec.static_friction, spec.dynamic_friction),
                              visual_material=look, collision_props=scene.PhysxCollisionPropertiesCfg())
    return scene.Object(name=spec.name, object_type=scene.ObjectType.BASE, spawner_cfg=cfg,
                        initial_pose=scene.Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                                rotation_xyzw=tuple(float(v) for v in spec.quat)))



# -- run time ------------------------------------------------------------------------------------------------------------

def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    if hasattr(x, "numpy") and not hasattr(x, "detach"):
        return np.asarray(x.numpy(), dtype=np.float64)
    return _t(x).detach().cpu().numpy().astype(np.float64)


def body_pose(environment, name: str):
    """A loose object's root pose: position (relative to the env origin) and quaternion (xyzw)."""
    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    pose = _np(base.scene[name].data.root_link_pose_w)[0]
    return pose[:3] - origin, pose[3:7]


def write_pose(environment, name: str, pos, quat_xyzw) -> None:
    """Put a loose object at ``pos`` (relative to the env origin), ``quat_xyzw``, at rest."""
    import torch

    base = environment.unwrapped
    asset = base.scene[name]
    origin = _np(base.scene.env_origins[0])
    dev = _t(asset.data.root_link_pose_w).device
    pose = torch.tensor([[*(np.asarray(pos, float) + origin), *quat_xyzw]], dtype=torch.float32, device=dev)
    vel = torch.zeros((1, 6), dtype=torch.float32, device=dev)
    for stem, kw in (("write_root_pose_to_sim", {"root_pose": pose}), ("write_root_velocity_to_sim", {"root_velocity": vel})):
        fn = getattr(asset, f"{stem}_index", None)
        if fn is not None:
            fn(env_ids=[0], **kw)
        else:
            getattr(asset, stem)(*kw.values())


def write_joint(environment, name: str, joint: str, *, limits=None, stiffness=None, position=None) -> None:
    """Change one joint of case ``name`` in the running simulation."""
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
    if position is not None:
        pos = torch.full((1, 1), float(position), device=dev)
        call("write_joint_state_to_sim", position=pos, velocity=torch.zeros_like(pos), joint_ids=ids, env_ids=[0])


def _joints(environment, name: str) -> dict:
    asset = environment.unwrapped.scene[name]
    names, q = list(asset.joint_names), _np(asset.data.joint_pos)[0]
    return {n: float(q[names.index(n)]) for n in names}


_MECH: dict = {}


def sealed(name: str) -> bool:
    """Whether the bottle's lever has been snapped over its centre with the stopper on the mouth (and is still over)."""
    return bool(_MECH.get(name, {}).get("sealed", False))


def bail_targets(theta: float, phi: float, was_sealed: bool, was_over: bool):
    """The bail's action at joint angles ``theta`` (stopper) and ``phi`` (lever), given whether the bottle was sealed
    and the lever over its centre at the last step: the drive targets for the stopper and the lever (None: follow the
    joint, no spring), and whether the bottle is now sealed. Pure: tested offline.

    The seal forms only as the lever crosses its centre with the stopper on the mouth (the bail pulls it down); a
    stopper put on the mouth after the lever is already shut is not clamped, as on a real bottle."""
    seated = theta < SEATED
    over = phi > LEVER_CENTRE
    now_sealed = over and (was_sealed or (seated and not was_over))
    if now_sealed:
        return -math.radians(3.0), math.radians(3.0), True         # clamped: both pressed onto their stops
    if seated and phi > LEVER_TIGHT:
        return None, LEVER_TIGHT, False                             # stretching the bail: sprung back open
    return None, None, False                                        # loose: both swing freely


def _mechanism(environment) -> None:
    import torch

    for name, spec in _SPECS.items():
        if not isinstance(spec, Bottle):
            continue
        st = _MECH.setdefault(name, {"sealed": False, "over": False, "settle": SETTLE_STEPS})
        if st.get("settle", 0) > 0:                                       # just reset: hold it open, where it stands
            st["settle"] -= 1
            _set_open(environment, name, spec)
            continue
        asset = environment.unwrapped.scene[name]
        names = list(asset.joint_names)
        q = _np(asset.data.joint_pos)[0]
        theta, phi = float(q[names.index("stopper")]), float(q[names.index("lever")])
        t_stop, t_lever, st["sealed"] = bail_targets(theta, phi, st["sealed"], st["over"])
        st["over"] = phi > LEVER_CENTRE
        tgt = [theta if t_stop is None else t_stop, phi if t_lever is None else t_lever]
        ids = [names.index("stopper"), names.index("lever")]
        dev = _t(asset.data.joint_pos).device
        fn = getattr(asset, "set_joint_position_target_index", None)
        target = torch.tensor([tgt], dtype=torch.float32, device=dev)
        if fn is not None:
            fn(target=target, joint_ids=ids, env_ids=[0])
        else:
            asset.set_joint_position_target(target, joint_ids=ids)


SETTLE_STEPS = 10                               # after a reset, the bottle's pose is held while its joints are set


def _set_open(environment, name: str, spec) -> None:
    """Put the stopper and lever at their open angles, at rest, and the bottle back where it stands: writing joint
    positions moves an articulation's root, so the root pose is written again after them."""
    import torch

    asset = environment.unwrapped.scene[name]
    names = list(asset.joint_names)
    ids = [names.index("stopper"), names.index("lever")]
    dev = _t(asset.data.joint_pos).device
    pos = torch.tensor([[float(STOPPER_OPEN), float(LEVER_OPEN)]], dtype=torch.float32, device=dev)
    fn = getattr(asset, "write_joint_state_to_sim_index", None)
    if fn is not None:
        fn(position=pos, velocity=torch.zeros_like(pos), joint_ids=ids, env_ids=[0])
    else:
        asset.write_joint_state_to_sim(pos, torch.zeros_like(pos), joint_ids=ids)
    tfn = getattr(asset, "set_joint_position_target_index", None)
    if tfn is not None:
        tfn(target=pos, joint_ids=ids, env_ids=[0])
    else:
        asset.set_joint_position_target(pos, joint_ids=ids)
    write_pose(environment, name, spec.pos, spec.quat)


def _mechanism_reset(environment) -> None:
    _MECH.clear()
    for name, spec in _SPECS.items():
        if isinstance(spec, Bottle):
            _set_open(environment, name, spec)
            _MECH[name] = {"sealed": False, "over": False, "settle": SETTLE_STEPS}


def _kit_state(environment, spec) -> KitState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.root_link_pose_w)[0]
    pos, quat = pose[:3] - origin, pose[3:7]
    vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    joints = _joints(environment, spec.name)
    lo, hi = np.array([-BODY_R, -BODY_R, 0.0]), np.array([BODY_R, BODY_R, MOUTH_Z])
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    pts = corners @ _rotation(quat).T + pos
    return KitState(pos=pos, quat_xyzw=quat, lin_vel=vel[:3], ang_vel=vel[3:6], low=pts.min(0), high=pts.max(0),
                    joints=joints)


def stopper_point(pos, quat, theta: float, out: float = 0.0) -> np.ndarray:
    """The stopper's centre (world, relative to the env origin) for a bottle at ``pos``, ``quat``; ``out`` moves the
    point along the stopper's axis, away from the pivot."""
    off = stopper_offset(theta) * (STOPPER_L + out) / STOPPER_L
    return np.asarray(pos, float) + _rotation(quat) @ (np.array([0.0, 0.0, PIVOT_Z]) + off)


def paddle_point(pos, quat, phi: float, along: float = LEVER_L) -> np.ndarray:
    """The lever's paddle centre (world) for a bottle at ``pos``, ``quat``."""
    return np.asarray(pos, float) + _rotation(quat) @ (np.array([0.0, 0.0, PIVOT_Z]) + lever_dir(phi) * along)


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
    if getattr(scene, "_bottlekit_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Bottle):
            return _bottle_object(spec)
        if isinstance(spec, Fixed):
            return _fixed_object(scene, spec)
        return stock_object(spec)

    def object_states(environment, names):
        mine = {n for n in names if isinstance(_SPECS.get(n), Bottle)}
        plain = [n for n in names if n not in mine]
        out = stock_states(environment, plain) if plain else {}
        for n in mine:
            out[n] = _kit_state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._bottlekit_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_bottlekit_patched", False):
        return
    stock_step, stock_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        _mechanism(self._env)
        for fn in _STEP:
            fn(self._env)
        return stock_step(self, *a, **k)

    def reset(self, *a, **k):
        obs = stock_reset(self, *a, **k)
        _ENV["env"] = self._env
        _mechanism_reset(self._env)
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._bottlekit_patched = True


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
