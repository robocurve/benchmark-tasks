"""pegkit: wooden clothes pegs, a photo print, and fixed parts with their own friction, for yam_arena task files.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from pegkit import Fixed, Peg, Photo, body_pose, on_reset, on_step

- :class:`Fixed` is a fixed box with its own friction and look (the line, its posts, the photo holder and the peg
  basket are built from these).
- :class:`Photo` is a loose photo print: a thin card with a printed picture drawn on its front.
- :class:`Peg` is a soft-grip wooden clothes peg (rubber pads on the outsides of its handles): one articulation of two legs on a pivot with a torsion spring. In the peg's
  frame the legs run along +z from the jaw tips (z = 0) to the handle ends, and part along x. The ``spring`` joint
  turns the second leg about the pivot; squeezing the handles together opens the jaws (the joint goes negative), and
  the spring closes them again on whatever is between them.

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

__all__ = ["Fixed", "KitState", "Peg", "Photo", "body_pose", "env", "finger_points", "install", "on_reset", "on_step",
           "write_joint", "write_pose"]

# -- a wooden clothes peg (m), in its own frame ------------------------------------------------------------------------
PEG_LEN, PEG_W, LEG_T = 0.072, 0.010, 0.005     # long, wide (along y), each leg's thickness (along x)
PIVOT_Z = 0.030                                 # the spring's pivot, this far up from the jaw tips
GAP = 0.0008                                    # between the jaws' faces, closed on nothing
SPLAY = math.radians(8.0)                       # each handle angles out this much from its jaw
OPEN_MAX = math.radians(18.0)                   # the second leg turns at most this far (handles meeting)
NOTCH_Z = (0.005, 0.014)                        # the jaws' groove for the line, up from the tips


@dataclass(frozen=True)
class Fixed(Box):
    """A fixed task-file ``Box`` with its own friction and look."""

    static_friction: float = 0.5
    dynamic_friction: float = 0.4
    roughness: float = 0.6
    metallic: float = 0.0
    quat: tuple = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class Photo(Box):
    """A loose photo print: ``size`` (wide, tall, thick) in its frame's x, z and y; the picture on its -y face.
    ``pos`` is its centre, ``quat`` (xyzw) its orientation."""

    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    friction: tuple = (0.5, 0.4)
    meta: dict = field(default_factory=dict, compare=False, hash=False)


@dataclass(frozen=True)
class Peg(Box):
    """A wooden clothes peg. ``pos`` is the pivot's middle, ``quat`` (xyzw) its orientation. ``spring`` (N m/rad) and
    ``preload`` (rad) set the torsion spring: closed on nothing it presses the jaws together with
    spring * preload / PIVOT_Z newtons."""

    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    spring: float = 0.25
    preload: float = math.radians(25.0)
    open0: float = 0.0                      # the spring joint's angle at the start (negative: jaws open on something)
    friction: tuple = (0.6, 0.5)
    wood: tuple = (0.80, 0.66, 0.46)
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")
        if self.mass is None or self.mass <= 0:
            raise ValueError(f"{self.name}: a Peg must be loose (give it a mass)")


@dataclass(frozen=True)
class KitState(ObjectState):
    """A kit object as the simulator has it; ``joints`` maps a peg's ``spring`` joint to its angle (rad, negative
    open)."""

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


def _qz(angle):
    return (math.cos(angle / 2), 0.0, 0.0, math.sin(angle / 2))      # wxyz


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


def _spawn_photo(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """A photo print: one thin box collider, the picture drawn as flat boxes on its front (-y) face."""
    spec: Photo = _SPECS[cfg.kit_name]
    a = _Author(prim_path, translation, orientation)
    a.body(prim_path, None, spec.mass, ang_damp=0.3)
    m = a.mat("card", *spec.friction)
    w, t, h = spec.size[0], spec.size[1], spec.size[2]
    a.box(f"{prim_path}/card", (0.0, 0.0, 0.0), (w, t, h), a.look((0.96, 0.96, 0.94), rough=0.3), m)
    f = -t / 2 - 0.0001                                                   # the printed face
    b = 0.004                                                             # white border
    a.box(f"{prim_path}/sky", (0.0, f, h * 0.18), (w - 2 * b, 0.0001, h * 0.64 - b), a.look((0.45, 0.70, 0.95), 0.4))
    a.box(f"{prim_path}/sea", (0.0, f - 0.00001, -h * 0.17), (w - 2 * b, 0.0001, h * 0.10), a.look((0.10, 0.40, 0.70), 0.3))
    a.box(f"{prim_path}/sand", (0.0, f - 0.00002, -h * 0.34), (w - 2 * b, 0.0001, h * 0.32 - 2 * b),
          a.look((0.92, 0.82, 0.55), 0.6))
    a.cyl_y(f"{prim_path}/sun", 0.011, (w * 0.28, f - 0.00003, h * 0.30), a.look((1.0, 0.85, 0.25), 0.4))
    for i, (x, z, s) in enumerate(((-0.030, -0.005, 0.016), (-0.012, 0.002, 0.020))):   # two sailing boats
        a.box(f"{prim_path}/sail_{i}", (x, f - 0.00004, z + 0.012), (s * 0.5, 0.0001, s), a.look((0.98, 0.98, 0.98), 0.4))
        a.box(f"{prim_path}/hull_{i}", (x, f - 0.00004, z), (s, 0.0001, s * 0.25), a.look((0.70, 0.20, 0.15), 0.5))
    return a.root


def _spawn_peg(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """The peg: leg_a (the articulation's root) and leg_b on a revolute ``spring`` joint at the pivot. Bodies are
    authored relative to the pivot's middle, which is where the peg's pose is given."""
    from pxr import Gf, PhysxSchema, Sdf, UsdPhysics

    spec: Peg = _SPECS[cfg.kit_name]
    a = _Author(prim_path, translation, orientation)
    wood = a.look(spec.wood, rough=0.75)
    steel = a.look((0.75, 0.76, 0.78), rough=0.25, metal=0.9)
    m = a.mat("wood", *spec.friction)
    rubber = a.look((0.15, 0.55, 0.35), rough=0.95)
    grip = a.mat("grip", 1.1, 1.0)
    legs = {}
    for side, name in ((-1, "leg_a"), (1, "leg_b")):
        path = f"{prim_path}/{name}"
        a.body(path, (0.0, 0.0, 0.0), spec.mass / 2)
        xc = side * (GAP / 2 + LEG_T / 2)                                 # leg's mid-line, x
        jaw_lo, jaw_hi = 0.0, PIVOT_Z
        # the jaw: below the pivot, straight, with a groove for the line (two blocks either side of it)
        g0, g1 = NOTCH_Z
        groove = 0.0015
        for k, (z0, z1, inset) in enumerate(((jaw_lo, g0, 0.0), (g0, g1, groove), (g1, jaw_hi, 0.0))):
            a.box(f"{path}/jaw_{k}", (xc + side * inset / 2, 0.0, (z0 + z1) / 2 - PIVOT_Z),
                  (LEG_T - inset, PEG_W, z1 - z0), wood, m)
        # the handle: above the pivot, angled out by SPLAY
        hl = PEG_LEN - PIVOT_Z
        cx = xc + side * math.sin(SPLAY) * hl / 2
        cz = math.cos(SPLAY) * hl / 2
        q = (math.cos(-side * SPLAY / 2), 0.0, math.sin(-side * SPLAY / 2), 0.0)
        a.box(f"{path}/handle", (cx, 0.0, cz), (LEG_T, PEG_W, hl), wood, m, quat_wxyz=q)
        # the soft grip: a rubber pad on the outside of the handle, where fingers squeeze
        gl = hl * 0.6
        gx = xc + side * (math.sin(SPLAY) * (hl * 0.55) + LEG_T / 2 + 0.0006)
        a.box(f"{path}/grip", (gx, 0.0, math.cos(SPLAY) * hl * 0.55), (0.0012, PEG_W * 0.9, gl), rubber, grip,
              quat_wxyz=q)
        legs[name] = path
    a.cyl_y(f"{legs['leg_a']}/spring", 0.0032, (0.0, 0.0, 0.0), steel, length=PEG_W + 0.003)

    stage = a.stage
    j = UsdPhysics.RevoluteJoint.Define(stage, f"{legs['leg_b']}/spring")
    j.CreateAxisAttr("Y")
    j.CreateBody0Rel().SetTargets([Sdf.Path(legs["leg_a"])])
    j.CreateBody1Rel().SetTargets([Sdf.Path(legs["leg_b"])])
    j.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLowerLimitAttr(float(-math.degrees(OPEN_MAX)))
    j.CreateUpperLimitAttr(0.0)
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(float(spec.spring) * math.radians(1.0))   # per degree
    drive.CreateDampingAttr(float(spec.spring) * 0.02 * math.radians(1.0))
    drive.CreateTargetPositionAttr(float(math.degrees(spec.preload)))
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(legs["leg_a"]))
    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(legs["leg_a"]))
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(8)
    return a.root


def _rigid_object(spec, spawn):
    from isaaclab.sim.spawners.spawner_cfg import RigidObjectSpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class KitSpawnerCfg(RigidObjectSpawnerCfg):
        func: object = clone(spawn)
        kit_name: str = ""

    _SPECS[spec.name] = spec
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=KitSpawnerCfg(kit_name=spec.name),
                  initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                    rotation_xyzw=tuple(float(v) for v in spec.quat)))


def _peg_object(spec: Peg):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class PegSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_peg)
        kit_name: str = ""

    class KitObject(Object):
        def _generate_articulation_cfg(self):
            actuators = {"spring": ImplicitActuatorCfg(joint_names_expr=["spring"], stiffness=spec.spring,
                                                       damping=spec.spring * 0.02)}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            cfg = self._add_initial_pose_to_cfg(cfg)
            cfg.init_state.joint_pos = {"spring": float(spec.open0)}
            return cfg

    _SPECS[spec.name] = spec
    return KitObject(name=spec.name, object_type=ObjectType.ARTICULATION, spawner_cfg=PegSpawnerCfg(kit_name=spec.name),
                     initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos),
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


def _kit_state(environment, spec) -> KitState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.root_link_pose_w)[0]
    pos, quat = pose[:3] - origin, pose[3:7]
    vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    joints = {}
    if isinstance(spec, Peg):
        names, q = list(asset.joint_names), _np(asset.data.joint_pos)[0]
        joints = {n: float(q[names.index(n)]) for n in names}
    half = np.asarray(spec.size, float) / 2
    if isinstance(spec, Peg):
        lo, hi = np.array([-0.012, -PEG_W / 2, -PIVOT_Z]), np.array([0.012, PEG_W / 2, PEG_LEN - PIVOT_Z])
    else:
        lo, hi = np.array([-half[0], -half[1], -half[2]]), np.array([half[0], half[1], half[2]])
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    pts = corners @ _rotation(quat).T + pos
    return KitState(pos=pos, quat_xyzw=quat, lin_vel=vel[:3], ang_vel=vel[3:6], low=pts.min(0), high=pts.max(0),
                    joints=joints)


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
    if getattr(scene, "_pegkit_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Peg):
            return _peg_object(spec)
        if isinstance(spec, Photo):
            return _rigid_object(spec, _spawn_photo)
        if isinstance(spec, Fixed):
            return _fixed_object(scene, spec)
        return stock_object(spec)

    def object_states(environment, names):
        mine = {n for n in names if isinstance(_SPECS.get(n), (Peg, Photo))}
        plain = [n for n in names if n not in mine]
        out = stock_states(environment, plain) if plain else {}
        for n in mine:
            out[n] = _kit_state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._pegkit_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_pegkit_patched", False):
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
    cls._pegkit_patched = True


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
