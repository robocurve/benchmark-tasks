"""ornkit: glass Christmas baubles with wire hooks, and fixed parts (boxes and round rods) with their own friction, for
yam_arena task files.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from ornkit import Fixed, Ornament, Rod, body_pose, on_reset, on_step

- :class:`Fixed` is a fixed box with its own friction and look.
- :class:`Rod` is a fixed round rod (a cylinder) along x, y or z: the stand's post and branches.
- :class:`Ornament` is a loose glass bauble: a 60 mm ball with a metal cap and a rigid wire hook rising from the cap
  and curling over into a crook. In its frame the ball's centre is the origin, z is up through the cap, and the crook
  curls toward +x.

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

__all__ = ["Fixed", "KitState", "Ornament", "Rod", "body_pose", "env", "finger_points", "install", "on_reset",
           "on_step", "write_pose"]

# -- a glass bauble with a wire hook (m), in its own frame ---------------------------------------------------------------
BALL_R = 0.030
CAP_R, CAP_Z = 0.0085, (0.024, 0.048)         # the eight-sided metal crown cap over the ball's neck (to its corners)
WIRE_R = 0.0009                                # the hook's wire (1.8 mm)
SHANK_Z = (0.046, 0.074)                       # the hook's straight shank, up from the cap
CROOK_R = 0.010                                # the crook's radius (to the wire's centre line): a 2 cm open hook
TIP_DROP = 0.010                               # the crook's free end comes this far back down


def crook_centre() -> np.ndarray:
    return np.array([CROOK_R, 0.0, SHANK_Z[1]])


@dataclass(frozen=True)
class Fixed(Box):
    """A fixed task-file ``Box`` with its own friction and look."""

    static_friction: float = 0.5
    dynamic_friction: float = 0.4
    roughness: float = 0.6
    metallic: float = 0.0
    quat: tuple = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class Rod(Box):
    """A fixed round rod: ``pos`` its centre, ``size`` (diameter, length, axis: 1 x, 2 y, 3 z)."""

    static_friction: float = 0.5
    dynamic_friction: float = 0.4
    roughness: float = 0.7
    meta: dict = field(default_factory=dict, compare=False, hash=False)


@dataclass(frozen=True)
class Ornament(Box):
    """A glass bauble. ``pos`` is the ball's centre, ``quat`` (xyzw) its orientation."""

    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    glass: tuple = (0.80, 0.10, 0.12)
    friction: tuple = (0.4, 0.3)
    meta: dict = field(default_factory=dict, compare=False, hash=False)


@dataclass(frozen=True)
class KitState(ObjectState):
    """A bauble as the simulator has it."""

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

    def _unused_cyl_y(self, path, r, centre, look, length=0.0001):
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


def _rod_object(scene, spec: Rod):
    sim_utils = scene.sim_utils
    look = sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(spec.color), roughness=spec.roughness)
    d, length, axis = spec.size
    cfg = sim_utils.CylinderCfg(radius=float(d) / 2, height=float(length), axis="XYZ"[int(axis) - 1],
                                physics_material=_material(sim_utils, spec.static_friction, spec.dynamic_friction),
                                visual_material=look, collision_props=scene.PhysxCollisionPropertiesCfg())
    return scene.Object(name=spec.name, object_type=scene.ObjectType.BASE, spawner_cfg=cfg,
                        initial_pose=scene.Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                                rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


def _spawn_ornament(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """The bauble: a sphere collider for the ball, a cylinder for the cap, capsules for the hook's wire."""
    from pxr import UsdGeom

    spec: Ornament = _SPECS[cfg.kit_name]
    a = _Author(prim_path, translation, orientation)
    a.body(prim_path, None, spec.mass, lin_damp=1.0, ang_damp=3.0)     # swing damped by the hook's friction and the air
    m = a.mat("glass", *spec.friction)
    glass = a.look(spec.glass, rough=0.08, metal=0.35)
    gold = a.look((0.85, 0.70, 0.30), rough=0.25, metal=0.9)
    wire = a.look((0.75, 0.75, 0.78), rough=0.3, metal=0.9)
    s = UsdGeom.Sphere.Define(a.stage, f"{prim_path}/ball")
    s.CreateRadiusAttr(BALL_R)
    a.paint(s.GetPrim(), glass)
    a.collide(s.GetPrim(), m)
    for i, z in enumerate((-0.012, 0.0, 0.012)):                          # gold bands, drawn
        r = math.sqrt(BALL_R ** 2 - z ** 2) + 0.0003
        a.cyl(f"{prim_path}/band_{i}", r, z - 0.0012, z + 0.0012, gold)
    side = 2 * CAP_R * math.cos(math.pi / 8)                             # an eight-sided crown: two squares, 45 degrees apart
    for i, ang in enumerate((0.0, math.pi / 4)):                          # (flats to the fingers, so it cannot twist)
        a.box(f"{prim_path}/cap_{i}", (0.0, 0.0, (CAP_Z[0] + CAP_Z[1]) / 2), (side, side, CAP_Z[1] - CAP_Z[0]), gold, m,
              quat_wxyz=(math.cos(ang / 2), 0.0, 0.0, math.sin(ang / 2)))

    def segment(name, p0, p1):
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        d = p1 - p0
        length = float(np.linalg.norm(d))
        c = UsdGeom.Capsule.Define(a.stage, f"{prim_path}/{name}")
        c.CreateAxisAttr("Z")
        c.CreateRadiusAttr(WIRE_R)
        c.CreateHeightAttr(max(0.0002, length))
        c.AddTranslateOp().Set(a.Gf.Vec3d(*((p0 + p1) / 2)))
        z = np.array([0.0, 0.0, 1.0])
        u = d / length
        axis = np.cross(z, u)
        s_ = float(np.linalg.norm(axis))
        ang = math.atan2(s_, float(z @ u))
        axis = axis / s_ if s_ > 1e-9 else np.array([1.0, 0.0, 0.0])
        c.AddOrientOp().Set(a.Gf.Quatf(math.cos(ang / 2), *(axis * math.sin(ang / 2))))
        a.paint(c.GetPrim(), wire)
        a.collide(c.GetPrim(), m, offset=0.0006)

    segment("shank", (0.0, 0.0, SHANK_Z[0]), (0.0, 0.0, SHANK_Z[1]))
    c = crook_centre()
    n = 6
    pts = [c + CROOK_R * np.array([-math.cos(math.pi * k / n), 0.0, math.sin(math.pi * k / n)]) for k in range(n + 1)]
    for k in range(n):
        segment(f"crook_{k}", pts[k], pts[k + 1])
    segment("tip", pts[-1], pts[-1] + [0.0, 0.0, -TIP_DROP])
    from pxr import UsdPhysics
    UsdPhysics.MassAPI(a.root).CreateCenterOfMassAttr().Set(a.Gf.Vec3f(0.0, 0.0, -0.003))   # the glass, not the wire
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
    lo = np.array([-BALL_R, -BALL_R, -BALL_R])
    hi = np.array([BALL_R + 2 * CROOK_R, BALL_R, SHANK_Z[1] + CROOK_R])
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    pts = corners @ _rotation(quat).T + pos
    return KitState(pos=pos, quat_xyzw=quat, lin_vel=vel[:3], ang_vel=vel[3:6], low=pts.min(0), high=pts.max(0))


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
    if getattr(scene, "_ornkit_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Ornament):
            return _rigid_object(spec, _spawn_ornament)
        if isinstance(spec, Rod):
            return _rod_object(scene, spec)
        if isinstance(spec, Fixed):
            return _fixed_object(scene, spec)
        return stock_object(spec)

    def object_states(environment, names):
        mine = {n for n in names if isinstance(_SPECS.get(n), Ornament)}
        plain = [n for n in names if n not in mine]
        out = stock_states(environment, plain) if plain else {}
        for n in mine:
            out[n] = _kit_state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._ornkit_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_ornkit_patched", False):
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
    cls._ornkit_patched = True


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
