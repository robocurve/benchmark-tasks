"""doughkit: a mixing bowl, a wooden spoon, a utensil cup and a batch of dough ingredients for yam_arena task files.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    from doughkit import Bowl, Fixed, Spoon, Beads, bead_states, on_reset, on_step

- :class:`Bowl` is a loose rigid mixing bowl: a disc base and a flared wall of overlapping segments.
- :class:`Spoon` is a loose rigid wooden spoon: a handle and a shallow head.
- :class:`Fixed` is a fixed box with its own friction (the utensil cup is built from these).
- :class:`Beads` is the batch: small spheres for the flour and for the wet ingredients. The beads are rigid bodies
  authored next to the scene objects but not registered with Isaac Lab (two hundred scene objects would make every
  state read slow); they are read and reset through one PhysX rigid-body view (:func:`bead_states`,
  :func:`reset_beads`), and :func:`tint_beads` recolours them as flour is wetted.

``on_step`` and ``on_reset`` register callbacks ``fn(env)``, run after each control step and after each reset.

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
from yam_arena.task_file import Box

__all__ = ["Beads", "Bowl", "Fixed", "Spoon", "bead_states", "body_pose", "env", "finger_points", "install", "on_reset", "on_step",
           "reset_beads", "tint_beads"]


@dataclass(frozen=True)
class Fixed(Box):
    """A fixed task-file ``Box`` with its own friction."""

    static_friction: float = 0.5
    dynamic_friction: float = 0.4
    combine: str = "average"
    roughness: float = 0.6


@dataclass(frozen=True)
class Bowl(Box):
    """A mixing bowl. ``pos`` is the centre of the underside of its base. Inner radius ``r_bottom`` at the floor,
    ``r_top`` at the rim, ``height`` overall, wall ``wall`` thick, ``segments`` wall pieces."""

    r_bottom: float = 0.065
    r_top: float = 0.100
    height: float = 0.085
    wall: float = 0.005
    base: float = 0.006
    segments: int = 32
    friction: tuple = (0.25, 0.20)          # underneath, on the worktop
    inner_friction: tuple = (0.4, 0.3)
    rim_color: tuple = (0.95, 0.95, 0.94)
    tab: tuple = (0.040, 0.005, 0.025)      # the grip tab on the +y side: out from the rim, thick, tall
    tab_color: tuple = (0.18, 0.18, 0.20)
    tab_friction: tuple = (0.4, 0.3)        # the tab (steel)
    clip: tuple = (0.022, 0.014, 0.020)     # the spoon clip on the -x side: reach out from the rim, notch width, post height
    meta: dict = field(default_factory=dict, compare=False, hash=False)


@dataclass(frozen=True)
class Spoon(Box):
    """A wooden spoon. ``pos`` is the handle's top end; the spoon hangs along -z (its own frame) from there, the head
    at the bottom. ``quat`` its orientation (xyzw)."""

    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    handle: tuple = (0.012, 0.008, 0.22)    # across, thick, long
    head: tuple = (0.045, 0.010, 0.060)
    friction: tuple = (0.8, 0.7)
    grip: tuple = (0.024, 0.024, 0.095)     # the built-up grip sleeve: across, deep, long (0 for none)
    grip_at: float = 0.06                   # the grip's middle, this far from the handle's top end
    grip_color: tuple = (0.20, 0.45, 0.70)
    grip_friction: tuple = (1.0, 0.9)       # silicone
    com: float = 0.0                        # the balance point, this far from the handle's top end (0: from the shapes)
    meta: dict = field(default_factory=dict, compare=False, hash=False)


@dataclass(frozen=True)
class Beads(Box):
    """The batch: ``positions`` (n x 3, world) and ``kinds`` (0 flour, 1 wet) of the beads, radius ``radius``,
    masses per kind. ``pos``/``size`` only place the (empty) anchor the loader sees."""

    positions: tuple = ()
    kinds: tuple = ()
    radius: float = 0.006
    masses: tuple = (0.0012, 0.0040)
    colors: tuple = ((0.97, 0.95, 0.88), (0.98, 0.80, 0.25), (0.86, 0.72, 0.45))   # flour, wet, wetted flour
    friction: tuple = (0.6, 0.5)
    meta: dict = field(default_factory=dict, compare=False, hash=False)


def _material(sim_utils, sf, df, combine="average"):
    try:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df, friction_combine_mode=combine)
    except TypeError:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df)


def _rotation(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


# -- spawning ---------------------------------------------------------------------------------------------------------

_SPECS: dict = {}
_BEADS: dict = {"spec": None, "root": None, "view": None, "looks": None, "tint": None}


def _fixed_object(scene, spec: Fixed):
    sim_utils = scene.sim_utils
    look = sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(spec.color), roughness=spec.roughness)
    cfg = sim_utils.CuboidCfg(size=tuple(spec.size),
                              physics_material=_material(sim_utils, spec.static_friction, spec.dynamic_friction,
                                                         spec.combine),
                              visual_material=look, collision_props=scene.PhysxCollisionPropertiesCfg())
    return scene.Object(name=spec.name, object_type=scene.ObjectType.BASE, spawner_cfg=cfg,
                        initial_pose=scene.Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                                rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


class _Author:
    """Small helpers for authoring one body's shapes with pxr."""

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
        self.looks: dict = {}
        self.mats: dict = {}

    def look(self, color, rough=0.5, metal=0.0, opacity=1.0, base=None) -> str:
        from yam_arena.station import linear_rgb

        key = (tuple(color), rough, metal, opacity)
        if key not in self.looks:
            path = f"{base or self.path}/Looks/v{len(self.looks)}"
            vis = self.sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough, metallic=metal,
                                                   opacity=opacity)
            vis.func(path, vis)
            self.looks[key] = path
        return self.looks[key]

    def mat(self, key, sf, df, base=None) -> str:
        if key not in self.mats:
            path = f"{base or self.path}/Looks/phys_{key}"
            pm = _material(self.sim_utils, sf, df)
            pm.func(path, pm)
            self.mats[key] = path
        return self.mats[key]

    def body(self, prim, mass, lin_damp=0.05, ang_damp=0.2):
        from pxr import PhysxSchema, UsdPhysics

        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.5)
        rb.CreateLinearDampingAttr(lin_damp)
        rb.CreateAngularDampingAttr(ang_damp)
        return rb

    def collide(self, prim, mat, offset=0.001):
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
        self.paint(cube.GetPrim(), look)
        if mat:
            self.collide(cube.GetPrim(), mat)
        return cube.GetPrim()

    def cyl(self, path, r, z0, z1, look, mat=None):
        c = self.UsdGeom.Cylinder.Define(self.stage, path)
        c.CreateAxisAttr("Z")
        c.CreateRadiusAttr(float(r))
        c.CreateHeightAttr(float(z1 - z0))
        c.AddTranslateOp().Set(self.Gf.Vec3d(0.0, 0.0, float(z0 + z1) / 2))
        self.paint(c.GetPrim(), look)
        if mat:
            self.collide(c.GetPrim(), mat)
        return c.GetPrim()


def _quat_wxyz_axis(axis, angle):
    s = math.sin(angle / 2)
    return (math.cos(angle / 2), axis[0] * s, axis[1] * s, axis[2] * s)


def _quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def _spawn_bowl(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec: Bowl = _SPECS[cfg.kit_name]
    a = _Author(prim_path, translation, orientation)
    a.body(a.root, spec.mass)
    body = a.look(spec.color, rough=0.25, metal=0.7)
    under = a.mat("under", *spec.friction)
    inner = a.mat("inner", *spec.inner_friction)
    a.cyl(f"{prim_path}/base", spec.r_bottom + spec.wall, 0.0, spec.base, body, under)
    flare = math.atan2(spec.r_top - spec.r_bottom, spec.height - spec.base)   # the wall's tilt out from vertical
    slant = (spec.height - spec.base) / math.cos(flare)
    n = spec.segments
    for i in range(n):
        th = 2 * math.pi * i / n                                           # a flat face at 0, 90, 180 and 270 degrees
        r_mid = (spec.r_bottom + spec.r_top) / 2 + spec.wall / 2
        width = 2 * math.pi * (r_mid + spec.wall) / n * 1.08                # overlap so nothing slips between
        centre = (r_mid * math.cos(th), r_mid * math.sin(th), spec.base + (spec.height - spec.base) / 2)
        q = _quat_mul(_quat_wxyz_axis((0, 0, 1), th), _quat_wxyz_axis((0, 1, 0), flare))
        a.box(f"{prim_path}/wall_{i}", centre, (spec.wall, width, slant), body, inner, quat_wxyz=q)
    rim = a.look(spec.rim_color, rough=0.3, metal=0.6)
    for i in range(n):                                                     # a rolled rim, drawn only
        th = 2 * math.pi * i / n                                           # a flat face at 0, 90, 180 and 270 degrees
        r = spec.r_top + spec.wall
        a.box(f"{prim_path}/rim_{i}", (r * math.cos(th), r * math.sin(th), spec.height),
              (0.007, 2 * math.pi * r / n * 1.05, 0.006), rim, quat_wxyz=_quat_wxyz_axis((0, 0, 1), th))
    tl, tt, th_ = spec.tab                                                   # the grip tab, at the rim, toward +y
    a.box(f"{prim_path}/tab", (0.0, spec.r_top + spec.wall / 2 + tl / 2, spec.height - th_ / 2 - 0.002),
          (tt, tl + 0.004, th_), a.look(spec.tab_color, rough=0.9), a.mat("tab", *spec.tab_friction))
    reach, notch, post = spec.clip                                          # the spoon clip, at the rim, toward -x
    if reach > 0:
        dark = a.look(spec.tab_color, rough=0.7)
        r0 = spec.r_top + spec.wall
        a.box(f"{prim_path}/clip_shelf", (-(r0 + reach / 2), 0.0, spec.height + 0.002),
              (reach + 0.004, notch + 0.016, 0.004), dark, inner)
        for side in (-1, 1):
            a.box(f"{prim_path}/clip_post_{'ab'[side > 0]}", (-(r0 + reach / 2), side * (notch / 2 + 0.003),
                                                             spec.height + 0.004 + post / 2),
                  (reach, 0.006, post), dark, inner)
    from pxr import UsdPhysics
    UsdPhysics.MassAPI(a.root).CreateCenterOfMassAttr().Set(a.Gf.Vec3f(0.0, 0.0, 0.02))
    return a.root


def _spawn_spoon(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    spec: Spoon = _SPECS[cfg.kit_name]
    a = _Author(prim_path, translation, orientation)
    a.body(a.root, spec.mass, ang_damp=0.5)
    wood = a.look(spec.color, rough=0.8)
    m = a.mat("wood", *spec.friction)
    hx, hy, hl = spec.handle
    a.box(f"{prim_path}/handle", (0.0, 0.0, -hl / 2), spec.handle, wood, m)
    a.box(f"{prim_path}/neck", (0.0, 0.0, -hl - 0.008), (hx * 1.6, hy, 0.016), wood, m)
    ex, ey, el = spec.head
    a.box(f"{prim_path}/head", (0.0, 0.0, -hl - 0.016 - el / 2), (ex, ey, el), wood, m)
    gx, gy, gl = spec.grip
    if gl > 0:
        a.box(f"{prim_path}/grip", (0.0, 0.0, -spec.grip_at), (gx, gy, gl), a.look(spec.grip_color, rough=0.9),
              a.mat("grip", *spec.grip_friction))
    if spec.com > 0:
        from pxr import UsdPhysics
        UsdPhysics.MassAPI(a.root).CreateCenterOfMassAttr().Set(a.Gf.Vec3f(0.0, 0.0, -float(spec.com)))
    for side in (-1, 1):                                                   # the head's rounded shoulders, drawn
        a.box(f"{prim_path}/shoulder_{'ab'[side > 0]}", (side * ex * 0.35, 0.0, -hl - 0.018),
              (ex * 0.35, ey * 0.98, 0.010), wood, quat_wxyz=_quat_wxyz_axis((0, 1, 0), side * 0.6))
    return a.root


def _spawn_beads(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """The anchor at ``prim_path`` is an empty fixed Xform; the beads go in a sibling group, not under it, so that
    Isaac Lab sees no rigid bodies under the anchor."""
    from pxr import UsdGeom

    spec: Beads = _SPECS[cfg.kit_name]
    a = _Author(prim_path, translation, None)
    a.box(f"{prim_path}/anchor", (0.0, 0.0, 0.0), (0.01, 0.01, 0.01), a.look((0.5, 0.5, 0.5)))   # under the table
    group = prim_path.rsplit("/", 1)[0] + "/dough_beads"
    UsdGeom.Xform.Define(a.stage, group)
    looks = [a.look(c, rough=0.9, base=group) for c in spec.colors]
    mat = a.mat("bead", *spec.friction, base=group)
    for i, (p, k) in enumerate(zip(spec.positions, spec.kinds)):
        s = UsdGeom.Sphere.Define(a.stage, f"{group}/bead_{i:03d}")
        s.CreateRadiusAttr(float(spec.radius))
        s.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(a.Gf.Vec3d(*(float(v) for v in p)))
        s.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(a.Gf.Quatd(1, 0, 0, 0))
        prim = s.GetPrim()
        a.body(prim, spec.masses[int(k)], lin_damp=0.3, ang_damp=1.0)
        a.collide(prim, mat, offset=0.0015)
        a.paint(prim, looks[int(k)])
    _BEADS.update(spec=spec, root=group, looks=looks, view=None, tint=[int(k) for k in spec.kinds])
    return a.root


def _custom_object(spec, spawn, kind):
    from isaaclab.sim.spawners.spawner_cfg import RigidObjectSpawnerCfg, SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    base_cfg = RigidObjectSpawnerCfg if kind == "rigid" else SpawnerCfg

    @configclass
    class KitSpawnerCfg(base_cfg):
        func: object = clone(spawn)
        kit_name: str = ""

    _SPECS[spec.name] = spec
    quat = tuple(float(v) for v in getattr(spec, "quat", (0.0, 0.0, 0.0, 1.0)))
    return Object(name=spec.name, object_type=ObjectType.RIGID if kind == "rigid" else ObjectType.BASE,
                  spawner_cfg=KitSpawnerCfg(kit_name=spec.name),
                  initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=quat))


# -- the beads at run time ---------------------------------------------------------------------------------------------

def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    if hasattr(x, "numpy") and not hasattr(x, "detach"):
        return np.asarray(x.numpy(), dtype=np.float64)                 # a warp array
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _like(ref, data: np.ndarray, dtype=None):
    """``data`` as the same kind of array as ``ref`` (warp or torch), on its device."""
    if hasattr(ref, "detach"):
        import torch
        return torch.tensor(data, dtype=dtype or ref.dtype, device=ref.device)
    import warp as wp
    return wp.array(data, dtype=dtype or ref.dtype, device=ref.device)


def _view(environment):
    if _BEADS["view"] is None:
        base = environment.unwrapped
        sim = getattr(base, "sim", None)
        psv = getattr(sim, "physics_sim_view", None)
        if psv is None:
            from isaacsim.core.simulation_manager import SimulationManager
            psv = SimulationManager.get_physics_sim_view()
        pattern = _BEADS["root"].replace("env_0", "env_*") + "/bead_*"
        _BEADS["view"] = psv.create_rigid_body_view(pattern)
    return _BEADS["view"]


def bead_states(environment):
    """Positions (n x 3, relative to the env origin) and linear velocities (n x 3) of the beads."""
    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    v = _view(environment)
    tf = _np(v.get_transforms())
    vel = _np(v.get_velocities())
    return tf[:, :3] - origin, vel[:, :3]


def body_pose(environment, name: str):
    """A loose kit object's body pose: position (relative to the env origin) and quaternion (xyzw)."""
    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    pose = _np(base.scene[name].data.root_link_pose_w)[0]
    return pose[:3] - origin, pose[3:7]


def reset_beads(environment) -> None:
    spec: Beads = _BEADS["spec"]
    v = _view(environment)
    origin = _np(environment.unwrapped.scene.env_origins[0])
    n = len(spec.positions)
    tf = np.zeros((n, 7), dtype=np.float32)
    tf[:, :3] = np.asarray(spec.positions) + origin
    tf[:, 6] = 1.0
    cur = v.get_transforms()
    idx = _like(cur, np.arange(n, dtype=np.int32), dtype=None if hasattr(cur, "detach") else None)
    if hasattr(cur, "detach"):
        import torch
        idx = torch.arange(n, dtype=torch.int32, device=cur.device)
    else:
        import warp as wp
        idx = wp.array(np.arange(n, dtype=np.int32), dtype=wp.int32, device=cur.device)
    v.set_transforms(_like(cur, tf), idx)
    v.set_velocities(_like(cur, np.zeros((n, 6), dtype=np.float32)), idx)
    tint_beads([int(k) for k in spec.kinds])


def tint_beads(kinds) -> None:
    """Recolour the beads whose kind changed (0 flour, 1 wet, 2 wetted flour)."""
    from isaaclab.sim.utils import bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage

    stage = get_current_stage()
    old = _BEADS["tint"]
    for i, k in enumerate(kinds):
        if old[i] != k:
            bind_visual_material(f"{_BEADS['root']}/bead_{i:03d}", _BEADS["looks"][k], stage=stage)
            old[i] = k


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
    if getattr(scene, "_doughkit_patched", False):
        return
    stock_object = scene.task_object

    def task_object(spec):
        if isinstance(spec, Bowl):
            return _custom_object(spec, _spawn_bowl, "rigid")
        if isinstance(spec, Spoon):
            return _custom_object(spec, _spawn_spoon, "rigid")
        if isinstance(spec, Beads):
            return _custom_object(spec, _spawn_beads, "base")
        if isinstance(spec, Fixed):
            return _fixed_object(scene, spec)
        return stock_object(spec)

    scene.task_object = task_object
    scene._doughkit_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_doughkit_patched", False):
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
    cls._doughkit_patched = True


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
