"""hhperson: a static, realistic person (a posed mannequin) for a yam_arena task file.

The person is a Microsoft Rocketbox avatar (MIT licence, https://github.com/microsoft/Microsoft-Rocketbox), posed
once offline (standing at the bench, leaning on it, hands resting flat) and baked into a static textured mesh:
``assets/person/<name>.usdc``. It never moves. In the simulator it is one kinematic body whose collision shapes are
invisible capsules (one per arm segment, hand, torso, head and leg), so the arms are physically blocked by it; the
visual mesh has no collider. ``<name>_proxies.json`` holds the same capsules and each hand's outline seen from above,
which the task's scorer uses.

yam_arena's task files spawn primitives and library assets only. Importing this module adds :class:`UsdBody` (a
fixed body loaded from a USD file in this folder) by hooking ``yam_arena.scene`` when it is imported, without changing
yam_arena (the same approach as the accepted lock-before-loading task's scenekit). Pure Python at import time.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import json
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from yam_arena.task_file import Box, ObjectState

import hhkit

HERE = Path(__file__).resolve().parent


@dataclass(frozen=True)
class UsdBody(Box):
    """A fixed (kinematic) body from ``usd`` (relative to this folder), its file origin at ``pos``. ``size`` is
    nominal (the bounding box, for the task-file checks)."""

    usd: str = ""

    def _check(self) -> None:
        if not (HERE / self.usd).is_file():
            raise FileNotFoundError(f"{self.name}: {HERE / self.usd} not found")
        if self.mass is not None:
            raise ValueError(f"{self.name}: a UsdBody here is fixed (mass=None)")


@dataclass(frozen=True)
class TrimBox(Box):
    """A loose box (its collider) drawn as ``trim``: visual-only boxes riding on it, each (name, size, offset from the
    box centre in its own frame, sRGB colour). The collider box itself is hidden. Used for a knife or a tool whose
    blade and handle must be told apart on camera; scoring treats it as the plain box it is."""

    trim: tuple = ()


@dataclass(frozen=True)
class FrictionBox(Box):
    """A loose box with its own surface friction (static, dynamic) instead of the scene's rubber-like default, e.g.
    planed timber on timber. PhysX averages the two surfaces' coefficients at a contact."""

    friction: tuple = (0.4, 0.35)


@dataclass(frozen=True)
class GlassBox(Box):
    """A fixed box drawn as clear glass (a fume-hood sash or roof). Collides like a plain fixed box."""


class Person:
    """A posed person: the USD body for OBJECTS and its collision capsules and hand outlines for scoring."""

    def __init__(self, name: str = "colleague", folder: str = "assets/person", stem: str = "colleague"):
        self.name = name
        data = json.loads((HERE / folder / f"{stem}_proxies.json").read_text())
        self.capsules = {c["name"]: (np.asarray(c["a"], float), np.asarray(c["b"], float), float(c["r"]))
                         for c in data["capsules"]}
        self.outlines = {k: np.asarray(v, float) for k, v in data["outlines"].items() if not k.endswith("_min_z")}
        self.points = {k: np.asarray(v, float) for k, v in data.get("points", {}).items()}  # skin samples per part
        self.source = data["source"]
        lo, hi = (np.asarray(v, float) for v in data["bounds"])
        self.bounds = (lo, hi)
        self.spec = UsdBody(name, pos=(0.0, 0.0, 0.0), size=tuple(float(v) for v in hi - lo),
                            color=(0.8, 0.7, 0.6), usd=f"{folder}/{stem}.usdc")

    def parts(self, *names: str) -> list:
        return [self.capsules[n] for n in names] if names else list(self.capsules.values())

    def robot_gap(self, robot_caps, parts=("R_hand", "R_forearm")) -> float:
        """Smallest gap (m) between any robot capsule and the skin of the named parts (sampled skin points)."""
        if not robot_caps:
            return float("inf")
        pts = np.vstack([self.points[p] for p in parts])
        return float(min(_points_capsule(pts, c).min() for c in robot_caps))

    def point_gap(self, p, parts=()) -> float:
        return min(hhkit.point_capsule(p, m) for m in self.parts(*parts))


def _points_capsule(P, cap) -> np.ndarray:
    a, b, r = np.asarray(cap[0], float), np.asarray(cap[1], float), float(cap[2])
    ab = b - a
    t = np.clip(((P - a) @ ab) / max(float(ab @ ab), 1e-12), 0.0, 1.0)
    return np.linalg.norm(P - (a + t[:, None] * ab), axis=1) - r


# -- the simulator side: spawn UsdBody objects and report their fixed state -------------------------------------
_SPECS: dict[str, UsdBody] = {}


def _usd_object(spec: UsdBody):
    import isaaclab.sim as sim_utils
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    addon = {"rigid_props": sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
             "physics_material": sim_utils.RigidBodyMaterialCfg(static_friction=0.8, dynamic_friction=0.7)}
    obj = Object(name=spec.name, usd_path=str(HERE / spec.usd), object_type=ObjectType.RIGID, spawn_cfg_addon=addon)
    obj.set_initial_pose(Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))
    return obj


def _trim_object(spec: TrimBox, scene):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils import clone
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxRigidBodyPropertiesCfg

    trim = tuple(spec.trim)

    @clone
    def _spawn(prim_path, cfg, *args, **kwargs):
        from pxr import Gf, UsdGeom

        prim = spawn_cuboid.__wrapped__(prim_path, cfg, *args, **kwargs)
        stage = prim.GetStage()
        mesh = stage.GetPrimAtPath(f"{prim_path}/geometry/mesh")
        if mesh.IsValid():
            UsdGeom.Imageable(mesh).MakeInvisible()
        for name, size, off, color in trim:
            path = f"{prim_path}/trim/{name}"
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in off]))
            cube.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in size]))
            if name.startswith("beam"):  # a laser's light: glowing and see-through, visual only
                look = sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(color),
                                                   emissive_color=scene.linear_rgb(color), opacity=0.35)
            else:
                look = sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(color), roughness=0.35,
                                                   metallic=0.8 if name.startswith("blade") or name.startswith("steel") else 0.0)
            look.func(f"{path}_material", look)
            sim_utils.bind_visual_material(path, f"{path}_material", stage=stage)
        return prim

    cfg = sim_utils.CuboidCfg(
        size=tuple(spec.size), func=_spawn,
        rigid_props=PhysxRigidBodyPropertiesCfg(), mass_props=sim_utils.MassPropertiesCfg(mass=spec.mass),
        collision_props=scene._contact_props(), physics_material=scene._grippy(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(spec.color), roughness=0.4))
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=scene._yaw_xyzw(spec.yaw)))


def _friction_object(spec: FrictionBox, scene):
    import isaaclab.sim as sim_utils
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxRigidBodyPropertiesCfg

    mat = sim_utils.RigidBodyMaterialCfg(static_friction=float(spec.friction[0]), dynamic_friction=float(spec.friction[1]))
    cfg = sim_utils.CuboidCfg(
        size=tuple(spec.size),
        rigid_props=PhysxRigidBodyPropertiesCfg(), mass_props=sim_utils.MassPropertiesCfg(mass=spec.mass),
        collision_props=scene._contact_props(), physics_material=mat,
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(spec.color), roughness=0.6))
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=scene._yaw_xyzw(spec.yaw)))


def _glass_object(spec: GlassBox, scene):
    import isaaclab.sim as sim_utils
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg

    try:
        look = sim_utils.GlassMdlCfg(glass_color=(0.92, 0.96, 0.97), thin_walled=True)
    except Exception:  # an Isaac Lab without the MDL glass config: a pale preview surface instead
        look = sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(spec.color), roughness=0.1, opacity=0.25)
    cfg = sim_utils.CuboidCfg(size=tuple(spec.size), collision_props=PhysxCollisionPropertiesCfg(), visual_material=look)
    return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=scene._yaw_xyzw(spec.yaw)))


def _state(spec: UsdBody) -> ObjectState:
    pos = np.asarray(spec.pos, float)
    half = np.asarray(spec.size, float) / 2
    return ObjectState(pos=pos, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos - half, high=pos + half)


def _patch_scene(scene) -> None:
    if getattr(scene, "_hhperson", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def task_object_hook(spec):
        if isinstance(spec, UsdBody):
            _SPECS[spec.name] = spec
            return _usd_object(spec)
        if isinstance(spec, TrimBox):
            return _trim_object(spec, scene)
        if isinstance(spec, FrictionBox):
            return _friction_object(spec, scene)
        if isinstance(spec, GlassBox):
            return _glass_object(spec, scene)
        return task_object(spec)

    def object_states_hook(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _state(_SPECS[n]) for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states = task_object_hook, object_states_hook
    scene._hhperson = True


class _Finder(importlib.abc.MetaPathFinder):
    """Patches yam_arena.scene as it is imported (it needs Isaac Sim, which starts after the task file loads)."""

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
        exec_module = spec.loader.exec_module

        def exec_and_patch(module):
            exec_module(module)
            _patch_scene(module)

        spec.loader.exec_module = exec_and_patch
        return spec


def install() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()


def local_centre(spec) -> np.ndarray:
    """The centre of a TrimBox's own-frame bounding box (collider and trim together). The simulator reports an
    object's ``pos`` as this point carried by the body, not the collider's centre, so trim that sticks out to one
    side (a jaw, a beam sheet) shifts it."""
    if not isinstance(spec, TrimBox) or not spec.trim:
        return np.zeros(3)
    lo, hi = -np.asarray(spec.size, float) / 2, np.asarray(spec.size, float) / 2
    for _name, size, off, _color in spec.trim:
        h = np.asarray(size, float) / 2
        lo, hi = np.minimum(lo, np.asarray(off, float) - h), np.maximum(hi, np.asarray(off, float) + h)
    return (lo + hi) / 2


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def body_states(objects: dict, specs) -> dict:
    """``objects`` with each TrimBox's ``pos`` moved back to its collider's centre (see ``local_centre``)."""
    import dataclasses

    out = dict(objects)
    for spec in specs:
        c = local_centre(spec)
        if spec.name in out and np.any(c != 0.0):
            s = out[spec.name]
            out[spec.name] = dataclasses.replace(s, pos=np.asarray(s.pos, float) - _rot(s.quat_xyzw) @ c)
    return out
