"""Rigid bodies made of several boxes (an L, a T, a hook ...) for yam_arena task files, and the grippers' positions.

yam_arena's task files offer single boxes, cylinders and spheres. A ``Compound`` is one rigid body whose colliders
are several boxes:

    from compound_body import Part, compound
    OBJECTS = [compound("half", pos=(0.3, 0.1, z), mass=0.18, color=WOOD,
                        parts=[Part((0, 0.055, 0), (0.05, 0.03, 0.15)), Part((0, -0.015, 0.06), (0.05, 0.11, 0.03))])]

``pos`` is the body's origin in the world and each part a box given by its centre (relative to the origin, before
``yaw``) and its edge lengths, in metres. Put the origin at the centre of the parts' bounding box: ``compound`` fills
in ``size`` with that box. The mass is spread evenly over the boxes (PhysX computes the centre of mass and the
inertia from them). With no ``mass`` the body is fixed, like a fixed ``Box``.

This module patches yam_arena when its modules are imported (or at once, if they already are); it imports nothing
from Isaac Sim itself, so a task file that uses it still loads before the simulator starts:

- ``yam_arena.scene.task_object`` spawns a Compound as one rigid body of box colliders (other objects as before);
- ``yam_arena.scene.object_states`` reports a Compound's origin and orientation, with the world-aligned bounds of its
  boxes as posed, and adds ``"_tcp_left"`` and ``"_tcp_right"``: each gripper's TCP (the point between its finger
  pads, 13 mm above their tips), from the arm's joint positions and the URDF's kinematics (``yam_arena.ik``), so a
  task can tell whether a gripper still holds an object.
"""

from __future__ import annotations

import importlib.abc
import math
import os
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena.task_file import Box, ObjectState

TCP_KEYS = {"left": "_tcp_left", "right": "_tcp_right"}
_DEBUG = os.environ.get("COMPOUND_DEBUG") == "1"


@dataclass(frozen=True)
class Part:
    """One box of a compound body: its centre relative to the body's origin (before yaw) and its edge lengths (m).
    ``color`` (sRGB) overrides the body's colour for this box."""

    center: tuple[float, float, float]
    size: tuple[float, float, float]
    color: tuple[float, float, float] | None = None

    def corners(self) -> np.ndarray:
        c, h = np.asarray(self.center, dtype=float), np.asarray(self.size, dtype=float) / 2
        signs = np.array([[a, b, d] for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)], dtype=float)
        return c + signs * h

    def distance(self, p: np.ndarray) -> float:
        """Distance (m) from a point in the body's frame to this box (0 inside it)."""
        d = np.abs(np.asarray(p, dtype=float) - np.asarray(self.center, dtype=float)) - np.asarray(self.size) / 2
        return float(np.linalg.norm(np.maximum(d, 0.0)))


@dataclass(frozen=True)
class Compound(Box):
    """A rigid body made of boxes. It subclasses ``Box`` only so that yam_arena's task-file checks accept it:
    ``pos`` is the body's origin and ``size`` the bounding box of its parts (see :func:`compound`)."""

    parts: tuple[Part, ...] = ()

    def _check(self) -> None:
        super()._check()
        if not self.parts:
            raise ValueError(f"{self.name}: a Compound needs at least one part")
        for part in self.parts:
            if len(part.center) != 3 or len(part.size) != 3 or any(not s > 0 for s in part.size):
                raise ValueError(f"{self.name}: bad part {part!r}")

    def corners(self) -> np.ndarray:
        """Every part's corners in the body's frame, (8 * parts, 3)."""
        return np.concatenate([p.corners() for p in self.parts])

    def distance(self, state: ObjectState, point) -> float:
        """Distance (m) from a world point to the body posed as in ``state``."""
        local = quat_matrix(state.quat_xyzw).T @ (np.asarray(point, dtype=float) - state.pos)
        return min(p.distance(local) for p in self.parts)


def compound(name: str, pos, parts, mass: float | None = None, color=(0.7, 0.7, 0.7), yaw: float = 0.0) -> Compound:
    """A Compound with ``size`` filled in from its parts' bounding box."""
    parts = tuple(parts)
    pts = np.concatenate([p.corners() for p in parts])
    size = tuple(float(v) for v in pts.max(axis=0) - pts.min(axis=0))
    return Compound(name=name, pos=tuple(float(v) for v in pos), size=size, mass=mass, color=tuple(color),
                    yaw=float(yaw), parts=parts)


def quat_matrix(q) -> np.ndarray:
    """Rotation matrix of an XYZW quaternion."""
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# -- spawning --------------------------------------------------------------------------------------------------------

_SPECS: dict[str, Compound] = {}


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the body under ``prim_path``: an Xform (the rigid body, with its mass) holding one box collider per
    part, each with the task objects' rubber-like friction and its colour."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    from yam_arena.constants import GRIPPY_FRICTION
    from yam_arena.station import linear_rgb

    spec = _SPECS[cfg.compound_name]
    stage = get_current_stage()
    double = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation if translation is not None else (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))
    prim = root.GetPrim()
    if spec.free:
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(spec.mass))
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim)

    physics = f"{prim_path}/Looks/physics"
    material = sim_utils.RigidBodyMaterialCfg(static_friction=GRIPPY_FRICTION[0], dynamic_friction=GRIPPY_FRICTION[1])
    material.func(physics, material)
    looks: dict[tuple, str] = {}

    def look(color) -> str:
        color = tuple(float(c) for c in color)
        if color not in looks:
            path = f"{prim_path}/Looks/color_{len(looks)}"
            surface = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.4)
            surface.func(path, surface)
            looks[color] = path
        return looks[color]

    for i, part in enumerate(spec.parts):
        geom = UsdGeom.Cube.Define(stage, f"{prim_path}/box_{i}")
        geom.CreateSizeAttr(1.0)
        geom.AddTranslateOp().Set(Gf.Vec3d(*(float(v) for v in part.center)))
        geom.AddScaleOp().Set(Gf.Vec3f(*(float(v) for v in part.size)))
        box = geom.GetPrim()
        UsdPhysics.CollisionAPI.Apply(box)
        physx = PhysxSchema.PhysxCollisionAPI.Apply(box)
        # As yam_arena's free objects: contacts start 5 mm out and rest at zero separation.
        physx.CreateContactOffsetAttr(0.005)
        physx.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(box.GetPath()), physics, stage=stage)
        bind_visual_material(str(box.GetPath()), look(part.color or spec.color), stage=stage)
    return prim


def _arena_object(spec: Compound):
    """The compound as an Isaac Lab-Arena object: a rigid object (free) or a static collider (fixed)."""
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    try:
        from isaaclab.sim.spawners.spawner_cfg import RigidObjectSpawnerCfg as _Base
    except ImportError:  # pragma: no cover - older or newer Isaac Lab layouts
        from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg as _Base

    _SPECS[spec.name] = spec

    @configclass
    class CompoundSpawnerCfg(_Base):
        func: object = clone(_spawn)
        compound_name: str = ""

    return Object(
        name=spec.name,
        object_type=ObjectType.RIGID if spec.free else ObjectType.BASE,
        spawner_cfg=CompoundSpawnerCfg(compound_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos),
                          rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))),
    )


# -- state -----------------------------------------------------------------------------------------------------------


def _numpy(x) -> np.ndarray:
    x = x.torch if hasattr(x, "torch") else x
    if hasattr(x, "detach"):
        x = x.detach().cpu().numpy()
    return np.asarray(x, dtype=np.float64)


def _compound_states(environment, names) -> dict[str, ObjectState]:
    base = environment.unwrapped
    world = base.arena_world
    origin = _numpy(base.scene.env_origins[0])
    out = {}
    for name in names:
        spec = _SPECS[name]
        pose = _numpy(world.get_pose_w(name)[0])
        pos, quat = pose[:3] - origin, pose[3:7]
        pts = spec.corners() @ quat_matrix(quat).T + pos
        moving = name in base.scene.rigid_objects
        lin = _numpy(world.get_root_linear_velocity_w(name)[0]) if moving else np.zeros(3)
        ang = _numpy(world.get_root_angular_velocity_w(name)[0]) if moving else np.zeros(3)
        out[name] = ObjectState(pos=pos, quat_xyzw=quat, lin_vel=lin, ang_vel=ang, low=pts.min(axis=0),
                                high=pts.max(axis=0))
        if _DEBUG and name not in _DEBUG_SEEN:
            _DEBUG_SEEN.add(name)
            data = base.scene[name].data if moving else None
            extra = ""
            for attr in ("root_link_pos_w", "root_com_pos_w", "root_pos_w"):
                if data is not None and hasattr(data, attr):
                    extra += f" {attr}={np.round(_numpy(getattr(data, attr))[0] - origin, 4)}"
            print(f"COMPOUND {name}: pose pos={np.round(pos, 4)} quat={np.round(quat, 4)}{extra}", flush=True)
    return out


_DEBUG_SEEN: set[str] = set()
_ARM_IDS: dict[str, list[int]] = {}


def _tcp_states(environment) -> dict[str, ObjectState]:
    """Each gripper's TCP from its arm's joint positions (URDF forward kinematics), as an ObjectState at rest."""
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS

    base = environment.unwrapped
    out = {}
    for side, key in TCP_KEYS.items():
        robot = base.scene[f"{side}_robot"]
        if side not in _ARM_IDS:
            _ARM_IDS[side] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
        q = _numpy(robot.data.joint_pos)[0, _ARM_IDS[side]]
        tcp, _ = ik.tcp_pose(q, station.BASES[side])
        zero = np.zeros(3)
        out[key] = ObjectState(pos=np.asarray(tcp, dtype=float), quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                               lin_vel=zero, ang_vel=zero, low=np.asarray(tcp, dtype=float),
                               high=np.asarray(tcp, dtype=float))
    return out


# -- patching yam_arena ----------------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_compound_patched", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def patched_task_object(spec):
        return _arena_object(spec) if isinstance(spec, Compound) else task_object(spec)

    def patched_object_states(environment, names):
        names = list(names)
        others = [n for n in names if n not in _SPECS]
        out = dict(object_states(environment, others)) if others else {}
        out.update(_compound_states(environment, [n for n in names if n in _SPECS]))
        out.update(_tcp_states(environment))
        return out

    scene.task_object, scene.object_states = patched_task_object, patched_object_states
    scene._compound_patched = True


_PATCHES = {"yam_arena.scene": _patch_scene}


class _CompoundPatchOnImport(importlib.abc.MetaPathFinder):
    """Finds the modules in ``_PATCHES`` with the other finders and patches each right after it executes."""

    def find_spec(self, fullname, path, target=None):
        if fullname not in _PATCHES:
            return None
        for finder in sys.meta_path:
            if finder is self or not hasattr(finder, "find_spec"):
                continue
            spec = finder.find_spec(fullname, path, target)
            if spec is not None:
                break
        else:
            return None
        execute, patch = spec.loader.exec_module, _PATCHES[fullname]

        def exec_module(module):
            execute(module)
            patch(module)

        spec.loader.exec_module = exec_module
        return spec


def install() -> None:
    for name, patch in _PATCHES.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(type(f).__name__ == "_CompoundPatchOnImport" for f in sys.meta_path):
        sys.meta_path.insert(0, _CompoundPatchOnImport())


install()
