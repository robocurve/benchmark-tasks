"""rigidparts: rigid parts made of several boxes and cylinders, for yam_arena task files.

yam_arena's task-file objects are single primitives. A :class:`RigidPart` is one rigid body (or, without a mass, one
fixed collider) built from several boxes, upright cylinders and spheres in its own frame: a ring with a square hole, a
door with hinge knuckles, a pin with a head and a rounded nose. Keep this file next to the task file and import it there::

    import sys, pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).parent))
    from rigidparts import RigidPart, rigid_part

Importing it wraps ``yam_arena.scene.task_object`` (which builds each task object) and ``yam_arena.scene.object_states``
(which reports them to ``success()`` and ``score()``), when that module loads; yam_arena itself is not modified, so
the task runs with the standard ``yam_arena`` embodiment and scorers. For a part, ``ObjectState.pos`` is the origin of
the part's own frame (not the bounding-box centre reported for primitives), and :func:`to_world` maps a point in the
part frame to the world.

Pure Python at import time; Isaac Sim modules are imported only when the scene is built.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena.task_file import Box

Color = tuple[float, float, float]
BoxSpec = tuple  # (centre, size, colour), part frame, m
CylSpec = tuple  # (centre, radius, height, colour), axis along the part's z
SphereSpec = tuple  # (centre, radius, colour)


@dataclass(frozen=True)
class RigidPart(Box):
    """A rigid body built from ``boxes``, ``cylinders`` and ``spheres`` (part frame). ``size`` is its bounding box,
    for checks."""

    boxes: tuple = ()
    cylinders: tuple = ()
    spheres: tuple = ()
    friction: float | None = None  # None: the station's grippy material


def rigid_part(name, pos, boxes=(), cylinders=(), spheres=(), mass=None, color=(0.5, 0.5, 0.5), yaw=0.0,
               friction=None) -> RigidPart:
    """A part at ``pos`` (its frame's origin, world, m) turned ``yaw`` about the vertical."""
    pts = [np.array(c) + s * np.array(size) / 2 for c, size, _ in boxes for s in (-1, 1)]
    pts += [np.array(c) + np.array([s * r, s * r, s * h / 2]) for c, r, h, _ in cylinders for s in (-1, 1)]
    pts += [np.array(c) + s * r for c, r, _ in spheres for s in (-1, 1)]
    pts = np.array(pts)
    size = tuple(float(v) for v in pts.max(0) - pts.min(0))
    return RigidPart(name=name, pos=tuple(pos), color=color, mass=mass, yaw=yaw, size=size,
                     boxes=tuple(boxes), cylinders=tuple(cylinders), spheres=tuple(spheres), friction=friction)


def quat_matrix(xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def to_world(state, point) -> np.ndarray:
    """A point given in a part's frame, in the world frame, for a part's ``ObjectState``."""
    return np.asarray(state.pos) + quat_matrix(state.quat_xyzw) @ np.asarray(point, dtype=float)


def axis_world(state, axis=(0.0, 0.0, 1.0)) -> np.ndarray:
    return quat_matrix(state.quat_xyzw) @ np.asarray(axis, dtype=float)


# -- building a part in Isaac Sim ---------------------------------------------------------------------------------

_SPECS: dict[str, RigidPart] = {}


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the part under ``prim_path``: an Xform that is the rigid body (if it has a mass), with one collider
    child per box, cylinder and sphere."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from yam_arena.constants import GRIPPY_FRICTION
    from yam_arena.station import linear_rgb

    spec = _SPECS[cfg.part_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
    prim = root.GetPrim()
    if spec.free:
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(spec.mass)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(16)

    static, dynamic = (spec.friction, spec.friction) if spec.friction is not None else GRIPPY_FRICTION
    material = f"{prim_path}/Looks/physics"
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=static, dynamic_friction=dynamic)
    pm.func(material, pm)
    looks: dict[Color, str] = {}

    def collide(geom, color):
        if color not in looks:
            looks[color] = f"{prim_path}/Looks/colour_{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
            vis.func(looks[color], vis)
        p = geom.GetPrim()
        UsdPhysics.CollisionAPI.Apply(p)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(p)
        cp.CreateContactOffsetAttr(0.002)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(p.GetPath()), material, stage=stage)
        bind_visual_material(str(p.GetPath()), looks[color], stage=stage)

    for i, (centre, size, color) in enumerate(spec.boxes):
        cube = UsdGeom.Cube.Define(stage, f"{prim_path}/box_{i}")
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        collide(cube, color)
    for i, (centre, radius, height, color) in enumerate(spec.cylinders):
        cyl = UsdGeom.Cylinder.Define(stage, f"{prim_path}/cyl_{i}")
        cyl.CreateAxisAttr("Z")
        cyl.CreateRadiusAttr(radius)
        cyl.CreateHeightAttr(height)
        cyl.AddTranslateOp().Set(Gf.Vec3d(*centre))
        collide(cyl, color)
    for i, (centre, radius, color) in enumerate(spec.spheres):
        ball = UsdGeom.Sphere.Define(stage, f"{prim_path}/sphere_{i}")
        ball.CreateRadiusAttr(radius)
        ball.AddTranslateOp().Set(Gf.Vec3d(*centre))
        collide(ball, color)
    return prim


def _arena_object(spec: RigidPart):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class RigidPartSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn)
        part_name: str = ""

    half = spec.yaw / 2
    return Object(
        name=spec.name,
        object_type=ObjectType.RIGID if spec.free else ObjectType.BASE,
        spawner_cfg=RigidPartSpawnerCfg(part_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, math.sin(half), math.cos(half))),
    )


def _patch_scene(scene) -> None:
    if getattr(scene, "_rigidparts", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def task_object_with_parts(spec):
        if isinstance(spec, RigidPart):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def object_states_with_parts(env, names):
        out = object_states(env, names)
        base = env.unwrapped
        origin = base.scene.env_origins[0]
        for name in names:
            if name in _SPECS:  # report the part frame's origin, not the bounding-box centre
                pose = base.arena_world.get_pose_w(name)[0]
                state = out[name]
                pos = (pose[:3] - origin).detach().cpu().numpy().astype(np.float64)
                out[name] = type(state)(pos, state.quat_xyzw, state.lin_vel, state.ang_vel, state.low, state.high)
        return out

    scene.task_object = task_object_with_parts
    scene.object_states = object_states_with_parts
    scene._rigidparts = True


class _PatchOnImport(importlib.abc.MetaPathFinder):
    """Patch ``yam_arena.scene`` as soon as it is imported (it imports Isaac Sim, so it loads after the task file)."""

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


if "yam_arena.scene" in sys.modules:
    _patch_scene(sys.modules["yam_arena.scene"])
elif not any(isinstance(f, _PatchOnImport) for f in sys.meta_path):
    sys.meta_path.insert(0, _PatchOnImport())
