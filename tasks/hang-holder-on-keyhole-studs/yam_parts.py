"""Compound parts for YAM Arena tasks.

A :class:`Part` is physically simulated as one rigid union of convex box,
cylinder, and sphere colliders.  PhysX derives the centre of mass and inertia
from those colliders at uniform density, and every collider is bound to a
friction material.  The pure-NumPy :meth:`Part.aabb` calculation is exact for
boxes; cylinder and sphere bounds conservatively rotate their enclosing boxes.

Importing this module installs a lazy scene hook.  Isaac Lab, USD, and Arena
runtime imports occur only after the simulator imports ``yam_arena.scene`` and
asks it to construct a Part.
"""

from __future__ import annotations

import functools
import importlib.abc
import importlib.machinery
import math
import sys
from collections.abc import Iterable
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from yam_arena.task_file import GREY, Box, ObjectState


@dataclass(frozen=True)
class _BoxShape:
    center: tuple[float, float, float]
    size: tuple[float, float, float]
    color: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class _CylinderShape:
    center: tuple[float, float, float]
    radius: float
    height: float
    axis: str = "z"
    color: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class _BallShape:
    center: tuple[float, float, float]
    radius: float
    color: tuple[float, float, float] | None = None


Shape = _BoxShape | _CylinderShape | _BallShape


def _tuple(values: Iterable[float]) -> tuple[float, ...]:
    return tuple(float(value) for value in values)


def box(center, size, color=None) -> _BoxShape:
    """Return a box shape in the part frame (metres)."""
    return _BoxShape(
        _tuple(center), _tuple(size), None if color is None else _tuple(color)
    )


def cyl(center, radius, height, axis="z", color=None) -> _CylinderShape:
    """Return a cylinder shape in the part frame (metres)."""
    return _CylinderShape(
        _tuple(center),
        float(radius),
        float(height),
        axis,
        None if color is None else _tuple(color),
    )


def ball(center, radius, color=None) -> _BallShape:
    """Return a sphere shape in the part frame (metres)."""
    return _BallShape(
        _tuple(center), float(radius), None if color is None else _tuple(color)
    )


def _shape_bounds(shape: Shape) -> tuple[np.ndarray, np.ndarray]:
    center = np.asarray(shape.center, dtype=np.float64)
    if isinstance(shape, _BoxShape):
        half = np.abs(np.asarray(shape.size, dtype=np.float64)) / 2.0
    elif isinstance(shape, _CylinderShape):
        half = np.full(3, abs(shape.radius), dtype=np.float64)
        if shape.axis in ("x", "y", "z"):
            half["xyz".index(shape.axis)] = abs(shape.height) / 2.0
    elif isinstance(shape, _BallShape):
        half = np.full(3, abs(shape.radius), dtype=np.float64)
    else:
        raise TypeError(f"unsupported Part shape: {shape!r}")
    return center - half, center + half


def _corners(low: np.ndarray, high: np.ndarray) -> np.ndarray:
    return np.asarray(
        [
            (x, y, z)
            for x in (low[0], high[0])
            for y in (low[1], high[1])
            for z in (low[2], high[2])
        ],
        dtype=np.float64,
    )


def _cylinder_rotation(axis: str) -> np.ndarray:
    """Rotation from a canonical z-axis cylinder to ``axis``."""
    if axis == "x":
        return np.asarray(((0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0)))
    if axis == "y":
        return np.asarray(((1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, -1.0, 0.0)))
    return np.eye(3)


@dataclass(frozen=True, init=False)
class Part(Box):
    """A rigid compound part whose ``pos`` is its local frame's world origin."""

    size: tuple[float, float, float] = field(init=False)
    shapes: tuple[Shape, ...]
    friction: tuple[float, float] | None = None
    contact_offset: float = 0.002
    grip: int | None = None

    def __init__(
        self,
        name,
        pos,
        shapes,
        mass=None,
        color=GREY,
        yaw=0.0,
        friction=None,
        contact_offset=0.002,
        grip=None,
    ) -> None:
        frozen_shapes = tuple(shapes) if shapes is not None else ()
        object.__setattr__(self, "name", name)
        object.__setattr__(self, "pos", _tuple(pos))
        object.__setattr__(self, "color", _tuple(color))
        object.__setattr__(self, "mass", mass)
        object.__setattr__(self, "yaw", float(yaw))
        object.__setattr__(self, "shapes", frozen_shapes)
        object.__setattr__(
            self, "friction", None if friction is None else _tuple(friction)
        )
        object.__setattr__(self, "contact_offset", float(contact_offset))
        object.__setattr__(self, "grip", grip)

        if frozen_shapes:
            try:
                bounds = [_shape_bounds(shape) for shape in frozen_shapes]
                low = np.min([bound[0] for bound in bounds], axis=0)
                high = np.max([bound[1] for bound in bounds], axis=0)
                size = tuple(float(value) for value in high - low)
            except (TypeError, ValueError, IndexError):
                size = (0.0, 0.0, 0.0)
        else:
            size = (0.0, 0.0, 0.0)
        object.__setattr__(self, "size", size)

    def _check(self) -> None:
        if not self.shapes:
            raise ValueError(f"{self.name}.shapes must contain at least one shape")
        super()._check()
        if not math.isfinite(self.yaw):
            raise ValueError(f"{self.name}.yaw must be finite")
        if not math.isfinite(self.contact_offset) or self.contact_offset < 0.0:
            raise ValueError(
                f"{self.name}.contact_offset must be finite and non-negative"
            )
        if self.friction is not None and (
            len(self.friction) != 2
            or not all(math.isfinite(value) and value >= 0.0 for value in self.friction)
        ):
            raise ValueError(
                f"{self.name}.friction must be two finite non-negative numbers"
            )
        if self.grip is not None and (
            isinstance(self.grip, bool)
            or not isinstance(self.grip, int)
            or not 0 <= self.grip < len(self.shapes)
        ):
            raise ValueError(
                f"{self.name}.grip must index one of {len(self.shapes)} shapes"
            )

        for index, shape in enumerate(self.shapes):
            label = f"{self.name}.shapes[{index}]"
            if not isinstance(shape, (_BoxShape, _CylinderShape, _BallShape)):
                raise TypeError(
                    f"{label} is not a shape made by box(), cyl(), or ball()"
                )
            center = np.asarray(shape.center, dtype=np.float64)
            if center.shape != (3,) or not np.isfinite(center).all():
                raise ValueError(f"{label}.center must be three finite numbers")
            if isinstance(shape, _BoxShape):
                size = np.asarray(shape.size, dtype=np.float64)
                if (
                    size.shape != (3,)
                    or not np.isfinite(size).all()
                    or np.any(size <= 0.0)
                ):
                    raise ValueError(
                        f"{label}.size must be three positive finite numbers"
                    )
            elif isinstance(shape, _CylinderShape):
                if not (
                    math.isfinite(shape.radius) and math.isfinite(shape.height)
                ) or not (shape.radius > 0.0 and shape.height > 0.0):
                    raise ValueError(
                        f"{label}.radius and height must be positive finite numbers"
                    )
                if shape.axis not in ("x", "y", "z"):
                    raise ValueError(f"{label}.axis must be 'x', 'y', or 'z'")
            elif not math.isfinite(shape.radius) or shape.radius <= 0.0:
                raise ValueError(f"{label}.radius must be a positive finite number")
            if shape.color is not None:
                color = np.asarray(shape.color, dtype=np.float64)
                if (
                    color.shape != (3,)
                    or not np.isfinite(color).all()
                    or np.any((color < 0.0) | (color > 1.0))
                ):
                    raise ValueError(f"{label}.color must be three sRGB values in 0..1")

    def local_aabb(self) -> tuple[np.ndarray, np.ndarray]:
        """Return the part-frame axis-aligned bounds as ``(low, high)``."""
        if not self.shapes:
            zero = np.zeros(3, dtype=np.float64)
            return zero.copy(), zero
        bounds = [_shape_bounds(shape) for shape in self.shapes]
        return np.min([bound[0] for bound in bounds], axis=0), np.max(
            [bound[1] for bound in bounds], axis=0
        )

    def aabb(self, origin, R) -> tuple[np.ndarray, np.ndarray]:
        """Return conservative world-axis-aligned bounds as ``(low, high)``."""
        origin_array = np.asarray(origin, dtype=np.float64)
        rotation = np.asarray(R, dtype=np.float64)
        points = []
        for shape in self.shapes:
            low, high = _shape_bounds(shape)
            points.append(origin_array + _corners(low, high) @ rotation.T)
        if not points:
            return origin_array.copy(), origin_array.copy()
        joined = np.concatenate(points, axis=0)
        return joined.min(axis=0), joined.max(axis=0)

    def world_shapes(
        self, origin, R
    ) -> tuple[tuple[str, np.ndarray, np.ndarray, tuple[Any, ...]], ...]:
        """Return ``(kind, centre, rotation, dimensions)`` for every shape."""
        origin_array = np.asarray(origin, dtype=np.float64)
        rotation = np.asarray(R, dtype=np.float64)
        out = []
        for shape in self.shapes:
            center = origin_array + rotation @ np.asarray(
                shape.center, dtype=np.float64
            )
            if isinstance(shape, _BoxShape):
                out.append(("box", center, rotation.copy(), tuple(shape.size)))
            elif isinstance(shape, _CylinderShape):
                out.append(
                    (
                        "cyl",
                        center,
                        rotation @ _cylinder_rotation(shape.axis),
                        (shape.radius, shape.height),
                    )
                )
            else:
                out.append(("ball", center, rotation.copy(), (shape.radius,)))
        return tuple(out)

    def footprints(self, origin, R) -> tuple[np.ndarray, ...]:
        """Return one counter-clockwise 2-D convex footprint per shape."""
        origin_array = np.asarray(origin, dtype=np.float64)
        rotation = np.asarray(R, dtype=np.float64)
        polygons = []
        for shape in self.shapes:
            center = origin_array + rotation @ np.asarray(
                shape.center, dtype=np.float64
            )
            if isinstance(shape, _BoxShape):
                half_x, half_y = np.asarray(shape.size[:2], dtype=np.float64) / 2.0
                local = np.asarray(
                    (
                        (-half_x, -half_y, 0.0),
                        (half_x, -half_y, 0.0),
                        (half_x, half_y, 0.0),
                        (-half_x, half_y, 0.0),
                    )
                )
                polygons.append((center + local @ rotation.T)[:, :2])
                continue

            low, high = _shape_bounds(shape)
            relative = (
                _corners(
                    low - np.asarray(shape.center), high - np.asarray(shape.center)
                )
                @ rotation.T
            )
            extent = np.max(np.abs(relative[:, :2]), axis=0)
            angles = np.arange(16, dtype=np.float64) * (2.0 * math.pi / 16.0)
            polygons.append(
                center[:2]
                + np.column_stack(
                    (extent[0] * np.cos(angles), extent[1] * np.sin(angles))
                )
            )
        return tuple(polygons)

    def volume_centroid(self) -> np.ndarray:
        """Return the uniform-density collider-volume centroid in the part frame."""
        volumes = []
        centers = []
        for shape in self.shapes:
            if isinstance(shape, _BoxShape):
                volume = float(np.prod(shape.size))
            elif isinstance(shape, _CylinderShape):
                volume = math.pi * shape.radius**2 * shape.height
            else:
                volume = 4.0 * math.pi * shape.radius**3 / 3.0
            volumes.append(volume)
            centers.append(np.asarray(shape.center, dtype=np.float64))
        if not volumes or sum(volumes) <= 0.0:
            raise ValueError(f"{self.name} has no positive-volume shapes")
        return np.average(np.asarray(centers), axis=0, weights=np.asarray(volumes))

    def grip_shape(self) -> Shape:
        """Return the explicitly selected grip shape, or the first shape."""
        return self.shapes[0 if self.grip is None else self.grip]


@dataclass(frozen=True)
class PartState(ObjectState):
    """Simulator state for a Part, including its root-frame origin."""

    origin: np.ndarray

    def rotation(self) -> np.ndarray:
        """Return the part-to-world rotation matrix from ``quat_xyzw``."""
        quat = np.asarray(self.quat_xyzw, dtype=np.float64)
        norm = np.linalg.norm(quat)
        if not norm > 0.0:
            raise ValueError("quat_xyzw must have non-zero length")
        x, y, z, w = quat / norm
        return np.asarray(
            (
                (
                    1.0 - 2.0 * (y * y + z * z),
                    2.0 * (x * y - z * w),
                    2.0 * (x * z + y * w),
                ),
                (
                    2.0 * (x * y + z * w),
                    1.0 - 2.0 * (x * x + z * z),
                    2.0 * (y * z - x * w),
                ),
                (
                    2.0 * (x * z - y * w),
                    2.0 * (y * z + x * w),
                    1.0 - 2.0 * (x * x + y * y),
                ),
            )
        )

    def to_world(self, p_local) -> np.ndarray:
        """Transform a part-frame point into the world frame."""
        return np.asarray(self.origin, dtype=np.float64) + self.rotation() @ np.asarray(
            p_local, dtype=np.float64
        )

    def to_local(self, p_world) -> np.ndarray:
        """Transform a world-frame point into the part frame."""
        return self.rotation().T @ (
            np.asarray(p_world, dtype=np.float64)
            - np.asarray(self.origin, dtype=np.float64)
        )

    def tilt_deg(self) -> float:
        """Return the angle in degrees between the part z axis and world z."""
        return math.degrees(math.acos(float(np.clip(self.rotation()[2, 2], -1.0, 1.0))))


_PART_NAMES: set[str] = set()
_SPECS: dict[str, Part] = {}  # Parts by name, for the spawner: a frozen Part must not live inside an Isaac Lab config
_SPAWNER_CFG_TYPE = None
_CLONED_SPAWN_PART = None
_PATCH_TOKEN = object()


def _part_object(spec: Part):
    """Build an Arena Object for a Part (GPU runtime only)."""
    global _CLONED_SPAWN_PART, _SPAWNER_CFG_TYPE

    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    if _SPAWNER_CFG_TYPE is None:

        @configclass
        class PartSpawnerCfg(sim_utils.SpawnerCfg):
            part_name: str = ""  # Isaac Lab copies configs field by field, which a frozen Part refuses

        _SPAWNER_CFG_TYPE = PartSpawnerCfg
    if _CLONED_SPAWN_PART is None:
        _CLONED_SPAWN_PART = clone(_spawn_part)
    _SPECS[spec.name] = spec
    cfg = _SPAWNER_CFG_TYPE(func=_CLONED_SPAWN_PART, part_name=spec.name)
    half_yaw = spec.yaw / 2.0
    return Object(
        name=spec.name,
        object_type=ObjectType.RIGID if spec.free else ObjectType.BASE,
        spawner_cfg=cfg,
        initial_pose=Pose(
            position_xyz=tuple(spec.pos),
            rotation_xyzw=(0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)),
        ),
    )


def _spawn_part(prim_path: str, cfg, *args, **kwargs):
    """Author a Part's compound USD body (called only on an Isaac Lab host)."""
    import isaaclab.sim as sim_utils
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    part = _SPECS[cfg.part_name]
    stage = kwargs.get("stage") or sim_utils.get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)

    translation = kwargs.get("translation", args[0] if len(args) >= 1 else None)
    orientation = kwargs.get("orientation", args[1] if len(args) >= 2 else None)
    if translation is not None:
        root.AddTranslateOp().Set(Gf.Vec3d(*(float(value) for value in translation)))
    if orientation is not None:
        # Isaac Lab 3 passes spawn orientations as (x, y, z, w); USD's orient op wants a quatd (w, xyz).
        x, y, z, w = (float(value) for value in orientation)
        root.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))

    static_friction, dynamic_friction = part.friction or (1.2, 1.0)
    physics_path = f"{prim_path}/physics_material"
    physics = sim_utils.RigidBodyMaterialCfg(
        static_friction=static_friction,
        dynamic_friction=dynamic_friction,
    )
    physics.func(physics_path, physics)

    for index, shape in enumerate(part.shapes):
        path = f"{prim_path}/shape_{index}"
        if isinstance(shape, _BoxShape):
            geom = UsdGeom.Cube.Define(stage, path)
            geom.CreateSizeAttr(1.0)
            geom.AddTranslateOp().Set(Gf.Vec3d(*shape.center))
            geom.AddScaleOp().Set(Gf.Vec3f(*shape.size))
        elif isinstance(shape, _CylinderShape):
            geom = UsdGeom.Cylinder.Define(stage, path)
            geom.CreateRadiusAttr(shape.radius)
            geom.CreateHeightAttr(shape.height)
            geom.CreateAxisAttr(getattr(UsdGeom.Tokens, shape.axis))
            geom.AddTranslateOp().Set(Gf.Vec3d(*shape.center))
        else:
            geom = UsdGeom.Sphere.Define(stage, path)
            geom.CreateRadiusAttr(shape.radius)
            geom.AddTranslateOp().Set(Gf.Vec3d(*shape.center))

        prim = geom.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        collision.CreateContactOffsetAttr().Set(part.contact_offset)
        collision.CreateRestOffsetAttr().Set(0.0)
        sim_utils.bind_physics_material(path, physics_path, stage=stage)

        color = part.color if shape.color is None else shape.color
        visual_path = f"{prim_path}/visual_material_{index}"
        visual = sim_utils.PreviewSurfaceCfg(
            diffuse_color=linear_rgb(color), roughness=0.4
        )
        visual.func(visual_path, visual)
        sim_utils.bind_visual_material(path, visual_path, stage=stage)

    root_prim = root.GetPrim()
    if part.free:
        UsdPhysics.RigidBodyAPI.Apply(root_prim)
        mass = UsdPhysics.MassAPI.Apply(root_prim)
        mass.CreateMassAttr().Set(part.mass)
        rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(root_prim)
        rigid.CreateSolverPositionIterationCountAttr().Set(16)
        rigid.CreateSolverVelocityIterationCountAttr().Set(4)
        rigid.CreateMaxDepenetrationVelocityAttr().Set(1.0)
    return root_prim


def _patch_scene(scene) -> None:
    if getattr(scene, "_yam_parts_patch_token", None) is _PATCH_TOKEN:
        return

    original_task_object = getattr(
        scene.task_object, "_yam_parts_original", scene.task_object
    )
    original_object_states = getattr(
        scene.object_states, "_yam_parts_original", scene.object_states
    )

    @functools.wraps(original_task_object)
    def task_object(spec):
        if isinstance(spec, Part):
            _PART_NAMES.add(spec.name)
            return _part_object(spec)
        _PART_NAMES.discard(getattr(spec, "name", ""))
        return original_task_object(spec)

    @functools.wraps(original_object_states)
    def object_states(env, names):
        states = original_object_states(env, names)
        part_names = _PART_NAMES.intersection(names)
        if not part_names:
            return states

        base = env.unwrapped
        world = base.arena_world
        env_origin = base.scene.env_origins[0]

        def array(tensor) -> np.ndarray:
            return tensor.detach().cpu().numpy().astype(np.float64)

        for name in part_names:
            state = states[name]
            states[name] = PartState(
                pos=state.pos,
                quat_xyzw=state.quat_xyzw,
                lin_vel=state.lin_vel,
                ang_vel=state.ang_vel,
                low=state.low,
                high=state.high,
                origin=array(world.get_pose_w(name)[0, :3] - env_origin),
            )
        return states

    task_object._yam_parts_original = original_task_object
    object_states._yam_parts_original = original_object_states
    scene.task_object = task_object
    scene.object_states = object_states
    scene._yam_parts_patch_token = _PATCH_TOKEN


class _SceneLoader(importlib.abc.Loader):
    def __init__(self, loader, patch) -> None:
        self.loader = loader
        self.patch = patch

    def create_module(self, spec):
        create = getattr(self.loader, "create_module", None)
        return None if create is None else create(spec)

    def exec_module(self, module) -> None:
        self.loader.exec_module(module)
        self.patch(module)


class _SceneFinder(importlib.abc.MetaPathFinder):
    _yam_parts_scene_finder = True

    def __init__(self, patch) -> None:
        self.patch = patch

    def find_spec(self, fullname, path=None, target=None):
        if fullname != "yam_arena.scene":
            return None
        spec = importlib.machinery.PathFinder.find_spec(fullname, path)
        if spec is None or spec.loader is None:
            return spec
        spec.loader = _SceneLoader(spec.loader, self.patch)
        return spec


def install() -> None:
    """Install the idempotent lazy patch for ``yam_arena.scene``."""
    finder = next(
        (
            item
            for item in sys.meta_path
            if getattr(item, "_yam_parts_scene_finder", False)
        ),
        None,
    )
    if finder is None:
        sys.meta_path.insert(0, _SceneFinder(_patch_scene))
    else:
        finder.patch = _patch_scene
    scene = sys.modules.get("yam_arena.scene")
    if scene is not None:
        _patch_scene(scene)


install()


__all__ = ["Part", "PartState", "ball", "box", "cyl", "install"]
