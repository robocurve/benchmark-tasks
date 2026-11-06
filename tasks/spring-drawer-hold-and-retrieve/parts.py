"""parts: compound rigid parts and spring-loaded sliders for yam_arena task files.

The hooking technique (patching ``yam_arena.scene`` and the embodiment from a task file's import, without modifying
yam_arena) follows the approach first shown in robocurve/benchmark-tasks PR #40 (tasks/bearing-press/partkit.py).
This module is an independent implementation of it for this task.

Use it from a task file that sits next to this one:

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).parent))
    from parts import Part, Slider, part, PartState, on_step, on_reset, env, gripper_open

    BLOCK = part("block", pos=(0.28, 0.10, 0.78), mass=0.30, boxes=[((0, 0, 0), (0.085, 0.11, 0.06), (0.6, 0.6, 0.6))])
    OBJECTS = [BLOCK]

yam_arena task files only take single primitives (Box, Cylinder, Sphere, Asset). A :class:`Part` is a Box subclass (so
the task-file checks accept it) that describes ONE rigid body made of many box and cylinder colliders: a block with a
hollow socket, a plug with a key rib. Give it :class:`Slider` s and it becomes an articulation: the part is the
floating root and each slider is a small rigid body on a prismatic joint along one of the part's axes, pushed back to
its rest stop by a joint-drive spring (a spring plunger).

Coordinates. Every shape centre, in the part AND in its sliders, is given in the part frame at rest, in metres. A slider's
``rest`` is only where its own frame (and its joint anchor) sits in the part frame. ``pos`` is the part's BODY ORIGIN in
the world frame, not its bounding-box centre; ``yaw`` turns the part about the vertical.

What importing this module does: it calls :func:`install`, which hooks ``yam_arena.scene.task_object`` (so a Part is
spawned as its compound body) and ``yam_arena.scene.object_states`` (so ``success()``/``score()`` receive a
:class:`PartState` for it), and ``YamArenaEmbodiment._task_step`` / ``reset`` (so :func:`on_step` / :func:`on_reset`
callbacks run). The hooks work whether ``yam_arena.scene`` was imported before or is imported later.

Everything that needs Isaac Sim, Isaac Lab, Arena or pxr is imported lazily inside functions, so importing this module
needs only numpy and the pure-Python parts of yam_arena.
"""

from __future__ import annotations

import functools
import importlib.abc
import logging
import math
import re
import sys
from dataclasses import dataclass, field, replace

import numpy as np
from yam_arena.constants import FINGER_JOINTS, FINGER_OPEN
from yam_arena.task_file import Box, ObjectState

_LOG = logging.getLogger("task_parts")

__all__ = [
    "Part",
    "PartState",
    "Slider",
    "author_part",
    "env",
    "gripper_open",
    "install",
    "on_reset",
    "on_step",
    "part",
    "write_pose",
]

# -- physics settings applied to every body and collider ---------------------------------------------------------------
CONTACT_OFFSET = 0.0005  # m; below the 1 mm fits, so tight parts do not touch on every face at once
REST_OFFSET = 0.0
SOLVER_POSITION_ITERATIONS = 32
SOLVER_VELOCITY_ITERATIONS = 4  # articulation only
ANGULAR_DAMPING = 1.0  # light: small stiff parts otherwise chatter in tight fits
MAX_DEPENETRATION_VELOCITY = 1.0  # m/s

_AXES = ("X", "Y", "Z", "+X", "+Y", "+Z", "-X", "-Y", "-Z")
_PRIM_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_RESERVED_SLIDER_NAMES = frozenset({"body", "Looks"})  # prims the part already owns under its root


# -- shape specs -----------------------------------------------------------------------------------------------------


def _vec3(value, what: str) -> tuple[float, float, float]:
    try:
        out = tuple(float(v) for v in value)
    except TypeError as exc:
        raise ValueError(f"{what} must be 3 numbers, got {value!r}") from exc
    if len(out) != 3 or not all(math.isfinite(v) for v in out):
        raise ValueError(f"{what} must be 3 finite numbers, got {value!r}")
    return out


def _canonical_box(spec, what: str):
    """``(centre, size, colour)`` as three float 3-tuples."""
    try:
        centre, size, colour = spec
    except (TypeError, ValueError):
        raise ValueError(f"{what}: a box is (centre, size, colour), got {spec!r}") from None
    return _vec3(centre, f"{what} box centre"), _vec3(size, f"{what} box size"), _vec3(colour, f"{what} box colour")


def _canonical_cylinder(spec, what: str):
    """``(centre, radius, height, colour, axis)``; the axis defaults to "Z" when the spec has only four entries."""
    try:
        if len(spec) == 4:
            centre, radius, height, colour = spec
            axis = "Z"
        else:
            centre, radius, height, colour, axis = spec
    except (TypeError, ValueError):
        raise ValueError(
            f"{what}: a cylinder is (centre, radius, height, colour[, axis]), got {spec!r}"
        ) from None
    return (
        _vec3(centre, f"{what} cylinder centre"),
        float(radius),
        float(height),
        _vec3(colour, f"{what} cylinder colour"),
        str(axis).upper(),
    )


def _cylinder_half_extent(radius: float, height: float, axis: str) -> np.ndarray:
    half = np.full(3, radius)
    half[_AXES.index(axis)] = height / 2
    return half


def _check_shapes(boxes, cylinders, what: str) -> None:
    for i, (_, size, colour) in enumerate(boxes):
        if any(s <= 0 for s in size):
            raise ValueError(f"{what}: box {i} size must be positive, got {size!r}")
        if any(not 0.0 <= c <= 1.0 for c in colour):
            raise ValueError(f"{what}: box {i} colour must be sRGB in 0..1, got {colour!r}")
    for i, (_, radius, height, colour, axis) in enumerate(cylinders):
        if not (math.isfinite(radius) and math.isfinite(height) and radius > 0 and height > 0):
            raise ValueError(f"{what}: cylinder {i} radius and height must be positive, got {radius!r}, {height!r}")
        if axis not in _AXES:
            raise ValueError(f"{what}: cylinder {i} axis must be one of {_AXES}, got {axis!r}")
        if any(not 0.0 <= c <= 1.0 for c in colour):
            raise ValueError(f"{what}: cylinder {i} colour must be sRGB in 0..1, got {colour!r}")


def _extent(boxes, cylinders) -> list[tuple[np.ndarray, np.ndarray]]:
    """(low, high) corners of every shape, part frame."""
    out = []
    for centre, size, _ in boxes:
        c, half = np.array(centre), np.array(size) / 2
        out.append((c - half, c + half))
    for centre, radius, height, _, axis in cylinders:
        c, half = np.array(centre), _cylinder_half_extent(radius, height, axis)
        out.append((c - half, c + half))
    return out


@dataclass(frozen=True)
class Slider:
    """Shapes on a prismatic joint along the part's ``axis`` ("X", "Y" or "Z").

    ``rest`` is where the slider's frame sits in the part frame (the joint anchor); the shapes' centres are in the part
    frame at rest. The slider can move ``travel`` metres in the axis's NEGATIVE direction from rest, and a spring of
    ``rate`` N/m (plus ``damping`` N s/m) pushes it back to rest. Its joint position is 0 at rest, negative when pressed.
    """

    name: str
    rest: tuple[float, float, float]
    boxes: tuple = ()
    cylinders: tuple = ()
    travel: float = 0.004
    rate: float = 2500.0
    damping: float = 20.0
    mass: float = 0.002
    axis: str = "Z"

    def __post_init__(self) -> None:
        what = f"slider {self.name!r}"
        object.__setattr__(self, "rest", _vec3(self.rest, f"{what} rest"))
        object.__setattr__(self, "boxes", tuple(_canonical_box(b, what) for b in self.boxes))
        object.__setattr__(self, "cylinders", tuple(_canonical_cylinder(c, what) for c in self.cylinders))
        object.__setattr__(self, "axis", str(self.axis).upper())

    def _check(self, owner: str) -> None:
        what = f"{owner}: slider {self.name!r}"
        if not isinstance(self.name, str) or not _PRIM_NAME.match(self.name) or self.name in _RESERVED_SLIDER_NAMES:
            raise ValueError(
                f"{what}: a slider name must be a plain identifier other than {sorted(_RESERVED_SLIDER_NAMES)}"
            )
        if self.axis not in _AXES:
            raise ValueError(f"{what}: axis must be one of {_AXES}, got {self.axis!r}")
        for label, value, strict in (("travel", self.travel, True), ("rate", self.rate, True),
                                     ("mass", self.mass, True), ("damping", self.damping, False)):
            if not (math.isfinite(value) and (value > 0 if strict else value >= 0)):
                raise ValueError(f"{what}: {label} must be {'positive' if strict else 'non-negative'}, got {value!r}")
        if not (self.boxes or self.cylinders):
            raise ValueError(f"{what}: a slider needs at least one box or cylinder")
        _check_shapes(self.boxes, self.cylinders, what)


@dataclass(frozen=True)
class Part(Box):
    """One rigid body made of ``boxes`` and ``cylinders`` (part frame), optionally with spring ``sliders``.

    It subclasses Box only so yam_arena's task-file checks accept it. ``size`` is the bounding box of all its shapes
    (sliders at rest included) and ``pos`` is the BODY ORIGIN in the world frame. ``mass`` is the part's own mass; the
    friction is one physics material shared by every shape (static = dynamic). Build it with :func:`part`.
    """

    boxes: tuple = ()  # ((centre), (size), (sRGB colour)), ...
    cylinders: tuple = ()  # ((centre), radius, height, (sRGB colour), axis), ...
    sliders: tuple = ()
    friction: float = 0.8
    collision: bool = True

    def __post_init__(self) -> None:
        what = f"part {self.name!r}"
        for key in ("pos", "color", "size"):
            object.__setattr__(self, key, _vec3(getattr(self, key), f"{what} {key}"))
        object.__setattr__(self, "boxes", tuple(_canonical_box(b, what) for b in self.boxes))
        object.__setattr__(self, "cylinders", tuple(_canonical_cylinder(c, what) for c in self.cylinders))
        object.__setattr__(self, "sliders", tuple(self.sliders))

    def _check(self) -> None:
        if not self.boxes:
            raise ValueError(f"{self.name}: a Part needs at least one box")
        if self.mass is None or not self.mass > 0:
            raise ValueError(f"{self.name}: a Part must be loose, give it a positive mass (got {self.mass!r})")
        super()._check()
        if not (math.isfinite(self.friction) and self.friction > 0):
            raise ValueError(f"{self.name}: friction must be positive, got {self.friction!r}")
        if not math.isfinite(self.yaw):
            raise ValueError(f"{self.name}: yaw must be finite, got {self.yaw!r}")
        _check_shapes(self.boxes, self.cylinders, self.name)
        names = [s.name for s in self.sliders if isinstance(s, Slider)]
        if len(names) != len(self.sliders):
            raise TypeError(f"{self.name}: sliders must be Slider objects")
        if len(set(names)) != len(names):
            raise ValueError(f"{self.name}: slider names must be unique, got {names}")
        for s in self.sliders:
            s._check(self.name)

    def local_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        """(low, high) corners of the axis-aligned box around every shape, sliders at rest, in the part frame."""
        spans = _extent(self.boxes, self.cylinders)
        for s in self.sliders:
            spans += _extent(s.boxes, s.cylinders)
        if not spans:
            return np.zeros(3), np.zeros(3)
        return np.min([lo for lo, _ in spans], axis=0), np.max([hi for _, hi in spans], axis=0)


def part(name, pos, boxes, mass, yaw=0.0, sliders=(), color=(0.5, 0.5, 0.5), friction=0.8, cylinders=(), collision=True) -> Part:
    """Build a :class:`Part`; its ``size`` is filled in from the shapes' extent.

    ``boxes`` is a list of ``(centre, size, colour)`` and ``cylinders`` a list of ``(centre, radius, height, colour,
    axis)`` (axis "X", "Y" or "Z", default "Z"), all in the part frame in metres with sRGB colours. ``pos`` is the body
    origin in the world frame. Validation happens in ``Part._check``, which yam_arena's task-file loader calls.
    """
    draft = Part(name=name, pos=pos, mass=mass, yaw=yaw, color=color, boxes=tuple(boxes), cylinders=tuple(cylinders),
                 sliders=tuple(sliders), friction=friction, collision=collision)
    low, high = draft.local_bounds()
    return replace(draft, size=tuple((high - low).tolist()))


# -- what success() and score() receive ------------------------------------------------------------------------------


def _quat_to_matrix(quat_xyzw) -> np.ndarray:
    q = np.asarray(quat_xyzw, dtype=float)
    norm = np.linalg.norm(q)
    if not norm > 0:
        raise ValueError(f"cannot build a rotation from the quaternion {quat_xyzw!r}")
    x, y, z, w = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


@dataclass(frozen=True)
class PartState(ObjectState):
    """A part as the simulator has it: ``pos`` and ``quat_xyzw`` are the BODY ORIGIN's pose (world frame, relative to the
    env origin), ``low``/``high`` the world-aligned box around the part's bounding-box corners, and ``sliders`` each
    slider's joint position in metres (0 at rest, negative when pressed)."""

    sliders: dict = field(default_factory=dict)

    def rotation(self) -> np.ndarray:
        """3x3 matrix taking part-frame vectors to the world frame."""
        return _quat_to_matrix(self.quat_xyzw)

    def to_local(self, world_point) -> np.ndarray:
        """A world point (3,) or a stack (N, 3) in the part frame."""
        return (np.asarray(world_point, dtype=float) - np.asarray(self.pos, dtype=float)) @ self.rotation()

    def to_world(self, local_point) -> np.ndarray:
        """A part-frame point (3,) or a stack (N, 3) in the world frame."""
        return np.asarray(local_point, dtype=float) @ self.rotation().T + np.asarray(self.pos, dtype=float)


def _state_from_arrays(spec: Part, pose_w, vel_w, origin, joint_names=None, joint_pos=None) -> PartState:
    """Assemble a PartState from raw simulator arrays (env 0): root link pose [x y z qx qy qz qw] and COM velocity
    [vx vy vz wx wy wz] in the world frame, the env origin, and (for parts with sliders) the joint names and positions."""
    pose, vel, origin = (np.asarray(a, dtype=np.float64).reshape(-1) for a in (pose_w, vel_w, origin))
    if pose.size != 7 or vel.size != 6 or origin.size != 3:
        raise ValueError(f"{spec.name}: expected a 7-D pose, 6-D velocity and 3-D origin, got {pose.size}, {vel.size}, {origin.size}")
    state = PartState(pos=pose[:3] - origin, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6],
                      low=np.zeros(3), high=np.zeros(3))
    low, high = spec.local_bounds()
    corners = np.array([[x, y, z] for x in (low[0], high[0]) for y in (low[1], high[1]) for z in (low[2], high[2])])
    world = state.to_world(corners)
    sliders = {}
    if spec.sliders:
        names = list(joint_names or [])
        q = np.asarray(joint_pos, dtype=np.float64).reshape(-1)
        for s in spec.sliders:
            joint = f"{s.name}_slide"
            if joint not in names:
                raise RuntimeError(f"{spec.name}: the simulator has no joint {joint!r} (it has {names}); "
                                   "was the part spawned without its sliders?")
            sliders[s.name] = float(q[names.index(joint)])
    return PartState(pos=state.pos, quat_xyzw=state.quat_xyzw, lin_vel=state.lin_vel, ang_vel=state.ang_vel,
                     low=world.min(axis=0), high=world.max(axis=0), sliders=sliders)


# -- USD authoring (pxr only; testable on a bare stage) ----------------------------------------------------------------


class _SchemaWriter:
    """Applies the PhysX schemas through ``pxr.PhysxSchema`` (available inside Isaac Sim)."""

    def __init__(self, schema) -> None:
        self._schema = schema

    def rigid_body(self, prim) -> None:
        api = self._schema.PhysxRigidBodyAPI.Apply(prim)
        api.CreateMaxDepenetrationVelocityAttr(MAX_DEPENETRATION_VELOCITY)
        api.CreateSolverPositionIterationCountAttr(SOLVER_POSITION_ITERATIONS)
        api.CreateAngularDampingAttr(ANGULAR_DAMPING)

    def collider(self, prim) -> None:
        api = self._schema.PhysxCollisionAPI.Apply(prim)
        api.CreateContactOffsetAttr(CONTACT_OFFSET)
        api.CreateRestOffsetAttr(REST_OFFSET)

    def articulation(self, prim) -> None:
        api = self._schema.PhysxArticulationAPI.Apply(prim)
        api.CreateEnabledSelfCollisionsAttr(False)
        api.CreateSolverPositionIterationCountAttr(SOLVER_POSITION_ITERATIONS)
        api.CreateSolverVelocityIterationCountAttr(SOLVER_VELOCITY_ITERATIONS)


class _PlainUsdWriter:
    """The same PhysX attributes written as plain USD (applied-schema token plus attributes), for stages built without
    ``pxr.PhysxSchema`` (a bare ``usd-core`` install). Inside Isaac Sim :class:`_SchemaWriter` is used instead."""

    @staticmethod
    def _write(prim, api: str, prefix: str, values: dict) -> None:
        prim.AddAppliedSchema(api)
        for name, (type_name, value) in values.items():
            prim.CreateAttribute(f"{prefix}:{name}", type_name).Set(value)

    def rigid_body(self, prim) -> None:
        from pxr import Sdf

        t = Sdf.ValueTypeNames
        self._write(prim, "PhysxRigidBodyAPI", "physxRigidBody", {
            "maxDepenetrationVelocity": (t.Float, MAX_DEPENETRATION_VELOCITY),
            "solverPositionIterationCount": (t.UInt, SOLVER_POSITION_ITERATIONS),
            "angularDamping": (t.Float, ANGULAR_DAMPING),
        })

    def collider(self, prim) -> None:
        from pxr import Sdf

        t = Sdf.ValueTypeNames
        self._write(prim, "PhysxCollisionAPI", "physxCollision", {
            "contactOffset": (t.Float, CONTACT_OFFSET),
            "restOffset": (t.Float, REST_OFFSET),
        })

    def articulation(self, prim) -> None:
        from pxr import Sdf

        t = Sdf.ValueTypeNames
        self._write(prim, "PhysxArticulationAPI", "physxArticulation", {
            "enabledSelfCollisions": (t.Bool, False),
            "solverPositionIterationCount": (t.UInt, SOLVER_POSITION_ITERATIONS),
            "solverVelocityIterationCount": (t.UInt, SOLVER_VELOCITY_ITERATIONS),
        })


def _physx_writer():
    try:
        from pxr import PhysxSchema
    except ImportError:
        return _PlainUsdWriter()
    return _SchemaWriter(PhysxSchema)


def author_part(stage, prim_path: str, spec: Part, bind_physics, bind_visual, translation=None, orientation_xyzw=None,
                physx=None):
    """Author ``spec`` under ``prim_path`` on ``stage`` and return the root prim.

    ``bind_physics(shape_path)`` must bind the part's one physics material to a collider, and ``bind_visual(shape_path,
    srgb_colour)`` a look to it; Isaac Lab supplies them in :func:`_isaac_spawn`, a test can pass recorders. The layout::

        <prim_path>                    Xform at ``translation``/``orientation_xyzw`` (the part's pose in the world)
          body                         rigid body (+ articulation root if there are sliders); box_i / cyl_i colliders
          <slider>                     one rigid body per slider, a sibling of ``body``; box_i / cyl_i colliders
            <slider>_slide             prismatic joint ``body`` -> slider, limits [-travel, 0], spring drive to 0
          Looks/...                    materials (made by the callables)

    ``physx`` writes the PhysX attributes (offsets, solver iterations, damping); by default it uses ``pxr.PhysxSchema``
    if that imports and plain USD attributes otherwise.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    physx = physx if physx is not None else _physx_writer()
    double = UsdGeom.XformOp.PrecisionDouble

    def place(xform, position, quat_xyzw=(0.0, 0.0, 0.0, 1.0)) -> None:
        x, y, z, w = (float(v) for v in quat_xyzw)
        xform.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(float(v) for v in position)))
        xform.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))

    def rigid_body(path: str, position, mass: float):
        xform = UsdGeom.Xform.Define(stage, path)
        place(xform, position)
        prim = xform.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        physx.rigid_body(prim)
        if not spec.collision:
            # Display-only marker (teleported by write_pose each step): kinematic, so gravity cannot drop it
            # between the pose write and the camera render.
            UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
        return prim

    def make_collider(geom, colour) -> None:
        prim = geom.GetPrim()
        if spec.collision:
            UsdPhysics.CollisionAPI.Apply(prim)
            physx.collider(prim)
            bind_physics(str(prim.GetPath()))
        bind_visual(str(prim.GetPath()), tuple(colour))

    def add_shapes(parent: str, boxes, cylinders, frame_origin) -> None:
        origin = np.array(frame_origin)
        for i, (centre, size, colour) in enumerate(boxes):
            cube = UsdGeom.Cube.Define(stage, f"{parent}/box_{i}")
            cube.CreateSizeAttr(1.0)  # a unit cube, scaled to the box's edge lengths
            cube.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - origin).tolist()))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            make_collider(cube, colour)
        for i, (centre, radius, height, colour, axis) in enumerate(cylinders):
            cyl = UsdGeom.Cylinder.Define(stage, f"{parent}/cyl_{i}")
            cyl.CreateAxisAttr(axis)
            cyl.CreateRadiusAttr(radius)
            cyl.CreateHeightAttr(height)
            cyl.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - origin).tolist()))
            make_collider(cyl, colour)

    root = UsdGeom.Xform.Define(stage, prim_path)
    place(root, translation if translation is not None else (0.0, 0.0, 0.0),
          orientation_xyzw if orientation_xyzw is not None else (0.0, 0.0, 0.0, 1.0))

    body_path = f"{prim_path}/body"
    body = rigid_body(body_path, (0.0, 0.0, 0.0), spec.mass)
    add_shapes(body_path, spec.boxes, spec.cylinders, (0.0, 0.0, 0.0))

    for s in spec.sliders:
        slider_path = f"{prim_path}/{s.name}"
        rigid_body(slider_path, s.rest, s.mass)
        add_shapes(slider_path, s.boxes, s.cylinders, s.rest)  # shape centres are part-frame; the body sits at rest
        joint = UsdPhysics.PrismaticJoint.Define(stage, f"{slider_path}/{s.name}_slide")
        axis_name = s.axis[-1]
        joint.CreateAxisAttr(axis_name)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(body_path)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(slider_path)])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*s.rest))  # the anchor on the body is where the slider's frame sits
        joint.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        if s.axis.startswith("+"):
            joint.CreateLowerLimitAttr(0.0)
            joint.CreateUpperLimitAttr(s.travel)
        else:
            joint.CreateLowerLimitAttr(-s.travel)
            joint.CreateUpperLimitAttr(0.0)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(s.rate)
        drive.CreateDampingAttr(s.damping)
        drive.CreateTargetPositionAttr(0.0)

    if spec.sliders:
        UsdPhysics.ArticulationRootAPI.Apply(body)
        physx.articulation(body)  # the plunger overlaps its wall collider by design: no self-collisions
    return root.GetPrim()


# -- Isaac glue (lazy imports) ---------------------------------------------------------------------------------------

_PARTS: dict[str, Part] = {}  # the Parts the scene was asked to build, by name


def _isaac_material_binders(stage, root_path: str, friction: float):
    """``(bind_physics, bind_visual)`` for :func:`author_part`: materials are made on first use, under ``<root>/Looks``."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material

    from yam_arena.station import linear_rgb

    physics_path = f"{root_path}/Looks/physics"
    looks: dict[tuple, str] = {}
    made = []

    def bind_physics(shape_path: str) -> None:
        if not made:
            cfg = sim_utils.RigidBodyMaterialCfg(static_friction=friction, dynamic_friction=friction)
            cfg.func(physics_path, cfg)
            made.append(physics_path)
        bind_physics_material(shape_path, physics_path, stage=stage)

    def bind_visual(shape_path: str, colour: tuple) -> None:
        if colour not in looks:
            look_path = f"{root_path}/Looks/colour_{len(looks)}"
            cfg = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(colour), roughness=0.5)
            cfg.func(look_path, cfg)
            looks[colour] = look_path
        bind_visual_material(shape_path, looks[colour], stage=stage)

    return bind_physics, bind_visual


def _isaac_spawn(prim_path: str, cfg, translation=None, orientation=None, **_unused):
    """Isaac Lab spawner function (wrapped with ``clone`` in :func:`_make_arena_object`): build the part's USD."""
    from isaaclab.sim.utils.stage import get_current_stage

    if cfg.part_name not in _PARTS:
        raise RuntimeError(f"parts: no Part named {cfg.part_name!r} was registered by yam_arena.scene.task_object")
    spec = _PARTS[cfg.part_name]
    stage = get_current_stage()
    bind_physics, bind_visual = _isaac_material_binders(stage, prim_path, spec.friction)
    return author_part(stage, prim_path, spec, bind_physics, bind_visual, translation=translation,
                       orientation_xyzw=orientation)


def _make_arena_object(spec: Part):
    """The Arena object for a Part: a rigid object, or an articulation (part + sliders) with spring actuators."""
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class PartSpawnerCfg(SpawnerCfg):
        func: object = clone(_isaac_spawn)
        part_name: str = ""

    class PartObject(Object):
        def _generate_articulation_cfg(self):
            # Implicit actuators carry the springs (stiffness = rate); their position target stays at the joint's
            # default of 0, which is the slider's rest stop.
            actuators = {
                s.name: ImplicitActuatorCfg(joint_names_expr=[f"{s.name}_slide"], stiffness=s.rate, damping=s.damping)
                for s in spec.sliders
            }
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    return PartObject(
        name=spec.name,
        object_type=ObjectType.ARTICULATION if spec.sliders else ObjectType.RIGID,
        spawner_cfg=PartSpawnerCfg(part_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos),
                          rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))),
    )


def _to_numpy(value) -> np.ndarray:
    """A float64 array from an Isaac Lab 3.0 data field (a wrapper with ``.torch``), a torch tensor or an array."""
    value = getattr(value, "torch", value)
    for step in ("detach", "cpu", "numpy"):
        if hasattr(value, step):
            value = getattr(value, step)()
    return np.asarray(value, dtype=np.float64)


def _read_part_state(environment, spec: Part) -> PartState:
    base = environment.unwrapped
    try:
        asset = base.scene[spec.name]
    except KeyError as exc:
        raise RuntimeError(f"parts: the scene has no object {spec.name!r}") from exc
    origin = _to_numpy(base.scene.env_origins[0])
    pose = _to_numpy(asset.data.root_link_pose_w)[0]  # the body origin's pose, not the centre of mass
    vel = _to_numpy(asset.data.root_vel_w)[0]
    names = list(asset.joint_names) if spec.sliders else None
    joint_pos = _to_numpy(asset.data.joint_pos)[0] if spec.sliders else None
    return _state_from_arrays(spec, pose, vel, origin, names, joint_pos)


_WRITE_POSE_WARNED: set[str] = set()
_WRITE_POSE_OK: set[str] = set()


def _write_pose_warn(name: str, why: str) -> None:
    """Log a failed teleport ONCE per object and reason, so a dead lamp/readout is visible in the run log."""
    key = f"{name}:{why}"
    if key not in _WRITE_POSE_WARNED:
        _WRITE_POSE_WARNED.add(key)
        _LOG.warning("write_pose(%s) failed: %s; the display marker will NOT move (scoring is unaffected)", name, why)


def write_pose(environment, name: str, pos, quat_xyzw=(0.0, 0.0, 0.0, 1.0), lin_vel=(0.0, 0.0, 0.0), ang_vel=(0.0, 0.0, 0.0)) -> bool:
    """Teleport an object's root body (env 0) in the simulator to the given world pose and velocity.

    Returns True when the pose was written. Every failure path logs a warning once (never silently disables the
    display); the first success per object logs one info line.
    """
    if environment is None:
        return False
    try:
        import torch
    except ImportError:
        _write_pose_warn(name, "torch is not importable")
        return False
    try:
        base = environment.unwrapped
        if not hasattr(base, "scene"):
            _write_pose_warn(name, "environment has no .scene")
            return False
        try:
            asset = base.scene[name]
        except (KeyError, IndexError, AttributeError) as exc:
            _write_pose_warn(name, f"no scene entry {name!r} ({type(exc).__name__}: {exc})")
            return False
        origin = getattr(base.scene.env_origins[0], "torch", base.scene.env_origins[0])
        pose = torch.tensor([*pos, *quat_xyzw], dtype=origin.dtype, device=origin.device).unsqueeze(0)
        pose[:, :3] += origin
        if hasattr(asset, "write_root_pose_to_sim_index"):
            asset.write_root_pose_to_sim_index(root_pose=pose)
        elif hasattr(asset, "write_root_pose_to_sim"):  # fallback for the legacy API name
            asset.write_root_pose_to_sim(root_pose=pose)
        else:
            _write_pose_warn(name, "asset has neither write_root_pose_to_sim_index nor write_root_pose_to_sim")
            return False
        v = torch.tensor([*lin_vel, *ang_vel], dtype=origin.dtype, device=origin.device).unsqueeze(0)
        if hasattr(asset, "write_root_velocity_to_sim_index"):
            asset.write_root_velocity_to_sim_index(root_velocity=v)
        elif hasattr(asset, "write_root_velocity_to_sim"):
            asset.write_root_velocity_to_sim(root_velocity=v)
    except Exception as exc:  # any simulator API mismatch: warn once, do not crash the episode
        _write_pose_warn(name, f"{type(exc).__name__}: {exc}")
        return False
    if name not in _WRITE_POSE_OK:
        _WRITE_POSE_OK.add(name)
        _LOG.info("write_pose(%s): first teleport written OK", name)
    return True


# -- gripper -------------------------------------------------------------------------------------------------------------

# Finger joint positions at "fully open", per joint. Both fingers open toward NEGATIVE positions: in the URDF
# joint7 has axis +y and joint8 axis -y, but joint8's origin is also turned 180 degrees, so both have limits
# [-0.04695, 0], and driving either one alone to FINGER_OPEN adds 47 mm to the pad gap (yam_arena's actions.py drives
# both to ``g * FINGER_OPEN``). The sign is kept per joint so a joint with the opposite convention is a one-line change.
_FINGER_OPEN_POS = {"joint7": FINGER_OPEN, "joint8": FINGER_OPEN}
_FRACTION_TOLERANCE = 1e-6  # joint positions arrive as float32: a finger exactly at the threshold must not flip on rounding
_LIMIT_TOLERANCE = 1e-4  # m


def _check_finger_limits(robot, names: list[str]) -> None:
    """Best effort: refuse to answer if the articulation's own limits do not contain the assumed open position."""
    try:
        limits = _to_numpy(robot.data.joint_pos_limits)[0]
        bounds = {joint: limits[names.index(joint)] for joint in FINGER_JOINTS}
    except (AttributeError, IndexError, TypeError, ValueError):
        return  # limits not readable here; the position read above is what matters
    for joint, (a, b) in bounds.items():
        if not (min(a, b) - _LIMIT_TOLERANCE <= _FINGER_OPEN_POS[joint] <= max(a, b) + _LIMIT_TOLERANCE):
            raise RuntimeError(
                f"gripper_open: {joint} has limits [{a:.5f}, {b:.5f}], which do not contain its assumed open "
                f"position {_FINGER_OPEN_POS[joint]:.5f}; the finger sign convention in parts.py is wrong for this asset"
            )


def _finger_openness(environment, arm: str) -> dict[str, float]:
    """Each finger joint's position as a fraction of the way to fully open (0 closed, 1 open)."""
    try:
        robot = environment.unwrapped.scene[f"{arm}_robot"]
        names = list(robot.joint_names)
        positions = _to_numpy(robot.data.joint_pos)[0]
    except (AttributeError, KeyError, IndexError, TypeError) as exc:
        raise RuntimeError(
            f"gripper_open: cannot read the {arm} arm's joint positions from env.unwrapped.scene['{arm}_robot'] ({exc!r})"
        ) from exc
    openness = {}
    for joint in FINGER_JOINTS:
        if joint not in _FINGER_OPEN_POS:
            raise RuntimeError(f"gripper_open: no open position is known for finger joint {joint!r}")
        if joint not in names:
            raise RuntimeError(f"gripper_open: the {arm} arm has no joint {joint!r} (joints: {names})")
        q = float(positions[names.index(joint)])
        if not math.isfinite(q):
            raise RuntimeError(f"gripper_open: {arm} {joint} position is {q}")
        openness[joint] = q / _FINGER_OPEN_POS[joint]
    _check_finger_limits(robot, names)
    return openness


def gripper_open(env, arm: str, threshold: float = 0.9) -> bool:
    """True when BOTH finger joints of ``arm`` ("left" or "right") are at least ``threshold`` of the way to fully open.

    Raises RuntimeError when the finger joints cannot be read; it never answers True on a guess.
    """
    if arm not in ("left", "right"):
        raise ValueError(f"arm must be 'left' or 'right', got {arm!r}")
    if not (math.isfinite(threshold) and 0.0 <= threshold <= 1.0):
        raise ValueError(f"threshold must be between 0 and 1, got {threshold!r}")
    if env is None:
        raise RuntimeError("gripper_open: there is no simulator env yet (env() is None until the first step or reset)")
    return all(frac >= threshold - _FRACTION_TOLERANCE for frac in _finger_openness(env, arm).values())


# -- callbacks -----------------------------------------------------------------------------------------------------------

_STEP_CALLBACKS: dict[tuple, object] = {}
_RESET_CALLBACKS: dict[tuple, object] = {}
_CURRENT = {"env": None}


def _callback_key(fn) -> tuple:
    code = getattr(fn, "__code__", None)
    where = (code.co_filename, code.co_firstlineno) if code is not None else (id(fn),)
    return (getattr(fn, "__module__", None), getattr(fn, "__qualname__", None), *where)


def on_step(fn):
    """Register ``fn(env)`` to run after every control step, before the task file is scored.

    A function with the same module, qualified name and source line as an earlier registration replaces it, so loading
    a task file twice in one process does not stack its callbacks.
    """
    _STEP_CALLBACKS[_callback_key(fn)] = fn
    return fn


def on_reset(fn):
    """Register ``fn(env)`` to run after every reset (once the arms and objects have settled). The observation that
    ``reset`` returns was taken before it ran."""
    _RESET_CALLBACKS[_callback_key(fn)] = fn
    return fn


def env():
    """The current Isaac Lab env (the gym env, so ``env().unwrapped`` is the Isaac Lab one), or None before the first
    step or reset."""
    return _CURRENT["env"]


# -- hooking yam_arena -----------------------------------------------------------------------------------------------

_HOOK_MARK = "_parts_hook"  # set on each wrapper we install, so hooking twice (or after a reload) is safe


def _marked(fn) -> bool:
    return bool(getattr(fn, _HOOK_MARK, False))


def _hook_scene(scene) -> None:
    """Make ``scene.task_object`` build Parts and ``scene.object_states`` report them."""
    for attr in ("task_object", "object_states"):
        if not callable(getattr(scene, attr, None)):
            raise RuntimeError(f"parts: {scene.__name__} has no {attr}(); this parts.py was written for yam_arena 0.3.1")

    if not _marked(scene.task_object):
        original_task_object = scene.task_object

        @functools.wraps(original_task_object)
        def task_object(spec):
            if isinstance(spec, Part):
                _PARTS[spec.name] = spec
                return _make_arena_object(spec)
            return original_task_object(spec)

        setattr(task_object, _HOOK_MARK, True)
        scene.task_object = task_object

    if not _marked(scene.object_states):
        original_object_states = scene.object_states

        @functools.wraps(original_object_states)
        def object_states(environment, names):
            names = list(names)
            ours = [n for n in names if n in _PARTS]
            plain = [n for n in names if n not in _PARTS]
            states = dict(original_object_states(environment, plain)) if plain else {}
            for n in ours:
                states[n] = _read_part_state(environment, _PARTS[n])
            return {n: states[n] for n in names}

        setattr(object_states, _HOOK_MARK, True)
        scene.object_states = object_states


def _hook_embodiment(module) -> None:
    """Run the step and reset callbacks around ``YamArenaEmbodiment._task_step`` and ``reset``."""
    cls = getattr(module, "YamArenaEmbodiment", None)
    if cls is None or not callable(getattr(cls, "_task_step", None)) or not callable(getattr(cls, "reset", None)):
        raise RuntimeError(
            f"parts: {module.__name__} has no YamArenaEmbodiment with _task_step() and reset(); "
            "this parts.py was written for yam_arena 0.3.1"
        )

    if not _marked(cls._task_step):
        original_task_step = cls._task_step

        @functools.wraps(original_task_step)
        def _task_step(self, *args, **kwargs):
            _CURRENT["env"] = self._env
            for fn in list(_STEP_CALLBACKS.values()):
                fn(self._env)
            return original_task_step(self, *args, **kwargs)

        setattr(_task_step, _HOOK_MARK, True)
        cls._task_step = _task_step

    if not _marked(cls.reset):
        original_reset = cls.reset

        @functools.wraps(original_reset)
        def reset(self, *args, **kwargs):
            observation = original_reset(self, *args, **kwargs)
            _CURRENT["env"] = self._env
            for fn in list(_RESET_CALLBACKS.values()):
                fn(self._env)
            return observation

        setattr(reset, _HOOK_MARK, True)
        cls.reset = reset


_HOOKS = {"yam_arena.scene": _hook_scene, "yam_arena.inspect_embodiment": _hook_embodiment}


class _HookAfterExec(importlib.abc.Loader):
    """Wraps a module's real loader: runs the module's code, then our hook on the finished module."""

    def __init__(self, inner, hook) -> None:
        self._inner, self._hook = inner, hook

    def create_module(self, spec):
        create = getattr(self._inner, "create_module", None)
        return create(spec) if create is not None else None

    def exec_module(self, module) -> None:
        self._inner.exec_module(module)
        self._hook(module)

    def __getattr__(self, name):  # get_source, get_code, is_package, ... stay the real loader's
        return getattr(self._inner, name)


class _HookFinder(importlib.abc.MetaPathFinder):
    """First on ``sys.meta_path``: for the hooked module names it asks the other finders as usual, then wraps the loader."""

    def __init__(self, hooks) -> None:
        self._hooks = dict(hooks)

    def find_spec(self, fullname, path=None, target=None):
        hook = self._hooks.get(fullname)
        if hook is None:
            return None
        for finder in list(sys.meta_path):
            if finder is self or not hasattr(finder, "find_spec"):
                continue
            spec = finder.find_spec(fullname, path, target)
            if spec is None:
                continue
            if spec.loader is None or not hasattr(spec.loader, "exec_module"):
                raise RuntimeError(f"parts: cannot hook {fullname}: its loader {spec.loader!r} has no exec_module()")
            spec.loader = _HookAfterExec(spec.loader, hook)
            return spec
        return None


def install() -> None:
    """Hook yam_arena: now for the modules already imported, and when the others are imported. Idempotent."""
    for name, hook in _HOOKS.items():
        module = sys.modules.get(name)
        if module is not None:
            hook(module)
    if not any(isinstance(f, _HookFinder) for f in sys.meta_path):
        sys.meta_path.insert(0, _HookFinder(_HOOKS))


install()
