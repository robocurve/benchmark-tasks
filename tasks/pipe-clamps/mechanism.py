# This file is adapted from robocurve/benchmark-tasks pull request #223 ("Open a panel held shut by a spring
# plunger", tasks/spring-plunger-panel/mechanism.py), which is published under the MIT License. Changes here: a joint
# may hang from another link (``Joint.parent``), so links form a chain; self-collision can be turned off
# (``Mechanism.self_collide``); the state reports each link's pose and velocity (``MechanismState.links``); a shape may
# be a convex hull of given points (``Shape.points``, kind "hull"); and a joint may have a dynamic friction
# (``Joint.dynamic_friction``); a revolute joint with infinite ``lower`` and ``upper`` turns freely (no limits authored).
# Changes for the tool-changer task: a loose rigid body of many shapes (``Body``, its mass spread by ``Shape.mass``,
# reported as a ``BodyState``), functions run after every control step before scoring (``on_step``), and a shape's
# size check that also refuses a hull with fewer than 4 points per ring.
# The original licence:
#
#   MIT License
#
#   Copyright (c) 2026 the authors of this task
#
#   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
#   documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
#   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
#   permit persons to whom the Software is furnished to do so, subject to the following conditions:
#
#   The above copyright notice and this permission notice shall be included in all copies or substantial portions of
#   the Software.
#
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
#   WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
#   OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
#   OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

"""A fixed mechanism with joints (a cabinet with a hinged panel and a spring plunger) for a yam_arena task file.

yam_arena's task files list single primitives (``Box``, ``Cylinder``, ``Sphere``) and library assets. This module adds
one more kind of object, :class:`Mechanism`: a fixed base and moving links, each built from boxes, cylinders and
capsules, joined by revolute or prismatic joints with springs, damping and friction. It becomes one fixed-base
Isaac Lab articulation. ``success()`` and ``score()`` receive a :class:`MechanismState` for it, with every joint's
position.

Importing this module hooks yam_arena without modifying it:

- ``yam_arena.scene.task_object`` builds a ``Mechanism`` as an articulation (other objects as before);
- ``yam_arena.scene.object_states`` reports a ``MechanismState`` for it;
- after ``YamArenaEmbodiment.reset`` returns, every mechanism's joints are put back to 0 at rest, its spring
  targets are set, the scene settles again as the reset does, and functions registered with :func:`on_reset` run;
- with the environment variable ``TOP_CAM_VIDEO=path.mp4`` set, the top camera of each episode is recorded there
  at its full render size (see :class:`_TopCamRecorder`).

The modules are patched when they are imported (or at once, if they already are). Nothing here imports Isaac Sim
at import time, so a task file that uses it still loads before the simulator starts.
"""

from __future__ import annotations

import atexit
import importlib.abc
import importlib.util
import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["Shape", "hull", "Link", "Joint", "Mechanism", "MechanismState", "Body", "BodyState", "body",
           "body_state", "on_reset", "on_step", "install", "top_cam_frame"]

Color = tuple[float, float, float]
AXES = ("X", "Y", "Z")


@dataclass(frozen=True)
class Shape:
    """One collider, which is also what is drawn. ``center`` is in the mechanism's frame (m).

    ``kind`` "box": ``size`` is its x, y, z edge lengths. "cylinder" and "capsule": ``size`` is (radius, length)
    along ``axis`` (a capsule's length is that of its straight part, without the two caps). "hull": the convex hull of
    ``points`` (mechanism frame, two rings of the same number of points, bottom then top, for the drawn faces);
    ``center`` and ``size`` are its bounding box's centre and edge lengths (:func:`hull`)."""

    kind: str
    center: tuple[float, float, float]
    size: tuple[float, ...]
    color: Color = (0.7, 0.7, 0.7)
    axis: str = "Z"
    material: str = "default"
    contact_offset: float = 0.002
    points: tuple = ()
    mass: float = 0.0  # kg; used only by a loose Body, whose mass is the sum of its shapes'

    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        c = np.asarray(self.center, float)
        if self.kind == "hull":
            pts = np.asarray(self.points, float)
            return pts.min(axis=0), pts.max(axis=0)
        if self.kind == "box":
            half = np.asarray(self.size, float) / 2
        else:
            radius, length = self.size
            half = np.full(3, radius)
            half[AXES.index(self.axis)] = length / 2 + (radius if self.kind == "capsule" else 0.0)
        return c - half, c + half


@dataclass(frozen=True)
class Link:
    """A rigid body: ``mass`` (kg) spread over its shapes."""

    name: str
    mass: float
    shapes: tuple[Shape, ...]


@dataclass(frozen=True)
class Joint:
    """A joint from the base to the link ``child``, through ``anchor`` (m, mechanism frame) along ``axis``.

    ``kind`` "revolute" (positions in rad) or "prismatic" (m); positive positions turn or slide the child about or
    along the axis by the right-hand rule. ``lower`` and ``upper`` are hard stops. A spring of ``stiffness``
    (N/m or N m/rad) pulls the joint toward ``target``, which may lie beyond a stop to preload it there. ``damping``
    is viscous (N s/m or N m s/rad); ``friction`` is the joint's friction as Isaac Lab's actuator takes it."""

    name: str
    kind: str
    child: str
    anchor: tuple[float, float, float]
    axis: str
    lower: float
    upper: float
    stiffness: float = 0.0
    damping: float = 0.0
    friction: float = 0.0
    target: float = 0.0
    parent: str = ""  # the link this joint hangs from; "" is the base
    dynamic_friction: float = 0.0  # the joint's friction while it moves (N or N m), as Isaac Lab's actuator takes it


@dataclass(frozen=True)
class Mechanism(Box):
    """A fixed mechanism, its frame at ``pos`` (world, m) turned by ``yaw``. It subclasses ``Box`` only so that
    yam_arena's task-file checks accept it: ``size`` is the bounding box of its shapes at rest, and it has no
    ``mass`` (yam_arena treats it as fixed and never jitters it).

    ``materials`` maps each shape's ``material`` name to its (static, dynamic) friction."""

    base: Link | None = None
    links: tuple[Link, ...] = ()
    joints: tuple[Joint, ...] = ()
    materials: dict = field(default_factory=dict, compare=False, hash=False)
    self_collide: bool = True

    def _check(self) -> None:
        super()._check()
        if self.base is None or self.mass is not None:
            raise ValueError(f"{self.name}: a Mechanism needs a base link and no mass (it is fixed)")
        names = [self.base.name] + [link.name for link in self.links]
        if len(set(names)) != len(names):
            raise ValueError(f"{self.name}: link names must be unique, got {names}")
        for link in (self.base, *self.links):
            if not link.shapes or not link.mass > 0:
                raise ValueError(f"{self.name}.{link.name}: a link needs shapes and a positive mass")
            for shape in link.shapes:
                if shape.kind not in ("box", "cylinder", "capsule", "hull") or shape.axis not in AXES:
                    raise ValueError(f"{self.name}.{link.name}: unknown shape {shape.kind!r} or axis {shape.axis!r}")
                if not all(v > 0 for v in shape.size):
                    raise ValueError(f"{self.name}.{link.name}: shape sizes must be positive, got {shape.size}")
                if (shape.kind == "hull") != bool(shape.points) or len(shape.points) % 2:
                    raise ValueError(f"{self.name}.{link.name}: a hull needs two rings of points, other shapes none")
                if shape.material not in self.materials:
                    raise ValueError(f"{self.name}.{link.name}: no friction given for material {shape.material!r}")
        for joint in self.joints:
            if joint.child not in names[1:] or joint.kind not in ("revolute", "prismatic") or joint.axis not in AXES:
                raise ValueError(f"{self.name}: bad joint {joint.name!r}")
            if joint.parent and (joint.parent not in names or joint.parent == joint.child):
                raise ValueError(f"{self.name}.{joint.name}: unknown parent {joint.parent!r}")
            if not joint.lower < joint.upper:
                raise ValueError(f"{self.name}.{joint.name}: lower must be below upper")
            if joint.kind == "prismatic" and not (math.isfinite(joint.lower) and math.isfinite(joint.upper)):
                raise ValueError(f"{self.name}.{joint.name}: a prismatic joint needs finite stops")
        if sorted(j.child for j in self.joints) != sorted(names[1:]):
            raise ValueError(f"{self.name}: every moving link needs exactly one joint")
        parent = {j.child: (j.parent or self.base.name) for j in self.joints}
        for name in names[1:]:  # every chain must end at the base: no loops
            seen = set()
            while name != self.base.name:
                if name in seen:
                    raise ValueError(f"{self.name}: the joints form a loop")
                seen.add(name)
                name = parent[name]

    def all_links(self) -> tuple[Link, ...]:
        return (self.base, *self.links)


def hull(points, color: Color = (0.7, 0.7, 0.7), material: str = "default", contact_offset: float = 0.002,
         mass: float = 0.0) -> Shape:
    """A convex-hull Shape of ``points``: two rings of n points each (bottom ring, then top ring, same winding)."""
    pts = np.asarray(points, float)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) < 8 or len(pts) % 2:
        raise ValueError(f"a hull needs two rings of at least 4 points, got shape {pts.shape}")
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    return Shape("hull", tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, material=material,
                 contact_offset=contact_offset, points=tuple(tuple(p) for p in pts.tolist()), mass=float(mass))


def _shape_inertia(shape: Shape) -> np.ndarray:
    """Diagonal inertia (kg m^2) of a shape about its own centre: a box, a solid cylinder or capsule along its axis
    (a capsule as a cylinder of its full length), or a hull as the box of its bounds [approximation]."""
    m = shape.mass
    if shape.kind in ("box", "hull"):
        a, b, c = shape.size
        return m / 12.0 * np.array([b * b + c * c, a * a + c * c, a * a + b * b])
    radius, length = shape.size
    if shape.kind == "capsule":
        length += 2 * radius
    axial = m * radius * radius / 2.0
    across = m * (3 * radius * radius + length * length) / 12.0
    out = np.full(3, across)
    out[AXES.index(shape.axis)] = axial
    return out


@dataclass(frozen=True)
class Body(Box):
    """A loose rigid body of many shapes (one PhysX rigid body), its frame at ``pos`` (world, m) turned by ``yaw``.
    It subclasses ``Box`` so yam_arena's task-file checks accept it: ``size`` is the bounding box of its shapes and
    ``mass`` (the sum of the shapes' ``mass``) makes it free. ``materials`` maps material names to (static, dynamic)
    friction. Shape centres and points are in the body frame."""

    shapes: tuple[Shape, ...] = ()
    materials: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        super()._check()
        if not self.shapes or self.mass is None or not self.mass > 0:
            raise ValueError(f"{self.name}: a Body needs shapes and a positive mass")
        for shape in self.shapes:
            if shape.kind not in ("box", "cylinder", "capsule", "hull") or shape.axis not in AXES:
                raise ValueError(f"{self.name}: unknown shape {shape.kind!r} or axis {shape.axis!r}")
            if not all(v > 0 for v in shape.size) or not shape.mass > 0:
                raise ValueError(f"{self.name}: shape sizes and masses must be positive, got {shape.size}, {shape.mass}")
            if shape.material not in self.materials:
                raise ValueError(f"{self.name}: no friction given for material {shape.material!r}")
        if abs(sum(s.mass for s in self.shapes) - self.mass) > 1e-9:
            raise ValueError(f"{self.name}: mass must be the sum of the shapes' masses")

    def inertia_tensor(self) -> tuple[np.ndarray, np.ndarray]:
        """(centre of mass, 3x3 inertia about it) in the body frame: each shape's own inertia (a hull as the box of its
        bounds [approximation]) moved to the centre of mass by the parallel-axis rule, products included."""
        masses = np.array([s.mass for s in self.shapes])
        centres = np.array([s.center for s in self.shapes], float)
        com = (masses[:, None] * centres).sum(axis=0) / masses.sum()
        inertia = np.zeros((3, 3))
        for s, c in zip(self.shapes, centres, strict=True):
            d = c - com
            inertia += np.diag(_shape_inertia(s)) + s.mass * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        return com, inertia

    def mass_properties(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(centre of mass, principal moments, principal axes as a w, x, y, z quaternion) in the body frame, as USD's
        MassAPI takes them."""
        com, inertia = self.inertia_tensor()
        moments, axes = np.linalg.eigh(inertia)
        if np.linalg.det(axes) < 0:
            axes[:, 2] *= -1
        return com, moments, _quat_wxyz(axes)

    def corners(self) -> np.ndarray:
        """The 8 corners of every shape's bounding box, body frame (n, 3)."""
        out = []
        for s in self.shapes:
            lo, hi = s.bounds()
            out += [[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
        return np.array(out)


def body(name: str, pos, shapes, materials: dict, yaw: float = 0.0, color: Color = (0.7, 0.7, 0.7)) -> Body:
    """Build a Body, its ``size`` and ``mass`` filled in from its shapes."""
    lows, highs = zip(*(s.bounds() for s in shapes))
    size = tuple((np.max(highs, axis=0) - np.min(lows, axis=0)).tolist())
    return Body(name=name, pos=tuple(pos), size=size, yaw=yaw, color=color, mass=float(sum(s.mass for s in shapes)),
                shapes=tuple(shapes), materials=dict(materials))


@dataclass(frozen=True)
class BodyState(ObjectState):
    """A Body's state: ``pos`` and ``quat_xyzw`` are its frame's pose (not its bounding box's centre), the velocities
    are that frame origin's, and ``low``/``high`` bound its shapes' boxes turned with it."""

    def to_world(self, p) -> np.ndarray:
        """A body-frame point in the world."""
        return _rot_xyzw(self.quat_xyzw) @ np.asarray(p, float) + np.asarray(self.pos, float)

    def to_body(self, p) -> np.ndarray:
        """A world point in the body frame."""
        return _rot_xyzw(self.quat_xyzw).T @ (np.asarray(p, float) - np.asarray(self.pos, float))


def _quat_wxyz(r) -> np.ndarray:
    """A rotation matrix as a unit quaternion (w, x, y, z)."""
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def _rot_xyzw(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mechanism(name: str, pos, base: Link, links, joints, materials: dict, yaw: float = 0.0,
              color: Color = (0.7, 0.7, 0.7), self_collide: bool = True) -> Mechanism:
    """Build a Mechanism, its ``size`` filled in from its shapes' extent at rest."""
    lows, highs = zip(*(s.bounds() for link in (base, *links) for s in link.shapes))
    size = tuple((np.max(highs, axis=0) - np.min(lows, axis=0)).tolist())
    return Mechanism(name=name, pos=tuple(pos), size=size, yaw=yaw, color=color, base=base, links=tuple(links),
                     joints=tuple(joints), materials=dict(materials), self_collide=self_collide)


@dataclass(frozen=True)
class MechanismState(ObjectState):
    """The base's pose (``pos`` is its frame's origin), zero velocities (it is fixed), the world-aligned bounds of
    its shapes at rest, each joint's position (rad or m, 0 at rest), and each moving link's state in ``links``: an
    ``ObjectState`` whose ``pos`` and ``quat_xyzw`` are the link frame's pose (the mechanism's frame at rest, carried by
    the link) and whose bounds are that point."""

    joints: dict = field(default_factory=dict)
    links: dict = field(default_factory=dict)


# -- spawning --------------------------------------------------------------------------------------------------------

_SPECS: dict[str, Mechanism] = {}


def _authoring(stage, prim_path: str, materials: dict):
    """Physics materials and colours under ``prim_path``/Looks, and a function ``author_shape(path, shape)`` that
    authors one collider (also what is drawn) with its material and colour."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    physics: dict[str, str] = {}
    for name, (static, dynamic) in materials.items():
        path = f"{prim_path}/Looks/physics_{name}"
        material = sim_utils.RigidBodyMaterialCfg(static_friction=static, dynamic_friction=dynamic)
        material.func(path, material)
        physics[name] = path
    looks: dict[Color, str] = {}

    def look(color: Color) -> str:
        if color not in looks:
            path = f"{prim_path}/Looks/color_{len(looks)}"
            surface = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
            surface.func(path, surface)
            looks[color] = path
        return looks[color]

    def collider(prim, shape: Shape) -> None:
        UsdPhysics.CollisionAPI.Apply(prim)
        physx = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        physx.CreateContactOffsetAttr(shape.contact_offset)
        physx.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), physics[shape.material], stage=stage)
        bind_visual_material(str(prim.GetPath()), look(shape.color), stage=stage)

    def author_shape(path: str, shape: Shape) -> None:
        if shape.kind == "hull":
            n = len(shape.points) // 2
            geom = UsdGeom.Mesh.Define(stage, path)
            faces = [list(range(n - 1, -1, -1)), list(range(n, 2 * n))]
            faces += [[i, (i + 1) % n, n + (i + 1) % n, n + i] for i in range(n)]
            geom.CreatePointsAttr([Gf.Vec3f(*p) for p in shape.points])
            geom.CreateFaceVertexCountsAttr([len(f) for f in faces])
            geom.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
            geom.CreateSubdivisionSchemeAttr("none")
            UsdPhysics.MeshCollisionAPI.Apply(geom.GetPrim()).CreateApproximationAttr("convexHull")
            collider(geom.GetPrim(), shape)
            return
        if shape.kind == "box":
            geom = UsdGeom.Cube.Define(stage, path)
            geom.CreateSizeAttr(1.0)
            geom.AddTranslateOp().Set(Gf.Vec3d(*shape.center))
            geom.AddScaleOp().Set(Gf.Vec3f(*shape.size))
        else:
            geom = (UsdGeom.Cylinder if shape.kind == "cylinder" else UsdGeom.Capsule).Define(stage, path)
            geom.CreateAxisAttr(shape.axis)
            geom.CreateRadiusAttr(shape.size[0])
            geom.CreateHeightAttr(shape.size[1])
            geom.AddTranslateOp().Set(Gf.Vec3d(*shape.center))
        collider(geom.GetPrim(), shape)

    return author_shape


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the mechanism under ``prim_path``: an Xform (the articulation root) holding one rigid body per link,
    a fixed joint from the world to the base, and the joints. Bodies and shapes are in the mechanism's frame."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

    spec: Mechanism = _SPECS[cfg.mechanism_name]
    stage = get_current_stage()
    double = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation if translation is not None else (0, 0, 0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))

    author_shape = _authoring(stage, prim_path, spec.materials)

    bodies = {}
    for link in spec.all_links():
        path = f"{prim_path}/{link.name}"
        body = UsdGeom.Xform.Define(stage, path)
        body.AddTranslateOp(precision=double).Set(Gf.Vec3d(0.0, 0.0, 0.0))
        body.AddOrientOp(precision=double).Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
        prim = body.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(link.mass)
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateMaxDepenetrationVelocityAttr(1.0)
        for i, shape in enumerate(link.shapes):
            author_shape(f"{path}/{shape.kind}_{i}", shape)
        bodies[link.name] = Sdf.Path(path)

    # The base is fixed to the world where it stands. With the articulation root on the parent Xform (not on the
    # base body), PhysX makes this a fixed-base articulation rather than a body held by a joint.
    base_prim = stage.GetPrimAtPath(bodies[spec.base.name])
    to_world = UsdGeom.Xformable(base_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    fixed = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/{spec.base.name}/fixed_to_world")
    fixed.CreateBody1Rel().SetTargets([bodies[spec.base.name]])
    fixed.CreateLocalPos0Attr(Gf.Vec3f(to_world.ExtractTranslation()))
    fixed.CreateLocalRot0Attr(Gf.Quatf(to_world.ExtractRotationQuat()))
    fixed.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    fixed.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    for joint in spec.joints:
        path = f"{prim_path}/{joint.child}/{joint.name}"
        revolute = joint.kind == "revolute"
        usd_joint = (UsdPhysics.RevoluteJoint if revolute else UsdPhysics.PrismaticJoint).Define(stage, path)
        usd_joint.CreateAxisAttr(joint.axis)
        usd_joint.CreateBody0Rel().SetTargets([bodies[joint.parent or spec.base.name]])
        usd_joint.CreateBody1Rel().SetTargets([bodies[joint.child]])
        # Both bodies' frames are the mechanism's frame at rest, so the anchor is the same point in each.
        usd_joint.CreateLocalPos0Attr(Gf.Vec3f(*joint.anchor))
        usd_joint.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        usd_joint.CreateLocalPos1Attr(Gf.Vec3f(*joint.anchor))
        usd_joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        unit = math.degrees(1.0) if revolute else 1.0  # USD gives revolute limits and targets in degrees
        # WHY: a roller turns many times over a stroke; a revolute joint with infinite limits gets none authored, and
        # USD Physics reads missing limits as free (-inf, inf)
        if math.isfinite(joint.lower):
            usd_joint.CreateLowerLimitAttr(joint.lower * unit)
        if math.isfinite(joint.upper):
            usd_joint.CreateUpperLimitAttr(joint.upper * unit)
        drive = UsdPhysics.DriveAPI.Apply(usd_joint.GetPrim(), "angular" if revolute else "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(joint.stiffness / unit)
        drive.CreateDampingAttr(joint.damping / unit)
        drive.CreateTargetPositionAttr(joint.target * unit)

    # The moving links collide with one another (the plunger's pin with the panel's strike) but not with the base,
    # whose parts they pass through or rest against by design (the pin runs inside its guide housing).
    UsdPhysics.FilteredPairsAPI.Apply(base_prim).CreateFilteredPairsRel().SetTargets(
        [bodies[link.name] for link in spec.links])
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    articulation = PhysxSchema.PhysxArticulationAPI.Apply(root.GetPrim())
    articulation.CreateEnabledSelfCollisionsAttr(bool(spec.self_collide))
    articulation.CreateSolverPositionIterationCountAttr(32)
    articulation.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


def _arena_object(spec: Mechanism):
    """The mechanism as an Isaac Lab-Arena object: an articulation whose joints are implicit actuators carrying
    each joint's spring, damping and friction."""
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    _SPECS[spec.name] = spec

    @configclass
    class MechanismSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn)
        mechanism_name: str = ""

    class MechanismObject(Object):
        def _generate_articulation_cfg(self):
            actuators = {
                joint.name: ImplicitActuatorCfg(joint_names_expr=[joint.name], stiffness=joint.stiffness,
                                                damping=joint.damping, friction=joint.friction,
                                                dynamic_friction=joint.dynamic_friction)
                for joint in spec.joints
            }
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    return MechanismObject(
        name=spec.name,
        object_type=ObjectType.ARTICULATION,
        spawner_cfg=MechanismSpawnerCfg(mechanism_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos),
                          rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))),
    )


_BODIES: dict[str, Body] = {}


def _spawn_body(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author a loose Body under ``prim_path``: an Xform (posed by the spawner) holding one rigid body ``body`` whose
    colliders are the Body's shapes, with the Body's mass, centre of mass and diagonal inertia."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    spec: Body = _BODIES[cfg.body_name]
    stage = get_current_stage()
    double = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation if translation is not None else (0, 0, 0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))
    author_shape = _authoring(stage, prim_path, spec.materials)
    path = f"{prim_path}/body"
    body_xf = UsdGeom.Xform.Define(stage, path)
    body_xf.AddTranslateOp(precision=double).Set(Gf.Vec3d(0.0, 0.0, 0.0))
    body_xf.AddOrientOp(precision=double).Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
    prim = body_xf.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    com, moments, axes = spec.mass_properties()
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr(float(spec.mass))
    mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*(float(v) for v in com)))
    mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*(float(v) for v in moments)))
    mass_api.CreatePrincipalAxesAttr(Gf.Quatf(*(float(v) for v in axes)))
    physx = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    physx.CreateMaxDepenetrationVelocityAttr(1.0)
    physx.CreateSolverPositionIterationCountAttr(32)
    physx.CreateSolverVelocityIterationCountAttr(4)
    for i, shape in enumerate(spec.shapes):
        author_shape(f"{path}/{shape.kind}_{i}", shape)
    return root.GetPrim()


def _body_object(spec: Body):
    """The Body as an Isaac Lab-Arena rigid object."""
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    _BODIES[spec.name] = spec

    @configclass
    class BodySpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_body)
        body_name: str = ""

    return Object(
        name=spec.name,
        object_type=ObjectType.RIGID,
        spawner_cfg=BodySpawnerCfg(body_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos),
                          rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))),
    )


# -- state, reset ----------------------------------------------------------------------------------------------------


def _torch(x):
    return getattr(x, "torch", x)  # Isaac Lab 3.0 wraps some buffers


def _numpy(x) -> np.ndarray:
    return _torch(x).detach().cpu().numpy().astype(np.float64)


def _joint_ids(asset, spec: Mechanism) -> list[int]:
    names = list(asset.joint_names)
    return [names.index(joint.name) for joint in spec.joints]


def _state(environment, spec: Mechanism) -> MechanismState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _numpy(base.scene.env_origins[0])
    pose = _numpy(asset.data.root_link_pose_w)[0]
    q = _numpy(asset.data.joint_pos)[0]
    joints = {joint.name: float(q[i]) for joint, i in zip(spec.joints, _joint_ids(asset, spec), strict=True)}
    poses = _numpy(asset.data.body_link_pose_w)[0]
    vels = _numpy(asset.data.body_link_vel_w)[0]
    body_names = list(asset.body_names)
    links = {}
    for link in spec.links:
        i = body_names.index(link.name)
        p = poses[i, :3] - origin
        links[link.name] = ObjectState(pos=p, quat_xyzw=poses[i, 3:7], lin_vel=vels[i, :3], ang_vel=vels[i, 3:6],
                                       low=p.copy(), high=p.copy())
    half = np.asarray(spec.size) / 2
    lows, highs = zip(*(s.bounds() for link in spec.all_links() for s in link.shapes))
    centre = (np.min(lows, axis=0) + np.max(highs, axis=0)) / 2
    c, s = math.cos(spec.yaw), math.sin(spec.yaw)
    middle = pose[:3] - origin + np.array([c * centre[0] - s * centre[1], s * centre[0] + c * centre[1], centre[2]])
    reach = np.array([abs(c) * half[0] + abs(s) * half[1], abs(s) * half[0] + abs(c) * half[1], half[2]])
    return MechanismState(pos=pose[:3] - origin, quat_xyzw=pose[3:7], lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                          low=middle - reach, high=middle + reach, joints=joints, links=links)


def _body_state(environment, spec: Body) -> BodyState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _numpy(base.scene.env_origins[0])
    pose = _numpy(asset.data.root_link_pose_w)[0]
    data = asset.data
    vel = _numpy(data.root_link_vel_w)[0] if hasattr(data, "root_link_vel_w") else _numpy(data.root_vel_w)[0]
    return body_state(spec, pose[:3] - origin, pose[3:7], vel[:3], vel[3:6])


def body_state(spec: Body, pos, quat_xyzw, lin_vel=(0.0, 0.0, 0.0), ang_vel=(0.0, 0.0, 0.0)) -> BodyState:
    """A Body's state at a frame pose (world, m): its bounds are its shapes' box corners turned with it. Also used
    by the offline tests to build states."""
    pos = np.asarray(pos, float)
    pts = spec.corners() @ _rot_xyzw(quat_xyzw).T + pos
    return BodyState(pos=pos, quat_xyzw=np.asarray(quat_xyzw, float), lin_vel=np.asarray(lin_vel, float),
                     ang_vel=np.asarray(ang_vel, float), low=pts.min(axis=0), high=pts.max(axis=0))


def _rest(environment, spec: Mechanism) -> None:
    """Every joint back to 0 and still, and each spring's target set (a target beyond a stop preloads it)."""
    import torch

    base = environment.unwrapped
    asset = base.scene[spec.name]
    ids = _joint_ids(asset, spec)
    device = _torch(asset.data.joint_pos).device
    zeros = torch.zeros((1, len(ids)), dtype=torch.float32, device=device)
    targets = torch.tensor([[joint.target for joint in spec.joints]], dtype=torch.float32, device=device)
    if hasattr(asset, "write_joint_state_to_sim_index"):  # Isaac Lab 3.0: keyword-only writers
        asset.write_joint_state_to_sim_index(position=zeros, velocity=zeros, joint_ids=ids)
        asset.set_joint_position_target_index(target=targets, joint_ids=ids)
        asset.set_joint_velocity_target_index(target=zeros, joint_ids=ids)
    else:  # Isaac Lab 2.x
        asset.write_joint_state_to_sim(zeros, zeros, joint_ids=ids)
        asset.set_joint_position_target(targets, joint_ids=ids)
        asset.set_joint_velocity_target(zeros, joint_ids=ids)
    asset.write_data_to_sim()


_RESET: list = []


def on_reset(fn):
    """Register ``fn()`` to run at the end of every reset, once the mechanisms are back at rest."""
    _RESET.append(fn)
    return fn


_STEP: list = []


def on_step(fn):
    """Register ``fn(env)`` (the Isaac Lab environment) to run after every control step of a task file, before
    yam_arena reads the object states and calls success() and score() for that step."""
    _STEP.append(fn)
    return fn


# -- hooking yam_arena -----------------------------------------------------------------------------------------------


def _patch_scene(scene) -> None:
    if getattr(scene, "_mechanism_patched", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def patched_task_object(spec):
        if isinstance(spec, Mechanism):
            return _arena_object(spec)
        if isinstance(spec, Body):
            return _body_object(spec)
        return task_object(spec)

    def patched_object_states(environment, names):
        others = [n for n in names if n not in _SPECS and n not in _BODIES]
        out = object_states(environment, others) if others else {}
        out.update({n: _state(environment, _SPECS[n]) for n in names if n in _SPECS})
        out.update({n: _body_state(environment, _BODIES[n]) for n in names if n in _BODIES})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states = patched_task_object, patched_object_states
    scene._mechanism_patched = True


TOP_CAM_VIDEO = "TOP_CAM_VIDEO"  # environment variable: record the top camera to this MP4


def top_cam_frame(embodiment) -> np.ndarray:
    """The station top camera's latest render at its full size (640 x 360), before yam_arena resizes it for the
    policy's observation."""
    out = embodiment._base.scene["top_cam"].data.output["rgb"]
    out = out.torch if hasattr(out, "torch") else out
    return np.ascontiguousarray(out[0, ..., :3].cpu().numpy(), dtype=np.uint8)


class _TopCamRecorder:
    """With ``TOP_CAM_VIDEO=path.mp4`` set, each episode's top camera is written to that H.264 MP4 at the control rate:
    the frame at the reset, then one per control step, at the camera's full render size whatever the policy's
    ``cam_width`` / ``cam_height``. The first frame is also saved beside it as ``path.png`` (the setup screenshot).
    The variable is read at each reset, so one process can record several episodes to different files."""

    def __init__(self) -> None:
        self.writer, self._exit_hook = None, False

    def start(self, embodiment) -> None:
        self.close()
        path = os.environ.get(TOP_CAM_VIDEO)
        if not path:
            return
        import imageio.v2 as imageio
        import imageio_ffmpeg

        path = os.path.abspath(os.path.expanduser(path))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        frame = top_cam_frame(embodiment)
        imageio.imwrite(os.path.splitext(path)[0] + ".png", frame)
        h, w = frame.shape[:2]
        hz = round(1.0 / embodiment._base.step_dt, 6)
        self.writer = imageio_ffmpeg.write_frames(
            path, (w, h), pix_fmt_in="rgb24", pix_fmt_out="yuv420p", fps=hz, quality=None, codec="libx264",
            macro_block_size=8, output_params=["-crf", "20", "-movflags", "+faststart"])
        self.writer.send(None)
        self.writer.send(frame)
        if not self._exit_hook:  # registered after yam_arena's (at the simulator's start), so it runs before Kit closes
            atexit.register(self.close)
            self._exit_hook = True

    def add(self, embodiment) -> None:
        if self.writer is not None:
            self.writer.send(top_cam_frame(embodiment))

    def close(self) -> None:
        if self.writer is not None:
            writer, self.writer = self.writer, None
            writer.close()


_RECORDER = _TopCamRecorder()


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_mechanism_patched", False):
        return
    reset, step, close, task_step = cls.reset, cls.step, cls.close, cls._task_step

    def patched_reset(self, scene, *, seed=None):
        observation = reset(self, scene, seed=seed)
        specs = [o for o in (self.task.objects if self.task is not None else ()) if isinstance(o, Mechanism)]
        if specs:
            for spec in specs:
                _rest(self._env, _SPECS.get(spec.name, spec))
            base = self._base
            for _ in range(max(1, round(module.SETTLE_S / base.step_dt))):  # settle again, as the reset does
                self._env.step(self._ready)
            for _ in range(module.RESET_RENDERS):
                base.sim.render()
            self._t = 0.0
            for fn in _RESET:
                fn()
            observation = self._observation()
        _RECORDER.start(self)
        return observation

    def patched_step(self, action):
        result = step(self, action)
        _RECORDER.add(self)
        return result

    def patched_close(self):
        _RECORDER.close()
        return close(self)

    def patched_task_step(self, *args, **kwargs):
        # WHY here: _task_step runs after the physics step and before object_states/success/score read that step
        # (inspect_embodiment.py YamArenaEmbodiment.step), so what fn(env) records belongs to the step being scored
        for fn in _STEP:
            fn(self._env)
        return task_step(self, *args, **kwargs)

    cls.reset, cls.step, cls.close, cls._task_step = patched_reset, patched_step, patched_close, patched_task_step
    cls._mechanism_patched = True


_PATCHES = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}


class _PatchOnImport(importlib.abc.MetaPathFinder):
    """Finds the modules in ``_PATCHES`` with the other finders, and patches each right after it executes."""

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
        if spec.loader is None:
            return None
        execute, patch = spec.loader.exec_module, _PATCHES[fullname]

        def exec_module(module):
            execute(module)
            patch(module)

        spec.loader.exec_module = exec_module
        return spec


def install() -> None:
    """Patch the yam_arena modules already imported, and the others when they are. Safe to call again."""
    for name, patch in _PATCHES.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _PatchOnImport) for f in sys.meta_path):
        sys.meta_path.insert(0, _PatchOnImport())


install()
