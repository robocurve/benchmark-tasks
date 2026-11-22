"""bonds: rigid bodies made of several boxes, boxes on slides and hinges, and a one-way drive between two of them,
for yam_arena task files.

yam_arena's task files spawn plain primitives. This module adds these boxes and joints:

- :class:`Compound`: a ``Box`` with a mass whose spawned cuboid keeps its size (so the bounds and the centre the
  scorer reads are the cuboid's) but loses its collider. In its place the body gets the box colliders ``parts``
  (centre and size in the box's own frame), all shapes of the one rigid body, and its centre of mass is set to
  ``com``. A part ``(name, centre, size, (static, dynamic))`` takes that friction, combined with what it touches by its
  minimum, the others yam_arena's rubber-like 1.2 and 1.0 (averaged). A bar clamp's bar, fixed jaw and handle: its
  grip rubbery, its foot sliding on the bench.
- :class:`Ratchet`: a ``Box`` with a mass on a prismatic joint ("slide") along one of its own axes to the free object
  ``to``, from ``lower`` to ``upper`` metres from where it spawns, held at its stop by a stiff spring (``hold`` N/m, a
  position drive). :func:`set_upper` moves the stop at run time, the slide's upper limit and the spring's target
  together: a task that lowers it to wherever the box has been pushed makes a one-way ratchet, which the box can be
  pushed past (against ``hold`` times how far) but never pulled back over. ``collide=False`` lets the box and ``to``
  pass through each other (a jaw whose collar rides on the bar).
- :class:`Slide`: a :class:`Compound` on such a slide (a bar with its jaw).
- :class:`Hinge`: a ``Box`` with a mass on a revolute joint to the free object ``to``, about one of its own axes
  through ``pivot`` (a point in the box's frame), from ``lower`` to ``upper`` rad, with a torsion spring (``spring``
  N m/rad toward ``rest`` rad, ``damping`` N m s/rad). A clamp's trigger.
- :func:`one_way`: a joint between two of these bodies that only pushes: a point of ``body1`` (say a trigger's drive
  arm) cannot pass a stop on ``body0`` (say the bar) along one axis; everything else is free. :func:`set_low` moves
  the stop at run time: a task that moves it to wherever the point has gone back to makes a drive that grips on the
  push and slips on the return, as a clamp's drive plate does.

A :class:`Compound` with ``visible=True`` is drawn as its part boxes, each in its colour (``colors``), with optional
render-only boxes (``decor``: pads, a rubber grip), and its own cuboid is hidden.

Every reset puts each ratchet's upper limit back where it spawned. Importing this module hooks
``yam_arena.scene.task_object`` when that module is imported (after Isaac Sim starts), and
``yam_arena.inspect_embodiment.YamArenaEmbodiment`` so a task file can run code after every reset (:func:`on_reset`)
and before every scoring step (:func:`on_step`). Nothing in yam_arena is changed. ``success()`` and ``score()`` get
the usual ``ObjectState``. Pure Python at import time: everything that needs Isaac Sim is imported lazily.

Adapted from the bonds and envelope modules of the latch-and-lift-graphics-card, drill-battery-release and
board-into-antistatic-bag tasks.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

from yam_arena.task_file import Box

__all__ = ["Compound", "Hinge", "Ratchet", "Slide", "install", "low", "on_reset", "on_spawn", "on_step", "one_way",
           "set_low", "set_upper", "upper"]

GRIPPY = (1.2, 1.0)  # yam_arena's friction for task objects (constants.GRIPPY_FRICTION)


@dataclass(frozen=True)
class Compound(Box):
    """A free box that collides as the boxes ``parts``, each ``(name, centre, size)`` or ``(name, centre, size,
    friction)`` in the box's own frame (its centre, axes turned by ``yaw``), all inside it. ``com`` is the centre of
    mass in the same frame."""

    parts: tuple = ()
    com: tuple[float, float, float] = (0.0, 0.0, 0.0)
    visible: bool = False  # drawn as its parts (and decor), its own cuboid hidden
    colors: tuple = ()  # ((part name, sRGB), ...): a part's colour when visible (default: the box's colour)
    decor: tuple = ()  # ((name, centre, size, sRGB), ...): render-only boxes in the box's frame

    def _check(self) -> None:
        super()._check()
        if self.mass is None:
            raise ValueError(f"{self.name}: a compound must have a mass")
        for name, centre, size, rgb in self.decor:
            if not (len(centre) == 3 and len(size) == 3 and min(size) > 0 and all(0 <= c <= 1 for c in rgb)):
                raise ValueError(f"{self.name}.decor {name}: centre, size and sRGB colour")
        if not self.parts:
            raise ValueError(f"{self.name}: a compound needs at least one part")
        half = [s / 2 for s in self.size]
        for name, centre, size, *friction in self.parts:
            if friction and not (len(friction[0]) == 2 and min(friction[0]) >= 0):
                raise ValueError(f"{self.name}.{name}: friction is (static, dynamic), not negative")
            if not name.isidentifier():
                raise ValueError(f"{self.name}: part name {name!r} must be an identifier")
            for c, s, h in zip(centre, size, half):
                if not (s > 0 and abs(c) + s / 2 <= h + 1e-9):
                    raise ValueError(f"{self.name}.{name} must lie inside the box")
        if any(abs(c) > h for c, h in zip(self.com, half)):
            raise ValueError(f"{self.name}: the centre of mass must lie inside the box")


@dataclass(frozen=True)
class Ratchet(Box):
    """A free box on a prismatic joint along its own ``axis`` to the free object ``to``, from ``lower`` to ``upper``
    metres from where it spawns, held at its stop (where it spawns, until :func:`set_upper` moves it) by a spring of
    ``hold`` N/m with ``damping`` N s/m. With ``collide=False`` the box and ``to`` don't collide."""

    to: str = ""
    axis: str = "Z"
    lower: float = -0.05
    upper: float = 0.0
    hold: float = 1.0e4
    damping: float = 20.0
    collide: bool = False

    def _check(self) -> None:
        super()._check()
        if self.mass is None:
            raise ValueError(f"{self.name}: a ratchet box must have a mass")
        if self.axis not in ("X", "Y", "Z"):
            raise ValueError(f"{self.name}.axis must be X, Y or Z")
        if not (self.lower <= 0.0 <= self.upper and self.lower < self.upper):
            raise ValueError(f"{self.name}: lower <= 0 <= upper is where it spawns, and the range must not be empty")
        if not (self.to and self.to.isidentifier()):
            raise ValueError(f"{self.name}.to must name the free object it slides on")
        if self.hold <= 0 or self.damping < 0:
            raise ValueError(f"{self.name}: hold must be positive and damping not negative")


@dataclass(frozen=True)
class Slide(Compound, Ratchet):
    """A :class:`Compound` on a :class:`Ratchet`'s slide: a bar and its jaw, sliding through a clamp's handle."""


@dataclass(frozen=True)
class Hinge(Box):
    """A free box on a revolute joint about its own ``axis`` through ``pivot`` (in the box's frame, from its centre) to
    the free object ``to``, from ``lower`` to ``upper`` rad from where it spawns, pulled toward ``rest`` rad by a
    torsion spring of ``spring`` N m/rad with ``damping`` N m s/rad (``rest`` beyond ``lower`` gives the spring a
    preload at the stop). It does not collide with ``to``."""

    to: str = ""
    pivot: tuple[float, float, float] = (0.0, 0.0, 0.0)
    axis: str = "Z"
    lower: float = 0.0
    upper: float = 0.5
    spring: float = 0.0
    rest: float = 0.0
    damping: float = 0.0

    def _check(self) -> None:
        super()._check()
        if self.mass is None:
            raise ValueError(f"{self.name}: a hinged box must have a mass")
        if self.axis not in ("X", "Y", "Z"):
            raise ValueError(f"{self.name}.axis must be X, Y or Z")
        if not (self.lower <= 0.0 <= self.upper and self.lower < self.upper):
            raise ValueError(f"{self.name}: lower <= 0 <= upper is where it spawns, and the range must not be empty")
        if not (self.to and self.to.isidentifier()):
            raise ValueError(f"{self.name}.to must name the free object it turns on")
        if self.spring < 0 or self.damping < 0:
            raise ValueError(f"{self.name}: spring and damping must not be negative")
        if any(abs(p) > s / 2 + 1e-9 for p, s in zip(self.pivot, self.size)):
            raise ValueError(f"{self.name}: the pivot must lie inside the box")


@dataclass(frozen=True)
class OneWay:
    """A one-way drive: the point ``at`` (world, where both bodies spawn) of ``body1`` may not move along the world axis
    ``axis`` to below the stop on ``body0`` (``low`` m from where it spawns, 0 at first); it is free in every other
    direction and in turning. Made by :func:`one_way`."""

    name: str
    body0: str
    body1: str
    at: tuple[float, float, float]
    axis: str = "Y"


# -- frames ----------------------------------------------------------------------------------------------------------


def _rz(yaw: float, v):
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * v[0] - s * v[1], s * v[0] + c * v[1], v[2])


# -- spawning --------------------------------------------------------------------------------------------------------

_SPECS: dict[str, Compound | Ratchet] = {}
_PLACED: dict[str, tuple] = {}  # every task-file shape's (pos, yaw, free), to anchor a joint on it
_PRIMS: dict = {}  # each ratchet's spawned prim, to find its joint at run time
_UPPER: dict[str, float] = {}  # each ratchet's upper limit now
_SPAWN: dict = {}
_SPAWNED: list = []  # callbacks run on every task-file box as it spawns
_ONE_WAY: dict[str, OneWay] = {}
_JOINTS: dict = {}  # each one-way drive's joint prim, once made
_LOW: dict[str, float] = {}  # each one-way drive's stop now


def one_way(name: str, body0: str, body1: str, at, axis: str = "Y") -> OneWay:
    """Register a one-way drive (see :class:`OneWay`) between two task-file boxes. Made when the later one spawns."""
    if axis not in ("X", "Y", "Z"):
        raise ValueError(f"{name}.axis must be X, Y or Z")
    spec = OneWay(name, body0, body1, tuple(float(a) for a in at), axis)
    _ONE_WAY[name] = spec
    _LOW[name] = 0.0
    return spec


def _material(stage, path, friction, combine="average"):
    from pxr import PhysxSchema, UsdPhysics, UsdShade

    static, dynamic = friction
    material = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(static)
    api.CreateDynamicFrictionAttr(dynamic)
    api.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr(combine)
    return material


def _compound(prim, spec: Compound) -> None:
    """Replace the cuboid's collider by the part boxes, and set the centre of mass."""
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

    stage = prim.GetStage()
    if spec.visible:
        for p in Usd.PrimRange(prim):
            if p.IsA(UsdGeom.Gprim):
                UsdGeom.Imageable(p).MakeInvisible()
    for p in [p for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.CollisionAPI)]:
        for api in ("PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMeshCollisionAPI", "MaterialBindingAPI"):
            if api in p.GetAppliedSchemas():
                p.RemoveAppliedSchema(api)
    grippy = _material(stage, prim.GetPath().AppendChild("parts_material"), GRIPPY)
    UsdGeom.Xform.Define(stage, prim.GetPath().AppendChild("parts"))
    for name, centre, size, *friction in spec.parts:
        material = grippy
        if friction:
            material = _material(stage, prim.GetPath().AppendChild(f"{name}_material"), tuple(friction[0]), "min")
        cube = UsdGeom.Cube.Define(stage, prim.GetPath().AppendPath(f"parts/{name}"))
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        if spec.visible:
            _paint(stage, cube, dict(spec.colors).get(name, spec.color))
        else:
            UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        contact = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
        contact.CreateContactOffsetAttr(0.004)
        contact.CreateRestOffsetAttr(0.0)
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(
            material, UsdShade.Tokens.strongerThanDescendants, "physics")
    for name, centre, size, rgb in spec.decor:
        cube = UsdGeom.Cube.Define(stage, prim.GetPath().AppendPath(f"parts/{name}"))
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        _paint(stage, cube, rgb)
    UsdPhysics.MassAPI.Apply(prim).CreateCenterOfMassAttr(Gf.Vec3f(*spec.com))


def _paint(stage, gprim, rgb, rough: float = 0.5) -> None:
    """Bind a plain preview-surface material of colour ``rgb`` (sRGB) to ``gprim`` for rendering."""
    from pxr import Gf, Sdf, UsdShade

    path = gprim.GetPath().AppendChild("look")
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path.AppendChild("shader"))
    shader.CreateIdAttr("UsdPreviewSurface")
    linear = tuple(c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in rgb)
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(gprim.GetPrim()).Bind(material)


def _ratchet(prim, spec: Ratchet) -> None:
    """The prismatic joint from ``spec.to`` to the spawned box, anchored at the box's centre, with its limits and the
    spring that holds it at its stop. (A velocity drive with a force limit, as a friction, let the box creep down at
    3 mm/s under its own weight whatever its damping, so the stop is a stiff spring instead.)"""
    from pxr import Gf, Sdf, UsdPhysics

    if spec.to not in _PLACED or not _PLACED[spec.to][2]:
        raise KeyError(f"{spec.name}: {spec.to} must be a free task-file object listed before it")
    joint = UsdPhysics.PrismaticJoint.Define(prim.GetStage(), prim.GetPath().AppendChild("slide"))
    pos, yaw, _ = _PLACED[spec.to]
    joint.CreateBody0Rel().SetTargets([Sdf.Path(prim.GetPath().GetParentPath().AppendChild(spec.to))])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*_rz(-yaw, tuple(a - p for a, p in zip(spec.pos, pos)))))
    turn = (spec.yaw - yaw) / 2
    joint.CreateLocalRot0Attr(Gf.Quatf(math.cos(turn), 0.0, 0.0, math.sin(turn)))
    joint.CreateBody1Rel().SetTargets([prim.GetPath()])
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateAxisAttr(spec.axis)
    joint.CreateLowerLimitAttr(float(spec.lower))
    joint.CreateUpperLimitAttr(float(spec.upper))
    joint.CreateCollisionEnabledAttr(bool(spec.collide))
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(float(spec.hold))
    drive.CreateDampingAttr(float(spec.damping))
    drive.CreateTargetPositionAttr(float(spec.upper))
    _UPPER[spec.name] = float(spec.upper)


def _hinge(prim, spec: Hinge) -> None:
    """The revolute joint from ``spec.to`` to the spawned box about ``spec.axis`` through ``spec.pivot``, its limits
    and its torsion spring (USD's angular limits, targets and gains are per degree)."""
    from pxr import Gf, Sdf, UsdPhysics

    if spec.to not in _PLACED or not _PLACED[spec.to][2]:
        raise KeyError(f"{spec.name}: {spec.to} must be a free task-file object listed before it")
    joint = UsdPhysics.RevoluteJoint.Define(prim.GetStage(), prim.GetPath().AppendChild("hinge"))
    pos, yaw, _ = _PLACED[spec.to]
    pivot = tuple(p + q for p, q in zip(spec.pos, _rz(spec.yaw, spec.pivot)))
    joint.CreateBody0Rel().SetTargets([Sdf.Path(prim.GetPath().GetParentPath().AppendChild(spec.to))])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*_rz(-yaw, tuple(a - p for a, p in zip(pivot, pos)))))
    turn = (spec.yaw - yaw) / 2
    joint.CreateLocalRot0Attr(Gf.Quatf(math.cos(turn), 0.0, 0.0, math.sin(turn)))
    joint.CreateBody1Rel().SetTargets([prim.GetPath()])
    joint.CreateLocalPos1Attr(Gf.Vec3f(*spec.pivot))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateAxisAttr(spec.axis)
    joint.CreateLowerLimitAttr(math.degrees(spec.lower))
    joint.CreateUpperLimitAttr(math.degrees(spec.upper))
    joint.CreateCollisionEnabledAttr(False)
    if spec.spring > 0 or spec.damping > 0:
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(float(spec.spring) * math.pi / 180)
        drive.CreateDampingAttr(float(spec.damping) * math.pi / 180)
        drive.CreateTargetPositionAttr(math.degrees(spec.rest))


_AXES = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}
_TO_X = {"X": (1.0, 0.0, 0.0, 0.0), "Y": (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)),
         "Z": (math.sqrt(0.5), 0.0, -math.sqrt(0.5), 0.0)}  # w, x, y, z: turns the joint's x axis onto that axis


def _make_one_way(stage, spec: OneWay) -> None:
    """A D6 joint whose only limited motion is its frames' offset along ``spec.axis`` (from ``spec.at``, where both
    frames spawn): at least ``low`` (0 at first), at most 1 m. Turning and the other two directions are free."""
    from pxr import Gf, Sdf, UsdPhysics

    paths = {}
    for body in (spec.body0, spec.body1):
        if body not in _PRIMS_ANY:
            return
        paths[body] = _PRIMS_ANY[body].GetPath()
    if spec.name in _JOINTS:
        return
    joint = UsdPhysics.Joint.Define(stage, paths[spec.body0].AppendChild(spec.name))
    for i, body in enumerate((spec.body0, spec.body1)):
        pos, yaw, _ = _PLACED[body]
        local = _rz(-yaw, tuple(a - p for a, p in zip(spec.at, pos)))
        w, x, y, z = _TO_X[spec.axis]
        c, s_ = math.cos(-yaw / 2), math.sin(-yaw / 2)  # the world axis seen from the body
        q = (c * w - s_ * z, c * x - s_ * y, c * y + s_ * x, c * z + s_ * w)
        (joint.CreateBody0Rel if i == 0 else joint.CreateBody1Rel)().SetTargets([paths[body]])
        (joint.CreateLocalPos0Attr if i == 0 else joint.CreateLocalPos1Attr)(Gf.Vec3f(*local))
        (joint.CreateLocalRot0Attr if i == 0 else joint.CreateLocalRot1Attr)(Gf.Quatf(*q))
    joint.CreateCollisionEnabledAttr(False)
    limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "transX")
    limit.CreateLowAttr(float(_LOW[spec.name]))
    limit.CreateHighAttr(1.0)
    _JOINTS[spec.name] = joint.GetPrim()


def low(name: str) -> float:
    """The stop of the one-way drive ``name`` now (m from where it was made)."""
    return _LOW[name]


def set_low(name: str, value: float) -> None:
    """Move the stop of the one-way drive ``name`` to ``value`` (m). Call it from an :func:`on_step` callback."""
    from pxr import UsdPhysics

    if name not in _JOINTS:
        raise KeyError(f"{name} has not been made (both bodies must have spawned)")
    value = float(min(value, 0.9))
    UsdPhysics.LimitAPI(_JOINTS[name], "transX").GetLowAttr().Set(value)
    _LOW[name] = value


_PRIMS_ANY: dict = {}  # every spawned task-file box's prim


def _spawn_cuboid(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Isaac Lab's ``spawn_cuboid``, then the parts of a compound or the joint of a ratchet box. Installed in place of
    the original, which the spawner configurations name by module and attribute, so a replacement on one
    configuration would be lost."""
    prim = _SPAWN["cuboid"](prim_path, cfg, translation, orientation, **kwargs)
    name = prim_path.rstrip("/").rsplit("/", 1)[-1]
    spec = _SPECS.get(name)
    if isinstance(spec, Compound):
        _compound(prim, spec)
    if isinstance(spec, Ratchet):
        _ratchet(prim, spec)
        _PRIMS[spec.name] = prim
    if isinstance(spec, Hinge):
        _hinge(prim, spec)
    if name in _PLACED:
        _PRIMS_ANY[name] = prim
        for oneway in _ONE_WAY.values():
            if name in (oneway.body0, oneway.body1):
                _make_one_way(prim.GetStage(), oneway)
        for fn in _SPAWNED:
            fn(name, prim)
    return prim


def _patch_spawner() -> None:
    if "cuboid" in _SPAWN:
        return
    import isaaclab.sim.spawners as spawners
    import isaaclab.sim.spawners.shapes as shapes
    import isaaclab.sim.spawners.shapes.shapes as module

    _SPAWN["cuboid"] = module.spawn_cuboid
    for m in (module, shapes, spawners):
        m.spawn_cuboid = _spawn_cuboid


# -- the ratchet at run time -----------------------------------------------------------------------------------------


def upper(name: str) -> float:
    """The upper limit of the ratchet box ``name``'s slide now (m from where it spawned)."""
    return _UPPER[name]


def set_upper(name: str, value: float) -> None:
    """Move the stop of the ratchet box ``name`` to ``value`` (m from where it spawned): its slide's upper limit and its
    spring's target. Call it from an :func:`on_step` callback."""
    from pxr import UsdPhysics

    if name not in _PRIMS:
        raise KeyError(f"{name} has not spawned (the joint exists inside the simulation)")
    spec = _SPECS[name]
    value = float(min(max(value, spec.lower + 1e-4), spec.upper))
    joint = _PRIMS[name].GetChild("slide")
    UsdPhysics.PrismaticJoint(joint).GetUpperLimitAttr().Set(value)
    UsdPhysics.DriveAPI(joint, "linear").GetTargetPositionAttr().Set(value)
    _UPPER[name] = value


def _restore(env=None) -> None:
    for name in _PRIMS:
        if _UPPER[name] != _SPECS[name].upper:
            set_upper(name, _SPECS[name].upper)
    for name in _JOINTS:
        if _LOW[name] != 0.0:
            set_low(name, 0.0)


# -- hooking yam_arena -----------------------------------------------------------------------------------------------

_RESET: list = [_restore]
_STEP: list = []


def on_spawn(fn):
    """Register ``fn(name, prim)`` to run on every task-file box right after it spawns, before the simulation
    starts (to add to its prim, as ``looks.py`` does)."""
    _SPAWNED.append(fn)
    return fn


def on_reset(fn):
    """Register ``fn(env)`` to run after every reset of the embodiment, once the scene stands at its start."""
    _RESET.append(fn)
    return fn


def on_step(fn):
    """Register ``fn(env)`` to run after every control step, before the task file's hooks score it."""
    _STEP.append(fn)
    return fn


def _patch_scene(scene) -> None:
    if getattr(scene, "_bonds", False):
        return
    _patch_spawner()
    task_object = scene.task_object

    def task_object_hook(spec):
        if hasattr(spec, "pos"):
            _PLACED[spec.name] = (tuple(spec.pos), float(spec.yaw), spec.free)
        if isinstance(spec, (Compound, Ratchet, Hinge)):
            _SPECS[spec.name] = spec
        return task_object(spec)

    scene.task_object = task_object_hook
    scene._bonds = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_bonds", False):
        return
    base_reset, base_task_step = cls.reset, cls._task_step

    def reset(self, *args, **kwargs):
        out = base_reset(self, *args, **kwargs)
        for fn in _RESET:
            fn(self._env)
        return out

    def _task_step(self, *args, **kwargs):
        for fn in _STEP:
            fn(self._env)
        return base_task_step(self, *args, **kwargs)

    cls.reset, cls._task_step = reset, _task_step
    cls._bonds = True


class _Finder(importlib.abc.MetaPathFinder):
    """Patches yam_arena's Isaac-side modules as they are imported (after the task file is loaded)."""

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
        exec_module = loader.exec_module

        def exec_and_patch(module):
            exec_module(module)
            patch(module)

        loader.exec_module = exec_and_patch
        return spec


def install() -> None:
    """Hook yam_arena now if it is already imported, and on import otherwise. Idempotent."""
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
