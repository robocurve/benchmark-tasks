"""bonds: task-file boxes on a PhysX hinge, with the hinge's drive steered at run time, and hooks into yam_arena.

yam_arena's task files spawn plain primitives. This module adds :class:`Hinged`: a ``Box`` with a mass on a revolute
joint ("hinge") about one of its own axes, through the point ``pivot`` (task-file frame), free to turn from ``lower``
to ``upper`` degrees from where it spawns. An angular spring drive (``stiffness`` N m per degree, ``damping`` N m s
per degree) pulls it toward a target angle that the task can move at run time with :func:`set_target`: a lever that
snaps open or shut (the task flips the target once the lever passes its middle), a door closer, a toggle. The other
side of the joint is the free object ``to``, or the bench.

Importing this module hooks ``yam_arena.scene.task_object`` when that module is imported (after Isaac Sim starts),
and ``yam_arena.inspect_embodiment.YamArenaEmbodiment`` so a task file can run code when a box spawns
(:func:`on_spawn`), after every reset (:func:`on_reset`) and before every scoring step (:func:`on_step`). Every reset
puts each drive target back where it spawned. Nothing in yam_arena is changed. ``success()`` and ``score()`` get the
usual ``ObjectState``. Pure Python at import time: everything that needs Isaac Sim is imported lazily.

Adapted from the bonds module of the latch-and-lift-graphics-card and drill-battery-release tasks (the joint frames
for turned boxes and the yam_arena hooks are theirs), with the revolute joint and its steered drive added.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

from yam_arena.task_file import Box

__all__ = ["Hinged", "install", "on_reset", "on_spawn", "on_step", "set_target", "target"]


@dataclass(frozen=True)
class Hinged(Box):
    """A free box on a revolute joint about its own ``axis`` ("X", "Y" or "Z", the world's at ``yaw`` 0) through the
    task-file point ``pivot``, from ``lower`` to ``upper`` degrees from where it spawns (positive turns by the right
    hand about the axis), pulled toward ``start_target`` degrees by a spring of ``stiffness`` N m per degree with
    ``damping`` N m s per degree. The joint's other side is the free object ``to``, or the bench."""

    to: str = ""
    axis: str = "Y"
    pivot: tuple[float, float, float] = (0.0, 0.0, 0.0)
    lower: float = 0.0
    upper: float = 90.0
    stiffness: float = 0.0
    damping: float = 0.0
    start_target: float = 0.0

    def _check(self) -> None:
        super()._check()
        if self.mass is None:
            raise ValueError(f"{self.name}: a hinged box must have a mass")
        if self.axis not in ("X", "Y", "Z"):
            raise ValueError(f"{self.name}.axis must be X, Y or Z")
        if not (self.lower <= 0.0 <= self.upper and self.lower < self.upper):
            raise ValueError(f"{self.name}: lower <= 0 <= upper is where it spawns, and the range must not be empty")
        if self.stiffness < 0 or self.damping < 0:
            raise ValueError(f"{self.name}: stiffness and damping must not be negative")
        if len(self.pivot) != 3:
            raise ValueError(f"{self.name}.pivot must be a point")
        if self.to and not self.to.isidentifier():
            raise ValueError(f"{self.name}.to must name an object")


# -- quaternions (x, y, z, w) ----------------------------------------------------------------------------------------


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def _qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


def _qrot(q, v):
    x, y, z, _ = _qmul(_qmul(q, (*v, 0.0)), _qconj(q))
    return (x, y, z)


def _quat_of(spec):
    q = getattr(spec, "quat_xyzw", None)
    if q is not None:
        return tuple(float(v) for v in q)
    return (0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))


# -- spawning --------------------------------------------------------------------------------------------------------

_SPECS: dict[str, Hinged] = {}
_PLACED: dict[str, tuple] = {}  # every task-file object's (pos, quat_xyzw, free), to anchor a joint on it
_PRIMS: dict = {}  # each hinged box's spawned prim, to find its joint at run time
_TARGETS: dict[str, float] = {}  # each hinge's drive target now (degrees), since the last reset
_SPAWN: dict = {}
_SPAWNED: list = []  # callbacks run on every task-file box as it spawns
DRIVE = "drive:angular:physics:targetPosition"


def _anchor(joint, prim, spec, anchor) -> None:
    """Author the joint's two frames: body 1 is the spawned box, body 0 the object ``spec.to`` or the world.
    ``anchor`` is the joint's position in the task file's frame. The joint's axes are the box's own."""
    from pxr import Gf, Sdf, Usd, UsdGeom

    world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
    origin = world - Gf.Vec3d(*spec.pos)  # the environment's origin: where the task file's frame sits in the stage
    q1 = _quat_of(spec)
    joint.CreateBody1Rel().SetTargets([prim.GetPath()])
    joint.CreateLocalPos1Attr(Gf.Vec3f(*_qrot(_qconj(q1), tuple(a - p for a, p in zip(anchor, spec.pos)))))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    if spec.to and spec.to not in _PLACED:
        raise KeyError(f"{spec.name}: {spec.to} was not placed through this module's hook (import simready first)")
    other = _PLACED.get(spec.to)
    if other is not None and other[2]:
        pos, q0, _ = other
        joint.CreateBody0Rel().SetTargets([Sdf.Path(prim.GetPath().GetParentPath().AppendChild(spec.to))])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*_qrot(_qconj(q0), tuple(a - p for a, p in zip(anchor, pos)))))
        rot0 = _qmul(_qconj(q0), q1)
    else:  # the bench: the joint's other side is the world, so its frame is in stage coordinates
        joint.CreateLocalPos0Attr(Gf.Vec3f(*(Gf.Vec3d(*anchor) + origin)))
        rot0 = q1
    joint.CreateLocalRot0Attr(Gf.Quatf(rot0[3], rot0[0], rot0[1], rot0[2]))


def _hinge(prim, spec: Hinged) -> None:
    """The revolute joint through the pivot, with its limits and its spring drive."""
    from pxr import UsdPhysics

    joint = UsdPhysics.RevoluteJoint.Define(prim.GetStage(), prim.GetPath().AppendChild("hinge"))
    _anchor(joint, prim, spec, spec.pivot)
    joint.CreateAxisAttr(spec.axis)
    joint.CreateLowerLimitAttr(float(spec.lower))
    joint.CreateUpperLimitAttr(float(spec.upper))
    joint.CreateCollisionEnabledAttr(True)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(float(spec.stiffness))
    drive.CreateDampingAttr(float(spec.damping))
    drive.CreateTargetPositionAttr(float(spec.start_target))


def _spawn_cuboid(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Isaac Lab's ``spawn_cuboid``, then the joint of a hinged box and the :func:`on_spawn` callbacks. Installed in
    place of the original, which the spawner configurations name by module and attribute, so a replacement on one
    configuration would be lost."""
    prim = _SPAWN["cuboid"](prim_path, cfg, translation, orientation, **kwargs)
    name = prim_path.rstrip("/").rsplit("/", 1)[-1]
    spec = _SPECS.get(name)
    if isinstance(spec, Hinged):
        _hinge(prim, spec)
        _PRIMS[spec.name] = prim
    if name in _PLACED:
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


# -- steering a hinge's drive at run time ----------------------------------------------------------------------------


def target(name: str) -> float:
    """The drive target (degrees) of the hinged box ``name`` now."""
    return _TARGETS.get(name, _SPECS[name].start_target if name in _SPECS else math.nan)


def set_target(name: str, degrees: float) -> None:
    """Move the drive target of the hinged box ``name`` (a no-op if it is already there). Call it from an
    :func:`on_step` callback: PhysX reads the change before the next physics step."""
    if name not in _PRIMS:
        raise KeyError(f"{name} is not a hinged box (or has not spawned: joints exist inside the simulation)")
    if target(name) == degrees:
        return
    _PRIMS[name].GetChild("hinge").GetAttribute(DRIVE).Set(float(degrees))
    _TARGETS[name] = float(degrees)


def _restore_targets(env=None) -> None:
    for name in list(_TARGETS):
        _PRIMS[name].GetChild("hinge").GetAttribute(DRIVE).Set(float(_SPECS[name].start_target))
    _TARGETS.clear()


# -- hooking yam_arena -----------------------------------------------------------------------------------------------

_RESET: list = [_restore_targets]
_STEP: list = []


def on_spawn(fn):
    """Register ``fn(name, prim)`` to run on every task-file box right after it spawns, before the simulation
    starts (to add to its prim, as ``looks.py`` and ``shell.py`` do)."""
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
            _PLACED[spec.name] = (tuple(spec.pos), _quat_of(spec), spec.free)
        if isinstance(spec, Hinged):
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
