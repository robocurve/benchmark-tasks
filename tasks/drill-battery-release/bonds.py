"""bonds: task-file boxes on PhysX joints to other objects, switched on and off at run time.

yam_arena's task files spawn plain primitives. This module adds boxes that get PhysX joints when they spawn:

- :class:`Sliding`: a ``Box`` with a mass on a prismatic joint ("slide") along one of its own axes, between ``lower``
  and ``upper`` (metres from where it spawns, which is its rest), pushed back to rest by a spring of ``rate`` N/m
  with ``damping`` N s/m. A button. The other side of the joint is the object ``to``, or the bench. ``collide=False``
  lets the two bodies pass through each other (a button that sinks into its housing).
- :class:`Slotted`: a ``Box`` with a mass that slides on the object ``to`` along one of its own axes, like a battery
  pack on a drill's foot. Two joints: a prismatic "slide" from ``lower`` to ``upper`` metres, held back by a steady
  ``friction`` (N, a velocity drive with that force limit), and a fixed "lock" that holds it where it spawns.
  :func:`set_joint` switches either joint off and on, so a task can unlock the pack while its buttons are pressed
  and free it once it has slid off.

The other side of a joint may be a task-file box or a ``simready.SimReady`` prop with a full orientation
(``quat_xyzw``): its frame is taken from its spec. Import ``simready`` before this module, so that this module's hook
on ``yam_arena.scene.task_object`` runs first and sees every object.

Importing this module hooks ``yam_arena.scene.task_object`` when that module is imported (after Isaac Sim starts),
and ``yam_arena.inspect_embodiment.YamArenaEmbodiment`` so a task file can run code after every reset
(:func:`on_reset`) and before every scoring step (:func:`on_step`). Every reset switches each joint back to how it
spawned. Nothing in yam_arena is changed. ``success()`` and ``score()`` get the usual ``ObjectState``. Pure Python at
import time: everything that needs Isaac Sim is imported lazily.

Adapted from the bonds module of the latch-and-lift-graphics-card task (full orientations, the slotted box, and
joints switched both ways).
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

from yam_arena.task_file import Box

__all__ = ["Sliding", "Slotted", "install", "joint_enabled", "on_reset", "on_spawn", "on_step", "set_joint"]


def _check_to(spec) -> None:
    if spec.mass is None:
        raise ValueError(f"{spec.name}: a jointed box must have a mass")
    if spec.axis not in ("X", "Y", "Z"):
        raise ValueError(f"{spec.name}.axis must be X, Y or Z")
    if not (spec.lower <= 0.0 <= spec.upper and spec.lower < spec.upper):
        raise ValueError(f"{spec.name}: lower <= 0 <= upper is where it spawns, and the range must not be empty")
    if spec.to and not spec.to.isidentifier():
        raise ValueError(f"{spec.name}.to must name an object")


@dataclass(frozen=True)
class Sliding(Box):
    """A free box on a prismatic joint along its ``axis`` ("X", "Y" or "Z", the box's own, which are the world's at
    ``yaw`` 0), free to move from ``lower`` to ``upper`` metres from where it spawns, and sprung back there with
    ``rate`` N/m and ``damping`` N s/m. The joint's other side is the free object ``to``, or the bench. With
    ``collide=False`` the box and ``to`` don't collide."""

    to: str = ""
    axis: str = "Z"
    lower: float = -0.004
    upper: float = 0.0
    rate: float = 1000.0
    damping: float = 10.0
    collide: bool = True

    def _check(self) -> None:
        super()._check()
        _check_to(self)
        if self.rate < 0 or self.damping < 0:
            raise ValueError(f"{self.name}: rate and damping must not be negative")


@dataclass(frozen=True)
class Slotted(Box):
    """A free box that slides on the free object ``to`` along its own ``axis``, from ``lower`` to ``upper`` metres
    from where it spawns, against a steady ``friction`` (N), and is locked where it spawns by a fixed joint until
    the task switches the "lock" off with :func:`set_joint`. The two collide."""

    to: str = ""
    axis: str = "X"
    lower: float = -0.06
    upper: float = 0.0
    friction: float = 10.0

    def _check(self) -> None:
        super()._check()
        _check_to(self)
        if not self.to:
            raise ValueError(f"{self.name}: a slotted box slides on another object")
        if self.friction < 0:
            raise ValueError(f"{self.name}: friction must not be negative")


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

_SPECS: dict[str, Sliding | Slotted] = {}
_PLACED: dict[str, tuple] = {}  # every task-file object's (pos, quat_xyzw, free), to anchor a joint on it
_PRIMS: dict = {}  # each jointed box's spawned prim, to find its joints at run time
_SPAWNED_ON: dict[tuple[str, str], bool] = {}  # (box, joint): whether it spawned switched on
_NOW_ON: dict[tuple[str, str], bool] = {}  # (box, joint): switched since the last reset
_SPAWN: dict = {}
_SPAWNED: list = []  # callbacks run on every task-file box as it spawns


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


def _prismatic(prim, spec, name):
    from pxr import UsdPhysics

    joint = UsdPhysics.PrismaticJoint.Define(prim.GetStage(), prim.GetPath().AppendChild(name))
    _anchor(joint, prim, spec, spec.pos)
    joint.CreateAxisAttr(spec.axis)
    joint.CreateLowerLimitAttr(float(spec.lower))
    joint.CreateUpperLimitAttr(float(spec.upper))
    return joint, UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")


def _slide(prim, spec: Sliding) -> None:
    """The sprung prismatic joint under the spawned box, anchored at its centre."""
    joint, drive = _prismatic(prim, spec, "slide")
    joint.CreateCollisionEnabledAttr(bool(spec.collide))
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(float(spec.rate))
    drive.CreateDampingAttr(float(spec.damping))
    drive.CreateTargetPositionAttr(0.0)
    _SPAWNED_ON[(spec.name, "slide")] = True


def _slot(prim, spec: Slotted) -> None:
    """The slide (prismatic, with a friction drive: zero target speed, stiff damping, force limit ``friction``) and
    the lock (fixed, at the spawn pose)."""
    from pxr import UsdPhysics

    joint, drive = _prismatic(prim, spec, "slide")
    joint.CreateCollisionEnabledAttr(True)
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(1.0e4)
    drive.CreateTargetVelocityAttr(0.0)
    drive.CreateMaxForceAttr(float(spec.friction))
    lock = UsdPhysics.FixedJoint.Define(prim.GetStage(), prim.GetPath().AppendChild("lock"))
    _anchor(lock, prim, spec, spec.pos)
    lock.CreateCollisionEnabledAttr(True)
    _SPAWNED_ON[(spec.name, "slide")] = True
    _SPAWNED_ON[(spec.name, "lock")] = True


def _spawn_cuboid(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Isaac Lab's ``spawn_cuboid``, then the joints of a sliding or slotted box. Installed in place of the
    original, which the spawner configurations name by module and attribute, so a replacement on one configuration
    would be lost."""
    prim = _SPAWN["cuboid"](prim_path, cfg, translation, orientation, **kwargs)
    name = prim_path.rstrip("/").rsplit("/", 1)[-1]
    spec = _SPECS.get(name)
    if isinstance(spec, Sliding):
        _slide(prim, spec)
    elif isinstance(spec, Slotted):
        _slot(prim, spec)
    if spec is not None:
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


# -- switching joints at run time ------------------------------------------------------------------------------------


def joint_enabled(name: str, joint: str) -> bool:
    """Whether the joint ``joint`` ("slide" or "lock") of the box ``name`` is switched on."""
    key = (name, joint)
    return _NOW_ON.get(key, _SPAWNED_ON[key])


def set_joint(name: str, joint: str, enabled: bool) -> None:
    """Switch the joint ``joint`` of the box ``name`` on or off (a no-op if it already is). Call it from an
    :func:`on_step` callback. A fixed joint switched back on pulls the box back to where it spawned on the other
    body."""
    from pxr import UsdPhysics

    key = (name, joint)
    if key not in _SPAWNED_ON:
        raise KeyError(f"{name} has no joint {joint!r} (or has not spawned: joints exist inside the simulation)")
    if joint_enabled(name, joint) == enabled:
        return
    UsdPhysics.Joint(_PRIMS[name].GetChild(joint)).CreateJointEnabledAttr(bool(enabled))
    _NOW_ON[key] = bool(enabled)


def _restore_joints(env=None) -> None:
    for (name, joint), on in list(_NOW_ON.items()):
        if on != _SPAWNED_ON[(name, joint)]:
            from pxr import UsdPhysics

            UsdPhysics.Joint(_PRIMS[name].GetChild(joint)).CreateJointEnabledAttr(_SPAWNED_ON[(name, joint)])
    _NOW_ON.clear()


# -- hooking yam_arena -----------------------------------------------------------------------------------------------

_RESET: list = [_restore_joints]
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
            _PLACED[spec.name] = (tuple(spec.pos), _quat_of(spec), spec.free)
        if isinstance(spec, (Sliding, Slotted)):
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
