"""bonds: task-file boxes held by a breakable bond, sliding on a sprung joint or turning on a hinge with detents.

yam_arena's task files spawn plain primitives. This module adds three boxes that get a PhysX joint when they spawn:

- :class:`Bonded`: a ``Box`` with a mass that starts fixed to the object named ``to`` (or to the bench, when ``to``
  is empty or names a fixed object) by a fixed joint whose ``physics:breakForce`` is ``hold`` newtons and
  ``physics:breakTorque`` is ``twist`` newton metres. Pull or push harder than ``hold``, or turn it harder than
  ``twist``, and the bond breaks for the rest of the trial. The joint sits at the centre of the box's bottom face.
  Old thermal paste, a glued-on label, a card locked in its slot. :func:`release` undoes a bond on cue (a latch that
  frees the card) by disabling its joint, and every reset enables the released bonds again.
- :class:`Sliding`: a ``Box`` with a mass on a prismatic joint along one of its own axes, between ``lower`` and
  ``upper`` (metres from where it spawns, which is its rest), pushed back to rest by a spring of ``rate`` N/m with
  ``damping`` N s/m. A button, a latch, a drawer. The other side of the joint is the object ``to``, or the bench.
- :class:`Hinged`: a ``Box`` with a mass on a revolute joint about one of its own axes through ``hinge`` (a point in
  the box's frame), turning from ``lower`` to ``upper`` degrees from where it spawns. A sprung drive of ``rate``
  N m/rad with ``damping`` N m s/rad pulls it to the nearest of its ``detents`` (degrees): past the midpoint between
  two detents, the drive's target switches to the next one, so a lever stays where it was pushed (an over-centre
  spring). :func:`hinge_angle` reads the angle. A ZIF socket's lever, a toggle, a lid.

Adapted from the bonds module of the latch-and-lift-graphics-card task (the hinge is new).

Importing this module hooks ``yam_arena.scene.task_object`` when that module is imported (after Isaac Sim starts),
and ``yam_arena.inspect_embodiment.YamArenaEmbodiment`` so a task file can run code after every reset
(:func:`on_reset`) and before every scoring step (:func:`on_step`). Nothing in yam_arena is changed. ``success()``
and ``score()`` get the usual ``ObjectState``. Pure Python at import time: everything that needs Isaac Sim is
imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

from yam_arena.task_file import Box

__all__ = ["Bonded", "Hinged", "Sliding", "hinge_angle", "install", "on_reset", "on_spawn", "on_step", "release",
           "released"]


@dataclass(frozen=True)
class Bonded(Box):
    """A free box bonded by its bottom face to the object ``to`` (a free box: the two move together until the bond
    breaks) or to the bench (``to`` empty, or a fixed object). ``hold`` is the force (N) and ``twist`` the torque
    (N m) that break the bond, ``math.inf`` for unbreakable."""

    to: str = ""
    hold: float = math.inf
    twist: float = math.inf

    def _check(self) -> None:
        super()._check()
        if self.mass is None:
            raise ValueError(f"{self.name}: a bonded box must have a mass")
        if not (self.hold > 0 and self.twist > 0):
            raise ValueError(f"{self.name}: hold and twist must be positive (math.inf for unbreakable)")
        if self.to and not self.to.isidentifier():
            raise ValueError(f"{self.name}.to must name an object")


@dataclass(frozen=True)
class Sliding(Box):
    """A free box on a prismatic joint along its ``axis`` ("X", "Y" or "Z", the box's own, which are the world's at
    ``yaw`` 0), free to move from ``lower`` to ``upper`` metres from where it spawns, and sprung back there with
    ``rate`` N/m and ``damping`` N s/m. The joint's other side is the free object ``to``, or the bench."""

    to: str = ""
    axis: str = "Z"
    lower: float = -0.004
    upper: float = 0.0
    rate: float = 1000.0
    damping: float = 10.0

    def _check(self) -> None:
        super()._check()
        if self.mass is None:
            raise ValueError(f"{self.name}: a sliding box must have a mass")
        if self.axis not in ("X", "Y", "Z"):
            raise ValueError(f"{self.name}.axis must be X, Y or Z")
        if not (self.lower <= 0.0 <= self.upper and self.lower < self.upper):
            raise ValueError(f"{self.name}: lower <= 0 <= upper is where it spawns, and the range must not be empty")
        if self.rate < 0 or self.damping < 0:
            raise ValueError(f"{self.name}: rate and damping must not be negative")
        if self.to and not self.to.isidentifier():
            raise ValueError(f"{self.name}.to must name an object")


@dataclass(frozen=True)
class Hinged(Box):
    """A free box on a revolute joint about its own ``axis`` ("X", "Y" or "Z", the box's own, which are the world's
    at ``yaw`` 0) through ``hinge`` (m, from the box's centre, in its own frame), free to turn from ``lower`` to
    ``upper`` degrees from where it spawns (the right-hand rule about the axis). A drive of ``rate`` N m/rad and
    ``damping`` N m s/rad holds it at the nearest of ``detents`` (degrees, within the limits). The joint's other
    side is the free object ``to``, or the bench."""

    to: str = ""
    axis: str = "X"
    hinge: tuple[float, float, float] = (0.0, 0.0, 0.0)
    lower: float = 0.0
    upper: float = 90.0
    detents: tuple[float, ...] = (0.0,)
    rate: float = 0.05
    damping: float = 0.005

    def _check(self) -> None:
        super()._check()
        if self.mass is None:
            raise ValueError(f"{self.name}: a hinged box must have a mass")
        if self.axis not in ("X", "Y", "Z"):
            raise ValueError(f"{self.name}.axis must be X, Y or Z")
        if not (self.lower <= 0.0 <= self.upper and self.lower < self.upper):
            raise ValueError(f"{self.name}: lower <= 0 <= upper is where it spawns, and the range must not be empty")
        if not self.detents or any(not (self.lower <= d <= self.upper) for d in self.detents):
            raise ValueError(f"{self.name}: every detent must be within the limits")
        if self.rate < 0 or self.damping < 0:
            raise ValueError(f"{self.name}: rate and damping must not be negative")
        if self.to and not self.to.isidentifier():
            raise ValueError(f"{self.name}.to must name an object")

    def detent_for(self, angle: float) -> float:
        """The detent the drive pulls toward at ``angle`` degrees: the nearest one."""
        return min(self.detents, key=lambda d: abs(d - angle))


# -- spawning --------------------------------------------------------------------------------------------------------

_SPECS: dict[str, Bonded | Sliding | Hinged] = {}
_PLACED: dict[str, tuple] = {}  # every task-file shape's (pos, yaw, free), to anchor a joint on it
_PRIMS: dict = {}  # each jointed box's spawned prim, to find its joint at run time
_RELEASED: set[str] = set()
_SPAWN: dict = {}
_SPAWNED: list = []  # callbacks run on every task-file box as it spawns


def _rz(yaw: float, v):
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * v[0] - s * v[1], s * v[0] + c * v[1], v[2])


def _anchor(joint, prim, spec, anchor) -> None:
    """Author the joint's two frames: body 1 is the spawned box, body 0 the object ``spec.to`` or the world.
    ``anchor`` is the joint's position in the task file's frame."""
    from pxr import Gf, Sdf, Usd, UsdGeom

    world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
    origin = world - Gf.Vec3d(*spec.pos)  # the environment's origin: where the task file's frame sits in the stage
    joint.CreateBody1Rel().SetTargets([prim.GetPath()])
    joint.CreateLocalPos1Attr(Gf.Vec3f(*_rz(-spec.yaw, tuple(a - p for a, p in zip(anchor, spec.pos)))))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    other = _PLACED.get(spec.to)
    if other is not None and other[2]:
        pos, yaw, _ = other
        joint.CreateBody0Rel().SetTargets([Sdf.Path(prim.GetPath().GetParentPath().AppendChild(spec.to))])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*_rz(-yaw, tuple(a - p for a, p in zip(anchor, pos)))))
    else:  # the bench: the joint's other side is the world, so its frame is in stage coordinates
        yaw = 0.0
        joint.CreateLocalPos0Attr(Gf.Vec3f(*(Gf.Vec3d(*anchor) + origin)))
    # Frame 1 is the box's own axes, so frame 0 is turned by the box's yaw relative to body 0: the joint holds the
    # box as placed, and a sliding box moves along its own axis.
    turn = (spec.yaw - yaw) / 2
    joint.CreateLocalRot0Attr(Gf.Quatf(math.cos(turn), 0.0, 0.0, math.sin(turn)))


def _bond(prim, spec: Bonded) -> None:
    """The fixed joint under the spawned box, anchored at the centre of its bottom face."""
    from pxr import UsdPhysics

    joint = UsdPhysics.FixedJoint.Define(prim.GetStage(), prim.GetPath().AppendChild("bond"))
    _anchor(joint, prim, spec, (spec.pos[0], spec.pos[1], spec.pos[2] - spec.size[2] / 2))
    joint.CreateCollisionEnabledAttr(True)  # the bonded faces keep touching after the bond breaks
    joint.CreateBreakForceAttr(float(spec.hold))
    joint.CreateBreakTorqueAttr(float(spec.twist))


def _slide(prim, spec: Sliding) -> None:
    """The prismatic joint under the spawned box, anchored at its centre, with its limits and spring."""
    from pxr import UsdPhysics

    joint = UsdPhysics.PrismaticJoint.Define(prim.GetStage(), prim.GetPath().AppendChild("slide"))
    _anchor(joint, prim, spec, spec.pos)
    joint.CreateAxisAttr(spec.axis)
    joint.CreateLowerLimitAttr(float(spec.lower))
    joint.CreateUpperLimitAttr(float(spec.upper))
    joint.CreateCollisionEnabledAttr(True)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(float(spec.rate))
    drive.CreateDampingAttr(float(spec.damping))
    drive.CreateTargetPositionAttr(0.0)


_AXES = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}
_TARGET: dict[str, float] = {}  # each hinge's drive target now (degrees)


def _hinge(prim, spec: Hinged) -> None:
    """The revolute joint of a hinged box, anchored at its hinge, with its limits and its detent drive."""
    import math as _m

    from pxr import UsdPhysics

    joint = UsdPhysics.RevoluteJoint.Define(prim.GetStage(), prim.GetPath().AppendChild("hinge"))
    h = _rz(spec.yaw, spec.hinge)
    _anchor(joint, prim, spec, tuple(p + d for p, d in zip(spec.pos, h)))
    joint.CreateAxisAttr(spec.axis)
    joint.CreateLowerLimitAttr(float(spec.lower))
    joint.CreateUpperLimitAttr(float(spec.upper))
    joint.CreateCollisionEnabledAttr(True)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    # USD angular drives take N m per degree and N m s per degree
    drive.CreateStiffnessAttr(float(spec.rate) * _m.pi / 180.0)
    drive.CreateDampingAttr(float(spec.damping) * _m.pi / 180.0)
    drive.CreateTargetPositionAttr(float(spec.detent_for(0.0)))
    _TARGET[spec.name] = spec.detent_for(0.0)


def hinge_angle(spec: Hinged, quat_xyzw) -> float:
    """The hinged box's angle (degrees) from where it spawned, from its orientation ``quat_xyzw`` (the joint's
    other side is the bench, so its turn about the hinge axis is the angle)."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    # remove the spawn yaw, then read the turn about the box's own axis
    c, s = math.cos(-spec.yaw / 2), math.sin(-spec.yaw / 2)
    qw, qx, qy, qz = c * w - s * z, c * x - s * y, c * y + s * x, c * z + s * w
    k = {"X": qx, "Y": qy, "Z": qz}[spec.axis]
    if qw < 0:
        qw, k = -qw, -k
    return math.degrees(2.0 * math.atan2(k, qw))


def _set_target(name: str, target: float) -> None:
    from pxr import UsdPhysics

    if _TARGET.get(name) == target:
        return
    UsdPhysics.DriveAPI(_PRIMS[name].GetChild("hinge"), "angular").GetTargetPositionAttr().Set(float(target))
    _TARGET[name] = target


def _detents(env) -> None:
    """Every step: each hinge's drive pulls toward the detent nearest its angle now."""
    hinges = [s for s in _SPECS.values() if isinstance(s, Hinged) and s.name in _PRIMS]
    if not hinges:
        return
    from yam_arena.scene import object_states

    states = object_states(env, [s.name for s in hinges])
    for spec in hinges:
        q = states[spec.name].quat_xyzw
        if all(math.isfinite(float(v)) for v in q):
            _set_target(spec.name, spec.detent_for(hinge_angle(spec, q)))


def _reset_detents(env=None) -> None:
    for spec in _SPECS.values():
        if isinstance(spec, Hinged) and spec.name in _PRIMS:
            _set_target(spec.name, spec.detent_for(0.0))


def _spawn_cuboid(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Isaac Lab's ``spawn_cuboid``, then the joint of a bonded or sliding box. Installed in place of the original,
    which the spawner configurations name by module and attribute, so a replacement on one configuration would be
    lost."""
    prim = _SPAWN["cuboid"](prim_path, cfg, translation, orientation, **kwargs)
    name = prim_path.rstrip("/").rsplit("/", 1)[-1]
    spec = _SPECS.get(name)
    if isinstance(spec, Bonded):
        _bond(prim, spec)
    elif isinstance(spec, Sliding):
        _slide(prim, spec)
    elif isinstance(spec, Hinged):
        _hinge(prim, spec)
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


# -- releasing a bond at run time ------------------------------------------------------------------------------------


def _set_bond(name: str, enabled: bool) -> None:
    from pxr import UsdPhysics

    joint = UsdPhysics.Joint(_PRIMS[name].GetChild("bond"))
    joint.CreateJointEnabledAttr(enabled)


def release(name: str) -> None:
    """Undo the bond of the Bonded box ``name`` for the rest of the episode: its joint is disabled, so the box is
    free from then on. A no-op if it is already released. Call it from an :func:`on_step` callback."""
    if name in _RELEASED:
        return
    if name not in _PRIMS:
        raise KeyError(f"{name} has not spawned (release() runs inside the simulation)")
    _set_bond(name, False)
    _RELEASED.add(name)


def released(name: str) -> bool:
    """Whether :func:`release` has undone ``name``'s bond this episode."""
    return name in _RELEASED


def _restore_bonds(env=None) -> None:
    for name in _RELEASED:
        _set_bond(name, True)
    _RELEASED.clear()


# -- hooking yam_arena -----------------------------------------------------------------------------------------------

_RESET: list = [_restore_bonds, _reset_detents]
_STEP: list = [_detents]


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
        if isinstance(spec, (Bonded, Sliding, Hinged)):
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
