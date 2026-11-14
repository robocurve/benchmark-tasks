"""bonds: a hinge that is stuck by layers, each a capped torsion spring, that can be switched off one by one and broken off.

yam_arena's task files spawn plain primitives. This module adds one box that gets PhysX joints when it spawns:

- :class:`Hinged`: a ``Box`` with a mass on a revolute joint about one of its own axes through ``hinge`` (a point in the
  box's frame), turning from ``lower`` to ``upper`` degrees from where it spawns, its other side a free object or the
  bench. ``layers`` are stuck-hinge layers, each a triple (stiffness in N m/rad, damping in N m s/rad, cap in N m): a
  layer is a second joint at the hinge whose twist is held by a drive toward the angle it spawned at, a torsion spring
  of that stiffness whose torque is capped at that value (above it the layer slips and drags with its cap). The layers
  add up. :func:`unstick` switches the next layer off, for the rest of the episode (every reset switches them all on
  again), so the hinge's grip falls in steps until it turns freely. A print-in-place hinge that the printer fused: the
  more it is worked, the looser it gets. :func:`snap` breaks the box off: its hinge and every layer are switched off, for
  the rest of the episode (every reset switches them on again), and it is a free body.

:func:`hinge_angle` reads a box's turn about a hinge axis from two orientations.

Adapted from the bonds module of the printer-door task (that file's catch, zones, bonds and slides are gone, the layers
and the snap are new).

Importing this module hooks ``yam_arena.scene.task_object`` when that module is imported (after Isaac Sim starts), and
``yam_arena.inspect_embodiment.YamArenaEmbodiment`` so a task file can run code after every reset (:func:`on_reset`)
and before every scoring step (:func:`on_step`). Nothing in yam_arena is changed. Pure Python at import time:
everything that needs Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

from yam_arena.task_file import Box

__all__ = ["Hinged", "axis_frame", "hinge_angle", "install", "on_reset", "on_spawn", "on_step", "snap",
           "snapped", "stuck_layers", "unstick"]

_AXES = ("X", "Y", "Z")


@dataclass(frozen=True)
class Hinged(Box):
    """A free box on a revolute joint about its own ``axis`` ("X", "Y" or "Z", the box's own, which are the world's at
    ``yaw`` 0) through ``hinge`` (m, from the box's centre, in its own frame), free to turn from ``lower`` to ``upper``
    degrees from where it spawns (the right-hand rule about the axis). The joint's other side is the free object
    ``to``, or the bench (a fixed object counts as the bench). ``damping`` is the hinge's own friction (N m s/rad).
    ``layers`` are (stiffness, damping, cap) triples of stuck-hinge layers, see the module's text."""

    to: str = ""
    axis: str = "Z"
    hinge: tuple[float, float, float] = (0.0, 0.0, 0.0)
    lower: float = -90.0
    upper: float = 0.0
    damping: float = 0.01
    layers: tuple[tuple[float, float, float], ...] = ()

    def _check(self) -> None:
        super()._check()
        if self.mass is None:
            raise ValueError(f"{self.name}: a hinged box must have a mass")
        if self.axis not in _AXES:
            raise ValueError(f"{self.name}.axis must be X, Y or Z")
        if not (self.lower <= 0.0 <= self.upper and self.lower < self.upper):
            raise ValueError(f"{self.name}: lower <= 0 <= upper is where it spawns, and the range must not be empty")
        if self.damping < 0 or any(k <= 0 or d < 0 or c <= 0 for k, d, c in self.layers):
            raise ValueError(f"{self.name}: damping must not be negative, a layer's stiffness and cap must be positive, its damping not negative")
        if self.to and not self.to.isidentifier():
            raise ValueError(f"{self.name}.to must name an object")


# -- joint frames ----------------------------------------------------------------------------------------------------


def _quat_mul_wxyz(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def axis_frame(axis: str) -> tuple[float, float, float, float]:
    """The quaternion (w, x, y, z) that turns a joint frame's own X axis onto the world axis ``axis``. A generic joint
    twists about its frame's X."""
    c = math.sqrt(0.5)
    return {"X": (1.0, 0.0, 0.0, 0.0), "Y": (c, 0.0, 0.0, c), "Z": (c, 0.0, -c, 0.0)}[axis]


def _rz(yaw: float, v):
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * v[0] - s * v[1], s * v[0] + c * v[1], v[2])


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def hinge_angle(spec, quat_xyzw, other_xyzw=(0.0, 0.0, 0.0, 1.0), other_yaw: float = 0.0) -> float:
    """A box's angle (degrees) from where it spawned: its turn about ``spec.axis`` relative to the body it is hinged
    to (orientation ``other_xyzw``, spawned at ``other_yaw``; the bench by default)."""
    ox, oy, oz, ow = (float(v) for v in other_xyzw)
    rel = _quat_mul((-ox, -oy, -oz, ow), tuple(float(v) for v in quat_xyzw))  # the box in the other's frame
    turn = (spec.yaw - other_yaw) / 2  # remove the spawn turn between them
    rel = _quat_mul((0.0, 0.0, -math.sin(turn), math.cos(turn)), rel)
    qx, qy, qz, qw = rel
    k = {"X": qx, "Y": qy, "Z": qz}[spec.axis]
    if qw < 0:
        qw, k = -qw, -k
    return math.degrees(2.0 * math.atan2(k, qw))


# -- spawning --------------------------------------------------------------------------------------------------------

_SPECS: dict[str, Hinged] = {}
_PLACED: dict[str, tuple] = {}  # every task-file shape's (pos, yaw, free), to anchor a joint on it
_PRIMS: dict = {}  # each jointed box's spawned prim, to find its joints at run time
_SPAWN: dict = {}
_SPAWNED: list = []  # callbacks run on every task-file box as it spawns
_OFF: dict[str, int] = {}  # how many layers of each hinge are switched off
_SNAPPED: set[str] = set()


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
    # box as placed.
    turn = (spec.yaw - yaw) / 2
    joint.CreateLocalRot0Attr(Gf.Quatf(math.cos(turn), 0.0, 0.0, math.sin(turn)))


def _anchor_point(spec) -> tuple[float, float, float]:
    h = _rz(spec.yaw, spec.hinge)
    return tuple(p + d for p, d in zip(spec.pos, h))


def _revolute(prim, spec, name: str, lower: float, upper: float):
    from pxr import UsdPhysics

    joint = UsdPhysics.RevoluteJoint.Define(prim.GetStage(), prim.GetPath().AppendChild(name))
    _anchor(joint, prim, spec, _anchor_point(spec))
    joint.CreateAxisAttr(spec.axis)
    joint.CreateLowerLimitAttr(float(lower))
    joint.CreateUpperLimitAttr(float(upper))
    joint.CreateCollisionEnabledAttr(False)  # the two parts of a printed hinge are one piece of plastic here
    return joint


def _angular_drive(joint, stiffness: float, damping: float, cap: float | None = None, dof: str = "angular") -> None:
    """USD angular drives take their gains per degree."""
    from pxr import UsdPhysics

    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), dof)
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(float(stiffness) * math.pi / 180.0)
    drive.CreateDampingAttr(float(damping) * math.pi / 180.0)
    drive.CreateTargetPositionAttr(0.0)
    drive.CreateTargetVelocityAttr(0.0)
    if cap is not None:
        drive.CreateMaxForceAttr(float(cap))


def _hinge(prim, spec: Hinged) -> None:
    """The revolute joint of a hinged box, with its limits and its own friction, and one extra joint per friction
    layer: a generic joint at the same place, free in every way, with only its twist held by a capped spring drive."""
    from pxr import Gf, UsdPhysics

    joint = _revolute(prim, spec, "hinge", spec.lower, spec.upper)
    _angular_drive(joint, 0.0, spec.damping)
    frame = axis_frame(spec.axis)
    for k, (stiffness, damping, cap) in enumerate(spec.layers):
        layer = UsdPhysics.Joint.Define(prim.GetStage(), prim.GetPath().AppendChild(f"stuck{k}"))
        _anchor(layer, prim, spec, _anchor_point(spec))
        # Both frames turned so that the joint's own X, the axis a generic joint twists about, is the hinge's axis.
        turn = layer.GetLocalRot0Attr().Get()
        layer.CreateLocalRot1Attr(Gf.Quatf(*frame))
        w, x, y, z = _quat_mul_wxyz((turn.GetReal(), *turn.GetImaginary()), frame)
        layer.CreateLocalRot0Attr(Gf.Quatf(w, x, y, z))
        layer.CreateCollisionEnabledAttr(False)
        _angular_drive(layer, stiffness, damping, cap, "rotX")


def _spawn_cuboid(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Isaac Lab's ``spawn_cuboid``, then the joints of a hinged box. Installed in place of the original,
    which the spawner configurations name by module and attribute, so a replacement on one configuration would be
    lost."""
    prim = _SPAWN["cuboid"](prim_path, cfg, translation, orientation, **kwargs)
    name = prim_path.rstrip("/").rsplit("/", 1)[-1]
    spec = _SPECS.get(name)
    if isinstance(spec, Hinged):
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


# -- changing the joints at run time ---------------------------------------------------------------------------------


def _set_enabled(prim, enabled: bool) -> None:
    from pxr import UsdPhysics

    UsdPhysics.Joint(prim).CreateJointEnabledAttr(enabled)


def stuck_layers(name: str) -> int:
    """How many friction layers of the hinged box ``name`` are still on."""
    return len(_SPECS[name].layers) - _OFF.get(name, 0)


def unstick(name: str) -> int:
    """Switch the next friction layer of the hinged box ``name`` off for the rest of the episode. Returns how many are
    still on. Call it from an :func:`on_step` callback."""
    if name not in _PRIMS:
        raise KeyError(f"{name} has not spawned (unstick() runs inside the simulation)")
    off = _OFF.get(name, 0)
    if off < len(_SPECS[name].layers):
        _set_enabled(_PRIMS[name].GetChild(f"stuck{off}"), False)
        _OFF[name] = off + 1
    return stuck_layers(name)


def snap(name: str) -> None:
    """Break the hinged box ``name`` off for the rest of the episode: its hinge and every layer are switched off."""
    if name in _SNAPPED:
        return
    if name not in _PRIMS:
        raise KeyError(f"{name} has not spawned (snap() runs inside the simulation)")
    prim = _PRIMS[name]
    _set_enabled(prim.GetChild("hinge"), False)
    for k in range(len(_SPECS[name].layers)):
        _set_enabled(prim.GetChild(f"stuck{k}"), False)
    _SNAPPED.add(name)


def snapped(name: str) -> bool:
    """Whether :func:`snap` has broken ``name`` off this episode."""
    return name in _SNAPPED


def _restore(env=None) -> None:
    for name in list(_SNAPPED):
        prim = _PRIMS[name]
        _set_enabled(prim.GetChild("hinge"), True)
        for k in range(len(_SPECS[name].layers)):
            _set_enabled(prim.GetChild(f"stuck{k}"), True)
    for name, off in list(_OFF.items()):
        if name in _SNAPPED:
            continue
        for k in range(off):
            _set_enabled(_PRIMS[name].GetChild(f"stuck{k}"), True)
    _OFF.clear()
    _SNAPPED.clear()


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
