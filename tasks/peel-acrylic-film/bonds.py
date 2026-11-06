"""bonds: task-file boxes held by a breakable bond, sliding on a sprung joint, or joined by hinges and glue.

yam_arena's task files spawn plain primitives. This module adds two boxes that get a PhysX joint when they spawn, and
:class:`Joint`, any number of further fixed or revolute joints between task-file boxes:

- :class:`Bonded`: a ``Box`` with a mass that starts fixed to the object named ``to`` (or to the bench, when ``to``
  is empty or names a fixed object) by a fixed joint whose ``physics:breakForce`` is ``hold`` newtons and
  ``physics:breakTorque`` is ``twist`` newton metres. Pull or push harder than ``hold``, or turn it harder than
  ``twist``, and the bond breaks for the rest of the trial. The joint sits at the centre of the box's bottom face.
  Old thermal paste, a glued-on label, a card locked in its slot. :func:`release` undoes a bond on cue (a latch that
  frees the card) by disabling its joint, and every reset enables the released bonds again.
- :class:`Sliding`: a ``Box`` with a mass on a prismatic joint along one of its own axes, between ``lower`` and
  ``upper`` (metres from where it spawns, which is its rest), pushed back to rest by a spring of ``rate`` N/m with
  ``damping`` N s/m. A button, a latch, a drawer. The other side of the joint is the object ``to``, or the bench.

- :class:`Joint`: a joint authored under the box ``body`` when it spawns, to the box ``to`` (or the bench), at the
  point ``at`` of the task file's frame. ``kind="fixed"`` glues the two, ``kind="revolute"`` hinges them about the
  body's own ``axis``, with optional limits, a spring and damping. Either breaks at ``hold`` newtons or ``twist``
  newton metres (PhysX's ``breakForce`` and ``breakTorque``, about the joint's point). Register it with :func:`add`.
- :func:`tune`: solver iterations and damping for one box's rigid body (a light chain of hinged boxes needs more
  iterations than the default to stay still).

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

__all__ = ["Bonded", "Joint", "Sliding", "add", "install", "on_reset", "on_step", "release", "released", "tune"]


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
class Joint:
    """A joint authored under the box ``body`` (the joint's body 1) when it spawns. Its other side (body 0) is the box
    ``to``, or the bench when ``to`` is empty or names a fixed box. ``at`` is the joint's point in the task file's
    frame. ``kind`` is "fixed" (glue), "revolute" (a hinge about the body's own ``axis``, "X", "Y" or "Z", which
    are the world's at ``yaw`` 0) or "ball" (the point is held, every rotation is free: a PhysX D6 joint with its
    three translations locked). A hinge turns freely, or between ``lower`` and ``upper`` degrees from where it
    spawns. A hinge or a ball is pulled back to where it spawned by a spring of ``stiffness`` N m/rad with ``damping``
    N m s/rad (a ball on each of its three axes). ``hold`` (N) and ``twist`` (N m) break it, ``math.inf`` for never.
    ``collide`` lets the two boxes collide with each other."""

    name: str
    body: str
    at: tuple[float, float, float]
    to: str = ""
    kind: str = "fixed"
    axis: str = "Y"
    lower: float | None = None
    upper: float | None = None
    stiffness: float = 0.0
    damping: float = 0.0
    hold: float = math.inf
    twist: float = math.inf
    collide: bool = True

    def __post_init__(self) -> None:
        if not (self.name.isidentifier() and self.body.isidentifier()):
            raise ValueError(f"joint {self.name!r}: name and body must be identifiers")
        if self.to and not self.to.isidentifier():
            raise ValueError(f"joint {self.name}: to must name an object")
        if self.kind not in ("fixed", "revolute", "ball"):
            raise ValueError(f"joint {self.name}: kind must be fixed, revolute or ball")
        if self.kind == "ball" and self.lower is not None:
            raise ValueError(f"joint {self.name}: a ball joint has no limits")
        if self.axis not in ("X", "Y", "Z"):
            raise ValueError(f"joint {self.name}: axis must be X, Y or Z")
        if len(self.at) != 3:
            raise ValueError(f"joint {self.name}: at must be a point (x, y, z)")
        if not (self.hold > 0 and self.twist > 0):
            raise ValueError(f"joint {self.name}: hold and twist must be positive (math.inf for unbreakable)")
        if self.stiffness < 0 or self.damping < 0:
            raise ValueError(f"joint {self.name}: stiffness and damping must not be negative")
        if (self.lower is None) != (self.upper is None) or (self.lower is not None and not self.lower < self.upper):
            raise ValueError(f"joint {self.name}: give both limits, lower < upper, or neither")


# -- spawning --------------------------------------------------------------------------------------------------------

_SPECS: dict[str, Bonded | Sliding] = {}
_PLACED: dict[str, tuple] = {}  # every task-file shape's (pos, yaw, free), to anchor a joint on it
_PRIMS: dict = {}  # each jointed box's spawned prim, to find its joint at run time
_RELEASED: set[str] = set()
_SPAWN: dict = {}
_SPAWNED: list = []  # callbacks run on every task-file box as it spawns
_JOINTS: dict[str, list[Joint]] = {}  # the extra joints, by the box they are authored under
_TUNE: dict[str, dict] = {}  # rigid-body settings, by box


def _rz(yaw: float, v):
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * v[0] - s * v[1], s * v[0] + c * v[1], v[2])


def _anchor(joint, prim, spec, anchor) -> None:
    """Author the joint's two frames: body 1 is the spawned box, body 0 the object ``spec.to`` or the world.
    ``anchor`` is the joint's position in the task file's frame."""
    _frames(joint, prim, tuple(spec.pos), float(spec.yaw), spec.to, anchor)


def _frames(joint, prim, body_pos, body_yaw, to, anchor) -> None:
    """:func:`_anchor` for a box placed at ``body_pos`` turned ``body_yaw``, joined to the object ``to``."""
    from pxr import Gf, Sdf, Usd, UsdGeom

    spec_pos, spec_yaw = body_pos, body_yaw
    world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
    origin = world - Gf.Vec3d(*spec_pos)  # the environment's origin: where the task file's frame sits in the stage
    joint.CreateBody1Rel().SetTargets([prim.GetPath()])
    joint.CreateLocalPos1Attr(Gf.Vec3f(*_rz(-spec_yaw, tuple(a - p for a, p in zip(anchor, spec_pos)))))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    other = _PLACED.get(to)
    if other is not None and other[2]:
        pos, yaw, _ = other
        joint.CreateBody0Rel().SetTargets([Sdf.Path(prim.GetPath().GetParentPath().AppendChild(to))])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*_rz(-yaw, tuple(a - p for a, p in zip(anchor, pos)))))
    else:  # the bench: the joint's other side is the world, so its frame is in stage coordinates
        yaw = 0.0
        joint.CreateLocalPos0Attr(Gf.Vec3f(*(Gf.Vec3d(*anchor) + origin)))
    # Frame 1 is the box's own axes, so frame 0 is turned by the box's yaw relative to body 0: the joint holds the
    # box as placed, and a sliding box moves along its own axis.
    turn = (spec_yaw - yaw) / 2
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


def _author(prim, j: Joint) -> None:
    """The extra joint ``j`` under the spawned box ``prim``."""
    from pxr import UsdPhysics

    stage, path = prim.GetStage(), prim.GetPath().AppendChild(j.name)
    if j.kind == "fixed":
        joint = UsdPhysics.FixedJoint.Define(stage, path)
    elif j.kind == "ball":
        # A D6 joint: the three translations locked (a limit whose low is above its high locks the axis), the three
        # rotations free, each with an optional spring.
        joint = UsdPhysics.Joint.Define(stage, path)
        for dof in ("transX", "transY", "transZ"):
            limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), dof)
            limit.CreateLowAttr(1.0)
            limit.CreateHighAttr(-1.0)
        if j.stiffness or j.damping:
            for dof in ("rotX", "rotY", "rotZ"):
                drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), dof)
                drive.CreateTypeAttr("force")
                drive.CreateStiffnessAttr(float(j.stiffness) * math.pi / 180)
                drive.CreateDampingAttr(float(j.damping) * math.pi / 180)
                drive.CreateTargetPositionAttr(0.0)
    else:
        joint = UsdPhysics.RevoluteJoint.Define(stage, path)
        joint.CreateAxisAttr(j.axis)
        if j.lower is not None:
            joint.CreateLowerLimitAttr(float(j.lower))
            joint.CreateUpperLimitAttr(float(j.upper))
        if j.stiffness or j.damping:
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
            drive.CreateTypeAttr("force")
            # USD angular drives are per degree
            drive.CreateStiffnessAttr(float(j.stiffness) * math.pi / 180)
            drive.CreateDampingAttr(float(j.damping) * math.pi / 180)
            drive.CreateTargetPositionAttr(0.0)
    pos, yaw, _ = _PLACED[j.body]
    _frames(joint, prim, pos, yaw, j.to, j.at)
    joint.CreateCollisionEnabledAttr(bool(j.collide))
    if math.isfinite(j.hold):
        joint.CreateBreakForceAttr(float(j.hold))
    if math.isfinite(j.twist):
        joint.CreateBreakTorqueAttr(float(j.twist))


def _tune(prim, settings: dict) -> None:
    from pxr import Sdf

    if "PhysxRigidBodyAPI" not in prim.GetAppliedSchemas():
        prim.AddAppliedSchema("PhysxRigidBodyAPI")
    kinds = {"solverPositionIterationCount": Sdf.ValueTypeNames.UInt,
             "solverVelocityIterationCount": Sdf.ValueTypeNames.UInt}
    for key, value in settings.items():
        kind = kinds.get(key, Sdf.ValueTypeNames.Float)
        prim.CreateAttribute(f"physxRigidBody:{key}", kind).Set(int(value) if key in kinds else float(value))


def add(*joints: Joint) -> None:
    """Register joints to author when their ``body`` spawns."""
    for j in joints:
        _JOINTS.setdefault(j.body, []).append(j)


def tune(name: str, **settings) -> None:
    """PhysX rigid-body settings for the box ``name`` (``physxRigidBody:<key>``), for example
    ``solverPositionIterationCount=32`` or ``maxDepenetrationVelocity=0.3``."""
    _TUNE.setdefault(name, {}).update(settings)


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
    if spec is not None:
        _PRIMS[spec.name] = prim
    for j in _JOINTS.get(name, ()):
        _author(prim, j)
    if name in _TUNE:
        _tune(prim, _TUNE[name])
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

_RESET: list = [_restore_bonds]
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
        if isinstance(spec, (Bonded, Sliding)):
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
