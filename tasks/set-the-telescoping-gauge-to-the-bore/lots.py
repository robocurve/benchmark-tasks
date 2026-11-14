"""Layout lots for yam_arena task files: trial k of a run uses lot k mod N, so a fixed habit cannot score full marks.
Copy this file next to your task file and import it there:

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    import lots  # noqa: E402

    OBJECTS = [RED, BLUE]                     # as written, this is lot A: the layout in the screenshot
    LOTS = lots.declare(
        OBJECTS,
        lots.Lot("A", target="red"),
        lots.Lot("B", target="blue", place={"red": BLUE.pos, "blue": RED.pos}),
    )

    def score(objects):
        lot = lots.current()                  # the lot of the trial being run
        ...objects[lot.target]...
    def demo():
        lot = lots.current()
        pick = lot.pos("red")                 # where this lot puts the red cube
        ...

A :class:`Lot` has a name, optional ``place`` (object name -> centre (x, y, z), world frame, m) and ``yaw``
(object name -> rad) overrides, and any keyword "roles" you like (``target="blue"``, ``faulty=2``), read back as
attributes. Objects a lot does not mention start where ``OBJECTS`` puts them. Only free objects (with a mass) can be
moved by a lot: fixed ones are welded where they were spawned. Colours cannot change between trials (the looks are
built once with the scene); swap the places of differently coloured objects instead.

How trials map to lots: Inspect Robots calls the embodiment's optional ``prepare_trial(scene)`` once before every
trial, before the policy's and the embodiment's ``reset()``. Importing this module gives the ``yam_arena``
embodiment a ``prepare_trial`` that advances a trial counter, so with one instruction and ``--epochs N`` trial k
(epoch k) uses lot k mod N; lot A is always the first trial. After the stock reset (and its settle steps) the
embodiment moves each object the lot places by (lot position - declared position), keeping any seeded jitter, sets
its yaw, zeroes its velocity and settles again. ``demo()`` runs in the policy's ``reset()``, after
``prepare_trial``, so it sees the same lot as the scorer. The environment variable ``TASK_LOT=B`` pins every trial
to one lot (to record a lot's demo, or debug it). Offline (near_misses.py, check.py), :func:`use` picks the lot.

Importing this file is pure Python; nothing in yam_arena is modified on disk (the embodiment class is patched in
the task's own process, as parts.py does).
"""

from __future__ import annotations

import importlib.abc
import math
import os
import sys
from collections.abc import Sequence
from dataclasses import dataclass, field
from typing import Any

SETTLE_S = 0.5  # as the embodiment's own settle after a reset
RENDERS = 4


@dataclass(frozen=True)
class Lot:
    name: str
    place: dict[str, tuple[float, float, float]] = field(default_factory=dict)
    yaw: dict[str, float] = field(default_factory=dict)
    joints: dict[str, dict[str, float]] = field(default_factory=dict)  # articulated part -> joint -> start position
    roles: dict[str, Any] = field(default_factory=dict)

    def __init__(self, name: str, *, place=None, yaw=None, joints=None, **roles):
        object.__setattr__(self, "name", str(name))
        object.__setattr__(self, "place", {k: tuple(float(v) for v in p) for k, p in (place or {}).items()})
        object.__setattr__(self, "yaw", {k: float(v) for k, v in (yaw or {}).items()})
        object.__setattr__(self, "joints", {k: {j: float(v) for j, v in q.items()} for k, q in (joints or {}).items()})
        object.__setattr__(self, "roles", dict(roles))

    def __getattr__(self, key):
        roles = object.__getattribute__(self, "roles")
        if key in roles:
            return roles[key]
        raise AttributeError(f"lot {object.__getattribute__(self, 'name')!r} has no role {key!r}")

    def pos(self, name: str) -> tuple[float, float, float]:
        """Where this lot starts object ``name`` (its declared pos unless the lot places it)."""
        if name in self.place:
            return self.place[name]
        return tuple(_DECLARED[name].pos)

    def yaw_of(self, name: str) -> float:
        return self.yaw.get(name, float(getattr(_DECLARED[name], "yaw", 0.0)))


_LOTS: list[Lot] = []
AFTER_MOVES: list = []  # fn(embodiment): run after a lot's moves, before its settle (a task's own start-up steps)
_DECLARED: dict[str, Any] = {}  # object name -> spec, from OBJECTS
_trial = [-1]  # trials started in this process (prepare_trial calls) minus one
_pinned: list[int | None] = [None]  # use() offline, or TASK_LOT


def declare(objects: Sequence[Any], *lots: Lot) -> list[Lot]:
    """Declare the task's lots, in order (the first is the layout ``objects`` already gives). Returns them."""
    if not lots:
        raise ValueError("declare at least one Lot")
    names = [lot.name for lot in lots]
    if len(set(names)) != len(names):
        raise ValueError(f"lot names must be unique, got {names}")
    _DECLARED.clear()
    _DECLARED.update({o.name: o for o in objects})
    for lot in lots:
        for name in (*lot.place, *lot.yaw, *lot.joints):
            if name not in _DECLARED:
                raise KeyError(f"lot {lot.name}: no object {name!r} in OBJECTS")
            if getattr(_DECLARED[name], "mass", None) is None:
                raise ValueError(f"lot {lot.name}: {name} is fixed (no mass); only free objects can move per lot")
    _LOTS[:] = lots
    env = os.environ.get("TASK_LOT")
    _pinned[0] = index(env) if env else None
    return list(_LOTS)


def index(name: str) -> int:
    for i, lot in enumerate(_LOTS):
        if lot.name.upper() == str(name).upper():
            return i
    raise KeyError(f"no lot {name!r}; lots are {[lot.name for lot in _LOTS]}")


def use(lot: int | str | None) -> None:
    """Pin every trial to one lot (an index or a name); None goes back to cycling. For offline checks."""
    _pinned[0] = None if lot is None else (lot if isinstance(lot, int) else index(lot)) % len(_LOTS)


def trial() -> int:
    """The index of the trial being run (0 before the first one starts)."""
    return max(_trial[0], 0)


def current() -> Lot:
    """The lot of the trial being run: trial k uses lot k mod N, unless pinned."""
    if not _LOTS:
        raise RuntimeError("lots.declare(OBJECTS, ...) has not been called")
    i = _pinned[0] if _pinned[0] is not None else trial() % len(_LOTS)
    return _LOTS[i]


def all_lots() -> list[Lot]:
    return list(_LOTS)


# -- runtime: patch the yam_arena embodiment (only touches Isaac inside the simulator) ----------------------------


def write_joints(emb, lot=None) -> None:
    """Put each articulated part the lot names back at its declared pose, at rest, with its joints where the lot
    says. A task's AFTER_MOVES step may call it again (e.g. once its own locks are released)."""
    lot = lot or current()
    torch = emb._torch
    base = emb._base
    for name, q in lot.joints.items():  # an articulated part's joints start where this lot puts them, at rest
        obj = base.scene[name]
        spec = _DECLARED[name]  # and its body goes back to its declared start pose, at rest
        pose = obj.data.root_link_pose_w
        pose = (pose.torch if hasattr(pose, "torch") else pose).clone()
        origin = base.scene.env_origins
        origin = origin.torch if hasattr(origin, "torch") else origin
        rot = getattr(spec, "quat", None) or (0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))
        pose[:, :3] = torch.tensor(spec.pos, dtype=pose.dtype, device=pose.device) + origin[:, :3]
        pose[:, 3:] = torch.tensor(rot, dtype=pose.dtype, device=pose.device)
        obj.write_root_pose_to_sim_index(root_pose=pose)
        vel = obj.data.default_root_vel
        vel = vel.torch if hasattr(vel, "torch") else vel
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(vel))
        ids = obj.find_joints(list(q), preserve_order=True)[0]
        pos = torch.tensor([[q[j] for j in q]], dtype=torch.float32, device=base.device)
        obj.write_joint_position_to_sim_index(position=pos, joint_ids=ids)
        obj.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(pos), joint_ids=ids)


def _apply(emb) -> bool:
    """Move the current lot's objects in the running env; True if any moved."""
    lot = current()
    names = sorted(set(lot.place) | set(lot.yaw))
    if not names and not lot.joints:
        return False
    torch = emb._torch
    base = emb._base
    write_joints(emb, lot)
    for name in names:
        spec = _DECLARED[name]
        obj = base.scene[name]
        pose = obj.data.root_link_pose_w
        pose = (pose.torch if hasattr(pose, "torch") else pose).clone()
        if name in lot.place:
            delta = [a - b for a, b in zip(lot.place[name], spec.pos)]
            pose[:, :3] += torch.tensor(delta, dtype=pose.dtype, device=pose.device)
        if name in lot.yaw:
            y = lot.yaw[name]
            quat = [0.0, 0.0, math.sin(y / 2), math.cos(y / 2)]  # xyzw, as the stack's poses
            pose[:, 3:] = torch.tensor(quat, dtype=pose.dtype, device=pose.device)
        obj.write_root_pose_to_sim_index(root_pose=pose)
        vel = obj.data.default_root_vel
        vel = vel.torch if hasattr(vel, "torch") else vel
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(vel))
    for fn in AFTER_MOVES:
        fn(emb)
    for _ in range(max(1, round(SETTLE_S / base.step_dt))):
        emb._env.step(emb._ready)
    for _ in range(RENDERS):
        base.sim.render()
    return True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_lots_patched", False):
        return
    reset = cls.reset
    from inspect_robots.embodiment import Readiness

    def prepare_trial(self, scene):
        _trial[0] += 1
        if _LOTS:
            print(f"[lots] trial {trial()}: lot {current().name}", file=sys.stderr, flush=True)
        return Readiness(ready=True)

    def patched_reset(self, scene, *, seed=None):
        obs = reset(self, scene, seed=seed)
        if _LOTS and self.task is not None and _apply(self):
            obs = self._observation()
        return obs

    cls.prepare_trial, cls.reset = prepare_trial, patched_reset
    cls._lots_patched = True


_TARGET = "yam_arena.inspect_embodiment"


class _Loader(importlib.abc.Loader):
    def __init__(self, loader):
        self.loader = loader

    def create_module(self, spec):
        return self.loader.create_module(spec)

    def exec_module(self, module):
        self.loader.exec_module(module)
        _patch_embodiment(module)


class _Finder(importlib.abc.MetaPathFinder):
    lots_kit = True

    def find_spec(self, fullname, path, target=None):
        if fullname != _TARGET:
            return None
        for finder in sys.meta_path:
            if finder is self or not hasattr(finder, "find_spec"):
                continue
            spec = finder.find_spec(fullname, path, target)
            if spec is not None and spec.loader is not None:
                spec.loader = _Loader(spec.loader)
                return spec
        return None


def _install() -> None:
    if _TARGET in sys.modules:
        _patch_embodiment(sys.modules[_TARGET])
    elif not any(getattr(f, "lots_kit", False) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


_install()
