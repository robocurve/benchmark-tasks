"""Scripted demos for yam_pair task files (CONTRACT.md §5). NumPy only (plus yam_arena's IK).

A task's ``demo(layout)`` returns ``{"a": X, "b": Y}``; each station's X is

1. ``{"left": [Waypoint, ...], "right": [...]}``: an open-loop plan in that station's frame (yam_arena's ArmPlan);
2. an object with ``act(view) -> 14-D`` (optional ``finished`` attribute): a closed-loop controller;
3. a generator function ``script(ctx)``: demokit style, ``ctx.left``/``ctx.right`` are :class:`Arm` (station frame);
   it yields once per control step.

All three are wrapped here as a :class:`StationController` with ``act(view) -> 14-D`` and ``finished``.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES

from yam_pair.layout import SIDES, Layout

CONTROL_HZ = 20.0
HOLD = np.array([*READY, 1.0, *READY, 1.0])


@dataclass
class StationView:
    """What a scripted station controller is given each control step."""

    station: str
    t: float  # seconds since the reset, at the end of this control step
    state: np.ndarray  # this station's measured 14-D state
    layout: Layout
    _objects: Callable[[], dict] | None = None

    @property
    def agent(self) -> str:  # the prototype's name
        return self.station

    @property
    def time(self) -> float:
        return self.t

    def objects(self) -> dict:
        """Privileged world-frame object states (scripted demos only)."""
        if self._objects is None:
            raise RuntimeError("no live embodiment: object states are not available")
        return self._objects()


# -- demokit-style Cartesian arm, in a station's frame ------------------------------------------------------------------
class Arm:
    """One arm commanded in Cartesian space in its station's frame (IK on the URDF); each move starts where the last
    command left it. ``pos``/``rot`` are the tool point (between the finger pads, 13 mm short of their tips)."""

    SEEDS = ([0.0, 1.0, 1.0, -0.6, 0.0, 0.0], [0.0, 0.8, 0.6, -0.6, 0.0, 1.5], [0.0, 0.8, 0.6, -0.6, 0.0, -1.5],
             [0.0, 1.2, 0.9, -0.3, 0.0, 2.0], [0.0, 1.2, 0.9, -0.3, 0.0, -2.0])

    def __init__(self, side: str, hz: float = CONTROL_HZ):
        self.side, self.base, self.hz = side, np.asarray(BASES[side], float), hz
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.home = (self.pos.copy(), self.rot.copy())
        self.worst_mm = 0.0

    def aim(self, pos, rot) -> None:
        q, err, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), q_init=self.q, base_pos=self.base, iters=80)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), np.asarray(rot, float)
        self.worst_mm = max(self.worst_mm, err * 1000)

    def move(self, pos, rot=None, seconds: float = 1.0, grip: float | None = None):
        rot = self.rot if rot is None else rot
        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(seconds * self.hz))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self.aim(p0 + s * (np.asarray(pos, float) - p0), ik.slerp_matrix(r0, rot, s))
            yield

    def set_grip(self, grip: float, seconds: float = 0.8):
        yield from self.move(self.pos, self.rot, seconds, grip)

    def solve_far(self, pos, rot) -> np.ndarray:
        quat = ik.matrix_to_quat_xyzw(rot)
        best = None
        for q0 in [self.q] + [np.array(q, float) for q in self.SEEDS]:
            q, err, rerr = ik.solve(pos, quat, q_init=q0, base_pos=self.base, iters=250)
            exact = err < 0.001 and rerr < math.radians(0.5)
            key = (not exact, float(np.abs(q - self.q).sum()) if exact else err)
            if best is None or key < best[0]:
                best = (key, q, err)
        self.worst_mm = max(self.worst_mm, best[2] * 1000)
        return best[1]

    def move_joints(self, pos, rot, seconds: float = 1.5, grip: float | None = None):
        q0, q1, g0 = self.q.copy(), self.solve_far(pos, rot), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(seconds * self.hz))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self.q = q0 + s * (q1 - q0)
            yield
        self.pos, self.rot = np.asarray(pos, float), np.asarray(rot, float)

    def go_home(self, seconds: float = 2.5):
        q0, q1, g0 = self.q.copy(), np.array(READY, float), self.grip
        n = max(1, round(seconds * self.hz))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (1.0 - g0)
            self.q = q0 + s * (q1 - q0)
            yield
        self.pos, self.rot = self.home[0].copy(), self.home[1].copy()

    def command(self) -> np.ndarray:
        return np.r_[self.q, self.grip]


def wait(seconds: float, hz: float = CONTROL_HZ):
    for _ in range(max(1, round(seconds * hz))):
        yield


def together(*gens):
    """Run generators side by side, one step each per control step (e.g. both arms at once)."""
    gens = [iter(g) for g in gens]
    while gens:
        for gen in list(gens):
            try:
                next(gen)
            except StopIteration:
                gens.remove(gen)
        if gens:
            yield


class Ctx:
    """What a generator script works with: its station's two arms, the measured scene, frame conversions."""

    def __init__(self, station: str, layout: Layout, hz: float):
        self.station, self.layout = station, layout
        self.left, self.right = Arm("left", hz), Arm("right", hz)
        self.flags: dict = {}
        self.view: StationView | None = None

    @property
    def t(self) -> float:
        return self.view.t if self.view else 0.0

    @property
    def state(self) -> np.ndarray:
        return self.view.state if self.view else HOLD.copy()

    def states(self) -> dict:
        return self.view.objects()

    def local(self, p) -> np.ndarray:
        return self.layout.to_local(self.station, p)

    def world(self, p) -> np.ndarray:
        return self.layout.to_world(self.station, p)

    def command(self) -> np.ndarray:
        return np.r_[self.left.command(), self.right.command()]


# -- wrappers ---------------------------------------------------------------------------------------------------------
class StationController:
    finished: bool = False

    def act(self, view: StationView) -> np.ndarray:  # pragma: no cover - interface
        raise NotImplementedError


class PlanController(StationController):
    """Open-loop waypoint plans per arm (station frame); an arm without one holds READY, gripper open."""

    def __init__(self, plans: dict[str, list]):
        unknown = sorted(set(plans) - set(SIDES))
        if unknown:
            raise ValueError(f"a station plan's keys are arms {SIDES}, got {unknown}")
        self.plans = {side: ArmPlan(list(w), BASES[side], finger_start=FINGER_OPEN) for side, w in plans.items()}
        self.duration = max((p.duration for p in self.plans.values()), default=0.0)

    def act(self, view: StationView) -> np.ndarray:
        per_arm = [self.plans[s].action7(view.t)[0] if s in self.plans else np.r_[READY, 1.0] for s in SIDES]
        self.finished = view.t > self.duration
        return np.concatenate(per_arm)


class ScriptController(StationController):
    def __init__(self, script: Callable, station: str, layout: Layout, hz: float):
        self.ctx = Ctx(station, layout, hz)
        self.script = script
        self.gen = None

    def act(self, view: StationView) -> np.ndarray:
        self.ctx.view = view
        if self.gen is None:
            self.gen = iter(self.script(self.ctx))
        if not self.finished:
            try:
                next(self.gen)
            except StopIteration:
                self.finished = True
        return self.ctx.command()


class ObjectController(StationController):
    def __init__(self, obj: Any):
        self.obj = obj

    @property
    def finished(self) -> bool:  # type: ignore[override]
        return bool(getattr(self.obj, "finished", False) or getattr(self.obj, "done", False))

    def act(self, view: StationView) -> np.ndarray:
        return np.asarray(self.obj.act(view), dtype=np.float64).reshape(-1)


def controller(value: Any, station: str, layout: Layout, hz: float = CONTROL_HZ) -> StationController:
    if value is None:
        return PlanController({})
    if isinstance(value, dict):
        return PlanController(value)
    if hasattr(value, "act"):
        return ObjectController(value)
    if callable(value):
        return ScriptController(value, station, layout, hz)
    raise TypeError(f"demo()[{station!r}] must be a plan dict, an object with act(view), or a generator function; "
                    f"got {value!r}")
