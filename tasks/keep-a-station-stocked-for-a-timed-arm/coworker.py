"""coworker: drive the right YAM arm from a fixed, pre-planned routine, so the policy under test controls the left arm only.

The yam_arena embodiment takes a 14-D action ``[left j0..j5, left grip, right j0..j5, right grip]``. ``install(plan_fn)``
wraps ``YamArenaEmbodiment.step`` so that, whatever the policy sends, the right half is replaced by the routine's joint
targets at the embodiment's own clock (``self._t``, seconds since reset). The routine is a list of
``yam_arena.demo.Waypoint`` played through the stock ``ArmPlan`` (smoothstep TCP interpolation + IK), rebuilt at every
reset, so it moves the same way in every trial and reads nothing from the scene.

The hook uses the same technique as ``labkit`` (patch ``yam_arena.inspect_embodiment`` now if imported, otherwise when
it is) and never touches yam_arena on disk. Offline (no Isaac), nothing is patched: an offline checker plays the task's
``demo()``, whose right-arm plan is the same routine.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import sys
import types

import numpy as np

_STATE: dict = {"plan_fn": None, "plan": None, "t_last": None}
TARGET = "yam_arena.inspect_embodiment"


def right_action(t: float) -> np.ndarray:
    """The routine's 7-D right-arm target (6 joints + normalized grip) at time ``t`` (s since reset)."""
    from yam_arena import station
    from yam_arena.demo import ArmPlan

    if _STATE["plan"] is None or _STATE["t_last"] is None or t < _STATE["t_last"]:
        _STATE["plan"] = ArmPlan(list(_STATE["plan_fn"]()), station.BASES["right"])
    _STATE["t_last"] = t
    a, _label = _STATE["plan"].action7(t)
    return np.asarray(a, dtype=np.float64)


def _patch(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_coworker", False):
        return
    base_step, base_reset = cls.step, cls.reset

    def step(self, action):
        data = np.asarray(action.data, dtype=np.float64).reshape(-1).copy()
        data[7:14] = right_action(float(self._t))
        return base_step(self, types.SimpleNamespace(data=data))

    def reset(self, *a, **k):
        _STATE["plan"], _STATE["t_last"] = None, None
        return base_reset(self, *a, **k)

    cls.step, cls.reset = step, reset
    cls._coworker = True


class _Loader(importlib.abc.Loader):
    def __init__(self, inner):
        self.inner = inner

    def create_module(self, spec):
        return self.inner.create_module(spec)

    def exec_module(self, module):
        self.inner.exec_module(module)
        _patch(module)


class _Finder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != TARGET:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        spec.loader = _Loader(spec.loader)
        return spec


def install(plan_fn) -> None:
    """Drive the right arm with ``plan_fn()`` (a list of Waypoints) from now on. Idempotent."""
    _STATE["plan_fn"] = plan_fn
    if TARGET in sys.modules:
        _patch(sys.modules[TARGET])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())
