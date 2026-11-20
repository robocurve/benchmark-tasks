"""contact: whether the arms touch the bag or the board, read from PhysX after every physics substep.

The scorer sees only the objects, not the grippers, so "the bag is released" cannot be read from their poses. This
module keeps a PhysX rigid-contact view (the mechanism behind Isaac Lab's ContactSensor) between every robot link
(the YAM USD has contact reporting on all of them) and the two free bodies, the bag and the board. After EVERY
physics substep (240 Hz) it reads the contact points of the view, so a fast motion cannot skip a touch, and records
the last substep in which a point had a separation of ``TOUCH_SEP`` or less: real contact, not "near".

:func:`released` is True when the view is working and no robot link has touched the bag or the board for the last
``RELEASE_S`` seconds. It fails closed: if the view cannot be built or read, it is False, so a task whose contacts
cannot be read never earns the full score.

Importing this module hooks Isaac Lab's ``PhysxManager`` when that module is imported (it needs Kit, so not at task
import) and clears the record on every reset (``bonds.on_reset``). Pure Python at import time.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import sys

import numpy as np
from bonds import on_reset

__all__ = ["TOUCH_SEP", "RELEASE_S", "ready", "released", "last_touch_s"]

TOUCH_SEP = 0.001  # m: a contact point this close (or closer, pressing in) is a touch
RELEASE_S = 0.1  # s: released means no touch for this long (two control steps at 20 Hz)
HELD = ("bag", "board")  # the bodies a gripper must let go of
ROBOT_PRIMS = ("/LeftArm/", "/RightArm/")

_MON: dict = {"view": None, "stage": None, "n": 0, "last": None, "dt": 1 / 240, "polls": 0, "err": None}


def _reset(env=None) -> None:
    _MON.update(view=None, stage=None, n=0, last=None, polls=0, err=None)


on_reset(_reset)


def ready() -> bool:
    """The contact view exists and has been read, with no error."""
    return bool(_MON["view"] is not None and _MON["polls"] > 0 and _MON["err"] is None)


def last_touch_s() -> float | None:
    """Seconds since a robot link last touched the bag or the board (None if it has not since the reset)."""
    return None if _MON["last"] is None else (_MON["n"] - _MON["last"]) * _MON["dt"]


def released() -> bool:
    """No robot link has touched the bag or the board for ``RELEASE_S`` s, and the contacts can be read."""
    if not ready() or _MON["n"] * _MON["dt"] < RELEASE_S:
        return False
    since = last_touch_s()
    return since is None or since >= RELEASE_S


def _np(x) -> np.ndarray:
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x)


def _enable_contact_processing() -> None:
    """Isaac Lab turns PhysX contact processing off unless a ContactSensor exists (this embodiment builds none)."""
    try:
        import carb

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
    except Exception:  # pragma: no cover  (outside Isaac Sim)
        pass


def _make_view(mgr) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    sim_view = mgr.get_physics_sim_view()
    if sim_view is None:
        return
    stage = get_current_stage()
    sensors, filters = [], []
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if "/env_0/" not in path:
            continue
        if any(r in path for r in ROBOT_PRIMS) and "PhysxContactReportAPI" in prim.GetAppliedSchemas():
            sensors.append(path)
        elif path.rsplit("/", 1)[-1] in HELD and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            filters.append(path)
    if not sensors or len(filters) != len(HELD):
        raise RuntimeError(f"contact monitor: {len(sensors)} robot links, {len(filters)} of {len(HELD)} bodies")
    _MON["view"] = sim_view.create_rigid_contact_view(
        sensors, filter_patterns=[filters] * len(sensors), max_contact_data_count=64 * len(sensors) * len(filters))
    _MON["stage"] = stage
    print(f"[contact] monitor: {len(sensors)} robot links x {len(filters)} bodies", flush=True)


def _poll(mgr) -> None:
    """Called after every physics substep."""
    if _MON["err"] is not None:
        return
    try:
        if _MON["view"] is None:
            _make_view(mgr)
        view = _MON["view"]
        if view is None:
            return
        _MON["dt"] = float(mgr.get_physics_dt())
        _, _, _, sep, counts, starts = view.get_contact_data(dt=_MON["dt"])
    except Exception as e:  # pragma: no cover  (needs Isaac Sim)
        _MON["err"] = repr(e)
        print(f"[contact] monitor error, the bag counts as not released: {e!r}", flush=True)
        return
    _MON["n"] += 1
    _MON["polls"] += 1
    counts = _np(counts).reshape(-1)
    if not counts.any():
        return
    starts, sep = _np(starts).reshape(-1), _np(sep).reshape(-1)
    idx = np.concatenate([np.arange(s, s + c) for s, c in zip(starts[counts > 0], counts[counts > 0])])
    if float(sep[idx].min()) <= TOUCH_SEP:
        _MON["last"] = _MON["n"]


_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"


def _patch_manager(mod) -> None:  # pragma: no cover  (needs Isaac Sim)
    mgr = mod.PhysxManager
    if getattr(mgr.step, "_contact_hooked", False):
        return
    base_cfg = mgr._configure_physics.__func__
    base_step = mgr.step.__func__

    def _configure_physics(cls) -> None:
        base_cfg(cls)
        _enable_contact_processing()

    def step(cls) -> None:
        base_step(cls)
        _poll(cls)

    step._contact_hooked = True
    mgr._configure_physics = classmethod(_configure_physics)
    mgr.step = classmethod(step)


class _MgrFinder(importlib.abc.MetaPathFinder):
    """Patches Isaac Lab's PhysxManager when it is first imported."""

    def find_spec(self, fullname, path, target=None):  # pragma: no cover  (needs Isaac Sim)
        if fullname != _PHYSX_MGR:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        exec_module = spec.loader.exec_module

        def exec_and_patch(module):
            exec_module(module)
            _patch_manager(module)

        spec.loader.exec_module = exec_and_patch
        return spec


def install() -> None:
    """Hook PhysxManager now if it is already imported, and on import otherwise. Idempotent."""
    if _PHYSX_MGR in sys.modules:  # pragma: no cover
        _patch_manager(sys.modules[_PHYSX_MGR])
    elif not any(isinstance(f, _MgrFinder) for f in sys.meta_path):
        sys.meta_path.insert(0, _MgrFinder())


install()
