"""contacts: which robot links touch a task-file box, read from PhysX after every physics substep.

``yam_arena`` builds no contact sensors, so the scorer cannot see a gripper's contacts. This module reads them from a
PhysX rigid-contact view (the mechanism behind Isaac Lab's ``ContactSensor``): its sensors are every robot link that
reports contacts (the YAM USD has contact reporting on every link) and its filter is the one box named in
:func:`watch`. A contact point counts as touching when its separation is ``TOUCH_SEP`` (1 mm) or less. The view is
read after every physics substep (240 Hz), so a quick touch between two control steps is not missed.

:func:`take` returns, for the control step that just ended, the robot links that touched the box in any substep, and
clears the record. A hand that pinches the box touches it with both finger pads: two links of one arm.

Pure Python at import time: the view is built lazily, inside Isaac Sim, when the first physics step runs.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import sys

__all__ = ["TOUCH_SEP", "arm_of", "install", "take", "watch"]

TOUCH_SEP = 0.001  # m: a contact point this close (or closer) is a touch
_ROBOT_PRIMS = {"/LeftArm/": "left", "/RightArm/": "right"}
_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"
_MON: dict = {"box": None, "view": None, "sim_view": None, "sensors": [], "links": set(), "err": None}


def watch(box: str) -> None:
    """Track the contacts between the robot's links and the task-file box ``box``."""
    _MON["box"] = box


def arm_of(link: str) -> str:
    """"left" or "right": the arm a link's prim path belongs to ("" if neither)."""
    return next((arm for part, arm in _ROBOT_PRIMS.items() if part in link), "")


def take() -> set[str]:
    """The robot links (prim paths) that touched the box in any physics substep since the last call, and forget them."""
    links = set(_MON["links"])
    _MON["links"].clear()
    return links


def _enable_contact_processing() -> None:
    """Isaac Lab turns PhysX contact processing off unless a ContactSensor exists (this embodiment builds none):
    switch it back on, as Isaac Lab's own ContactSensor does."""
    try:
        import carb

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
    except Exception:  # pragma: no cover  (outside Isaac Sim)
        pass


def _np(x):
    import numpy as np

    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x)


def _make_view(mgr) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    sim_view = mgr.get_physics_sim_view()
    if sim_view is None:
        return
    sensors, filters = [], []
    for prim in get_current_stage().Traverse():
        path = prim.GetPath().pathString
        if "/env_0/" not in path:
            continue
        if arm_of(path) and "PhysxContactReportAPI" in prim.GetAppliedSchemas():
            sensors.append(path)
        elif path.endswith("/" + str(_MON["box"])) and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            filters.append(path)
    if not sensors or not filters:
        raise RuntimeError(f"contact monitor: {len(sensors)} robot links, {len(filters)} boxes named {_MON['box']}")
    _MON["view"] = sim_view.create_rigid_contact_view(
        sensors, filter_patterns=[filters] * len(sensors), max_contact_data_count=64 * len(sensors) * len(filters))
    _MON["sim_view"], _MON["sensors"] = sim_view, sensors
    print(f"[contacts] monitor: {len(sensors)} robot links x {_MON['box']}", flush=True)


def _poll(mgr) -> None:
    """Called after every physics substep."""
    if _MON["box"] is None:
        return
    try:
        sim_view = mgr.get_physics_sim_view()
        if _MON["view"] is None or sim_view is not _MON["sim_view"]:  # the first step, or the environment was rebuilt
            _make_view(mgr)
        view = _MON["view"]
        if view is None:
            return
        _, _, _, sep, counts, starts = view.get_contact_data(dt=mgr.get_physics_dt())
    except Exception as e:  # pragma: no cover
        if _MON["err"] is None:
            _MON["err"] = repr(e)
            print(f"[contacts] monitor error: {e!r}", flush=True)
        return
    counts, starts, sep = _np(counts).reshape(-1), _np(starts).reshape(-1), _np(sep).reshape(-1)
    for i in (counts > 0).nonzero()[0]:  # one filter, so one row per sensor
        if float(sep[starts[i]:starts[i] + counts[i]].min()) <= TOUCH_SEP:
            _MON["links"].add(_MON["sensors"][i])


def _patch_manager(module) -> None:
    mgr = module.PhysxManager
    if getattr(mgr.step, "_contacts_hooked", False):
        return
    base_cfg = mgr._configure_physics.__func__
    base_step = mgr.step.__func__

    def _configure_physics(cls) -> None:
        base_cfg(cls)
        _enable_contact_processing()

    def step(cls) -> None:
        base_step(cls)
        _poll(cls)

    step._contacts_hooked = True
    mgr._configure_physics = classmethod(_configure_physics)
    mgr.step = classmethod(step)


class _Finder(importlib.abc.MetaPathFinder):
    """Patches Isaac Lab's PhysxManager when it is first imported (it needs Kit, so not when the task is loaded)."""

    def find_spec(self, fullname, path, target=None):
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

        def patched(module):
            exec_module(module)
            _patch_manager(module)

        spec.loader.exec_module = patched
        return spec


def install() -> None:
    """Hook the physics step now if Isaac Lab's manager is already imported, and on import otherwise. Idempotent."""
    if _PHYSX_MGR in sys.modules:
        _patch_manager(sys.modules[_PHYSX_MGR])
    elif not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
