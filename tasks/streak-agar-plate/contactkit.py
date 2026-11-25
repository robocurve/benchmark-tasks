"""Contact force between the loop and the agar, from PhysX contact reports.

Isaac Lab turns PhysX contact processing off unless a ContactSensor exists; importing this module switches it back
on when the physics is configured (as touch.py does). The loop is spawned with contact reports on
(``UsdBody(report=True)``). ``loops_on_agar(environment, names)`` returns, after the last physics step, the total
normal force (N) each named loop's tip (neck and ring) presses on the agar.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import sys

import numpy as np

_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"
_VIEW: dict = {"view": None}


def _enable_contact_processing() -> None:
    try:
        import carb

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
    except Exception:  # pragma: no cover  (outside Isaac Sim)
        pass


def _patch_manager(mod) -> None:
    mgr = mod.PhysxManager
    if getattr(mgr._configure_physics, "_touch_hooked", False):
        return
    base_cfg = mgr._configure_physics.__func__

    def _configure_physics(cls) -> None:
        base_cfg(cls)
        _enable_contact_processing()

    _configure_physics._touch_hooked = True
    mgr._configure_physics = classmethod(_configure_physics)


class _MgrFinder(importlib.abc.MetaPathFinder):
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
        orig_exec = spec.loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _patch_manager(module)

        spec.loader.exec_module = exec_module
        return spec


if _PHYSX_MGR in sys.modules:
    _patch_manager(sys.modules[_PHYSX_MGR])
elif not any(isinstance(f, _MgrFinder) for f in sys.meta_path):
    sys.meta_path.insert(0, _MgrFinder())


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x)


def reset() -> None:
    _VIEW.update(view=None)


def _body_path(environment, name: str) -> str:
    from isaaclab.sim.utils.stage import get_current_stage

    path = environment.unwrapped.scene[name].root_view.prim_paths[0]
    prim = get_current_stage().GetPrimAtPath(path)
    if "PhysicsRigidBodyAPI" not in prim.GetAppliedSchemas():
        path = next(c.GetPath().pathString for c in prim.GetChildren()
                    if "PhysicsRigidBodyAPI" in c.GetAppliedSchemas())
    return path


def _tip_path(environment, name: str) -> str:
    """The rigid body named 'tip' (neck and ring) inside the loop asset ``name``."""
    from isaaclab.sim.utils.stage import get_current_stage

    root = environment.unwrapped.scene[name].root_view.prim_paths[0]
    base = root if root.rstrip("/").endswith("/" + name) else root.rsplit("/", 1)[0]
    for p in get_current_stage().Traverse():
        q = p.GetPath().pathString
        if q.startswith(base + "/") and p.GetName() == "tip" and "PhysicsRigidBodyAPI" in p.GetAppliedSchemas():
            return q
    raise RuntimeError(f"no tip body under {base}")


def _make_view(environment, names) -> None:
    from isaaclab_physx.physics import PhysxManager

    _enable_contact_processing()
    agar = _body_path(environment, "agar")
    tips = [_tip_path(environment, n) for n in names]
    assert len(set(tips)) == len(tips), tips
    view = PhysxManager.get_physics_sim_view().create_rigid_contact_view(
        tips, filter_patterns=[[agar]] * len(tips), max_contact_data_count=1024)
    _VIEW.update(view=view, names=list(names))
    print(f"[contactkit] {tips} x {agar}", flush=True)


def loops_on_agar(environment, names) -> list:
    """The total normal force (N) each loop in ``names`` pressed on the agar in the last physics step."""
    if _VIEW["view"] is None or _VIEW.get("names") != list(names):
        _make_view(environment, names)
    force, _, _, _, counts, starts = _VIEW["view"].get_contact_data(dt=environment.unwrapped.physics_dt)
    force = np.abs(_np(force).reshape(-1))
    counts, starts = _np(counts).reshape(-1), _np(starts).reshape(-1)
    return [float(force[int(starts[i]):int(starts[i] + counts[i])].sum()) for i in range(len(names))]
