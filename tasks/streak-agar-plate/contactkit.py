"""Contact force between the loop and the agar, from PhysX contact reports.

Isaac Lab turns PhysX contact processing off unless a ContactSensor exists; importing this module switches it back
on when the physics is configured (as touch.py does). The loop is spawned with contact reports on
(``UsdBody(report=True)``). ``loop_on_agar(environment)`` returns, after the last physics step, the total normal
force (N) the loop presses on the agar.
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


def _make_view(environment) -> None:
    from isaaclab_physx.physics import PhysxManager

    _enable_contact_processing()
    from isaaclab.sim.utils.stage import get_current_stage

    agar = _body_path(environment, "agar")
    stage = get_current_stage()
    root = environment.unwrapped.scene["loop"].root_view.prim_paths[0]
    loop = next(p.GetPath().pathString for p in stage.Traverse()
                if p.GetPath().pathString.startswith(root.rsplit("/", 1)[0] + "/") and p.GetName() == "tip"
                and "PhysicsRigidBodyAPI" in p.GetAppliedSchemas())
    view = PhysxManager.get_physics_sim_view().create_rigid_contact_view(
        [loop], filter_patterns=[[agar]], max_contact_data_count=512)
    _VIEW.update(view=view)
    print(f"[contactkit] {loop} x {agar}", flush=True)


def loop_on_agar(environment) -> float:
    """The total normal force (N) the loop pressed on the agar in the last physics step."""
    if _VIEW["view"] is None:
        _make_view(environment)
    force, _, _, _, counts, starts = _VIEW["view"].get_contact_data(dt=environment.unwrapped.physics_dt)
    force = np.abs(_np(force).reshape(-1))
    n, s = int(_np(counts).reshape(-1)[0]), int(_np(starts).reshape(-1)[0])
    return float(force[s:s + n].sum())
