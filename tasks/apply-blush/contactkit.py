"""Contact force between each brush and the bodies it presses on (the head, its pan), from PhysX contact reports.

Isaac Lab turns PhysX contact processing off unless a ContactSensor exists; importing this module switches it back
on when the physics is configured (as in touch.py). The brushes are spawned with contact reports on
(``UsdBody(report=True)``). ``pressure(environment)`` returns, after the last physics step, the total normal force
(N) each brush presses on the head and on its own pan.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import sys

import numpy as np

_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"
_VIEW: dict = {"view": None}
SIDES = ("right", "left")


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
    brushes = [_body_path(environment, f"brush_{s}") for s in SIDES]
    targets = {"bust": [_body_path(environment, "bust")], **{f"pan_{s}": [_body_path(environment, f"pan_{s}")] for s in SIDES}}
    names = list(targets)
    filters = [targets[n][0] for n in names]
    view = PhysxManager.get_physics_sim_view().create_rigid_contact_view(
        brushes, filter_patterns=[filters] * len(brushes), max_contact_data_count=256 * len(brushes) * len(filters))
    _VIEW.update(view=view, names=names)
    print(f"[contactkit] {brushes} x {filters}", flush=True)


def pressure(environment) -> dict:
    """{(side, target): total normal force (N) that side's brush pressed on the target in the last physics step};
    the targets are "bust" and "pan"."""
    if _VIEW["view"] is None:
        _make_view(environment)
    names = _VIEW["names"]
    force, _, _, _, counts, starts = _VIEW["view"].get_contact_data(dt=environment.unwrapped.physics_dt)
    force = np.abs(_np(force).reshape(-1))
    counts = _np(counts).reshape(len(SIDES), len(names))
    starts = _np(starts).reshape(len(SIDES), len(names))
    out = {}
    for i, side in enumerate(SIDES):
        for j, n in enumerate(names):
            f = float(force[int(starts[i, j]):int(starts[i, j] + counts[i, j])].sum())
            if n == "bust":
                out[(side, "bust")] = f
            elif n == f"pan_{side}":
                out[(side, "pan")] = f
    return out
