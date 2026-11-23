"""Contact forces between the iron and the steel block (sensors) and each insert and the plate (targets), from PhysX
contact reports.

Isaac Lab turns PhysX contact processing off unless a ContactSensor exists; importing this module switches it back
on when the physics is configured (as touch.py does). The iron and block are spawned with contact reports on
(``UsdBody(report=True)``). ``forces(environment)`` returns, after the last physics step, {(tool, target): total normal
force (N)} for tool in (iron, block) and target in (insert_a, insert_b, plate).
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


SENSORS = ("iron", "block")
TARGETS = ("insert_a", "insert_b", "base")


def _make_view(environment) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from isaaclab_physx.physics import PhysxManager

    _enable_contact_processing()
    stage = get_current_stage()
    root = environment.unwrapped.scene["plate"].root_view.prim_paths[0]
    env = root.rsplit("/", 1)[0] + "/"

    def link(name):
        return next(p.GetPath().pathString for p in stage.Traverse() if p.GetPath().pathString.startswith(env)
                    and p.GetName() == name and "PhysicsRigidBodyAPI" in p.GetAppliedSchemas())

    sensors = [_body_path(environment, s) for s in SENSORS]
    targets = [link(t) for t in TARGETS]
    view = PhysxManager.get_physics_sim_view().create_rigid_contact_view(
        sensors, filter_patterns=[targets] * len(sensors), max_contact_data_count=256 * len(sensors) * len(targets))
    _VIEW.update(view=view)
    print(f"[contactkit] {sensors} x {targets}", flush=True)


def forces(environment) -> dict:
    """{(tool, target): total normal force (N) in the last physics step}; target "base" is the plate."""
    if _VIEW["view"] is None:
        _make_view(environment)
    force, _, _, _, counts, starts = _VIEW["view"].get_contact_data(dt=environment.unwrapped.physics_dt)
    force = np.abs(_np(force).reshape(-1))
    counts = _np(counts).reshape(len(SENSORS), len(TARGETS))
    starts = _np(starts).reshape(len(SENSORS), len(TARGETS))
    out = {}
    for i, s in enumerate(SENSORS):
        for j, t in enumerate(TARGETS):
            out[(s, "plate" if t == "base" else t)] = float(force[int(starts[i, j]):int(starts[i, j] + counts[i, j])].sum())
    return out
