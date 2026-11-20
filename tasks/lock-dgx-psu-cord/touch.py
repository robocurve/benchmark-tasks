"""Which of the task's bodies a robot link touches, from PhysX contacts (for "released": no gripper touches the handle).
A name is a scene object, or "object/link" for one link of an articulation.

Isaac Lab turns PhysX contact processing off unless a ContactSensor exists (this embodiment builds none); importing
this module switches it back on when the physics is configured, as Isaac Lab's own ContactSensor does. The robot's
links already report contacts. `touching(environment, names)` reads, after the last physics step, whether any robot
link is within TOUCH_SEP of each named body.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import sys

import numpy as np

TOUCH_SEP = 0.001  # m: a contact this close or closer is a touch
ROBOT_PRIMS = ("/LeftArm/", "/RightArm/")
_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"
_VIEW: dict = {"view": None, "names": None}


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
    """Patches Isaac Lab's PhysxManager when it is first imported (it needs Kit, so not at task import)."""

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
    _VIEW.update(view=None, names=None)  # rebuilt on the next read (the scene may have been rebuilt)


def _make_view(environment, names) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from isaaclab_physx.physics import PhysxManager

    _enable_contact_processing()
    scene = environment.unwrapped.scene
    stage = get_current_stage()
    bodies = []
    for n in names:
        n, _, link = n.partition("/")  # "panel/handle_n2": that link of the articulation "panel"
        path = scene[n].root_view.prim_paths[0]
        prim = stage.GetPrimAtPath(path)
        if link:
            path = next(p.GetPath().pathString for p in stage.Traverse()
                        if p.GetPath().pathString.startswith(path.rsplit("/", 1)[0] + "/") and p.GetName() == link
                        and "PhysicsRigidBodyAPI" in p.GetAppliedSchemas())
        elif "PhysicsRigidBodyAPI" not in prim.GetAppliedSchemas():  # an articulation's root prim: use its main link
            path = next(c.GetPath().pathString for c in prim.GetChildren()
                        if "PhysicsRigidBodyAPI" in c.GetAppliedSchemas())
        bodies.append(path)
    env = bodies[0].rsplit("/", 1)[0] + "/"
    if not bodies[0].rsplit("/", 2)[-2].startswith("env"):
        env = bodies[0].rsplit("/", 2)[0] + "/"
    sensors = [p.GetPath().pathString for p in stage.Traverse()
               if p.GetPath().pathString.startswith(env) and any(r in p.GetPath().pathString for r in ROBOT_PRIMS)
               and "PhysxContactReportAPI" in p.GetAppliedSchemas()]
    if not sensors:
        raise RuntimeError("touch: no robot links report contacts")
    view = PhysxManager.get_physics_sim_view().create_rigid_contact_view(
        sensors, filter_patterns=[bodies] * len(sensors), max_contact_data_count=256 * len(sensors) * len(bodies))
    _VIEW.update(view=view, names=list(names), sensors=sensors)
    print(f"[touch] {len(sensors)} robot links x {', '.join(names)}", flush=True)


def touching(environment, names) -> dict:
    """{name: whether any robot link touched that body in the last physics step}."""
    if _VIEW["view"] is None or _VIEW["names"] != list(names):
        _make_view(environment, names)
    _, _, _, sep, counts, starts = _VIEW["view"].get_contact_data(dt=environment.unwrapped.physics_dt)
    sep = _np(sep).reshape(-1)
    counts = _np(counts).reshape(len(_VIEW["sensors"]), len(names))
    starts = _np(starts).reshape(len(_VIEW["sensors"]), len(names))
    out = {}
    for j, n in enumerate(names):
        idx = [k for i in range(counts.shape[0]) for k in range(int(starts[i, j]), int(starts[i, j] + counts[i, j]))]
        out[n] = bool(idx) and float(sep[idx].min()) <= TOUCH_SEP
    return out
