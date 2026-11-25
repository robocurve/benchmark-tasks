"""Contact forces on the cuff, from PhysX contact reports (adapted from the shared contact helper).

Isaac Lab turns PhysX contact processing off unless a ContactSensor exists; importing this module switches it back on
when the physics is configured. The cuff's plates and hems are spawned with contact reports on (cuffkit).
``forces(environment)`` returns, for the last physics step, (1) the total normal force each hook body (the four hook
plates and the hook end's hem) presses on the cuff's other plates, and (2) the total normal force each of the robot's
four finger pads presses on each cuff body.
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



_V: dict = {"hook": None}
NSEG = 20
HOOK_SEGS = (16, 17, 18, 19)
RECEIVERS = tuple(range(0, 15))


def reset() -> None:
    _V.update(hook=None)


def _paths(environment):
    """The cuff's plate and hem prims and the four finger pads (env 0), found on the stage by name."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    stage = get_current_stage()
    ns = "/World/envs/env_0/"
    segs, plugs, pads = {}, {}, []
    for pr in stage.Traverse():
        path = pr.GetPath().pathString
        if not path.startswith(ns) or not pr.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        name, parent = pr.GetName(), pr.GetParent().GetName()
        if parent == "cuff" and name.startswith("seg_"):
            segs[int(name[4:])] = path
        elif parent == "cuff" and name in ("start_plug", "end_plug"):
            plugs[name] = path
        elif name in ("tip_left", "tip_right"):
            pads.append(path)
    if len(pads) != 4:  # robots are instanced: walk each robot's root prim with instance proxies (as the umbrella task does)
        from pxr import Usd
        pads = []
        scene = environment.unwrapped.scene
        for arm in ("left_robot", "right_robot"):
            root = scene[arm].root_physx_view.prim_paths[0]
            top = root.split("/")[:5]  # /World/envs/env_0/<robot>
            prim = stage.GetPrimAtPath("/".join(top))
            rng = Usd.PrimRange(prim, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate))
            found = sorted(str(q.GetPath()) for q in rng
                           if q.HasAPI(UsdPhysics.RigidBodyAPI) and q.GetName() in ("tip_left", "tip_right"))
            print(f"[presskit] {arm} root {root} pads {found}", flush=True)
            pads += found
    if len(segs) != NSEG or len(plugs) != 2 or len(pads) != 4:
        raise RuntimeError(f"presskit: found {len(segs)} plates, {list(plugs)}, pads {pads}")
    return [segs[i] for i in range(NSEG)], [plugs["start_plug"], plugs["end_plug"]], sorted(pads)


def _make(environment) -> None:
    from isaaclab_physx.physics import PhysxManager

    _enable_contact_processing()
    segs, plugs, pads = _paths(environment)
    sim = PhysxManager.get_physics_sim_view()
    hooks = [segs[i] for i in HOOK_SEGS] + [plugs[1]]
    receivers = [segs[i] for i in RECEIVERS]
    hook = sim.create_rigid_contact_view(hooks, filter_patterns=[receivers] * len(hooks),
                                         max_contact_data_count=64 * len(hooks) * len(receivers))
    bodies = segs + plugs
    pad = sim.create_rigid_contact_view(bodies, filter_patterns=[pads] * len(bodies),
                                        max_contact_data_count=32 * len(bodies) * len(pads))
    _V.update(pad_paths=pads)
    _V.update(hook=hook, pad=pad, n_hook=len(hooks), n_rec=len(receivers), n_body=len(bodies), n_pad=len(pads))
    print(f"[presskit] hooks {hooks} x {len(receivers)} plates; {len(bodies)} cuff bodies x pads {pads}", flush=True)


def _sum(view, rows, cols, dt):
    force, _, _, _, counts, starts = view.get_contact_data(dt=dt)
    force = np.abs(_np(force).reshape(-1))
    counts = _np(counts).reshape(rows, cols)
    starts = _np(starts).reshape(rows, cols)
    out = np.zeros((rows, cols))
    for i in range(rows):
        for j in range(cols):
            c = int(counts[i, j])
            if c:
                s0 = int(starts[i, j])
                out[i, j] = float(force[s0:s0 + c].sum())
    return out


def _vec(view, rows, cols, dt, centers):
    """(rows, cols, 3): for each cuff body and finger pad, the force vector the pad puts on the body: each contact's
    force along its normal, the normal turned to point from the contact point into the body's centre (a pad can only
    push), so the reported normal's sign convention is not relied on. ``centers``: (rows, 3) body centres, world frame."""
    force, point, normal, _, counts, starts = view.get_contact_data(dt=dt)
    force = _np(force).reshape(-1)
    point = _np(point).reshape(-1, 3)
    normal = _np(normal).reshape(-1, 3)
    counts = _np(counts).reshape(rows, cols)
    starts = _np(starts).reshape(rows, cols)
    out = np.zeros((rows, cols, 3))
    for i in range(rows):
        for j in range(cols):
            c = int(counts[i, j])
            if c:
                s0 = int(starts[i, j])
                n = normal[s0:s0 + c]
                into = centers[i][None] - point[s0:s0 + c]
                sign = np.where((n * into).sum(axis=1) >= 0.0, 1.0, -1.0)
                out[i, j] = (force[s0:s0 + c, None] * n * sign[:, None]).sum(axis=0)
    return out


def pad_names() -> list:
    """[(side, link)] for the four finger pads, in the order of the "pads" and "pad_vec" columns."""
    return [("left" if ("/LeftArm/" in p or "left_robot" in p) else "right", p.rsplit("/", 1)[-1]) for p in _V["pad_paths"]]


def forces(environment, centers=None) -> dict:
    """{"hook_on_cuff": (5,) N, each hook body on the cuff's plates 0-14; "pads": (22, 4) N, each cuff body (plates 0-19,
    start hem, end hem) against each finger pad; "pad_vec": (22, 4, 3), the force vector each pad puts on each body (pointing into the body);
    ``centers``: (22, 3) the bodies' centres in the station frame (plates 0-19, start hem, end hem);
    "pad_names": [(side, link)] naming the four pad columns}, for the last physics step."""
    if _V["hook"] is None:
        _make(environment)
    dt = environment.unwrapped.physics_dt
    hook = _sum(_V["hook"], _V["n_hook"], _V["n_rec"], dt).sum(axis=1)
    pads = _sum(_V["pad"], _V["n_body"], _V["n_pad"], dt)
    origin = _np(environment.unwrapped.scene.env_origins[0]).astype(float)
    world = np.asarray(centers, float) + origin if centers is not None else np.zeros((_V["n_body"], 3))
    pad_vec = _vec(_V["pad"], _V["n_body"], _V["n_pad"], dt, world)
    return {"hook_on_cuff": hook, "pads": pads, "pad_vec": pad_vec, "pad_names": pad_names()}
