"""Simulator-side helpers for the stud-finder task (imported by the task file; nothing from the simulator is imported
until a reset runs on a simulator host).

1. Layout by seed: at every reset the trial's wall section is seed mod 4 (no seed: section 0). Only the hidden
   screws' positions change; no body moves, and every section looks the same.
2. The magnets' pull: before every physics substep, the force of each magnet on each screw head (magnet_law, from
   the finder's pose) is summed and applied to the finder at the magnets, as a force at its centre of mass plus the
   matching torque. Nothing else is written to the simulation.
3. Diagnostics: a STUD line every 20 control steps (every 5 while a magnet pulls), for GPU probes. The scorer never
   reads them.
"""
import functools
import math

import numpy as np

import magnet_law

_CFG = None
_LOG = {"token": None}


def _arr(x):
    x = getattr(x, "torch", x)
    return x.detach().cpu().numpy().astype(np.float64) if hasattr(x, "detach") else np.asarray(x, np.float64)


def _quat_matrix(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def finder_wrench(origin, R):
    """Total magnetic force on the finder (world, N), the torque about its centre of mass (N m), and per-magnet
    detail, for the finder's frame at `origin` with rotation `R` and this trial's screws."""
    axis = R[:, 0]
    com = origin + R @ np.asarray(_CFG["COM_F"], float)
    force, torque, detail = np.zeros(3), np.zeros(3), []
    for z in _CFG["MAGNET_Z"]:
        face = origin + R @ np.array([-_CFG["SKIN"], 0.0, z])
        best = None
        for head in _CFG["screw_heads"]():
            f = magnet_law.magnet_force(face, axis, head)
            force += f
            torque += np.cross(face - com, f)
            d = head - face
            gap = float(d @ axis)
            rho = float(np.linalg.norm(d - gap * axis))
            if best is None or rho < best[1]:
                best = (gap, rho, float(np.linalg.norm(f)))
        detail.append(best)
    return force, torque, detail


def _install_force(env):
    base = env.unwrapped
    try:
        obj = base.scene.rigid_objects["finder"]
    except (AttributeError, KeyError):
        obj = base.scene["finder"]
    if getattr(obj, "_stud_hooked", False):
        return
    import warp as wp
    orig = obj.write_data_to_sim
    origins = _arr(base.scene.env_origins)

    def write(*args, **kwargs):
        result = orig(*args, **kwargs)
        poses = _arr(obj.data.root_link_pose_w)
        n = len(poses)
        forces, torques = np.zeros((n, 3)), np.zeros((n, 3))
        rows = []
        for e in range(n):
            origin, R = poses[e, :3] - origins[e], _quat_matrix(poses[e, 3:7])
            forces[e], torques[e], detail = finder_wrench(origin, R)
            rows.append((origin, R, detail))
        if np.any(forces) or np.any(torques):
            dev = base.device
            view = getattr(obj, "root_view", None) or obj.root_physx_view
            view.apply_forces_and_torques_at_position(
                force_data=wp.array(forces.ravel().astype(np.float32), dtype=wp.float32, device=dev),
                torque_data=wp.array(torques.ravel().astype(np.float32), dtype=wp.float32, device=dev),
                position_data=None,
                indices=obj._ALL_INDICES,
                is_global=True,
            )
        _diagnostic(rows[0], forces[0])
        return result

    obj.write_data_to_sim = write
    obj._stud_hooked = True
    print(f"STUD_FORCE_HOOK installed on {type(obj).__name__}; com_F_mm={np.round(np.asarray(_CFG['COM_F']) * 1000, 2)}",
          flush=True)


def _diagnostic(row, force):
    run = _CFG["_RUN"]
    token = (run["episode"], run["step"])
    if token == _LOG["token"] or run["t"] is None:
        return
    pulling = float(np.linalg.norm(force)) > 0.05
    if run["step"] % (5 if pulling else 20):
        return
    _LOG["token"] = token
    origin, R, detail = row
    wall_x = _CFG["WALL_X"]
    gaps = [wall_x - float((origin + R @ np.asarray(p))[0]) for p in _CFG["BACK_POINTS"]]
    mags = " ".join(f"{name}(gap={g * 1000:.2f},rho={r * 1000:.2f},F={f:.2f})"
                    for name, (g, r, f) in zip(("upper", "lower"), detail))
    tilt = math.degrees(math.acos(float(np.clip(R[0, 0], -1.0, 1.0))))
    print(f"STUD t={run['t']:.2f} layout={_CFG['layout_index']()} origin_mm=({origin[0] * 1000:.1f},"
          f"{origin[1] * 1000:.1f},{(origin[2] - _CFG['TOP']) * 1000:.1f}) back_tilt_deg={tilt:.2f} "
          f"back_gaps_mm=[{','.join(f'{g * 1000:.2f}' for g in gaps)}] {mags} "
          f"F_N=({force[0]:.2f},{force[1]:.2f},{force[2]:.2f})", flush=True)


def install(module):
    global _CFG
    _CFG = module if isinstance(module, dict) else vars(module)
    try:
        from yam_arena.inspect_embodiment import YamArenaEmbodiment as cls
    except ImportError:
        return
    if getattr(cls, "_stud_wrapped", False):
        cls._stud_cfg[0] = _CFG
        return
    inner_reset = cls.reset
    holder = [_CFG]

    @functools.wraps(inner_reset)
    def reset(self, *args, **kwargs):
        cfg = holder[0]
        seed = kwargs.get("seed")
        cfg["ACTIVE_LAYOUT"] = cfg["layout_for_seed"](seed)
        result = inner_reset(self, *args, **kwargs)
        if self.task is not None and "finder" in self.task.names:
            _install_force(self._env)
            print(f"STUD_LAYOUT seed={seed} layout={cfg['layout_index']()} screws_mm="
                  f"{[tuple(round(float(v) * 1000, 1) for v in (h[1], h[2] - cfg['TOP'])) for h in cfg['screw_heads']()]}",
                  flush=True)
        return result

    cls.reset = reset
    cls._stud_wrapped = True
    cls._stud_cfg = holder
