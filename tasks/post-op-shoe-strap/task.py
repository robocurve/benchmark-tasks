"""Fasten a post-op shoe on a practice leg: draw the instep strap back through its D-ring, snug, lay it back over and
smooth it closed.

On the station table sits a practice lower leg and foot (an adult man's, the foot in a padded dressing), hung from a
sprung, damped two-axis knee pivot on a stand at the table's far side, with its sole resting on the table and its
toes toward the robot. It wears an open-toe post-operative shoe (men's Large, outsole about 29 cm: rigid rocker sole,
navy padded upper, two hook-and-loop straps through D-rings), already on the foot. The forefoot strap is already
fastened. The instep strap (25 mm) is anchored on the inner side of the shoe, crosses the instep and passes through a
black plastic D-ring on the outer side (toward the left arm); it starts opened outward, its rubberised pull tab
standing about 2 cm proud of the shoe, and the outer wing of the upper splayed so that the opening's edges are 40 mm
apart.

The robot must draw the strap back through its D-ring until the shoe is snug, fold it back over and smooth its hook
patch down onto the strap's loop patch, and let go. Scoring: ``shoescore.py`` (the text is in task.json). Model:
``shoekit.py`` (the text is in task.json's fidelity).
"""

from __future__ import annotations

import json
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import robotpts  # noqa: E402
import scenekit  # noqa: E402
import shoekit  # noqa: E402
import shoescore  # noqa: E402
import steptime  # noqa: E402
from shoekit import Shoe  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
MODE = os.environ.get("TASK_DEMO", "success")  # the scripted demo's variant (shoedemo.py)

W0 = np.array((0.24, 0.0, TOP + 0.0005))  # the shoe frame's origin: the middle of the sole's underside
SHOE = Shoe(name="shoe", pos=tuple(W0), size=(0.30, 0.12, 0.10), color=(0.13, 0.17, 0.30))
OBJECTS = [SHOE]
SUCCESS_HOLD_S = 0.0  # the scorer holds its own 1 s
TAGS = "ab"


def _new_state() -> dict:
    return {"n": 0, "log": [], "fast": None, "scorer": shoescore.Scorer(n_straps=len(SHOE.straps))}


_S = _new_state()


# -- measurements ------------------------------------------------------------------------------------------------

def _shoe_surface_pts(st) -> np.ndarray:
    """Points standing for the shoe in the 3 cm hands-off check: every strap plate, the loop, and a grid on the
    shoe's outline (sole edge, upper, heel)."""
    R = shoekit.rotmat(st.leg_quat[None])[0]
    grid = []
    for x in np.linspace(-0.145, 0.145, 13):
        for y, z in ((-0.050, 0.015), (0.050, 0.015), (-0.045, 0.060), (0.045, 0.060), (0.0, 0.088)):
            grid.append((x, y, z))
    for y in np.linspace(-0.025, 0.025, 3):
        grid.append((0.146, y, 0.090))
    grid = np.array(grid) @ R.T + st.leg_pos
    links = np.vstack(st.links) if st.links else np.empty((0, 3))
    rings = np.array([c for c, _ in st.ring]) if st.ring else np.empty((0, 3))
    return np.vstack((grid, links, rings))


def ring_loc(st, k):
    """Strap k's plate centres in its loop's frame: (across the strap, through the loop, out of the flap)."""
    c, Rf = st.ring[k]
    loc = (st.links[k] - c) @ Rf
    return np.stack((loc[:, 0], loc[:, 1:] @ shoekit.FLAP_D, loc[:, 1:] @ shoekit.FLAP_N), axis=1)


def _frame(env, st, t) -> shoescore.Frame:
    s_ring, ov = [], []
    for k in range(len(SHOE.straps)):
        s_ring.append(shoescore.ring_crossing(ring_loc(st, k), SHOE.link_len, shoekit.RING_IN_W / 2 + shoekit.RING_BAR,
                                              shoekit.RING_IN_H / 2 + shoekit.RING_BAR))  # within the loop's frame
        ov.append(shoescore.overlap_m(st.pairs[k], SHOE.link_len))
    robot = robotpts.robot_points(env)
    shoe = _shoe_surface_pts(st)
    d = float(np.min(np.linalg.norm(robot[:, None, :] - shoe[None, :, :], axis=2))) if len(robot) else 1.0
    return shoescore.Frame(t=t, s_ring=tuple(s_ring), gap=st.gap, overlap=tuple(ov),
                           leg_off=float(np.linalg.norm(st.leg_pos - W0)), robot_dist=d)


# -- the scorer's tab pull (the spring scale) -------------------------------------------------------------------------

PULL_K, PULL_STROKE, PULL_SPEED = 3000.0, 0.012, 0.012  # the drive's cap holds 5 N until the tab has moved 10 mm


def _drive(stage, path, target_rel, k, cap, cap_z=None, damping=None):
    from pxr import UsdPhysics
    prim = stage.GetPrimAtPath(path)
    for i, ax in enumerate(("transX", "transY", "transZ")):
        d = UsdPhysics.DriveAPI(prim, ax)
        d.GetStiffnessAttr().Set(float(k))
        d.GetDampingAttr().Set(float(k * 0.01 if damping is None else damping))
        d.GetMaxForceAttr().Set(float(cap_z if (cap_z is not None and ax == "transZ") else cap))
        if target_rel is not None:
            d.GetTargetPositionAttr().Set(float(target_rel[i]))


def _sim_step(env):
    base = env.unwrapped
    base.scene.write_data_to_sim()
    base.sim.step(render=False)
    base.scene.update(base.sim.get_physics_dt())


def _pull_test(env) -> list:
    """For the strap (each strap in turn), the arms frozen: a 5 N spring-scale pull on the tab's end along the strap,
    away from the D-ring, for 1 s, then released. The pull is a capped spring drive whose target advances at 12 mm/s
    to 12 mm: it reaches the 5 N cap within about 0.15 s and keeps pulling at 5 N until the tab has moved 10 mm. It
    holds if the tab moved under 3 mm and the engaged hook patch is still at least 35 mm."""
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    root = shoekit.PATHS["shoe"]
    cen, _ = SHOE.strap_links()
    dt = env.unwrapped.sim.get_physics_dt()
    fast = _S["fast"]
    out = []
    for k, sg in enumerate(SHOE.straps):
        tag = TAGS[k]
        raw = shoekit.raw_state(env, "shoe")
        last = f"strap_{tag}_{SHOE.n_links - 1:02d}"
        p0 = raw["pos"][last]
        direction = shoekit.rotmat(raw["quat"][last][None])[0][:, 0]
        start_w = W0 + np.array((sg.x, cen[-1][0], cen[-1][1]))  # the drive's frame: the tab's start position
        path = f"{root}/pull_gauge_{tag}"
        moved = 0.0
        for i in range(int(round(1.0 / dt))):
            if i % 12 == 0:
                tgt = p0 + direction * min(PULL_STROKE, PULL_SPEED * (i + 12) * dt)
                _drive(stage, path, tgt - start_w, PULL_K, shoescore.PULL_N, damping=3.0)
            _sim_step(env)
            if i % 12 == 11:
                r = shoekit.raw_state(env, "shoe")
                fast.update(_S["n"], r)
                moved = max(moved, float(np.dot(r["pos"][last] - p0, direction)))
        _drive(stage, path, None, 0.0, 0.0, damping=0.0)
        for i in range(int(round(0.25 / dt))):
            _sim_step(env)
            if i % 12 == 11:
                fast.update(_S["n"], shoekit.raw_state(env, "shoe"))
        ov = shoescore.overlap_m(tuple(fast.on[tag].keys()), SHOE.link_len)
        out.append(bool(moved < shoescore.PULL_MOVE_M and ov >= shoescore.OVERLAP_M))
        _S.setdefault("pulls", []).append({"step": _S["n"], "strap": tag, "moved_mm": round(1000 * moved, 2),
                                           "overlap_mm": round(1000 * ov, 1), "holds": out[-1]})
    return out


# -- hooks ------------------------------------------------------------------------------------------------------------

def _set_knee(env):
    """Calibrate the knee springs' set points during the first second (before the arms arrive): an integral trim that
    brings both knee joints back to their start angle, so the springs carry the leg's weight at its mark with the sole
    resting on the table (on the real rig: adjust the spring preload until the sole just rests on its tape mark)."""
    import torch
    art = shoekit.articulation(env, "shoe")
    names = list(art.joint_names)
    jid = [names.index("knee_flex"), names.index("knee_side")]
    q = shoekit.as_np(art.data.joint_pos)[0][jid]
    _S["knee_set"] = _S.get("knee_set", np.zeros(2)) - 0.5 * q
    dev = art.data.joint_pos.torch.device if hasattr(art.data.joint_pos, "torch") else art.data.joint_pos.device
    art.set_joint_position_target_index(target=torch.tensor([_S["knee_set"]], device=dev, dtype=torch.float32),
                                        joint_ids=jid)


@scenekit.on_reset
def _reset(env):
    global _S
    if _S["fast"] is not None:
        _S["fast"].release_all()
    _S = _new_state()


@scenekit.on_step
def _step(env):
    try:
        _step_inner(env)
    except Exception:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        raise


def _step_inner(env):
    _S["n"] += 1
    t = _S["n"] / 20.0
    if _S["n"] <= 20:
        _set_knee(env)
    raw = shoekit.raw_state(env, "shoe")
    if _S["fast"] is None:
        _S["fast"] = shoekit.Fastener(env, "shoe")
    fast = _S["fast"]
    forces = fast.update(_S["n"], raw)
    st = shoekit.state_from_raw(SHOE, raw, tuple(tuple(sorted(fast.on[k])) for k in fast.on))
    sc = _S["scorer"]
    sc.update(_frame(env, st, t))
    if sc.want_pull:
        sc.pull_result(_pull_test(env))
        raw = shoekit.raw_state(env, "shoe")
        st = shoekit.state_from_raw(SHOE, raw, tuple(tuple(sorted(fast.on[k])) for k in fast.on))
        sc.update(_frame(env, st, t))
    _log(st, sc, forces)


def _log(st, sc, forces) -> None:
    out = os.environ.get("TASK_SCORE_DIR")
    if not out:
        return
    rec = {"step": _S["n"], "score": sc.score, "check": sc.last, "pairs": [list(map(list, p)) for p in st.pairs],
           "flap_deg": [round(float(np.degrees(a)), 1) for a in st.flap_angle],
           "leg": np.round(st.leg_pos - W0, 4).tolist(),
           "f": {f"{k[0]}{k[1]}-{k[2]}": round(v, 2) for k, v in forces.items() if v > 0.3},
           "events": [e for e in _S["fast"].events if e[0] == _S["n"]],
           "tabs": [np.round(lk[-1] - W0, 4).tolist() for lk in st.links]}
    if "pulls" in _S:
        rec["pulls"] = _S["pulls"]
    _S["log"].append(rec)
    if _S["n"] % 20 == 0 or sc.pull is not None:
        pathlib.Path(out).mkdir(parents=True, exist_ok=True)
        (pathlib.Path(out) / "task_log.json").write_text(json.dumps(_S["log"], default=float))


def success(objects) -> bool:
    steptime.tick()
    return bool(_S["scorer"].success)


def score(objects) -> float:
    return float(_S["scorer"].score)


def demo():
    """Waypoints for each arm, played open loop by the harness's ``yam_scripted`` policy (shoedemo.py)."""
    import shoedemo
    return shoedemo.plan(MODE, W0, SHOE)
