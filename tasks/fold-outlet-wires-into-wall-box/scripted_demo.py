"""The scripted demo for this task, and the logger used to build it. Runs in Isaac Sim with the yam_arena environment.

    python scripted_demo.py fold_outlet_wires_into_wall_box.py out --sweep j-run --budget-min 40

``--sweep`` plays one or more named trials (outlet_logic.TRIALS) in one simulator launch; ``j-run`` is the demo. For
each trial it writes, under ``out/<trial>/``, one MP4 per camera, ``probe.log`` (a line every half second: where the
outlet is, how far each strap corner stands off the wall, whether its screw holes are on the boss holes, each wire's
points, the arms' targets and torques, and the task's score) and ``result.json``.

The demo is closed loop. It reads from the simulator: each wire loop's highest point in front of the wall, just before
the left gripper's move on that wire; the wire's sideways position while the fingertips sweep it into the box; and the
outlet's position and roll while the right gripper pushes it home (to keep it centred on the opening, and to hold the
push back until it is). It never moves an object or edits the simulation.

Without ``--sweep`` it plays one of the early open-loop development variants chosen by OUTLET_PROBE (hang, straight,
bend, fold-down, ...), kept for the record; none of those is the demo.

Cameras render natively at 640x360 (yam_arena resizes to --cam-width x --cam-height; larger only upscales), and
Isaac's default DLSS renders at 427x240 and upscales, so this switches it to DLAA (native) first.
"""

import argparse
import json
import os
import time
from pathlib import Path

import numpy as np

CONTROL_HZ = 20


def _write_mp4(frames, path) -> None:
    import imageio

    # quality=None drops imageio's own crf 23; 18 keeps the thin wires readable
    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=CONTROL_HZ, codec="libx264", quality=None,
                    macro_block_size=8, pixelformat="yuv420p", output_params=["-crf", "18", "-movflags", "+faststart"])


def _sharp_render() -> dict:
    """Render cameras at their full size. Isaac's default anti-aliasing is DLSS, which renders at a fraction of the
    size and upscales (427x240 for these 640x360 cameras): soft video. DLAA is the same network at native size.
    Verified on the same VM for the drain task on 15 Nov (aa/op read 3, set to 4). Logged before and after."""
    try:
        import carb
    except ImportError:
        return {"render": "carb not importable; left as is"}
    s = carb.settings.get_settings()
    keys = ("/rtx/post/aa/op", "/rtx/post/dlss/execMode")
    before = {k: s.get(k) for k in keys}
    s.set_int("/rtx/post/aa/op", 4)  # 4 = DLAA
    s.set_int("/rtx/post/dlss/execMode", 2)  # Quality, should anything still fall back to DLSS
    return {"render": {"before": before, "after": {k: s.get(k) for k in keys}}}


ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
FINGERS = ["joint7", "joint8"]


def _arms_row(env, labkit, arms, acts, t) -> dict:
    """Since round 3: where each hand was told to be and where it is, how far each arm joint lags its target, the torque
    each applies (the YAM's limits: 27 N m for joints 1-3, 7 N m for 4-6), and the fingers. Never fails the run."""
    try:
        from yam_arena.runtime import tcp_world

        base = env.unwrapped
        origin = labkit._np(base.scene.env_origins)[0]
        out = {}
        for arm, plan in arms.items():
            robot = base.scene[f"{arm}_robot"]
            names = list(robot.joint_names)
            want = np.asarray(plan.target(t)[0], float)
            got = np.asarray(tcp_world(robot), float) - origin
            q = labkit._np(robot.data.joint_pos)[0]
            q_arm = np.array([q[names.index(n)] for n in ARM_JOINTS])
            q_err = np.degrees(np.asarray(acts[arm][0][:6], float) - q_arm)
            row = {"q_deg": [round(float(np.degrees(v)), 3) for v in q_arm],
                   "hand_want_mm": [round(1000 * v, 1) for v in want], "hand_got_mm": [round(1000 * v, 1) for v in got],
                   "hand_err_mm": round(1000 * float(np.linalg.norm(want - got)), 1),
                   "joint_err_deg": [round(float(v), 2) for v in q_err],
                   "fingers_mm": [round(1000 * float(q[names.index(n)]), 1) for n in FINGERS if n in names]}
            torque = getattr(robot.data, "applied_torque", None)
            if torque is not None:
                tq = labkit._np(torque)[0]
                row["torque_nm"] = [round(float(tq[names.index(n)]), 2) for n in ARM_JOINTS]
            out[arm] = row
        return out
    except Exception as err:  # noqa: BLE001 - logging must never stop a probe
        return {"arms_error": f"{type(err).__name__}: {err}"}


def _row(t, task, labkit, env, info, wires=False) -> dict:
    L = task.L
    s = labkit.state(env, "outlet")
    gaps = L.strap_gaps(s.pos, s.quat_xyzw)
    tabs = L.tab_offsets(s.pos, s.quat_xyzw)
    front = {n: float(L.WALL_X - np.min(p[:, 0])) for n, p in s.polylines.items()}  # >0: reaches in front
    rest = labkit.harness_rest("outlet")
    finite = bool(np.all(np.isfinite(s.pos)) and all(np.all(np.isfinite(p)) for p in s.polylines.values()))
    return {
        "t": round(t, 2),
        "device_mm": [round(1000 * v, 1) for v in s.pos],
        "strap_gap_mm": [round(1000 * g, 1) for g in gaps],
        "tab_off_mm": [round(1000 * v, 1) for v in tabs],
        "wire_front_mm": {n: round(1000 * v, 1) for n, v in front.items()},
        "set_deg_max": None if rest is None else round(float(np.degrees(np.max(np.abs(rest)))), 1),
        "set_deg_sum": None if rest is None else round(float(np.degrees(np.sum(np.abs(rest)))), 1),
        "speed_mm_s": round(1000 * float(np.linalg.norm(s.lin_vel)), 1),
        "finite": finite,
        "flush": L.flush(s.pos, s.quat_xyzw),
        "showing": L.wire_showing(s.polylines),
        "wires_outside": L.wires_outside(s.polylines),  # round 13: in front of the wall, over the edge or under the strap
        "hardened_max_per_m": (lambda h: None if h is None else round(float(np.max(h)), 1))(
            getattr(labkit, "harness_hardening", lambda n: None)("outlet")),
        "score": info.get("score"),
        **({"wires_mm": {n: np.round(1000 * p, 1).tolist() for n, p in s.polylines.items()}} if wires else {}),
    }


def _tracking(log: list, arm: str = "right") -> dict:
    """How well the (working) right hand followed its plan: the worst gap between planned and actual hand position,
    the gap when the last hold ended (just before letting go), how far the hand really moved during the move against
    how far it was told to, and each joint's peak torque (limits 27 N m for joints 1-3, 7 N m for 4-6)."""
    rows = [(r, r.get("arms", {}).get(arm)) for r in log]
    rows = [(r, a) for r, a in rows if isinstance(a, dict) and "hand_err_mm" in a]
    if not rows:
        return {"note": "no arm data logged"}
    out = {"err_max_mm": max(a["hand_err_mm"] for _, a in rows)}
    grip = [i for i, (r, _) in enumerate(rows) if str(r.get(arm, "")).endswith("grip")]
    holds = [i for i, (r, _) in enumerate(rows) if str(r.get(arm, "")).endswith("hold")]
    if holds:
        end = rows[holds[-1]][1]
        out["err_at_hold_end_mm"] = end["hand_err_mm"]
        start = rows[grip[-1]][1] if grip else rows[0][1]
        want = np.subtract(end["hand_want_mm"], start["hand_want_mm"])
        got = np.subtract(end["hand_got_mm"], start["hand_got_mm"])
        out["move_told_mm"] = [round(float(v), 1) for v in want]
        out["move_done_mm"] = [round(float(v), 1) for v in got]
    torques = [a["torque_nm"] for _, a in rows if "torque_nm" in a]
    if torques:
        out["torque_peak_nm"] = [round(float(v), 2) for v in np.max(np.abs(np.asarray(torques)), axis=0)]
    return out


def _verdict(variant: str, log: list, task) -> dict:
    """What each variant was run to find out, with the pass line the spec set before any run."""
    L = task.L
    first, last = log[0], log[-1]
    dev = lambda r: np.asarray(r["device_mm"]) / 1000  # noqa: E731
    out = {"stable": all(r["finite"] for r in log) and last["speed_mm_s"] < 10.0}
    out["tracking"] = _tracking(log)
    if variant == "hang":
        sag = float(dev(first)[2] - dev(last)[2])
        out.update(sag_mm=round(1000 * sag, 1), passed=out["stable"] and sag < 0.010)
    elif variant == "air":  # no outlet involved: did the hand get where it was told?
        tr = out["tracking"]
        out.update(passed=bool(tr.get("err_at_hold_end_mm") is not None and tr["err_at_hold_end_mm"] < 5.0))
    elif variant == "bend":
        # Carried sideways (either way) while held, then let go: the farthest it got, and how much of that it kept.
        y0 = float(dev(first)[1])
        far = max((float(dev(r)[1]) for r in log), key=lambda v: abs(v - y0))
        moved = abs(far - y0)
        kept = (float(dev(last)[1]) - y0) / (far - y0) if moved > 0.005 else 0.0
        if moved >= 0.025:
            outcome = "kept" if kept >= 0.7 else "springs_back" if kept < 0.3 else "partly_kept"
        else:
            outcome = "springs_back" if moved > 0.005 and kept < 0.3 else "unclear"  # too short a carry to judge
        out.update(moved_mm=round(1000 * moved, 1), kept_fraction=round(kept, 2), big_enough=bool(moved >= 0.025),
                   outcome=outcome, passed=bool(out["stable"] and outcome == "kept"))
    else:  # straight and the fold paths: did it seat?
        best = float(min(max(r["strap_gap_mm"]) for r in log))
        out.update(best_gap_mm=best, gap_at_end_mm=max(last["strap_gap_mm"]), tab_off_end_mm=last["tab_off_mm"],
                   flush_at_end=bool(last["flush"]), showing_at_end=list(last["showing"]),
                   wire_front_end_mm=last["wire_front_mm"],
                   passed=bool(out["stable"] and last["flush"] and not last["showing"]))
    out["stable"] = bool(out["stable"])
    return out


def run(task_path, out_dir, max_steps=1200, cam_width=640, cam_height=360) -> dict:
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.demo import ArmPlan
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    from yam_arena.station import BASES

    variant = os.environ.get("OUTLET_PROBE", "straight")
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=str(Path(task_path).resolve()), object_jitter=0, control_hz=CONTROL_HZ,
                               cam_width=cam_width, cam_height=cam_height)
    task = emb.task.module
    import labkit  # the copy next to task.py, already imported by it

    obs = emb.reset(Scene(id="probe", instruction=task.INSTRUCTION))
    print(json.dumps(_sharp_render()), flush=True)
    top = next(c for c in obs.images if "top" in c)
    frames = {c: [np.asarray(i)] for c, i in obs.images.items()}
    plan = task.probe_demo()
    arms = {arm: ArmPlan(ws, BASES[arm]) for arm, ws in plan.items()}
    seconds = task.L.plan_seconds(task.L.probe_plan(variant))
    steps = min(max_steps, int((seconds + 2.0) * CONTROL_HZ))
    log, info = [], {}
    with (out / "probe.log").open("w", encoding="utf-8") as f:
        for step in range(1, steps + 1):
            t = step / CONTROL_HZ
            acts = {arm: p.action7(t) for arm, p in arms.items()}
            res = emb.step(Action(data=pack(acts["left"][0], acts["right"][0])))
            obs, info = res.observation, res.info
            for c, image in obs.images.items():
                frames[c].append(np.asarray(image))
            if step % 10 == 0 or step == 1:
                row = _row(t, task, labkit, emb._env, info)
                row.update(left=acts["left"][1], right=acts["right"][1])
                row["arms"] = _arms_row(emb._env, labkit, arms, acts, t)
                log.append(row)
                f.write(json.dumps(row) + "\n")
                print(json.dumps(row), flush=True)
                if not row["finite"]:
                    print("NaN in the harness: stopping", flush=True)
                    break
    settings = {k: os.environ.get(k) for k in ("OUTLET_PROBE", "OUTLET_STYLE", "OUTLET_ARMATURE",
                                               "OUTLET_YIELD_SCALE", "OUTLET_STIFFNESS_SCALE")}
    result = {"variant": variant, "settings": settings, "steps": step, **_verdict(variant, log, task),
              "worst_ik_mm": {arm: round(p.worst_ik_mm, 2) for arm, p in arms.items()}}
    import imageio

    for c, fs in frames.items():
        _write_mp4(fs, out / f"probe_{c}.mp4")
    imageio.imwrite(out / "setup.png", frames[top][min(5, len(frames[top]) - 1)])
    (out / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    return result


# -- round 5: many trials in one launch -------------------------------------------------------------------------------------
ARM_LIMIT_NM = np.array([27.0, 27.0, 27.0, 7.0, 7.0, 7.0])


def _logic(task_path):
    """outlet_logic, loaded on its own (numpy only), so trial names are checked before the simulator starts."""
    import importlib.util
    import sys

    folder = Path(task_path).resolve().parent
    if "outlet_logic" in sys.modules:
        return sys.modules["outlet_logic"]
    spec = importlib.util.spec_from_file_location("outlet_logic", folder / "outlet_logic.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules["outlet_logic"] = mod
    spec.loader.exec_module(mod)
    return mod


def _trial_names(task_path, which: str) -> list:
    logic = _logic(task_path)
    names = list(logic.TRIAL_NAMES) if which == "all" else [n.strip() for n in which.split(",") if n.strip()]
    for n in names:
        logic.trial(n)
    return names


def _hand_wrench(q_deg, dtau, arm: str) -> list:
    """The force (N) and moment (N m) the hand pushes with, from how much the joint torques changed (dtau) against a
    reference with no contact: dtau = J^T w, so w = pinv(J^T) dtau, J by finite differences of yam_arena's own
    forward kinematics. An estimate: where the reference pose differs, gravity's share differs a little."""
    from yam_arena import ik
    from yam_arena.station import BASES

    q = np.radians(np.asarray(q_deg, float))
    p0, r0 = ik.tcp_pose(q, BASES[arm])
    jac = np.zeros((6, 6))
    h = 1e-5
    for i in range(6):
        dq = q.copy()
        dq[i] += h
        p, r = ik.tcp_pose(dq, BASES[arm])
        dr = r @ r0.T
        jac[:3, i] = (p - p0) / h
        jac[3:, i] = np.array([dr[2, 1] - dr[1, 2], dr[0, 2] - dr[2, 0], dr[1, 0] - dr[0, 1]]) / (2 * h)
    w = np.linalg.pinv(jac.T) @ np.asarray(dtau, float)
    return [round(float(v), 2) for v in w]


def _arm(r, arm="right"):
    a = r.get("arms", {}).get(arm)
    return a if isinstance(a, dict) and "q_deg" in a and "torque_nm" in a else None


def _rows_labelled(log, label, arm="right"):
    return [r for r in log if str(r.get(arm, "")) == label]


def _push_verdict(log) -> dict:
    """Did it seat while held (the hand still pressing: in the real task the screws hold it there) and after the hand
    let go and backed off? How hard was the hand pushing at the end of the hold?"""
    last = log[-1]
    holds = _rows_labelled(log, "hold")
    h = holds[-1] if holds else last
    out = {"best_gap_mm": float(min(max(r["strap_gap_mm"]) for r in log)),
           "gap_at_hold_end_mm": max(h["strap_gap_mm"]), "tab_off_at_hold_end_mm": h["tab_off_mm"],
           "flush_at_hold_end": bool(h["flush"]), "showing_at_hold_end": list(h["showing"]),
           "wire_front_at_hold_end_mm": h["wire_front_mm"],
           "gap_at_end_mm": max(last["strap_gap_mm"]), "tab_off_end_mm": last["tab_off_mm"],
           "flush_at_end": bool(last["flush"]), "showing_at_end": list(last["showing"]),
           "wire_front_end_mm": last["wire_front_mm"], "set_deg_sum_end": last["set_deg_sum"]}
    out["springback_mm"] = round(out["gap_at_end_mm"] - out["gap_at_hold_end_mm"], 1)
    out["seated_held"] = bool(out["flush_at_hold_end"] and not out["showing_at_hold_end"])
    grip = _rows_labelled(log, "grip hold")  # round 10's aim-thumb: the gripped push's hold, before the face press
    if grip:
        g = grip[-1]
        out.update(gap_at_grip_hold_mm=max(g["strap_gap_mm"]), tab_off_at_grip_hold_mm=g["tab_off_mm"],
                   showing_at_grip_hold=list(g["showing"]),
                   seated_grip_held=bool(g["flush"] and not g["showing"]), seated_face_held=out["seated_held"])
        out["seated_held"] = bool(out["seated_held"] or out["seated_grip_held"])  # either hold counts
    out["seated_after_release"] = bool(out["flush_at_end"] and not out["showing_at_end"])
    free = [r for r in _rows_labelled(log, "watch it spring back") if _arm(r)]
    if holds and free and _arm(h):
        a = _arm(h)
        dtau = np.subtract(a["torque_nm"], _arm(free[-1])["torque_nm"])
        out["hand_push_at_hold_end"] = {"force_n_and_moment_nm": _hand_wrench(a["q_deg"], dtau, "right"),
                                        "note": "estimate: the reference is the free hand 7 cm back"}
        out["hand_lag_x_at_hold_end_mm"] = round(a["hand_want_mm"][0] - a["hand_got_mm"][0], 1)
    torques = [_arm(r)["torque_nm"] for r in log if _arm(r)]
    if torques:
        out["torque_frac_of_limit_peak"] = [round(float(v), 2) for v in
                                            np.max(np.abs(np.asarray(torques)), axis=0) / ARM_LIMIT_NM]
    return out


def _press_verdict(log, steps) -> dict:
    """Wall press: at the end of each hold, how far the hand trails its aim, the push force, and each joint's torque as
    a share of its limit. The reference is the hand just touching the wall (nearly the same pose, no push)."""
    touch = [r for r in _rows_labelled(log, "touch the wall") if _arm(r)]
    if not touch:
        return {"note": "no touch row logged"}
    ref = _arm(touch[-1])
    stages = []
    for d in steps[1:]:
        rows = [r for r in _rows_labelled(log, f"hold {1000 * d:.0f} mm") if _arm(r)]
        if not rows:
            continue
        a = _arm(rows[-1])
        w = _hand_wrench(a["q_deg"], np.subtract(a["torque_nm"], ref["torque_nm"]), "right")
        stages.append({"aim_past_wall_mm": round(1000 * d),
                       "hand_lag_x_mm": round(a["hand_want_mm"][0] - a["hand_got_mm"][0], 1),
                       "hand_err_mm": a["hand_err_mm"], "push_force_n": w[:3], "moment_nm": w[3:],
                       "torque_frac_of_limit": [round(abs(v) / lim, 2) for v, lim in zip(a["torque_nm"],
                                                                                         ARM_LIMIT_NM)],
                       "joint_err_deg": a["joint_err_deg"]})
    return {"stages": stages}


def sweep(task_path, out_dir, names, budget_min=24.0, cam_width=640, cam_height=360) -> dict:
    """Run trials back to back in one simulator launch, each from a fresh reset, in the order given, skipping any that
    would overrun the budget. Each trial's wire torque cap is set while the sim runs (labkit.set_harness_torque_cap),
    unless OUTLET_TORQUE_CAP built it in: then only the capped trials run."""
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.demo import ArmPlan
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    from yam_arena.station import BASES

    t0 = time.time()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=str(Path(task_path).resolve()), object_jitter=0, control_hz=CONTROL_HZ,
                               cam_width=cam_width, cam_height=cam_height)
    task = emb.task.module
    L = task.L
    import labkit

    built = os.environ.get("OUTLET_TORQUE_CAP") or None
    ground = getattr(task, "GROUND", "below")  # the wires' start shape is built in: one launch per ground shape
    summary = {"round": 13, "built_cap": built, "ground": ground, "yield_moment_nm": round(L.yield_moment(), 4),
               "budget_min": budget_min, "order": list(names), "trials": {}, "skipped": {}, "cap_writes": []}
    path = out / "sweep.json"

    def save():
        summary["elapsed_min"] = round((time.time() - t0) / 60, 1)
        path.write_text(json.dumps(summary, indent=2))

    sec_per_step, writer_ok = None, None
    for name in names:
        tr = L.trial(name)
        if tr.ground != ground:
            summary["skipped"][name] = f"other ground shape: run it with OUTLET_GROUND={tr.ground}"
            continue
        if built and tr.cap is False:
            summary["skipped"][name] = "uncapped trial: this launch has the cap built in"
            continue
        if writer_ok is False and tr.cap:
            summary["skipped"][name] = "needs the cap and no runtime writer worked: rerun with OUTLET_TORQUE_CAP=yield"
            save()
            continue
        steps_plan = L.trial_plan(tr)
        steps = int((L.plan_seconds(steps_plan) + 0.5) * CONTROL_HZ)
        elapsed = time.time() - t0
        if sec_per_step is not None and elapsed + sec_per_step * steps + 30 > 60 * budget_min:
            summary["skipped"][name] = (f"budget: about {sec_per_step * steps / 60:.1f} min needed, "
                                        f"{budget_min - elapsed / 60:.1f} left")
            save()
            continue
        print(json.dumps({"trial": name, "why": tr.why, "steps": steps}), flush=True)
        obs = emb.reset(Scene(id=f"probe-{name}", instruction=task.INSTRUCTION))
        if "render" not in summary:
            summary["render"] = _sharp_render()
            summary["startup_s"] = round(time.time() - t0, 1)
        env = emb._env
        if "wire_joints" not in summary:  # round 13: what the articulation really has, against what the copper rule assumes
            try:
                asset = env.unwrapped.scene["outlet"]
                names_ = list(asset.joint_names)
                summary["wire_joints"] = {"count": len(names_), "expected": task.HARNESS.dof_count(),
                                          "first": names_[:6], "last": names_[-4:],
                                          "segment_mm": round(1000 * L.SEGMENT, 2), "hard": getattr(task, "HARD", None),
                                          "first_cap_nmm": round(1000 * task.HARNESS.flow_m[0], 1)}
            except Exception as err:  # noqa: BLE001 - a log line, never a reason to stop
                summary["wire_joints"] = f"{type(err).__name__}: {err}"
        if not built and tr.cap is not None:
            try:
                summary["cap_writes"].append({name: labkit.set_harness_torque_cap(
                    env, "outlet", L.yield_moment() if tr.cap else None)})
                writer_ok = True
            except Exception as err:  # noqa: BLE001 - recorded; the runner relaunches with the cap built in
                summary["cap_writes"].append({name: f"{type(err).__name__}: {err}"})
                writer_ok = False
                if tr.cap:
                    summary["skipped"][name] = "needs the cap and no runtime writer worked"
                    save()
                    continue
        s0 = labkit.state(env, "outlet")
        rest = labkit.harness_rest("outlet")
        reset = {"device_off_start_mm": round(1000 * float(np.linalg.norm(np.asarray(s0.pos) - L.DEVICE_START)), 1),
                 "rest_deg_sum": None if rest is None else round(float(np.degrees(np.sum(np.abs(rest)))), 2)}
        arms = {arm: ArmPlan(ws, BASES[arm]) for arm, ws in task.plan_waypoints(steps_plan).items()}
        # Round 10's aim-thumb: steer the right hand so the held outlet stays centred on the opening (an integral
        # correction on y and z, every 0.5 s), then aim the fingertip press at the outlet's measured y and z. The open-
        # loop arm let the outlet droop 1 to 30 mm and drift 2 to 6 mm sideways in rounds 5-9, against 3.6 mm of
        # clearance to the bosses.
        # Round 12 (Trial.steer "fast"): every 0.1 s, with the roll too (the wrist turned about the push axis); and
        # Trial.gate: the push's x stops advancing while the outlet is off-centre or turned, then catches up.
        steer = tr.path == "aim-thumb" and getattr(tr, "steer", "slow") != "none"
        fast = steer and getattr(tr, "steer", "slow") == "fast"
        every = L.FAST_EVERY if fast else 10
        g = {"twist": 0.0, "x": None, "frozen_s": 0.0, "started": False}
        corr, face_snap, plan_target = np.zeros(3), None, None
        if tr.path == "aim-thumb":
            plan_target = arms["right"].target

            def _steered(t_, _o=plan_target):
                out_ = list(_o(t_))
                out_[0] = np.asarray(out_[0], float) + corr
                if g["x"] is not None:
                    out_[0][0] = g["x"]
                if g["twist"]:
                    out_[1] = labkit.rolled(np.asarray(out_[1], float), g["twist"])
                return tuple(out_)

            arms["right"].target = _steered  # action7 and the logged hand_want both read target()
        corr_l, left_plan = np.zeros(3), None
        if getattr(tr, "track", False):
            left_plan = arms["left"].target

            def _tracked(t_, _o=left_plan):
                out_ = list(_o(t_))
                out_[0] = np.asarray(out_[0], float) + corr_l
                if g.get("lean_hold") is not None:
                    out_[0][1] = g["lean_hold"]
                return tuple(out_)

            arms["left"].target = _tracked
        tdir = out / name
        tdir.mkdir(exist_ok=True)
        keep = [c for c in obs.images if "left" not in c or getattr(tr, "crest", ())]  # left camera when it works
        frames = {c: [np.asarray(obs.images[c])] for c in keep}
        log, ts, step = [], time.time(), 0
        with (tdir / "probe.log").open("w", encoding="utf-8") as f:
            for step in range(1, steps + 1):
                t = step / CONTROL_HZ
                if plan_target is not None:
                    planned = plan_target(t)
                    label = planned[3]
                    st = labkit.state(env, "outlet")
                    dev = np.asarray(st.pos, float)
                    roll = L.roll_deg(st.quat_xyzw)
                    if steer and step % every == 0:
                        if label in L.AIM_LABELS:
                            gain, cy, cz = (L.FAST_GAIN, L.FAST_Y, L.FAST_Z) if fast else (L.AIM_GAIN, L.AIM_Y, L.AIM_Z)
                            corr[1] = np.clip(corr[1] + gain * (L.OPEN_C[0] - dev[1]), -cy, cy)
                            corr[2] = np.clip(corr[2] + gain * (L.OPEN_C[1] - dev[2]), *cz)
                            if fast:
                                g["twist"] = float(np.clip(g["twist"] - L.TWIST_GAIN * roll, -L.TWIST_CAP, L.TWIST_CAP))
                        elif label in L.FACE_LABELS:
                            if face_snap is None:
                                face_snap = dev.copy()
                            corr[1] = np.clip(face_snap[1] - L.OPEN_C[0], -L.AIM_Y, L.AIM_Y)
                            corr[2] = np.clip(face_snap[2] - L.OPEN_C[1], *L.AIM_Z)
                    if getattr(tr, "gate", False):  # every step: the push's x, held back while the outlet is off
                        plan_x = float(planned[0][0])
                        if label in L.GATED_LABELS:
                            g["started"] = True
                        if g["started"] and label in L.AIM_LABELS:
                            off = max(abs(dev[1] - L.OPEN_C[0]), abs(dev[2] - L.OPEN_C[1]))
                            aligned = off <= L.GATE_MM / 1000 and abs(roll) <= L.GATE_DEG
                            if g["x"] is None:
                                g["x"] = plan_x
                            if label in L.GATED_LABELS and not aligned and g["frozen_s"] < L.GATE_MAX_S:
                                g["frozen_s"] += 1.0 / CONTROL_HZ
                            else:  # catch up with the plan, no faster than the gate's speed
                                g["x"] = min(plan_x, g["x"] + L.GATE_SPEED / CONTROL_HZ) if plan_x > g["x"] \
                                    else plan_x
                        elif g["started"]:
                            g["x"] = None  # the gripped push is over: the plan's own x again
                aimed = g.get("aimed", 0)
                if getattr(tr, "live", False) and tr.crest and aimed < len(tr.crest) and t >= (
                        steps_plan["left"][0][0] + L.crest_start(aimed) - (0.3 if aimed == 0 else 0.05)):
                    # Round 13b/c: just before the left hand starts on a wire, read where the loops are and aim that
                    # move (and the later ones) there. Same step times, so the right hand's plan still matches.
                    import dataclasses

                    tops = L.loop_tops(labkit.state(env, "outlet").polylines, tuple(set(tr.crest)))
                    aims = g.setdefault("aims", [None] * len(tr.crest))
                    for k_ in range(aimed, len(tr.crest)):
                        aims[k_] = tops[tr.crest[k_]]
                    whole = L.trial_plan(dataclasses.replace(tr, pieces=0)) if getattr(tr, "pieces", 0) else steps_plan
                    left_new = [whole["left"][0]] + L.live_plan(tr, aims)
                    rest_s = L.plan_seconds(whole) - sum(s_[0] for s_ in left_new)
                    if rest_s > 1e-9:
                        left_new.append((rest_s, None, "open", "ready", "wait"))
                    piece_end = L.piece_end(tr, whole) if getattr(tr, "pieces", 0) else None
                    if piece_end is not None:  # a run stopped after one of its pieces (Trial.pieces)
                        left_new = L.cut_steps(left_new, piece_end)
                    arms["left"].waypoints = task.plan_waypoints({"left": left_new})["left"]
                    g["aimed"] = aimed + 1
                    g["tops"] = {f"{k_}:{tr.crest[k_]}": None if v_ is None else [round(1000 * float(c_), 1) for c_ in v_]
                                 for k_, v_ in enumerate(aims)}
                if left_plan is not None:
                    # Round 13f: the left hand's moves read the wire as they go (a scripted demo may read the scene).
                    try:
                        lab_l = str(left_plan(t)[3])
                        if lab_l.startswith(("sweep the ", "lean the ")):
                            from yam_arena.runtime import tcp_world

                            base_ = env.unwrapped
                            tcp_l = np.asarray(tcp_world(base_.scene["left_robot"]), float) - labkit._np(base_.scene.env_origins)[0]
                            tip_l = tcp_l + L.PAD_TIP * (L.crest_rotation(0.0) @ np.array([0.0, 0.0, 1.0]))
                            wire_l = np.asarray(labkit.state(env, "outlet").polylines[lab_l.split()[2]], float)
                            if lab_l.startswith("sweep the "):  # keep the fingertips on the wire's own y
                                near = wire_l[np.hypot(wire_l[:, 0] - tip_l[0], wire_l[:, 2] - tip_l[2]) < 0.025]
                                if len(near):
                                    corr_l[1] = float(np.clip(corr_l[1] + 0.3 * (float(np.mean(near[:, 1])) - tip_l[1]), -0.03, 0.03))
                            elif g.get("lean_hold") is None:  # a lean stops once the loop's top is near the centre
                                top_l = L.loop_tops({"w": wire_l}, ("w",))["w"]
                                if top_l is None or abs(float(top_l[1]) - float(L.OPEN_C[0])) <= L.LEAN_TO + 0.002:
                                    g["lean_hold"] = float(tcp_l[1])
                        elif lab_l != "hold it there":
                            corr_l[:] = 0.0
                            g["lean_hold"] = None
                    except Exception as err:  # noqa: BLE001 - never stop the run for this
                        g["track_error"] = f"{type(err).__name__}: {err}"
                acts = {arm: p.action7(t) for arm, p in arms.items()}
                res = emb.step(Action(data=pack(acts["left"][0], acts["right"][0])))
                for c in keep:
                    frames[c].append(np.asarray(res.observation.images[c]))
                if step % 40 == 0:  # a snapshot every 2 s of the run, so it can be watched while it runs
                    try:
                        import imageio

                        for c in keep:
                            imageio.imwrite(tdir / f"live_{c}.png", frames[c][-1])
                    except Exception:  # noqa: BLE001 - only a convenience
                        pass
                if step % 10 == 0 or step == 1:
                    row = _row(t, task, labkit, env, res.info, wires=True)
                    row.update(left=acts["left"][1], right=acts["right"][1])
                    row["arms"] = _arms_row(env, labkit, arms, acts, t)
                    st_ = labkit.state(env, "outlet")
                    seen = getattr(task, "SEEN", {})
                    if "hand_clear" in seen:  # task.py's on_step check: the grippers' clearance from the outlet
                        hc = seen["hand_clear"]
                        row["hand_clear_mm"] = None if hc == float("inf") else round(1000 * hc, 1)
                        row["clear_s"] = round(float(seen.get("clear_s", 0.0)), 2)  # let go and still for this long
                        if seen.get("error"):
                            row["hand_clear_error"] = seen["error"]
                    row["roll_deg"] = round(L.roll_deg(st_.quat_xyzw), 2)
                    row["tilt_deg"] = round(L.tilt_from_upright(st_.quat_xyzw), 2)
                    if plan_target is not None:
                        row["steer_mm"] = [round(1000 * float(v), 1) for v in corr]
                        row["twist_deg"] = round(g["twist"], 2)
                        if g["x"] is not None:
                            row["gate_held_back_mm"] = round(1000 * (float(plan_target(t)[0][0]) - g["x"]), 1)
                            row["gate_frozen_s"] = round(g["frozen_s"], 2)
                    log.append(row)
                    f.write(json.dumps(row) + "\n")
                    if not row["finite"]:
                        print("NaN in the harness: stopping this trial", flush=True)
                        break
        run_s = time.time() - ts
        sec_per_step = run_s / max(step, 1)
        first, last = log[0], log[-1]
        result = {"trial": name, "path": tr.path, "ground": ground, "aim_mm": round(1000 * tr.aim), "push_s": tr.push_s,
                  "cap": True if built else tr.cap, "why": tr.why, "steps": step, "reset": reset,
                  "stable": bool(all(r["finite"] for r in log) and last["speed_mm_s"] < 10.0),
                  "wall_s_per_step": round(sec_per_step, 3), "tracking": _tracking(log),
                  "worst_ik_mm": {arm: round(p.worst_ik_mm, 2) for arm, p in arms.items()}}
        grips = _rows_labelled(log, "grip")
        if grips:
            result["sag_before_grip_mm"] = round(first["device_mm"][2] - grips[0]["device_mm"][2], 1)
        if tr.path == "wall-press":
            result.update(_press_verdict(log, L.PRESS_STEPS))
        elif tr.path == "bend":
            result.update({k: v for k, v in _verdict("bend", log, task).items() if k not in ("stable", "tracking")})
        else:
            result.update(_push_verdict(log))
        # Round 12: the task's own scorer (task.py score(): grippers 3 cm clear, upright, at rest; 1.0 seated, 0.5 flush
        # but off its holes or a wire showing, 0.25 within 15 mm), as yam_arena reported it every logged row.
        scores = [r["score"] for r in log if isinstance(r.get("score"), (int, float))]
        if scores:
            result["task_score_end"] = scores[-1]
            result["task_score_max"] = max(scores)
            full = [r["t"] for r in log if r.get("score") == 1.0]
            result["task_score_1_from_s"] = full[0] if full else None
        result["wires_outside_end"] = last.get("wires_outside")
        result["track_error"] = g.get("track_error")
        result["loop_tops_mm"] = g.get("tops")  # live crest trials: where the left hand aimed
        result["hardened_max_per_m"] = last.get("hardened_max_per_m")
        result["roll_end_deg"] = last.get("roll_deg")
        result["device_off_end_mm"] = [round(last["device_mm"][1] - 1000 * float(L.OPEN_C[0]), 1),
                                       round(last["device_mm"][2] - 1000 * float(L.OPEN_C[1]), 1)]
        import imageio

        for c, fs in frames.items():
            _write_mp4(fs, tdir / f"{c}.mp4")
        if not (out / "setup.png").exists():
            top = next((c for c in keep if "top" in c), keep[0])
            imageio.imwrite(out / "setup.png", frames[top][min(5, len(frames[top]) - 1)])
        del frames
        (tdir / "result.json").write_text(json.dumps(result, indent=2))
        summary["trials"][name] = result
        save()
        print(json.dumps({"trial_done": name, "minutes": round(run_s / 60, 1), "result": {
            k: result.get(k) for k in ("task_score_end", "task_score_max", "wires_outside_end", "seated_held", "seated_grip_held",
                                       "seated_after_release", "gap_at_end_mm", "tab_off_end_mm", "showing_at_end",
                                       "roll_end_deg", "device_off_end_mm", "gap_at_grip_hold_mm", "gap_at_hold_end_mm",
                                       "stages", "outcome", "kept_fraction")}}), flush=True)
    pushes = {n: r for n, r in summary["trials"].items() if "seated_held" in r}
    summary["seated_held_by"] = [n for n, r in pushes.items() if r["seated_held"]]
    summary["seated_after_release_by"] = [n for n, r in pushes.items() if r["seated_after_release"]]
    summary["task_score_1_by"] = [n for n, r in pushes.items() if r.get("task_score_end") == 1.0]
    summary["task_scores_end"] = {n: r.get("task_score_end") for n, r in pushes.items()}
    if pushes:
        best = min(pushes, key=lambda n: pushes[n]["gap_at_hold_end_mm"])
        summary["closest"] = {"trial": best, "gap_at_hold_end_mm": pushes[best]["gap_at_hold_end_mm"]}
    summary["done"] = True
    save()
    print(json.dumps({"sweep_done": True, "task_score_1_by": summary["task_score_1_by"],
                      "task_scores_end": summary["task_scores_end"], "seated_held_by": summary["seated_held_by"],
                      "skipped": summary["skipped"]}), flush=True)
    return summary


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1200)
    ap.add_argument("--cam-width", type=int, default=640, help="the cameras render 640x360; larger only upscales")
    ap.add_argument("--cam-height", type=int, default=360)
    ap.add_argument("--sweep", default=None, help="round 5: 'all' or comma-separated trial names (outlet_logic.TRIALS)")
    ap.add_argument("--budget-min", type=float, default=24.0, help="don't start a trial that would overrun this")
    a = ap.parse_args()
    if a.sweep:
        names = _trial_names(a.task, a.sweep)  # unknown names fail here, before the simulator starts
        sweep(a.task, a.out, names, a.budget_min, a.cam_width, a.cam_height)
    else:
        run(a.task, a.out, a.max_steps, a.cam_width, a.cam_height)
