"""Connector bench (not a benchmark task): the parts alone in the task's scene and physics (240 Hz, CPU), the arms
parked. A "virtual hand" holds the housing where it lies (a stiff spring and damper on its pose), the contact starts
lined up in cavity 3's mouth (the nose 2 mm in), and a push along the cavity, ramped, drives it in; the task's own tug
follows once the push stops. Cases:

- seat: pushed on the crimp until seated (force vs depth: where it clicks), then the tug;
- far: pushed on the wire 60 mm behind the crimp (does the wire buckle before it clicks?);
- half: pushed until the shoulder rides under the finger (finger open), then the tug (should pull out);
- lock: seated, then the secondary lock set over the slot and pressed down (should go fully in);
- forced: half-seated, then the lock pressed down hard (blocked by the open finger: forced, latched).

    python ct_bench.py --case seat --out ../../results/ct_bench_1      # one JSON per case, prints BENCH {...}
    python ct_bench.py --case all
"""

from __future__ import annotations

import argparse
import ast
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
CASES = ("seat", "far", "half", "lock", "forced", "drop", "probe", "latch", "overlap", "wall")
PUSH_SPEED = 10e-3  # m/s: the hand pushes harder while the contact moves slower than this ...
PUSH_GAIN = 2000.0  # ... the push changing by this much per (m/s of shortfall) per second (20 N/s when stopped) ...
PUSH_MAX = 40.0  # ... capped (N). (A stiff spring toward a moving target, 4000 N/m on a 10 g contact, blew up, bench 16.)
FAR_LINK = 6  # the wire link pushed in case "far" (60-70 mm behind the crimp)
HALF_OPEN = 0.7e-3  # case half/forced: stop pushing once a finger is this far open (the shoulder under it)
LOCK_PRESS = (15.0, 40.0)  # N on the lock: case lock, case forced
HALF_HOLD = 3.0  # N: case forced, the contact pushed under the finger while the lock is pressed (finger ~0.3 mm open)
HOLD_K, HOLD_D, HOLD_KR, HOLD_DR = 200.0, 3.0, 0.02, 2e-4  # weak: the hand mainly takes the push's reaction
# (explicit per-step gains this light keep stable on a 50 g housing; 3000 N/m and 3 Nm/rad blew up, bench 1)
SETTLE_S, WAIT_S = 0.3, 4.0
LOG_EVERY = 12  # physics steps between log rows


def run(emb, case: str) -> dict:
    import torch
    from inspect_robots import Scene
    from inspect_robots.types import Action

    obs = emb.reset(Scene(id=f"ct-bench-{case}", instruction="bench"), seed=0)
    emb._env.unwrapped.cfg.sim.render_interval = 10**9
    hold = Action(data=np.asarray(obs.state["joint_pos"], dtype=float))
    base = emb._env.unwrapped
    task = emb.task.module
    ct = task.ct
    from yam_arena.scene import object_states

    dev = base.device
    arts = {k: base.scene.articulations[ct._NAMES[k]] for k in ("housing", "contact", "lock")}
    origin = base.scene.env_origins[0].detach().cpu().numpy()

    def tensor(rows):
        return torch.tensor(rows, dtype=torch.float32, device=dev)

    poses = ct.body_poses(base, ct._NAMES["housing"])
    h0_p, h0_r = poses["base"]
    xc, zc = ct.CAVITIES[ct.TARGET_CAVITY]

    def place(kind, pos, rot):
        q = ct._quat_xyzw(rot)
        art = arts[kind]
        pose = tensor([[*(np.asarray(pos) + origin), q[0], q[1], q[2], q[3]]])
        k = len(art.joint_names)  # (joints first: written after the root, they put the root back, bench 2-3)
        art.write_joint_position_to_sim_index(position=torch.zeros((1, k), device=dev), env_ids=[0])
        art.write_joint_velocity_to_sim_index(velocity=torch.zeros((1, k), device=dev), env_ids=[0])
        art.write_root_pose_to_sim_index(root_pose=pose, env_ids=[0])
        art.write_root_velocity_to_sim_index(root_velocity=tensor([[0.0] * 6]), env_ids=[0])

    # the contact lined up in cavity 3: its x axis along -y of the housing, the nose 2 mm in
    c_rot = h0_r @ np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # x -> -y_h, y -> x_h
    crimp0 = h0_p + h0_r @ np.array([xc, 2e-3 + ct.NOSE_X, zc])
    if case == "probe":  # (debug) the shoulder 1 mm before the finger, pushed with 2 N, logged every step
        crimp0 = h0_p + h0_r @ np.array([xc, ct.Y_LAT[0] - 1e-3 + ct.SHOULDER_FRONT_X, zc])
    if case == "overlap":  # (debug) the shoulder's middle at the finger's middle (0.4 mm into it), no push
        crimp0 = h0_p + h0_r @ np.array([xc, (ct.Y_LAT[0] + ct.Y_LAT[1]) / 2 + (ct.SHOULDER_FRONT_X
                                                                              + ct.SHOULDER_REAR_X) / 2, zc])
    if case == "wall":  # (debug) in a plain section (shoulder 5 mm short of the fingers), then pushed sideways
        crimp0 = h0_p + h0_r @ np.array([xc, ct.Y_LAT[0] - 5e-3 + ct.SHOULDER_FRONT_X, zc])
    if case == "drop":  # (debug) lying across the housing's top, 10 mm above it: does it land on it?
        crimp0 = h0_p + h0_r @ np.array([0.0, 30e-3, ct.H_H / 2 + 10e-3])
    place("contact", crimp0, c_rot)

    def wrench(kind, forces_by_body, torques_by_body=None):
        art = arts[kind]
        n = len(art.body_names)
        f = torch.zeros((1, n, 3), dtype=torch.float32, device=dev)
        t = torch.zeros_like(f)
        for b, v in forces_by_body.items():
            f[0, list(art.body_names).index(b)] = tensor(list(v))
        for b, v in (torques_by_body or {}).items():
            t[0, list(art.body_names).index(b)] = tensor(list(v))
        art.permanent_wrench_composer.set_forces_and_torques_index(forces=f, torques=t, body_ids=list(range(n)),
                                                                   is_global=True)

    def hold_housing(reaction=(0.0, 0.0, 0.0)):
        """The housing held: ``reaction`` (the push's, taken by the hand) plus a weak spring and damper on its
        pose."""
        p, r = ct.body_poses(base, ct._NAMES["housing"])["base"]
        v = ct.body_vels(base, ct._NAMES["housing"])["base"]
        dr = r @ h0_r.T
        rotvec = 0.5 * np.array([dr[2, 1] - dr[1, 2], dr[0, 2] - dr[2, 0], dr[1, 0] - dr[0, 1]])
        f = -HOLD_K * (p - h0_p) - HOLD_D * v[0] + np.asarray(reaction)
        tq = -HOLD_KR * rotvec - HOLD_DR * v[1]
        wrench("housing", {"base": f}, {"base": tq})

    hp = ct.body_poses(base, ct._NAMES["housing"])
    print("LATCHES " + json.dumps({b: [round(float(v) * 1e3, 3) for v in h0_r.T @ (p - h0_p)] for b, (p, _) in hp.items()}))
    if case == "wall":
        for k in range(480):
            f = 0.0 if k < 120 else (5.0 if k < 360 else 0.0)
            wrench("contact", {"contact": h0_r @ np.array([0.0, 0.0, -f])})  # down, at the contact's middle
            emb.step(hold)
            if k % 40 == 0 or k in (119, 359):
                g = ct.geometry(base)
                print("WALL", k, "F", f, "crimp_xz", [round(float(g["crimp"][i]) * 1e3, 3) for i in (0, 2)],
                      "nose_xz", [round(float(g["nose"][i]) * 1e3, 3) for i in (0, 2)])
        return {"case": case, "events": {}, "log": []}
    if case in ("latch", "overlap"):  # (debug) 5 N outward on finger 3: does it open (5 N / 30 N/mm = 0.17 mm)?
        for k in range(240):
            if case == "latch":
                wrench("housing", {f"latch{ct.TARGET_CAVITY}o": h0_r @ np.array([5.0, 0.0, 0.0])})
            emb.step(hold)
            if k % 40 == 0:
                g = ct.geometry(base)
                print("LATCH", k, [round(v * 1e3, 3) for v in g["latch_open"]], "crimp", [round(v * 1e3, 3) for v in g["crimp"]],
                      "nose", [round(v * 1e3, 3) for v in g["nose"]])
        return {"case": case, "events": {}, "log": []}
    ct.HANDS_ON[0] = True
    dt = 1.0 / 240.0
    t, phase, t_phase, push = 0.0, "push", 0.0, 0.0
    log, events = [], {}
    push_body = f"w{FAR_LINK}" if case == "far" else "contact"
    max_depth = -1.0
    target = None
    peak = {"force_N": 0.0, "crimp_mm": None}
    while t < 30.0:
        st = object_states(emb._env, emb.task.names)
        t = float(base.episode_length_buf[0]) * dt
        h = st["housing"]
        geo = ct.geometry(base)
        crimp_y = float(geo["crimp"][1])
        max_depth = max(max_depth, crimp_y)
        opn = geo["latch_open"][ct.TARGET_CAVITY - 1]
        axis_in = h0_r @ np.array([0.0, 1.0, 0.0])
        if case == "drop":
            if t > 2.0:
                phase = "done"
        elif phase == "push":
            if target is None:
                target = crimp_y
            target += PUSH_SPEED * dt
            vel = float(ct.body_vels(base, ct._NAMES["contact"])["contact"][0] @ axis_in)
            cap = 8.0 if case == "far" else PUSH_MAX  # (far: the wire buckles at a few N; 40 N blew the chain up)
            push = 2.0 if case == "probe" else float(np.clip(push + PUSH_GAIN * (PUSH_SPEED - vel) * dt, 0.0, cap))
            if push > peak["force_N"]:
                peak = {"force_N": round(push, 2), "crimp_mm": round(crimp_y * 1e3, 3),
                        "latch_open_mm": round(opn * 1e3, 3)}
            hold_housing(-push * axis_in)
            wrench("contact", {push_body: push * axis_in})
            done_half = (case in ("half", "forced") and opn >= HALF_OPEN) or (case == "probe" and t > 1.0)
            stalled = push >= cap  # pushing as hard as the hand may: it doesn't get further
            if h.seated or done_half or stalled:
                events["peak"] = peak
                events["pushed"] = {"t": round(t, 3), "force_N": round(push, 2), "crimp_mm": round(crimp_y * 1e3, 3),
                                    "seated": bool(h.seated), "latch_open_mm": round(opn * 1e3, 3)}
                phase, t_phase = "settle", t
        elif phase == "settle":
            keep = HALF_HOLD if case == "forced" else 0.0
            hold_housing(-keep * axis_in)
            wrench("contact", {"contact": keep * axis_in})
            if t - t_phase >= SETTLE_S:
                events["settled"] = {"t": round(t, 3), "crimp_mm": round(crimp_y * 1e3, 3), "seated": bool(h.seated),
                                     "locked": bool(h.locked), "latch_open_mm": round(opn * 1e3, 3)}
                if case == "forced":
                    events["settled"]["note"] = "the contact pushed back under the finger for the press"
                if case in ("lock", "forced"):
                    lp = h0_p + h0_r @ np.array([0.0, ct.Y_LOCK, ct.H_H / 2 + 1.5e-3])
                    place("lock", lp, h0_r)
                    phase, t_phase = "press", t
                else:
                    ct.HANDS_ON[0] = False
                    wrench("housing", {})
                    phase, t_phase = "tug", t
        elif phase == "press":
            f = LOCK_PRESS[0 if case == "lock" else 1] * min(1.0, (t - t_phase) / 0.5)
            keep = HALF_HOLD if case == "forced" else 0.0  # (forced: the contact kept under the open finger)
            hold_housing(-keep * axis_in)
            wrench("contact", {"contact": keep * axis_in})
            wrench("lock", {"bar": (0.0, 0.0, -f)})
            if t - t_phase >= 2.0:
                events["pressed"] = {"t": round(t, 3), "lock_depth_mm": round(h.lock_depth * 1e3, 3),
                                     "lock_in": bool(h.lock_in), "lock_force_N": round(h.lock_force, 2),
                                     "forced": bool(h.forced), "latch_open_mm": round(opn * 1e3, 3)}
                wrench("lock", {})
                ct.HANDS_ON[0] = False
                wrench("housing", {})
                phase, t_phase = "tug", t
        elif phase == "tug":
            if h.tug in ("passed", "failed") or t - t_phase > WAIT_S:
                phase = "done"
        if case == "probe" or int(round(t / dt)) % LOG_EVERY == 0:
            cpw = ct.body_poses(base, ct._NAMES["contact"])["contact"][0]
            hpw = ct.body_poses(base, ct._NAMES["housing"])["base"][0]
            log.append({"t": round(t, 3), "phase": phase, "crimp_xz_mm": [round(float(geo["crimp"][0]) * 1e3, 3),
                                                                   round(float(geo["crimp"][2]) * 1e3, 3)],
                        "nose_xz_mm": [round(float(geo["nose"][0]) * 1e3, 3), round(float(geo["nose"][2]) * 1e3, 3)],
                        "fingers3_mm": [round(v * 1e3, 3) for v in (ct.joints(base, ct._NAMES["housing"])["lat3o"],
                                                                     ct.joints(base, ct._NAMES["housing"])["lat3i"])],
                        "contact_w": [round(float(v), 4) for v in cpw],
                        "housing_w": [round(float(v), 4) for v in hpw], "push_N": round(push, 2), "crimp_mm": round(crimp_y * 1e3, 3),
                        "latch_open_mm": [round(v * 1e3, 3) for v in geo["latch_open"]], "cavity": h.cavity,
                        "seated": bool(h.seated), "locked": bool(h.locked), "lock_depth_mm": round(h.lock_depth * 1e3, 3),
                        "lock_force_N": round(h.lock_force, 2), "tug": h.tug, "tug_moved_mm": round(h.tug_moved * 1e3, 3),
                        "score": float(task.score(st))})
        emb.step(hold)
        if phase == "done":
            break
    st = object_states(emb._env, emb.task.names)
    h = st["housing"]
    events["end"] = {"t": round(t, 3), "cavity": h.cavity, "crimp_mm": round(h.crimp_depth * 1e3, 3),
                     "max_crimp_mm": round(max_depth * 1e3, 3), "seated": bool(h.seated), "locked": bool(h.locked),
                     "lock_in": bool(h.lock_in), "forced": bool(h.forced), "tug": h.tug,
                     "tug_moved_mm": round(h.tug_moved * 1e3, 3), "score": float(task.score(st))}
    ct.HANDS_ON[0] = False
    return {"case": case, "events": events, "log": log}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--case", default="all", help="a case, several separated by commas, or all: " + ", ".join(CASES))
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--out", type=Path, default=Path("bench_runs"))
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="override a constant: --set PUSH_MAX=30, or a part's with --set ct.LAT_K=2e4")
    args = ap.parse_args(argv)
    for item in args.set:
        name, value = item.split("=", 1)
        if name.startswith("ct."):
            sys.path.insert(0, str(HERE))
            import ct_parts

            target = vars(ct_parts)
        else:
            target = globals()
        name = name.removeprefix("ct.")
        if name not in target:
            ap.error(f"--set: no constant {name!r}")
        try:
            target[name] = ast.literal_eval(value)
        except (ValueError, SyntaxError):
            target[name] = value
    cases = list(CASES) if args.case == "all" else args.case.split(",")
    for c in cases:
        if c not in CASES:
            ap.error(f"no case {c!r}")
    import os

    os.environ["CT_BENCH_NO_REST"] = "1"  # (the contact goes straight into a cavity: its wire would run through the rest)
    from yam_arena.inspect_embodiment import YamArenaEmbodiment

    emb = YamArenaEmbodiment(task=str(HERE / "task.py"), control_hz=240.0, object_jitter=0.0, cam_height=32,
                             cam_width=32, device=args.device)
    args.out.mkdir(parents=True, exist_ok=True)
    for case in cases:
        res = run(emb, case)
        (args.out / f"ct_bench_{case}.json").write_text(json.dumps(res, indent=1, default=float))
        print("BENCH " + json.dumps({"case": case, **res["events"]}, default=float))
        sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
