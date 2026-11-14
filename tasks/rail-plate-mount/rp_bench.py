"""Thread bench (not a benchmark task): the screws' thread mechanism alone, under the task's physics (240 Hz, CPU).

Three mounting plates side by side on the station table; on them screws standing in holes, next to a hole, or lying.
A "virtual screwdriver" turns a screw: a torque about its axis from a torque-limited speed controller (1 turn per
second clockwise, at most 0.5 Nm) while pressing it down with 10 N, the way a hand would; until it catches, a loose
screw is also held upright (a restoring torque, as fingers hold it). The arms stay in their
ready pose.

    python rp_bench.py [--out bench_runs] [--device cpu]     # writes rp_bench.json, prints BENCH {...}

Cases (one screw each):
- a_rail: through a rail slot into hole 0 of plate 0; turned until tight. The rail is free and not held: how far
  does it turn and slide while the first screw is tightened (the reason a second hand holds it)?
- b_rail: through the rail's other slot into hole 1 of plate 0 (pinning the rail); tightened after a_rail. Then the
  rail is pushed along its length (ramp to 20 N): does the clamp hold it?
- c_plain: straight into hole 0 of plate 1 (no rail): catch, turns to the head's contact, tight.
- d_off: standing on plate 1, 15 mm beside hole 0, and turned for 3 s: must not catch.
- e_loosen: into hole 1 of plate 2, tightened, then turned back out until it is loose again; then pulled up gently:
  does it come out?
- f_lying: dropped lying on plate 2: does it rest quietly?
- g_drag: through a slot of a second rail into hole 1 of plate 1, no other screw: tightened. How far does the
  unheld rail turn with it?
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

TASK_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TASK_DIR))
import rp_parts as rp  # noqa: E402

from yam_arena.task_file import TABLE_TOP  # noqa: E402

rp.install()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 1e-3
PLATE_X = 0.35
PLATE_YS = (-0.45, -0.12, 0.21)
PLATE_Z = TOP + rp.PLATE_FLANGE  # the plates' top surface
SEATED_TIP = PLATE_Z - rp.DOG_LEN + 0.2 * MM  # a screw standing in a hole, dropped from 0.2 mm


def hole(p, i):
    return (PLATE_X, PLATE_YS[p] + rp.HOLE_Y[i])


OBJECTS = [rp.part(f"plate_{i}", "plate", (PLATE_X, y, PLATE_Z), fixed=True) for i, y in enumerate(PLATE_YS)]
# plate 0: the rail lined up (shifted 12.5 mm along so slots 0 and 8 sit over the holes), web-down on the plate
RAIL_FIXED = bool(os.environ.get("RP_RAIL_FIXED"))  # experiment: the first rail as a fixed collider
OBJECTS.append(rp.part("rail", "rail", (PLATE_X, PLATE_YS[0] + 12.5 * MM, PLATE_Z + 0.1 * MM), fixed=RAIL_FIXED))
# plate 1: a second rail over hole 1 only (its middle slot there; it overhangs the plate's end), held by nothing
OBJECTS.append(rp.part("rail_2", "rail", (PLATE_X, hole(1, 1)[1] + 12.5 * MM, PLATE_Z + 0.1 * MM)))
SCREWS = {
    "a_rail": (*hole(0, 0), SEATED_TIP + 0.1 * MM),
    "b_rail": (*hole(0, 1), SEATED_TIP + 0.1 * MM),
    "c_plain": (*hole(1, 0), SEATED_TIP),
    "d_off": (hole(1, 0)[0] + 15 * MM, hole(1, 0)[1], PLATE_Z + 0.2 * MM),
    "e_loosen": (*hole(2, 1), SEATED_TIP),
    "g_drag": (*hole(1, 1), SEATED_TIP + 0.1 * MM),
}
for k, (name, pos) in enumerate(SCREWS.items()):
    OBJECTS += rp.screw_parts(name, pos, park_y=-0.3 + 0.1 * k)
# lying on its side (axis along y), dropped from 0.3 mm above the head's rim
OBJECTS += rp.screw_parts("f_lying", (PLATE_X + 40 * MM, hole(2, 0)[1] + 30 * MM, PLATE_Z + rp.HEAD_R + 0.3 * MM),
                          park_y=-0.3 + 0.1 * len(SCREWS), roll=-math.pi / 2)
NAMES = [o.name for o in OBJECTS]

SETTLE_S = 0.5
TURN_SPEED = 2 * math.pi * 1.0  # rad/s
TURN_MAX = 0.5  # Nm
KD_TWIN, KP_TWIN = 0.01, 0.3  # Nm s/rad, Nm/rad: on the twin (armature 2e-4 kg m2), as in end-bracket-mount's bench
LOOSE_MAX = 0.02  # Nm on a loose screw: enough to overcome the shank's friction on the hole's edge (~0.006 Nm)
KP_LOOSE, KD_LOOSE = 0.01, 1.5e-4  # Nm/rad, Nm s/rad: on the loose screw's 5e-7 kg m2 (about critically damped)
PUSH_DOWN = 10.0  # N
START_PUSH = 1.0  # N: pressing a loose screw while starting it (10 N on its 5.5 mm high centre of mass outweighs any
# upright torque that is stable at 240 Hz on its 5e-7 kg m2)
KP_UP, KD_UP = 0.02, 2e-4  # Nm/rad, Nm s/rad: the "fingers" keeping a loose screw upright while it is started
HOLD_S = 0.5
OFF_TURN_S = 3.0
PUSH_RAMP_S = 4.0
RAIL_PUSH = 60.0  # N
LIFT = 0.0005  # N over its weight (lifts it ~10 cm in 1 s if it is free)


def _tip(base, name) -> float:
    """A loose screw's origin (its tip) height, env frame."""
    pose = rp._np(base.scene.rigid_objects[name].data.root_pose_w)
    return float(pose[2] - base.scene.env_origins[0, 2].item())


def run(out: Path, device: str = "cpu", max_s: float = 40.0) -> dict:
    import torch
    from inspect_robots import Scene
    from inspect_robots.types import Action

    from yam_arena.inspect_embodiment import YamArenaEmbodiment

    emb = YamArenaEmbodiment(task=__file__, control_hz=240.0, object_jitter=0.0, cam_height=32, cam_width=32,
                             device=device)
    obs = emb.reset(Scene(id="thread-bench", instruction="bench"), seed=0)
    emb._env.unwrapped.cfg.sim.render_interval = 10**9
    hold = Action(data=np.asarray(obs.state["joint_pos"], dtype=float))
    base = emb._env.unwrapped
    dt = 1.0 / 240.0
    from yam_arena.scene import object_states  # patched: runs the thread swap

    def wrench(name, force, torque):
        """Force and torque (world frame) on whichever body of a screw is live."""
        if rp._TRACK.get(name) and rp._TRACK[name].engaged:
            art = base.scene.articulations[rp.twin_name(name)]
            body = art.find_bodies("screw")[0][0]
            comp = art.permanent_wrench_composer
        else:
            obj = base.scene.rigid_objects[name]
            body, comp = 0, obj.permanent_wrench_composer
        f = torch.tensor([[list(force)]], dtype=torch.float32, device=base.device)
        t = torch.tensor([[list(torque)]], dtype=torch.float32, device=base.device)
        comp.add_forces_and_torques_index(forces=f, torques=t, body_ids=[body], is_global=True)

    def reset_wrenches():
        for name in [*SCREWS, "f_lying"]:
            base.scene.rigid_objects[name].permanent_wrench_composer.reset()
            base.scene.articulations[rp.twin_name(name)].permanent_wrench_composer.reset()
        if not RAIL_FIXED:
            base.scene.rigid_objects["rail"].permanent_wrench_composer.reset()
        base.scene.rigid_objects["rail_2"].permanent_wrench_composer.reset()

    phase = {"a_rail": "settle", "b_rail": "wait", "c_plain": "settle", "d_off": "settle", "e_loosen": "settle",
             "g_drag": "settle"}
    t_phase = dict.fromkeys(phase, 0.0)
    target = dict.fromkeys(phase, 0.0)
    events = {n: {} for n in [*phase, "f_lying", "rail", "rail_2"]}
    log = {n: [] for n in [*phase, "f_lying", "rail", "rail_2"]}
    loose = {}  # angle tracking for loose screws being turned
    rail_phase, rail_t = "wait", 0.0
    rail0 = None
    n_steps = round(max_s / dt)
    t = 0.0
    for step in range(n_steps):
        t = step * dt
        states = object_states(emb._env, NAMES)
        reset_wrenches()
        if rail0 is None:
            rail0 = {}
        for rn in ("rail", "rail_2"):
            rail = states[rn]
            rail_rot = rp._rot(rail.quat_xyzw)
            if rn not in rail0:
                rail0[rn] = (np.asarray(rail.pos, dtype=float).copy(), math.atan2(rail_rot[1, 0], rail_rot[0, 0]))
            r0 = rail0[rn]
            row_r = {"t": round(t, 4), "dx_mm": (rail.pos[0] - r0[0][0]) / MM, "dy_mm": (rail.pos[1] - r0[0][1]) / MM,
                     "dz_mm": (rail.pos[2] - r0[0][2]) / MM,
                     "yaw_deg": math.degrees(math.atan2(rail_rot[1, 0], rail_rot[0, 0]) - r0[1])}
            log[rn].append(row_r)
            if rn == "rail":
                rail_row = row_r
            else:
                rail2_row = row_r
        for name in [*phase, "f_lying"]:
            st = states[name]
            tr = rp._TRACK.get(name)
            rot = rp._rot(st.quat_xyzw)
            row = {"t": round(t, 4), "phase": phase.get(name, "rest"), "engaged": bool(st.engaged),
                   "turns": st.turns, "clamp_N": st.clamp_force, "z_mm": (st.low[2] - PLATE_Z) / MM,
                   "tilt_deg": math.degrees(math.acos(min(1.0, max(-1.0, rot[2, 2])))),
                   "wz": float(st.ang_vel[2]), "swaps": tr.swaps if tr else 0}
            if st.engaged:  # the tip's height (the screw's origin) above the plate's top
                row["tip_mm"] = (rp._twin_screw_pose(base, rp.twin_name(name))[0][2] - PLATE_Z) / MM
            else:
                row["tip_mm"] = (_tip(base, name) - PLATE_Z) / MM
            log[name].append(row)
            if name == "f_lying":
                continue
            ev = events[name]
            if st.engaged and "caught_t" not in ev:
                ev["caught_t"] = round(t, 4)
            if st.engaged and st.clamp_force >= rp.CONTACT_CLAMP and "contact_turns" not in ev:
                ev["contact_turns"] = round(st.turns, 3)
            p = phase[name]
            if p == "settle" and t >= SETTLE_S:
                phase[name], t_phase[name] = "turn", t
                p = "turn"
            if p == "turn" and name == "d_off" and t >= t_phase[name] + OFF_TURN_S:
                phase[name] = p = "done"
                ev["caught"] = bool(st.engaged)
            if p in ("turn", "unturn"):
                sign = -1.0 if p == "turn" else 1.0
                tight = st.tight()
                if p == "turn" and tight and "tight_t" not in ev:
                    ev.update(tight_t=round(t, 4), tight_turns=round(st.turns, 3), tight_clamp_N=round(st.clamp_force, 1),
                              rail_at_tight=dict(rail_row) if name.endswith("_rail") else
                              dict(rail2_row) if name == "g_drag" else None)
                if p == "turn" and "tight_t" in ev:
                    torque = 0.0
                    if t >= ev["tight_t"] + HOLD_S:
                        ev["after_hold"] = {"turns": round(st.turns, 3), "clamp_N": round(st.clamp_force, 1),
                                            "past_contact": round(st.past_contact, 3), "tight": st.tight()}
                        if name == "a_rail":
                            phase[name], phase["b_rail"], t_phase["b_rail"] = "done", "turn", t
                        elif name == "b_rail":
                            phase[name], rail_phase, rail_t = "done", "push", t
                        elif name == "e_loosen":
                            phase[name], t_phase[name] = "unturn", t
                        else:
                            phase[name] = "done"
                elif st.engaged:
                    angle = -2 * math.pi * st.turns
                    target[name] += sign * TURN_SPEED * dt
                    target[name] = angle + sign * min(abs(target[name] - angle), TURN_MAX / KP_TWIN)
                    torque = KP_TWIN * (target[name] - angle) - KD_TWIN * st.ang_vel[2]
                    torque = max(-TURN_MAX, min(TURN_MAX, torque))
                else:  # loose: follow an angle ramp about its axis (its yaw, unwrapped)
                    yaw = math.atan2(rot[1, 0], rot[0, 0])
                    lt = loose.setdefault(name, {"angle": yaw, "last": yaw, "target": yaw})
                    lt["angle"] += (yaw - lt["last"] + math.pi) % (2 * math.pi) - math.pi
                    lt["last"] = yaw
                    lt["target"] += sign * TURN_SPEED * dt
                    lt["target"] = lt["angle"] + sign * min(abs(lt["target"] - lt["angle"]), LOOSE_MAX / KP_LOOSE)
                    torque = KP_LOOSE * (lt["target"] - lt["angle"]) - KD_LOOSE * st.ang_vel[2]
                    torque = max(-LOOSE_MAX, min(LOOSE_MAX, torque))
                    target[name] = 0.0  # the twin starts at 0 turns when it catches
                if st.engaged:
                    loose.pop(name, None)
                if p == "unturn" and not st.engaged:
                    ev.setdefault("released_t", round(t, 4))
                    if t >= ev["released_t"] + 0.3:
                        phase[name], t_phase[name] = "lift", t
                        ev["z_at_release_mm"] = round(row["z_mm"], 3)
                axis = rot[:, 2]
                if st.engaged:
                    wrench(name, tuple(-PUSH_DOWN * axis) if p == "turn" else (0.0, 0.0, 0.0), tuple(torque * axis))
                else:
                    # a loose screw is held upright, as fingers hold it: pressed straight down lightly, and a torque
                    # that turns its axis back to vertical (pressed with 10 N along its own axis, the 5 mm tall socket
                    # head screw toppled 20-50 degrees before it caught, bench runs 11-12)
                    w = np.asarray(st.ang_vel, dtype=float)
                    up = KP_UP * np.cross(axis, (0.0, 0.0, 1.0)) - KD_UP * (w - np.dot(w, axis) * axis)
                    wrench(name, (0.0, 0.0, -START_PUSH) if p == "turn" else (0.0, 0.0, 0.0),
                           tuple(torque * axis + up))
                row["torque"] = torque
            if p == "lift":
                wrench(name, (0.0, 0.0, LIFT + rp.SCREW_MASS * 9.81), (0.0, 0.0, 0.0))
                if t >= t_phase[name] + 1.0:
                    phase[name] = "done"
                    ev["lifted_mm"] = round(row["z_mm"] - ev["z_at_release_mm"], 3)
        if rail_phase == "push" and RAIL_FIXED:
            rail_phase = "done"
        if rail_phase == "push":
            f = RAIL_PUSH * min((t - rail_t) / PUSH_RAMP_S, 1.0)
            obj = base.scene.rigid_objects["rail"]
            obj.permanent_wrench_composer.add_forces_and_torques_index(
                forces=torch.tensor([[[0.0, f, 0.0]]], dtype=torch.float32, device=base.device),
                torques=torch.tensor([[[0.0, 0.0, 0.0]]], dtype=torch.float32, device=base.device),
                body_ids=[0], is_global=True)
            rail_row["force_N"] = f
            if "push_start" not in events["rail"]:
                events["rail"]["push_start"] = dict(rail_row)
            p0 = events["rail"]["push_start"]
            if "breakaway_N" not in events["rail"] and math.hypot(rail_row["dx_mm"] - p0["dx_mm"],
                                                                  rail_row["dy_mm"] - p0["dy_mm"]) > 0.3:
                events["rail"]["breakaway_N"] = round(f, 1)
            if t >= rail_t + PUSH_RAMP_S + 0.5:
                rail_phase = "done"
                events["rail"]["push_end"] = dict(rail_row)
        emb.step(hold)
        if all(p == "done" for p in phase.values()) and rail_phase == "done" and t > 3.0:
            break

    def rng(rows, key):
        vals = [r[key] for r in rows if r["t"] >= SETTLE_S]
        return [round(min(vals), 4), round(max(vals), 4)] if vals else None

    summary = {"constants": {"pitch_mm": rp.PITCH / MM, "tight_clamp_N": rp.TIGHT_CLAMP,
                             "clamp_spring_N_per_mm": rp.CLAMP_SPRING / 1000, "free_torque_Nm": rp.FREE_TORQUE}}
    for name in phase:
        rows = log[name]
        summary[name] = {"events": events[name], "final_phase": phase[name],
                         "final": {k: rows[-1][k] for k in ("engaged", "turns", "clamp_N", "tip_mm", "tilt_deg", "swaps")},
                         "max_tilt_deg": round(max(r["tilt_deg"] for r in rows), 3)}
    summary["f_lying"] = {"z_range_mm": rng(log["f_lying"], "z_mm"), "tilt_range_deg": rng(log["f_lying"], "tilt_deg"),
                          "final": log["f_lying"][-1]}
    summary["rail"] = {"events": events["rail"], "final": log["rail"][-1],
                       "max_abs_yaw_deg": round(max(abs(r["yaw_deg"]) for r in log["rail"]), 3)}
    summary["rail_2"] = {"final": log["rail_2"][-1],
                         "max_abs_yaw_deg": round(max(abs(r["yaw_deg"]) for r in log["rail_2"]), 3)}
    for name in phase:  # still tight at the end?
        summary[name]["final"]["tight"] = bool(states[name].tight())
        summary[name]["final"]["past_contact"] = round(states[name].past_contact, 3)
    summary["sim_s"] = round(t, 3)
    out.mkdir(parents=True, exist_ok=True)
    (out / "rp_bench.json").write_text(json.dumps({"summary": summary, "log": log}, indent=1, default=float))
    return summary


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="override an rp_parts constant for an experiment")
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--max-s", type=float, default=40.0)
    ap.add_argument("--out", type=Path, default=Path("bench_runs"))
    args = ap.parse_args(argv)
    import ast
    for item in args.set:
        name, value = item.split("=", 1)
        if not hasattr(rp, name):
            ap.error(f"--set: rp_parts has no constant {name!r}")
        setattr(rp, name, ast.literal_eval(value))
    rp.MASSES.update(rail=rp.RAIL_MASS, screw=rp.SCREW_MASS, driver=rp.DRIVER_MASS)  # (read when the USD is written)
    summary = run(args.out, args.device, args.max_s)
    print("BENCH " + json.dumps(summary, default=float))
    sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
