"""Bracket bench (not a benchmark task): the end bracket's mechanism alone, under exactly the task's physics.

Fixed DIN rails side by side on the station table, each with a bracket seated on it (hooked under the -x lip,
screw backed out), and one free-lying screwdriver. A "virtual screwdriver" turns each screw: an external torque on
the screw, from a torque-limited speed controller (half a turn per second, at most 1 Nm), the way a hand would.
The arms stay in their ready pose.

    python bracket_bench.py [--out bench_runs] [--device cpu]      # about 4 minutes; writes bracket_bench.json

Cases:
- turn: the screw is turned in until it is tight (TIGHT_TURNS with the jaw pressing at least TIGHT_CLAMP): turns to
  contact and to tight, clamp curve, jaw travel.
- slide_loose: an untightened seated bracket is pushed along the rail 10 mm above the lips; breakaway force.
- slide_tight: tightened first, then pushed the same way (ramp to 40 N); breakaway force.
- pull_tight: tightened first, then pulled straight up at its centre (ramp to 60 N): does it come off?
- loosen: tightened, then turned back out a full turn: does the jaw release?
- driver: the screwdriver lying on the table: how it comes to rest.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

TASK_DIR = Path(__file__).resolve().parent  # the task's own files
sys.path.insert(0, str(TASK_DIR))
import eb_parts as ep  # noqa: E402

from yam_arena.task_file import TABLE_TOP  # noqa: E402

ep.install()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 1e-3
CASES = ("turn", "slide_loose", "slide_tight", "pull_tight", "loosen")
RAIL_X = {c: 0.20 + 0.07 * i for i, c in enumerate(CASES)}
RAIL_Y = 0.0
BRACKET_Y = -0.05  # along each rail, clear of its beige block
RAIL_Z = TOP + 0.2 * MM
SEAT_Z = RAIL_Z + ep.RAIL_HEIGHT

OBJECTS = [ep.part(f"rail_{c}", "rail", (RAIL_X[c], RAIL_Y, RAIL_Z), fixed=True) for c in CASES]
OBJECTS += [ep.part(f"bracket_{c}", "bracket", (RAIL_X[c], RAIL_Y + BRACKET_Y, SEAT_Z + 0.1 * MM)) for c in CASES]
# the driver lying on the table, along y, its handle resting on the table
OBJECTS.append(ep.part("driver", "driver", (RAIL_X[CASES[-1]] + 0.08, -0.10, TOP + ep.HANDLE_R + 0.5 * MM),
                       roll=-math.pi / 2))

SETTLE_S = 1.0
TURN_SPEED = 2 * math.pi * 0.5  # rad/s, clockwise seen from above
TURN_KP, TURN_KD, TURN_MAX = 0.3, 0.01, 1.0  # Nm/rad, Nm s/rad, Nm (stable at 240 Hz on the screw's inertia)
PUSH_DOWN = 10.0  # N: how hard the virtual screwdriver presses into the slot
HOLD_S = 0.5  # the stall torque is held this long before the next phase
PUSH_RAMP_S = 4.0
PEAK = {"slide_loose": 10.0, "slide_tight": 40.0, "pull_tight": 60.0}


def run(out: Path, device: str = "cuda:0", max_s: float = 14.0) -> dict:
    import torch
    from inspect_robots import Scene
    from inspect_robots.types import Action

    from yam_arena.inspect_embodiment import YamArenaEmbodiment

    emb = YamArenaEmbodiment(task=__file__, control_hz=240.0, object_jitter=0.0, cam_height=32, cam_width=32,
                             device=device)
    obs = emb.reset(Scene(id="bracket-bench", instruction="bench"), seed=0)
    emb._env.unwrapped.cfg.sim.render_interval = 10**9
    hold = Action(data=np.asarray(obs.state["joint_pos"], dtype=float))
    base = emb._env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    dt = 1.0 / 240.0
    from yam_arena.scene import object_states  # patched: brackets' mechanism states, and the screw friction update

    def state(c):
        art = base.scene.articulations[f"bracket_{c}"]
        p = art.data.root_pose_w.torch[0].detach().cpu().numpy().astype(np.float64)
        q = art.data.joint_pos.torch[0].detach().cpu().numpy().astype(np.float64)
        qd = art.data.joint_vel.torch[0].detach().cpu().numpy().astype(np.float64)
        names = list(art.joint_names)
        mech = ep.mechanism(q, names)
        mech["screw_vel"] = float(qd[names.index("screw")])
        mech["carrier"] = float(q[names.index("carrier")])
        rot0 = ep._rot(p[3:])
        bp = art.data.body_pos_w.torch[0].detach().cpu().numpy().astype(np.float64)
        bnames = list(art.body_names)
        for link in ("jaw", "carrier", "screw"):  # link origins in the bracket's frame, mm
            mech[f"{link}_local_mm"] = [round(v / MM, 4) for v in rot0.T @ (bp[bnames.index(link)] - p[:3])]
        return p[:3] - origin, rot0, mech

    def wrench(c, body, force=(0, 0, 0), torque=(0, 0, 0), point=None):
        art = base.scene.articulations[f"bracket_{c}"]
        bid = art.find_bodies(body)[0][0]
        f = None if not any(force) else torch.tensor([[list(force)]], dtype=torch.float32, device=base.device)
        t = None if not any(torque) else torch.tensor([[list(torque)]], dtype=torch.float32, device=base.device)
        if f is None and t is None:
            return
        kw = {}
        if point is not None:
            kw["positions"] = torch.tensor([[list(point)]], dtype=torch.float32, device=base.device)
        art.permanent_wrench_composer.add_forces_and_torques_index(forces=f, torques=t, body_ids=[bid],
                                                                   is_global=True, **kw)

    start = {c: state(c) for c in CASES}
    phase = {c: "settle" for c in CASES}
    t_phase = {c: 0.0 for c in CASES}
    target = {c: 0.0 for c in CASES}
    log = {c: [] for c in CASES}
    driver_log = []
    events = {c: {} for c in CASES}
    n_steps = round(max_s / dt)
    for step in range(n_steps):
        t = step * dt
        object_states(emb._env, emb.task.names)  # updates every screw's friction from its clamp
        for c in CASES:
            art = base.scene.articulations[f"bracket_{c}"]
            art.permanent_wrench_composer.reset()
            pos, rot, m = state(c)
            rel = rot.T @ (pos - start[c][0])  # in the bracket's own (start) frame, roughly
            row = {"t": round(t, 4), "phase": phase[c], "dx_mm": (pos - start[c][0])[0] / MM,
                   "dy_mm": (pos - start[c][0])[1] / MM, "dz_mm": (pos - start[c][0])[2] / MM,
                   "tilt_deg": math.degrees(math.acos(min(1.0, max(-1.0, rot[2, 2])))),
                   "roll_deg": math.degrees(math.atan2(rot[2, 0], rot[2, 2])),  # about the rail's axis: + lifts -x
                   "jaw_top_mm": (start[c][1].T @ (pos - start[c][0]))[2] / MM - ep.JAW_GAP / MM - ep.LIP_THICK / MM
                   + m["jaw_travel"] * math.sin(ep.JAW_ANGLE) / MM,  # rough: the jaw's top vs the lip underside
                   "turns": m["screw_turns"], "carrier_mm": m["carrier"] / MM, "jaw_mm": m["jaw_travel"] / MM,
                   "comp_mm": m["jaw_compression"] / MM, "jaw_local_mm": m["jaw_local_mm"],
                   "carrier_local_mm": m["carrier_local_mm"], "screw_local_mm": m["screw_local_mm"], "clamp_N": m["clamp_force"]}
            del rel
            # the phases
            if phase[c] == "settle" and t >= SETTLE_S:
                phase[c] = "turn" if c != "slide_loose" else "push"
                t_phase[c] = t
                target[c] = -2 * math.pi * m["screw_turns"]
            applied = 0.0
            if phase[c] in ("turn", "unturn"):
                sign = -1.0 if phase[c] == "turn" else 1.0
                angle = -2 * math.pi * m["screw_turns"]
                target[c] += sign * TURN_SPEED * dt
                # don't wind the target up past where the screw is (a hand doesn't either)
                target[c] = angle + sign * min(abs(target[c] - angle), TURN_MAX / TURN_KP)
                applied = TURN_KP * (target[c] - angle) - TURN_KD * m["screw_vel"]
                applied = max(-TURN_MAX, min(TURN_MAX, applied))
                # tight: turned TIGHT_TURNS with the jaw pressing (the simulation's stand-in for 0.5 Nm); stop there
                stalled = phase[c] == "turn" and m["screw_turns"] >= ep.TIGHT_TURNS and m["clamp_force"] >= ep.TIGHT_CLAMP
                if stalled and "stall_t" not in events[c]:
                    events[c]["stall_t"] = t
                    events[c]["stall_turns"] = m["screw_turns"]
                    events[c]["stall_clamp_N"] = m["clamp_force"]
                if phase[c] == "turn" and "stall_t" in events[c]:
                    applied = 0.0  # let go: the screw's friction holds it
                    if t >= events[c]["stall_t"] + HOLD_S:
                        events[c]["tight"] = {k: row[k] for k in ("turns", "comp_mm", "clamp_N", "dz_mm", "tilt_deg")}
                        if c == "turn":
                            phase[c] = "done"
                        elif c == "loosen":
                            phase[c], t_phase[c] = "unturn", t
                            target[c] = angle
                        else:
                            phase[c], t_phase[c] = "push", t
                if phase[c] == "unturn" and m["screw_turns"] <= events[c]["stall_turns"] - 1.0:
                    phase[c] = "done"
                    events[c]["loosened"] = {k: row[k] for k in ("turns", "comp_mm", "clamp_N", "jaw_mm")}
                # a hand presses the driver into the slot while turning: PUSH_DOWN on the screw along its axis
                wrench(c, "screw", force=tuple(-PUSH_DOWN * rot[:, 2]), torque=tuple(applied * rot[:, 2]))
            row["torque_Nm"] = applied
            if phase[c] == "push":
                ramp = min((t - t_phase[c]) / PUSH_RAMP_S, 1.0)
                f = PEAK[c] * ramp
                centre = pos + origin
                if c.startswith("slide"):
                    # along the rail (+y, toward the beige block), 10 mm above the lips, at the face's middle
                    point = centre + rot @ np.array([0.0, -ep.BRACKET_WIDTH / 2, 10 * MM])
                    wrench(c, "body", force=(0.0, f, 0.0), point=point)
                else:
                    point = centre + rot @ np.array([0.0, 0.0, 20 * MM])
                    wrench(c, "body", force=(0.0, 0.0, f), point=point)
                row["force_N"] = f
                if ramp >= 1.0 and t >= t_phase[c] + PUSH_RAMP_S + 0.5:
                    phase[c] = "done"
            log[c].append(row)
        dpos = base.scene.rigid_objects["driver"].data.root_pose_w.torch[0].detach().cpu().numpy()
        driver_log.append({"t": round(t, 4), "z_mm": (dpos[2] - origin[2] - TOP) / MM})
        emb.step(hold)
        if all(p == "done" for p in phase.values()):
            break

    def first(rows, cond):
        for r in rows:
            if cond(r):
                return r
        return None

    summary = {"constants": {"turns_to_contact_geometry": ep.TURNS_TO_CONTACT, "jaw_spring_N_per_mm": ep.JAW_SPRING / 1000,
                             "tight_turns": ep.TIGHT_TURNS, "tight_clamp_N": ep.TIGHT_CLAMP, "screw_friction_Nm": ep.FREE_TORQUE}}
    rows = log["turn"]
    contact = first(rows, lambda r: r["comp_mm"] > 0.01)
    summary["turn"] = {
        "settled": {k: round(rows[min(round(SETTLE_S / dt), len(rows)) - 1][k], 4) for k in ("dz_mm", "tilt_deg", "turns", "jaw_mm")},
        "contact_turns": None if contact is None else round(contact["turns"], 3),
        "events": events["turn"],
        "torque_curve": [{k: round(r[k], 4) for k in ("t", "turns", "comp_mm", "clamp_N", "torque_Nm")}
                         for r in rows[:: 24]],
        "max_jaw_mm": round(max(r["jaw_mm"] for r in rows), 3),
        "max_tilt_deg": round(max(r["tilt_deg"] for r in rows), 3),
        "final": {k: round(rows[-1][k], 4) for k in ("dz_mm", "tilt_deg", "turns", "comp_mm", "clamp_N")},
    }
    for c in ("slide_loose", "slide_tight", "pull_tight"):
        rows = [r for r in log[c] if r["phase"] == "push"]
        key = "dy_mm" if c.startswith("slide") else "dz_mm"
        base_val = rows[0][key] if rows else 0.0
        hit = first(rows, lambda r: abs(r[key] - base_val) > (0.3 if c.startswith("slide") else 1.0))
        summary[c] = {"events": events[c], "breakaway_N": None if hit is None else round(hit["force_N"], 2),
                      "max_force_N": round(max((r.get("force_N", 0) for r in rows), default=0.0), 2),
                      "moved_mm": round(rows[-1][key] - base_val, 3) if rows else None,
                      "final": {k: round(log[c][-1][k], 4) for k in ("dy_mm", "dz_mm", "tilt_deg", "comp_mm", "clamp_N")}}
    summary["loosen"] = {"events": events["loosen"]}
    summary["driver"] = {"final_z_mm": round(driver_log[-1]["z_mm"], 3)}
    summary["sim_s"] = round(t, 3)
    out.mkdir(parents=True, exist_ok=True)
    (out / "bracket_bench.json").write_text(json.dumps({"summary": summary, "log": log}, indent=1, default=float))
    return summary


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="override an eb_parts constant for an experiment")
    ap.add_argument("--device", default="cuda:0")
    ap.add_argument("--max-s", type=float, default=14.0)
    ap.add_argument("--out", type=Path, default=Path("bench_runs"))
    args = ap.parse_args(argv)
    import ast
    for item in args.set:
        name, value = item.split("=", 1)
        if not hasattr(ep, name):
            ap.error(f"--set: eb_parts has no constant {name!r}")
        setattr(ep, name, ast.literal_eval(value))
    summary = run(args.out, args.device, args.max_s)
    print("BENCH " + json.dumps({k: v for k, v in summary.items() if k != "turn"} | {
        "turn": {k: v for k, v in summary["turn"].items() if k != "torque_curve"}}, default=float))
    sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
