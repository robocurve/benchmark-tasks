"""Clamp bench (not a benchmark task): the cord clamp alone, in the task's own scene, under its physics (240 Hz,
CPU), the arms parked. A "virtual hand" pushes the cable in (a force on the cable outside the box, inward, until the
sheath's cut end is SHEATH_TARGET past the bar's middle) and holds it, then turns the screws in turns: a torque about
each screw's axis from a torque-limited speed controller (a quarter turn per second clockwise, at most 1.5 Nm), each
time to the next step of the case's plan; then lets go, and the task's own pull test follows.

    python cc_bench.py --case snug [--out bench_runs]        # writes cc_bench_<case>.json, prints BENCH {...}
    python cc_bench.py --case all

Cases (screw targets in turns past each head's first contact; the screws turned half a turn at a time in
alternation, from the start, each until it has reached its target):
- snug: both to 0.5 (inside the 0.25-1.75 window, in turns, the bar parallel);
- uneven: s0 to 0.35, s1 to 1.2 (both snug, but their turns past contact far apart: not tightened in turns);
- loose: both to 0.12 (short of snug: does it hold?);
- crush: both to 2.1 (crushed);
- cores: no push (the cores under the bar), both to 0.5 (the cores clamped);
- let_go / let_go_held: pushed in, held 0.5 s / 20 s, let go with the screws untouched: where the sheath's end settles
  (it must spring back to under 5 mm past the bar: the cable must be held while the screws are tightened).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
CASES = {"snug": (True, (0.5, 0.5)), "uneven": (True, (0.35, 1.2)), "loose": (True, (0.12, 0.12)),
         "crush": (True, (2.1, 2.1)), "cores": (False, (0.5, 0.5)), "let_go": (True, None), "let_go_held": (True, None)}
LET_GO_HOLD_S = {"let_go": 0.5, "let_go_held": 20.0}  # pushed in, held this long, let go with the screws loose
LET_GO_WAIT_S = 10.0  # then watched this long (where does the sheath's end settle?)
SHEATH_TARGET = 18e-3  # the sheath's end this far past the bar's middle (the goal is 9 mm)
PUSH_N, HOLD_N = 8.0, 3.0  # N: pushing, then holding it in (the cores' bow pushes it back: at 0.5 N it slid back, bench 22)
TURN_SPEED = 2 * math.pi / 4
INC = 0.5  # turns per go on each screw
NUT_TORQUE, FEED, KD = 1.5, 0.05, 0.95  # stalled: up to 1.5 Nm, a hand on a short hex key (PhysX joint friction grows
# with the joint's load, as a thread's does: stalled at ~0.1 Nm the screws stopped ~0.4 turn past contact, bench 10;
# at 0.44 Nm ~0.6, bench 11)
WAIT_S = 4.0
BRAKE_S, BRAKE_KD = 0.5, 0.5
STALL_S = 4.0  # no progress on either screw this long: let go (the targets out of the hand's reach)


def run(emb, case: str) -> dict:
    import torch
    from inspect_robots import Scene
    from inspect_robots.types import Action

    push, targets = CASES[case]
    obs = emb.reset(Scene(id=f"clamp-bench-{case}", instruction="bench"), seed=0)
    emb._env.unwrapped.cfg.sim.render_interval = 10**9
    hold = Action(data=np.asarray(obs.state["joint_pos"], dtype=float))
    base = emb._env.unwrapped
    task = emb.task.module
    cc = task.cc
    from yam_arena.scene import object_states

    def tensor(rows):
        return torch.tensor(rows, dtype=torch.float32, device=base.device)

    cable = base.scene.articulations[cc._NAMES["cable"]]
    clamp = base.scene.articulations[cc._NAMES["clamp"]]
    push_body = list(cable.body_names).index("link2")  # level, just outside the entry
    _, R = cc.clamp_frame()

    def cable_force(f):
        n = len(cable.body_names)
        forces = torch.zeros((1, n, 3), dtype=torch.float32, device=base.device)
        forces[0, push_body] = tensor((R @ [0.0, f, 0.0]).tolist())
        cable.permanent_wrench_composer.set_forces_and_torques_index(forces=forces, torques=torch.zeros_like(forces),
                                                                     body_ids=list(range(n)), is_global=True)

    def screw_torque(s, tau, others=None):
        """Torque about z on screw ``s`` (and on the screws in ``others``, {name: tau}); zero on the rest."""
        n = len(clamp.body_names)
        torques = torch.zeros((1, n, 3), dtype=torch.float32, device=base.device)
        torques[0, list(clamp.body_names).index(s), 2] = tau
        for o, t_o in (others or {}).items():
            torques[0, list(clamp.body_names).index(o), 2] = t_o
        clamp.permanent_wrench_composer.set_forces_and_torques_index(forces=torch.zeros_like(torques),
                                                                     torques=torques, body_ids=list(range(n)),
                                                                     is_global=True)

    # tightening in turns: INC turns on one screw, then the other, from the start, until each has reached its target
    # past its head's contact
    cur, turn_from = 0, None
    best_total, best_t = -1.0, 0.0
    turn_best, turn_t = 0.0, None
    cc.HANDS_ON[0] = True
    phase, k, t_phase = ("push" if push else "turn"), 0, 0.0
    log, events = [], {}
    dt = 1.0 / 240.0
    t = 0.0
    while t < 60.0:
        st = object_states(emb._env, emb.task.names)
        t = float(base.episode_length_buf[0]) * dt
        k_st, c_st, b_st = st["clamp"], st["cable"], st["bar"]
        if phase == "push":
            if c_st.sheath_y >= SHEATH_TARGET:
                events["pushed"] = {"t": round(t, 3), "sheath_y_mm": round(c_st.sheath_y * 1000, 2)}
                phase = "turn" if targets is not None else "hold"
                t_phase = t
            cable_force(PUSH_N if phase == "push" else HOLD_N)
        elif phase == "turn":
            reached = [k_st.contact_turns[i] is not None and k_st.past_contact[i] >= targets[i] for i in range(2)]
            total = sum(k_st.turns)
            if total > best_total + 0.01:
                best_total, best_t = total, t
            stalled = t - best_t > STALL_S  # neither screw turns any more: as far as the hand gets them
            if stalled:
                events["stalled"] = {"t": round(t, 3), "past": [round(v, 3) for v in k_st.past_contact]}
            if all(reached) or stalled:
                phase, t_phase = "brake", t
            else:
                if turn_from is None:
                    turn_from = k_st.turns[cur]
                if turn_t is None or k_st.turns[cur] > turn_best + 0.01:
                    turn_best, turn_t = k_st.turns[cur], t
                if reached[cur] or k_st.turns[cur] - turn_from >= INC or t - turn_t > STALL_S / 2:
                    screw_torque(cc.SCREWS[cur], 0.0)
                    if not reached[1 - cur]:
                        cur = 1 - cur
                    turn_from = k_st.turns[cur]
                    turn_t = None
                    events.setdefault("steps", []).append({"t": round(t, 3), "next": cc.SCREWS[cur],
                                                           "past": [round(v, 3) for v in k_st.past_contact],
                                                           "tilt_deg": round(b_st.tilt_deg, 2)})
                s = cc.SCREWS[cur]
                w = float(cc._np(clamp.data.joint_vel)[list(clamp.joint_names).index(f"{s}_spin")])
                tau = -float(np.clip(FEED + KD * (TURN_SPEED + w), 0.0, NUT_TORQUE))  # clockwise: negative about z
                screw_torque(s, tau)
            if push and phase == "turn":
                cable_force(HOLD_N)
        elif phase == "brake":  # the hand stops both screws before letting go (released while turning, a screw
            # coasted on by up to 0.35 turn, bench run 15)
            qd = cc._np(clamp.data.joint_vel)
            brake = {sc: float(np.clip(-BRAKE_KD * float(qd[list(clamp.joint_names).index(f"{sc}_spin")]),
                                       -NUT_TORQUE, NUT_TORQUE)) for sc in cc.SCREWS}
            screw_torque(cc.SCREWS[0], brake[cc.SCREWS[0]], {cc.SCREWS[1]: brake[cc.SCREWS[1]]})
            if push:
                cable_force(HOLD_N)
            if t - t_phase >= BRAKE_S:
                screw_torque(cc.SCREWS[0], 0.0)
                phase, t_phase = "released", t
                cable_force(0.0)
                cc.HANDS_ON[0] = False
                events["released"] = {"t": round(t, 3), "past": [round(v, 3) for v in k_st.past_contact],
                                      "turns": [round(v, 3) for v in k_st.turns],
                                      "force_N": [round(v, 1) for v in k_st.clamp_force],
                                      "tilt_deg": round(b_st.tilt_deg, 2), "sheath_y_mm": round(c_st.sheath_y * 1000, 2)}
        elif phase == "hold":
            cable_force(HOLD_N)
            if t - t_phase >= LET_GO_HOLD_S[case]:
                cable_force(0.0)
                cc.HANDS_ON[0] = False
                phase, t_phase = "let_go", t
                events["let_go"] = {"t": round(t, 3), "sheath_y_mm": round(c_st.sheath_y * 1000, 2)}
        elif phase == "let_go":
            if t - t_phase >= LET_GO_WAIT_S:
                events["settled"] = {"t": round(t, 3), "sheath_y_mm": round(c_st.sheath_y * 1000, 2),
                                     "past_goal": bool(task.sheath_past(st)), "score": float(task.score(st))}
                phase = "done"
        elif phase == "released":
            if k_st.pull in ("passed", "failed") or t - t_phase > WAIT_S:
                phase = "done"
        if int(round(t / dt)) % 24 == 0:
            log.append({"t": round(t, 3), "phase": phase, "past": [round(v, 3) for v in k_st.past_contact],
                        "turns": [round(v, 3) for v in k_st.turns], "force_N": [round(v, 1) for v in k_st.clamp_force],
                        "tilt_deg": round(b_st.tilt_deg, 2), "sheath_y_mm": round(c_st.sheath_y * 1000, 2),
                        "pull": k_st.pull, "moved_mm": round(k_st.pull_moved * 1000, 3),
                        "crushed": k_st.crushed, "cores": k_st.cores_clamped, "score": float(task.score(st)),
                        "bar_z_mm": round((float(cc.rigid_pose(base, cc._NAMES["bar"])[0][2]) - task.TOP) * 1000, 3),
                        "cable_z_mm": [round(float(p[2]) * 1000, 3) for p in cc.cable_geometry(base)["starts"][4:7]],
                        "heads_z_mm": [round((float(cc.body_poses(base, cc._NAMES["clamp"])[s_][0][2]) - task.TOP)
                                             * 1000, 3) for s_ in cc.SCREWS]})
        emb.step(hold)
        if phase == "done":
            break
    st = object_states(emb._env, emb.task.names)
    k_st = st["clamp"]
    events["end"] = {"t": round(t, 3), "pull": k_st.pull, "moved_mm": round(k_st.pull_moved * 1000, 3),
                     "crushed": k_st.crushed, "cores_clamped": k_st.cores_clamped,
                     "past": [round(v, 3) for v in k_st.past_contact], "contact_turns": k_st.contact_turns,
                     "tilt_deg": round(st["bar"].tilt_deg, 2), "sheath_y_mm": round(st["cable"].sheath_y * 1000, 2),
                     "score": float(task.score(st))}
    return {"case": case, "events": events, "log": log}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--case", default="all", help="a case, several separated by commas, or all: " + ", ".join(CASES))
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--out", type=Path, default=Path("bench_runs"))
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="override a constant: --set LET_GO_WAIT_S=5, or a part's with --set cc.EI_CORES=6e-4")
    args = ap.parse_args(argv)
    import ast

    for item in args.set:
        name, value = item.split("=", 1)
        if name.startswith("cc."):
            sys.path.insert(0, str(HERE))
            import cc_parts

            target = vars(cc_parts)
        else:
            target = globals()
        name = name.removeprefix("cc.")
        if name not in target:
            ap.error(f"--set: no constant {name!r}")
        try:
            target[name] = ast.literal_eval(value)
        except (ValueError, SyntaxError):
            target[name] = value  # a plain word: a string (GRIP_LINK=link4)
    from yam_arena.inspect_embodiment import YamArenaEmbodiment

    emb = YamArenaEmbodiment(task=str(HERE / "task.py"), control_hz=240.0, object_jitter=0.0, cam_height=32,
                             cam_width=32, device=args.device)
    args.out.mkdir(parents=True, exist_ok=True)
    cases = list(CASES) if args.case == "all" else args.case.split(",")
    for case in cases:
        if case not in CASES:
            ap.error(f"no case {case!r}")
    for case in cases:
        res = run(emb, case)
        (args.out / f"cc_bench_{case}.json").write_text(json.dumps(res, indent=1, default=float))
        print("BENCH " + json.dumps({"case": case, **res["events"]}, default=float))
        sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
