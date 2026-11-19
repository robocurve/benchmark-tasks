"""Gland bench (not a benchmark task): the cap nut, the seal and the pull test alone, under the task's physics
(240 Hz, CPU), on four glands side by side, each with a short cable already pushed through it (straight along the
axis, its sheath end 20 mm past the wall's outer face = 11 mm past the gland's inner end, its clamp in the air).

A "virtual hand" turns each cap nut: a torque about the gland's axis from a torque-limited speed controller (a quarter
turn per second clockwise, at most NUT_TORQUE) until the nut has turned a set amount past the seal's first contact, then
lets go. The arms stay in their ready pose, far from the glands; while the virtual hand works, the gland counts as
held (no pull test). Once it lets go, the task's own pull test runs (30 N after 1 s hands off).

    python cg_bench.py [--out bench_runs] [--device cpu]     # writes cg_bench.json, prints BENCH {...}

Cases (the nut turned to so many turns past the seal's first contact, then let go; the task's pull test follows):
- a_020: 0.2 turn (short of the lower mark); b_030: 0.3 turn; c_065: 0.65 turn (between the marks, the demo's
  target); d_112: 1.12 turns (past the upper mark, short of crushing at 1.3).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

os.environ.setdefault("CG_SHEATH_LINKS", "8")  # a short cable (160 mm of sheath)
TASK_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TASK_DIR))
import cg_parts as cg  # noqa: E402

from yam_arena.task_file import TABLE_TOP  # noqa: E402

cg.install()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 1e-3
GLAND_X = 0.30
CASES = {"a_020": (-0.36, 0.2, False), "b_030": (-0.12, 0.3, False), "c_065": (0.12, 0.65, False),
         "d_112": (0.36, 1.12, False)}  # suffix: (y, turns past contact, cable held)
if os.environ.get("CG_BENCH_TURNS"):  # the calibration: the nut turned to each of these (up to four), then pulled
    _turns = [float(v) for v in os.environ["CG_BENCH_TURNS"].split(",")][:4]
    CASES = {f"t_{round(t * 100):03d}": (y, t, False) for y, t in zip((-0.36, -0.12, 0.12, 0.36), _turns)}
SHEATH_X0 = 20 * MM  # the sheath's end at the start (gland frame)

OBJECTS = []
for case, (y, _, _) in CASES.items():
    sfx = "_" + case
    origin = (GLAND_X, y, TOP + cg.AXIS_H)
    OBJECTS += [cg.part("box" + sfx, "box", origin), cg.part("gland" + sfx, "gland", origin),
                cg.part("cable" + sfx, "cable", (GLAND_X + SHEATH_X0 - cg.SHEATH_END_X, y, TOP + cg.AXIS_H))]
NAMES = [o.name for o in OBJECTS]

SETTLE_S = 0.5
TURN_SPEED = 2 * math.pi / 4  # rad/s: a quarter turn per second (the seal counts as touching after 0.25 s; at 1 turn/s the
# low cases overshot to 0.37 turn, bench run 5)
NUT_TORQUE = 0.5  # Nm, the virtual hand's limit
NUT_FEED = 0.2  # Nm, feed-forward (over the thread friction)
KD_NUT = 0.03  # Nm s/rad (on the nut's armature 2e-4 kg m2: KD dt / I = 0.6, stable at 240 Hz)
K_HOLD, D_HOLD = 0.5, 2e-3  # Nm/rad, Nm s/rad: the "hand" holding the cable against twisting
WAIT_S = 4.0  # after letting go: time for the pull test


def run(out: Path, device: str = "cpu", max_s: float = 30.0) -> dict:
    import torch
    from inspect_robots import Scene
    from inspect_robots.types import Action

    from yam_arena.inspect_embodiment import YamArenaEmbodiment

    emb = YamArenaEmbodiment(task=__file__, control_hz=240.0, object_jitter=0.0, cam_height=32, cam_width=32,
                             device=device)
    obs = emb.reset(Scene(id="gland-bench", instruction="bench"), seed=0)
    emb._env.unwrapped.cfg.sim.render_interval = 10**9
    hold = Action(data=np.asarray(obs.state["joint_pos"], dtype=float))
    base = emb._env.unwrapped
    from yam_arena.scene import object_states  # patched: runs the gland update

    def tensor(rows):
        return torch.tensor(rows, dtype=torch.float32, device=base.device)

    def torque_on(art_name, body, torque_x):
        art = base.scene.articulations[art_name]
        i = list(art.body_names).index(body)
        art.permanent_wrench_composer.set_forces_and_torques_index(
            forces=tensor([[[0.0, 0.0, 0.0]]]), torques=tensor([[[torque_x, 0.0, 0.0]]]), body_ids=[i],
            is_global=True)

    phase = dict.fromkeys(CASES, "settle")
    t_phase = dict.fromkeys(CASES, 0.0)
    events = {c: {} for c in CASES}
    log = {c: [] for c in CASES}
    max_torque = dict.fromkeys(CASES, 0.0)
    for c in CASES:
        cg.HANDS_ON.add("gland_" + c)
    dt = 1.0 / 240.0
    t = 0.0
    while t < max_s:
        states = object_states(emb._env, NAMES)
        t = float(base.episode_length_buf[0]) * dt
        for c, (_, target, held) in CASES.items():
            g, k = states["gland_" + c], states["cable_" + c]
            gname, cname = "gland_" + c, "cable_" + c
            art = base.scene.articulations[gname]
            w = float(cg._np(art.data.joint_vel)[list(art.joint_names).index("spin")])
            tau = 0.0
            if phase[c] == "settle" and t >= SETTLE_S:
                phase[c], t_phase[c] = "turn", t
            if phase[c] == "turn":
                tau = float(np.clip(NUT_FEED + KD_NUT * (TURN_SPEED - w), -NUT_TORQUE, NUT_TORQUE))
                torque_on(gname, "nut", tau)
                if held:
                    geo = cg.cable_geometry(base, gname)
                    link = cg.cable_links()[geo["outside"]][0]
                    cab = base.scene.articulations[cname]
                    i = list(cab.body_names).index(link)
                    wx = float(cg._t(cab.data.body_ang_vel_w)[0, i, 0])
                    torque_on(cname, link, -K_HOLD * math.radians(geo["twist_deg"]) - D_HOLD * wx)
                if g.contact_turns is not None and "contact_turns" not in events[c]:
                    events[c]["contact_turns"] = round(g.contact_turns, 3)
                    events[c]["contact_t"] = round(t, 3)
                if g.contact_turns is not None and g.past_contact >= target:
                    torque_on(gname, "nut", 0.0)
                    if held:
                        base.scene.articulations[cname].permanent_wrench_composer.reset()
                    cg.HANDS_ON.discard(gname)
                    phase[c], t_phase[c] = "released", t
                    events[c]["release"] = {"t": round(t, 3), "turns": round(g.turns, 3),
                                            "past": round(g.past_contact, 3), "twist_deg": round(k.twist_deg, 1),
                                            "defl_mm": round(g.pad_deflection / MM, 3)}
            elif phase[c] == "released":
                if g.pull in ("passed", "failed") or t - t_phase[c] > WAIT_S:
                    phase[c] = "done"
                    events[c]["end"] = {"t": round(t, 3), "pull": g.pull, "moved_mm": round(g.pull_moved / MM, 3),
                                        "crushed": g.crushed, "past": round(g.past_contact, 3),
                                        "twist_deg": round(k.twist_deg, 1), "sheath_x_mm": round(k.sheath_x / MM, 2),
                                        "through": k.through}
            max_torque[c] = max(max_torque[c], abs(tau))
            if int(round(t / dt)) % 12 == 0:
                log[c].append({"t": round(t, 3), "phase": phase[c], "turns": round(g.turns, 4),
                               "past": round(g.past_contact, 4), "defl_mm": round(g.pad_deflection / MM, 4),
                               "crushed": g.crushed, "pull": g.pull, "moved_mm": round(g.pull_moved / MM, 3),
                               "twist_deg": round(k.twist_deg, 2), "sheath_x_mm": round(k.sheath_x / MM, 3),
                               "tau": round(tau, 4), "w": round(w, 3)})
        emb.step(hold)
        if all(p == "done" for p in phase.values()):
            break

    summary = {"constants": {"pitch_mm": cg.PITCH / MM, "seal_ratio": cg.SEAL_RATIO, "pad_k_N_per_mm": cg.PAD_K / 1e3,
                             "pull_N": cg.PULL_FORCE, "snug_turns": cg.SNUG_TURNS}}
    for c in CASES:
        summary[c] = {"events": events[c], "final_phase": phase[c], "max_torque_Nm": round(max_torque[c], 3),
                      "first_sheath_x_mm": log[c][0]["sheath_x_mm"] if log[c] else None}
    summary["sim_s"] = round(t, 3)
    out.mkdir(parents=True, exist_ok=True)
    (out / "cg_bench.json").write_text(json.dumps({"summary": summary, "log": log}, indent=1, default=float))
    return summary


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--max-s", type=float, default=30.0)
    ap.add_argument("--out", type=Path, default=Path("bench_runs"))
    args = ap.parse_args(argv)
    summary = run(args.out, args.device, args.max_s)
    print("BENCH " + json.dumps(summary, default=float))
    sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
