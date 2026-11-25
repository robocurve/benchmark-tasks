"""Runner: the yam_pair embodiment and the task's demo controllers, without the Inspect CLI.

    python dev/run.py OUT [--steps N] [--frames K] [--native] [--variant V]
"""
import argparse, json, os, sys, time
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("out")
ap.add_argument("--steps", type=int, default=700)
ap.add_argument("--frames", type=int, default=0, help="save the overview every K steps (0: none)")
ap.add_argument("--native", action="store_true", help="also save the six station cameras with each frame")
ap.add_argument("--variant", default="")
ap.add_argument("--print-every", type=int, default=10)
ap.add_argument("--window", default="", help="FROM:TO steps printed every step")
args = ap.parse_args()
os.environ["RACKPAIR_VARIANT"] = args.variant
HERE = Path(__file__).resolve().parents[1]
out = Path(args.out); out.mkdir(parents=True, exist_ok=True)

import numpy as np
from inspect_robots import Scene
from inspect_robots.types import Action
from yam_pair.embodiment import YamPairEmbodiment
from yam_pair import demo as ydemo, boundary
from yam_pair.layout import STATIONS

t0 = time.time()
emb = YamPairEmbodiment(task=str(HERE / "task.py"), overview=bool(args.frames))
obs = emb.reset(Scene(id="dev", instruction="rack"))
print("reset done", round(time.time() - t0, 1), "s", flush=True)
task = emb.task
plan = task.call_layout("demo", emb.layout)
ctrls = {s: ydemo.controller(plan.get(s), s, emb.layout, 20.0) for s in STATIONS}
from PIL import Image
log = []
for step in range(args.steps):
    t = float(obs.state_time) + 0.05
    rows = {}
    for s in STATIONS:
        state = np.asarray(obs.state[boundary.state_key(s)], float)
        rows[s] = ctrls[s].act(ydemo.StationView(s, t, state, emb.layout, emb.object_states))
    res = emb.step(Action(data=boundary.join_actions(rows["a"], rows["b"])))
    obs = res.observation
    rep = res.info.get("report", {})
    phases = {s: getattr(getattr(c, "obj", c), "phase", None) for s, c in ctrls.items()}
    cmds = {s: getattr(getattr(c, "obj", c), "last_cmd_w", None) for s, c in ctrls.items()}
    rec = {"step": step + 1, "t": round(t, 2), "phases": phases, "score": res.info.get("score"), "cmd": cmds, **rep}
    log.append(rec)
    w0, w1 = (int(v) for v in args.window.split(":")) if args.window else (-1, -1)
    if (step + 1) % 50 == 0:
        (out / "log.json").write_text(json.dumps(log, default=float))
    if (step + 1) % args.print_every == 0 or res.terminated or w0 <= step + 1 <= w1:
        keys = ["score", "max_lift_mm", "z_mm", "y_mm", "x_mm", "tilt_deg", "pitch_deg", "yaw_deg", "tip_a_mm", "tip_b_mm",
                "inside_a", "inside_b", "ear_gap_a_mm", "ear_gap_b_mm", "lifted_together", "dropped",
                "a_left_kg", "a_right_kg", "b_left_kg", "b_right_kg"]
        print(step + 1, phases, {k: (round(rep[k], 1) if isinstance(rep.get(k), float) else rep.get(k)) for k in keys if k in rep},
              cmds if w0 <= step + 1 <= w1 else "", flush=True)
    if args.frames and (step % args.frames == 0 or res.terminated):
        im = obs.images.get("overview_cam")
        if im is not None:
            Image.fromarray(im).save(out / f"ov_{step:04d}.png")
        if args.native:
            for s in STATIONS:
                row = np.concatenate([obs.images[f"{s}_{c}_cam"] for c in ("top", "left", "right")], axis=1)
                Image.fromarray(row).save(out / f"{s}_{step:04d}.png")
    if res.terminated:
        print("TERMINATED", res.termination_reason, "at step", step + 1, flush=True)
        break
    if all(c.finished for c in ctrls.values()) and step > 40 and not any(v for k, v in rep.items() if k.endswith("_touch")):
        pass
(out / "log.json").write_text(json.dumps(log, default=float))
final = log[-1]
summary = {"steps": len(log), "final_score": final.get("score"), "variant": args.variant,
           "worst_ik_mm": {s: {r: getattr(getattr(c, "obj", c), "arm", {}).get(r).worst_mm if hasattr(getattr(c, "obj", c), "arm") else None
                                for r in ("front", "rear")} for s, c in ctrls.items()},
           "ik_by_phase": {s: {k: round(v, 1) for k, v in getattr(getattr(c, "obj", c), "ik_by_phase", {}).items() if v > 2}
                           for s, c in ctrls.items()},
           "ctrl_peak_load": {s: getattr(getattr(c, "obj", c), "peak_load", None) for s, c in ctrls.items()},
           "final": final, "wall_s": round(time.time() - t0, 1)}
(out / "summary.json").write_text(json.dumps(summary, indent=1, default=float))
print("SUMMARY", json.dumps({k: summary[k] for k in ("steps", "final_score", "wall_s")}), flush=True)
os._exit(0)
