"""Run several demo variants in one Isaac process (one reset each), logging report() every step.

    python dev/run_many.py OUTDIR VARIANT:STEPS [VARIANT:STEPS ...]     (variant "demo" = the full demo)
    DEV_PEAK_TORQUE=1 python dev/run_many.py ...    check C1b: the arm drives at the motors' peak torque (27 / 7 N m)
                                                    instead of task.py's continuous-rating cap
"""
import json, os, sys, time
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
out_root = Path(sys.argv[1]); out_root.mkdir(parents=True, exist_ok=True)
specs = [s.split(":") for s in sys.argv[2:]]

import numpy as np
from inspect_robots import Scene
from inspect_robots.types import Action
from yam_pair.embodiment import YamPairEmbodiment
from yam_pair import demo as ydemo, boundary
from yam_pair.layout import STATIONS

emb = YamPairEmbodiment(task=str(HERE / "task.py"), overview=False)
results = {}
_peak_set = False


def peak_torque():
    """Raise every arm's drive limit back to the motors' peaks (after task.py's on_build capped it)."""
    import torch
    from yam_arena.constants import ARM_JOINTS
    sys.path.insert(0, str(HERE))
    import geometry as g
    for s in ("a", "b"):
        for side in ("left", "right"):
            robot = emb._env.unwrapped.scene[f"{s}_{side}_robot"]
            ids = robot.find_joints(ARM_JOINTS, preserve_order=True)[0]
            robot.write_joint_effort_limit_to_sim_index(
                limits=torch.tensor([list(g.PEAK)], dtype=torch.float32, device=robot.device), joint_ids=ids)


for name, steps in specs:
    t0 = time.time()
    os.environ["RACKPAIR_VARIANT"] = "" if name == "demo" else name.split("#")[0]
    obs = emb.reset(Scene(id=name, instruction="rack"))
    if os.environ.get("DEV_PEAK_TORQUE") == "1" and not _peak_set:
        peak_torque()
        _peak_set = True
        obs = emb.reset(Scene(id=name, instruction="rack"))
    plan = emb.task.call_layout("demo", emb.layout)
    ctrls = {s: ydemo.controller(plan.get(s), s, emb.layout, 20.0) for s in STATIONS}
    log, term = [], None
    for step in range(int(steps)):
        t = float(obs.state_time) + 0.05
        rows = {s: ctrls[s].act(ydemo.StationView(s, t, np.asarray(obs.state[boundary.state_key(s)], float), emb.layout,
                                                   emb.object_states)) for s in STATIONS}
        res = emb.step(Action(data=boundary.join_actions(rows["a"], rows["b"])))
        obs = res.observation
        rep = res.info.get("report", {})
        log.append({"step": step + 1, "phase": getattr(getattr(ctrls["a"], "obj", ctrls["a"]), "phase", None),
                    "score": res.info.get("score"), **{k: v for k, v in rep.items() if not isinstance(v, dict)}})
        if res.terminated:
            term = res.termination_reason
            break
    d = out_root / f"check_{name.replace('#', '_')}"
    d.mkdir(exist_ok=True)
    (d / "log.json").write_text(json.dumps(log, default=float))
    f = log[-1]
    results[name] = {"steps": len(log), "termination": term, "final_score": f.get("score"),
                     "max_lift_mm": f.get("max_lift_mm"), "lifted_together": f.get("lifted_together"),
                     "dropped": f.get("dropped"), "engaged": [f.get("engaged_a"), f.get("engaged_b")],
                     "on_top": [f.get("on_top_a"), f.get("on_top_b")], "tips_mm": [f.get("tip_a_mm"), f.get("tip_b_mm")],
                     "ear_gaps_mm": [f.get("ear_gap_a_mm"), f.get("ear_gap_b_mm")],
                     "peak_kg": {a: f.get(a + "_peak_kg") for a in ("a_left", "a_right", "b_left", "b_right")},
                     "tau_cont_peak": {a: f.get(a + "_tau_cont_peak") for a in ("a_left", "a_right", "b_left", "b_right")},
                     "over_cont_s": {a: f.get(a + "_over_cont_s") for a in ("a_left", "a_right", "b_left", "b_right")},
                     "cross_contact_peak_n": f.get("cross_contact_peak_n"), "untouched_steps": f.get("untouched_steps"),
                     "peak_torque": os.environ.get("DEV_PEAK_TORQUE") == "1",
                     "wall_s": round(time.time() - t0, 1)}
    print("RESULT", name, json.dumps(results[name], default=float), flush=True)
    (out_root / "results.json").write_text(json.dumps(results, indent=1, default=float))
os._exit(0)
