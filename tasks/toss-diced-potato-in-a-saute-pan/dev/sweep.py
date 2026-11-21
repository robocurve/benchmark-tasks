"""Dev only: play several open-loop plans in one simulator session and trace each (see dev/trace.py). Each episode is a
JSON object {"label": ..., "tosses": [[knot rows], ...], "variant": "" | "shake" | "poke" | "none"} in the file given.

    flock <lock> python dev/sweep.py episodes.json out_dir
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)


def main(spec_path, out_dir):
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack

    from pan_toss.embodiment import make_embodiment

    os.makedirs(out_dir, exist_ok=True)
    emb = make_embodiment(task=os.path.join(HERE, "task.py"), object_jitter=0, control_hz=20, cam_width=320, cam_height=176)
    m = emb.task.module
    ready = np.r_[m.READY, 1.0]
    for ep in json.load(open(spec_path)):
        m.VARIANT = ep.get("variant", "")
        if "tosses" in ep:
            os.environ["PAN_TOSS"] = json.dumps(ep["tosses"])
        else:
            os.environ.pop("PAN_TOSS", None)
        plan = m.demo()["left"][0]
        emb.log_path = os.path.join(out_dir, ep["label"] + ".npz")
        emb.reset(Scene(id="dev", instruction=m.INSTRUCTION), seed=0)
        steps = len(plan.q) + int(ep.get("hold_s", 2.0) * 20)
        info = {}
        for k in range(steps):
            a, _ = plan.action7((k + 1) * 0.05)
            info = emb.step(Action(data=pack(a, ready))).info
        emb.flush_log()
        toss_t = [i * 0.05 for i, (a, b) in enumerate(zip([""] + plan.labels, plan.labels)) if b.startswith("toss") and a != b]
        json.dump({"label": ep["label"], "toss_t": toss_t, "score": info.get("score"), "counts": m.counts(emb._states()), "trial_touched": bool(m.trial()["touched"])},
                  open(os.path.join(out_dir, ep["label"] + ".json"), "w"))
        print("EPISODE", ep["label"], "score", info.get("score"), m.counts(emb._states()), "touched", m.trial()["touched"], flush=True)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
