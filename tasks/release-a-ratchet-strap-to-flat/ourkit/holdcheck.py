#!/usr/bin/env python3
"""holdcheck: does every mechanism hold its state with no command? MuJoCo, on physcheck's own scene builder.

    nice -n 15 env OMP_NUM_THREADS=1 PYTHONPATH=<yam-isaac-lab-arena> python3 <task>/ourkit/holdcheck.py <task> \\
        [--seconds 5] [--soft] [--pipeline .planning/pipeline]

Builds the task's scene exactly as physcheck does (arms, room, every Mech through the kit's own authoring), runs the
embodiment's reset (0.5 s settle, then the kit's reset hooks and rules: preloads, start catches), then keeps both arms
in their start pose for ``--settle`` s (the rules' start states take hold) and then ``--seconds`` more with the task's rules running. It
prints one JSON line: for each mech joint its position at the reset, after the settle, and its largest change after
the settle (mm or deg: creep or drift), the body's largest move (mm), the score at the end,
and the rule events. ``--soft`` uses physcheck's default soft joint friction; the default is ``--rigid-friction stiff``
(PhysX-like: no creep under the breakaway effort).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("task", type=Path)
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--soft", action="store_true")
    ap.add_argument("--settle", type=float, default=0.5, help="s after the reset before the reference state is taken")
    ap.add_argument("--pipeline", type=Path, default=None)
    a = ap.parse_args()
    here = str(Path(__file__).resolve().parent)
    sys.path[:] = [p for p in sys.path if Path(p or ".").resolve() != Path(here)]  # never import kit files top-level
    task_dir = a.task.resolve()
    pipe = a.pipeline
    if pipe is None:
        pipe = next((d / ".planning" / "pipeline" for d in task_dir.parents
                     if (d / ".planning" / "pipeline" / "physcheck.py").exists()), None)
    if pipe is None:
        raise SystemExit("holdcheck: no .planning/pipeline found (pass --pipeline)")
    sys.path.insert(0, str(pipe.resolve()))
    import numpy as np
    import physcheck as pc

    pc.RIGID_FRICTION = None if a.soft else "stiff"
    task, b, _ = pc.build(task_dir)
    sim = pc.Sim(b.xml(), b, task.scene)
    emb = sys.modules["yam_arena.inspect_embodiment"].YamArenaEmbodiment(sim, task)
    emb.reset(None)
    S = sys.modules["ourkit.scene"]
    from yam_arena.constants import READY

    ready = np.r_[READY, 1.0, READY, 1.0]
    st = S.LATEST["states"]
    mechs = {n: st[n] for n in S.MECHS if n in st}
    reset_q = {n: dict(s.q) for n, s in mechs.items()}
    res = None
    for _ in range(int(round(a.settle * pc.CONTROL_HZ))):  # the rules' start states take hold (a catch, a preload)
        res = emb.step(ready)
    st = S.LATEST["states"]
    q0 = {n: dict(st[n].q) for n in mechs}
    p0 = {n: np.asarray(st[n].pos, dtype=float) for n in mechs}
    drift = {n: {j: 0.0 for j in q0[n]} for n in mechs}
    body = {n: 0.0 for n in mechs}
    for _ in range(int(round(a.seconds * pc.CONTROL_HZ))):
        res = emb.step(ready)
        now = S.LATEST["states"]
        for n in mechs:
            for j, v in now[n].q.items():
                drift[n][j] = max(drift[n][j], abs(v - q0[n][j]))
            body[n] = max(body[n], float(np.linalg.norm(np.asarray(now[n].pos) - p0[n])))
    out = {"seconds": a.seconds, "settle": a.settle, "friction": "soft" if a.soft else "rigid", "joints": {}, "body_mm": {},
           "score": None if res is None else res.info.get("score"),
           "events": [list(e) for rs in S._RULES.values() for e in rs.events()]}
    for n, spec in S.MECHS.items():
        if n not in mechs:
            continue
        for jt in spec.joints:
            k = 1000.0 if jt.kind == "slide" else 180.0 / math.pi
            out["joints"][f"{n}.{jt.name}"] = {"unit": "mm" if jt.kind == "slide" else "deg",
                                               "at_reset": round(reset_q[n][jt.name] * k, 4),
                                               "q0": round(q0[n][jt.name] * k, 4),
                                               "drift": round(drift[n][jt.name] * k, 4)}
        out["body_mm"][n] = round(body[n] * 1000, 4)
    print(json.dumps(out))


if __name__ == "__main__":
    main()
