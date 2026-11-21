"""Run several demo variants in one Isaac process, each after a fresh reset, no video.

    python demo.py task.py OUT_DIR --variants full,one,big   (or: python variants.py task.py OUT_JSON full one big)

Prints and writes each variant's score, steps, violations and breath episodes.
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def main(task_path, out_json, variants, max_steps=2400):
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    import demo
    from demokit import CONTROL_HZ, Ctx

    task_path = str(Path(task_path).resolve())
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=320,
                               cam_height=176)
    sys.modules.setdefault("task", emb.task.module)
    results = {}
    for v in variants:
        emb.reset(Scene(id=f"variant-{v}", instruction=getattr(emb.task.module, "INSTRUCTION", "demo")))
        ctx = Ctx(emb)
        gen = demo.script(ctx, v)
        steps, success, info, t0 = 0, False, {}, time.time()
        while steps < max_steps:
            try:
                next(gen)
            except StopIteration:
                pass
            res = emb.step(ctx.action())
            steps += 1
            info = res.info
            if steps % 100 == 0:
                print(f"[{v}] step {steps} {demo.log(ctx, steps, info)[:300]}", flush=True)
            if res.terminated:
                success = res.termination_reason == "success"
                break
        S = ctx.task._STATE
        B = S["breaths"]
        r = {"score": 1.0 if success else float(info.get("score", 0.0) or 0.0), "success": success, "steps": steps,
             "violations": list(S["why"]), "breaths": B.breaths, "too_big": B.too_big, "void": B.void,
             "seated_2s": S["seated_2s"], "held_off": S["held_off"], "margins": S.get("margins", []),
             "seconds": round(time.time() - t0)}
        results[v] = r
        print(f"RESULT {v} {json.dumps(r)}", flush=True)
        Path(out_json).write_text(json.dumps(results, indent=2))
    return results


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3:])
