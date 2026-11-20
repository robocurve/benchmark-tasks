"""Bench probe (development): spawn, arms at READY, step N control steps, report the bodies and the scorer.

    python probe.py task.py OUT [--steps N]
Writes OUT/probe.json and OUT/probe_<camera>.png (first and last frames).
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'src'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('task')
    ap.add_argument('out')
    ap.add_argument('--steps', type=int, default=60)
    a = ap.parse_args()
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    import imageio
    import jointkit
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    emb = yam_arena_embodiment(task=str(Path(a.task).resolve()), object_jitter=0, control_hz=20, cam_width=320,
                               cam_height=176)
    obs = emb.reset(Scene(id='probe', instruction='probe'))
    print(f'PROBE reset {time.time() - t0:.0f} s', flush=True)
    for cam, img in obs.images.items():
        imageio.imwrite(out / f'probe_{cam}_first.png', np.asarray(img))
    rt = jointkit.runtime()
    rows = []
    q = np.r_[READY, 1.0]
    t1 = time.time()
    for k in range(a.steps):
        res = emb.step(Action(data=pack(q, q)))
        poses = rt.read()
        rows.append(dict(step=k + 1, score=res.info.get('score'), detail=jointkit.STATE['detail'],
                         bodies={n: np.round(p * 1000, 3).tolist() for n, (p, r) in poses.items()}))
        if k % 10 == 0:
            print('PROBE', json.dumps(rows[-1]['detail'].get('measure', {})), rows[-1]['bodies']['bracket'], flush=True)
    for cam, img in res.observation.images.items():
        imageio.imwrite(out / f'probe_{cam}_last.png', np.asarray(img))
    res = dict(wall_reset_s=round(t1 - t0, 1), wall_step_s=round((time.time() - t1) / a.steps, 3), rows=rows,
               nonfinite=jointkit.STATE['nonfinite'])
    (out / 'probe.json').write_text(json.dumps(res, indent=1, default=float))
    print('PROBE done', res['wall_reset_s'], res['wall_step_s'], flush=True)


if __name__ == '__main__':
    main()
