"""Horn friction probe (development / fidelity check): do the jointed horns resist turning as authored? By default
only the bottom horn is jointed (the top horn is fixed); with SO101_HORN_JOINTS=articulation both are.

    SO101_HORN_JOINTS=articulation|maximal python probe_horn.py task.py OUT

Arms stay at READY; nothing touches the horns. A torque about the servo axis is applied to each horn on every physics
step (PhysX step callback, tensor API), in phases of 1 s: 0, 0.3, 0.45, 0.9 x the horn's authored friction torque,
then 0 again. With Coulomb friction as authored the horn must not turn below 1.0 x (no creep), must turn above it
and must stop when the torque is removed.
Writes OUT/probe_horn.json (angle of each horn per control step and the per-phase turn in degrees).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'src'))

PHASES = (0., .3, .45, .9, 1.5, 0.)   # x authored friction torque, 1 s each (1.5 x: must turn)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('task')
    ap.add_argument('out')
    ap.add_argument('--device', default='cpu', help='PhysX device (as the task runs: cpu)')
    a = ap.parse_args()
    import torch
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    import jointkit
    import layout as L
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    emb = yam_arena_embodiment(task=str(Path(a.task).resolve()), object_jitter=0, control_hz=20, cam_width=64,
                               cam_height=36, device=a.device)
    emb.reset(Scene(id='probe', instruction='probe'))
    print(f'PROBE reset {time.time() - t0:.0f} s mode {jointkit.HORN_JOINTS}', flush=True)
    import omni.physx  # only importable once the app runs
    rt = jointkit.runtime()
    view = rt.view
    n = view.count
    axis = L.R_S[:, 2]
    friction = {'horn_top': L.HORN_TOP_FRICTION_NM, 'horn_bottom': L.HORN_BOTTOM_FRICTION_NM}
    scale = {'k': 0.}
    calls = {'n': 0}

    def on_step(dt):
        calls['n'] += 1
        tq = torch.zeros((n, 3), dtype=torch.float32, device=view.get_transforms().device)
        for name, f in friction.items():
            tq[rt.index[name]] = torch.tensor(axis * f * scale['k'], dtype=torch.float32)
        idx = torch.arange(n, dtype=torch.int32, device=tq.device)
        view.apply_forces_and_torques_at_position(None, tq, None, idx, True)

    sub = omni.physx.get_physx_interface().subscribe_physics_step_events(on_step)

    def angle(name):
        r = rt.read()[name][1]
        v = L.R_S.T @ r[:, 0]
        return math.degrees(math.atan2(v[1], v[0]))

    q = np.r_[READY, 1.0]
    rows, phases = [], []
    for k_scale in PHASES:
        scale['k'] = k_scale
        a0 = {h: angle(h) for h in friction}
        for _ in range(20):
            emb.step(Action(data=pack(q, q)))
            rows.append(dict(scale=k_scale, **{h: round(angle(h), 4) for h in friction}))
        turn = {h: round(((angle(h) - a0[h] + 180.) % 360.) - 180., 4) for h in friction}
        phases.append(dict(scale=k_scale, torque_nm={h: round(f * k_scale, 4) for h, f in friction.items()},
                           turn_deg=turn))
        print('PROBE phase', json.dumps(phases[-1]), flush=True)
    sub = None
    # holds below its friction torque (no creep), turns above it, stops once the torque is removed (last phase:
    # angle constant after its first control step; its turn_deg is not used because the horn was spinning)
    last = [r['horn_top'] for r in rows[-19:]]
    ok = all(abs(p['turn_deg']['horn_top']) < .05 for p in phases[:4]) and \
        abs(phases[4]['turn_deg']['horn_top']) > 1. and max(last) - min(last) < .05
    res = dict(mode=jointkit.HORN_JOINTS, friction_nm=friction, phases=phases, physics_step_callbacks=calls['n'],
               behaves_as_authored=bool(ok), rows=rows, nonfinite=jointkit.STATE['nonfinite'])
    (out / 'probe_horn.json').write_text(json.dumps(res, indent=1, default=float))
    print('PROBE done', json.dumps(dict(mode=res['mode'], behaves_as_authored=ok, callbacks=calls['n'])), flush=True)


if __name__ == '__main__':
    main()
