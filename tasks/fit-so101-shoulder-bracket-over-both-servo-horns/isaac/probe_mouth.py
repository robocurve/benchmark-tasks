"""Slot-mouth probe (development only, not used by the task or the demo): places the free bracket (nothing holds it,
arms at READY) at the slot mouths with a sideways offset / roll / pitch, pushes it along the slots with a constant
force at the grasp point (applied every physics step) and reports how far it got: shows whether the mouth corners
steer the round screw head and boss into their slots on their own.

    python probe_mouth.py task.py OUT [--yaw 25] [--force 3] [--device cpu]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'src'))

GRASP_B = np.array([.0356, 0., .0300])  # demo.py's grasp point on the upright (bracket frame)
MID_B = np.array([0., 0., -.0182])      # hole axis, midway between the cheek inner faces (bracket frame)


def rx(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])


def ry(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0., s], [0., 1., 0.], [-s, 0., c]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('task')
    ap.add_argument('out')
    ap.add_argument('--yaw', type=float, default=25.)
    ap.add_argument('--force', type=float, default=3.)
    ap.add_argument('--s-mm', type=float, default=14.5)
    ap.add_argument('--at', default='grasp', choices=('grasp', 'mid'), help='push point: grasp point, or the hole '
                    'axis midway between the cheeks (no tipping moment)')
    ap.add_argument('--trials', default='', help='"py,roll,pitch;..." (mm, deg, deg) instead of the default grid')
    ap.add_argument('--device', default='cpu')
    a = ap.parse_args()
    import torch
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    import jointkit
    import layout as L
    from scoring import measure
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=str(Path(a.task).resolve()), object_jitter=0, control_hz=20, cam_width=64,
                               cam_height=36, device=a.device)
    emb.reset(Scene(id='probe', instruction='probe'))
    import omni.physx  # only importable once the app runs
    rt = jointkit.runtime()
    view = rt.view
    n = view.count
    i = rt.index['bracket']
    push = {'on': False}

    def on_step(dt):
        if not push['on']:
            return
        pb, rb = rt.read()['bracket']
        f = torch.zeros((n, 3), dtype=torch.float32, device=view.get_transforms().device)
        pos = torch.zeros((n, 3), dtype=torch.float32, device=f.device)
        f[i] = torch.tensor(-rb[:, 0] * a.force, dtype=torch.float32)  # toward home along the slots
        pos[i] = torch.tensor(pb + rb @ (GRASP_B if a.at == 'grasp' else MID_B) + rt.origin, dtype=torch.float32)
        idx = torch.tensor([i], dtype=torch.int32, device=f.device)
        view.apply_forces_and_torques_at_position(f, None, pos, idx, True)

    sub = omni.physx.get_physx_interface().subscribe_physics_step_events(on_step)  # noqa: F841
    q = np.r_[READY, 1.0]
    trials = ([tuple(float(x) for x in t.split(',')) for t in a.trials.split(';')] if a.trials else
              [(py, roll, 0.) for py in (0., .3, .6, .9) for roll in (0., .4, -.4)] + [(.3, 0., .4), (.3, 0., -.4)])
    rows = []
    for py, roll, pitch in trials:
        p, r = L.bracket_home(a.yaw)
        p = p + r @ np.array([a.s_mm / 1000., -py / 1000., 0.])
        r = r @ rx(roll) @ ry(pitch)
        tf = view.get_transforms().clone()
        tf[i, :3] = torch.tensor(p + rt.origin, dtype=tf.dtype)
        tf[i, 3:7] = torch.tensor(jointkit._quat(r), dtype=tf.dtype)
        idx = torch.tensor([i], dtype=torch.int32, device=tf.device)
        view.set_transforms(tf, idx)
        view.set_velocities(torch.zeros_like(view.get_velocities()), idx)
        m0 = measure(*rt.read()['bracket'], rt.read()['horn_top'][1], L.O_S, L.R_S).as_dict()
        push['on'] = True
        track = []
        for k in range(60):  # 3 s
            emb.step(Action(data=pack(q, q)))
            if k % 10 == 9:
                m = measure(*rt.read()['bracket'], rt.read()['horn_top'][1], L.O_S, L.R_S).as_dict()
                v = view.get_velocities()[i].cpu().numpy()
                track.append([round(m['px_mm'], 2), round(m['py_mm'], 2), round(m['tilt_deg'], 2),
                              round(float(np.linalg.norm(v[:3])) * 1e3, 1), round(float(np.degrees(np.linalg.norm(v[3:]))), 1)])
        push['on'] = False
        m1 = measure(*rt.read()['bracket'], rt.read()['horn_top'][1], L.O_S, L.R_S).as_dict()
        row = dict(py0=py, roll0=roll, pitch0=pitch, start={k: m0[k] for k in ('px_mm', 'py_mm', 'pz_mm', 'tilt_deg')},
                   end={k: m1[k] for k in ('px_mm', 'py_mm', 'pz_mm', 'tilt_deg', 'yaw_deg')}, track=track)
        rows.append(row)
        print('MOUTH', json.dumps(row), flush=True)
    (out / 'probe_mouth.json').write_text(json.dumps(rows, indent=1))
    print('PROBE done', flush=True)


if __name__ == '__main__':
    main()
