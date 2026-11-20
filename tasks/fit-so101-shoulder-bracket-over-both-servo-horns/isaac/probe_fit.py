"""Fit probe (development only, not used by the task or the demo): places the bracket at poses along its slide path
(teleport between trials, arms at READY, nothing holds the bracket) and reports where PhysX moves it after 0.5 s:
an overlap shows up as a depenetration push; a clear path shows only the bracket settling on the horns.

    python probe_fit.py task.py OUT [--yaw 25] [--device cpu]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'src'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('task')
    ap.add_argument('out')
    ap.add_argument('--yaw', type=float, default=25.)
    ap.add_argument('--poses', default='', help='"px,py;px,py;..." (mm, horn axis in the bracket frame) instead of '
                    'the default home approach (3 / 1 / 0.5 / 0.2 / 0 mm out)')
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
    rt = jointkit.runtime()
    try:  # Isaac Lab turns PhysX contact processing off; the contact listing below needs it (development only)
        import carb
        carb.settings.get_settings().set_bool('/physics/disableContactProcessing', False)
    except Exception as e:  # noqa: BLE001
        print('PROBE carb setting failed', repr(e), flush=True)
    q = np.r_[READY, 1.0]
    i = rt.index['bracket']
    rows = []
    poses = ([tuple(float(x) for x in t.split(',')) for t in a.poses.split(';')] if a.poses
             else [(-s, 0.) for s in (3., 1., .5, .2, 0.)])
    for pose in poses:
        px0, py0 = pose[:2]
        s_mm = -px0
        for dz in ((pose[2] / 1000.,) if len(pose) > 2 else (0.,)):  # dz: bracket offset along z (mm)
            p, r = L.bracket_home(a.yaw)
            p = p + r @ np.array([s_mm / 1000., -py0 / 1000., 0.]) + np.array([0., 0., dz])
            tf = rt.view.get_transforms().clone()
            tf[i, :3] = torch.tensor(p + rt.origin, dtype=tf.dtype)
            tf[i, 3:7] = torch.tensor(jointkit._quat(r), dtype=tf.dtype)
            idx = torch.tensor([i], dtype=torch.int32, device=tf.device)
            rt.view.set_transforms(tf, idx)
            rt.view.set_velocities(torch.zeros_like(rt.view.get_velocities()), idx)
            m0 = measure(*rt.read()['bracket'], rt.read()['horn_top'][1], L.O_S, L.R_S)
            for _ in range(10):
                emb.step(Action(data=pack(q, q)))
            contacts = {}
            try:
                from omni.physx import get_physx_simulation_interface
                from pxr import PhysicsSchemaTools
                headers, data = get_physx_simulation_interface().get_contact_report()
                for h in headers:
                    a0 = str(PhysicsSchemaTools.intToSdfPath(h.collider0))
                    a1 = str(PhysicsSchemaTools.intToSdfPath(h.collider1))
                    pts = [data[h.contact_data_offset + k] for k in range(h.num_contact_data)]
                    key = ' | '.join(sorted((a0.split('/')[-2] + '/' + a0.split('/')[-1], a1.split('/')[-2] + '/' + a1.split('/')[-1])))
                    imp = sum(float(np.linalg.norm(np.asarray(c.impulse))) for c in pts)
                    sep = min(float(c.separation) for c in pts) if pts else None
                    old = contacts.get(key, (0., None, None, None))
                    pb_, rb_ = rt.read()['bracket']
                    loc = [np.round(rb_.T @ (np.asarray(c.position) - rt.origin - pb_) * 1000, 2).tolist() for c in pts[:2]]
                    nrm = [np.round(rb_.T @ np.asarray(c.normal), 2).tolist() for c in pts[:1]]
                    contacts[key] = (round(old[0] + imp, 6), sep if old[1] is None else min(old[1], sep), loc, nrm)
            except Exception as e:  # noqa: BLE001
                contacts = {'error': repr(e)}
            m1 = measure(*rt.read()['bracket'], rt.read()['horn_top'][1], L.O_S, L.R_S)
            d0, d1 = m0.as_dict(), m1.as_dict()
            poses = rt.read()
            pb, rb = poses['bracket']
            rel0 = rb.T @ (L.O_S - pb)
            ax = rb.T @ L.R_S[:, 2]
            for h in (0., -.0182, -.0364):
                t = (h - rel0[2]) / ax[2]
                d1[f'axis_xy_at_{round(h * 1000, 1)}_mm'] = np.round((rel0[:2] + t * ax[:2]) * 1000, 3).tolist()
            d1['horn_top_off_mm'] = np.round((poses['horn_top'][0] - L.O_S) * 1000, 3).tolist()
            d1['horn_bottom_off_mm'] = np.round((poses['horn_bottom'][0] - L.O_S) * 1000, 3).tolist()
            row = dict(s_mm=s_mm, py0_mm=py0, dz_mm=dz * 1000, start={k: d0[k] for k in ('px_mm', 'py_mm', 'pz_mm', 'tilt_deg')},
                       end={k: d1[k] for k in ('px_mm', 'py_mm', 'pz_mm', 'tilt_deg', 'yaw_deg', 'horn_top_off_mm',
                                              'horn_bottom_off_mm', 'axis_xy_at_0.0_mm', 'axis_xy_at_-18.2_mm',
                                              'axis_xy_at_-36.4_mm')})
            row['contacts'] = contacts
            rows.append(row)
            print('FIT', json.dumps(row), flush=True)
    (out / 'probe_fit.json').write_text(json.dumps(rows, indent=1))
    print('PROBE done', flush=True)


if __name__ == '__main__':
    main()
