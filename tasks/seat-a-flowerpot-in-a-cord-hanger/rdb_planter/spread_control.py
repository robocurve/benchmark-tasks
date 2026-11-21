"""Isolate whether the left gripper physically retains and spreads the basket cords."""
import argparse
import json
from pathlib import Path

import numpy as np
from inspect_robots import Action, Scene
from yam_arena import ik
from . import fixed_scene as s
from .arena import make_embodiment


def orientation():
    phi = np.deg2rad(30.)
    forward = np.array([np.cos(phi), np.sin(phi), -.35])
    forward /= np.linalg.norm(forward)
    close = np.array([0., 1., 0.])
    close -= (forward @ close) * forward
    close /= np.linalg.norm(close)
    return np.column_stack((close, np.cross(forward, close), forward))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--condition', choices=('oblique', 'across_two_cords','open_for_rim'), default='oblique')
    args = parser.parse_args()
    out = args.out
    out.mkdir(exist_ok=True, parents=True)
    rig = make_embodiment(out)
    try:
        obs = rig.reset(Scene(id='spread-test', instruction=s.INSTRUCTION), seed=0)
        initial = np.asarray(obs.state['joint_pos']).copy()
        q = initial.copy()
        base = [0., .305, .75]
        rot = orientation()
        if args.condition != 'oblique':
            incline = np.deg2rad(15.)
            forward = np.array([0., -np.cos(incline), -np.sin(incline)])
            across = np.array([-1., 0., 0.])
            rot = np.column_stack((across, np.cross(forward, across), forward))
        pad_offset = np.array([0., -.004, .008])
        start = np.mean([rig.poses[f'cord_{i}_10'][0] for i in (0, 1)], axis=0)
        trace = []
        phases = [('clear of cords', 60, -.075, .35), ('approach', 40, -.030, .35),
                  ('slide in', 44, 0., .28), ('close', 36, 0., .006),
                  ('pull the gripped pair', 118 if args.condition=='open_for_rim' else 75, 0., .006), ('hold', 45, 0., .006)]
        for phase, duration, offset, jaw in phases:
            prior = q.copy()
            tcp, actual = ik.tcp_pose(np.asarray(obs.state['joint_pos'])[:6], base)
            pad = tcp + actual @ pad_offset
            pair = np.mean([rig.poses[f'cord_{i}_10'][0] for i in (0, 1)], axis=0)
            if phase == 'pull the gripped pair':
                destination = pad + (np.array([0., .155, .018]) if args.condition=='open_for_rim' else np.array([0., .09, .006]))
            elif phase in ('close', 'hold'):
                destination = pad
            else:
                destination = pair + offset * rot[:, 2]
            joint_target, pos_error, angular_error = ik.solve(destination - rot @ pad_offset,
                ik.matrix_to_quat_xyzw(rot), prior[:6], base, iters=160, rot_weight=.18)
            if phase in ('close', 'hold'):
                joint_target = prior[:6]
            for i in range(1, duration+1):
                fraction=ik.smoothstep(i/duration)
                q[:6]=prior[:6]+fraction*(joint_target-prior[:6])
                q[6]=prior[6]+fraction*(jaw-prior[6])
                outcome=rig.step(Action(data=q.copy(),meta={}))
                obs=outcome.observation
                row=dict(phase=phase,tick=rig.tick,ik_error_m=float(pos_error),ik_rotation_error=float(angular_error))
                row.update(outcome.info['fit'])
                row['measured_jaw']=float(obs.state['joint_pos'][6])
                trace.append(row)
        held=[]
        for r in trace[-30:]:
            section=r['cradle_section_m']
            width=float(np.ptp(np.array(section)[:,1])) if section else 0.
            loads=np.asarray(r['left_finger_cord_normals_n'])
            if args.condition=='open_for_rim':
                # Holding the intended pair matters. Width is measured between
                # the two groups of material, even if the four cords twist.
                section=np.asarray(section)
                gap=section[:2,1].min()-section[2:,1].max() if section.size else 0.
                by_cord=loads.reshape(2,4,18).max(axis=2)
                held.append(bool(gap>2*s.RIM_RADIUS and np.min(by_cord[:,:2].max(axis=0))>.1 and by_cord[:,2:].max()<.02))
            else:
                held.append(bool(width>.080 and np.min(loads.max(axis=1))>.1))
        result=dict(condition={'oblique':'left_spread','across_two_cords':'crosswise_spread','open_for_rim':'rim_width_spread'}[args.condition],passed=all(held),reference_run=False,initial_pair=start.tolist(),trace=trace)
        (out/'CONTROL.json').write_text(json.dumps(result))
    finally:
        rig.close()


if __name__=='__main__':
    main()
