"""CPU-only evidence audit of a saved run; no simulator or source-state mutation."""
import argparse
import json
import os
from pathlib import Path


def audit(directory):
    os.environ['FURNITURE_SLIDERS_SPEC']=str((directory/'spec-used.json').resolve())
    import numpy as np
    from scoring import Scorer
    from config import FLOOR_Z
    from yam_arena import ik
    rows=[json.loads(line) for line in (directory/'measurements.jsonl').read_text().splitlines()]
    scorer=Scorer()
    actions=[];stages={}
    for row in rows:
        scorer.update(row['measurement'])
        if row['action'] is not None:actions.append(row['action'])
        stage=stages.setdefault(row['stage'],{'first':row['measurement']['step'],'last':0,
                                            'robot_contact_observations':0,'minimum_hand_clearance_m':float('inf'),
                                            'maximum_observed_finger_peak_N':None})
        stage['last']=row['measurement']['step']
        stage['robot_contact_observations']+=int(row['measurement'].get('robot_contact',False))
        stage['minimum_hand_clearance_m']=min(stage['minimum_hand_clearance_m'],row['measurement']['hand_clearance_m'])
        peaks=[max(v) for arm in row['diagnostics']['arms'].values()
               for v in arm.get('finger_peak_contacts_N',{}).values()]
        if peaks:stage['maximum_observed_finger_peak_N']=max(stage['maximum_observed_finger_peak_N'] or 0.,max(peaks))
    actions=np.asarray(actions)
    q=actions[:,[0,1,2,3,4,5,7,8,9,10,11,12]]
    grips=actions[:,[6,13]]
    limits_ok=bool(np.all(q>=np.tile(ik.LOWER,2)) and np.all(q<=np.tile(ik.UPPER,2)))
    arm_metrics={}
    for side in ('left','right'):
        tau=np.array([r['diagnostics']['arms'][side]['applied_torque'][:6] for r in rows])
        arm_metrics[side]={'peak_absolute_applied_torque_Nm':np.abs(tau).max(0).tolist(),
                           'maximum_fraction_of_effort_limit':float((np.abs(tau)/[27,27,27,7,7,7]).max())}
        for field,label in [('finger_contacts_N','maximum_control_average_finger_contact_N'),
                            ('finger_peak_contacts_N','maximum_observed_physics_step_finger_contact_N')]:
            values=[max(v) for r in rows for v in r['diagnostics']['arms'][side].get(field,{}).values()]
            arm_metrics[side][label]=max(values,default=None)
    move=[r for r in rows if r['stage']=='move']
    pad_metrics={}
    for name in rows[0]['measurement']['pads']:
        pad_metrics[name]={'maximum_origin_floor_deviation_during_move_m':
            max((abs(r['measurement']['pads'][name][2]-FLOOR_Z) for r in move),default=None),
            'minimum_load_during_move_N':min((r['measurement']['pad_loads_N'][name] for r in move),default=None),
            'maximum_load_during_move_N':max((r['measurement']['pad_loads_N'][name] for r in move),default=None)}
    result=json.loads((directory/'result.json').read_text())
    detail={k:v for k,v in scorer.detail.items() if k!='foot_corners'}
    return {'run':str(directory),'observations':len(rows),'actions_14d_finite':bool(actions.shape[1]==14 and np.isfinite(actions).all()),
            'commanded_joint_limits_ok':limits_ok,'gripper_command_bounds_ok':bool(np.all((grips>=0)&(grips<=1))),
            'arms':arm_metrics,'pads':pad_metrics,'stages':stages,'replayed_scoring':detail,
            'recorded_score_matches':scorer.score==result['scoring']['score'],
            'recorded_one_pad_gate_matches':scorer.gate_pass==result['scoring']['one_pad_gate'],
            'device_verified':bool(result.get('device',{}).get('verified'))}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--out',type=Path)
    args=parser.parse_args()
    result=audit(args.run.resolve());text=json.dumps(result,indent=2)+'\n'
    if args.out:
        with args.out.open('x') as output:output.write(text)
    print(text)
