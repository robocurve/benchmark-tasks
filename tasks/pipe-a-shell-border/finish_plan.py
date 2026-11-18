"""Slow the physical return and preserve tool-pose feedback until set-down.

The unchanged prefix makes the six shells. Only ordinary robot targets change;
no pressure, material state, object pose or score is supplied to this compiler.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np

from plan import GRASP_TRANSLATION, GRASP_ROTATION


def compile_finish(plan):
    from yam_arena import ik, station
    result=copy.deepcopy(plan)
    frames=[]
    phases={'move dispenser to rest','turn dispenser above rest','lower dispenser to rest'}
    previous=None
    for frame in plan['frames']:
        action=np.asarray(frame['action14'],float)
        if frame['phase'] in phases:
            if previous is None:raise ValueError('Return phase requires a preceding physical action')
            for fraction in (.5,1.):
                row=copy.deepcopy(frame)
                q=previous+(action-previous)*fraction
                position,rotation=ik.tcp_pose(q[7:13],station.BASES['right'])
                row['action14']=q.tolist()
                row['scripted_tool_target']={
                    'barrel_position_m':(position+rotation@GRASP_TRANSLATION).tolist(),
                    'barrel_rotation':(rotation@GRASP_ROTATION).tolist()}
                row['joint_feedforward14']=[0.]*14
                frames.append(row)
        else:frames.append(copy.deepcopy(frame))
        previous=action
    result['frames']=frames
    result['finish_change']='Return, turn and lower over four seconds each; retain the existing bounded measured-grasp servo until set-down. Shell-making prefix unchanged.'
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('plan',type=Path)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    result=compile_finish(json.loads(args.plan.read_text()))
    result['finish_source_plan_sha256']=hashlib.sha256(args.plan.read_bytes()).hexdigest()
    args.out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'actions':len(result['frames']),'change':result['finish_change']}))
