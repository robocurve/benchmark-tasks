"""Plan a smooth two-arm transfer above the clear tabletop beside the bin.

Only robot TCP commands are produced. No object state is altered.
"""
import json
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation

def plan(state, steps=160, final_z=1.38, final_x=-.03):
    from yam_arena.ik import solve,tcp_pose
    from yam_arena import station
    q=np.array(state['commanded_joints'],float)
    start=np.array(state['tcp'],float)
    top=float(np.mean(np.array(state['tcp'])[:,2]))
    centres=np.array([[.14,0.,max(top,final_z)],[.01,0.,max(final_z,min(top,1.47))],[final_x,0.,final_z]])
    targets=np.array([start,*[np.array([c+[0,.06,0],c+[0,-.06,0]]) for c in centres]])
    curve=CubicSpline(np.linspace(0,1,len(targets)),targets,axis=0,bc_type='clamped')
    operations=[];maximum_error=0.;maximum_joint_step=0.
    for step in range(1,steps+1):
        goals=curve(step/steps);rotations=[];old=q.copy()
        for k,side in enumerate(['left','right']):
            q[k],err,_=solve(goals[k],None,q_init=q[k],base_pos=station.BASES[side],iters=300,tol=.0002)
            if err>.002:raise RuntimeError(f'Table path unreachable at {step}, arm {k}: {err}')
            maximum_error=max(maximum_error,err)
            rotations.append(Rotation.from_matrix(tcp_pose(q[k],station.BASES[side])[1]).as_quat().tolist())
        maximum_joint_step=max(maximum_joint_step,float(np.abs(q-old).max()))
        operations.append({'orientations':rotations,'targets':goals.tolist(),'steps':1})
    if maximum_joint_step>.08:raise RuntimeError('Table path has an abrupt joint change')
    return {'label':'two_arm_table_release','operations':operations}, {
        'steps':steps,'maximum_ik_error_m':maximum_error,
        'maximum_joint_increment_rad':maximum_joint_step,
        'scope':'Kinematic preflight, not collision or physics evidence.'}

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args()
    trace=json.loads((a.run/'trace.json').read_text())
    command,report=plan(trace[-1])
    (a.run/'table_transfer_plan.json').write_text(json.dumps(command))
    (a.run/'table_transfer_preflight.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report))
