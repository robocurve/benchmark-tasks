"""Precompute diagnostic joint actions from declared waypoints, without a simulator.

Only robot kinematics are read. The generated file contains ordinary 14-D joint
commands and never supplies cloth poses, contacts, attachments or success flags.
A passing IK audit is not evidence that the robot grasps the fabric.
"""
import argparse
import json
from pathlib import Path
import numpy as np


def front_rotation(degrees=-30.,side='left'):
    angle=np.deg2rad(degrees)
    r=np.array([[np.sin(angle),0.,np.cos(angle)],
                     [0.,1.,0.],[-np.cos(angle),0.,np.sin(angle)]])
    return r@np.diag((-1.,-1.,1.)) if side=='right' else r

ROTATIONS={side:front_rotation(side=side) for side in ('left','right')}


def compile_grasp(grasp_x=.210,approach_depth=0.,grasp_targets=None):
    from yam_arena import ik, station
    from yam_arena.constants import READY, FINGER_OPEN, SHOULDER, WRIST, GRIPPER
    from scene import SUPPORT_BOARD_THICKNESS,PHYSICS_HZ
    targets=[]
    for side, sign in [('left',1),('right',-1)]:
        hem_z=.863 if side=='left' else .867
        pre_z=hem_z+.035
        # Preserve the pinch plane while taking a deeper bite along the actual
        # 30-degree downward finger-approach axis; horizontal motion alone
        # also moves across the closing plane.
        bite_x=grasp_x+float(approach_depth)*np.cos(np.deg2rad(30.))
        bite_z=hem_z-float(approach_depth)*np.sin(np.deg2rad(30.))
        bite_position=(bite_x,sign*.115,bite_z) if grasp_targets is None else tuple(grasp_targets[side])
        targets.append([
            (3.,(.1494,sign*.115,pre_z),.6,'joint approach above exposed hem'),
            (2.,(.1494,sign*.115,pre_z),.6,'hold clear approach pose'),
            (2.,bite_position,.6,'approach exposed hem'),
            (1.,bite_position,0.,'close fingers on hem'),
            (2.,(.1494,sign*.115,pre_z),0.,'pull clear of support'),
            (3.,(.260,sign*.115,1.100),0.,'lift flexible garment'),
            (2.,(.350,sign*.115,1.100),0.,'carry above clear support'),
            (2.,(.350,sign*.115,1.100),0.,'hold diagnostic lift'),
            (2.,(.350,sign*.115,1.100),1.,'release after diagnostic'),
            (2.,(.280,sign*.115,1.100),1.,'withdraw empty hand'),
        ])
    actions_by_arm=[]
    audits=[]
    for (side,_), schedule in zip([('left',1),('right',-1)],targets):
        q=np.array(READY,dtype=float)
        opening=1.
        values=[]
        for seconds, position, target_opening, label in schedule:
            target_rotation=front_rotation(0.,side) if label in ('lift flexible garment','carry above clear support','hold diagnostic lift','release after diagnostic','withdraw empty hand') else ROTATIONS[side]
            p0,r0=ik.tcp_pose(q,station.BASES[side])
            q0=q.copy()
            opening0=opening
            segment_pe=segment_re=0.
            joint_goal=None
            if label=='joint approach above exposed hem':
                joint_goal,pe,re=ik.solve(position,ik.matrix_to_quat_xyzw(target_rotation),
                    READY,base_pos=station.BASES[side],iters=400)
                if pe>.003 or re>.035:
                    raise ValueError(f'Initial fixed approach waypoint is unreachable: {side}, {pe:.5f} m, {re:.5f} rad')
            for step in range(1,round(seconds*20)+1):
                t=ik.smoothstep(step/(seconds*20))
                pos=p0+t*(np.asarray(position)-p0)
                rot=ik.slerp_matrix(r0,target_rotation,t)
                if joint_goal is not None:
                    q=q0+t*(joint_goal-q0)
                    pe=re=0.  # A joint-space approach has no Cartesian path constraint.
                else:
                    q,pe,re=ik.solve(pos,ik.matrix_to_quat_xyzw(rot),q,
                                     base_pos=station.BASES[side],iters=60)
                segment_pe=max(segment_pe,float(pe))
                segment_re=max(segment_re,float(re))
                opening=opening0+t*(target_opening-opening0)
                values.append(dict(action=np.r_[q,opening].tolist(),stage=label))
            audits.append(dict(side=side,stage=label,seconds=seconds,
                               target_xyz_m=list(position),max_position_error_m=segment_pe,
                               max_rotation_error_rad=segment_re,
                               interpolation='joint' if joint_goal is not None else 'Cartesian'))
        actions_by_arm.append(values)
    if len(actions_by_arm[0])!=len(actions_by_arm[1]):
        raise ValueError('Bimanual schedules differ in length')
    frames=[dict(action14=a['action']+b['action'],stage=a['stage'])
            for a,b in zip(*actions_by_arm)]
    action_array=np.asarray([np.r_[READY,1.,READY,1.]]+[f['action14'] for f in frames])
    limit=np.array([SHOULDER['velocity']/20]*3+[WRIST['velocity']/20]*3+
                   [GRIPPER['velocity']/abs(FINGER_OPEN)/20])*1.001
    max_delta=np.max(np.abs(np.diff(action_array,axis=0)),axis=0)
    if np.any(max_delta>np.tile(limit,2)):
        raise ValueError('Fixed trajectory exceeds normal joint speed limits')
    if max(a['max_position_error_m'] for a in audits)>.004 or max(a['max_rotation_error_rad'] for a in audits)>.04:
        raise ValueError('Fixed trajectory fails its 4 mm / 2.3 degree kinematic gate')
    return dict(schema='cloth-fixed-motion-v1',kind='diagnostic_not_submission_demo',
                control_hz=20,score=None,fixture_board_thickness_m=SUPPORT_BOARD_THICKNESS,physics_hz=PHYSICS_HZ,feedback='none; precomputed joint commands only',
                action_count=len(frames),frames=frames,kinematic_audit=audits,
                max_joint_delta=max_delta.tolist(),
                source_waypoints='Offline batch015 actual YAM reset hem,8 mm board/480 Hz; no online cloth feedback')


def load(path):
    data=json.loads(Path(path).read_text())
    if data.get('schema')!='cloth-fixed-motion-v1' or data.get('control_hz')!=20:
        raise ValueError('Unexpected motion schema or rate')
    frames=data.get('frames',[])
    actions=np.asarray([f['action14'] for f in frames],dtype=float)
    if actions.ndim!=2 or actions.shape[1]!=14 or not 1<=len(actions)<=2400 or not np.isfinite(actions).all():
        raise ValueError('A motion probe needs 1–2400 finite 14-D joint commands')
    if np.any(actions[:,[6,13]]<0.) or np.any(actions[:,[6,13]]>1.):
        raise ValueError('Grippers must be normalized to [0,1]')
    return data,actions


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True)
    args=p.parse_args()
    plan=compile_grasp()
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(plan,indent=2)+'\n')
    audit=plan['kinematic_audit']
    print(json.dumps(dict(actions=plan['action_count'],seconds=plan['action_count']/20,
        max_position_error_m=max(a['max_position_error_m'] for a in audit),
        max_rotation_error_rad=max(a['max_rotation_error_rad'] for a in audit)),indent=2))
