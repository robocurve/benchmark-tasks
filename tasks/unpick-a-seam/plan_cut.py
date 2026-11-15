"""Frozen first-tack cutting candidate, unrun and unscored.

Append a physical left-hand press and one slow blade feed to the pickup/hover
plan. Uses only URDF kinematics, the declared pre-drape envelope and a fixed
offline grasp calibration from native coupon 029. No live object feedback.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from geometry import TABLE_TOP
from tool_geometry import BLADE_OFFSET,GUARDS
from plan import DOWN,rx,rz

HERE=Path(__file__).resolve().parent
# Native coupon 029 actions 140--159, rounded to 0.01 mm. Held component ranges
# were below 1.2 micrometres. This is an offline tool calibration, not a sensor
# read or online adjustment in the policy.
GRASP_OFFSET=np.array((-.00010,.00003,.01002))


def compile_cut_plan(pickup):
    from yam_arena import ik,station
    from yam_arena.constants import FINGER_OPEN,READY,SHOULDER,WRIST,GRIPPER
    if pickup.get('control_hz')!=20 or pickup.get('no_object_feedback') is not True:
        raise ValueError('Expected the frozen pickup/hover plan')
    frames=[dict(row) for row in pickup['frames']]
    if len(frames)!=280:raise ValueError('Expected full 280-action high-hover prefix')
    state={side:{'q':np.asarray(frames[-1]['action14'][index:index+6]),
                 'finger':frames[-1]['action14'][index+6]*FINGER_OPEN}
           for side,index in (('left',0),('right',7))}
    position_error=rotation_error=0.
    def transition(seconds,label,goals):
        nonlocal position_error,rotation_error
        starts={side:(*ik.tcp_pose(s['q'],station.BASES[side]),s['finger']) for side,s in state.items()}
        count=round(seconds*20)
        for step in range(1,count+1):
            w=ik.smoothstep(step/count);action=[]
            for side,s in state.items():
                p0,r0,f0=starts[side];p1,r1,f1=goals.get(side,(p0,r0,f0))
                p=p0+(np.asarray(p1)-p0)*w;r=ik.slerp_matrix(r0,np.asarray(r1),w)
                q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(r),s['q'],station.BASES[side],iters=200,tol=1e-6)
                if pe>.001 or re>np.deg2rad(.5):raise RuntimeError(f'Unreachable {label}: {side} {pe}, {re}')
                position_error=max(position_error,pe);rotation_error=max(rotation_error,re)
                s['q']=q;s['finger']=f0+(f1-f0)*w
                action.extend([*q.tolist(),float(np.clip(s['finger']/FINGER_OPEN,0.,1.))])
            frames.append({'phase':label,'action14':action})
    # Closed finger tips press the upper-left patch against the ordinary table.
    # Complete URDF fingertip bounds end 0.358 mm above the table: a shallow
    # 0.042 mm nominal compression of the 0.4 mm sheet. The earlier 0.058 mm
    # gap would overconstrain its two 0.2 mm contact-rest distances. Contact,
    # force and cloth response still need native measurement; none is imposed.
    press=np.array((.255,.020,TABLE_TOP+.0132))
    transition(2.,'position left hand above cloth',{'left':(press+[0.,0.,.060],DOWN,0.)})
    transition(2.,'press cloth with closed left fingertips',{'left':(press,DOWN,0.)})
    transition(.5,'hold physical cloth press',{})
    turn=rx(-np.deg2rad(2.))@rz(np.pi/2)
    blade=np.array((.300,-.046,TABLE_TOP+.012))
    def tcp(edge):return edge-turn@np.asarray(BLADE_OFFSET)+turn@GRASP_OFFSET
    transition(1.,'set shallow blade approach',{'right':(tcp(blade),turn@DOWN,-.0055)})
    guard_low=min((turn@np.array((x-BLADE_OFFSET[0],0.,z)))[2]-radius
                  for start,length,z,radius in GUARDS for x in (start,start+length))
    # The pre-drape's highest surface is below 1.71 mm. A fixed 1.75 mm envelope
    # plus 0.30 mm clearance keeps the long point above it before the feed.
    blade[2]=TABLE_TOP+.00175+.00030-guard_low
    transition(2.,'lower leading point behind first tack',{'right':(tcp(blade),turn@DOWN,-.0055)})
    blade[1]=-.027
    transition(2.,'feed blade through first tack',{'right':(tcp(blade),turn@DOWN,-.0055)})
    transition(.5,'hold blade at end of feed',{})
    blade[2]+=.030
    transition(2.,'lift tool clear of fabric',{'right':(tcp(blade),turn@DOWN,-.0055)})
    transition(1.,'release physical cloth press',{'left':(press+[0.,0.,.060],DOWN,0.)})
    actions=np.asarray([row['action14'] for row in frames])
    ready=np.tile(np.r_[READY,1.],2)
    limits=np.tile([SHOULDER['velocity']/20]*3+[WRIST['velocity']/20]*3+
                   [GRIPPER['velocity']/abs(FINGER_OPEN)/20],2)
    ratio=np.abs(np.diff(np.vstack((ready,actions)),axis=0))/limits
    if np.max(ratio)>1.001:raise RuntimeError('Cutting plan exceeds ordinary action rate limits')
    return {'schema':'fixed-joint-plan-v1','status':'unrun first-tack probe; no score or removal claim',
            'control_hz':20,'no_object_feedback':True,'frames':frames,
            'maximum_ik_position_error_m':position_error,'maximum_ik_rotation_error_rad':rotation_error,
            'maximum_action_rate_fraction':float(ratio.max()),'grasp_offset_in_tool_frame_m':GRASP_OFFSET.tolist(),
            'calibration_evidence':'evidence/native-controls-029.json','requires_pre_draped_seam':True,
            'requires_junction_audit':True,
            'physical_cloth_press_tcp_m':press.tolist(),'feed_blade_height_above_table_m':float(.00175+.00030-guard_low),
            'feed_distance_m':.019,'approach_pitch_degrees':2.,
            'pickup_plan_sha256':hashlib.sha256(json.dumps(pickup,sort_keys=True).encode()).hexdigest(),
            'plan_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pickup-plan',type=Path,default=HERE/'evidence/pickup-plan.json')
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();result=compile_cut_plan(json.loads(args.pickup_plan.read_text()))
    args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({key:value for key,value in result.items() if key!='frames'}))
