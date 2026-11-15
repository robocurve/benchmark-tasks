"""Fixed, precompiled ripper pickup and first-tack approach diagnostic.

Only URDF kinematics and the declared reset layout enter this plan. No live
object pose, thread state or simulator measurement can change its actions.
This is not a validated unpicking policy or a scored demonstration.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from geometry import TABLE_TOP
from tool_geometry import BLADE_OFFSET

DOWN=np.array([[0.,-1.,0.],[-1.,0.,0.],[0.,0.,-1.]])


def rz(a):
    c,s=np.cos(a),np.sin(a)
    return np.array(((c,-s,0.),(s,c,0.),(0.,0.,1.)))


def rx(a):
    c,s=np.cos(a),np.sin(a)
    return np.array(((1.,0.,0.),(0.,c,-s),(0.,s,c)))


def compile_plan():
    from yam_arena import ik,station
    from yam_arena.constants import READY,FINGER_OPEN
    state={side:{'q':np.asarray(READY,float),'finger':FINGER_OPEN} for side in ('left','right')}
    frames=[];position_error=rotation_error=0.
    def transition(seconds,label,goals):
        nonlocal position_error,rotation_error
        starts={side:(*ik.tcp_pose(s['q'],station.BASES[side]),s['finger']) for side,s in state.items()}
        for step in range(1,round(seconds*20)+1):
            w=ik.smoothstep(step/round(seconds*20));action=[]
            for side,s in state.items():
                p0,r0,f0=starts[side];p1,r1,f1=goals.get(side,(p0,r0,f0))
                p=p0+(np.asarray(p1)-p0)*w;r=ik.slerp_matrix(r0,np.asarray(r1),w)
                q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(r),s['q'],station.BASES[side],iters=200,tol=1e-6)
                position_error=max(position_error,pe);rotation_error=max(rotation_error,re)
                if pe>.001 or re>np.deg2rad(.5):
                    raise RuntimeError(f'Unreachable fixed trajectory: {label} {side}: {pe:.6f} m, {re:.6f} rad')
                s['q']=q;s['finger']=f0+(f1-f0)*w
                action.extend([*q.tolist(),float(np.clip(s['finger']/FINGER_OPEN,0,1))])
            frames.append({'phase':label,'action14':action})
    centre=np.array((.200,-.120,TABLE_TOP+.008));offset=np.array((0.,0.,.008))
    # The handle lies along X; DOWN closes pads across its 12 mm Y width.
    transition(2.,'approach ripper',{'right':(centre+[0,0,.08],DOWN,FINGER_OPEN)})
    transition(2.,'lower around handle',{'right':(centre+offset,DOWN,FINGER_OPEN)})
    transition(1.,'grasp handle',{'right':(centre+offset,DOWN,-.0055)})
    transition(2.,'lift ripper',{'right':(centre+offset+[0,0,.08],DOWN,-.0055)})
    # The fork starts vertical within the square handle's footprint. Turn the
    # shaft across the tack and pitch down 5 degrees without rolling the broad
    # gripper palm sideways into the table. This remains a high hover only.
    turn=rx(-np.deg2rad(5.))@rz(np.pi/2)
    transition(2.,'turn cutting edge across tack',{'right':(centre+offset+[0,0,.08],turn@DOWN,-.0055)})
    # Keep the leading point behind the first tack before any later descent.
    # Transform the grasp offset with the tool; it is not always world +Z.
    blade=np.array((.300,-.030-.016,TABLE_TOP+.0048+.012))
    approach=blade-turn@np.asarray(BLADE_OFFSET)+turn@offset
    transition(3.,'approach first tack above fabric',{'right':(approach,turn@DOWN,-.0055)})
    transition(2.,'hold for native contact inspection',{})
    return {'schema':'fixed-joint-plan-v1','control_hz':20,'no_object_feedback':True,
            'status':'unvalidated pickup and approach only','frames':frames,
            'tool_fork_plane':'XZ; long point below blade at reset',
            'approach_pitch_degrees':5.,'blade_hover_xyz_m':blade.tolist(),
            'maximum_ik_position_error_m':position_error,'maximum_ik_rotation_error_rad':rotation_error,
            'plan_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    result=compile_plan();a.out.parent.mkdir(parents=True,exist_ok=True)
    a.out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='frames'}))


if __name__=='__main__':main()
