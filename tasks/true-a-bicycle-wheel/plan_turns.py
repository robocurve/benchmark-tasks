"""Offline quarter-turn reach search from the published setup, not live state.

Only the robot's URDF and fixed nominal geometry enter this planning diagnostic.
No simulator, Dream call, assessor feedback or object pose correction is used.
Reachability is not evidence that the tool was grasped or that contact succeeds.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from mechanics import WheelSpec
from wheel_scene import x_frame,spoke_phase,wheel_world_frame,NIPPLE_CENTER_INSET_M,WRENCH_TCP_M


def nominal_nipple(spoke,flat_phase=0.,clocking=0.,fixture_layout='reference'):
    from fixture_layout import axle_position
    spec=WheelSpec();theta=spoke*2*math.pi/spec.spokes;side=1 if spoke%2==0 else -1
    anchor=np.array((spec.hub_flange_radius_m*math.cos(theta+side*spec.cross_angle_rad),
                     spec.hub_flange_radius_m*math.sin(theta+side*spec.cross_angle_rad),side*spec.hub_half_width_m))
    end=np.array((spec.rim_radius_m*math.cos(theta),spec.rim_radius_m*math.sin(theta),0.))
    wheel_R=wheel_world_frame(clocking)
    frame=x_frame(end-anchor)
    return axle_position(fixture_layout)+wheel_R@(end-frame[:,0]*NIPPLE_CENTER_INSET_M),wheel_R@frame@spoke_phase(flat_phase)


def rz(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array(((c,-s,0.),(s,c,0.),(0.,0.,1.)))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--flat-phase-deg',type=float,choices=(0.,45.),default=0.)
    parser.add_argument('--clocking-deg',type=float,choices=(0.,10.,20.,50.),default=0.)
    parser.add_argument('--spokes',type=int,nargs='+',default=[0,2,4,6])
    parser.add_argument('--arms',choices=('left','right'),nargs='+',default=['right'])
    args=parser.parse_args()
    from yam_arena import ik,station,YAM_URDF
    from yam_arena.constants import READY
    report={'kind':'nominal_kinematics_only','collision_validated':False,'task_score':None,
        'urdf_sha256':hashlib.sha256(Path(YAM_URDF).read_bytes()).hexdigest(),
        'ik_sha256':hashlib.sha256(Path(ik.__file__).read_bytes()).hexdigest(),
        'parameters':'Two tool faces, four square engagements, two across-handle grasps. Clockwise from rim tightens.',
        'nipple_flat_phase_deg':args.flat_phase_deg,'wheel_clocking_deg':args.clocking_deg,'candidates':[]}
    # The gripper closes along local X. Grip across the16mm handle width,
    # not its55mm length. These180-degree alternatives keep the same pinch.
    grips=(np.diag((1.,-1.,-1.)),np.diag((-1.,1.,-1.)))
    tcp_in_tool=np.array(WRENCH_TCP_M)
    for spoke in args.spokes:
        center,nipple=nominal_nipple(spoke,np.deg2rad(args.flat_phase_deg),np.deg2rad(args.clocking_deg))
        for arm in args.arms:
            for face in (1,-1):
                base=nipple[:,[1,2,0]]@(np.eye(3) if face==1 else np.diag((1.,-1.,-1.)))
                for quarter in range(4):
                    for grip_index,grip in enumerate(grips):
                        q=np.array(READY);rows=[]
                        for angle in np.linspace(0.,-face*math.pi/2,19):
                            tool_R=base@rz(quarter*math.pi/2+angle)
                            position=center+tool_R@tcp_in_tool
                            q,pe,re=ik.solve(position,ik.matrix_to_quat_xyzw(tool_R@grip),q,station.BASES[arm],
                                iters=250,damping=.008,tol=1e-7)
                            rows.append(dict(angle_deg=float(np.degrees(angle)),q=q.tolist(),position_error_m=pe,
                                rotation_error_rad=re,tcp_position_m=position.tolist(),tcp_rotation=(tool_R@grip).tolist()))
                            if pe>.002 or re>.02:break
                        passed=len(rows)==19 and max(r['position_error_m'] for r in rows)<.0001 and max(r['rotation_error_rad'] for r in rows)<.001
                        report['candidates'].append(dict(spoke=spoke,arm=arm,face=face,quarter=quarter,grip=grip_index,
                            complete_quarter_turn_reachable=passed,rows=rows))
        args.out.parent.mkdir(parents=True,exist_ok=True)
        args.out.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'candidates':len(report['candidates']),
        'reachable':[{k:v for k,v in c.items() if k!='rows'} for c in report['candidates'] if c['complete_quarter_turn_reachable']]}))


if __name__=='__main__':main()
