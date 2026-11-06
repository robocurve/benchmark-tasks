"""CPU-only trajectory check for the physical demo's nominal tool poses."""
import argparse
import json
from pathlib import Path
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY
from demo import CUP,FRONT,GRASP,DOWN,spoon_rotation


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--pitch',type=float,default=0);parser.add_argument('--lip-z',type=float,default=.0064)
    parser.add_argument('--pushed-ball-m',type=float,default=.060);parser.add_argument('--yaw',type=float,default=-45);parser.add_argument('--above-z',type=float,default=.075);args=parser.parse_args()
    layout=json.loads((Path(__file__).resolve().parent/'layout.json').read_text())
    top=layout['table_top_z'];clean=np.array(layout['objects']['CleanSpoon']['position'])
    ball=np.array(layout['objects']['Meatball_1']['position'])+[0,0,.018]
    plate=np.array(layout['objects']['Plate']['position']);entry_r=spoon_rotation(args.yaw,args.pitch)
    long=spoon_rotation(args.yaw,0)[:,0]
    lip=ball-long*.025;lip[2]=top+args.lip_z;entry=lip+entry_r@(CUP-FRONT)
    above=entry.copy();above[2]=top+args.above_z
    centered=ball+long*(args.pushed_ball_m-.008);centered[2]=max(entry[2]+.006,top+.060)
    lift=ball+long*args.pushed_ball_m;lift[2]=top+.095
    goals=[('above_clean_handle',clean+GRASP+[0,0,.13],DOWN),
           ('descend_to_handle',clean+GRASP,DOWN),
           ('lift_clean_spoon',clean+GRASP+[0,0,.11],DOWN)]
    for label,cup,r in [('above_source_bowl',above,entry_r),
                       ('lower_scoop_lip',entry,entry_r),
                       ('scoop_under_meatball',entry+long*.095,entry_r),
                       ('curl_meatball_into_scoop',centered,spoon_rotation(args.yaw,-15)),
                       ('level_and_lift_meatball',lift,spoon_rotation(args.yaw,-15)),
                       ('carry_to_empty_plate',plate+[0,0,.090],spoon_rotation(0,-10)),
                       ('pour_one_meatball',plate+[0,0,.060],spoon_rotation(45,75)),
                       ('withdraw_spoon',plate+[.07,.08,.095],spoon_rotation(45,0)),
                       ('return_spoon_above_clean_plate',clean+CUP+[0,0,.12],np.eye(3)),
                       ('lower_spoon_to_clean_plate',clean+CUP+[0,0,.002],np.eye(3))]:
        goals.append((label,cup+r@(GRASP-CUP),r@DOWN))
    q=np.array(READY,float);start,r0=ik.tcp_pose(q,station.BASES['left']);reports=[]
    for label,p,r in goals:
        errors=[]
        for t in np.linspace(0,1,41)[1:]:
            t=ik.smoothstep(t);rr=ik.slerp_matrix(r0,r,t)
            if label in ('above_clean_handle','descend_to_handle','lift_clean_spoon'):
                target=start+t*(p-start)
            else:
                start_cup=start+(r0@DOWN.T)@(CUP-GRASP)
                goal_cup=p+(r@DOWN.T)@(CUP-GRASP)
                target=start_cup+t*(goal_cup-start_cup)+(rr@DOWN.T)@(GRASP-CUP)
            q,pe,re=ik.solve(target,ik.matrix_to_quat_xyzw(rr),q,base_pos=station.BASES['left'],iters=150,damping=.03)
            errors.append((pe,float(np.degrees(re))))
        row={'phase':label,'position_error_m':max(v[0] for v in errors),'rotation_error_deg':max(v[1] for v in errors)}
        row['passed']=row['position_error_m']<.003 and row['rotation_error_deg']<3
        reports.append(row);print(row,flush=True);start,r0=p,r
    args.out.parent.mkdir(parents=True,exist_ok=True)
    result={'passed':all(r['passed'] for r in reports),'targets':reports,
            'note':'Nominal kinematics only; no collision-free path or scoop success is established by this check.'}
    args.out.write_text(json.dumps(result,indent=2)+'\n')
    raise SystemExit(0 if result['passed'] else 2)


if __name__=='__main__':main()
