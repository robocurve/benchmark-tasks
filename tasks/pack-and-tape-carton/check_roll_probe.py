"""Nominal full315 roll-pickup screen; explicit closed/untaped carton hypothesis.

No native contact, friction retention or film-end acquisition is certified.
"""
import json,math
from pathlib import Path
import numpy as np
from yam_arena import ik
from yam_arena.constants import READY,FINGER_OPEN
from yam_arena.urdf import quat_xyzw_to_matrix
from roll_probe import Geometry,grip_target,BASES,OPEN,HOLD
from config import *
from carton_scene import layout


def nominal_poses(lift=0.):
 poses={p['name']:(np.array(p['pos'])+CENTER,quat_xyzw_to_matrix([*p['quat'][1:],p['quat'][0]])) for p in layout(top_angle=0.)[0]}
 for n,p,_,_,_ in PRODUCTS:poses[n]=(np.array(p),np.eye(3))
 poses['roll']=(np.array(ROLL_START)+[0,0,lift],np.eye(3));poses['cutter']=(np.array(CUTTER_START),np.eye(3))
 for i in range(CELLS):p,y=tape_cell(i);c,s=math.cos(y),math.sin(y);poses[f'tape_{i}']=(np.array(p)+[0,0,lift],np.array([[c,-s,0],[s,c,0],[0,0,1]]))
 return poses


def check():
 geometry=Geometry();poses=nominal_poses();goal,R=grip_target(poses['roll']);start,S=ik.tcp_pose(READY,BASES['right']);q=np.array(READY);rows=[]
 for phase in ('approach','enter','close','lift'):
  for v in np.linspace(0,1,9):
   s=ik.smoothstep(v);lift=0.;f=OPEN
   if phase=='approach':p=start+s*(goal+[0,0,.04]-start);Q=ik.slerp_matrix(S,R,s);f=1+s*(OPEN-1)
   elif phase=='enter':p=goal+[0,0,.04*(1-s)];Q=R
   elif phase=='close':p=goal;Q=R;f=OPEN+s*(HOLD-OPEN)
   else:p=goal+[0,0,.06*s];Q=R;f=HOLD;lift=.06*s
   q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(Q),q,base_pos=BASES['right'],iters=240,tol=1e-6)
   row=dict(phase=phase,fraction=float(v),ik_position_error_m=pe,ik_rotation_error_rad=re)
   if pe>.0005 or re>.005:row.update(safe=False,reason='IK');rows.append(row);continue
   g=geometry.check([np.array(READY),q],[[-.04695]*2,[FINGER_OPEN*f]*2],nominal_poses(lift),contact=phase!='approach')
   row.update(g);rows.append(row)
  print(phase,sum(r['safe'] for r in rows if r['phase']==phase),'/9',flush=True)
 return rows

if __name__=='__main__':
 rows=check();path=Path('/private/tmp/packing-roll-pickup-nominal.json');path.write_text(json.dumps({'scope':'Nominal sampled36poses/full315; no continuous swept/native/contact proof','closed_untaped_carton':True,'rows':rows,'pass':all(r['safe'] for r in rows)},indent=2)+'\n');print(path)
