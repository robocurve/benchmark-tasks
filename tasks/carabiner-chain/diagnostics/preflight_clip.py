"""Nominal first-link trajectory. Actual simulation must verify its grasp."""
import json,os,numpy as np
from pathlib import Path
from chain_task.geometry import *
from chain_task.motion import *
from collisions import *
from yam_arena import ik,station
from yam_arena.constants import READY
ROOT=Path(__file__).resolve().parents[1];index=int(os.environ.get('CHAIN_LINK','2'));d=layout(17)['link'+str(index)];p,r=d['position'],d['rotation'];style=os.environ.get('CHAIN_GRASP_STYLE','arch');flip=int(os.environ.get('CHAIN_GRASP_FLIP','1' if style=='long' else '0'));grip,tool=grasp(p,r,flip,style=style);offset=r.T@(grip-p);relative=tool.T@r;s=1 if r[2,2]>0 else -1
rd=rz(np.deg2rad(-10))@rx(s*np.pi/2);pre=np.array([.42,0,TABLE+.2225-s*(.020 if style!='arch' else .005)])-rd@np.array([.022,0,0]);targets=pickup(p,r,flip,style=style)
for seconds,bp,label in [(4,np.array([.34,-.16,.94]),'orient'),(2,np.array([.34,-.16,.94]),'settle grip'),(4,pre+[0,-.09,0],'side'),(4,pre,'align'),(2,pre,'settle')]:targets.append(Target(seconds,bp+rd@offset,rd@relative.T,0.,label))
if style!='arch':
 rod=np.array([.42,0,TABLE+.2225])
 for seconds,local,label in [(4,[.007,.020,0],'press'),(3,[.007,.024,0],'round'),(3,[.002,.024,0],'enter'),(3,[.002,.024,0],'close')]:targets.append(Target(seconds,rod-rd@np.array(local)+rd@offset,rd@relative.T,0.,label))
else:targets.append(Target(6,pre+rd@np.array([.028,0,0])+rd@offset,rd@relative.T,0.,'clip'))
plan=Plan(targets,'right');rows=[];error=None;obs=static_scene(ROOT)
for t in np.arange(.1,plan.duration+.001,.1):
 try:a,label=plan.action7(t)
 except RuntimeError as exc:error=str(exc);break
 shapes=robot(a[:6],max(a[6],(.061 if style=='long' and t>=6 else .0045)/.0939),'right');hits=against(shapes,obs)
 if t>=6.:
  ht,hr=ik.tcp_pose(a[:6],station.BASES['right']);br=hr@relative;bp=ht-br@offset;own=asset('held_frame',ROOT/'assets'/('frame'+str(index)+'.usda'),bp,br)+asset('held_gate',ROOT/'assets/gate.usda',bp+br@GATE_OFFSET,br);hits+=against(shapes,own,{(tip,'held_frame') for tip in ['tip_left','tip_right']});hits+=[('other_arm',str(v)) for v in cross(shapes,robot(np.array(READY),1.,'left'))]
 if hits:rows.append((float(t),label,hits))
print(json.dumps(dict(error=error,position_error=plan.worst_position_error,rotation_error=plan.worst_rotation_error,stand_table_collisions=rows),indent=2))
