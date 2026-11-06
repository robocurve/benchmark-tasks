"""Nominal first-link handoff and tip-rounding collision/IK screen."""
from pathlib import Path
import json,numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY
from chain_task.geometry import *
from chain_task.motion import *
from collisions import *
ROOT=Path(__file__).resolve().parents[1];d=layout(17)['link1'];p,r=d['position'],d['rotation'];tcp,tool=grasp(p,r,1,'long');off=r.T@(tcp-p);rel=tool.T@r;center=np.array([.30,0,.90]);right=Plan(pickup(p,r,1,'long')+[Target(5,center+r@off,r@rel.T,0.,'present'),Target(2,center+r@off,r@rel.T,0.,'settle')],'right');records={};error=None;pickup_hits=[]
for t in np.arange(.1,right.duration+.001,.1):
 try:ra,label=right.action7(t)
 except RuntimeError as exc:error=str(exc);break
 shapes=robot(ra[:6],max(ra[6],.0644/.0939) if t>=6 else ra[6],'right');ht,hr=ik.tcp_pose(ra[:6],station.BASES['right']);br=hr@rel;bp=ht-br@off
 if t<6:bp,br=p,r
 obs=static_scene(ROOT)+asset('frame',ROOT/'assets/frame1.usda',bp,br)+asset('gate',ROOT/'assets/gate.usda',bp+br@GATE_OFFSET,br);allowed={(tip,'frame') for tip in ['tip_left','tip_right']} if t>=4.5 else set();own=against(shapes,obs,allowed);inter=cross(shapes,robot(np.array(READY),1.,'left'))
 if own or inter:pickup_hits.append((float(t),label,own,inter))
records['pickup_present']=dict(error=error,position_error=right.worst_position_error,rotation_error=right.worst_rotation_error,hits=pickup_hits)
rq=right.q.copy();rp,rr=ik.tcp_pose(rq,station.BASES['right']);right_release=Plan([Target(2,rp,rr,1.,'release'),Target(3,rp-rr[:,2]*.02+[0,-.14,0],rr,1.,'withdraw'),Target(3,*ik.tcp_pose(np.array(READY),station.BASES['right']),1.,'ready')],'right',rq,.0644/.0939)
M=np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])@np.diag([-1.,-1.,1.]);lt=r@M;lp=center+r@np.array([-.020,0,0]);loff=np.array([-.020,0,0]);rd=rz(np.deg2rad(-10))@rx(np.pi/2);rod=np.array([.42,0,TABLE+.2225]);pre=rod-rd@np.array([.022,.020,0]);targets=[Target(3,lp-lt[:,2]*.07,lt,.15,'approach spine'),Target(3,lp,lt,.15,'enter'),Target(2,lp,lt,0.,'pinch'),Target(10,lp,lt,0.,'handoff hold')]
for seconds,bp,br,label in [(4,np.array([.32,.12,.98]),rd,'orient'),(2,np.array([.32,.12,.98]),rd,'settle'),(4,pre+[0,.09,0],rd,'approach eye'),(4,pre,rd,'align'),(2,pre,rd,'settle')]:targets.append(Target(seconds,bp+br@loff,br@M,0.,label))
for seconds,local,label in [(4,[.007,.020,0],'press'),(3,[.007,.024,0],'round tip'),(3,[.002,.024,0],'enter frame'),(3,[.002,.024,0],'close')]:targets.append(Target(seconds,rod-rd@np.array(local)+rd@loff,rd@M,0.,label))
left=Plan(targets,'left');hits=[];error=None
for t in np.arange(.1,left.duration+.001,.1):
 try:a,label=left.action7(t)
 except RuntimeError as exc:error=str(exc);break
 ls=robot(a[:6],max(a[6],.0045/.0939),'left');ht,hr=ik.tcp_pose(a[:6],station.BASES['left']);br=hr@M.T;bp=ht-br@loff
 if t<6.:bp,br=center,r
 obs=static_scene(ROOT)+asset('frame',ROOT/'assets/frame1.usda',bp,br)+asset('gate',ROOT/'assets/gate.usda',bp+br@GATE_OFFSET,br)
 allowed={(n,'frame') for n in ['tip_left','tip_right']} if t>=5.5 else set();own=against(ls,obs,allowed)
 if t<10:rs=robot(rq,.0644/.0939,'right')
 elif t<=18:
  try:ra,_=right_release.action7(t-10)
  except RuntimeError as exc:error='right release '+str(exc);break
  rs=robot(ra[:6],max(ra[6],.0644/.0939),'right');own+=against(rs,static_scene(ROOT))
 else:rs=robot(np.array(READY),1.,'right')
 inter=cross(ls,rs)
 if own or inter:hits.append((float(t),label,own,inter))
records['handoff_clip']=dict(error=error,position_error=left.worst_position_error,rotation_error=left.worst_rotation_error,hits=hits,right_release_errors=[right_release.worst_position_error,right_release.worst_rotation_error]);print(json.dumps(records,indent=2))
