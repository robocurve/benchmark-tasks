"""Bounded reference-only measured alignment using ordinary stock joint targets."""
import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena import ik

class PunchAlignment:
 def __init__(self,G):
  self.G=G;self.q=None;self.origin=None;self.R0=None;self.last=-1;self.phase=None;self.streak=0;self.latest={};self.previous_shoulder=None
 def update(self,row,nominal,phase):
  if row['control']<=self.last:raise ValueError('Alignment needs fresh actual body poses')
  self.last=row['control']
  if phase!=self.phase:self.phase=phase;self.streak=0;self.previous_shoulder=None
  base=self.G['robot_bases_m']['left']
  if self.q is None:
   self.q=np.array(nominal,float);self.origin,self.R0=ik.tcp_pose(self.q,base)
  b=row['rigid_bodies'];ring=np.array(b['anchor_ring']['pose_xyzw']);punch=np.array(b['punch']['pose_xyzw'])
  if not np.isfinite(np.r_[ring,punch]).all():raise ValueError('Unknown actual alignment geometry')
  Rr=Rotation.from_quat(ring[3:]).as_matrix();Rp=Rotation.from_quat(punch[3:]).as_matrix()
  top=ring[:3]+Rr[:,2]*.0396875;shoulder=punch[:3]+Rp[:,2]*.0414875
  error=top-shoulder;error[2]=0.
  axis=np.cross(Rp[:,2],Rr[:,2]);angle=float(np.arctan2(np.linalg.norm(axis),np.dot(Rp[:,2],Rr[:,2])))
  rot=np.zeros(3)if np.linalg.norm(axis)<1e-10 else axis/np.linalg.norm(axis)*angle
  lateral=float(np.linalg.norm(error));arrived=lateral<.0003 and angle<np.deg2rad(.4)
  increment=None if self.previous_shoulder is None else float(np.linalg.norm(shoulder-self.previous_shoulder))
  quiet=increment is not None and increment<.000025
  self.previous_shoulder=shoulder.copy()
  self.streak=self.streak+1 if arrived and quiet else 0
  p,R=ik.tcp_pose(self.q,base)
  delta=.35*error
  if np.linalg.norm(delta)>.00015:delta*=.00015/np.linalg.norm(delta)
  dr=.35*rot
  # The .30mm arrival deadband is an internal reference policy; formal .4mm score is unchanged.
  # Hold an arrived target so measured stiction/actuator delay cannot wind up the integral.
  if arrived and quiet:delta=np.zeros(3);dr=np.zeros(3)
  if np.linalg.norm(dr)>np.deg2rad(.1):dr*=np.deg2rad(.1)/np.linalg.norm(dr)
  target=p+delta;target[2]=self.origin[2]
  Rtarget=Rotation.from_rotvec(dr).as_matrix()@R
  bias=float(np.linalg.norm(target-self.origin));rbias=float(Rotation.from_matrix(Rtarget@self.R0.T).magnitude())
  if bias>.006 or rbias>np.deg2rad(4):raise ValueError('Actual alignment exceeded ordinary bounded correction')
  q,pe,re=ik.solve(target,ik.matrix_to_quat_xyzw(Rtarget),self.q,base_pos=base,iters=160,tol=1e-6,damping=.015)
  if pe>.0001 or re>.001:raise ValueError('Actual alignment IK unreachable')
  if np.max(np.abs(q-self.q))>.04:raise ValueError('Alignment joint increment exceeds bounded slow motion')
  self.q=q
  self.latest=dict(actual_control=row['control'],phase=phase,lateral_m=lateral,axis_error_rad=angle,fresh_good_streak=self.streak,actual_shoulder_increment_m=increment,arrival_deadband_m=.0003,quiet_increment_limit_m=.000025,integrator_held_in_arrival_deadband=bool(arrived and quiet),
   commanded_TCP_bias_m=bias,commanded_rotation_bias_rad=rbias,axial_TCP_target_unchanged=True,no_object_pose_or_force_write=True)
  return self.q.copy(),self.latest

def collision_guard(row,q,G,side='left',held='punch'):
 """Actual scene/other-arm readback against every proposed convex hull of one arm; `held` is the tool in its hand."""
 other='right'if side=='left'else'left'
 from scipy.spatial import ConvexHull
 from scipy.optimize import linprog
 from yam_arena.analysis import world_hulls,fk_link_poses_world,link_hulls
 from yam_arena.constants import ALL_JOINTS
 targets=[]
 for part in G['parts']:
  body=np.array(row['rigid_bodies'][part['name']]['pose_xyzw']);R=Rotation.from_quat(body[3:]).as_matrix()
  for shape in part['geoms']:
   v=np.array(shape['vertices'])@R.T+body[:3];targets.append((part['name'],v,ConvexHull(v).equations))
 r=row['robots'][other];names=r['body_names'];poses=np.array(r['body_link_pose_w'])
 if len(names)!=9 or poses.shape!=(9,7)or not np.isfinite(poses).all():raise ValueError('Missing complete actual other arm')
 for name,pose in zip(names,poses):
  R=Rotation.from_quat(pose[3:]).as_matrix()
  for hull in link_hulls()[name]:
   v=hull.vertices@R.T+pose[:3];targets.append(('other_robot',v,np.c_[hull.planes[:,:3]@R.T,-hull.planes[:,3]-(hull.planes[:,:3]@R.T)@pose[:3]]))
 from yam_arena import station
 c=np.array(station.TABLE_CENTER);sz=np.array(station.TABLE_SIZE)
 v=np.array([c+[a,b,d]for a in[-sz[0]/2,sz[0]/2]for b in[-sz[1]/2,sz[1]/2]for d in[-sz[2]/2,sz[2]/2]])
 targets.append(('table',v,ConvexHull(v).equations))
 low=np.array([v.min(0)for _,v,_ in targets]);high=np.array([v.max(0)for _,v,_ in targets])
 fingers=row['robots'][side]['joint_positions'][-2:]
 hulls=world_hulls(fk_link_poses_world(dict(zip(ALL_JOINTS,list(q)+list(fingers))),G['robot_bases_m'][side]))
 checks=0
 for name,hull in hulls:
  v=hull.vertices;A=np.c_[hull.planes[:,:3],-hull.planes[:,3]]
  for j in np.flatnonzero(np.max(np.maximum(low-v.max(0),v.min(0)-high),axis=1)<0):
   part,_,E=targets[j]
   if name in ['tip_left','tip_right']and part==held:continue
   H=np.vstack([A,E]);result=linprog([0,0,0,-1],A_ub=np.c_[H[:,:3],np.linalg.norm(H[:,:3],axis=1)],b_ub=-H[:,3],bounds=[(None,None)]*4,method='highs');checks+=1
   if not result.success or result.x[3]>2e-5:raise ValueError('Actual correction collision '+side+' '+name+'/'+part)
 order={n:i for i,n in enumerate(['base','link1','link2','link3','link4','link5','gripper'])};order.update(tip_left=6,tip_right=6)
 lows=np.array([h.vertices.min(0)for _,h in hulls]);highs=np.array([h.vertices.max(0)for _,h in hulls])
 for j,(name,hull)in enumerate(hulls):
  for k in np.flatnonzero(np.max(np.maximum(lows[j+1:]-highs[j],lows[j]-highs[j+1:]),axis=1)<0)+j+1:
   name2,hull2=hulls[k]
   if abs(order[name]-order[name2])<=1:continue
   H=np.vstack([np.c_[hull.planes[:,:3],-hull.planes[:,3]],np.c_[hull2.planes[:,:3],-hull2.planes[:,3]]])
   result=linprog([0,0,0,-1],A_ub=np.c_[H[:,:3],np.linalg.norm(H[:,:3],axis=1)],b_ub=-H[:,3],bounds=[(None,None)]*4,method='highs');checks+=1
   if not result.success or result.x[3]>2e-5:raise ValueError('Actual correction self collision '+side+' '+name+'/'+name2)
 return dict(arm_hulls=len(hulls),actual_other_arm_hulls=sum(len(h)for h in link_hulls().values()),exact_overlap_LP=checks)
