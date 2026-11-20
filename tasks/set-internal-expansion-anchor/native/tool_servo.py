"""Reference-only pose feedback for a tool held in the hand, through ordinary joint targets.

The arm's position drives sag a few millimetres and degrees under a held tool. Before a blow, and before the tool
is laid back down, the reference reads the tool's actual pose and nudges the hand's joint targets until one point
of the tool and the tool's orientation match what the next motion expects. It never writes an object pose.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena import ik

class ToolServo:
 GAIN=.2;STEP_M=.001;STEP_RAD=float(np.deg2rad(.4))           # per control
 ARRIVE_M=.0015;ARRIVE_RAD=float(np.deg2rad(.8));QUIET_M=.0004
 MAX_BIAS_M=.045;MAX_BIAS_RAD=float(np.deg2rad(18));MAX_JOINT_STEP=.05
 def __init__(self,base,tool,point):
  self.base=np.asarray(base,float);self.tool=tool;self.point=np.asarray(point,float)
  self.delta=np.zeros(6);self.streak=0;self.last=-1;self.previous=None;self.latest={}
 def update(self,row,nominal,want_point,want_R):
  if row['control']<=self.last:raise ValueError('Tool servo needs a fresh actual sample')
  self.last=row['control'];nominal=np.asarray(nominal,float)
  pose=np.array(row['rigid_bodies'][self.tool]['pose_xyzw'],float)
  if pose.shape!=(7,)or not np.isfinite(pose).all():raise ValueError('Unknown actual tool pose')
  Rt=Rotation.from_quat(pose[3:]).as_matrix();at=pose[:3]+Rt@self.point
  ep=np.asarray(want_point,float)-at;er=Rotation.from_matrix(np.asarray(want_R)@Rt.T).as_rotvec()
  moved=None if self.previous is None else float(np.linalg.norm(at-self.previous));self.previous=at.copy()
  quiet=moved is not None and moved<self.QUIET_M
  arrived=float(np.linalg.norm(ep))<self.ARRIVE_M and float(np.linalg.norm(er))<self.ARRIVE_RAD
  self.streak=self.streak+1 if arrived and quiet else 0
  q=nominal+self.delta
  if not(arrived and quiet):
   dp=self.GAIN*ep;dr=self.GAIN*er
   if np.linalg.norm(dp)>self.STEP_M:dp*=self.STEP_M/np.linalg.norm(dp)
   if np.linalg.norm(dr)>self.STEP_RAD:dr*=self.STEP_RAD/np.linalg.norm(dr)
   p,R=ik.tcp_pose(q,self.base);S=Rotation.from_rotvec(dr).as_matrix()
   target=at+S@(p-at)+dp;Rtarget=S@R
   p0,R0=ik.tcp_pose(nominal,self.base)
   bias=float(np.linalg.norm(target-p0));rbias=float(Rotation.from_matrix(Rtarget@R0.T).magnitude())
   if bias>self.MAX_BIAS_M or rbias>self.MAX_BIAS_RAD:raise ValueError('Tool pose correction exceeded its bound')
   new,pe,re=ik.solve(target,ik.matrix_to_quat_xyzw(Rtarget),q,base_pos=self.base,iters=80,tol=1e-6,damping=.015)
   if pe>.0002 or re>.002:raise ValueError('Tool pose correction unreachable')
   if np.max(np.abs(new-q))>self.MAX_JOINT_STEP:raise ValueError('Tool pose correction joint step too large')
   q=new;self.delta=q-nominal
  self.latest=dict(actual_control=row['control'],tool=self.tool,point_error_m=ep.tolist(),rotation_error_rad=er.tolist(),
   fresh_good_streak=self.streak,actual_point_increment_m=moved,joint_bias_rad=self.delta.tolist(),no_object_pose_or_force_write=True)
  return q.copy(),self.latest
