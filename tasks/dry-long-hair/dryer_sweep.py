"""Bounded upper-layer dryer raster; a diagnostic motion, not dry-success logic.

The measured rigid grasp maps ordinary LEFT TCP commands to a desired dryer
pose. The physical head/hair are never commanded. Exact command-space entry
absorbs the already-issued motor target once; no extra Cartesian bias is added.
"""
import numpy as np
from yam_arena import ik
from yam_arena.urdf import quat_xyzw_to_matrix,rotation_angle_deg
from config import HEAD,HEAD_QUAT

NOZZLE_LOCAL=np.array((.088,0.,0.))
MAX_SWEEP_SPEED=.02
MAX_SWEEP_ROTATION_DEG_S=10.


def pose(value):
 p,r=value
 p,r=np.asarray(p,dtype=float),np.asarray(r,dtype=float)
 if p.shape!=(3,) or r.shape!=(3,3) or not np.isfinite(p).all() or not np.isfinite(r).all():raise ValueError('finite rigid pose required')
 if not np.allclose(r.T@r,np.eye(3),atol=1e-6) or abs(np.linalg.det(r)-1)>1e-6:raise ValueError('proper rotation required')
 return p.copy(),r.copy()


class DryerSweepPlan:
 def __init__(self,issued_hand,measured_hand,measured_dryer):
  hp,hr=pose(measured_hand);dp,dr=pose(measured_dryer);cp,cr=pose(issued_hand)
  self.offset=dr.T@(hp-dp);self.rotation=dr.T@hr
  # This is an implied COMMAND reference, not an invented measured body pose.
  r0=cr@self.rotation.T;p0=cp-r0@self.offset
  self.entry_hand=(cp,cr);self.entry_dryer=(p0,r0)
  headR=quat_xyzw_to_matrix(HEAD_QUAT)
  axis=headR@np.array((-1.,0.,0.));side=headR@np.array((0.,1.,0.));up=np.array((0.,0.,1.))
  targetR=np.column_stack((axis,np.cross(up,axis),up))
  self.ends=[];start=self.entry_dryer;self.duration=0.
  # Reach-checked upper two bands only. Lower/inner layers need a separate
  # measured manipulation/access plan; no timer marks them dry.
  for height,lateral in ((.02,.055),(.02,-.055),(-.03,-.055),(-.03,.055)):
   nozzle=np.asarray(HEAD)-.20*axis+height*up+lateral*side
   end=(nozzle-targetR@NOZZLE_LOCAL,targetR)
   angle=np.radians(rotation_angle_deg(start[1],end[1]))
   distance=float(np.linalg.norm(end[0]-start[0]))+max(.14,float(np.linalg.norm(self.offset)))*angle
   duration=max(1.5*distance/MAX_SWEEP_SPEED,
                1.5*rotation_angle_deg(start[1],end[1])/MAX_SWEEP_ROTATION_DEG_S,.05)
   self.ends.append((start,end,duration));self.duration+=duration;start=end

 def target(self,clock):
  if not np.isfinite(clock) or clock<0:raise ValueError('finite nonnegative path clock required')
  if clock==0:return tuple(v.copy() for v in self.entry_hand)
  remaining=clock
  for start,end,duration in self.ends:
   s=ik.smoothstep(min(1.,remaining/duration))
   r=ik.slerp_matrix(start[1],end[1],s);p=start[0]+s*(end[0]-start[0])
   if remaining<=duration:break
   remaining-=duration
  return p+r@self.offset,r@self.rotation

 def grasp_error(self,measured_hand,measured_dryer):
  hp,hr=pose(measured_hand);dp,dr=pose(measured_dryer)
  return float(np.linalg.norm(dr.T@(hp-dp)-self.offset)),rotation_angle_deg(dr.T@hr,self.rotation)


def bounds_gap(first,second):
 """Conservative distance of closed mesh/primitive AABBs, no false clearance."""
 amin,amax=first;bmin,bmax=second
 delta=np.maximum(np.asarray(amin)[:,None]-np.asarray(bmax)[None,:],np.asarray(bmin)[None,:]-np.asarray(amax)[:,None])
 return float(np.linalg.norm(np.maximum(delta,0.),axis=-1).min())


def tool_and_hand_bounds(hand_triangles,dryer_pose):
 p,r=pose(dryer_pose)
 mins=[a.min((0,1)) for a in hand_triangles];maxs=[a.max((0,1)) for a in hand_triangles]
 # Boxes enclose the unmodified native cylindrical barrel/nozzle and handle.
 for center,size in (((0,0,0),(.09,.058,.058)),((.065,0,0),(.045,.036,.036)),((-.019,0,-.065),(.030,.026,.085)),((-.037,0,-.043),(.006,.013,.018))):
  corners=np.array([(x,y,z) for x in (-1,1) for y in (-1,1) for z in (-1,1)])*np.asarray(size)/2+center
  world=corners@r.T+p;mins.append(world.min(0));maxs.append(world.max(0))
 return np.array(mins),np.array(maxs)
