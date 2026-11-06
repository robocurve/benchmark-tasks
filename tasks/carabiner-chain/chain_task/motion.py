"""Measured-body grasp targets and guarded Cartesian interpolation."""
from dataclasses import dataclass
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY
@dataclass
class Target:
 seconds:float
 pos:np.ndarray
 rot:np.ndarray
 grip:float
 label:str
class Plan:
 def __init__(self,targets,side,q=None,grip=1.):
  self.targets=targets;self.base=station.BASES[side];self.q=np.array(READY if q is None else q);self.start_p,self.start_r=ik.tcp_pose(self.q,self.base);self.start_g=grip;self.duration=sum(t.seconds for t in targets);self.worst_position_error=0.;self.worst_rotation_error=0.
 def action7(self,time):
  p0,r0,g0=self.start_p,self.start_r,self.start_g;label='done'
  for t in self.targets:
   if time<t.seconds:
    s=ik.smoothstep(time/t.seconds);p=p0+s*(t.pos-p0);r=ik.slerp_matrix(r0,t.rot,s);g=g0+s*(t.grip-g0);label=t.label;break
   time-=t.seconds;p0,r0,g0=t.pos,t.rot,t.grip
  else:p,r,g=p0,r0,g0
  self.q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(r),self.q,base_pos=self.base,iters=80);self.worst_position_error=max(pe,self.worst_position_error);self.worst_rotation_error=max(re,self.worst_rotation_error)
  if pe>.002 or re>.02:raise RuntimeError('IK guard '+label+f' position={pe} rotation={re}')
  return np.r_[self.q,g],label

def grasp(p,r,flip=0,style="arch"):
 s=1 if r[2,2]>0 else -1
 x=r[:,1] if style=='long' else r[:,0];z=np.array([0.,0.,-1.]);y=np.cross(z,x);tool=np.column_stack([x,y,z])@np.diag([-1.,-1.,1.]) if flip else np.column_stack([x,y,z])
 local=np.array([0.,-s*.032,0.]) if style=='arch' else np.array([-.016 if style=='spine' else -.006,0.,0.]);tcp=p+r@local+np.array([0,0,.0125]);return tcp,tool

def pickup(p,r,flip=0,style="arch"):
 tcp,tool=grasp(p,r,flip,style);opening={"arch":.55,"long":.85,"spine":.20}[style]
 return [Target(3,tcp+[0,0,.075],tool,opening,'above loose carabiner'),Target(2,tcp,tool,opening,'descend bottom arch'),Target(1,tcp,tool,0.,'pinch bottom arch'),Target(3,tcp+[0,0,.12],tool,0.,'lift carabiner'),Target(2,tcp+[0,0,.12],tool,0.,'observe physical grasp')]

def transition_grip(previous,targets,measured):
 """A blocked jaw's opening is not its commanded closure.

Continue a requested pinch across replans; otherwise the measured opening
would remove contact force at the beginning of a long hold.
"""
 return 0. if previous is not None and previous.targets[-1].grip==0. and targets[0].grip==0. else measured

def parent_normalization(pp,pr,hand_p,hand_r,yaw_degrees=-10.):
 """Robot targets rotating a held parent about its upper arch; no body writes."""
 from chain_task.geometry import rz,rx
 local=np.array([-.002,.030,0.]);pivot=pp+pr@local;goal=rz(np.deg2rad(yaw_degrees))@rx(np.pi/2);off=pr.T@(hand_p-pp);relative=hand_r.T@pr;targets=[]
 for fraction in np.linspace(0.,1.,41)[1:]:
  s=ik.smoothstep(float(fraction));r=ik.slerp_matrix(pr,goal,s);p=pivot+np.array([0,0,.0015*s])-r@local;targets.append(Target(.1,p+r@off,r@relative.T,0.,'stabilize parent through robot contact'))
 targets.append(Target(2,targets[-1].pos,targets[-1].rot,0.,'settle stabilized parent'));return targets,p,goal
