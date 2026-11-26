"""Numeric state-based scoring; material estimates are exposed, never hidden goals."""
import math,numpy as np
from mechanism import U

def rotation(q):
 x,y,z,w=np.asarray(q,float);n=x*x+y*y+z*z+w*w
 if not np.isfinite(n)or n<=0:raise ValueError('invalid quaternion')
 s=2/n
 return np.array([[1-s*(y*y+z*z),s*(x*y-z*w),s*(x*z+y*w)],[s*(x*y+z*w),1-s*(x*x+z*z),s*(y*z-x*w)],[s*(x*z-y*w),s*(y*z+x*w),1-s*(x*x+y*y)]])

class Scorer:
 def __init__(self):self.last=0;self.play=False;self.hold=0;self.failure=None
 def update(self,step,objects,root_poses,robot,mechanics=None,**kwargs):
  m=mechanics or {};score=0.;valid=False;clear=False;proper=False
  try:
   required=['cam_angle_rad','cam_speed_rad_s','rear_q_m','rear_dq_m_s','axial_elastic_load_estimate_N','drag_torque_estimate_Nm','gap_fraction','robot_contact_impulse_Ns']
   if not all(k in m for k in required):raise ValueError('missing mechanics')
   vals=np.array([m[k]for k in required[:-1]],float)
   if not np.isfinite(vals).all()or not m.get('finite')or m.get('callback_error'):raise ValueError('invalid native measurement')
   valid=True;theta,speed,q,dq,load,drag,gap=vals
   impulses=m['robot_contact_impulse_Ns'];rear_imp=float(impulses['rear']);total=float(impulses['rear'])+float(impulses['cam'])
   if not np.isfinite([rear_imp,total]).all():raise ValueError('invalid contact')
   if load>U['load_limit_N']:self.failure='published_axial_load_limit_exceeded'
   if not -math.radians(42)<theta<math.radians(78):self.failure='cam_left_original_ramp_index'
   # An unloaded, positive down-ramp movement is useful partial work. Pressing
   # the rear nut directly cannot manufacture this partial-credit event.
   if theta>=math.radians(12)and load<=1. and q<=.00004 and rear_imp==0:self.play=True
   clear=total==0
   for side in ['left','right']:
    r=robot[side];tcp=np.asarray(r['tcp'],float);f=np.asarray(r['fingers'],float)
    if tcp.shape!=(3,)or f.shape!=(2,)or not np.isfinite(np.r_[tcp,f]).all():raise ValueError('invalid robot measurement')
    clear=bool(clear and np.linalg.norm(tcp-np.asarray(root_poses['cam'][:3])-np.array([0,0,.025]))>=.10 and min(abs(f))>=.03)
   proper=bool(U['drag_torque_Nm'][0]<=drag<=U['drag_torque_Nm'][1]and 1/3<=gap<=1/2 and abs(speed)<=.02 and abs(dq)<=.00005)
   self.hold=self.hold+1 if step==self.last+1 and proper and clear and not self.failure else 0
   if not self.failure:
    score=1. if self.hold>=40 else .5 if proper and clear else .25 if self.play else 0.
  except (KeyError,ValueError,TypeError,IndexError):self.hold=0
  if step!=self.last+1:self.hold=0;score=0.
  self.last=step
  if self.failure:score=0.
  return {'score':score,'success':score==1.,'failure':self.failure,'valid_measurement':valid,'unloaded_play_reached':self.play,'proper_preload':proper,'hands_clear':clear,'hold_frames':self.hold,'drag_torque_estimate_Nm':m.get('drag_torque_estimate_Nm'),'gap_fraction':m.get('gap_fraction'),'axial_elastic_load_estimate_N':m.get('axial_elastic_load_estimate_N')}
