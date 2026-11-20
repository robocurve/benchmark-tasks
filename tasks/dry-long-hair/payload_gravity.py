"""Payload gravity feed-forward via ordinary joint-position targets.

The existing action term already compensates robot-link gravity. This helper
adds only a measured held payload; no actuator gains, efforts or object forces
are changed. The payload remains coupled through real gripper contacts.
"""
import numpy as np
from yam_arena import ik
from yam_arena.constants import SHOULDER,WRIST
STIFFNESS=np.array([SHOULDER['stiffness']]*3+[WRIST['stiffness']]*3)
EFFORT=np.array([SHOULDER['effort']]*3+[WRIST['effort']]*3)

def gravity_torque(q,mass,com_tcp,base):
 q=np.asarray(q,dtype=float);com=np.asarray(com_tcp,dtype=float)
 if q.shape!=(6,) or com.shape!=(3,) or not np.isfinite([*q,*com,mass]).all() or not 0<=mass<=1. or np.linalg.norm(com)>.25:raise ValueError('invalid measured payload')
 def height(joints):
  p,R=ik.tcp_pose(joints,base);return float((p+R@com)[2])
 eps=1e-5;tau=[]
 for i in range(6):
  step=np.zeros(6);step[i]=eps;tau.append(mass*9.81*(height(q+step)-height(q-step))/(2*eps))
 tau=np.asarray(tau)
 if np.any(np.abs(tau)>EFFORT-.5):raise ValueError('payload contribution alone exceeds actuator effort')
 return tau

def target_offset(q,mass,com_tcp,base,blend=1.):
 if not np.isfinite(blend) or not 0<=blend<=1:raise ValueError('invalid payload ramp')
 return blend*gravity_torque(q,mass,com_tcp,base)/STIFFNESS
