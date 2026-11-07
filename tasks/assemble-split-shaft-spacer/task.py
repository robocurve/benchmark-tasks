"""Fit both blue halves of the spacer collar around the captive square shaft below its wide head. Bring the halves together from opposite sides. Leave both seams closed to within 4 mm. Release both halves and keep the grey shaft fixed. The fixture is unpowered."""
import math
import numpy as np
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,PAD_TIP_PAST_TCP as D,FINGER_JOINTS
from yam_articulated import Compound
from yam_arena.urdf import quat_xyzw_to_matrix
SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=2.;SCORE_OVER='end'
def p(pos,size):return dict(pos=pos,size=size)
SHAFT=Compound('shaft',pos=(.28,0,TOP),size=(.107,.30,.072),color=(.65,.65,.65),parts=(p((0,0,.029),(.020,.020,.058)),p((0,0,.065),(.070,.070,.014)),p((-.0475,0,.007),(.012,.30,.014)),p((.0475,0,.007),(.012,.30,.014))))
HALVES=[]
for sign,name in [(1,'left_half'),(-1,'right_half')]:
 HALVES.append(Compound(name,pos=(.16,sign*.16,TOP+.010),size=(.080,.115,.035),mass=.055,color=(.12,.35,.85),parts=(p((0,sign*.026,0),(.080,.018,.020)),p((-.031,sign*.015,0),(.018,.030,.020)),p((.031,sign*.015,0),(.018,.030,.020)),p((0,sign*.060,.0025),(.026,.070,.025)),p((0,sign*.095,.015),(.026,.040,.020)))))
OBJECTS=[SHAFT,*HALVES]
_bad=False;_open=False
def reset_history():
 global _bad,_open
 _bad=False;_open=False
def observe(states,t,base):
 global _bad,_open
 if any(states[n].low[2]<TOP-.005 for n in ['left_half','right_half']):_bad=True
 _open=all(float(np.max(np.abs(base.scene[n].data.joint_pos.torch[0,base.scene[n].find_joints(FINGER_JOINTS,preserve_order=True)[0]].detach().cpu().numpy())))>.035 for n in ['left_robot','right_robot'])
def root_position(s,sign):
 rotation=quat_xyzw_to_matrix(s.quat_xyzw)
 spec=HALVES[0 if sign==1 else 1]
 signs=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
 corners=np.concatenate([signs*np.asarray(part['size'])/2+part['pos'] for part in spec.parts])@rotation.T
 return s.pos-(corners.min(axis=0)+corners.max(axis=0))/2

def fitted(s,sign):
 q=np.asarray(s.quat_xyzw)
 if not all(np.isfinite(v).all() for v in [s.pos,q,s.lin_vel,s.ang_vel]):return False
 x,y,z,w=q;yaw=abs(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
 if not (abs(s.low[2]-TOP)<.003 and 1-2*(x*x+y*y)>math.cos(math.radians(4)) and yaw<math.radians(4) and s.resting(.008,.1)):return False
 rotation=quat_xyzw_to_matrix(q);root=root_position(s,sign)
 corners=np.array([[.28+dx,dy,TOP+.010] for dx in [-.010,.010] for dy in [-.010,.010]])
 local=(corners-root)@rotation
 return bool(np.max(np.abs(local[:,0]))<=.021 and np.max(sign*local[:,1])<=.016 and np.min(sign*local[:,1])>=-.030)
def seams_closed(states):
 ends=[]
 for sign,name in [(1,'left_half'),(-1,'right_half')]:
  s=states[name];rotation=quat_xyzw_to_matrix(s.quat_xyzw);root=root_position(s,sign)
  ends.append(np.array([root+rotation@np.array([x,0,0]) for x in [-.031,.031]]))
 return bool(np.max(np.linalg.norm(ends[0]-ends[1],axis=1))<=.004)
def score(states):
 if _bad:return 0.
 count=sum(fitted(states[n],s) for s,n in [(1,'left_half'),(-1,'right_half')])
 return (1. if _open else .75) if count==2 and seams_closed(states) else .4 if count else 0.
def success(states):return score(states)==1.
def distance(states):return sum(float(np.linalg.norm(states[n].pos-np.array([.28,s*.0575,TOP+.0175]))) for s,n in [(1,'left_half'),(-1,'right_half')])
def demo():
 result={}
 for sign,arm in [(1,'left'),(-1,'right')]:
  place_x=.2805
  yaw=0.;pick=(.16,sign*.255,TOP+.021+D);entry=(place_x,sign*.220,TOP+.022+D);end=(place_x,sign*.092,TOP+.022+D)
  result[arm]=[Waypoint(1.5,(pick[0],pick[1],TOP+.14),O,yaw=yaw),Waypoint(1.5,pick,O,yaw=yaw),Waypoint(.8,pick,C,yaw=yaw),Waypoint(1.2,(pick[0],pick[1],TOP+.14),C,yaw=yaw),Waypoint(1.8,(entry[0],entry[1],TOP+.14),C,yaw=yaw),Waypoint(1.5,entry,C,yaw=yaw),Waypoint(4.,end,C,yaw=yaw),Waypoint(1.,end,O,yaw=yaw),Waypoint(1.5,(end[0],end[1],TOP+.14),O,yaw=yaw)]
 return result
