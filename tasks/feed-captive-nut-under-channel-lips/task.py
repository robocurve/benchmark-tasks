"""Hold the grey channel still. Feed the blue T nut through its open front end and slide it to the white end mark. The wide head must be below both retaining lips. Release both grippers. Keep the channel within 5 mm of its starting position. All equipment is unpowered."""
import math
import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix
from yam_arena.task_file import TABLE_TOP,Box
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,PAD_TIP_PAST_TCP as D,FINGER_JOINTS
from yam_articulated import Compound
SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=2.;SCORE_OVER='end'
def p(pos,size):return dict(pos=pos,size=size)
parts=[p((0,0,.005),(.092,.18,.010)),p((0,.165,.008),(.045,.150,.016)),p((0,.225,.020),(.028,.025,.024))]
for sign in [-1,1]:parts.extend([p((sign*.036,0,.025),(.012,.18,.040)),p((sign*.026,0,.043),(.028,.18,.008))])
CHANNEL=Compound('channel',pos=(.26,.03,TOP),size=(.092,.330,.047),mass=.60,color=(.65,.65,.65),parts=tuple(parts))
NUT=Compound('nut',pos=(.17,-.19,TOP+.015),size=(.044,.030,.063),mass=.035,color=(.12,.35,.85),parts=(p((0,0,-.009),(.044,.030,.012)),p((0,0,.012),(.014,.026,.040)),p((0,0,.040),(.032,.028,.016))))
MARK=Box('mark',pos=(.26,.060,TOP+.0005),size=(.09,.003,.001),color=(1.,1.,1.))
OBJECTS=[CHANNEL,NUT,MARK]
_bad=False;_open=False;_reference=None;_entered=False
def reset_history():
 global _bad,_open,_reference,_entered
 _bad=False;_open=False;_reference=None;_entered=False
def observe(states,t,base):
 global _bad,_open,_reference,_entered
 c=states['channel'];s=states['nut']
 if _reference is None:_reference=c.pos.copy()
 if np.linalg.norm(c.pos-_reference)>.005 or s.low[2]<TOP-.005:_bad=True
 if 2*math.acos(min(1.,abs(float(c.quat_xyzw[3]))))>math.radians(5):_bad=True
 x,y,z,w=s.quat_xyzw
 upright=1-2*(x*x+y*y)>math.cos(math.radians(5))
 yaw=abs(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
 if not (abs(s.pos[0]-.26)<.010 and -.100<s.pos[1]<.100 and abs(s.low[2]-TOP-.010)<.008 and upright and yaw<math.radians(5)):_entered=False
 if abs(s.pos[0]-.26)<.005 and -.085<s.pos[1]<-.055 and abs(s.low[2]-TOP-.010)<.006 and upright and yaw<math.radians(5):_entered=True
 _open=all(float(np.max(np.abs(base.scene[n].data.joint_pos.torch[0,base.scene[n].find_joints(FINGER_JOINTS,preserve_order=True)[0]].detach().cpu().numpy())))>.035 for n in ['left_robot','right_robot'])
def head_top(s):
 rotation=quat_xyzw_to_matrix(s.quat_xyzw)
 signs=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
 points=[(signs*np.asarray(part['size'])/2+part['pos'])@rotation.T for part in NUT.parts]
 all_points=np.concatenate(points)
 root=s.pos-(all_points.min(axis=0)+all_points.max(axis=0))/2
 return float((points[0]+root)[:,2].max())

def score(states):
 if _bad:return 0.
 s=states['nut'];q=np.asarray(s.quat_xyzw)
 if not all(np.isfinite(v).all() for v in [s.pos,q,s.lin_vel,s.ang_vel]):return 0.
 x,y,z,w=q;upright=1-2*(x*x+y*y)>math.cos(math.radians(5));yaw=abs(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
 if not upright or yaw>math.radians(5):return 0.
 seated=abs(s.pos[0]-.26)<.004 and abs(s.pos[1]-.060)<.005 and abs(s.low[2]-TOP-.010)<.004
 if seated and _entered and s.resting(.008,.1):return 1. if _open else .75
 if _entered and abs(s.pos[0]-.26)<.006 and -.02<s.pos[1]<.066 and head_top(s)<TOP+.039:return .5
 if _entered:return .25
 return 0.
def success(states):return score(states)==1.
def distance(states):return float(np.linalg.norm(states['nut'].pos-np.array([.26,.060,TOP+.0415])))
def demo():
 grip=(.26,.255,TOP+.020+D)
 left=[Waypoint(1.5,(grip[0],grip[1],TOP+.14),O),Waypoint(1.5,grip,O),Waypoint(.8,grip,C),Waypoint(19.,grip,C),Waypoint(.8,grip,O),Waypoint(1.5,(grip[0],grip[1],TOP+.14),O)]
 pick=(.17,-.19,TOP+.049+D);entry=(.26,-.090,TOP+.065+D);end=(.26,.060,TOP+.065+D)
 right=[Waypoint(1.5,(pick[0],pick[1],TOP+.14),O),Waypoint(2.3,(pick[0],pick[1],TOP+.14),O),Waypoint(1.5,pick,O),Waypoint(.8,pick,C),Waypoint(1.2,(pick[0],pick[1],TOP+.14),C),Waypoint(1.8,(entry[0],entry[1],TOP+.14),C),Waypoint(1.5,entry,C),Waypoint(6.,end,C),Waypoint(.8,end,O),Waypoint(1.5,(end[0],end[1],TOP+.14),O)]
 for waypoint in right[5:]:waypoint.yaw=math.radians(-3)
 return dict(left=left,right=right)
