"""Hold the grey shaft fixture still with one arm. With the other arm slide the blue retaining fork sideways into the shaft groove below its square head. Both tines must reach beyond the shaft centre. Release both grippers. Keep the fixture within 5 mm of its starting position. All equipment is unpowered."""
import math
import numpy as np
from yam_arena.task_file import TABLE_TOP,Box
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,PAD_TIP_PAST_TCP as D,FINGER_JOINTS
from yam_articulated import Compound
SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=2.;SCORE_OVER='end'
def p(pos,size):return dict(pos=pos,size=size,contact_offset=.00025)
FIXTURE=Compound('fixture',pos=(.28,0,TOP),size=(.12,.29,.066),mass=.65,color=(.65,.65,.65),parts=(
 p((0,.075,.005),(.12,.29,.010)),p((0,0,.025),(.032,.032,.030)),
 p((0,0,.044),(.010,.010,.008)),p((0,0,.057),(.050,.050,.018)),
 p((0,.200,.025),(.024,.024,.050)),p((0,.200,.050),(.032,.035,.026))))
FORK=Compound('fork',pos=(.19,-.14,TOP+.043),size=(.054,.165,.053),mass=.035,color=(.12,.35,.85),parts=(
 p((-.020,.005,0),(.014,.060,.006)),p((.020,.005,0),(.014,.060,.006)),
 p((0,-.030,0),(.054,.010,.006)),p((0,-.075,.005),(.026,.090,.010)),p((0,-.115,.025),(.026,.030,.050))))
STAGING=Compound('staging_stand',pos=(.19,-.19,TOP),size=(.10,.22,.043),color=(.95,.95,.95),parts=(p((0,0,.020),(.10,.22,.040)),p((0,-.065,.0415),(.05,.04,.003))))
STOPS=[Box('side_stop_'+str(i),pos=(.28+sign*.070,.075,TOP+.006),size=(.012,.29,.012),color=(.45,.45,.45)) for i,sign in enumerate([-1,1])]+[Box('end_stop_'+str(i),pos=(.28,y,TOP+.006),size=(.124,.012,.012),color=(.45,.45,.45)) for i,y in enumerate([-.079,.229])]
OBJECTS=[FIXTURE,FORK,STAGING,*STOPS]
_bad=False;_open=False;_reference=None;_fork_start_low=None
def reset_history():
 global _bad,_open,_reference,_fork_start_low
 _bad=False;_open=False;_reference=None;_fork_start_low=None
def observe(states,t,base):
 global _bad,_open,_reference,_fork_start_low
 f=states['fixture'];s=states['fork']
 if _reference is None:_reference=f.pos.copy()
 if _fork_start_low is None:_fork_start_low=float(s.low[2])
 if np.linalg.norm(f.pos-_reference)>.005 or s.low[2]<TOP-.005:_bad=True
 _open=all(float(np.max(np.abs(base.scene[n].data.joint_pos.torch[0,base.scene[n].find_joints(FINGER_JOINTS,preserve_order=True)[0]].detach().cpu().numpy())))>.035 for n in ['left_robot','right_robot'])
def score(states):
 if _bad:return 0.
 s=states['fork'];q=np.asarray(s.quat_xyzw)
 if not all(np.isfinite(v).all() for v in [s.pos,q,s.lin_vel,s.ang_vel]):return 0.
 x,y,z,w=q;upright=1-2*(x*x+y*y)>math.cos(math.radians(5));yaw=abs(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
 if not upright or yaw>math.radians(5):return 0.
 target=np.array([.28,-.0475,TOP+.0665])
 if np.max(np.abs(s.pos-target))<.004 and s.resting(.008,.1):return 1. if _open else .75
 if abs(s.pos[0]-.28)<.006 and abs(s.pos[2]-target[2])<.005 and -.0825<s.pos[1]<-.0475:return .5
 if _fork_start_low is not None and s.low[2]>_fork_start_low+.015:return .25
 return 0.
def success(states):return score(states)==1.
def distance(states):return float(np.linalg.norm(states['fork'].pos-np.array([.28,-.0475,TOP+.0665])))
def demo():
 grip=(.28,.200,TOP+.050+D)
 left=[Waypoint(1.5,(.28,.200,TOP+.14),O),Waypoint(1.5,grip,O),Waypoint(.8,grip,C),Waypoint(14.,grip,C),Waypoint(.8,grip,O),Waypoint(1.5,(.28,.200,TOP+.14),O)]
 pick=(.19,-.255,TOP+.068+D);entry=(.2802,-.180,TOP+.0695+D);end=(.2802,-.115,TOP+.0695+D)
 right=[Waypoint(1.5,(pick[0],pick[1],TOP+.14),O, yaw=0.),Waypoint(2.3,(pick[0],pick[1],TOP+.14),O,yaw=0.),Waypoint(1.5,pick,O,yaw=0.),Waypoint(.8,pick,C,yaw=0.),Waypoint(1.2,(pick[0],pick[1],TOP+.14),C,yaw=0.),Waypoint(1.8,(entry[0],entry[1],TOP+.14),C,yaw=math.radians(-3)),Waypoint(1.5,entry,C,yaw=math.radians(-3)),Waypoint(3.,end,C,yaw=math.radians(-3)),Waypoint(.8,end,O,yaw=math.radians(-3)),Waypoint(1.5,(end[0],end[1],TOP+.14),O,yaw=math.radians(-3))]
 return dict(left=left,right=right)
