"""Hold the lower drive hub still by its long lever. Seat the orange coupling over both unequal drive dogs. Release the lower lever and turn the orange coupling through 90 degrees so the lower hub turns with it. Release both grippers. Leave the coupling seated. All equipment is unpowered."""
import math
import numpy as np
import drive_contact
from yam_arena.task_file import TABLE_TOP
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,PAD_TIP_PAST_TCP as D,FINGER_JOINTS
from yam_articulated import Articulated,Compound
SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=2.;SCORE_OVER='end'
def p(pos,size):return dict(pos=pos,size=size)
disc=dict(pos=(0,0,0),size=(.080,.080,.012),shape='cylinder',radius=.040,height=.012,axis='Z')
HUB=Articulated('hub_fixture',pos=(.28,-.03,TOP),size=(.10,.30,.047),color=(.65,.65,.65),fixed_base=True,
 parts=(p((0,.070,.004),(.10,.30,.008)),),
 links=(dict(name='rotor',mass=.15,pos=(0,0,.016),parts=(disc,p((-.022,0,.017),(.014,.024,.020)),p((.022,0,.017),(.014,.016,.020)),p((0,.110,0),(.026,.220,.012)),p((0,.220,.012),(.030,.030,.024)))),),
 joints=(dict(name='rotor_angle',body0='root',body1='rotor',type='revolute',axis='Z',anchor0=(0,0,.016),anchor1=(0,0,0),lower=-180.,upper=180.,stiffness=0.,damping=.005,max_force=0.),))
parts=[p((0,0,.024),(.080,.070,.008)),p((-.039,0,.010),(.012,.070,.020)),p((.039,0,.010),(.012,.070,.020)),p((0,0,.010),(.022,.070,.020)),p((0,0,.055),(.026,.026,.054))]
for y in [-1,1]:
 parts.extend([p((-.022,y*.020,.010),(.022,.012,.020)),p((.022,y*.016,.010),(.022,.012,.020))])
COUPLING=Compound('coupling',pos=(.17,-.20,TOP),size=(.090,.070,.082),mass=.085,color=(.95,.4,.08),parts=tuple(parts))
OBJECTS=[HUB,COUPLING]
_q=0.;_engaged=False;_open=False;_bad=False
GOAL=np.array([.28,-.03,TOP+.064])
def reset_history():
 global _q,_engaged,_open,_bad
 _q=0.;_engaged=False;_open=False;_bad=False
 drive_contact.reset()
def orientation(s):
 x,y,z,w=s.quat_xyzw
 return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)),1-2*(x*x+y*y)
def seated(s):
 if not all(np.isfinite(v).all() for v in [s.pos,s.quat_xyzw,s.lin_vel,s.ang_vel]):return False
 yaw,vertical=orientation(s);relative=abs((yaw-_q+math.pi)%(2*math.pi)-math.pi)
 return np.max(np.abs(s.pos-GOAL))<.004 and vertical>math.cos(math.radians(5)) and relative<math.radians(5)
def observe(states,t,base):
 global _q,_engaged,_open,_bad
 drive_contact.bind(base)
 if drive_contact.failed():_bad=True
 art=base.scene['hub_fixture'];index=art.find_joints('rotor_angle')[0][0];_q=float(art.data.joint_pos.torch[0,index].item())
 s=states['coupling']
 if abs(_q)<math.radians(5) and seated(s):_engaged=True
 if _engaged and not seated(s):_engaged=False
 if abs(_q)>=math.radians(5) and not _engaged:_bad=True
 if s.low[2]<TOP-.004:_bad=True
 _open=all(float(np.max(np.abs(base.scene[n].data.joint_pos.torch[0,base.scene[n].find_joints(FINGER_JOINTS,preserve_order=True)[0]].detach().cpu().numpy())))>.035 for n in ['left_robot','right_robot'])
def score(states):
 if _bad:return 0.
 s=states['coupling']
 if seated(s):
  if _engaged and math.radians(80)<_q<math.radians(100) and s.resting(.008,.1):return 1. if _open else .75
  return .5
 if _engaged:return .25
 return 0.
def success(states):return score(states)==1.
def distance(states):return float(np.linalg.norm(states['coupling'].pos-GOAL))
def demo():
 hold=(.28,.190,TOP+.028+D)
 left=[Waypoint(1.5,(hold[0],hold[1],TOP+.14),O),Waypoint(1.5,hold,O),Waypoint(.8,hold,C),Waypoint(10.3,hold,C,'hold lower rotor'),Waypoint(.8,hold,O,'release rotor for drive test'),Waypoint(1.2,(hold[0],hold[1],TOP+.14),O)]
 pick=(.17,-.20,TOP+.065+D);place=(.28,-.0265,TOP+.088+D)
 right=[Waypoint(1.5,(pick[0],pick[1],TOP+.14),O),Waypoint(2.3,(pick[0],pick[1],TOP+.14),O),Waypoint(1.5,pick,O),Waypoint(.8,pick,C),Waypoint(1.2,(pick[0],pick[1],TOP+.14),C),Waypoint(1.8,(place[0],place[1],TOP+.14),C),Waypoint(1.2,place,C),Waypoint(.8,place,O,'settle coupling into dogs'),Waypoint(1.,place,O,'allow seating'),Waypoint(.8,place,C,'grasp seated coupling'),Waypoint(3.5,place,C,'wait for release of lower lever')]
 for angle in [33,66,100]:right.append(Waypoint(1.3,place,C,'turn coupled drive',math.radians(angle)))
 right.extend([Waypoint(.8,place,O,'release coupling',math.radians(100)),Waypoint(1.2,(place[0],place[1],TOP+.14),O,'retreat',math.radians(100))])
 return dict(left=left,right=right)
