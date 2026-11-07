"""Hold the blue optical bar level with the left arm before pulling its grey support carriage out with the right arm. Keep the bar supported while the carriage clears it. Lower the bar into the white storage cradle. Release both grippers. All equipment is unpowered and no person is present."""
import math
import numpy as np
from yam_arena.task_file import TABLE_TOP,Box
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,PAD_TIP_PAST_TCP as D,FINGER_JOINTS
from yam_articulated import Articulated,Compound
SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=2.;SCORE_OVER='end'
def p(pos,size):return dict(pos=pos,size=size)
SUPPORT=Articulated('support_carriage',pos=(.26,-.07,TOP),size=(.24,.24,.09),color=(.65,.65,.65),fixed_base=True,parts=(p((0,0,.004),(.24,.22,.008)),),links=(dict(name='saddle',mass=.20,pos=(0,0,0),parts=(p((0,0,.015),(.20,.15,.014)),p((-.075,.07,.0475),(.020,.025,.065)),p((.075,.07,.0475),(.020,.025,.065)),p((0,-.09,.036),(.030,.030,.040)))),),joints=(dict(name='support_travel',body0='root',body1='saddle',type='prismatic',axis='Y',lower=-.13,upper=0.,stiffness=0.,damping=2.,max_force=0.),))
BAR=Compound('optical_bar',pos=(.26,0,TOP+.090),size=(.19,.03,.065),mass=.10,color=(.12,.35,.85),parts=(p((0,0,0),(.19,.03,.020)),p((0,0,.025),(.026,.026,.050))))
CRADLE=Compound('storage_cradle',pos=(.16,.19,TOP),size=(.23,.08,.03),color=(.95,.95,.95),parts=(p((0,0,.005),(.23,.08,.010)),p((0,-.027,.020),(.23,.014,.020)),p((0,.027,.020),(.23,.014,.020))))
OBJECTS=[SUPPORT,BAR,CRADLE]
_q=0.;_cleared=False;_bad=False;_open=False;_last=None
_relative_travel=0.
SUPPORT_LINE_Y=SUPPORT.pos[1]+.070
GOAL=np.array([.16,.19,TOP+.040])
def reset_history():
 global _q,_cleared,_bad,_open,_last
 global _relative_travel
 _q=0.;_cleared=False;_bad=False;_open=False;_last=None
 _relative_travel=0.
def observe(states,t,base):
 global _q,_cleared,_bad,_open,_last
 global _relative_travel
 a=base.scene['support_carriage'];index=a.find_joints('support_travel')[0][0];_q=float(a.data.joint_pos.torch[0,index].item())
 s=states['optical_bar'];x,y,z,w=s.quat_xyzw
 if s.low[2]<TOP+.007 or 1-2*(x*x+y*y)<math.cos(math.radians(10)):_bad=True
 if not _cleared and s.low[2]<TOP+.067:_bad=True
 _relative_travel=float(s.pos[1]-(SUPPORT_LINE_Y+_q))
 if _q<-.105 and s.low[1]>SUPPORT_LINE_Y+_q+.0125+.005:_cleared=True
 if _last is not None and s.low[2]<_last-.012:_bad=True
 _last=s.low[2]
 _open=all(float(np.max(np.abs(base.scene[n].data.joint_pos.torch[0,base.scene[n].find_joints(FINGER_JOINTS,preserve_order=True)[0]].detach().cpu().numpy())))>.035 for n in ['left_robot','right_robot'])
def score(states):
 if _bad:return 0.
 s=states['optical_bar'];x,y,z,w=s.quat_xyzw;yaw=abs(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
 if _cleared:
  if np.max(np.abs(s.pos-GOAL))<.004 and yaw<math.radians(5) and s.resting(.008,.1):return 1. if _open else .75
  return .5
 if _q<-.040 and _relative_travel>.040:return .25
 return 0.
def success(states):return score(states)==1.
def distance(states):return float(np.linalg.norm(states['optical_bar'].pos-GOAL))
def demo():
 grip=(.26,0,TOP+.115+D);raised=(.26,0,TOP+.135+D);end=(.1655,.1885,TOP+.045+D)
 left=[Waypoint(1.5,(grip[0],grip[1],TOP+.18),O),Waypoint(1.5,grip,O),Waypoint(.8,grip,C),Waypoint(1.2,raised,C),Waypoint(5.,raised,C,'support bar during withdrawal'),Waypoint(2.,(.1655,.1885,TOP+.135+D),C),Waypoint(2.,end,C),Waypoint(.8,end,O),Waypoint(1.5,(end[0],end[1],TOP+.18),O)]
 grip=(.26,-.16,TOP+.036+D);back=(.26,-.28,grip[2])
 right=[Waypoint(1.5,(grip[0],grip[1],TOP+.18),O),Waypoint(3.5,(grip[0],grip[1],TOP+.18),O),Waypoint(1.5,grip,O),Waypoint(.8,grip,C),Waypoint(2.,back,C),Waypoint(.8,back,O),Waypoint(1.5,(back[0],back[1],TOP+.18),O)]
 return dict(left=left,right=right)
