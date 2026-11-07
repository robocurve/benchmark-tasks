"""Place one blue chock on each side of the grey rolling cradle before removing its orange load. Put the load on the white storage pad. Leave both chocks against the cradle. Release both grippers. Keep the cradle within 10 mm of its starting position. The cradle has a fixed internal ballast. All equipment is unpowered."""
import math
import numpy as np
from yam_arena.task_file import TABLE_TOP,Box
from yam_arena.demo import Waypoint,pick_place_waypoints
from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,PAD_TIP_PAST_TCP as D,FINGER_JOINTS
from yam_articulated import Compound
SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=2.;SCORE_OVER='end'
def p(pos,size):return dict(pos=pos,size=size)
CRADLE=Compound('rolling_cradle',pos=(.28,0,TOP+.045),size=(.14,.10,.105),mass=2.4,center_of_mass=(0.,0.,-.024),color=(.65,.65,.65),parts=(dict(p((0,0,0),(.14,.09,.09)),shape='cylinder',axis='X',radius=.045,height=.14),p((0,0,.055),(.12,.10,.010))))
CHOCKS=[]
for n,sign,x in [('left',1,.16),('right',-1,.21)]:
 CHOCKS.append(Compound(n+'_chock',pos=(x,sign*.13,TOP),size=(.12,.139,.072),mass=.10,color=(.12,.35,.85),parts=(p((0,0,.015),(.12,.030,.030)),p((0,sign*.060,.008),(.024,.120,.016)),p((0,sign*.110,.040),(.024,.028,.064)))))
LOAD=Compound('load',pos=(.28,0,TOP+.115),size=(.045,.045,.100),mass=.10,color=(.95,.4,.08),parts=(p((0,0,0),(.045,.045,.020)),p((0,0,.045),(.024,.024,.090))))
PAD=Box('storage_pad',pos=(.20,-.30,TOP+.001),size=(.070,.070,.002),color=(.95,.95,.95))
OBJECTS=[CRADLE,*CHOCKS,LOAD,PAD]
_reference=None;_bad=False;_open=False;_restrained=False;_count=0;_load_reference=None
def reset_history():
 global _reference,_bad,_open,_restrained,_count,_load_reference
 _reference=None;_bad=False;_open=False;_restrained=False;_count=0;_load_reference=None
def chock_ok(s,sign):
 x,y,z,w=s.quat_xyzw
 return abs(s.pos[0]-.28)<.004 and abs(s.pos[1]-sign*.1145)<.004 and abs(s.low[2]-TOP)<.003 and 1-2*(x*x+y*y)>math.cos(math.radians(5)) and abs(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))<math.radians(5)
def observe(states,t,base):
 global _reference,_bad,_open,_restrained,_count,_load_reference
 c=states['rolling_cradle'];l=states['load']
 if _reference is None:_reference=np.asarray(c.pos).copy()
 if _load_reference is None:_load_reference=np.asarray(l.pos-c.pos).copy()
 if np.linalg.norm(c.pos-_reference)>.010 or l.low[2]<TOP-.004:_bad=True
 _count=sum(chock_ok(states[n+'_chock'],sign) for n,sign in [('left',1),('right',-1)])
 _restrained=_count==2
 if not _restrained and np.max(np.abs((l.pos-c.pos)-_load_reference))>.005:_bad=True
 _open=all(float(np.max(np.abs(base.scene[n].data.joint_pos.torch[0,base.scene[n].find_joints(FINGER_JOINTS,preserve_order=True)[0]].detach().cpu().numpy())))>.035 for n in ['left_robot','right_robot'])
def score(states):
 if _bad:return 0.
 if _count==2 and _restrained:
  l=states['load'];x,y,z,w=states['rolling_cradle'].quat_xyzw
  if np.max(np.abs(l.pos-np.array([PAD.pos[0],PAD.pos[1],TOP+.052])))<.005 and l.resting(.008,.1) and states['rolling_cradle'].resting(.008,.1) and 1-2*(x*x+y*y)>math.cos(math.radians(5)):return 1. if _open else .75
  return .5
 if _count==1:return .25
 return 0.
def success(states):return score(states)==1.
def distance(states):return float(np.linalg.norm(states['load'].pos-np.array([PAD.pos[0],PAD.pos[1],TOP+.052])))
def demo():
 plans={}
 for arm,y,sign in [('left',.13,1),('right',-.13,-1)]:
  pick=(.16 if arm=='left' else .21,y+sign*.110,TOP+.050+D);place=(.2828 if arm=='left' else .2824,.1690 if arm=='left' else -.1646,TOP+.050+D)
  if arm=='right':
   outer=(place[0],place[1]-.022,place[2])
   plans[arm]=[Waypoint(1.5,(pick[0],pick[1],TOP+.22),O),Waypoint(1.5,pick,O),Waypoint(.8,pick,C),Waypoint(1.5,(pick[0],pick[1],TOP+.22),C),Waypoint(1.5,(outer[0],outer[1],TOP+.22),C),Waypoint(1.5,outer,C),Waypoint(1.5,place,C),Waypoint(1.0,place,O),Waypoint(1.5,(place[0],place[1],TOP+.22),O)]
  else:
   plans[arm]=pick_place_waypoints(pick,place,travel_z=TOP+.22,yaw=0.)
 grip=(.28,0,TOP+.185+D);end=(.2480,-.2862,TOP+.082+D)
 plans['right'].extend([Waypoint(1.5,(grip[0],grip[1],TOP+.24),O),Waypoint(1.5,grip,O),Waypoint(.8,grip,C),Waypoint(1.5,(grip[0],grip[1],TOP+.24),C),Waypoint(2.,(end[0],end[1],TOP+.24),C),Waypoint(1.5,end,C),Waypoint(.8,end,O),Waypoint(1.5,(end[0],end[1],TOP+.24),O)])
 return plans
