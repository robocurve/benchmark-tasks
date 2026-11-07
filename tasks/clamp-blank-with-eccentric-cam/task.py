"""Place the orange blank flat against both grey fixture stops. Hold it there while turning the blue cam lever to its closed stop past centre. Release both grippers. Leave the blank clamped after release. The fixture is bolted down and unpowered."""
import math
import numpy as np
import release_contact
from yam_arena.urdf import quat_xyzw_to_matrix
from yam_arena.task_file import TABLE_TOP,Box
from yam_arena.demo import Waypoint,pick_place_waypoints
from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,PAD_TIP_PAST_TCP as D,FINGER_JOINTS
from yam_articulated import Articulated,Compound
SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=2.;SCORE_OVER='end'
def p(pos,size):return dict(pos=pos,size=size)
disc=dict(pos=(.008,0,0),size=(.056,.056,.018),shape='cylinder',radius=.028,height=.018,axis='Z',color=(.12,.35,.85))
FIXTURE=Articulated('cam_fixture',pos=(.28,-.035,TOP),size=(.18,.30,.044),color=(.65,.65,.65),fixed_base=True,
 parts=(p((0,.065,.004),(.18,.30,.008)),p((.038,.065,.023),(.016,.072,.030))),
 links=(dict(name='cam',mass=.080,pos=(0,0,.014),parts=(disc,dict(p((-.060,0,.004),(.120,.026,.020)),color=(.12,.35,.85)))),dict(name='compliant_stop',mass=.050,pos=(0,.101,.023),parts=(p((0,0,0),(.10,.012,.030)),))),
 joints=(dict(name='cam_angle',body0='root',body1='cam',type='revolute',axis='Z',anchor0=(0,0,.014),anchor1=(0,0,0),lower=0.,upper=95.,stiffness=0.,damping=.002,max_force=0.),dict(name='stop_compliance',body0='root',body1='compliant_stop',type='prismatic',axis='Y',anchor0=(0,.101,.023),anchor1=(0,0,0),lower=0.,upper=.004,stiffness=1500.,damping=4.,max_force=15.)))
BLANK=Compound('blank',pos=(.16,.14,TOP+.022),size=(.060,.154,.065),mass=.155,color=(.95,.4,.08),parts=(p((0,0,0),(.060,.060,.028)),p((0,.022,.013),(.026,.016,.040)),p((0,.070,.027),(.026,.100,.012)),p((0,.110,.037),(.030,.028,.028))))
STAND=Box('starting_pad',pos=(.16,.14,TOP+.004),size=(.080,.080,.008),color=(.95,.95,.95))
OBJECTS=[FIXTURE,BLANK,STAND]
_q=0.;_loaded=False;_open=False;_bad=False
_stop=0.;_released=False
GOAL=np.array([.28,.077,TOP+.0405])
def reset_history():
 global _q,_loaded,_open,_bad
 global _stop,_released
 _q=0.;_loaded=False;_open=False;_bad=False
 _stop=0.;_released=False;release_contact.reset()
def aligned(s):
 q=np.asarray(s.quat_xyzw)
 if not all(np.isfinite(v).all() for v in [s.pos,q,s.lin_vel,s.ang_vel]):return False
 x,y,z,w=q;yaw=abs(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
 return np.max(np.abs(s.pos-GOAL))<.003 and 1-2*(x*x+y*y)>math.cos(math.radians(5)) and yaw<math.radians(5)
def observe(states,t,base):
 global _q,_loaded,_open,_bad
 global _stop,_released
 release_contact.bind(base);_released=release_contact.clear()
 art=base.scene['cam_fixture'];index=art.find_joints('cam_angle')[0][0]
 _q=float(art.data.joint_pos.torch[0,index].item())
 index=art.find_joints('stop_compliance')[0][0];_stop=float(art.data.joint_pos.torch[0,index].item())
 if _q<math.radians(5) and aligned(states['blank']):_loaded=True
 if states['blank'].low[2]<TOP-.004:_bad=True
 _open=all(float(np.max(np.abs(base.scene[n].data.joint_pos.torch[0,base.scene[n].find_joints(FINGER_JOINTS,preserve_order=True)[0]].detach().cpu().numpy())))>.035 for n in ['left_robot','right_robot'])
def cam_gap(s):
 rotation=quat_xyzw_to_matrix(s.quat_xyzw)
 signs=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])
 points=np.concatenate([signs*np.asarray(part['size'])/2+part['pos'] for part in BLANK.parts])@rotation.T
 root=s.pos-(points.min(axis=0)+points.max(axis=0))/2
 centre=np.asarray(FIXTURE.pos)+np.array([.008*math.cos(_q),.008*math.sin(_q),.014])
 local=(centre-root)@rotation
 delta=np.abs(local[:2])-np.array([.030,.030])
 return float(np.linalg.norm(np.maximum(delta,0))+min(float(np.max(delta)),0)-.028)

def score(states):
 if _bad:return 0.
 s=states['blank']
 if aligned(s):
  if _loaded and _q>=math.radians(92) and _stop>=.0001 and -.002<=cam_gap(s)<=.001 and s.resting(.008,.1):return 1. if _open and _released else .75
  if _loaded and _q>=math.radians(45):return .5
  return .25
 return 0.
def success(states):return score(states)==1.
def distance(states):return float(np.linalg.norm(states['blank'].pos-GOAL))
def demo():
 yaw=0.;pick=(.16,.250,TOP+.057+D);place=(.28,.140,TOP+.057+D)
 path=pick_place_waypoints(pick,place,travel_z=TOP+.14,yaw=yaw)
 left=path[:6]+[Waypoint(9.,place,C,'hold registered blank',yaw),Waypoint(.8,place,O,'release clamped blank',yaw),Waypoint(1.5,(place[0],place[1],TOP+.14),O,'retreat',yaw)]
 grip=(.18,-.035,TOP+.018+D)
 right=[Waypoint(1.5,(grip[0],grip[1],TOP+.14),O),Waypoint(1.5,grip,O),Waypoint(.8,grip,C),Waypoint(5.5,grip,C,'wait for loaded blank')]
 for angle in [19,38,57,76,95]:
  theta=math.radians(angle);target=(.28-.1*math.cos(theta),-.035-.1*math.sin(theta),grip[2])
  right.append(Waypoint(1.,target,C,'turn eccentric cam',theta))
 right.extend([Waypoint(.8,target,O,'release locked cam',theta),Waypoint(1.2,(target[0],target[1],TOP+.14),O,'retreat',theta)])
 return dict(left=left,right=right)
