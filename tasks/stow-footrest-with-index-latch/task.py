"""Stow the footrest module so the marked transfer space is clear. Pull the grey latch back and hold it while swinging the blue footrest through 90 degrees. Return the latch into the second index slot. Release both handles. The module is unoccupied and unpowered."""
import math
import numpy as np
from yam_arena.task_file import TABLE_TOP,Box
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,PAD_TIP_PAST_TCP as D,FINGER_JOINTS
from yam_articulated import Articulated
SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=2.;SCORE_OVER='end'
def p(pos,size,color=None):
 value=dict(pos=pos,size=size)
 if color is not None:value['color']=color
 return value
blue=(.12,.35,.85)
wheel=[]
for x in np.arange(-.035,.036,.010):
 for y in np.arange(-.035,.036,.010):
  if (y>.020 and abs(x)<.006) or (x>.020 and abs(y)<.006):continue
  wheel.append(p((float(x),float(y),0),(.010,.010,.012),blue))
wheel.extend([p((0,-.075,-.015),(.080,.150,.010),blue),p((0,-.020,-.006),(.030,.040,.024),blue),p((0,-.110,.005),(.026,.030,.050),blue),p((0,-.110,.020),(.026,.030,.035),blue)])
HINGE=Articulated('footrest_module',pos=(.16,0,TOP),size=(.10,.22,.10),color=(.65,.65,.65),fixed_base=True,
 parts=(p((0,0,.004),(.10,.10,.008)),p((0,0,.017),(.040,.040,.034))),
 links=(dict(name='footrest',mass=.16,pos=(0,0,.045),parts=tuple(wheel)),),
 joints=(dict(name='footrest_angle',body0='root',body1='footrest',type='revolute',axis='Z',anchor0=(0,0,.045),anchor1=(0,0,0),lower=0.,upper=90.,stiffness=0.,damping=.004,max_force=0.),))
LATCH=Articulated('spring_latch',pos=(.16,.115,TOP+.045),size=(.08,.20,.090),color=(.65,.65,.65),fixed_base=True,
 parts=(p((0,0,-.041),(.08,.12,.008)),p((-.014,0,-.0175),(.010,.040,.055)),p((.014,0,-.0175),(.010,.040,.055))),
 links=(dict(name='pin',mass=.040,pos=(0,0,0),parts=(p((0,-.030,0),(.008,.120,.010)),p((0,.005,.015),(.012,.018,.040)),p((0,.045,.030),(.012,.100,.012)),p((0,.080,.030),(.030,.028,.028)))),),
 joints=(dict(name='latch_travel',body0='root',body1='pin',type='prismatic',axis='Y',lower=0.,upper=.050,target=0.,stiffness=40.,damping=.8,max_force=2.),))
ZONE=Box('transfer_space',pos=(.16,-.115,TOP+.0005),size=(.16,.11,.001),color=(.95,.85,.15))
OBJECTS=[HINGE,LATCH,ZONE]
_angle=0.;_pin=0.;_speed=0.;_opened=False;_open=False;_bad=False
_reached_index=False
def reset_history():
 global _angle,_pin,_speed,_opened,_open,_bad
 global _reached_index
 _angle=0.;_pin=0.;_speed=0.;_opened=False;_open=False;_bad=False
 _reached_index=False
def observe(states,t,base):
 global _angle,_pin,_speed,_opened,_open,_bad
 global _reached_index
 a=base.scene['footrest_module'];index=a.find_joints('footrest_angle')[0][0]
 _angle=float(a.data.joint_pos.torch[0,index].item());_speed=abs(float(a.data.joint_vel.torch[0,index].item()))
 a=base.scene['spring_latch'];index=a.find_joints('latch_travel')[0][0];_pin=float(a.data.joint_pos.torch[0,index].item())
 if _pin>=.025:_opened=True
 if not _opened and _angle>math.radians(8):_bad=True
 if _angle>=math.radians(87):_reached_index=True
 if _angle<math.radians(80):_reached_index=False
 if not _reached_index and _angle>math.radians(8) and _pin<.025:_bad=True
 _open=all(float(np.max(np.abs(base.scene[n].data.joint_pos.torch[0,base.scene[n].find_joints(FINGER_JOINTS,preserve_order=True)[0]].detach().cpu().numpy())))>.035 for n in ['left_robot','right_robot'])
def score(states):
 if _bad:return 0.
 if math.radians(87)<=_angle<=math.radians(92) and _opened:
  if _pin<=.003 and _speed<.03:return 1. if _open else .75
  return .5
 if _opened:return .25
 return 0.
def success(states):return score(states)==1.
def distance(states):return abs(_angle-math.pi/2)*.11
def demo():
 initial=(.16,.195,TOP+.075+D);back=(.16,.240,initial[2])
 left=[Waypoint(1.5,(initial[0],initial[1],TOP+.14),O),Waypoint(1.5,initial,O),Waypoint(.8,initial,C),Waypoint(1.5,back,C,'withdraw latch'),Waypoint(8.,back,C,'hold latch clear'),Waypoint(1.5,initial,C,'return pin into stowed slot'),Waypoint(.8,initial,O),Waypoint(1.5,(initial[0],initial[1],TOP+.14),O)]
 initial=(.16,-.110,TOP+.065+D)
 right=[Waypoint(1.5,(initial[0],initial[1],TOP+.14),O),Waypoint(3.8,(initial[0],initial[1],TOP+.14),O,'wait for latch'),Waypoint(1.5,initial,O),Waypoint(.8,initial,C)]
 for angle in [20,40,60,80,92]:
  theta=math.radians(angle);target=(.16+.110*math.sin(theta),-.110*math.cos(theta),initial[2])
  right.append(Waypoint(1.,target,C,'swing footrest',theta))
 right.extend([Waypoint(3.5,target,C,'wait for latch engagement',theta),Waypoint(.8,target,O),Waypoint(1.5,(target[0],target[1],TOP+.14),O,yaw=theta)])
 return dict(left=left,right=right)
