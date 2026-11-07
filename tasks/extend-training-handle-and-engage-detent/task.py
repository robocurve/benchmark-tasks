"""Retract the green locking pin with the left arm. Extend the blue training aid handle to the white 80 mm mark with the right arm. Return the pin into the second hole. Release both grippers with the handle locked at that length. No person is present and the module must not support a person."""
import math
import numpy as np
from yam_arena.task_file import TABLE_TOP,Box
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,PAD_TIP_PAST_TCP as D,FINGER_JOINTS
from yam_articulated import Articulated
SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=2.;SCORE_OVER='end'
def p(pos,size):return dict(pos=pos,size=size)
parts=[p((0,0,-.010),(.22,.028,.008)),p((0,0,.010),(.22,.028,.008)),p((.10,0,.030),(.026,.026,.060))]
for a,b in [(-.11,-.086),(-.074,-.006),(.006,.11)]:parts.append(p(((a+b)/2,0,0),(b-a,.028,.012)))
SLIDER=Articulated('adjustable_handle',pos=(.14,-.03,TOP+.060),size=(.34,.09,.11),color=(.12,.35,.85),fixed_base=True,parts=(p((.04,0,-.054),(.34,.09,.012)),p((.04,0,-.019),(.34,.05,.010)),p((-.01,-.023,0),(.12,.012,.05)),p((-.035,.023,0),(.050,.012,.05)),p((.035,.023,0),(.050,.012,.05))),links=(dict(name='handle',mass=.10,pos=(0,0,0),parts=tuple(parts)),),joints=(dict(name='handle_extension',body0='root',body1='handle',type='prismatic',axis='X',lower=0.,upper=.085,stiffness=0.,damping=2.,max_force=0.),))
PIN=Articulated('locking_pin',pos=(.14,.025,TOP+.060),size=(.08,.19,.10),color=(.12,.70,.3),fixed_base=True,parts=(p((0,.025,-.054),(.08,.14,.012)),p((-.014,.025,-.020),(.010,.025,.080)),p((.014,.025,-.020),(.010,.025,.080))),links=(dict(name='pin',mass=.035,pos=(0,0,0),parts=(p((0,-.025,0),(.008,.090,.008)),p((0,.010,.020),(.012,.020,.048)),p((0,.050,.035),(.012,.10,.012)),p((0,.09,.035),(.028,.028,.028)))),),joints=(dict(name='pin_retraction',body0='root',body1='pin',type='prismatic',axis='Y',lower=0.,upper=.060,target=0.,stiffness=35.,damping=.8,max_force=2.5),))
MARK=Box('length_mark',pos=(.32,-.075,TOP+.0005),size=(.003,.025,.001),color=(1.,1.,1.))
OBJECTS=[SLIDER,PIN,MARK]
_q=0.;_pin=0.;_speed=0.;_unlocked=False;_bad=False;_open=False
def reset_history():
 global _q,_pin,_speed,_unlocked,_bad,_open
 _q=0.;_pin=0.;_speed=0.;_unlocked=False;_bad=False;_open=False
def observe(states,t,base):
 global _q,_pin,_speed,_unlocked,_bad,_open
 a=base.scene['adjustable_handle'];i=a.find_joints('handle_extension')[0][0];_q=float(a.data.joint_pos.torch[0,i].item());_speed=abs(float(a.data.joint_vel.torch[0,i].item()))
 a=base.scene['locking_pin'];i=a.find_joints('pin_retraction')[0][0];_pin=float(a.data.joint_pos.torch[0,i].item())
 if _pin>.045:_unlocked=True
 if not _unlocked and _q>.005:_bad=True
 if .005<_q<.075 and _pin<=.045:_bad=True
 _open=all(float(np.max(np.abs(base.scene[n].data.joint_pos.torch[0,base.scene[n].find_joints(FINGER_JOINTS,preserve_order=True)[0]].detach().cpu().numpy())))>.035 for n in ['left_robot','right_robot'])
def score(states):
 if _bad:return 0.
 if _unlocked:
  if abs(_q-.080)<.002:
   if _pin<.002 and _speed<.004:return 1. if _open else .75
   return .5
  return .25
 return 0.
def success(states):return score(states)==1.
def distance(states):return abs(_q-.080)
def demo():
 grip=(.14,.115,TOP+.095+D);back=(.14,.170,grip[2])
 left=[Waypoint(1.5,(grip[0],grip[1],TOP+.18),O),Waypoint(1.5,grip,O),Waypoint(.8,grip,C),Waypoint(1.5,back,C),Waypoint(5.,back,C),Waypoint(2.,grip,C),Waypoint(.8,grip,O),Waypoint(1.5,(grip[0],grip[1],TOP+.18),O)]
 grip=(.24,-.03,TOP+.090+D);end=(.32,-.03,grip[2])
 right=[Waypoint(1.5,(grip[0],grip[1],TOP+.18),O),Waypoint(1.5,grip,O),Waypoint(.8,grip,C),Waypoint(2.,grip,C,'wait for pin'),Waypoint(3.,end,C),Waypoint(4.,end,C,'wait for locking pin'),Waypoint(.8,end,O),Waypoint(1.5,(end[0],end[1],TOP+.18),O)]
 return dict(left=left,right=right)
