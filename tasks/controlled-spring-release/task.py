"""Unload the spring fixture without allowing its platform to snap upward.

Support the blue platform before withdrawing the orange retaining bar. Keep platform speed
below 80 mm per second. Let it rise to its unloaded position under control. Put the bar on
the white return pad. Finish with both grippers open. This is an unpowered test fixture.
"""
import math
import numpy as np
from yam_arena.task_file import Box,TABLE_TOP
from yam_arena.demo import Waypoint
from yam_arena.constants import FINGER_OPEN,FINGER_CLOSED,PAD_TIP_PAST_TCP
from yam_articulated import Articulated,Compound
SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=1.;SCORE_OVER='end'
def p(pos,size):return dict(pos=pos,size=size)
fixed=[p((0,0,.004),(.12,.14,.008))]
for y in [-.045,.045]:
    fixed.extend([p((-.018,y,.030),(.014,.014,.060)),p((.018,y,.030),(.014,.014,.060)),p((0,y,.065),(.05,.014,.014))])
platform=(p((0,0,.04),(.08,.07,.012)),p((0,.075,.04),(.024,.08,.012)))
FIXTURE=Articulated('spring_fixture',pos=(.28,0,TOP),size=(.12,.185,.10),mass=None,
    color=(.1,.35,.85),parts=tuple(fixed),fixed_base=True,
    links=(dict(name='platform',mass=.04,pos=(0,0,0),parts=platform),),
    joints=(dict(name='platform_spring',body0='root',body1='platform',type='prismatic',axis='Z',
                 lower=0.,upper=.06,target=.06,stiffness=40.,damping=.15,max_force=3.),))
BAR=Compound('retaining_bar',pos=(.28,0,TOP+.051),size=(.03,.15,.02),mass=.03,color=(.95,.4,.08),
    parts=(p((0,0,0),(.014,.12,.010)),p((0,-.075,.005),(.03,.03,.02))))
PAD=Box('return_pad',pos=(.18,-.19,TOP+.0005),size=(.07,.18,.001),mass=None,color=(.95,.95,.95))
OBJECTS=[FIXTURE,BAR,PAD]
_q=0.;_speed=0.;_peak=0.;_violation=False;_open=False;_bar_clear=False
def reset_history():
    global _q,_speed,_peak,_violation,_open,_bar_clear
    _q=0.;_speed=0.;_peak=0.;_violation=False;_open=False;_bar_clear=False
def observe(states,t,base):
    global _q,_speed,_peak,_violation,_open,_bar_clear
    art=base.scene['spring_fixture']
    _q=float(art.data.joint_pos.torch[0,0].item())
    _speed=abs(float(art.data.joint_vel.torch[0,0].item()))
    _peak=max(_peak,_speed)
    if _speed>.08 or states['retaining_bar'].low[2]<TOP-.008:_violation=True
    _bar_clear=states['retaining_bar'].high[1]<-.055
    from yam_arena.constants import FINGER_JOINTS
    _open=all(float(np.max(np.abs(base.scene[name].data.joint_pos.torch[0,base.scene[name].find_joints(FINGER_JOINTS,preserve_order=True)[0]].detach().cpu().numpy())))>.035 for name in ['left_robot','right_robot'])
def _stored(s):
    return s.low[0]>.145 and s.high[0]<.215 and s.low[1]>-.28 and s.high[1]<-.10 and TOP-.001<s.low[2]<TOP+.004 and s.high[2]<TOP+.04 and s.resting(.01,.1)
def score(states):
    if _violation:return 0.
    if _bar_clear and .047<_q<.054 and _speed<.01 and _stored(states['retaining_bar']):return 1. if _open else .75
    if _bar_clear and _q>.035:return .5
    if _bar_clear:return .25
    return 0.
def success(states):return score(states)==1.
def distance(states):return abs(_q-.05019)
def demo():
    grip=(.28,.105,TOP+.046+PAD_TIP_PAST_TCP)
    left=[Waypoint(1.5,(.28,.105,TOP+.13),FINGER_OPEN,'approach platform tab'),
          Waypoint(1.5,grip,FINGER_OPEN,'lower to tab'),Waypoint(.8,grip,FINGER_CLOSED,'support loaded platform'),
          Waypoint(7.,grip,FINGER_CLOSED,'hold while bar is withdrawn'),
          Waypoint(4.,(grip[0],grip[1],grip[2]+.05),FINGER_CLOSED,'release spring energy slowly'),
          Waypoint(.8,(grip[0],grip[1],grip[2]+.05),FINGER_OPEN,'release unloaded platform'),
          Waypoint(1.2,(.28,.105,TOP+.14),FINGER_OPEN,'retreat')]
    yaw=math.pi/2;pick=(.28,-.075,TOP+.050+PAD_TIP_PAST_TCP)
    end=(.18,-.250,TOP+.011+PAD_TIP_PAST_TCP)
    right=[Waypoint(1.5,(.28,-.075,TOP+.14),FINGER_OPEN,'approach bar handle',yaw),
           Waypoint(3.3,(.28,-.075,TOP+.14),FINGER_OPEN,'wait for platform support',yaw),
           Waypoint(1.5,pick,FINGER_OPEN,'lower to handle',yaw),Waypoint(.8,pick,FINGER_CLOSED,'grasp bar',yaw),
           Waypoint(3.,(.28,-.205,pick[2]),FINGER_CLOSED,'withdraw retaining bar',yaw),
           Waypoint(1.2,(.28,-.205,TOP+.14),FINGER_CLOSED,'lift clear of guide',yaw),
           Waypoint(2.,(end[0],end[1],TOP+.14),FINGER_CLOSED,'carry to return pad',yaw),
           Waypoint(1.5,end,FINGER_CLOSED,'lower bar to pad',yaw),Waypoint(.8,end,FINGER_OPEN,'release bar',yaw),
           Waypoint(1.2,(end[0],end[1],TOP+.14),FINGER_OPEN,'retreat',yaw)]
    return dict(left=left,right=right)
