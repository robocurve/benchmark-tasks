"""Inspect the travel of all three loose sliders and mark the one with the most free play.

Move each blue knob gently to both hard stops. Place the orange marker on the white pad
beside the slider with the greatest total travel. Release every knob and the marker.
The fixtures are unpowered. Do not dismantle them.
"""
import numpy as np
from yam_arena.task_file import Box,TABLE_TOP
from yam_arena.demo import Waypoint,pick_place_waypoints
from yam_arena.constants import FINGER_OPEN,FINGER_CLOSED,PAD_TIP_PAST_TCP
from yam_articulated import Articulated
SCENE='station';TOP=TABLE_TOP[SCENE];SUCCESS_HOLD_S=1.;SCORE_OVER='end'
def p(pos,size):return dict(pos=pos,size=size)
JIGS=[];PADS=[]
for i,(y,stroke) in enumerate([(.12,.020),(0.,.006),(-.12,.012)]):
    JIGS.append(Articulated(f'gauge_{i}',pos=(.23,y,TOP),size=(.065,.075,.04),mass=None,
        color=(.1,.35,.85),fixed_base=True,
        parts=(p((0,0,.004),(.065,.075,.008)),p((-.025,0,.018),(.008,.07,.028)),p((.025,0,.018),(.008,.07,.028))),
        links=(dict(name='knob',mass=.025,pos=(0,0,0),parts=(p((0,0,.025),(.028,.028,.018)),)),),
        joints=(dict(name='free_play',body0='root',body1='knob',type='prismatic',axis='Y',
                     lower=-stroke/2,upper=stroke/2,stiffness=0.,damping=.01,max_force=0.),)))
    PADS.append(Box(f'pad_{i}',pos=(.13,y,TOP+.0005),size=(.045,.045,.001),mass=None,color=(.95,.95,.95)))
MARKER=Box('marker',pos=(.17,.23,TOP+.010),size=(.020,.020,.020),mass=.015,color=(.95,.4,.08))
OBJECTS=[*JIGS,*PADS,MARKER]
_minimum=np.zeros(3);_maximum=np.zeros(3);_open=False;_violation=False
def reset_history():
    global _minimum,_maximum,_open,_violation
    _minimum=np.zeros(3);_maximum=np.zeros(3);_open=False;_violation=False
def observe(states,t,base):
    global _minimum,_maximum,_open,_violation
    q=np.array([float(base.scene[f'gauge_{i}'].data.joint_pos.torch[0,0].item()) for i in range(3)])
    _minimum=np.minimum(_minimum,q);_maximum=np.maximum(_maximum,q)
    if states['marker'].low[2]<TOP-.008:_violation=True
    from yam_arena.constants import FINGER_JOINTS
    _open=all(float(np.max(np.abs(base.scene[name].data.joint_pos.torch[0,base.scene[name].find_joints(FINGER_JOINTS,preserve_order=True)[0]].detach().cpu().numpy())))>.035 for name in ['left_robot','right_robot'])
def score(states):
    if _violation:return 0.
    half=np.array([.010,.003,.006]);tested=(_minimum<=-.9*half)&(_maximum>=.9*half)
    value=.2*int(tested.sum())
    s=states['marker'];pad=PADS[0]
    marked=s.low[0]>pad.pos[0]-.0225 and s.high[0]<pad.pos[0]+.0225 and s.low[1]>.0975 and s.high[1]<.1425 and TOP-.001<s.low[2]<TOP+.004 and s.resting(.01,.1)
    if tested.all() and marked:return 1. if _open else .8
    return value
def success(states):return score(states)==1.
def distance(states):return float(np.linalg.norm(states['marker'].pos-np.array([.13,.12,TOP+.011])))
def probe(y,wait=0.):
    z=TOP+.034+PAD_TIP_PAST_TCP
    points=[Waypoint(1.5,(.23,y,TOP+.14),FINGER_OPEN,'approach slider')]
    if wait:points.append(Waypoint(wait,(.23,y,TOP+.14),FINGER_OPEN,'wait clear'))
    points += [Waypoint(1.5,(.23,y,z),FINGER_OPEN,'lower around knob'),
               Waypoint(.8,(.23,y,z),FINGER_CLOSED,'grasp knob'),
               Waypoint(2.,(.23,y-.016,z),FINGER_CLOSED,'probe negative hard stop'),
               Waypoint(2.,(.23,y+.016,z),FINGER_CLOSED,'probe positive hard stop'),
               Waypoint(1.,(.23,y,z),FINGER_CLOSED,'return toward centre'),
               Waypoint(.6,(.23,y,z),FINGER_OPEN,'release knob'),
               Waypoint(1.2,(.23,y,TOP+.14),FINGER_OPEN,'retreat')]
    return points
def demo():
    left=probe(.12)+probe(0.)
    left+=pick_place_waypoints((.17,.23,TOP+.010+PAD_TIP_PAST_TCP),(.13,.12,TOP+.011+PAD_TIP_PAST_TCP),travel_z=TOP+.14,speed=.8)
    right=probe(-.12)
    right.append(Waypoint(1.2,(.17,-.25,TOP+.14),FINGER_OPEN,'clear the central inspection'))
    return dict(left=left,right=right)
