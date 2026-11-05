"""Robot joint actions turn each wheel, then pull the physical hinged handle."""
import math
import numpy as np
from yam_arena.demo import Waypoint,ArmPlan,grip_rotation
from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C
from yam_arena.scripted import YamScriptedPolicy
from yam_arena.urdf import axis_angle_matrix
from rotary_box.scoring import PITCH

def make_plan(top,centers,initial,code):
    plans={'left':[],'right':[]}
    def turn(i):
        x,y,z=centers[i];start=(initial[i] if initial[i]<5 else initial[i]-10)*PITCH;goal=(code[i] if code[i]<5 else code[i]-10)*PITCH
        if i==0:start-=math.pi;goal-=math.pi
        if i==2:start+=math.pi;goal+=math.pi
        def w(t,height,f,label,yaw):return Waypoint(t,(x,y,top+height),f,label,yaw)
        return [w(2,.15,O,'approach',start),w(2,.070,O,'lower',start),w(1,.070,C,'grasp',start),w(3,.070,C,'rotate',goal),w(2,.070,C,'hold digit',goal),w(1,.070,O,'release',goal),w(2,.15,O,'retreat',goal)]
    plans['left']=turn(0)+[Waypoint(3,(.15,.32,top+.18),O,'park',-PITCH*2),Waypoint(28,(.15,.32,top+.18),O,'wait',-PITCH*2)]
    plans['right']=turn(2)+turn(1)
    plans['right'] += [Waypoint(3,(.36,-.12,top+.17),O,'handle approach'),Waypoint(2,(.36,-.12,top+.097),O,'handle lower'),Waypoint(1,(.36,-.12,top+.097),C,'handle grasp')]
    for deg in range(5,71,5):
        rot=axis_angle_matrix(np.array([0,1,0]),-math.radians(deg));pos=np.array([.25,-.12,top+.065])+rot@np.array([.11,0,.032])
        plans['right'].append(Waypoint(.6,tuple(pos),C,'lid '+str(deg)))
    plans['right'].append(Waypoint(3,plans['right'][-1].pos,C,'lid 70'))
    return plans

class LidPlan(ArmPlan):
    def target(self,t):
        pos,rot,finger,label=super().target(t)
        if label.startswith('lid '):
            # Infer the rotation from the point on the handle's circular path.
            from yam_arena.station import TABLE_TOP
            offset=pos-np.array([.25,-.12,TABLE_TOP+.065]);a=math.atan2(offset[2],offset[0])-math.atan2(.032,.11)
            rot=axis_angle_matrix(np.array([0,1,0]),-a)@grip_rotation()
        if label=='done' and hasattr(self,'last'):return self.last
        self.last=(pos,rot,finger,label);return self.last

class RotaryPolicy(YamScriptedPolicy):
    def reset(self,scene):
        plan=self.plan_for(scene.instruction,scene.target)
        self._plans={side:LidPlan(points,self._bases[side]) for side,points in plan.items()};self._end=max(p.duration for p in self._plans.values())

def make_policy(**kwargs):return RotaryPolicy(**kwargs)
