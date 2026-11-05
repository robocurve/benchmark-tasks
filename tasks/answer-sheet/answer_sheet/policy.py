"""Privileged marker feedback changes only joint actions, never object poses."""
import numpy as np
from yam_arena import ik
from yam_arena.constants import READY, FINGER_OPEN as O, FINGER_CLOSED as C
from yam_arena.demo import ArmPlan, Waypoint, grip_rotation
from yam_arena.urdf import axis_angle_matrix
from yam_arena.scripted import YamScriptedPolicy
from inspect_robots_yam.packing import split

TILT=axis_angle_matrix(np.array([0,1,0]),-np.pi/3)

class WritingPlan(ArmPlan):
    def target(self,t):
        pos,rot,finger,label=super().target(t)
        if label=='done' and hasattr(self,'last_target'):
            return self.last_target
        from answer_sheet.embodiment import FEEDBACK
        if label not in ('approach','lower','grasp','lift','hold lifted'):
            rot=TILT@grip_rotation()
            if label=='tilt':
                rot=ik.slerp_matrix(grip_rotation(),rot,ik.smoothstep((t-10)/3))
            if label.startswith(('carry','touch','stroke','raise','hover')) and FEEDBACK:
                # Waypoints for these phases describe the desired physical tip.
                tip=FEEDBACK['tip'];tcp,actual_rot=self.actual_pose
                if label.startswith('carry'):
                    if hasattr(self,'tip_offset'):del self.tip_offset
                    pos=pos+tcp-tip
                else:
                    if not hasattr(self,'tip_offset'):
                        self.tip_offset=tip-tcp
                        self.bias=np.zeros(3)
                    if label.startswith(('touch','stroke')):
                        self.bias=np.clip(self.bias+.03*(pos-tip),[-.006,-.006,-.003],[.006,.006,.003])
                    pos=pos-self.tip_offset+self.bias
        self.last_target=(pos.copy(),rot.copy(),finger,label)
        return self.last_target

class WritingPolicy(YamScriptedPolicy):
    def reset(self,scene):
        top=self._task.module.PAPER_TOP
        def w(d,x,y,z,f,label):return Waypoint(d,(x,y,top+z),f,label)
        points=[w(2,.24,-.24,.09,O,'approach'),w(2,.24,-.24,.018,O,'lower'),w(1,.24,-.24,.018,C,'grasp'),w(3,.24,-.24,.09,C,'lift'),w(2,.24,-.24,.09,C,'hold lifted'),w(3,.24,-.24,.09,C,'tilt')]
        for row,answer in enumerate(self._task.module.ANSWERS):
            x,y=self._task.module.BOXES[row][answer]
            a=(x,y+.007);b=(x-.005,y+.002);c=(x+.007,y-.007)
            points += [w(3,*a,.035,C,'carry'),w(2,*a,.0018,C,'touch'),w(1,*a,.0018,C,'touch hold'),w(2,*b,.0018,C,'stroke one'),w(3,*c,.0018,C,'stroke two'),w(2,*c,.035,C,'raise')]
        points += [w(2,.24,-.24,.08,C,'hover end')]
        self._plans={'right':WritingPlan(points,self._bases['right'])}
        p=self._plans['right'];p.actual_pose=ik.tcp_pose(np.asarray(READY),self._bases['right'])
        self._end=p.duration

    def act(self,obs):
        q=split(obs.state['joint_pos'])[1]
        self._plans['right'].actual_pose=ik.tcp_pose(q[:6],self._bases['right'])
        return super().act(obs)

def make_policy(**kwargs):
    return WritingPolicy(**kwargs)
