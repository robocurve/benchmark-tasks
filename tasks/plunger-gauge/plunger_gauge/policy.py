"""Pose-feedback brush sweep; only robot joint targets are commanded."""
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY,FINGER_OPEN as O,FINGER_CLOSED as C
from yam_arena.demo import Waypoint,ArmPlan,grip_rotation
from yam_arena.scripted import YamScriptedPolicy
from yam_arena.urdf import quat_xyzw_to_matrix
from inspect_robots_yam.packing import split

def make_plan(top):
 def w(t,x,y,z,f,label):return Waypoint(t,(x,y,top+z),f,label)
 left=[w(2,.20,.27,.13,O,'approach'),w(2,.20,.27,.074,O,'lower'),w(1,.20,.27,.074,C,'grasp'),w(2,.20,.27,.13,C,'lift')]
 for i,y in enumerate((.14,.04)):
  left += [w(2,.35,y,.13,C,f'carry {i}'),w(1,.35,y,.13,C,f'align {i}'),w(2.5,.35,y,.100,C,f'enter {i}'),w(2,.35,y,.079,C,f'press {i}'),w(1,.35,y,.079,C,f'hold {i}'),w(2,.35,y,.13,C,f'withdraw {i}')]
 left += [w(2,.20,.27,.13,C,'return'),w(3,.20,.27,.074,C,'park'),w(1,.20,.27,.074,O,'release'),w(2,.20,.27,.18,O,'retreat')]
 return {'left':left,'right':[w(sum(p.duration for p in left),.18,-.25,.20,O,'wait')]}

class FeedbackPlan(ArmPlan):
 def target(self,t):
  pos,rot,finger,label=super().target(t)
  from plunger_gauge.embodiment import DEMO_STATES
  if self.side=='left' and 'punch' in DEMO_STATES and label not in ('approach','lower','grasp','release','retreat','done'):
   tool=DEMO_STATES['punch'];tcp,actual_rot=self.actual_pose;tr=quat_xyzw_to_matrix(tool.quat_xyzw)
   correction=tr.T@actual_rot@grip_rotation().T
   self.correction=ik.slerp_matrix(getattr(self,'correction',np.eye(3)),correction,.12)
   rot=self.correction@grip_rotation()
   desired_center=pos+(0,0,-.023)
   pos=desired_center+tcp-tool.pos
   self.last=(pos.copy(),rot.copy())
  elif self.side=='left' and hasattr(self,'last'):
   if label=='release':pos,rot=self.last
   elif label=='retreat':
    previous,rot=self.last;pos=pos+previous-(.20,.27,self.top+.074)
  return pos,rot,finger,label
class PlungerPolicy(YamScriptedPolicy):
 def reset(self,scene):
  plans=self.plan_for(scene.instruction,scene.target)
  self._plans={side:FeedbackPlan(list(ws),self._bases[side]) for side,ws in plans.items()}
  for side,p in self._plans.items():p.side=side;p.top=self._task.table_top;p.actual_pose=ik.tcp_pose(np.asarray(READY),self._bases[side])
  self._end=max(p.duration for p in self._plans.values())
 def act(self,obs):
  for side,q in zip(('left','right'),split(obs.state['joint_pos'])):self._plans[side].actual_pose=ik.tcp_pose(q[:6],self._bases[side])
  return super().act(obs)
def make_policy(**kwargs):return PlungerPolicy(**kwargs)
