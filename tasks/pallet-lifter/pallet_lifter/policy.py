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
 left=[w(2,.14,.28,.14,O,'approach'),w(2,.14,.28,.051,O,'lower'),w(1,.14,.28,.051,C,'grasp'),
 w(2,.14,.28,.116,C,'lift tool'),w(3,.04,.08,.116,C,'carry in front'),w(2,.04,.08,.026,C,'lower outside pallet'),
 w(6,.20,.08,.026,C,'insert tines'),w(1,.20,.08,.026,C,'align under pallet'),w(3,.20,.08,.106,C,'lift pallet'),
 # Measured release compensation: +8 mm x, -10 mm y; scoring targets stay fixed.
 w(7,.178,-.130,.106,C,'carry pallet'),w(3,.178,-.130,.046,C,'lower pallet'),w(1,.178,-.130,.046,C,'settle pallet'),
 w(2,.178,-.130,.031,C,'lower fork clear'),w(4,-.025,-.130,.031,C,'withdraw fork'),w(2,-.025,-.130,.116,C,'lift clear'),
 w(3,.14,.28,.116,C,'return'),w(2,.14,.28,.051,C,'park'),w(1,.14,.28,.051,O,'release'),w(2,.14,.28,.17,O,'retreat')]
 return {'left':left,'right':[w(sum(p.duration for p in left),.18,-.25,.20,O,'wait')]}

class FeedbackPlan(ArmPlan):
 def target(self,t):
  pos,rot,finger,label=super().target(t)
  from pallet_lifter.embodiment import DEMO_STATES
  if self.side=='left' and 'lifter' in DEMO_STATES and label not in ('approach','lower','grasp','release','retreat','done'):
   tool=DEMO_STATES['lifter'];tcp,actual_rot=self.actual_pose;tr=quat_xyzw_to_matrix(tool.quat_xyzw)
   correction=tr.T@actual_rot@grip_rotation().T
   self.correction=ik.slerp_matrix(getattr(self,'correction',np.eye(3)),correction,.12)
   rot=self.correction@grip_rotation()
   desired_center=pos+(.1175,0,-.0035)
   if label == 'carry pallet' and 'pallet' in DEMO_STATES:
    # Correct measured payload slip using robot motion; the pallet remains free.
    desired_payload_xy=pos[:2]+(.20,0.)
    desired_tool_xy=desired_payload_xy-DEMO_STATES['pallet'].pos[:2]+tool.pos[:2]
    self.payload_bias=desired_tool_xy-desired_center[:2]
    desired_center[:2]=desired_tool_xy
   elif label in ('lower pallet','settle pallet','lower fork clear','withdraw fork','lift clear'):
    desired_center[:2]+=getattr(self,'payload_bias',np.zeros(2))
   pos=desired_center+tcp-tool.pos
   self.last=(pos.copy(),rot.copy())
  elif self.side=='left' and hasattr(self,'last'):
   if label=='release':pos,rot=self.last
   elif label=='retreat':
    previous,rot=self.last;pos=pos+previous-(.14,.28,self.top+.051)
  return pos,rot,finger,label
class PalletPolicy(YamScriptedPolicy):
 def reset(self,scene):
  plans=self.plan_for(scene.instruction,scene.target)
  self._plans={side:FeedbackPlan(list(ws),self._bases[side]) for side,ws in plans.items()}
  for side,p in self._plans.items():p.side=side;p.top=self._task.table_top;p.actual_pose=ik.tcp_pose(np.asarray(READY),self._bases[side])
  self._end=max(p.duration for p in self._plans.values())
 def act(self,obs):
  for side,q in zip(('left','right'),split(obs.state['joint_pos'])):self._plans[side].actual_pose=ik.tcp_pose(q[:6],self._bases[side])
  return super().act(obs)
def make_policy(**kwargs):return PalletPolicy(**kwargs)
