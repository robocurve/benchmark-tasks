"""Pose-feedback rake sweep; only robot joint targets are commanded."""
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY,FINGER_OPEN as O,FINGER_CLOSED as C
from yam_arena.demo import Waypoint,ArmPlan,grip_rotation
from yam_arena.scripted import YamScriptedPolicy
from yam_arena.urdf import quat_xyzw_to_matrix
from inspect_robots_yam.packing import split

def make_plan(top):
 def w(t,x,y,z,f,label):return Waypoint(t,(x,y,top+z),f,label)
 left=[w(2,.14,.27,.16,O,'approach'),w(2,.14,.27,.109,O,'lower'),w(1,.14,.27,.109,C,'grasp'),w(2,.14,.27,.13,C,'lift')]
 for i,y in enumerate((.075,.13,.185)):
  left += [w(2.5,.35,y,.13,C,f'carry {i}'),w(2,.35,y,.095,C,f'enter from rear {i}'),w(1,.35,y,.095,C,f'align {i}'),w(6,.09,y,.095,C,f'rake channel {i}'),w(1,.09,y,.095,C,f'settle {i}'),w(2,.09,y,.13,C,f'lift clear {i}')]
 left += [w(2,.14,.27,.13,C,'return'),w(2,.14,.27,.109,C,'park'),w(1,.14,.27,.109,O,'release'),w(2,.14,.27,.19,O,'retreat')]
 return {'left':left,'right':[w(sum(p.duration for p in left),.18,-.25,.20,O,'wait')]}

class FeedbackPlan(ArmPlan):
 def desired_center(self,pos):return pos+(.08875,0,-.0335)
 def may_advance(self):
  from chip_sweep.embodiment import DEMO_STATES
  pos,_,_,label=ArmPlan.target(self,self.clock)
  if not label.startswith(('enter from rear','rake channel')):return True
  if 'rake' not in DEMO_STATES:return False
  obj=DEMO_STATES['rake'];center=self.desired_center(pos)
  R=quat_xyzw_to_matrix(obj.quat_xyzw)
  angle=np.arccos(np.clip((np.trace(R)-1)/2,-1.,1.))
  return np.linalg.norm(center[:2]-obj.pos[:2])<.001 and abs(center[2]-obj.pos[2])<.0015 and angle<np.radians(.6)
 def target(self,t):
  pos,rot,finger,label=super().target(t)
  from chip_sweep.embodiment import DEMO_STATES
  if self.side=='left' and 'rake' in DEMO_STATES and label not in ('approach','lower','grasp','release','retreat','done'):
   obj=DEMO_STATES['rake'];tcp,tcp_rot=self.actual_pose
   actual=quat_xyzw_to_matrix(obj.quat_xyzw);correction=actual.T
   center=self.desired_center(pos)
   if not hasattr(self,'bias'):self.bias=np.zeros(3);self.rotation_bias=np.eye(3)
   # Integral correction removes joint-drive tracking error without changing drives.
   if label.startswith(('enter','align','rake channel','park')):
    self.bias=np.clip(self.bias+.04*(center-obj.pos),-.012,.012);self.bias[2]=np.clip(self.bias[2],-.004,.004)
    self.rotation_bias=ik.slerp_matrix(np.eye(3),correction,.04)@self.rotation_bias
    q=ik.matrix_to_quat_xyzw(self.rotation_bias);angle=2*np.arccos(np.clip(abs(q[3]),0,1))
    if angle>np.radians(6):self.rotation_bias=ik.slerp_matrix(np.eye(3),self.rotation_bias,np.radians(6)/angle)
   rot=self.rotation_bias@correction@tcp_rot
   pos=center+correction@(tcp-obj.pos)+self.bias
   self.last=(pos.copy(),rot.copy())
  elif self.side=='left' and hasattr(self,'last'):
   if label=='release':pos,rot=self.last
   elif label=='retreat':
    previous,rot=self.last;pos=pos+previous-(.14,.27,self.top+.109)
  return pos,rot,finger,label
class ChipPolicy(YamScriptedPolicy):
 def reset(self,scene):
  plans=self.plan_for(scene.instruction,scene.target)
  self._plans={side:FeedbackPlan(list(ws),self._bases[side]) for side,ws in plans.items()}
  for side,p in self._plans.items():p.side=side;p.top=self._task.table_top;p.actual_pose=ik.tcp_pose(np.asarray(READY),self._bases[side]);p.clock=0.;p.blocked=0
  self._end=max(p.duration for p in self._plans.values())
 def act(self,obs):
  from inspect_robots.types import Action,ActionChunk
  from inspect_robots_yam.packing import pack
  for side,q in zip(('left','right'),split(obs.state['joint_pos'])):self._plans[side].actual_pose=ik.tcp_pose(q[:6],self._bases[side])
  actions=[]
  for side in ('left','right'):
   p=self._plans[side]
   if side!='left' or p.may_advance():p.clock+=self._dt;p.blocked=0
   else:p.blocked+=1
   action,_=p.action7(p.clock);actions.append(action)
  failed=any(p.blocked>240 for p in self._plans.values())
  done=all(p.clock>p.duration+self.hold_s for p in self._plans.values())
  return ActionChunk(actions=[Action(data=pack(*actions),meta={'request_stop':True} if failed or done else {})],control_hz=1/self._dt)

def make_policy(**kwargs):return ChipPolicy(**kwargs)
