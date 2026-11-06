"""Physical two-link assembly milestone."""
from dataclasses import replace
import numpy as np
from yam_arena import ik
from yam_arena.scripted import YamScriptedPolicy
from inspect_robots.types import Action,ActionChunk
from chain_task.geometry import TABLE,rx
from chain_task.motion import Target,pickup,parent_normalization
from chain_task.policy import FirstClipPolicy
from chain_task.handoff import HandoffClipPolicy

def parent_rod(f,parent="link1"):
 return f['poses'][parent]+f['rotations'][parent]@np.array([-.002,-.030,0.])
def child_rotation(f,parent="link1"):
 n=f['rotations'][parent][:,0].copy();n/=np.linalg.norm(n);y=np.array([0.,0.,1.])-n*n[2];y/=np.linalg.norm(y);return np.column_stack([np.cross(y,n),y,n])

class SecondClipPolicy(HandoffClipPolicy):
 child="link0";parent="link1";child_side="left";parent_side="right";parent_yaw=-10.;center=np.array([.30,.16,.96]);counter_diagonal=np.diag([1.,-1.,-1.])
 def reset(self,scene):super().reset(scene);self.dynamic=[]
 def frame(self,side,f):
  p,r=f['poses'][self.child],f['rotations'][self.child];h=f['hands'][side];self.frames[side]=(r.T@(np.array(h['tcp'])-p),np.array(h['rotation']).T@r)
 def relative_targets(self,points,f):
  rd=child_rotation(f,self.parent);rod=parent_rod(f,self.parent);targets=[];self.dynamic=[]
  for seconds,local,label in points:
   local=np.array(local);target=self.body(seconds,self.child_side,rod-rd@local,rd,label);targets.append(target);self.dynamic.append((target,local))
  return targets
 def act(self,o):
  from chain_task.embodiment import FEEDBACK as f
  t=float(o.state_time)+self._dt;p,r=f['poses'][self.child],f['rotations'][self.child]
  if self.phase=='start':
   self.before=p.copy();self.set(self.parent_side,[self.ready(600,self.parent_side)],t,f);self.stage('second_pickup',self.child_side,pickup(p,r,1,'spine'),t,f)
  if t>=self.end:
   phase=self.phase
   if phase not in ['second_pickup','release_parent','abort'] and p[2]<TABLE+.06:self.dynamic=[];self.abort(t,f,'Second carabiner lost from physical grasp')
   elif phase=='second_pickup':
    if p[2]-self.before[2]<.08:self.abort(t,f,'Second pickup did not lift')
    else:
     self.set(self.child_side,[self.hold(600,self.child_side,f)],t,f)
     pp,pr=f['poses'][self.parent],f['rotations'][self.parent];tool=pr@self.counter_diagonal@rx(np.deg2rad(-15));tcp=pp+pr@np.array([-.016,0,0.])-tool[:,2]*.004;self.stage('counterhold_parent',self.parent_side,[Target(3,tcp-tool[:,2]*.020,tool,.18,'approach hanging spine'),Target(3,tcp,tool,.18,'enter parent spine pinch'),Target(2,tcp,tool,0.,'counterhold first link'),Target(1,tcp,tool,0.,'settle physical counterhold')],t,f)
   elif phase=='counterhold_parent':
    h=f['hands'][self.parent_side];targets,_,_=parent_normalization(f['poses'][self.parent],f['rotations'][self.parent],np.array(h['tcp']),np.array(h['rotation']),self.parent_yaw);self.stage('standardize_parent',self.parent_side,targets,t,f)
   elif phase=='standardize_parent':
    self.set(self.parent_side,[self.hold(600,self.parent_side,f)],t,f);self.frame(self.child_side,f);rd=child_rotation(f,self.parent);center=self.center;self.stage('orient_child',self.child_side,[self.body(4,self.child_side,center,rd,'orient second link'),self.body(2,self.child_side,center,rd,'settle second orientation')],t,f)
   elif phase=='orient_child':
    self.set(self.parent_side,[self.hold(600,self.parent_side,f)],t,f);self.frame(self.child_side,f);targets=self.relative_targets([(4,[.022,.020,.070],'approach outside parent plane'),(4,[.022,.020,0.],'align second gate'),(2,[.022,.020,0.],'settle second alignment')],f);self.stage('second_align',self.child_side,targets,t,f)
   elif phase=='second_align':
    self.frame(self.child_side,f);targets=self.relative_targets([(4,[.007,.020,0.],'press second gate'),(3,[.007,.024,0.],'round second free tip'),(3,[.002,.024,0.],'enter previous link'),(3,[.002,.024,0.],'allow second spring closure')],f);self.stage('second_clip',self.child_side,targets,t,f)
   elif phase=='second_clip':
    self.dynamic=[];h=f['hands'][self.child_side];hp,hr=np.array(h['tcp']),np.array(h['rotation']);print('CHAIN_SECOND_BEFORE_RELEASE',f['angles'],flush=True);self.stage('release_child',self.child_side,[Target(2,hp,hr,.3,'release second spine'),Target(3,hp-hr[:,2]*.07,hr,.3,'withdraw second clipping hand'),self.ready(4,self.child_side)],t,f)
   elif phase=='release_child':
    h=f['hands'][self.parent_side];hp,hr=np.array(h['tcp']),np.array(h['rotation']);self.stage('release_parent',self.parent_side,[Target(2,hp,hr,.3,'release first spine'),Target(3,hp-hr[:,2]*.08,hr,.3,'withdraw counterhold'),self.ready(4,self.parent_side),self.ready(12,self.parent_side,'observe freely suspended pair')],t,f)
   elif phase in ['release_parent','abort']:self.stopping=True;self.phase='done';self.end=t+600;print('CHAIN_TWO_LINK_RESULT',f.get('score'),f.get('score_reasons'),flush=True)
  if self.dynamic:
   rd,rod=child_rotation(f,self.parent),parent_rod(f,self.parent);off,rel=self.frames[self.child_side]
   for target,local in self.dynamic:target.pos=rod-rd@local+rd@off;target.rot=rd@rel.T
  try:target=np.concatenate([self.plans[s].action7(t-self.clocks[s])[0] for s in ['left','right']])
  except RuntimeError as exc:
   self.dynamic=[];self.abort(t,f,str(exc));target=np.concatenate([self.plans[s].action7(0.)[0] for s in ['left','right']])
  actual=np.asarray(o.state['joint_pos']);self.integral+=1.5*self._dt*(target-actual);self.integral[[6,13]]=0.;self.integral=np.clip(self.integral,-.25,.25)
  return ActionChunk(actions=[Action(data=target+self.integral,meta={'request_stop':True} if self.stopping else {})],control_hz=1/self._dt)

class PairPolicy(YamScriptedPolicy):
 def __init__(self,**kwargs):
  super().__init__(**kwargs);self.first=FirstClipPolicy(grasp_style='spine',index=1,grasp_flip=0,**kwargs);self.second=SecondClipPolicy(**kwargs);self.info=replace(self.info,name='carabiner_two_link_scripted')
 def bind(self,info):super().bind(info);self.first.bind(info);self.second.bind(info)
 def reset(self,scene):self.first.reset(scene);self._scene=scene;self._second_started=False
 def act(self,o):
  if self._second_started:return self.second.act(o)
  action=self.first.act(o)
  if self.first.stop_at is not None:
   from chain_task.embodiment import FEEDBACK as f
   if self.first.mechanism_proved:
    self.second.reset(self._scene);self._second_started=True;print('CHAIN_FIRST_LINK_PROOF_BEFORE_SECOND',o.state_time,dict(actual_benchmark_score=f['score'],linked_component_quiet_seconds=self.first.proof_seconds,full_scorer_unchanged=True),flush=True);return self.second.act(o)
  return action

def make_policy(**kwargs):return PairPolicy(**kwargs)
