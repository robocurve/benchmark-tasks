"""One-link prototype using a physical two-arm handoff; no assembly shortcuts."""
from dataclasses import replace
import numpy as np
from yam_arena import ik,station
from yam_arena.constants import READY
from yam_arena.scripted import YamScriptedPolicy
from inspect_robots.types import Action,ActionChunk
from chain_task.motion import Plan,Target,pickup,transition_grip
from chain_task.geometry import TABLE,rx,rz

M=np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])@np.diag([-1.,-1.,1.])
TRANSFER=np.array([.30,0.,.90])
ROD=np.array([.42,0.,TABLE+.2225])
class HandoffClipPolicy(YamScriptedPolicy):
 def __init__(self,**kwargs):super().__init__(**kwargs);self.info=replace(self.info,name='carabiner_handoff_clip_scripted')
 def reset(self,scene):
  self.phase='start';self.plans={};self.clocks={};self.integral=np.zeros(14);self.frames={};self.end=0.;self.stopping=False
 def set(self,side,targets,t,f):
  h=f['hands'][side];self.plans[side]=Plan(targets,side,h['joints'],transition_grip(self.plans.get(side),targets,float(-np.mean(h['fingers'])/.04695)));self.clocks[side]=t;self.integral[:7] *= 0 if side=='left' else 1;self.integral[7:] *= 0 if side=='right' else 1
 def stage(self,name,side,targets,t,f):
  self.set(side,targets,t,f);self.phase=name;self.end=t+sum(v.seconds for v in targets);print('CHAIN_POLICY',name,t,flush=True)
 def ready(self,seconds,side,label='ready'):
  return Target(seconds,*ik.tcp_pose(np.array(READY),station.BASES[side]),1.,label)
 def frame(self,side,f):
  r=f['rotations']['link1'];p=f['poses']['link1'];h=f['hands'][side];self.frames[side]=(r.T@(np.array(h['tcp'])-p),np.array(h['rotation']).T@r)
 def body(self,seconds,side,p,r,label):
  off,rel=self.frames[side];return Target(seconds,p+r@off,r@rel.T,0.,label)
 def hold(self,seconds,side,f):
  h=f['hands'][side];return Target(seconds,np.array(h['tcp']),np.array(h['rotation']),0.,'hold physical grasp')
 def abort(self,t,f,reason):
  print('CHAIN_ABORT',reason,flush=True)
  for side in ['left','right']:self.set(side,[self.ready(4,side,'release and retreat'),self.ready(10,side,'observe released failure')],t,f)
  self.phase='abort';self.end=t+14
 def act(self,o):
  from chain_task.embodiment import FEEDBACK as f
  t=float(o.state_time)+self._dt;p=f['poses']['link1'];r=f['rotations']['link1']
  if self.phase=='start':
   self.before=p.copy();self.set('left',[self.ready(600,'left')],t,f);self.stage('pickup','right',pickup(p,r,1,'long'),t,f)
  if t>=self.end:
   phase=self.phase
   if phase not in ['pickup','release_left','abort'] and p[2]<TABLE+.06:self.abort(t,f,'Carabiner lost from physical grasp')
   elif phase=='pickup':
    if p[2]-self.before[2]<.08:self.abort(t,f,'Pickup failed')
    else:self.frame('right',f);self.stage('present','right',[self.body(5,'right',TRANSFER,r,'present horizontal carabiner'),self.body(2,'right',TRANSFER,r,'settle presentation')],t,f)
   elif phase=='present':
    self.set('right',[self.hold(600,'right',f)],t,f);tool=r@M;tcp=p+r@np.array([-.020,0.,0.]);self.stage('receive','left',[Target(3,tcp-tool[:,2]*.07,tool,.15,'approach spine'),Target(3,tcp,tool,.15,'enter spine pinch'),Target(2,tcp,tool,0.,'close spine grasp'),Target(2,tcp,tool,0.,'settle two-arm contact')],t,f)
   elif phase=='receive':
    self.set('left',[self.hold(600,'left',f)],t,f);h=f['hands']['right'];hp=np.array(h['tcp']);hr=np.array(h['rotation']);self.stage('release_right','right',[Target(2,hp,hr,1.,'release outer arches'),Target(3,hp-hr[:,2]*.02+[0,-.14,0],hr,1.,'withdraw pickup hand'),self.ready(3,'right')],t,f)
   elif phase=='release_right':
    self.frame('left',f);self.desired=rz(np.deg2rad(-10))@rx(np.pi/2);self.stage('orient_left','left',[self.body(4,'left',np.array([.32,.12,.98]),self.desired,'orient spine-held carabiner'),self.body(2,'left',np.array([.32,.12,.98]),self.desired,'settle oriented grasp')],t,f)
   elif phase=='orient_left':
    self.frame('left',f);pre=ROD-self.desired@np.array([.022,.020,0.]);self.stage('align','left',[self.body(4,'left',pre+[0,.09,0],self.desired,'approach eye'),self.body(4,'left',pre,self.desired,'align free gate end'),self.body(2,'left',pre,self.desired,'settle alignment')],t,f)
   elif phase=='align':
    self.frame('left',f);points=[(4,[.007,.020,0.],'press gate'),(3,[.007,.024,0.],'round free gate tip'),(3,[.002,.024,0.],'enter frame'),(3,[.002,.024,0.],'allow passive closure')];self.stage('clip','left',[self.body(seconds,'left',ROD-self.desired@np.array(local),self.desired,label) for seconds,local,label in points],t,f)
   elif phase=='clip':
    h=f['hands']['left'];hp=np.array(h['tcp']);hr=np.array(h['rotation']);print('CHAIN_BEFORE_RELEASE',f['angles'],flush=True);self.stage('release_left','left',[Target(2,hp,hr,.3,'release spine'),Target(3,hp-hr[:,2]*.08,hr,.3,'withdraw clipping hand'),self.ready(4,'left'),self.ready(12,'left','released hanging observation')],t,f)
   elif phase in ['release_left','abort']:self.stopping=True;self.phase='done';self.end=t+600;print('CHAIN_MILESTONE_RESULT',f.get('score'),f.get('score_reasons'),flush=True)
  values=[self.plans[s].action7(t-self.clocks[s])[0] for s in ['left','right']];target=np.concatenate(values);actual=np.asarray(o.state['joint_pos']);self.integral+=1.5*self._dt*(target-actual);self.integral[[6,13]]=0.;self.integral=np.clip(self.integral,-.25,.25)
  return ActionChunk(actions=[Action(data=target+self.integral,meta={'request_stop':True} if self.stopping else {})],control_hz=1/self._dt)
def make_policy(**kwargs):return HandoffClipPolicy(**kwargs)
