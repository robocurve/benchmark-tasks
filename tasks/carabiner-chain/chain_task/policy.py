"""Initial quiet/reset probe; physical clipping controller follows preflight."""
from dataclasses import replace
import numpy as np
from yam_arena.constants import READY
from yam_arena.scripted import YamScriptedPolicy
from inspect_robots.types import Action,ActionChunk
class HoldPolicy(YamScriptedPolicy):
 def reset(self,scene):pass
 def act(self,observation):return ActionChunk(actions=[Action(data=np.r_[READY,1.,READY,1.],meta={'request_stop':True} if observation.state_time>=20 else {})],control_hz=1/self._dt)
def make_hold_policy(**kwargs):return HoldPolicy(**kwargs)

class FirstClipPolicy(YamScriptedPolicy):
 """One-link physical milestone. No second/third assembly is scripted yet."""
 def __init__(self,grasp_style='arch',grasp_flip=0,index=2,**kwargs):super().__init__(**kwargs);self.grasp_style=grasp_style;self.grasp_flip=int(grasp_flip);self.index=int(index);self.info=replace(self.info,name={"arch":"carabiner_first_clip_scripted","long":"carabiner_long_clip_scripted","spine":"carabiner_spine_clip_scripted"}[grasp_style])
 def reset(self,scene):
  self.phase='start';self.plan=None;self.start=0.;self.integral=np.zeros(14);self.stop_at=None;self.mechanism_proved=False;self.proof_seconds=0.
 def begin(self,phase,targets,t,f):
  from chain_task.motion import Plan,transition_grip
  hand=f['hands']['right'];self.plan=Plan(targets,'right',hand['joints'],transition_grip(self.plan,targets,float(-np.mean(hand['fingers'])/.04695)));self.start=t;self.end=t+self.plan.duration;self.phase=phase;self.integral[:]=0.;print('CHAIN_POLICY',phase,t,flush=True)
 def body_target(self,seconds,p,r,label,grip=0.):
  from chain_task.motion import Target
  return Target(seconds,p+r@self.offset,r@self.relative.T,grip,label)
 def act(self,observation):
  from chain_task.embodiment import FEEDBACK as f
  from chain_task.motion import pickup,Target
  from chain_task.geometry import rx,rz,TABLE
  from yam_arena import ik,station
  t=float(observation.state_time)+self._dt
  if self.phase=='start':
   p,r=f['poses']['link'+str(self.index)],f['rotations']['link'+str(self.index)];self.before=p.copy();self.face=1 if r[2,2]>0 else -1;self.begin('pickup',pickup(p,r,self.grasp_flip,style=self.grasp_style),t,f)
  if self.phase=='release' and t-self.start>=9.:
   checks=f.get('score_reasons',{});name='link'+str(self.index);speeds=checks.get('body_speeds',{});valid=not checks.get('invalid',False) and checks.get('edges',{}).get('eye')==[name] and checks.get('closed',{}).get(name,False) and checks.get('released',False) and all(k in speeds and speeds[k]['linear_m_s']<=.01 and speeds[k]['angular_rad_s']<=.2 for k in [name,'gate'+str(self.index)]);self.proof_seconds=self.proof_seconds+self._dt if valid else 0.;self.mechanism_proved=self.proof_seconds>=2.-1e-9
   if self.mechanism_proved:self.end=t
  if t>=self.end:
   p,r=f['poses']['link'+str(self.index)],f['rotations']['link'+str(self.index)];hand=f['hands']['right'];tcp=np.array(hand['tcp']);tool=np.array(hand['rotation'])
   if self.phase=='pickup':
    if p[2]-self.before[2]<.08:raise RuntimeError('Physical frame grasp did not lift carabiner')
    self.offset=r.T@(tcp-p);self.relative=tool.T@r;self.desired=rz(np.deg2rad(-10))@rx(self.face*np.pi/2);self.pre=np.array([.42,0,TABLE+.2225])-self.desired@np.array([.022,.020 if self.grasp_style!='arch' else .005,0]);self.begin('orient',[self.body_target(4,np.array([.34,-.16,.94]),self.desired,'orient loose link'),self.body_target(2,np.array([.34,-.16,.94]),self.desired,'settle oriented grasp')],t,f)
   elif self.phase=='orient':
    if p[2]<TABLE+.06:raise RuntimeError('Carabiner lost during rotation')
    self.offset=r.T@(tcp-p);self.relative=tool.T@r;self.begin('align',[self.body_target(4,self.pre+[0,-.09,0],self.desired,'approach eye from side'),self.body_target(4,self.pre,self.desired,'align gate outside eye'),self.body_target(2,self.pre,self.desired,'settle alignment')],t,f)
   elif self.phase=='align':
    # Re-measure the ordinary grasp to compensate contact slip; never move a body.
    self.offset=r.T@(tcp-p);self.relative=tool.T@r
    if self.grasp_style!='arch':
     rod=np.array([.42,0,TABLE+.2225]);points=[(4,np.array([.007,.020,0.]),'press gate near free end'),(3,np.array([.007,.024,0.]),'round free gate tip'),(3,np.array([.002,.024,0.]),'enter closed frame'),(3,np.array([.002,.024,0.]),'allow spring closure')];targets=[self.body_target(seconds,rod-self.desired@local,self.desired,label) for seconds,local,label in points]
    else:
     end=self.pre+self.desired@np.array([.028,0,0]);targets=[self.body_target(6,end,self.desired,'push eye rod through passive gate'),self.body_target(3,end,self.desired,'allow spring closure')]
    self.begin('clip',targets,t,f)
   elif self.phase=='clip':
    self.begin('release',[Target(2,tcp,tool,1.,'open fingers'),Target(3,tcp-tool[:,2]*.09,tool,1.,'withdraw from hanging link'),Target(4,*ik.tcp_pose(np.array(READY),station.BASES['right']),1.,'return right ready'),Target(30,*ik.tcp_pose(np.array(READY),station.BASES['right']),1.,'released hanging observation')],t,f)
   elif self.phase=='release':self.stop_at=t;self.end=t+100.;self.phase='done';print('CHAIN_MILESTONE_RESULT',f.get('score'),f.get('score_reasons'),flush=True)
  a,label=self.plan.action7(t-self.start);target=np.r_[READY,1.,a];actual=np.asarray(observation.state['joint_pos']);self.integral+=1.5*self._dt*(target-actual);self.integral[[6,13]]=0.;self.integral=np.clip(self.integral,-.25,.25)
  return ActionChunk(actions=[Action(data=target+self.integral,meta={'request_stop':True} if self.stop_at is not None else {})],control_hz=1/self._dt)

def make_first_clip_policy(**kwargs):return FirstClipPolicy(**kwargs)

def make_long_clip_policy(**kwargs):return FirstClipPolicy(grasp_style="long",grasp_flip=1,index=1,**kwargs)

def make_spine_clip_policy(**kwargs):return FirstClipPolicy(grasp_style="spine",index=1,grasp_flip=0,**kwargs)
