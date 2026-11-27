"""Predict contact dynamics and execute ordinary joint-controlled paddle strokes."""
import numpy as np,math
from inspect_robots.policy import PolicyBase,PolicyInfo
from inspect_robots.types import Action,ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik
from .config import *
from .embodiment import FEEDBACK
from .dynamics import intercept,choose_stroke,impact
class RallyPolicy(PolicyBase):
 def __init__(self):self.info=PolicyInfo(name='rally_scripted',action_space=action_box(np.r_[ik.LOWER,0,ik.LOWER,0],np.r_[ik.UPPER,1,ik.UPPER,1]))
 def bind(self,info):pass
 def reset(self,scene):self.q=Q0.copy();self.plan=None;self.last_hits=0;self.last_p=HANDLE.copy();self.last_R=GRIP_ROT.copy();self.plan_phase=None;self.trace=[]
 def act(self,obs):
  now=float(obs.state_time);p=HANDLE.copy();R=GRIP_ROT;hits=FEEDBACK.get('hits',0)
  if hits!=self.last_hits:self.plan=None;self.last_hits=hits
  b=np.array(FEEDBACK.get('ball',[.68,.25,1.]));v=np.array(FEEDBACK.get('velocity',[0,0,0]));phase=FEEDBACK.get('phase');w=float(FEEDBACK.get('angular_velocity',[0,0,0])[1])
  outbound=hits>0 and phase=='wall' and v[0]>.1 and .43<b[0]<WALL_CENTRE
  inbound=phase in ('table','paddle') and v[0]<-.1 and b[0]>.35
  replan=hits>0 and self.plan_phase!=phase
  if now>2.02 and (outbound or inbound) and (self.plan is None or replan):
   pred=intercept(b[[0,2]],v[[0,2]],w,phase,.35,TABLE,wall=WALL_CENTRE)
   if pred:
    dt,z,vin,omega=pred;choice=choose_stroke(z,vin,omega,.35,WALL_CENTRE,TABLE)
    # A repeatable feed has a nominal preparation stroke. Replan once
    # the real table bounce provides measured velocity and spin.
    if hits==0 and phase=='table':
     theta=.16;speed=.55;vout,_=impact(vin,omega,[math.cos(theta),math.sin(theta)],speed)
     choice=(0.,theta,speed,vout,TABLE+.16,0.)
    if choice and .80<z<1.00:
     cost,theta,speed,vout,hn,eta=choice;n=np.array([math.cos(theta),0,math.sin(theta)]);target=np.array([.35,b[1]+v[1]*dt,z])-.0265*n;self.plan=(now+dt,target,n,theta,speed);self.plan_phase=phase
     self.trace.append(dict(t=now,hit_time=now+dt,z=z,vin=vin.tolist(),omega=omega,theta=theta,speed=speed,predicted_out=vout.tolist(),predicted_next_z=hn,phase=phase))
  if 1.<now<=2.02:
   u=ik.smoothstep((now-1.)/.65);n=np.array([math.cos(.16),0.,math.sin(.16)])
   preparation=np.array([.35,.12,.9555078554])-.0815*n
   p=(1-u)*HANDLE+u*preparation;R=ry(-.16*u)@GRIP_ROT
   self.last_p=p.copy();self.last_R=R.copy()
  if self.plan:
   when,target,n,theta,speed=self.plan;stroke=float(np.clip(now+.12-when,-.10,.10))*speed;head=target+n*stroke;p=head-np.array([0,.13,0]);R=ry(-theta)@GRIP_ROT;self.last_p=p.copy();self.last_R=R.copy()
  elif now>2.:p=self.last_p.copy();R=self.last_R
  q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(R),self.q,base_pos=BASES['left'],iters=180)
  if pe>.008 or re>.05:raise RuntimeError(f'IK intercept target{p}, {pe}, {re}')
  self.q=q;state=np.asarray(obs.state['joint_pos']);return ActionChunk(actions=[Action(data=np.r_[q,.10,state[7:13],1.])],control_hz=20.)
 def transcript(self):return {'plans':self.trace}
def make_policy(**kw):return RallyPolicy(**kw)
