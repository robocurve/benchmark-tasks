"""Two-stage table-supported turnover using ordinary fingertip pushes.

Upright -> side wall -> bottom-up. Targets follow measured rigid-body poses;
no carton pose/velocity/joint state is written. Native feasibility is unproved.
"""
import math
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN,READY
from yam_arena.demo import grip_rotation
from yam_arena.urdf import quat_xyzw_to_matrix
from config import TOP
from model import HEIGHT,WIDTH
from scene import layout


def rx(angle):
 c,s=math.cos(angle),math.sin(angle);return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def body_frame(poses):
 p,R=poses['north'];return p-R@np.array([0,WIDTH/2,HEIGHT/2]),R


def roll_wrist(R,angle_deg):
 a=math.radians(angle_deg);c,t=math.cos(a),math.sin(a)
 return R@np.array([[c,-t,0],[t,c,0],[0,0,1]])

def solve_target(p,R,seed,base,starts=10):
 """Bounded deterministic search; a failed local minimum is not reach proof.

 Returns the best bounded solution without moving any actor or joint. Runtime
 still slews and guards the actual issued command; nominal success alone does
 not certify a continuous branch or physical contact.
 """
 seed=np.asarray(seed,dtype=float)
 bank=[seed,np.asarray(READY),np.array([0.,2.,1.,-1.,0.,0.]),np.array([0.,1.,2.,-1.,0.,0.])]
 rng=np.random.default_rng(847)
 bank.extend(rng.uniform(ik.LOWER,ik.UPPER) for _ in range(max(0,starts-len(bank))))
 best=None
 for initial in bank[:starts]:
  q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(R),initial,base_pos=base,iters=220,damping=.02,tol=1e-6)
  if best is None or pe+.1*re<best[1]+.1*best[2]:best=(q,pe,re)
  if pe<.0001 and re<.002:break
 return best

def brace_target(origin,R,front,press=.001,roll_deg=0.):
 z=-R[:,1];Q=np.column_stack([R[:,0],np.cross(z,R[:,0]),z])
 Q=roll_wrist(Q,roll_deg)
 return origin+R@np.array([0,WIDTH/2+.0045,.65*HEIGHT])-z*(front-press),Q

def push_target(origin,R,face,front,press=.001,roll_deg=0.):
 if face=='south':point=np.array([0,-WIDTH/2-.0045,.80*HEIGHT]);inward=np.array([0,1,0])
 elif face=='bottom':point=np.array([0,-.045,-.0015]);inward=np.array([0,0,1])
 else:raise ValueError('unknown physical push face')
 z=R@inward;x=R[:,0];hand=np.column_stack([x,np.cross(z,x),z]);contact=origin+R@point
 return contact-hand[:,2]*(front-press),roll_wrist(hand,roll_deg)

def advance_support(origin,R,world_vertices,angle):
 vertices=np.asarray(world_vertices);low=vertices[:,2].min();support=vertices[vertices[:,2]<=low+.0005];pivot=support[np.argmax(support[:,1])].copy();turn=rx(-math.radians(angle));new_R=turn@R;new_origin=pivot+turn@(origin-pivot);changed=(vertices-pivot)@turn.T+pivot
 new_origin[2]+=max(0.,TOP+.0002-changed[:,2].min())
 return new_origin,new_R

class CartonTurnoverProbe:
 def __init__(self,env):
  from turnover_safety import Geometry
  self.env=env;self.geometry=Geometry();self.parts={p['name']:p for p in layout()[0]};self.previous=env._ready.detach().cpu().numpy().reshape(-1).copy();self.failure=None;self.done=False;self.stage='approach';self.started=0.;self.starts=None;self.last_raw=0.;self.angle=0.;self.best_angle=0.;self.last_progress=0.;self.bias=[np.zeros(3),np.zeros(3)];self.ready_start=None;self.ready_stable=0;self.front=max(float(t[:,:,2].max()) for _,n,t in self.geometry.local_hand((.012,.012)) if n.startswith('tip_'));self.report={}
 def poses(self):
  result={}
  for name in self.parts:
   d=self.env._base.arena_world.get_pose_w(name);d=d.torch if hasattr(d,'torch') else d;p=d.detach().cpu().numpy()[0];result[name]=(p[:3],quat_xyzw_to_matrix(p[3:]))
  return result
 def measure(self):
  result=[]
  for side in ('left','right'):
   ids,fids=self.env._ids[side+'_robot'];d=self.env._base.scene[side+'_robot'].data.joint_pos;d=d.torch if hasattr(d,'torch') else d;q=d.detach().cpu().numpy()[0];p,R=ik.tcp_pose(q[ids],self.env.rig.bases[side]);result.append((q[ids],q[fids],p,R))
  return result
 def change(self,name,time,measured):
  self.stage=name;self.started=time;self.starts=[(m[2].copy(),m[3].copy()) for m in measured];self.bias=[np.zeros(3),np.zeros(3)];self.last_progress=time
 def command(self,time):
  poses=self.poses();measured=self.measure();origin,R=body_frame(poses);raw=-math.degrees(math.atan2(R[2,1],R[1,1]));delta=(raw-self.last_raw+180)%360-180
  if abs(delta)>15:self.failure='turnover moved more than15degrees/control interval'
  self.angle+=delta;self.last_raw=raw
  if np.linalg.norm(R[:,0]-[1,0,0])>.10:self.failure='carton yaw/pitch left planned table-roll plane'
  if self.angle>self.best_angle+.5:self.best_angle=self.angle;self.last_progress=time
  if self.starts is None:self.change('approach',time,measured)
  age=time-self.started;vertices=np.concatenate([p+(np.array([[-1,-1,-1],[-1,-1,1],[-1,1,-1],[-1,1,1],[1,-1,-1],[1,-1,1],[1,1,-1],[1,1,1]])*np.array(self.parts[n]['size'])/2)@Q.T for n,(p,Q) in poses.items()])
  if vertices[:,2].min()<TOP-.003:self.failure='carton passed more than3mm below tabletop'
  park=ik.tcp_pose(self.env._ready.detach().cpu().numpy().reshape(-1)[:6],self.env.rig.bases['left']);right_goal=push_target(origin,R,'south',self.front,0.);left_goal=brace_target(origin,R,self.front)
  targets=[left_goal,right_goal];contact=[('north','north_top'),('south','south_top')]
  if self.stage in ('approach','contact'):
   duration=4. if self.stage=='approach' else 2.;blend=ik.smoothstep(age/duration)
   for i,(p,Q) in enumerate(targets):
    if self.stage=='approach':p=p-Q[:,2]*.035
    targets[i]=(self.starts[i][0]+blend*(p-self.starts[i][0]),ik.slerp_matrix(self.starts[i][1],Q,blend))
   if age>=duration and max(np.linalg.norm(targets[i][0]-measured[i][2]) for i in (0,1))<.004:self.change('contact' if self.stage=='approach' else 'first_roll',time,measured)
  elif self.stage in ('first_roll','second_roll'):
   second=self.stage=='second_roll';limit=180. if second else 90.;lead=max(0.,min(1.2,limit-self.angle));goal_o,goal_R=advance_support(origin,R,vertices,lead);targets[1]=push_target(goal_o,goal_R,'bottom' if second else 'south',self.front,.001)
   if second:contact[1]=('north_bottom','south_bottom','east_bottom','west_bottom')
   if second or self.angle>25.:targets[0]=park;contact[0]=()
   if self.angle>=limit-2. and age>1:self.change('finish_clear' if second else 'clear_first',time,measured)
   if time-self.last_progress>5:self.failure='table-roll push produced no measured angular progress for5s'
  elif self.stage=='clear_first':
   targets=[park,(self.starts[1][0]-self.starts[1][1][:,2]*min(.10,.04*age),self.starts[1][1])];contact=[(),()]
   if age>=3:self.change('bottom_hover',time,measured)
  elif self.stage in ('bottom_hover','bottom_contact'):
   p,Q=push_target(origin,R,'bottom',self.front,0.);targets[0]=park;contact[0]=();contact[1]=('north_bottom','south_bottom','east_bottom','west_bottom')
   if self.stage=='bottom_hover':p=p-Q[:,2]*.06
   blend=ik.smoothstep(age/3);targets[1]=(self.starts[1][0]+blend*(p-self.starts[1][0]),ik.slerp_matrix(self.starts[1][1],Q,blend))
   if age>=3 and np.linalg.norm(p-measured[1][2])<.004:self.change('bottom_contact' if self.stage=='bottom_hover' else 'second_roll',time,measured)
  elif self.stage=='finish_clear':
   targets=[park,(self.starts[1][0]-self.starts[1][1][:,2]*min(.12,.04*age),self.starts[1][1])];contact=[(),()]
   if age>=3 and 175<=self.angle<=185 and vertices[:,2].max()<TOP+HEIGHT+.03:self.change('return_ready',time,measured)
  ready_q=None;ready_grip=None;ready_done=False
  if self.stage=='return_ready':
   if self.ready_start is None:
    self.ready_start=self.previous.copy();self.ready_started=time
    ready=self.env._ready.detach().cpu().numpy().reshape(-1)
    self.ready_duration=max(3.,1.5*max(float(np.max(np.abs(ready[i:i+6]-self.ready_start[i:i+6]))) for i in (0,7))/.15)
   ready=self.env._ready.detach().cpu().numpy().reshape(-1);fraction=ik.smoothstep((time-self.ready_started)/self.ready_duration)
   ready_q=[self.ready_start[i:i+6]+fraction*(ready[i:i+6]-self.ready_start[i:i+6]) for i in (0,7)]
   ready_grip=[self.ready_start[i]+fraction*(ready[i]-self.ready_start[i]) for i in (6,13)]
   targets=[ik.tcp_pose(q,self.env.rig.bases[side]) for q,side in zip(ready_q,('left','right'))];contact=[(),()]
   settled=fraction>=1. and all(np.max(np.abs(measured[i][0]-ready[i*7:i*7+6]))<.01 and np.max(np.abs(measured[i][1]-FINGER_OPEN))<.001 for i in (0,1))
   self.ready_stable=self.ready_stable+1 if settled else 0;ready_done=self.ready_stable>=5
   if time-self.ready_started>self.ready_duration+5.:self.failure='READY return did not settle within bounded path budget'
  if self.stage!='return_ready' and age>18:self.failure='turnover phase exceeded18s without reaching measured endpoint'
  if self.failure or self.done:return self.previous.copy(),[dict(phase=self.stage,failure=self.failure,complete=self.done,roll_angle_deg=self.angle)]
  output=self.previous.copy();errors=[]
  for i,side in enumerate(('left','right')):
   p,Q=targets[i]
   if ready_q is not None:q=ready_q[i];pe=re=0.
   else:
    self.bias[i]=np.clip(self.bias[i]+.06*(p-measured[i][2]),-.008,.008)
    q,pe,re=solve_target(p+self.bias[i],Q,measured[i][0],self.env.rig.bases[side],starts=4)
   if pe>.004 or re>.06:self.failure=side+' turnover IK unavailable'
   output[i*7:i*7+6]=q;output[i*7+6]=ready_grip[i] if ready_grip is not None else .012/abs(FINGER_OPEN);errors.append(float(np.linalg.norm(p-measured[i][2])))
  slew=np.array([.025]*6+[.04]+[.025]*6+[.04]);output=np.clip(output,self.previous-slew,self.previous+slew);output=np.clip(output,self.env._low,self.env._high)
  obs={n:(p,Q,self.parts[n]['size']) for n,(p,Q) in poses.items()};tool=self.env._base.arena_world.get_pose_w('cutter');tool=tool.torch if hasattr(tool,'torch') else tool;tool=tool.detach().cpu().numpy()[0];tr=quat_xyzw_to_matrix(tool[3:])
  for name,size,offset in [('handle',(.1,.025,.025),(0,0,0)),('guard',(.02,.03,.02),(.052,0,.002)),('blade',(.014,.0005,.008),(.062,0,-.016))]:obs['cutter_'+name]=(tool[:3]+tr@offset,tr,size)
  checks=[]
  for i,side in enumerate(('left','right')):
   other=1-i;pieces=self.geometry.parts(('left','right')[other],measured[other][0],measured[other][1]);g=self.geometry.guard(side,measured[i][0],measured[i][1],obs,pieces,intended=bool(contact[i]),intended_targets=contact[i]);checks.append(g)
   if not g['safe'] or g['target_gap_m']<-.0006:self.failure=side+' measured turnover clearance failed'
   pieces=self.geometry.parts(('left','right')[other],output[other*7:other*7+6],[FINGER_OPEN*output[other*7+6]]*2);g=self.geometry.guard(side,output[i*7:i*7+6],[FINGER_OPEN*output[i*7+6]]*2,obs,pieces,intended=bool(contact[i]),intended_targets=contact[i]);checks.append(g)
   if not g['safe'] or g['target_gap_m']<-.002:self.failure=side+' issued turnover clearance failed'
  report=dict(phase=self.stage,failure=self.failure,complete=self.done,roll_angle_deg=self.angle,tracking_errors_m=errors,clearance=checks,table_supported=bool(abs(vertices[:,2].min()-TOP)<=.003),lowest_vertex_above_table_m=float(vertices[:,2].min()-TOP))
  # A rejected command is not issued. No extra env.step on guard failure.
  if self.failure:return self.previous.copy(),[report]
  ready_issued=ready_q is not None and np.max(np.abs(output-self.env._ready.detach().cpu().numpy().reshape(-1)))<1e-7
  self.done=bool(ready_done and ready_issued);report['complete']=self.done
  self.previous=output.copy();return output,[report]
