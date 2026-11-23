"""Two-stage table-supported turnover using ordinary fingertip pushes.

Upright -> side wall -> bottom-up. Targets follow measured rigid-body poses;
no carton pose/velocity/joint state is written. Native feasibility is unproved.
"""
import math
import copy
import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN,READY
from yam_arena.demo import grip_rotation
from yam_arena.urdf import quat_xyzw_to_matrix
from config import TOP,CUTTER_PAD_START
from model import HEIGHT,WIDTH
from scene import layout
from turnover_targets import wall_target,tip_vertices


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
  self.env=env;self.geometry=Geometry();self.parts={p['name']:p for p in layout()[0]};self.previous=env._ready.detach().cpu().numpy().reshape(-1).copy();self.failure=None;self.done=False;self.brace_clear=None;self.phase_goal_q=None;self.stage='narrow_ready';self.started=0.;self.starts=None;self.last_raw=0.;self.angle=0.;self.best_angle=0.;self.last_progress=0.;self.bias=[np.zeros(3),np.zeros(3)];self.ready_start=None;self.ready_stable=0;self.settle_stable=0;self.tip_vertices=tip_vertices(self.geometry);self.front=max(float(t[:,:,2].max()) for _,n,t in self.geometry.local_hand((.012,.012)) if n.startswith('tip_'));self.report={};self.contact_stable=0;self.contact_previous_tcp=None;self.contact_previous_time=None;self.contact_acquire_started=None;self.force_last_sample_id=None;self.force_last_control_time=None;self.force_last_clock=None;self.force_last_mode=None;self.bottom_acquire_started=None;self.force_contact_loss_since=None
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
  self.stage=name;self.started=time;self.phase_goal_q=None;self.start_q=[m[0].copy() for m in measured];self.starts=[(m[2].copy(),m[3].copy()) for m in measured];self.bias=[np.zeros(3),np.zeros(3)];self.last_progress=time
  if name in ('contact','bottom_contact'):
   self.contact_stable=0;self.contact_previous_tcp=None;self.contact_previous_time=None
   if name=='contact':self.contact_acquire_started=None
   else:self.bottom_acquire_started=None
 def _candidate_state(self):
  # Observed carton angles/progress are retained; phase/planner/PI changes
  # become effective only after the complete issued vector passes the guard.
  fixed={'env','geometry','parts','tip_vertices','front','previous','angle','last_raw','best_angle','force_last_sample_id','force_last_control_time','force_last_clock','force_last_mode'}
  return {k:copy.deepcopy(v) for k,v in self.__dict__.items() if k not in fixed}
 def _restore_candidate_state(self,state):
  fixed={'env','geometry','parts','tip_vertices','front','previous','angle','last_raw','best_angle','force_last_sample_id','force_last_control_time','force_last_clock','force_last_mode'}
  for k in list(self.__dict__):
   if k not in fixed and k not in state:del self.__dict__[k]
  for k,v in state.items():setattr(self,k,copy.deepcopy(v))
 def contact_forces(self,time,R,mode='walls'):
  """Consume one verified native20Hz sample; signs are fingertip reactions."""
  from turnover_contacts import TARGETS
  if mode not in ('walls','bottom'):raise ValueError('unknown force contact mode')
  reader=getattr(self.env,'_turnover_contact_reader',None)
  if reader is None:raise ValueError('native turnover contact reader is missing')
  sample=reader.read(time,phase='pre_action')
  if sample.get('clock_verified') is not True:raise ValueError('native force clock is unverified')
  requested=sample.get('requested_control_time_s');sid=sample.get('sample_id')
  if not isinstance(sid,int) or isinstance(sid,bool) or not isinstance(requested,(int,float)) or not math.isfinite(requested) or abs(requested-time)>1e-6:raise ValueError('invalid force sample id/control clock')
  clocks=sample.get('sensor_clock_s',{});updates=sample.get('sensor_update_time_s',{})
  clock=np.array([clocks[s] for s in ('left','right')],float);update=np.array([updates[s] for s in ('left','right')],float)
  if not np.isfinite(clock).all() or not np.isfinite(update).all() or np.max(np.abs(clock-update))>1e-5 or abs(clock[0]-clock[1])>1e-5:raise ValueError('stale or unsynchronised native force clock')
  last_id=getattr(self,'force_last_sample_id',None);last_time=getattr(self,'force_last_control_time',None);last_clock=getattr(self,'force_last_clock',None)
  if last_id is not None:
   if sid<=last_id or np.any(clock<=last_clock):raise ValueError('stale/backwards native force measurement')
   if last_time is not None and getattr(self,'force_last_mode',None)==mode:
    dt=time-last_time
    if abs(dt-.05)>1e-5 or np.max(np.abs(clock-last_clock-dt))>1e-4:raise ValueError('stale/backwards/non20Hz force measurement')
  signed=[]
  for i,side in enumerate(('left','right')):
   data=sample['sides'][side];force=np.asarray(data['family_normal_force_on_fingertips_w_N'],float)
   if force.shape!=(3,) or not np.isfinite(force).all() or (len(data.get('body_names',()))!=2 or set(data['body_names'])!={'tip_left','tip_right'}) or len(data.get('configured_target_family',()))!=len(TARGETS) or set(data['configured_target_family'])!=set(TARGETS):raise ValueError('unverified finite carton force family')
   normal=R[:,1] if i==0 else -R[:,1] if mode=='walls' else -R[:,2]
   signed.append(float(force@normal))
  self.force_last_sample_id=sid;self.force_last_control_time=time;self.force_last_clock=clock.copy();self.force_last_mode=mode
  return dict(sample_id=sid,contact_mode=mode,required_sides=['left','right'] if mode=='walls' else ['right'],signed_normal_force_N=signed,both_opposed_normal_forces=bool(min(signed)>.1),required_force_contacts=bool(min(signed)>.1 if mode=='walls' else signed[1]>.1),clock_verified=True,sensor_clock_s=clock.tolist())
 def command(self,time):
  poses=self.poses();measured=self.measure();origin,R=body_frame(poses);raw=-math.degrees(math.atan2(R[2,1],R[1,1]));delta=(raw-self.last_raw+180)%360-180
  if abs(delta)>15:self.failure='turnover moved more than15degrees/control interval'
  self.angle+=delta;self.last_raw=raw
  if np.linalg.norm(R[:,0]-[1,0,0])>.10:self.failure='carton yaw/pitch left planned table-roll plane'
  if self.angle>self.best_angle+.5:self.best_angle=self.angle;self.last_progress=time
  if self.starts is None:self.change('narrow_ready',time,measured)
  force_report=None
  if self.stage in ('first_roll','second_roll') and hasattr(self,'wall_preload_offsets' if self.stage=='first_roll' else 'bottom_preload_offset'):
   try:
    force_report=self.contact_forces(time,R,'walls' if self.stage=='first_roll' else 'bottom')
    required=force_report['required_force_contacts'] if self.stage=='second_roll' or self.angle<=25 else force_report['signed_normal_force_N'][1]>.1
    if required:self.force_contact_loss_since=None
    elif self.force_contact_loss_since is None:self.force_contact_loss_since=time
    elif time-self.force_contact_loss_since>.25:self.failure='acquired contact force lost before planned release'
   except (ValueError,KeyError,TypeError,IndexError) as error:self.failure='rolling contact force evidence: '+str(error)
  candidate_state=self._candidate_state();stage_before=self.stage
  age=time-self.started;vertices=np.concatenate([p+(np.array([[-1,-1,-1],[-1,-1,1],[-1,1,-1],[-1,1,1],[1,-1,-1],[1,-1,1],[1,1,-1],[1,1,1]])*np.array(self.parts[n]['size'])/2)@Q.T for n,(p,Q) in poses.items()])
  if vertices[:,2].min()<TOP-.003:self.failure='carton passed more than3mm below tabletop'
  park=ik.tcp_pose(self.env._ready.detach().cpu().numpy().reshape(-1)[:6],self.env.rig.bases['left']);right_goal=wall_target(origin,R,'right',HEIGHT,WIDTH,self.tip_vertices,0.);left_goal=wall_target(origin,R,'left',HEIGHT,WIDTH,self.tip_vertices,0.)
  targets=[left_goal,right_goal];contact=[('north',),('south','south_top')]
  forced_q={};path_q=None;acquire_side=None;contact_acquiring=False;contact_press=None
  if self.stage=='narrow_ready':
   targets=[ik.tcp_pose(np.asarray(READY),self.env.rig.bases[side]) for side in ('left','right')];contact=[(),()];path_q=[np.asarray(READY)]*2
   if age>=2 and all(np.max(np.abs(m[1]+.012))<.001 for m in measured):self.change('approach',time,measured)
  elif self.stage in ('approach','contact'):
   duration=8. if self.stage=='approach' else 3.
   if self.phase_goal_q is None:
    self.phase_goal_q=[]
    for i,(side,(p,Q)) in enumerate(zip(('left','right'),targets)):
     normal=np.array([0.,-1. if side=='left' else 1.,0.])
     if self.stage=='approach':p=p-normal*.05
     q,pe,re=solve_target(p,Q,measured[i][0],self.env.rig.bases[side],starts=10)
     if pe>.0003 or re>.003:self.failure=side+' initial contact path endpoint IK failed'
     self.phase_goal_q.append(q)
   blend=ik.smoothstep(age/duration)
   path_q=[start+blend*(goal-start) for start,goal in zip(self.start_q,self.phase_goal_q)]
   targets=[ik.tcp_pose(q,self.env.rig.bases[side]) for q,side in zip(path_q,('left','right'))]
   if self.stage=='approach':contact=[(),()]
   if self.stage=='approach' and age>=duration and all(np.max(np.abs(q-m[0]))<.02 for q,m in zip(path_q,measured)):self.change('contact',time,measured)
   if stage_before=='contact':
    try:force_report=self.contact_forces(time,R)
    except (ValueError,KeyError,TypeError,IndexError) as error:self.failure='contact acquisition force evidence: '+str(error)
    endpoint=all(np.max(np.abs(q-m[0]))<.02 for q,m in zip(path_q,measured))
    if force_report is not None and age>=3. and (endpoint or self.contact_acquire_started is not None):
     if self.contact_acquire_started is None:self.contact_acquire_started=time
     contact_acquiring=True;path_q=None
     signed=force_report['signed_normal_force_N'];acquire_side=0 if signed[0]<=.1 else 1 if signed[1]<=.1 else None
     press=min(.001,.00015*max(0.,time-self.contact_acquire_started))
     targets=[]
     for i,side in enumerate(('left','right')):
      if i==acquire_side:targets.append(wall_target(origin,R,side,HEIGHT,WIDTH,self.tip_vertices,press))
      else:
       forced_q[i]=self.previous[i*7:i*7+6].copy();targets.append(ik.tcp_pose(forced_q[i],self.env.rig.bases[side]))
     contact_press=press
  elif self.stage in ('first_roll','second_roll'):
   second=self.stage=='second_roll';limit=180. if second else 90.;lead=max(0.,min(.3,limit-self.angle));goal_o,goal_R=advance_support(origin,R,vertices,lead);targets[1]=push_target(goal_o,goal_R,'bottom',self.front,.0002,roll_deg=90.) if second else wall_target(goal_o,goal_R,'right',HEIGHT,WIDTH,self.tip_vertices,.0002)
   if not second and hasattr(self,'wall_preload_offsets'):
    p,Q=wall_target(origin,R,'left',HEIGHT,WIDTH,self.tip_vertices,0.);targets[0]=(p+R@self.wall_preload_offsets[0],Q@self.wall_preload_rotations[0])
    p,Q=wall_target(goal_o,goal_R,'right',HEIGHT,WIDTH,self.tip_vertices,.0002);targets[1]=(p+goal_R@self.wall_preload_offsets[1],Q@self.wall_preload_rotations[1])
   if second and hasattr(self,'bottom_preload_offset'):
    p,Q=push_target(goal_o,goal_R,'bottom',self.front,.0002,roll_deg=90.);targets[1]=(p+goal_R@self.bottom_preload_offset,Q@self.bottom_preload_rotation)
   if second:contact[1]=('north_bottom','south_bottom','east_bottom','west_bottom')
   if second:targets[0]=park;contact[0]=()
   elif self.angle>25.:
    if self.brace_clear is None:
     p,Q=measured[0][2:];clear,pe,re=solve_target(p+np.array([0.,.05,.05]),Q,measured[0][0],self.env.rig.bases['left'],starts=10)
     if pe>.0003 or re>.003:self.failure='left brace withdrawal IK failed'
     self.brace_clear=(time,measured[0][0].copy(),clear)
    t0,start,clear=self.brace_clear;elapsed=time-t0
    qleft=start+ik.smoothstep(elapsed/3.)*(clear-start) if elapsed<3 else clear+ik.smoothstep((elapsed-3)/3.)*(np.asarray(READY)-clear)
    targets[0]=ik.tcp_pose(qleft,self.env.rig.bases['left']);forced_q[0]=qleft;contact[0]=('north',) if elapsed<6 else ()
    if elapsed<6:
     forced_q[1]=self.previous[7:13].copy();targets[1]=ik.tcp_pose(forced_q[1],self.env.rig.bases['right']);self.last_progress=time
   # Gravity torque about the measured leading support edge must favour
   # the requested roll. Use actual panel COM, not a fixed aspect-ratio angle.
   support=vertices[vertices[:,2]<=vertices[:,2].min()+.0005]
   pivot_y=float(support[:,1].max())
   total_mass=sum(part['mass'] for part in self.parts.values())
   com_y=sum(self.parts[n]['mass']*float(pos[1]) for n,(pos,_) in poses.items())/total_mass
   self.report['com_beyond_leading_support_m']=com_y-pivot_y
   if com_y-pivot_y>.003 and age>1:
    self.change('finish_clear' if second else 'clear_first',time,measured);self.settle_stable=0
   if time-self.last_progress>5:self.failure='table-roll push produced no measured angular progress for5s'
  elif self.stage=='clear_first':
   targets=[park,(self.starts[1][0]+np.array([0.,-.05,.05])*ik.smoothstep(age/3.),self.starts[1][1])];contact=[(),('south','south_top')]
   self.settle_stable=self.settle_stable+1 if 85<=self.angle<=95 else 0
   if age>=3 and self.settle_stable>=10:self.change('bottom_hover',time,measured)
  elif self.stage in ('bottom_hover','bottom_contact'):
   p,Q=push_target(origin,R,'bottom',self.front,0.,roll_deg=90.);targets[0]=park;contact[0]=();contact[1]=('north_bottom','south_bottom','east_bottom','west_bottom')
   if self.stage=='bottom_hover':p=p-Q[:,2]*.06
   blend=ik.smoothstep(age/3);targets[1]=(self.starts[1][0]+blend*(p-self.starts[1][0]),ik.slerp_matrix(self.starts[1][1],Q,blend))
   if self.stage=='bottom_hover' and age>=3 and np.linalg.norm(p-measured[1][2])<.004:self.change('bottom_contact',time,measured)
   if stage_before=='bottom_contact':
    try:force_report=self.contact_forces(time,R,'bottom')
    except (ValueError,KeyError,TypeError,IndexError) as error:self.failure='bottom contact acquisition force evidence: '+str(error)
    if force_report is not None and age>=3 and (np.linalg.norm(p-measured[1][2])<.004 or self.bottom_acquire_started is not None):
     if self.bottom_acquire_started is None:self.bottom_acquire_started=time
     contact_acquiring=True;acquire_side=1 if not force_report['required_force_contacts'] else None
     forced_q[0]=self.previous[:6].copy();targets[0]=ik.tcp_pose(forced_q[0],self.env.rig.bases['left'])
     contact_press=min(.001,.00015*max(0.,time-self.bottom_acquire_started))
     if acquire_side==1:targets[1]=push_target(origin,R,'bottom',self.front,contact_press,roll_deg=90.)
     else:forced_q[1]=self.previous[7:13].copy();targets[1]=ik.tcp_pose(forced_q[1],self.env.rig.bases['right'])
  elif self.stage=='finish_clear':
   targets=[park,(self.starts[1][0]-self.starts[1][1][:,2]*min(.12,.04*age),self.starts[1][1])];contact=[(),('north_bottom','south_bottom','east_bottom','west_bottom')]
   self.settle_stable=self.settle_stable+1 if 175<=self.angle<=185 and vertices[:,2].max()<TOP+HEIGHT+.03 else 0
   if age>=3 and self.settle_stable>=10:self.change('return_ready',time,measured)
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
  if self.stage!='return_ready' and age>(35 if self.stage in ('first_roll','second_roll') else 18):self.failure='turnover phase exceeded bounded endpoint time'
  if self.failure or self.done:
   failure=self.failure;done=self.done;self._restore_candidate_state(candidate_state);self.failure=failure;self.done=done
   return self.previous.copy(),[dict(phase=self.stage,failure=self.failure,complete=self.done,roll_angle_deg=self.angle)]
  output=self.previous.copy();errors=[]
  for i,side in enumerate(('left','right')):
   p,Q=targets[i]
   if i in forced_q:q=forced_q[i];pe=re=0.
   elif path_q is not None:q=path_q[i];pe=re=0.
   elif ready_q is not None:q=ready_q[i];pe=re=0.
   else:
    self.bias[i]=np.clip(self.bias[i]+.06*(p-measured[i][2]),-.008,.008)
    q,pe,re=solve_target(p+self.bias[i],Q,measured[i][0],self.env.rig.bases[side],starts=4)
   if pe>.004 or re>.06:self.failure=side+' turnover IK unavailable'
   output[i*7:i*7+6]=q;output[i*7+6]=self.previous[i*7+6] if i in forced_q else ready_grip[i] if ready_grip is not None else .012/abs(FINGER_OPEN);errors.append(float(np.linalg.norm(p-measured[i][2])))
  slew=np.array([.025]*6+[.04]+[.025]*6+[.04]);output=np.clip(output,self.previous-slew,self.previous+slew);output=np.clip(output,self.env._low,self.env._high)
  obs={n:(p,Q,self.parts[n]['size']) for n,(p,Q) in poses.items()};tool=self.env._base.arena_world.get_pose_w('cutter');tool=tool.torch if hasattr(tool,'torch') else tool;tool=tool.detach().cpu().numpy()[0];tr=quat_xyzw_to_matrix(tool[3:])
  for name,size,offset in [('handle',(.1,.025,.025),(0,0,0)),('guard',(.02,.03,.02),(.052,0,.002)),('blade',(.014,.0005,.008),(.062,0,-.016))]:obs['cutter_'+name]=(tool[:3]+tr@offset,tr,size)
  obs['cutter_pad']=(np.array(CUTTER_PAD_START),np.eye(3),(.17,.07,.002))
  proposed_phase=self.stage;proposed_bias=[b.copy() for b in self.bias]
  def guarded_action(action):
   action=np.asarray(action,float)
   if action.shape!=(14,) or not np.isfinite(action).all():return False,[]
   if any(float(action[j])<float(self.env._low[j])-1e-10 or float(action[j])>float(self.env._high[j])+1e-10 for j in range(14)):return False,[]
   guards=[]
   for i,side in enumerate(('left','right')):
    other=1-i;pieces=self.geometry.parts(('left','right')[other],action[other*7:other*7+6],[FINGER_OPEN*action[other*7+6]]*2)
    g=self.geometry.guard(side,action[i*7:i*7+6],[FINGER_OPEN*action[i*7+6]]*2,obs,pieces,intended=bool(contact[i]),intended_targets=contact[i]);guards.append(g)
   return all(g['safe'] and math.isfinite(g['target_gap_m']) and g['target_gap_m']>=-.002 for g in guards),guards
  actual=[]
  for i,side in enumerate(('left','right')):
   other=1-i;pieces=self.geometry.parts(('left','right')[other],measured[other][0],measured[other][1])
   g=self.geometry.guard(side,measured[i][0],measured[i][1],obs,pieces,intended=bool(contact[i]),intended_targets=contact[i]);actual.append(g)
   if not np.isfinite(measured[i][0]).all() or any(float(measured[i][0][j])<float(ik.LOWER[j])-1e-6 or float(measured[i][0][j])>float(ik.UPPER[j])+1e-6 for j in range(6)):self.failure=side+' measured turnover joint bound failed'
   if not g['safe'] or not math.isfinite(g['target_gap_m']) or g['target_gap_m']<-.0006:self.failure=side+' measured turnover clearance failed'
  # Recheck the OLD command against THIS measured scene and BOTH commanded
  # arms. Holding an obsolete command is not automatically safe.
  previous_safe,previous_guards=guarded_action(self.previous)
  if not previous_safe:self.failure='previous issued turnover command no longer has fresh clearance'
  alpha=0.;issued=previous_guards;proposal_rejected=False;advance_trials=[];arm_fractions=[0.,0.];branch_name='hold';route_committed=False;integrator_committed=[]
  if not self.failure:
   proposal=output.copy();accepted=False
   if contact_acquiring:
    branches=[('hold',(0.,0.))] if acquire_side is None else [('left' if acquire_side==0 else 'right',(1.,0.) if acquire_side==0 else (0.,1.))]
   elif stage_before in ('first_roll','second_roll') and self.stage==stage_before:
    branches=[('combined',(1.,1.)),('right',(0.,1.)),('left',(1.,0.))]
   else:branches=[('combined',(1.,1.))]
   for name,mask in branches:
    fractions=(1.,.5,.25,.125,.0625) if name!='hold' else (1.,)
    for fraction in fractions:
     candidate=self.previous.copy()
     for i in (0,1):candidate[i*7:i*7+7]+=fraction*mask[i]*(proposal[i*7:i*7+7]-self.previous[i*7:i*7+7])
     safe,gg=guarded_action(candidate)
     advance_trials.append({'branch':name,'fraction':fraction,'arm_fractions':[fraction*m for m in mask],'safe':safe,'clearance':gg,'action_14d':candidate.tolist()})
     if safe:
      output=candidate;issued=gg;alpha=fraction;arm_fractions=[fraction*m for m in mask];branch_name=name;accepted=True;break
     proposal_rejected=True
    if accepted:break
   if not accepted:output=self.previous.copy();issued=previous_guards;alpha=0.
   route_committed=bool(accepted and np.max(np.abs(output-proposal))<1e-12)
   # Whole-route phase/cache changes require exactly the planned14Dvector.
   # An independently accepted full arm may commit ONLY its own PI state.
   if not route_committed:self._restore_candidate_state(candidate_state)
   old_bias=candidate_state['bias']
   for i in (0,1):
    if arm_fractions[i]==1. and i not in forced_q:
     self.bias[i]=proposed_bias[i].copy();integrator_committed.append(('left','right')[i])
    else:self.bias[i]=old_bias[i].copy()
  checks=[actual[0],issued[0] if issued else {},actual[1],issued[1] if issued else {}]
  report=dict(targets_tcp_world_m=[p.tolist() for p,Q in targets],accepted_tcp_world_m=[ik.tcp_pose(output[i*7:i*7+6],self.env.rig.bases[side])[0].tolist() for i,side in enumerate(('left','right'))],phase=self.stage,candidate_phase=proposed_phase,failure=self.failure,complete=self.done,roll_angle_deg=self.angle,tracking_errors_m=errors,com_beyond_leading_support_m=self.report.get('com_beyond_leading_support_m'),clearance=checks,previous_command_clearance=previous_guards,advance_fraction=alpha,advance_trials=advance_trials,planner_and_integrator_frozen=bool(not route_committed and not integrator_committed),planner_route_committed=route_committed,accepted_branch=branch_name,arm_advance_fractions=arm_fractions,integrator_committed_arms=integrator_committed,contact_force_evidence=force_report,contact_acquisition_press_m=contact_press,table_supported=bool(abs(vertices[:,2].min()-TOP)<=.003),lowest_vertex_above_table_m=float(vertices[:,2].min()-TOP))
  if self.failure:
   failure=self.failure;self._restore_candidate_state(candidate_state);self.failure=failure;report['failure']=failure
   return self.previous.copy(),[report]
  if stage_before in ('contact','bottom_contact') and self.stage==stage_before:
   # GEOMETRIC proximity/settling only: these signed mesh gaps are not force
   # telemetry and cannot establish load-bearing or a successful roll.
   positions=np.array([m[2] for m in measured]);prior=getattr(self,'contact_previous_tcp',None);prior_t=getattr(self,'contact_previous_time',None)
   dt=None if prior_t is None else time-prior_t
   speed=float('inf') if prior is None or dt is None or not 0<dt<=.1 else float(np.linalg.norm(positions-prior,axis=1).max()/dt)
   endpoint_settled=(path_q is not None and all(np.max(np.abs(q-m[0]))<.02 for q,m in zip(path_q,measured))) or (contact_acquiring and all(error<.003 for error in errors))
   required_ids=(0,1) if stage_before=='contact' else (1,)
   proximity=all(-.0006<=actual[i]['target_gap_m']<=.0003 for i in required_ids)
   force_ok=force_report is not None and force_report['required_force_contacts']
   settled=age>=3. and endpoint_settled and proximity and force_ok and speed<=.005 and all(np.max(np.abs(measured[i][1]+.012))<=.001 for i in required_ids)
   self.contact_stable=getattr(self,'contact_stable',0)+1 if settled else 0
   self.contact_previous_tcp=positions.copy();self.contact_previous_time=time
   report['geometric_opposed_settle']={'stable_frames':self.contact_stable,'required_frames':5,'max_tcp_speed_m_s':None if not math.isfinite(speed) else speed,'both_mesh_gaps_in_range':proximity,'contact_force_verified':force_ok,'force_threshold_N':.1,'force_sample_id':None if force_report is None else force_report['sample_id'],'contact_mode':'walls' if stage_before=='contact' else 'bottom','required_sides':['left','right'] if stage_before=='contact' else ['right']}
   if self.contact_stable>=5:
    # Store ONLY the command actually accepted above, expressed relative to
    # the measured body. PI is reset by change(); its accepted effect is now
    # explicit preload, so no accepted load is silently removed or doubled.
    if stage_before=='contact':
     offsets=[];rotations=[]
     for i,side in enumerate(('left','right')):
      p,Q=ik.tcp_pose(output[i*7:i*7+6],self.env.rig.bases[side]);base_p,base_Q=wall_target(origin,R,side,HEIGHT,WIDTH,self.tip_vertices,0.)
      offsets.append(R.T@(p-base_p));rotations.append(base_Q.T@Q)
     self.change('first_roll',time,measured);self.wall_preload_offsets=offsets;self.wall_preload_rotations=rotations
    else:
     p,Q=ik.tcp_pose(output[7:13],self.env.rig.bases['right']);base_p,base_Q=push_target(origin,R,'bottom',self.front,0.,roll_deg=90.)
     offset=R.T@(p-base_p);rotation=base_Q.T@Q
     self.change('second_roll',time,measured);self.bottom_preload_offset=offset;self.bottom_preload_rotation=rotation
    self.force_contact_loss_since=None
  ready_issued=ready_q is not None and np.max(np.abs(output-self.env._ready.detach().cpu().numpy().reshape(-1)))<1e-7
  self.done=bool(ready_done and ready_issued);report['complete']=self.done
  self.previous=output.copy();return output,[report]
