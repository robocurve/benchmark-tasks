"""Measured state feasibility expert. Commands robot joints only."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import os,json,math
import numpy as np
from inspect_robots.types import Action,ActionChunk
from inspect_robots_yam.packing import pack
from yam_arena.scripted import YamScriptedPolicy
from yam_arena import ik
from yam_arena.constants import PAD_TIP_PAST_TCP as D,FINGER_OPEN as O,FINGER_CLOSED as C
from yam_arena.urdf import quat_xyzw_to_matrix
import yam_articulated_base as common

def bounds(spec):
 lo=np.min([np.array(p['pos'])-np.array(p['size'])/2 for p in spec.parts],axis=0)
 hi=np.max([np.array(p['pos'])+np.array(p['size'])/2 for p in spec.parts],axis=0)
 return lo,(lo+hi)/2

def rz(a):
 c,s=math.cos(a),math.sin(a)
 return np.array([[c,-s,0],[s,c,0],[0,0,1.]])

def aligned(obj,goal,tol=.0025):
 R=quat_xyzw_to_matrix(obj.quat_xyzw)
 yaw=abs(math.atan2(R[1,0],R[0,0]));tilt=math.acos(np.clip(R[2,2],-1,1))
 return np.max(np.abs(obj.pos-goal))<tol and yaw<math.radians(3) and tilt<math.radians(3) and np.linalg.norm(obj.lin_vel)<.03

class FeedbackExpert(YamScriptedPolicy):
 def __init__(self,**kwargs):
  super().__init__(**kwargs);self.info=replace(self.info,name='yam_feedback')
 def reset(self,scene):
  super().reset(scene)
  names={s.name for s in self._task.objects}
  self.kind='chocks' if 'rolling_cradle' in names else 'hopper' if 'receiving_tray' in names else 'fork' if 'fork' in names else 'unsupported'
  self._initial={};self._fork_release=None;self._hop_ready=None;self._hop_caught=None;self._gate_release=None;self._tray_release=None
  self._chock_done={a:None for a in ['left','right']};self._both_time=None;self._load_release=None;self._release_pose={};self._actions=0;self._empty_counts={'left':0,'right':0};self._abort=False;self._irreversible_failure=False
  self._left_end=self._plans['left'].duration
  self._chock_place=None
  if self.kind=='chocks':
   for t in np.arange(4.,self._left_end,self._dt):
    if self._plans['left'].target(t)[2]/O>.02:self._chock_place=float(t-self._dt);break
   assert self._chock_place is not None
  print('MEASURED_SCRIPTED_EXPERT',self.kind,str(self._task.path),flush=True)
 def read_state(self,t):
  states=getattr(common,'EXPERT_STATES',{});joints=getattr(common,'EXPERT_JOINTS',{});self._irreversible_failure=bool(getattr(common,'EXPERT_FAILURE',False))
  path=os.environ.get('YAM_TELEMETRY')
  if not states and path and Path(path).exists():
   with Path(path).open('rb') as f:
    f.seek(0,2);f.seek(max(0,f.tell()-131072));lines=f.read().splitlines()
   for line in reversed(lines):
    try:row=json.loads(line)
    except (ValueError,UnicodeDecodeError):continue
    if t-row['t']>.3:raise RuntimeError('Measured state receipt is stale')
    states={n:SimpleNamespace(pos=np.array(v['pos']),quat_xyzw=np.array(v['quat']),lin_vel=np.array(v['lin_vel']),ang_vel=np.array(v['ang_vel']),low=np.array(v['low']),high=np.array(v['high'])) for n,v in row['objects'].items()};joints=row['joints'];self._irreversible_failure=bool(row['info'].get('irreversible_failure',False));break
  if t>1. and not states:raise RuntimeError('Measured state receipt is absent')
  if states and not self._initial:self._initial={n:v.pos.copy() for n,v in states.items()}
  return states,joints
 def pose(self,arm,joints):
  j=joints[arm+'_robot'];by=dict(zip(j['names'],j['q']));q=np.array([by['joint'+str(i)] for i in range(1,7)])
  p,R=ik.tcp_pose(q,self._bases[arm]);return q,p,R
 def released(self,key,t,ee,R):
  if key not in self._release_pose:self._release_pose[key]=(t,ee.copy(),R.copy())
  at,p,rot=self._release_pose[key];target=p+np.array([0,0,min(.10,max(0,t-at-.7)*.06)])
  return target,rot,O,None,None,None
 def fork_command(self,arm,t,states,joints,ee,R):
  target,rot,finger,_=self._plans[arm].target(t);obj=None;goal=None;goalR=None
  m=self._task.module
  if arm=='right' and t>=6.1 and self._fork_release is None:
   obj=states['fork'];goal=target+np.array([0,.0675,-.0025-D])
   if t>=9.1:goal[0]=states['fixture'].pos[0]
   if t>=13.55:
    goal=np.array([states['fixture'].pos[0],-.0475,m.TOP+.067]);finger=C
    if aligned(obj,goal):self._fork_release=t
  if arm=='right' and self._fork_release is not None:return self.released('fork',t,ee,R)
  if arm=='left' and t>=3.8 and (self._fork_release is None or t<self._fork_release+.8):
   obj=states['fixture'];goal=self._initial['fixture'];finger=C
  elif arm=='left' and self._fork_release is not None:return self.released('fixture',t,ee,R)
  return target,rot,finger,obj,goal,goalR
 def hopper_command(self,arm,t,states,joints,ee,R):
  m=self._task.module;tray=states['receiving_tray'];_,centre=bounds(m.TRAY)
  trayR=quat_xyzw_to_matrix(tray.quat_xyzw);root=tray.pos-trayR@centre
  catch_root=np.array([.26,-.11,m.TOP+.055]);catch_goal=catch_root+centre
  if self._hop_ready is None and aligned(tray,catch_goal,tol=.007):self._hop_ready=t
  q=float(joints['hopper']['q'][0])
  caught=all(abs(states['part_'+str(i)].pos[0]-root[0])<.090 and abs(states['part_'+str(i)].pos[1]-root[1])<.045 and root[2]+.001<states['part_'+str(i)].low[2]<root[2]+.050 for i in range(3))
  if self._hop_caught is None and q>math.radians(80) and caught:self._hop_caught=t
  if arm=='left':
   vt=min(t,8.8) if self._hop_caught is None else 15.8+t-self._hop_caught
   target,rot,finger,_=self._plans[arm].target(vt)
   obj=tray if t>=3.8 else None
   goal=target-np.array([0,0,D])+centre-np.array(m.GRASP_LOCAL) if obj is not None else None
   if self._hop_caught is not None and vt>=20.8 and self._tray_release is None:
    goal=np.array(m.TRAY.pos)+centre;finger=C
    if aligned(tray,goal):self._tray_release=t
   if self._tray_release is not None:return self.released('tray',t,ee,R)
   return target,rot,finger,obj,goal,None
  vt=min(t,9.5) if self._hop_ready is None else 9.5+t-self._hop_ready
  target,rot,finger,_=self._plans[arm].target(min(vt,15.3))
  if q>math.radians(88) and self._gate_release is None:self._gate_release=t
  if self._gate_release is not None:return self.released('gate',t,ee,R)
  if vt<11.8:return target,rot,finger,None,None,None
  finger=C
  a=math.atan2(target[1]+.16,target[0]-.30)-math.atan2(.17,-.04)
  a=float(np.clip((a+math.pi)%(2*math.pi)-math.pi,0,math.pi/2))
  anchor=np.array([.30,-.16,m.TOP+.268]);knob=anchor+rz(q)@np.array([-.04,.17,.030])
  obj=SimpleNamespace(pos=knob,quat_xyzw=np.array([0,0,math.sin(q/2),math.cos(q/2)]))
  return target,rot,finger,obj,target-np.array([0,0,D]),rz(a)
 def chock_command(self,arm,t,states,joints,ee,R):
  m=self._task.module;sign=1 if arm=='left' else -1
  if self._both_time is None:
   vt=min(t,self._chock_place);target,rot,finger,_=self._plans[arm].target(vt)
   obj=states[arm+'_chock'];goal=None
   if t>=3.8:
    goal=target-np.array([0,0,D])+np.array([0,-sign*.0555,-.014]);finger=C
    if t>=self._chock_place-.6:goal=np.array([.28,sign*.1145,m.TOP+.036])
    if t>=self._chock_place and aligned(obj,goal,tol=.002):
     if self._chock_done[arm] is None:self._chock_done[arm]=t
   if self._chock_done[arm] is not None:
    if all(v is not None for v in self._chock_done.values()):self._both_time=t
    return self.released(arm+'_chock',t,ee,R)
   return target,rot,finger,obj if goal is not None else None,goal,None
  if arm=='left':return self.released('left_chock',t,ee,R)
  if t<self._both_time+2.5:return self.released('right_chock',t,ee,R)
  vt=self._left_end+t-self._both_time-2.5
  target,rot,finger,_=self._plans[arm].target(vt)
  if vt>=self._left_end+7.3 and self._load_release is None:
   low,centre=bounds(m.LOAD);pad=m.PAD;goal=np.array(pad.pos)+np.array([0,0,pad.size[2]/2-low[2]])+centre
   obj=states['load'];finger=C
   if aligned(obj,goal):self._load_release=t
   if self._load_release is None:return target,rot,finger,obj,goal,None
  if self._load_release is not None:return self.released('load',t,ee,R)
  return target,rot,finger,None,None,None
 def act(self,observation):
  t=float(observation.state_time)+self._dt;states,joints=self.read_state(t)
  if not states:return super().act(observation)
  if self.kind=='unsupported':raise RuntimeError('No measured expert for this task')
  if self._irreversible_failure:
   held=[]
   for arm in ['left','right']:
    q,_,_=self.pose(arm,joints);j=joints[arm+'_robot'];by=dict(zip(j['names'],j['q']));grip=float(np.clip((by['joint7']+by['joint8'])/(2*O),0,1));held.append(np.r_[q,grip])
   print('IRREVERSIBLE_TASK_FAILURE',self.kind,t,flush=True)
   return ActionChunk(actions=[Action(data=pack(*held),meta={'request_stop':True})],control_hz=1/self._dt)
  values=[];receipt={}
  for arm in ['left','right']:
   nominal=None
   if self.kind=='fork' and t<(6.1 if arm=='right' else 3.8):nominal=t
   elif self.kind=='hopper':
    if arm=='left' and t<3.8:nominal=t
    if arm=='right' and self._gate_release is None:
     vt=min(t,9.5) if self._hop_ready is None else 9.5+t-self._hop_ready
     if vt<11.8:nominal=vt
   elif self.kind=='chocks':
    if self._both_time is None and t<3.8:nominal=t
    elif arm=='right' and self._both_time is not None and t>=self._both_time+2.5:
     vt=self._left_end+t-self._both_time-2.5
     if vt<self._left_end+3.8:nominal=vt
   if nominal is not None:
    values.append(self._plans[arm].action7(nominal)[0]);continue
   measured,ee,eeR=self.pose(arm,joints)
   handler={'fork':self.fork_command,'hopper':self.hopper_command,'chocks':self.chock_command}[self.kind]
   target,rot,finger,obj,goal,goalR=handler(arm,t,states,joints,ee,eeR)
   if obj is not None and goal is not None:
    by=dict(zip(joints[arm+'_robot']['names'],joints[arm+'_robot']['q']))
    width=max(abs(by['joint7']),abs(by['joint8']))
    self._empty_counts[arm]=self._empty_counts[arm]+1 if width<.004 else 0
    if self._empty_counts[arm]>=10:
     self._abort=True;print('GRASP_NOT_ESTABLISHED',self.kind,arm,t,flush=True)
   if obj is not None and goal is not None:
    actualR=quat_xyzw_to_matrix(obj.quat_xyzw);desiredR=np.eye(3) if goalR is None else goalR
    if goalR is None:
     target=goal+(ee-obj.pos)
    else:
     target=goal+desiredR@actualR.T@(ee-obj.pos);rot=desiredR@actualR.T@eeR
    receipt[arm]=float(np.linalg.norm(goal-obj.pos))
   delta=target-ee;length=np.linalg.norm(delta)
   if length>.010:target=ee+delta*(.010/length)
   angle=math.acos(np.clip((np.trace(eeR.T@rot)-1)/2,-1,1))
   if angle>math.radians(3):rot=ik.slerp_matrix(eeR,rot,math.radians(3)/angle)
   q,pe,re=ik.solve(target,ik.matrix_to_quat_xyzw(rot),measured,base_pos=self._bases[arm],iters=80)
   self._plans[arm].q=q
   values.append(np.r_[q,float(np.clip(finger/O,0,1))])
  self._actions+=1
  if self._actions%20==0:
   path=Path(os.environ['YAM_TELEMETRY']).parent/'feedback_receipts.jsonl'
   with path.open('a') as f:f.write(json.dumps(dict(t=t,kind=self.kind,position_errors_m=receipt))+'\n')
  meta={'request_stop':True} if t>45 or self._abort else {}
  return ActionChunk(actions=[Action(data=pack(*values),meta=meta)],control_hz=1/self._dt)

def policy(**kwargs):return FeedbackExpert(**kwargs)
