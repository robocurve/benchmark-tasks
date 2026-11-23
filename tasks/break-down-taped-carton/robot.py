"""Bimanual plain-rim grip/lift diagnostic, using ordinary YAM targets only."""
import numpy as np
from yam_arena import ik
from yam_arena.demo import grip_rotation
from yam_arena.constants import FINGER_OPEN
from config import *
class CartonGripProbe:
 def __init__(self,env):
  self.env=env;self.q=[];self.starts=[];self.bias=[np.zeros(3),np.zeros(3)];self.previous=env._ready.detach().cpu().numpy().reshape(-1).copy();self.down=grip_rotation()
  for side in ['left','right']:
   ids,_=env._ids[side+'_robot'];d=env._base.scene[side+'_robot'].data.joint_pos;d=d.torch if hasattr(d,'torch') else d;q=d.detach().cpu().numpy()[0,ids];self.q.append(q.copy());self.starts.append(ik.tcp_pose(q,env.rig.bases[side]))
 def command(self,time):
  out=self.env._ready.detach().cpu().numpy().reshape(-1).copy();report=[]
  for i,side in enumerate(['left','right']):
   sign=1 if i==0 else -1;goal=np.array((CENTER[0],sign*.09,TOP+.087));hover=goal+np.array((0,0,.10));start,rstart=self.starts[i]
   if time<3:
    phase='approach_rim';s=ik.smoothstep(time/3);p=start+s*(hover-start);r=ik.slerp_matrix(rstart,self.down,s);g=1-s*.70
   elif time<6:
    phase='enter_rim';s=ik.smoothstep((time-3)/3);p=hover+s*(goal-hover);r=self.down;g=.30
   elif time<8:
    phase='pinch_plain_rim';p=goal;r=self.down;g=.30*(1-ik.smoothstep((time-6)/2))
   elif time<11:
    phase='lift_carton';p=goal+np.array((0,0,.09*ik.smoothstep((time-8)/3)));r=self.down;g=0.
   elif time<13:
    phase='hold_carton';p=goal+np.array((0,0,.09));r=self.down;g=0.
   elif time<16:
    phase='lower_carton';p=goal+np.array((0,0,.09*(1-ik.smoothstep((time-13)/3))));r=self.down;g=0.
   elif time<18:
    phase='release_rim';p=goal;r=self.down;g=.30*ik.smoothstep((time-16)/2)
   else:
    phase='retreat';p=goal+np.array((0,0,.10*ik.smoothstep((time-18)/3)));r=self.down;g=.30
   ids,_=self.env._ids[side+'_robot'];d=self.env._base.scene[side+'_robot'].data.joint_pos;d=d.torch if hasattr(d,'torch') else d;qa=d.detach().cpu().numpy()[0,ids];tcp,_=ik.tcp_pose(qa,self.env.rig.bases[side]);self.bias[i]=np.clip(self.bias[i]+.08*(p-tcp),-.020,.020)
   self.q[i],pe,re=ik.solve(p+self.bias[i],ik.matrix_to_quat_xyzw(r),self.q[i],base_pos=self.env.rig.bases[side],iters=90);off=i*7;out[off:off+6]=self.q[i];out[off+6]=g;report.append({'side':side,'phase':phase,'target':p.tolist(),'tcp':tcp.tolist(),'position_error':float(np.linalg.norm(p-tcp)),'ik_error':pe,'ik_rotation_error':re,'grip':g})
  delta=np.array([.04]*6+[.05]+[.04]*6+[.05]);out=np.clip(out,self.previous-delta,self.previous+delta);out=np.clip(out,self.env._low,self.env._high);self.previous=out
  return out,report

class CartonCutProbe:
 """Actual blade feedback and physical retraces of remaining native tape cells."""
 fold_x=.1095
 fold_depth=-.00075
 def __init__(self,env):
  from yam_arena.demo import ArmPlan,Waypoint as W
  from model import FILM_Z
  self.env=env;self.previous=env._ready.detach().cpu().numpy().reshape(-1).copy();self.failure=None;self.done=False;self.index=0;self.motion=None;self.bias=np.zeros(3);self.last_time=0.;self.calibrated=False
  self.rest_tool=None;self.rest_time=None;self.completed_motion=None;self.repass_round=0;self.cuts_verified=False;self.coverage_report=None
  x,y,z=CUTTER_START;hover=TOP+.17;grip=-.011
  self.pickup=ArmPlan([W(2.,(x,y,hover),FINGER_OPEN),W(2.,(x,y,z+.006),FINGER_OPEN),W(1.,(x,y,z+.006),grip),W(2.,(x,y,hover),grip)],env.rig.bases['left'])
  self.park=ArmPlan([W(3.,(.13,-.24,TOP+.17),FINGER_OPEN)],env.rig.bases['right'])
  self.seam_depth=-.001
  self.phases=[
   ('staging',(0.,-.05,-.020),None),('seam_orient',None,np.pi),
   ('seam_hover',(.1065,0.,-.020),np.pi),('seam_lower',(.1065,0.,self.seam_depth),np.pi),
   ('seam_cut',(-.107,0.,self.seam_depth),np.pi),('seam_lift',(-.107,0.,-.020),np.pi),
   ('front_orient',None,-np.pi/2),('front_hover',(-self.fold_x,-.034,-.020),-np.pi/2),
   ('front_lower',(-self.fold_x,-.034,self.fold_depth),-np.pi/2),('front_cut',(-self.fold_x,.034,self.fold_depth),-np.pi/2),
   ('front_lift',(-self.fold_x,.034,-.020),-np.pi/2),('back_hover',(self.fold_x,-.034,-.020),-np.pi/2),
   ('back_lower',(self.fold_x,-.034,self.fold_depth),-np.pi/2),('back_cut',(self.fold_x,.034,self.fold_depth),-np.pi/2),
   ('back_lift',(self.fold_x,.034,-.020),-np.pi/2),('return_hover',None,None),
   ('return_lower',None,None),('release_cutter',None,None),('retreat',None,None)]

 def pose(self,name):
  from yam_arena.urdf import quat_xyzw_to_matrix
  d=self.env._base.arena_world.get_pose_w(name);d=d.torch if hasattr(d,'torch') else d;p=d.detach().cpu().numpy()[0]
  return p[:3],quat_xyzw_to_matrix(p[3:])

 def schedule_remaining_cuts(self):
  """Read the native coverage kernel; it is never mutated by this controller."""
  from feedback_geometry import missing_cut_spans
  from model import FILM_Z
  import task
  tracker=getattr(task,'_tracker',None)
  if tracker is None or not hasattr(tracker,'coverage'):
   self.failure='native cut coverage unavailable before cutter parking';return
  coverage=tracker.coverage
  self.coverage_report=coverage.groups()
  remaining=missing_cut_spans(tuple(coverage.cells),frozenset(coverage.cut))
  if not remaining:
   self.cuts_verified=True;return
  if self.repass_round>=3:
   self.failure='native tape cells remain after three measured physical retraces';return
  # Validate the entire plan before inserting anything. Boundary-cell margins
  # can otherwise drive the knife back through the observed flap-root obstacle.
  for span in remaining:
   if span['group']=='seam':
    xs=np.asarray((span['start'][0],span['end'][0]),dtype=float)
    if not np.isfinite(xs).all() or np.any(xs<-.107-1e-9) or np.any(xs>.1065+1e-9):
     self.failure=('unreviewed seam repass X span '+repr(xs.tolist())+
                   ' for '+str(span['missing_cells'])+' missing cells; reviewed interior [-0.107, 0.1065]m')
     return
  self.repass_round+=1;extra=[]
  # Carry the incoming carton-frame yaw explicitly while translating to the
  # reviewed central staging point; None would reset to pickup orientation.
  incoming_yaw=next((p[2] for p in reversed(self.phases[:self.index]) if p[2] is not None),np.pi)
  for span in remaining:
   group=span['group'];yaw=np.pi if group=='seam' else -np.pi/2
   # Keep the original certified internal-repass depth; the initial seam
   # uses the separately reviewed1mm depth to avoid its old entry contact.
   depth=(FILM_Z+.001)/2 if group=='seam' else self.fold_depth
   start=np.array(span['start']);end=np.array(span['end']);start[2]=end[2]=depth
   # Move the commanded fold plane inside its physical tape band; the
   # read-only coverage cells stay at their original wall-root coordinates.
   if group!='seam':start[0]=end[0]=-self.fold_x if group=='front_fold' else self.fold_x
   high_start=start.copy();high_start[2]=-.020;high_end=end.copy();high_end[2]=-.020
   name='repass_'+str(self.repass_round)+'_'+group
   if group=='seam':
    center=(0.,-.05,-.040)
    extra.extend([(name+'_staging',center,incoming_yaw),(name+'_orient',center,yaw)])
   else:extra.append((name+'_orient',None,yaw))
   extra.extend([(name+'_hover',tuple(high_start),yaw),
                 (name+'_lower',tuple(start),yaw),(name+'_cut',tuple(end),yaw),
                 (name+'_lift',tuple(high_end),yaw)])
   incoming_yaw=yaw
  self.phases[self.index:self.index]=extra

 def command(self,time):
  from yam_arena.urdf import axis_angle_matrix
  from feedback_geometry import blade_tcp_target,measured_blade_grasp,TrackedProgress,cut_entry_bias,cut_watchdog,matching_lower_cut,cut_endpoint_ready,matching_hover_lower,bounded_cut_integral,lower_lateral_advance
  from model import WIDTH,HEIGHT,FILM_Z
  dt=min(.05,max(.001,time-self.last_time));self.last_time=time
  ids,fids=self.env._ids['left_robot'];d=self.env._base.scene['left_robot'].data.joint_pos;d=d.torch if hasattr(d,'torch') else d;actualq=d.detach().cpu().numpy()[0];qa=actualq[ids];tcp,rhand=ik.tcp_pose(qa,self.env.rig.bases['left']);opening=float(np.clip(actualq[fids].mean()/FINGER_OPEN,0,1));cutter,rc=self.pose('cutter')
  measured_grasp,measured_tip,tip=measured_blade_grasp(tcp,rhand,cutter,rc)
  # The blade initially overlaps the 2 mm pad. Capture its settled native
  # rest pose while the hand still hovers, before descending for pickup.
  if 1.5<=time<2.:
   self.rest_tool=(cutter.copy(),rc.copy());self.rest_time=time
  b,_=self.park.action7(min(time+.05,self.park.duration))
  if self.failure or self.done:
   return self.previous.copy(),[{'side':'left','phase':'failed' if self.failure else 'cuts_verified_cutter_parked','failure':self.failure,'coverage':self.coverage_report},{'side':'right','phase':'park'}]
  if time<self.pickup.duration:
   a,_=self.pickup.action7(time+.05);out=np.r_[a,b]
   report=[{'side':'left','phase':'pickup_cutter','position_error':float(np.linalg.norm(self.pickup.target(time+.05)[0]-tcp)),'ik_error':self.pickup.worst_ik_mm/1000,'opening':opening},{'side':'right','phase':'park'}]
  else:
   if not self.calibrated:
    if cutter[2]<TOP+.07 or np.linalg.norm(cutter-tcp)>.05 or not .08<opening<.45:
     self.failure='cutter pickup not established from native height, hand distance and finger opening'
     return self.command(time)
    self.initial_tool_rotation=rc.copy();self.initial_grasp_rotation=measured_grasp.copy();self.initial_tip_in_hand=measured_tip.copy();self.calibrated=True
   # A force grasp can slip and twist by millimetres during cutting. Re-read
   # the actual relationship instead of reusing one pickup-time calibration.
   self.tool_in_hand=measured_grasp;self.tip_in_hand=measured_tip
   north,reference=self.pose('north');origin=north-reference@np.array((0,WIDTH/2,HEIGHT/2));local_tip=reference.T@(tip-origin)
   phase,local_goal,yaw=self.phases[self.index]
   cutting=phase.endswith('_cut');lowering=phase.endswith('_lower') and not phase.startswith('return_')
   handoff_hold=False
   if self.motion is None:
    hand_path=phase=='retreat'
    if phase.startswith(('return_','release_')):
     if self.rest_tool is None:
      self.failure='settled cutter rest pose was not observed before pickup';return self.command(time)
     rest_position,rest_rotation=self.rest_tool
     goal=rest_position+rest_rotation@np.array((.062,0.,-.020));tool_rotation=rest_rotation.copy()
     if phase=='return_hover':goal=goal+np.array((0,0,.11))
    elif hand_path:goal=tcp+np.array((0.,0.,.08));tool_rotation=rhand.copy()
    else:
     goal=tip.copy() if local_goal is None else origin+reference@np.array(local_goal)
     tool_rotation=self.initial_tool_rotation if yaw is None else reference@np.diag([1.,-1.,-1.])@axis_angle_matrix(np.array((0.,0.,1.)),yaw)
    p0=tcp.copy() if hand_path else tip.copy();r0=rhand.copy() if hand_path else rc.copy()
    if cutting:
     # Follow the intended straight seam rather than interpolating from a
     # slightly displaced measured endpoint left by the lowering phase.
     previous_goal=self.phases[self.index-1][1]
     if previous_goal is not None:p0=origin+reference@np.array(previous_goal)
     r0=tool_rotation.copy()
    previous_motion=getattr(self,'completed_motion',None)
    handoff_hold=bool(lowering and matching_hover_lower(previous_motion,phase,reference.T@(goal-origin),reference.T@tool_rotation))
    if handoff_hold:
     p0=origin+reference@np.asarray(previous_motion['local_endpoint']);r0=reference@np.asarray(previous_motion['local_rotation'])
    angle=float(np.linalg.norm(ik._rot_error(tool_rotation,r0)));length=float(np.linalg.norm(goal-p0))
    speed=(.008 if self.repass_round else .012 if 'seam' in phase else .016) if cutting else .040
    if lowering:speed=.010/1.5
    progress=TrackedProgress(length,angle,speed=speed)
    self.motion={'p0':p0,'r0':r0,'goal':goal,'rotation':tool_rotation,'progress':progress,'started':time,
                 'timeout':100. if cutting else max(12.,length/speed*2.5+angle/.35+8.),'hand_path':hand_path,'precision_frames':0,'last_advance':time}
    if not phase.startswith(('return_','release_','retreat')):
     self.motion.update(local_start=reference.T@(p0-origin),local_goal=reference.T@(goal-origin),local_r0=reference.T@r0,local_r1=reference.T@tool_rotation)
    previous_bias=self.bias.copy()
    self.bias=cut_entry_bias(previous_bias,getattr(self,'completed_motion',None),phase,self.motion.get('local_start'),self.motion.get('local_r0')) if cutting else previous_bias.copy() if handoff_hold else np.zeros(3)
    self.motion['bias_carried_from_lower']=bool(cutting and matching_lower_cut(getattr(self,'completed_motion',None),phase,self.motion.get('local_start'),self.motion.get('local_r0')))
   m=self.motion
   if 'local_goal' in m:m.update(p0=origin+reference@m['local_start'],goal=origin+reference@m['local_goal'],r0=reference@m['local_r0'],rotation=reference@m['local_r1'])
   actual_point=tcp if m['hand_path'] else tip;actual_rotation=rhand if m['hand_path'] else rc
   progress=m['progress'];s=ik.smoothstep(progress.fraction);command_point=m['p0']+s*(m['goal']-m['p0']);tool_rotation=ik.slerp_matrix(m['r0'],m['rotation'],s)
   command_error=command_point-actual_point;orientation_error=float(np.linalg.norm(ik._rot_error(tool_rotation,actual_rotation)));endpoint_error=float(np.linalg.norm(m['goal']-actual_point));endpoint_rotation=float(np.linalg.norm(ik._rot_error(m['rotation'],actual_rotation)))
   direction=m['goal']-m['p0'];length=np.linalg.norm(direction);correction=command_error.copy()
   if not handoff_hold:
    if cutting or lowering:self.bias=bounded_cut_integral(self.bias,correction,dt)
    else:self.bias=np.clip(self.bias+2.*dt*correction,-.003,.003) if np.linalg.norm(command_error)<.012 else self.bias*.95
   if m['hand_path']:target,rotation=command_point+self.bias,tool_rotation
   else:target,rotation=blade_tcp_target(command_point+self.bias,tool_rotation,self.tool_in_hand,self.tip_in_hand)
   q,pe,re=ik.solve(target,ik.matrix_to_quat_xyzw(rotation),qa,base_pos=self.env.rig.bases['left'],iters=140)
   if handoff_hold:
    q=self.previous[:6].copy();target,rotation=ik.tcp_pose(q,self.env.rig.bases['left'])
   elapsed=time-m['started'];g=float(np.clip(-.011/FINGER_OPEN,0,1))
   if phase=='release_cutter':g=g+(1-g)*ik.smoothstep(min(1.,elapsed))
   if phase=='retreat':g=1.
   out=np.r_[q,g,b]
   local_command=reference.T@(command_point-origin);local_endpoint=reference.T@(m['goal']-origin)
   local_error=local_command-local_tip;xy_direction=reference.T@direction;xy_direction[2]=0.;xy_length=np.linalg.norm(xy_direction)
   lateral_error=float(np.linalg.norm(local_error[:2]))
   if xy_length>1e-10:lateral_error=float(abs(local_error[0]*xy_direction[1]-local_error[1]*xy_direction[0])/xy_length)
   film_valid=bool(FILM_Z<=local_tip[2]<=.001)
   next_cut_direction=np.array((1.,0.,0.))
   if lowering and self.index+1<len(self.phases) and self.phases[self.index+1][0]==phase[:-6]+'_cut':next_cut_direction=np.asarray(self.phases[self.index+1][1])-local_endpoint
   cut_gate=(not cutting or (film_valid and lateral_error<=.00065)) and (not lowering or lower_lateral_advance(local_tip,local_endpoint,next_cut_direction)) and not handoff_hold
   if handoff_hold:out=self.previous.copy()
   old_fraction=progress.fraction
   complete=progress.update(dt,float(np.linalg.norm(command_error)),orientation_error,endpoint_error,endpoint_rotation,
                            position_tolerance=.0015 if cutting or lowering else .004,
                            tracking_tolerance=.0025 if cutting else .006,
                            rotation_tracking_tolerance=.05 if cutting else .07,advance_allowed=cut_gate)
   if progress.fraction>old_fraction+1e-12:m['last_advance']=time
   if cutting:
    timeout=cut_watchdog(elapsed,time-m['last_advance'])
    if timeout:self.failure=phase+': '+timeout
   if cutting or lowering:
    # End-fold depth may be limited by the real board surface. Require the
    # actual valid film window and XY endpoint, not penetration to a fixed Z.
    precise=cut_endpoint_ready(progress.fraction,film_valid,local_tip,local_endpoint,endpoint_rotation)
    m['precision_frames']=m['precision_frames']+1 if precise else 0
    complete=m['precision_frames']>=5
   if phase=='release_cutter':
    rest_position,_=self.rest_tool
    complete=complete and elapsed>=1.2 and opening>.8 and np.linalg.norm(cutter-rest_position)<.01
   if elapsed>m['timeout'] and not self.failure:self.failure=phase+' timed out waiting for actual '+('hand' if m['hand_path'] else 'blade')+' alignment'
   if phase not in ('release_cutter','retreat') and (opening<.06 or opening>.5 or np.linalg.norm(cutter-tcp)>.065):self.failure='cutter grip lost during '+phase
   report=[{'side':'left','phase':phase,'position_error':endpoint_error,'tracking_error':float(np.linalg.norm(command_error)),'orientation_error_rad':endpoint_rotation,
            'path_fraction':progress.fraction,'blade_target':m['goal'].tolist() if not m['hand_path'] else None,'blade_command':command_point.tolist() if not m['hand_path'] else None,
            'blade_actual':tip.tolist(),'blade_local':local_tip.tolist(),'blade_lateral_tracking_error_m':lateral_error,'blade_in_film_window':film_valid,
            'tcp_target':target.tolist(),'tcp_actual':tcp.tolist(),'actual_joint_angles':qa.tolist(),'actual_tcp_rotation':rhand.tolist(),'actual_tool_rotation':rc.tolist(),
            'tool_in_hand_rotation':self.tool_in_hand.tolist(),'tip_in_hand':self.tip_in_hand.tolist(),
            'tip_grasp_drift_m':float(np.linalg.norm(self.tip_in_hand-self.initial_tip_in_hand)),
            'opening':opening,'ik_error':pe,'ik_rotation_error':re,'bias_m':self.bias.tolist(),'hand_retreat':m['hand_path'],
            'hover_lower_handoff_hold':handoff_hold,'lower_lateral_advance_allowed':lower_lateral_advance(local_tip,local_endpoint,next_cut_direction) if lowering else None,'bias_carried_from_lower':m['bias_carried_from_lower'],'accepted_cursor_idle_s':time-m['last_advance'],'phase_timeout_s':m['timeout'],
            'native_coverage':self.coverage_report,'physical_repass_round':self.repass_round,'cuts_verified':self.cuts_verified,'rest_pose_capture_time':self.rest_time,'failure':self.failure},
           {'side':'right','phase':'park'}]
   if complete:
    self.completed_motion=dict(phase=phase,local_endpoint=m.get('local_goal'),local_rotation=m.get('local_r1'))
    self.index+=1;self.motion=None
    if self.index<len(self.phases) and self.phases[self.index][0]=='return_hover':self.schedule_remaining_cuts()
    if self.index>=len(self.phases):self.done=True
  delta=np.array([.04]*6+[.05]+[.04]*6+[.05]);out=np.clip(out,self.previous-delta,self.previous+delta);out=np.clip(out,self.env._low,self.env._high);self.previous=out
  return out,report

class CartonOpenProbe(CartonGripProbe):
 """Grip each ordinary free flap edge and verify native angular progress.

 This diagnostic requires an explicitly unsealed starting carton. It never
 edits hinges or object state; completion of it is not full task completion.
 """
 def __init__(self,env):
  super().__init__(env)
  from scene import layout
  self.hinges={h['a']:h for h in layout()[1] if h['b'].endswith('_bottom')}
  self.order=['south','north','east','west'];self.index=0;self.stage='approach';self.started=0.;self.failure=None;self.offset=None;self.last_progress=0.;self.last_angle=-180.;self.command_angle=0.;self.entered=False
 def pose(self,name):
  from yam_arena.urdf import quat_xyzw_to_matrix
  d=self.env._base.arena_world.get_pose_w(name);d=d.torch if hasattr(d,'torch') else d;p=d.detach().cpu().numpy()[0];return p[:3],quat_xyzw_to_matrix(p[3:])
 def edge(self,name,angle=None):
  from yam_arena.urdf import axis_angle_matrix
  h=self.hinges[name];wp,wr=self.pose(name);fp,fr=self.pose(name+'_bottom')
  observer=getattr(self.env._base,'_carton_angle_observer',None)
  if observer is None or observer.tracker.failure:raise RuntimeError('trusted continuous carton angles unavailable')
  a=float(observer.tracker.angles[['north','south','east','west'].index(name)])
  if angle is not None:
   fr=wr@axis_angle_matrix(np.array([1.,0,0]),np.radians(angle));fp=wp+wr@np.array(h['anchor_a'])-fr@np.array(h['anchor_b'])
  offset=np.array((0.,-(.18/2-.003)/2+.010,-.006)) if self.offset is None else self.offset
  p=fp+fr@offset;outward=wr[:,1].copy();outward[2]=0.;outward/=np.linalg.norm(outward)
  theta=np.radians(max(35.,50.-(a if angle is None else angle)));approach=outward*np.sin(theta)-np.array([0.,0.,1.])*np.cos(theta);sign=1 if name in ['south','east'] else -1
  from feedback_geometry import flap_grip_frame
  r=flap_grip_frame(outward,approach,sign)
  if angle is not None and hasattr(self,'grasp_rotation'):r=r@self.grasp_rotation
  return p,r,a
 def command(self,time):
  out=self.env._ready.detach().cpu().numpy().reshape(-1).copy();report=[]
  if self.index>=4 or self.failure:
   return self.previous.copy(),[{'side':'left','phase':'failed' if self.failure else 'flaps_opened','failure':self.failure},{'side':'right','phase':'hold'}]
  if self.stage=='park':
   i=self.park_i;s=ik.smoothstep(min(1.,(time-self.started)/3));out=self.previous.copy();out[7*i:7*i+6]=self.park_q+s*(self.env._ready.detach().cpu().numpy().reshape(-1)[7*i:7*i+6]-self.park_q);out[7*i+6]=self.park_g+(1-self.park_g)*s;self.previous=out
   ids,fids=self.env._ids[['left_robot','right_robot'][i]];d=self.env._base.scene[['left_robot','right_robot'][i]].data.joint_pos;d=d.torch if hasattr(d,'torch') else d;native=d.detach().cpu().numpy()[0];ready=self.env._ready.detach().cpu().numpy().reshape(-1)[7*i:7*i+6];park_error=float(np.max(np.abs(native[ids]-ready)));opening=float(np.clip(native[fids].mean()/FINGER_OPEN,0,1))
   if time-self.started>=3 and park_error<.04 and opening>.8:self.index+=1;self.offset=None;self.entered=False;self.engaged=False;self.stage='approach';self.started=time
   elif time-self.started>8:self.failure='arm did not reach measured ready pose between flaps'
   return out,[{'side':['left','right'][i],'phase':'park_between_flaps'},{'side':['right','left'][i],'phase':'hold'}]
  name=self.order[self.index];i=1 if name in ['south','east'] else 0;side=['left','right'][i];p,r,angle=self.edge(name);elapsed=time-self.started;opened=.018/(2*abs(FINGER_OPEN));closed=.002/(2*abs(FINGER_OPEN))
  ids,fids=self.env._ids[side+'_robot'];d=self.env._base.scene[side+'_robot'].data.joint_pos;d=d.torch if hasattr(d,'torch') else d;actualq=d.detach().cpu().numpy()[0];actual,actualR=ik.tcp_pose(actualq[ids],self.env.rig.bases[side]);opening=float(np.clip(actualq[fids].mean()/FINGER_OPEN,0,1));gap=opening*2*abs(FINGER_OPEN);stage_before=self.stage
  if not self.entered:self.startp=actual.copy();self.startr=actualR.copy();self.entered=True
  if self.stage=='approach':
   s=ik.smoothstep(min(1.,elapsed/3));target=self.startp+s*(p-r[:,2]*.075-self.startp);rotation=ik.slerp_matrix(self.startr,r,s);g=opened
   if elapsed>=3 and np.linalg.norm(target-actual)<.004 and np.linalg.norm(ik._rot_error(rotation,actualR))<.07 and opening>=opened*.85:self.stage='enter';self.started=time;self.startp=actual.copy()
  elif self.stage=='enter':
   s=ik.smoothstep(min(1.,elapsed/2));target=self.startp+s*(p-self.startp);rotation=r;g=opened
   if elapsed>=2 and np.linalg.norm(p-actual)<.003 and np.linalg.norm(ik._rot_error(rotation,actualR))<.07:self.stage='pinch';self.started=time
  elif self.stage=='pinch':
   target=p;rotation=r;g=opened+(closed-opened)*ik.smoothstep(min(1.,elapsed))
   if elapsed>=1.4 and np.linalg.norm(p-actual)<.005 and .0023<gap<.015:
    fp,fr=self.pose(name+'_bottom');self.offset=fr.T@(actual-fp);self.grasp_rotation=r.T@actualR;self.stage='rotate';self.started=time;self.command_angle=angle;self.last_angle=angle;self.last_progress=time;self.engagement_angle=angle;self.engaged=False;self.slip_since=None
  elif self.stage=='rotate':
   if angle>self.last_angle+2:self.last_progress=time;self.last_angle=angle
   if angle>=self.engagement_angle+5:self.engaged=True
   if time-self.last_progress>4:self.failure=name+' flap did not follow the gripper'
   if elapsed>3 and not self.engaged:self.failure=name+' edge pinch did not produce measured flap rotation'
   slip=float(np.linalg.norm(p-actual))
   self.slip_since=time if (slip>.012 or gap<.0023) and self.slip_since is None else self.slip_since if slip>.012 or gap<.0023 else None
   if self.slip_since is not None and time-self.slip_since>.5:self.failure=name+' edge grip lost by native gap or flap/TCP readback'
   self.command_angle=min(195.,angle+12.,self.command_angle+.8);target,rotation,_=self.edge(name,self.command_angle);g=closed
   if angle>=185:self.stage='release';self.started=time;self.startp=actual.copy();self.startr=actualR.copy()
  elif self.stage=='release':
   target=self.startp;rotation=self.startr;g=closed+(opened-closed)*ik.smoothstep(min(1.,elapsed))
   if elapsed>=1.3 and gap>=.015:self.stage='retreat';self.started=time
   elif elapsed>5:self.failure=name+' fingers did not release the edge'
  elif self.stage=='settle_backing':
   target=self.startp;rotation=self.startr;g=opened
   speed=abs(angle-self.settle_angle)/.05;self.settle_angle=angle
   self.settle_count=self.settle_count+1 if 260.<=angle<=274.5 and speed<5. else 0
   if self.settle_count>=5:self.stage='park';self.started=time;self.park_i=i;self.park_q=actualq[ids].copy();self.park_g=opened
   elif elapsed>10:self.failure=name+' flap did not settle on the physical wall-backed stack'
  else:
   target=self.startp-self.startr[:,2]*.075*ik.smoothstep(min(1.,elapsed/2));rotation=self.startr;g=opened
   if elapsed>=2 and np.linalg.norm(target-actual)<.005:
    if angle<150:self.failure=name+' flap did not remain open after release'
    else:self.stage='settle_backing';self.started=time;self.startp=actual.copy();self.startr=actualR.copy();self.settle_angle=angle;self.settle_count=0
  if stage_before=='retreat' and self.stage==stage_before and elapsed>8:self.failure=name+' hand did not clear the released flap'
  if stage_before in ('approach','enter','pinch') and self.stage==stage_before and elapsed>10:self.failure=name+':'+stage_before+' did not reach measured pose/grip gate'
  error=target-actual;self.bias[i]=np.clip(self.bias[i]+.06*error,-.015,.015);q,pe,re=ik.solve(target+self.bias[i],ik.matrix_to_quat_xyzw(rotation),actualq[ids],base_pos=self.env.rig.bases[side],iters=90);out[7*i:7*i+6]=q;out[7*i+6]=g
  # The idle arm holds its previous ordinary target rather than crossing the carton.
  j=1-i;out[7*j:7*j+7]=self.previous[7*j:7*j+7]
  delta=np.array([.04]*6+[.05]+[.04]*6+[.05]);out=np.clip(out,self.previous-delta,self.previous+delta);out=np.clip(out,self.env._low,self.env._high);self.previous=out
  return out,[{'side':side,'phase':name+':'+self.stage,'angle_deg':angle,'position_error':float(np.linalg.norm(error)),'rotation_error_rad':float(np.linalg.norm(ik._rot_error(rotation,actualR))),'opening':opening,'finger_gap_m':gap,'edge_target':p.tolist(),'tcp_actual':actual.tolist(),'tcp_target':target.tolist(),'target_rotation':rotation.tolist(),'engagement_verified':getattr(self,'engaged',False),'ik_error':pe,'ik_rotation_error':re,'failure':self.failure},{'side':['left','right'][j],'phase':'hold'}]
