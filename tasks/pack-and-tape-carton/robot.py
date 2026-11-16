"""Ordinary robot pickup/place prefix, no product or carton pose writes."""
import numpy as np
from yam_arena.demo import ArmPlan,Waypoint as W
from yam_arena.constants import FINGER_OPEN
from config import *
class ClosureProbe:
 """Preloaded contents; feedback grasps close the four ordinary rims.

 All manipulation uses ordinary arm/finger commands. Angles, finger aperture
 and acquired contact offsets gate progress; no phase contributes task credit.
 """
 def __init__(self,env):
  from carton_scene import layout
  self.env=env;self.previous=env._ready.detach().cpu().numpy().reshape(-1).copy();self.order=['east','west','north','south'];self.index=0;self.stage='preloaded';self.started=0.;self.failure=None;self.done=False;self.entered=False;self.bias=np.zeros(3);self.contact_offset=None;self.rotation_offset=None;self.last_time=0.;self.closed_hold=0.
  parts,hinges=layout();self.parts={p['name']:p for p in parts};self.hinges={h['a']:h for h in hinges if h['b'].endswith('_top')}
 def pose(self,name):
  from yam_arena.urdf import quat_xyzw_to_matrix
  d=self.env._base.arena_world.get_pose_w(name);d=d.torch if hasattr(d,'torch') else d;p=d.detach().cpu().numpy()[0]
  return p[:3],quat_xyzw_to_matrix(p[3:])
 def angle(self,name):
  _,wr=self.pose(name);_,fr=self.pose(name+'_top');r=wr.T@fr
  return float(np.degrees(np.arctan2(r[2,1],r[1,1])))
 def target(self,name,angle=None):
  from closure_geometry import GRASPS,flap_target
  _,edge,tangent,tilt=GRASPS[name];wp,wr=self.pose(name)
  return flap_target(wp,wr,self.hinges[name],self.parts[name+'_top']['size'],
                     self.angle(name) if angle is None else angle,edge,tangent,tilt,
                     self.contact_offset,self.rotation_offset)
 def change(self,stage,time,tcp,rotation):
  self.stage=stage;self.started=time;self.startp=tcp.copy();self.startr=rotation.copy();self.bias[:]=0.;self.settled=0
 def command(self,time):
  from yam_arena import ik
  from yam_arena.demo import grip_rotation
  from closure_geometry import GRASPS,PAD_CONTACT_FORWARD
  dt=min(.05,max(.001,time-self.last_time));self.last_time=time
  if self.failure or self.done:return self.previous.copy(),{'phase':'closure_failed' if self.failure else 'four_flaps_closed','failure':self.failure,'flaps_completed':self.index}
  angles={n:self.angle(n) for n in self.order}
  if self.index>=4:
   _,reference=self.pose('north');heights={}
   for n in self.order:
    fp,fr=self.pose(n+'_top');size=np.asarray(self.parts[n+'_top']['size']);vertices=np.array([fp+fr@(size*np.array((x,y,z))/2) for x in (-1,1) for y in (-1,1) for z in (-1,1)]);height=vertices@reference[:,2];heights[n]=(float(height.min()),float(height.max()))
   inner_under=max(heights[n][1] for n in ('east','west'))<=min(heights[n][0] for n in ('north','south'))+.0005
   self.closed_hold=self.closed_hold+dt if all(abs(a)<=5. for a in angles.values()) and inner_under else 0.
   if self.closed_hold>=1.:self.done=True
   elif time-self.started>5:self.failure='closed flap angles did not persist after both hands cleared'
   return self.previous.copy(),{'phase':'verify_released_closure','flap_angles_deg':angles,'closed_hold_s':self.closed_hold,'inner_flaps_under_outer':bool(inner_under),'failure':self.failure}
  name=self.order[self.index];side=GRASPS[name][0];i=0 if side=='left' else 1
  ids,fids=self.env._ids[side+'_robot'];d=self.env._base.scene[side+'_robot'].data.joint_pos;d=d.torch if hasattr(d,'torch') else d;native=d.detach().cpu().numpy()[0];qa=native[ids];tcp,rhand=ik.tcp_pose(qa,self.env.rig.bases[side]);opening=float(np.clip(native[fids].mean()/FINGER_OPEN,0,1));gap=opening*2*abs(FINGER_OPEN);angle=angles[name]
  if self.stage=='preloaded':self.change('approach',time,tcp,rhand)
  p,r,contact=self.target(name);elapsed=time-self.started;opened=.020/(2*abs(FINGER_OPEN));closed=.0015/(2*abs(FINGER_OPEN));stage_before=self.stage;g=opened
  if self.stage=='approach':
   blend=ik.smoothstep(elapsed/3);target=self.startp+blend*(p-r[:,2]*.040-self.startp);rotation=ik.slerp_matrix(self.startr,r,blend)
   if elapsed>=3 and np.linalg.norm(target-tcp)<.004 and np.linalg.norm(ik._rot_error(rotation,rhand))<.07 and gap>.016:self.change('enter',time,tcp,rhand)
  elif self.stage=='enter':
   blend=ik.smoothstep(elapsed/2);target=self.startp+blend*(p-self.startp);rotation=r
   if elapsed>=2 and np.linalg.norm(p-tcp)<.003 and np.linalg.norm(ik._rot_error(r,rhand))<.07:self.change('pinch',time,tcp,rhand)
  elif self.stage=='pinch':
   target=p;rotation=r;g=opened+(closed-opened)*ik.smoothstep(elapsed)
   held=np.linalg.norm(p-tcp)<.005 and .0023<gap<.012 and np.linalg.norm(ik._rot_error(r,rhand))<.10
   self.settled=self.settled+1 if held and elapsed>=1.3 else 0
   if self.settled>=3:
    fp,fr=self.pose(name+'_top');self.contact_offset=fr.T@(tcp+rhand[:,2]*PAD_CONTACT_FORWARD-fp);self.rotation_offset=r.T@rhand;self.command_angle=angle;self.engagement_angle=angle;self.last_angle=angle;self.last_progress=time;self.slip_since=None;self.engaged=False;self.change('fold',time,tcp,rhand)
  elif self.stage=='fold':
   if angle>self.last_angle+1.:self.last_angle=angle;self.last_progress=time
   if angle>self.engagement_angle+5.:self.engaged=True
   acquired_error=float(np.linalg.norm(tcp+rhand[:,2]*PAD_CONTACT_FORWARD-contact))
   bad_grip=acquired_error>.010 or gap<.0023 or gap>.014
   self.slip_since=time if bad_grip and self.slip_since is None else self.slip_since if bad_grip else None
   if self.slip_since is not None and time-self.slip_since>.5:self.failure=name+' plain-rim grasp slipped by native gap/contact-point readback'
   if elapsed>3 and not self.engaged:self.failure=name+' pinch did not produce measured hinge motion'
   if time-self.last_progress>4:self.failure=name+' flap stopped following the gripper'
   self.command_angle=min(1.,angle+8.,self.command_angle+20.*dt);target,rotation,_=self.target(name,self.command_angle);g=closed
   if angle>=-.3 and abs(angle)<=5.:self.change('release',time,tcp,rhand)
  elif self.stage=='release':
   target=self.startp;rotation=self.startr;g=closed+(opened-closed)*ik.smoothstep(elapsed)
   if elapsed>=1.2 and gap>.016:self.change('clear_edge',time,tcp,rhand)
  elif self.stage=='clear_edge':
   target=self.startp-self.startr[:,2]*.045*ik.smoothstep(elapsed/1.5);rotation=self.startr
   if elapsed>=1.5 and np.linalg.norm(target-tcp)<.004:self.change('lift_clear',time,tcp,rhand)
  elif self.stage=='lift_clear':
   target=self.startp+np.array((0.,0.,.04))*ik.smoothstep(elapsed/1.5);rotation=self.startr
   if elapsed>=1.5 and np.linalg.norm(target-tcp)<.004:self.change('park',time,tcp,rhand)
  else:
   park=np.array((.13,.24 if i==0 else -.24,TOP+.17));blend=ik.smoothstep(elapsed/3);target=self.startp+blend*(park-self.startp);rotation=ik.slerp_matrix(self.startr,grip_rotation(),blend);g=opened+(1-opened)*blend
   if elapsed>=3 and np.linalg.norm(target-tcp)<.005 and np.linalg.norm(ik._rot_error(rotation,rhand))<.08 and opening>.8:
    if any(abs(angles[n])>5 for n in self.order[:self.index+1]):self.failure='a released flap reopened before the next fold'
    else:
     self.index+=1;self.contact_offset=None;self.rotation_offset=None;self.change('approach',time,tcp,rhand)
     if self.index<4:self.stage='preloaded'
  if elapsed>(18. if stage_before=='fold' else 12.):self.failure=name+':'+stage_before+' timed out at measured pose/grip gate'
  error=target-tcp
  self.bias=np.clip(self.bias+.06*error,-.012,.012) if np.linalg.norm(error)<.025 else self.bias*.95
  q,pe,re=ik.solve(target+self.bias,ik.matrix_to_quat_xyzw(rotation),qa,base_pos=self.env.rig.bases[side],iters=120)
  if (pe>.002 or re>.03) and stage_before in ('approach','enter','park'):
   # Free-hand recovery from a local IK branch. The ordinary joint command is
   # still rate limited, and actual pose gates remain mandatory before contact.
   seed=self.env._ready.detach().cpu().numpy().reshape(-1)[7*i:7*i+6]
   alternate,ape,are=ik.solve(target,ik.matrix_to_quat_xyzw(rotation),seed,base_pos=self.env.rig.bases[side],iters=200)
   if ape+.05*are<pe+.05*re:q,pe,re=alternate,ape,are;self.bias[:]=0.
  out=self.previous.copy();out[7*i:7*i+6]=q;out[7*i+6]=g
  delta=np.array([.04]*6+[.05]+[.04]*6+[.05]);out=np.clip(out,self.previous-delta,self.previous+delta);out=np.clip(out,self.env._low,self.env._high);self.previous=out
  return out,{'phase':name+':'+stage_before,'flap_angles_deg':angles,'active_arm':side,'finger_gap_m':gap,'position_error_m':float(np.linalg.norm(error)),'rotation_error_rad':float(np.linalg.norm(ik._rot_error(rotation,rhand))),'tcp_actual':tcp.tolist(),'tcp_target':target.tolist(),'grip_contact_target':contact.tolist(),'command_angle_deg':getattr(self,'command_angle',angle),'engagement_verified':getattr(self,'engaged',False),'ik_error_m':pe,'ik_error_rad':re,'flaps_completed':self.index,'failure':self.failure}
