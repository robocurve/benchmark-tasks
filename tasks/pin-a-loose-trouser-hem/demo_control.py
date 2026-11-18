"""Pinch a fold and guide the straight needle through both sides using robot joints."""
import numpy as np
from scipy.optimize import least_squares
class DemoControl:
 def __init__(self):self.start=None;self.closure=None;self.last_q=None;self.released_at=None;self.left_offset=None;self.right_offset=None;self.feedback_goal=None;self.threaded_at=None;self.exit_names=None;self.exit_axis=None;self.exit_depth=None;self.exit_anchor=None;self.seating_anchor=None
 def action(self,k,wanted,m,hands,current):
  from yam_arena import ik,station
  from geometry import BODIES
  if self.left_offset is None:
   names=['hem_8_0','hem_9_0'];nominal=np.mean([b['pos'] for b in BODIES if b['name'] in names],axis=0)
   self.left_offset=np.mean([m.p(n) for n in names],axis=0)-nominal
  if k<=1000:
   pp,RR=ik.tcp_pose(wanted[:6],station.BASES['left']);target=pp+self.left_offset*min(1.,k/60)
   if 60<k<=120:
    names=['front_8_0','front_9_0','hem_8_0','hem_9_0']
    centre=np.mean([m.p(n) for n in names],axis=0)
    grasp=centre-.006*RR[:,2];u=np.clip((k-60)/40,0,1)
    target=(1-u)*target+u*grasp
    if k==120:self.left_offset=grasp-pp
   seed=np.clip(current[:6],ik.LOWER+1e-6,ik.UPPER-1e-6)
   def correct(q):
    p,R=ik.tcp_pose(q,station.BASES['left']);return np.r_[p-target,.12*ik._rot_error(RR,R),.001*(q-seed)]
   wanted=wanted.copy();wanted[:6]=least_squares(correct,seed,bounds=(np.maximum(ik.LOWER,seed-.12),np.minimum(ik.UPPER,seed+.12)),max_nfev=30).x
  if k<=220:
   from geometry import PIN
   pp,RR=ik.tcp_pose(wanted[7:13],station.BASES['right'])
   if self.right_offset is None:self.right_offset=m.p('pin_frame')-PIN
   target=pp+self.right_offset*min(1.,k/60)
   if 60<k<=120:
    grasp=m.p('pin_frame')-.004*RR[:,2];u=np.clip((k-60)/40,0,1);target=(1-u)*target+u*grasp
    if k==120:self.right_offset=grasp-pp
   seed=np.clip(current[7:13],ik.LOWER+1e-6,ik.UPPER-1e-6)
   predicted,rotation=ik.tcp_pose(seed,station.BASES['right'])
   bias=hands['right']['tcp']-predicted;rotation_bias=hands['right']['rotation']@rotation.T
   def grip_error(q):
    pos,rot=ik.tcp_pose(q,station.BASES['right']);return np.r_[pos+bias-target,.12*ik._rot_error(RR,rotation_bias@rot),.001*(q-seed)]
   wanted=wanted.copy();wanted[7:13]=least_squares(grip_error,seed,bounds=(np.maximum(ik.LOWER,seed-.12),np.minimum(ik.UPPER,seed+.12)),max_nfev=30).x
   if k in [100,120,220]:m.events.append(dict(event='demo_coil_grip_check',step=k,coil_local_mm=(1000*hands['right']['rotation'].T@(m.p('pin_frame')-hands['right']['tcp'])).tolist(),fk_bias_mm=(1000*bias).tolist()))
   return wanted
  if self.threaded_at is None and m.measure()["both_layers_held"]:
   axis=m.orientation('pin_arm')[:,0];tip=m.p('needle_point',[.001,0,0])
   depth=min(float((tip-m.p(name,local))@axis) for name,local in m.holes.items())
   if depth>=.006:self.threaded_at=k
  if self.threaded_at is not None:return self.close(700+k-self.threaded_at,wanted,m,hands,current)
  if k>1000:return self.close(k-300,wanted,m,hands,current)
  h=hands['right'];tip=m.p('needle_point',[.001,0,0]);axis=m.orientation('pin_arm')[:,0]
  if np.linalg.norm(m.p('pin_frame')-h['tcp'])>.055:raise RuntimeError('The pin slipped from the native grip; stop this failed development trial.')
  if self.start is None:
   from path_search import select_path
   self.start=tip.copy();self.initial_axis=axis.copy()
   self.entry,self.end,self.direction,names=select_path(m)
   self.fabric_targets=[]
   for name in names:
    normal=m.orientation(name)[:,2]
    distance=float(normal@(m.p(name)-self.entry)/(normal@self.direction))
    point=self.entry+distance*self.direction
    self.fabric_targets.append((name,m.orientation(name).T@(point-m.p(name))))
   self.splay_direction=np.array([0.,0.,-1.]);self.splay_direction-=self.direction*(self.splay_direction@self.direction);self.splay_direction/=np.linalg.norm(self.splay_direction)
   # Cant the open pin 60 degrees around its needle axis to keep the wrist
   # inside its ordinary joint limits while the frame remains below the point.
   from scipy.spatial.transform import Rotation
   self.splay_direction=Rotation.from_rotvec(np.radians(60)*self.direction).apply(self.splay_direction)
   m.events.append(dict(event='demo_selected_straight_pin_path',start=self.entry.tolist(),end=self.end.tolist(),patches=names,time=m.time))
   # Select a reachable wrist branch before descending to the cloth. A local
   # incremental solve can otherwise stop at the wrist's negative limit.
   from yam_arena.constants import READY
   self.approach_start=np.asarray(current[7:13]).copy()
   relative_tip=h['rotation'].T@(tip-h['tcp']);relative_axis=h['rotation'].T@axis
   cap=m.p('pin_frame',[.0685,0,.002]);splay=cap-tip;splay-=axis*(splay@axis);splay/=np.linalg.norm(splay);relative_splay=h['rotation'].T@splay
   target=self.entry+[0,0,.05]
   def approach_error(q):
    pp,RR=ik.tcp_pose(q,station.BASES['right']);return np.r_[pp+RR@relative_tip-target,.1*(RR@relative_axis-self.direction),.05*(RR@relative_splay-self.splay_direction)]
   solutions=[least_squares(approach_error,seed,bounds=(ik.LOWER,ik.UPPER),max_nfev=180) for seed in [np.clip(current[7:13],ik.LOWER+1e-6,ik.UPPER-1e-6),np.asarray(READY),(ik.LOWER+ik.UPPER)/2]]
   solution=min(solutions,key=lambda x:np.linalg.norm(x.fun))
   if np.linalg.norm(solution.fun)>.005:raise RuntimeError('No reachable needle approach for the observed grasp.')
   self.approach_q=solution.x
   m.events.append(dict(event='demo_reachable_needle_approach',residual=float(np.linalg.norm(solution.fun)),time=m.time))
  if k<=340:
   u=np.clip((k-220)/120,0,1);u=u*u*(3-2*u);out=wanted.copy();out[7:13]=(1-u)*self.approach_start+u*self.approach_q;self.last_q=out[7:13].copy();return out
  def blend(a,b,start,end):
   u=np.clip((k-start)/(end-start),0,1);u=u*u*(3-2*u);return a*(1-u)+b*u
  if k<=340:goal=blend(self.start,self.entry+[0,0,.05],220,340);goalaxis=blend(self.initial_axis,self.direction,220,340)
  elif k<=400:goal=blend(self.entry+[0,0,.05],self.entry,340,400);goalaxis=self.direction.copy()
  elif k<=640:goal=blend(self.entry,self.end,400,640);goalaxis=self.direction.copy()
  else:goal=self.end;goalaxis=self.direction.copy()
  if k>400:
   remaining=[(name,local) for name,local in self.fabric_targets if name not in m.holes]
   if remaining:
    name,local=remaining[0]
    # Follow the observed material as it deflects; apply force through the
    # needle, with at most 20 mm/s advancement, instead of chasing a stale
    # world-space line through a now-displaced fold.
    target=m.p(name,local)+.004*self.direction
    if self.feedback_goal is None:self.feedback_goal=self.entry.copy()
    delta=target-self.feedback_goal
    self.feedback_goal+=delta*min(1.,.001/max(np.linalg.norm(delta),1e-9))
    goal=self.feedback_goal.copy()
   else:goal=self.end
   goalaxis=self.direction.copy()
   if m.measure()['both_layers_held']:
    # Seat both layers a few millimetres along the shaft, then close around
    # their free edge. Scoring allows any intact, retained pinning route.
    if self.seating_anchor is None:
     self.seating_anchor=np.mean([m.p(name,local) for name,local in m.holes.items()],axis=0)
    goal=self.seating_anchor+.010*self.direction
    goalaxis=self.direction.copy()
  goalaxis/=np.linalg.norm(goalaxis)
  from scipy.spatial.transform import Rotation
  down=np.array([0.,0.,-1.]);down-=goalaxis*(down@goalaxis);down/=max(np.linalg.norm(down),1e-9)
  splay_goal=Rotation.from_rotvec(np.radians(60)*goalaxis).apply(down)
  relative_tip=h['rotation'].T@(tip-h['tcp']);relative_axis=h['rotation'].T@axis
  cap=m.p('pin_frame',[.0685,0,.002]);splay=cap-tip;splay-=axis*(splay@axis);splay/=np.linalg.norm(splay);relative_splay=h['rotation'].T@splay
  seed=np.clip(current[7:13],ik.LOWER+1e-6,ik.UPPER-1e-6)
  def residual(q):
   pp,R=ik.tcp_pose(q,station.BASES['right']);return np.r_[pp+R@relative_tip-goal,.1*(R@relative_axis-goalaxis),.05*(R@relative_splay-splay_goal),.001*(q-seed)]
  sol=least_squares(residual,seed,bounds=(np.maximum(ik.LOWER,seed-.12),np.minimum(ik.UPPER,seed+.12)),max_nfev=30)
  out=wanted.copy();out[7:13]=sol.x;self.last_q=sol.x.copy();return out

 def close(self,k,wanted,m,hands,current):
  from yam_arena import ik,station
  if self.closure is None:
   if not m.measure()['both_layers_held']:raise RuntimeError('Threading incomplete before closure.')
   self.closure=dict(left=hands['left']['tcp'].copy(),leftR=hands['left']['rotation'].copy(),right=hands['right']['tcp'].copy(),rightR=hands['right']['rotation'].copy())
   m.events.append(dict(event='demo_begin_pin_closure',time=m.time))
  c=self.closure
  if k<=860 or 'grasp' not in c:
   # Pinch both arms near the shield, with the closing axis across their gap.
   # The ordinary fingertips provide the force; the catch still requires
   # the measured point to enter its shield at the correct angle.
   a=m.p('pin_arm',[.061,0,0]);b=m.p('pin_frame',[.061,0,0]);x=a-b;x/=np.linalg.norm(x)
   z=np.cross(m.orientation('pin_arm')[:,0],x);z/=np.linalg.norm(z)
   if z[2]>0:z=-z
   R=np.column_stack([x,np.cross(z,x),z]);c['R']=R;c['grasp']=(a+b)/2-.006*z
  R=c['R'];z=R[:,2];near=c['grasp']-.035*z
  def mix(a,b,start,end):
   u=np.clip((k-start)/(end-start),0,1);u=u*u*(3-2*u);return a*(1-u)+b*u
  def solve(side,p,targetR):
   o=0 if side=='left' else 7;seed=np.clip(current[o:o+6],ik.LOWER+1e-6,ik.UPPER-1e-6)
   def residual(q):
    pp,rr=ik.tcp_pose(q,station.BASES[side]);return np.r_[pp-p,.12*ik._rot_error(targetR,rr),.001*(q-seed)]
   return least_squares(residual,seed,bounds=(np.maximum(ik.LOWER,seed-.12),np.minimum(ik.UPPER,seed+.12)),max_nfev=40).x
  out=wanted.copy();out[7:13]=self.last_q;out[13]=0.
  if k<=740:p=c['left'];targetR=c['leftR'];grip=.7
  elif k<=800:p=mix(c['left'],c['left']+[0,0,.08],740,800);targetR=c['leftR'];grip=.7
  elif k<=860:
   p=mix(c['left']+[0,0,.08],near,800,860);targetR=ik.slerp_matrix(c['leftR'],R,np.clip((k-800)/60,0,1));grip=.5
  elif k<=920:p=mix(near,c['grasp'],860,920);targetR=R;grip=.5
  else:p=c['grasp'];targetR=R;grip=.5*(1-np.clip((k-920)/80,0,1))
  if m.latched and self.released_at is None:self.released_at=k
  if self.released_at is not None:
   u=np.clip((k-self.released_at)/80,0,1);p=c['grasp']-u*.10*z;targetR=R;grip=.7
   out[7:13]=solve('right',c['right']+[0,-.07*u,.10*u],c['rightR']);out[13]=.7
  elif k>1100:raise RuntimeError('The native pinch did not close the safety-pin catch.')
  out[:6]=solve('left',p,targetR);out[6]=grip;return out
