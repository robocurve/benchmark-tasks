"""Reference uses actual body feedback and only stock robot joint commands."""
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from inspect_robots import Action,ActionChunk
from yam_arena import ik
from yam_arena.constants import READY,FINGER_OPEN
from . import layout as L
from .fixed_scene import ARMS,BASES,CONTROL_HZ,TOP
OPEN=float(FINGER_OPEN)
def frame(approach,closing):
 z=np.array(approach,float);z/=np.linalg.norm(z);x=np.array(closing,float);x-=z*(x@z);x/=np.linalg.norm(x);return np.column_stack((x,np.cross(z,x),z))
class DryerReference:
 def __init__(self):
  self.joints={a:np.array(READY,float) for a in ARMS};self.aperture=dict.fromkeys(ARMS,OPEN);self.points={};self.rotations={};self.state=None;self.tick=0;self.phases=[];self.worst_ik=0.
 def observe(self,joint_pos,state):self.state=state;self.measured=np.array(joint_pos)
 def phase(self,name):
  self.phases.append([self.tick,name]);print('DRYER_PHASE',self.tick,name,flush=True)
  if self.state is not None:
   print('DRYER_STATE',self.state['grade'],flush=True)
   print('DRYER_BODY',self.state['poses'][:2].tolist(),flush=True)
   print('DRYER_CONTACTS',np.array(self.state['contacts']['robot_contact_counts']).sum(0).tolist(),flush=True)
   print('DRYER_PAD_FORCES',[self.state['robot_forces'][i,:2,:].tolist() for i in (7,8,16,17)],flush=True)
   if hasattr(self,'measured'):
    print('DRYER_TCP',[(a,ik.tcp_pose(self.measured[n:n+6],BASES[a])[0].tolist(),self.points.get(a,np.zeros(3)).tolist()) for a,n in [('left',0),('right',7)]],flush=True)
 def emit(self):
  self.tick+=1;return ActionChunk(actions=[Action(data=np.concatenate([np.r_[self.joints[a],np.clip(self.aperture[a]/OPEN,0,1)] for a in ARMS]),meta={})],control_hz=CONTROL_HZ)
 def hold(self,t):
  for _ in range(max(1,round(t*CONTROL_HZ))):yield self.emit()
 def solve(self,a,p,R):
  side=1 if a=='left' else -1;best=None
  for seed in [self.joints[a],np.array((0.,1.6,1.4,-.8,0,0)),np.array((side*.4,2.2,1.,-.5,side,0)),np.array((0.,2.1,1.,-.4,side*1.5,-.1))]:
   q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(R),seed,BASES[a],iters=150,damping=.01,tol=1e-6)
   if best is None or pe+.01*re<best[1]+.01*best[2]:best=q,pe,re
   if pe<.000005 and re<.0001:break
  q,pe,re=best
  if pe>.003 or re>.06:raise RuntimeError(f'{a} unreachable {p}: {pe}/{re}')
  self.worst_ik=max(self.worst_ik,pe);return q
 def go(self,a,p,R,t=1.5):
  before=self.joints[a].copy();target=self.solve(a,p,R);count=max(1,round(t*CONTROL_HZ))
  for i in range(1,count+1):
   f=i/count;f=f*f*(3-2*f);self.joints[a]=before+f*(target-before);yield self.emit()
  self.points[a]=np.array(p);self.rotations[a]=R.copy()
 def move(self,targets,t=1.,gaps=None):
  starts={a:self.points[a].copy() for a in targets};old=self.aperture.copy();count=max(1,round(t*CONTROL_HZ))
  for i in range(1,count+1):
   f=i/count;f=f*f*(3-2*f)
   for a,d in targets.items():self.points[a]=starts[a]+f*(np.array(d)-starts[a]);self.joints[a]=self.solve(a,self.points[a],self.rotations[a])
   for a,d in (gaps or {}).items():self.aperture[a]=old[a]+f*(d-old[a])
   yield self.emit()
 def choose(self,a,p,choices):
  errors=[]
  for R in choices:
   try:self.solve(a,p,R);return R
   except RuntimeError as e:errors.append(str(e))
  raise RuntimeError(str(errors))
 def calibrate_at(self,a,j,local,t=.6):
  for _ in range(round(t*CONTROL_HZ)):
   p=self.state['poses'][j];desired=p[:3]+L.quat_matrix(p[3:])@local
   n=0 if a=='left' else 7;actual,_=ik.tcp_pose(self.measured[n:n+6],BASES[a])
   self.points[a]+=np.clip(.4*(desired-actual),-.0006,.0006);self.joints[a]=self.solve(a,self.points[a],self.rotations[a]);yield self.emit()
 def close(self,a,target,t=2.):
  # Close at a fixed Cartesian command; the object is never chased upward by
  # a body-following feedback loop during jaw contact.
  before=self.aperture[a];n=round(t*CONTROL_HZ)
  for k in range(1,n+1):
   f=k/n;f=f*f*(3-2*f);self.aperture[a]=before+f*(target-before);yield self.emit()
 def roll_folded(self,t=2.):
  h=self.state['poses'][L.HEAD].copy();b=self.state['poses'][L.HANDLE].copy();H=L.quat_matrix(h[3:]);B=L.quat_matrix(b[3:]);rel=H.T@B
  held={};bias={}
  for arm,index,n in [('right',L.HEAD,7),('left',L.HANDLE,0)]:
   p=self.state['poses'][index];C=L.quat_matrix(p[3:]);actual,_=ik.tcp_pose(self.measured[n:n+6],BASES[arm]);held[arm]=(C.T@(actual-p[:3]),C.T@self.rotations[arm]);bias[arm]=self.points[arm]-actual
  count=round(t*CONTROL_HZ)
  for k in range(1,count+1):
   f=k/count;f=f*f*(3-2*f);C=H@Rotation.from_euler('x',-12*f,degrees=True).as_matrix();D=C@rel
   for arm,P,R in [('right',h[:3],C),('left',h[:3]+C@L.HINGE,D)]:
    local,Q=held[arm];self.points[arm]=P+R@local+bias[arm];self.rotations[arm]=R@Q;self.joints[arm]=self.solve(arm,self.points[arm],self.rotations[arm])
   yield self.emit()
 def push_fold(self,t=14.):
  arm='left';brace='right';local=np.array((.02037,.006,-.049))
  actual,_=ik.tcp_pose(self.measured[:6],BASES[arm]);bias=self.points[arm]-actual
  # The barrel already rests on the counter; hold its existing robot target.
  # No competing fast Cartesian integral is needed on the supporting hand.
  brace_command=self.points[brace].copy()
  for k in range(1,round(t*CONTROL_HZ)+1):
   self.points[brace]=brace_command.copy()
   p=self.state['poses'][L.HANDLE];B=L.quat_matrix(p[3:]);angle=float(self.state['grade']['angle_deg'])
   # Follow the actual face and press normally, so the finger stays at the
   # same real longitudinal patch even if the freely moving handle runs ahead.
   press=min(.004,.0003*k);target=local-np.array((press,0,0));desired=p[:3]+B@target
   actual,_=ik.tcp_pose(self.measured[:6],BASES[arm]);bias+=np.clip(.025*(desired-actual),-.0001,.0001);bias=np.clip(bias,-.015,.015)
   self.points[arm]=desired+bias;self.rotations[arm]=frame(-B[:,1],-B[:,0]);self.joints[arm]=self.solve(arm,self.points[arm],self.rotations[arm])
   if angle>=88.:
    self.phase('actual handle reached its folded stop under exterior pad contact');break
   if k%(2*CONTROL_HZ)==0:self.phase('push the real outside handle face toward the barrel')
   yield self.emit()
 def turn_barrel(self,roll_deg,t=6.,nose_up=0.):
  arm='right';h=self.state['poses'][L.HEAD].copy();H=L.quat_matrix(h[3:]);actual,_=ik.tcp_pose(self.measured[7:13],BASES[arm])
  local=H.T@(actual-h[:3]);relative=H.T@self.rotations[arm];bias=self.points[arm]-actual
  count=round(t*CONTROL_HZ)
  for k in range(1,count+1):
   f=k/count;f=f*f*(3-2*f)
   C=Rotation.from_euler('y',-nose_up*f,degrees=True).as_matrix()@H@Rotation.from_euler('x',roll_deg*f,degrees=True).as_matrix()
   self.points[arm]=h[:3]+C@local+bias;self.rotations[arm]=C@relative;self.joints[arm]=self.solve(arm,self.points[arm],self.rotations[arm])
   if k%(2*CONTROL_HZ)==0:self.phase('rotate the held barrel so gravity folds the free handle')
   yield self.emit()
 def return_for_counter_support(self,t=4.,lift=.030,yaw=30.):
  arm='right';h=self.state['poses'][L.HEAD].copy();H=L.quat_matrix(h[3:]);actual,_=ik.tcp_pose(self.measured[7:13],BASES[arm])
  local=H.T@(actual-h[:3]);relative=H.T@self.rotations[arm];bias=self.points[arm]-actual
  roll=np.degrees(np.arctan2(H[2,1],H[2,2]));centre=h[:3]+[0,0,lift]
  yield from self.move({arm:self.points[arm]+[0,0,lift]},2.)
  count=round(t*CONTROL_HZ)
  for k in range(1,count+1):
   f=k/count;f=f*f*(3-2*f)
   C=Rotation.from_euler('z',yaw*f,degrees=True).as_matrix()@H@Rotation.from_euler('x',-roll*f,degrees=True).as_matrix()
   self.points[arm]=centre+C@local+bias;self.rotations[arm]=C@relative;self.joints[arm]=self.solve(arm,self.points[arm],self.rotations[arm])
   if k%(2*CONTROL_HZ)==0:self.phase('turn the folded grip below the barrel for real counter support')
   yield self.emit()
 def settle_grasp_orientation(self,t=1.5):
  arm='right';_,actualR=ik.tcp_pose(self.measured[7:13],BASES[arm]);target=Rotation.from_euler('y',5,degrees=True).as_matrix()@actualR
  interpolation=Slerp([0,1],Rotation.from_matrix([self.rotations[arm],target]));count=round(t*CONTROL_HZ)
  for k in range(1,count+1):
   f=k/count;f=f*f*(3-2*f);self.rotations[arm]=interpolation([f]).as_matrix()[0];self.joints[arm]=self.solve(arm,self.points[arm],self.rotations[arm]);yield self.emit()
 def counter_fold(self,t=6.):
  arm='right';start=self.points[arm][2]
  for k in range(round(t*CONTROL_HZ)):
   angle=float(self.state['grade']['angle_deg']);force=float(self.state['robot_forces'][9:,:2,2].sum())
   if angle>=75.:break
   # A bounded native contact force, not an applied task-body force. The
   # reference only advances the existing robot position target slowly.
   if force<8. and start-self.points[arm][2]<.035:self.points[arm][2]-=.0005
   self.joints[arm]=self.solve(arm,self.points[arm],self.rotations[arm])
   if k%CONTROL_HZ==0:
    self.phase('use real counter reaction to fold the supported handle')
    print('COUNTER_PRESS',start-self.points[arm][2],force,flush=True)
   yield self.emit()
 def sequence(self):
  brace='right'
  self.phase('inspect the unfolded unplugged dryer and attached loose cable')
  yield from self.hold(1.)
  p=self.state['poses'][L.HEAD];B=L.quat_matrix(p[3:]);local=np.array((.005,0,0));point=p[:3]+B@local
  R=frame((.65,-.55,-np.sqrt(1-.65**2-.55**2)),(0,-1,0))
  self.aperture[brace]=OPEN
  yield from self.go(brace,point-R[:,2]*.070+[0,0,.030],R,2.)
  yield from self.move({brace:point},1.5)
  yield from self.calibrate_at(brace,L.HEAD,local,.7)
  yield from self.close(brace,-.0295,2.)
  self.phase('grip the actual barrel near its centre of mass')
  c=np.asarray(self.state['contacts']['robot_contact_counts'])
  if not(c[16,L.HEAD]>0 and c[17,L.HEAD]>0):raise RuntimeError('Both right pads must actually grip the barrel')
  yield from self.move({brace:self.points[brace]+[0,0,.085]},2.)
  self.phase('lift the free dryer and its attached loose cable')
  H=L.quat_matrix(self.state['poses'][L.HEAD,3:]);roll=np.degrees(np.arctan2(H[2,1],H[2,2]))
  yield from self.turn_barrel(165-roll,6.,nose_up=5.)
  for k in range(round(12.*CONTROL_HZ)):
   angle=self.state['grade']['angle_deg'];contacts=np.asarray(self.state['contacts']['robot_contact_counts'])
   if angle>=88. or (angle>=75. and np.any(contacts[9:,L.HANDLE])):break
   if k%(2*CONTROL_HZ)==0:self.phase('wait for the free native handle to fold under gravity')
   yield self.emit()
  self.phase('check the actual gravity folding before supported release')
  if self.state['grade']['angle_deg']<70:raise RuntimeError('The handle did not begin its real gravity fold')
  # Put the grip underneath before release. Counter force and the barrel's
  # own weight then finish folding and hold the true native stop.
  yield from self.return_for_counter_support()
  self.phase('position the folded grip beneath the barrel')
  h=self.state['poses'][L.HEAD];H=L.quat_matrix(h[3:])
  lowest=float(self.state['grade']['support_clearance_mm'])/1000.;drop=max(0.,lowest-.006)
  yield from self.move({brace:self.points[brace]-[0,0,drop]},3.)
  self.phase('lower the folded barrel close to the counter')
  yield from self.settle_grasp_orientation()
  yield from self.counter_fold()
  self.phase('withdraw the barrel fingers while the counter supports the grip')
  yield from self.move({brace:self.points[brace]-self.rotations[brace][:,2]*.050+[0,0,.020]},1.2,gaps={brace:OPEN})
  yield from self.move({brace:self.points[brace]+[0,0,.10]},2.)
  self.phase('release both hands and wait for the folded dryer to settle')
  while True:yield self.emit()
