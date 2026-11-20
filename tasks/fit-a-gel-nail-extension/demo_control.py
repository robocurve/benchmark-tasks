"""Track the observed rigid tip through ordinary robot joint targets."""
import numpy as np
from scipy.optimize import least_squares
class DemoControl:
 def __init__(self):self.grasp=None;self.grasp_rotation=None;self.correction=np.zeros(3);self.seating_z=None;self.rotation_correction=np.zeros(3);self.flash_hand=None;self.seated_hand=None
 def action(self,k,wanted,m,hands,observed):
  if self.flash_hand is None and np.count_nonzero(m.dose>=10.)>=int(np.ceil(.9*len(m.dose))):
   self.flash_hand=(k,hands['right']['tcp'].copy(),hands['right']['rotation'].copy())
  if self.flash_hand is not None:
   # The real procedure releases the holding hand after flash cure, then
   # completes exposure hands-free. Keep the actual hand pose while opening,
   # withdraw along the dry free edge, and remain clear for the proof load.
   from yam_arena import ik,station
   from geometry import FRAME
   k0,p0,R=self.flash_hand;u=ik.smoothstep(float(np.clip((k-k0-20)/60,0,1)))
   target=p0+FRAME@np.array([.09*u,0,0]);h=hands['right']
   seed=np.clip(observed[7:13],ik.LOWER+1e-6,ik.UPPER-1e-6)
   pp,rr=ik.tcp_pose(seed,station.BASES['right']);bias=h['tcp']-pp;rbias=h['rotation']@rr.T
   def f(q):
    pos,ori=ik.tcp_pose(q,station.BASES['right']);return np.r_[pos+bias-target,.10*ik._rot_error(R,rbias@ori),.001*(q-seed)]
   out=wanted.copy();out[7:13]=least_squares(f,seed,bounds=(np.maximum(ik.LOWER,seed-.10),np.minimum(ik.UPPER,seed+.10)),max_nfev=25).x
   out[13]=.16+(.7-.16)*ik.smoothstep(float(np.clip((k-k0)/20,0,1)))
   return out
  if self.seated_hand is not None:
   # Hold the actual robot pose still once the entire curved interface is wet.
   # Continued seating corrections during curing can disturb new adhesive bonds.
   from yam_arena import ik,station
   target,R=self.seated_hand;h=hands['right'];seed=np.clip(observed[7:13],ik.LOWER+1e-6,ik.UPPER-1e-6)
   pp,rr=ik.tcp_pose(seed,station.BASES['right']);bias=h['tcp']-pp;rbias=h['rotation']@rr.T
   def hold(q):
    pp,rr=ik.tcp_pose(q,station.BASES['right']);return np.r_[pp+bias-target,.1*ik._rot_error(R,rbias@rr),.001*(q-seed)]
   out=wanted.copy();out[7:13]=least_squares(hold,seed,bounds=(np.maximum(ik.LOWER,seed-.10),np.minimum(ik.UPPER,seed+.10)),max_nfev=25).x;out[13]=.16
   return out
  if k==220:
   from geometry import REST
   if m.p('extension')[2]<REST[2]+.02:raise RuntimeError('The extension was not lifted; stop this diagnostic.')
  if 220<=k<=1720:
   h=hands['right'];rel=h['rotation'].T@(m.p('extension')-h['tcp'])
   if self.grasp is None:
    g=m.pinch.held.get('right')
    if g is None:raise RuntimeError('No finite contact grip on the extension.')
    self.grasp=g['local_point'].copy();self.grasp_rotation=g['local_rotation'].copy()
   elif np.linalg.norm(rel-self.grasp)>.008:raise RuntimeError('Extension slipped from the physical grip; stop diagnostic.')
  if k<160:return wanted
  from yam_arena import ik,station
  h=hands['right'];R0=np.array([[0,0,-1],[1,0,0],[0,-1,0.]])
  p,R=ik.tcp_pose(wanted[7:13],station.BASES['right'])
  if wanted[13]>.2:return wanted
  # The plan's TCP is 3 mm above the tip origin. Preserve that tool relation,
  # correcting only for observed slip, not moving the object itself.
  desired_R=R@R0.T;goal=p-desired_R@np.array([0,0,.003])
  # Correct steady motor tracking error through bounded hand targets.
  # Use observed compression as an anti-windup guard while seating.
  if 340<=k<=1720:
   from geometry import PATCHES,BASE,arch,tip_width
   from scipy.spatial.transform import Rotation
   contact_count=0;gaps=[]
   for patch in PATCHES:
    q=m.orientation('extension').T@(m.p(patch['name'])+[0,0,.00012]-m.p('extension'))
    gap=float(q[2]-(.003-arch(np.clip(q[1],-.013,.013))))
    gaps.append(gap)
    contact_count+=bool(-.020<=q[0]<=.008 and abs(q[1])<=tip_width(q[0]) and -.00015<gap<.00020)
   contact_fraction=contact_count/len(PATCHES)
   if contact_fraction>=.9 and np.mean(m.wet>=1.)>=.9:self.seated_hand=(h['tcp'].copy(),h['rotation'].copy())
   rotation_error=Rotation.from_matrix(desired_R@m.orientation('extension').T).as_rotvec()
   self.rotation_correction=np.clip(self.rotation_correction+.025*rotation_error,-.05,.05)
   desired_R=Rotation.from_rotvec(self.rotation_correction).as_matrix()@desired_R
   error=goal-m.p('extension')
   error[2]=0.
   self.correction=np.clip(self.correction+.08*error,-.003,.003)
   goal+=self.correction
   # Slowly advance a commanded seating height, then hold the contacted tip.
   # Referencing each target to the measured height alone stalled against
   # motor tracking error before contact. This bounded contact controller
   # changes only robot commands; the gel and tip remain native bodies.
   if self.seating_z is None:self.seating_z=float(m.p('extension')[2])
   # Bidirectional geometric feedback avoids driving past the contact band.
   median_gap=float(np.median(gaps))
   if contact_fraction<.9:
    self.seating_z-=float(np.clip(.12*(.00002-median_gap),-.00002,.00002))
   self.seating_z=float(np.clip(self.seating_z,BASE[2]-.002,BASE[2]+.025))
   goal[2]=self.seating_z
  # Use the compliant grip's captured rest transform. Chasing its transient
  # deflection made the controller fight the native restoring drive.
  relative=self.grasp if self.grasp is not None else h['rotation'].T@(m.p('extension')-h['tcp'])
  relative_R=self.grasp_rotation if self.grasp_rotation is not None else h['rotation'].T@m.orientation('extension')
  seed=np.clip(observed[7:13],ik.LOWER+1e-6,ik.UPPER-1e-6)
  p0,r0=ik.tcp_pose(seed,station.BASES['right']);bias=h['tcp']-p0;rbias=h['rotation']@r0.T
  def f(q):
   pp,rr=ik.tcp_pose(q,station.BASES['right']);rr=rbias@rr
   return np.r_[pp+bias+rr@relative-goal,.10*ik._rot_error(desired_R,rr@relative_R),.001*(q-seed)]
  out=wanted.copy();out[7:13]=least_squares(f,seed,bounds=(np.maximum(ik.LOWER,seed-.10),np.minimum(ik.UPPER,seed+.10)),max_nfev=25).x;return out
