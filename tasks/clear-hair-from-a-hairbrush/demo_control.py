"""Object-local scripted waypoints, implemented only as robot joint targets."""
import numpy as np
from scipy.optimize import least_squares
class DemoControl:
 def __init__(self):self.offset=None;self.brush_R=np.eye(3)
 def action(self,k,wanted,m,observed,hands):
  from yam_arena import ik,station
  from geometry import BRUSH,TOOL
  from common import down
  if self.offset is None:self.offset={'left':np.zeros(3),'right':m.p('comb')-TOOL}
  if k<=100:
   self.offset['left']=m.p('brush',[0,.105,0])-(BRUSH+[0,.105,0]);self.brush_R=m.orientation('brush').copy()
  out=wanted.copy();u=min(1.,k/60)
  for side,o in [('left',0),('right',7)]:
   target,R=ik.tcp_pose(wanted[o:o+6],station.BASES[side]);tool_tip_local=None;tip_target=None;tool_axis_local=None;axis_target=None
   if side=='left':
    target+=u*self.offset['left'];R=ik.slerp_matrix(np.eye(3),self.brush_R,u)@R
   elif 180<k<=940:
    target=m.p('brush')+m.orientation('brush')@(target-BRUSH);R=m.orientation('brush')@R
   else:target+=u*self.offset['right']
   if side=='right' and 180<=k<=1000:
    H=hands['right'];t_rel=H['rotation'].T@(m.p('comb')-H['tcp']);r_rel=H['rotation'].T@m.orientation('comb');Rtool=R@down().T
    tool_tip_local=t_rel+r_rel@np.array([0,.13,0]);tip_target=target+Rtool@np.array([0,.13,0]);tool_axis_local=r_rel[:,1];axis_target=Rtool[:,1]
    R=Rtool@r_rel.T;target=target-R@t_rel
   seed=np.clip(observed[o:o+6],ik.LOWER+1e-6,ik.UPPER-1e-6)
   p0,R0=ik.tcp_pose(seed,station.BASES[side]);bias=hands[side]['tcp']-p0;rbias=hands[side]['rotation']@R0.T
   def residual(q):
    pp,rr=ik.tcp_pose(q,station.BASES[side]);actual_R=rbias@rr
    position=pp+bias-target if tool_tip_local is None else pp+bias+actual_R@tool_tip_local-tip_target
    orientation=ik._rot_error(R,actual_R) if tool_axis_local is None else axis_target-actual_R@tool_axis_local
    return np.r_[position*np.array([3.,1.,3.]),(.15 if tool_axis_local is not None else .08)*orientation,.001*(q-seed)]
   out[o:o+6]=least_squares(residual,seed,bounds=(np.maximum(ik.LOWER,seed-.12),np.minimum(ik.UPPER,seed+.12)),max_nfev=25).x
  return out
