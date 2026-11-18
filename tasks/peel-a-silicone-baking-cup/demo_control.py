"""Correct the flat pull-tab approach for native settling using robot joints."""
import numpy as np
from scipy.optimize import least_squares
from geometry import C
class DemoControl:
 def __init__(self):self.offset=None
 def action(self,k,wanted,stage,m,observed):
  from yam_arena import ik,station
  if self.offset is None:self.offset=m.p('cup_tab_8',[0,.006,0])-(C+[0,-.048,.032])
  p,R=ik.tcp_pose(wanted[7:13],station.BASES['right']);p+=min(1.,k/60)*self.offset
  seed=np.clip(observed[7:13],ik.LOWER+1e-6,ik.UPPER-1e-6)
  def residual(q):
   pp,rr=ik.tcp_pose(q,station.BASES['right']);return np.r_[pp-p,.15*ik._rot_error(R,rr),.001*(q-seed)]
  out=wanted.copy();out[7:13]=least_squares(residual,seed,bounds=(np.maximum(ik.LOWER,seed-.12),np.minimum(ik.UPPER,seed+.12)),max_nfev=35).x
  return out
