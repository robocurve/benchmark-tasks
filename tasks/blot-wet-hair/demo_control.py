"""Ordinary joint-space tracking; no towel or hair poses are assigned."""
import numpy as np
from scipy.optimize import least_squares
class DemoControl:
 def action(self,k,wanted,m,observed,hands):
  if k<1260 and set(m.grasp_state)!={'left','right'}:raise RuntimeError('Towel grip lost; stop diagnostic')
  from yam_arena import ik,station
  out=wanted.copy()
  for s,o in [('left',0),('right',7)]:
   target,R=ik.tcp_pose(wanted[o:o+6],station.BASES[s]);h=hands[s]
   seed=np.clip(observed[o:o+6],ik.LOWER+1e-6,ik.UPPER-1e-6);p0,R0=ik.tcp_pose(seed,station.BASES[s]);bias=h['tcp']-p0;rbias=h['rotation']@R0.T
   def residual(q):
    p,r=ik.tcp_pose(q,station.BASES[s]);return np.r_[p+bias-target,.1*ik._rot_error(R,rbias@r),.001*(q-seed)]
   out[o:o+6]=least_squares(residual,seed,bounds=(np.maximum(ik.LOWER,seed-.12),np.minimum(ik.UPPER,seed+.12)),max_nfev=25).x
  return out
