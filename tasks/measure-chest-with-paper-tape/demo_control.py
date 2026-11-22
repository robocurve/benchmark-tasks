"""Measured TCP correction; outputs joint targets only, never object poses."""
import numpy as np,json
from pathlib import Path
from scipy.optimize import least_squares
from geometry import *
GRIPS={'left':0,'right':58}
class DemoControl:
 def __init__(self):
  self.frames=json.loads(Path(__file__).with_name('demo-plan.json').read_text())['frames'];self.offset={s:np.zeros(3) for s in GRIPS}
 def action(self,k,wanted,m,observed,hands):
  from yam_arena import ik,station
  phase=self.frames[k-1]['stage'];out=wanted.copy()
  for side,o in [('left',0),('right',7)]:
   target,R=ik.tcp_pose(wanted[o:o+6],station.BASES[side]);i=GRIPS[side]
   if phase in ['align','pinch'] and side not in m.pinch.held:self.offset[side]=.8*self.offset[side]+.2*(m.p(CELLS[i])-np.array(BODIES[2+i]['pos']))
   if phase in ['align','pinch']:
    target+=self.offset[side]
    if side not in m.pinch.held:R=m.orientation(CELLS[i])@Rotation.from_quat(BODIES[2+i]['quat']).as_matrix().T@R
   seed=np.clip(observed[o:o+6],ik.LOWER+1e-6,ik.UPPER-1e-6);p0,R0=ik.tcp_pose(seed,station.BASES[side]);bias=hands[side]['tcp']-p0;rbias=hands[side]['rotation']@R0.T
   def f(q):
    p,r=ik.tcp_pose(q,station.BASES[side]);return np.r_[p+bias-target,.1*ik._rot_error(R,rbias@r),.001*(q-seed)]
   out[o:o+6]=least_squares(f,seed,bounds=(np.maximum(ik.LOWER,seed-.12),np.minimum(ik.UPPER,seed+.12)),max_nfev=25).x
  return out
