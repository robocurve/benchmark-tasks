"""Track the settled cap for pickup using robot commands only."""
import numpy as np
from scipy.optimize import least_squares
class DemoControl:
 def __init__(self):
  from pathlib import Path
  import json
  self.labels=[f['stage'] for f in json.loads(Path(__file__).with_name('demo-plan.json').read_text())['frames']]
 def action(self,k,wanted,m,observed,hands):
  label=self.labels[k-1]
  if label in ('hold the loose cap by its rim','stretch the loose cap opening','settle the stretched opening around the crown','lower both sides of the rim') and set(m.grasp_state)!={'left','right'}:
   raise RuntimeError('Scripted demonstration lost a required grip; stop this diagnostic.')
  from yam_arena import ik,station
  out=wanted.copy()
  for side,o in [('left',0),('right',7)]:
   target,R=ik.tcp_pose(wanted[o:o+6],station.BASES[side])
   seed=np.clip(observed[o:o+6],ik.LOWER+1e-6,ik.UPPER-1e-6)
   p0,R0=ik.tcp_pose(seed,station.BASES[side]);bias=hands[side]['tcp']-p0;rbias=hands[side]['rotation']@R0.T
   def residual(q):
    pp,rr=ik.tcp_pose(q,station.BASES[side]);return np.r_[pp+bias-target,.10*ik._rot_error(R,rbias@rr),.001*(q-seed)]
   out[o:o+6]=least_squares(residual,seed,bounds=(np.maximum(ik.LOWER,seed-.12),np.minimum(ik.UPPER,seed+.12)),max_nfev=25).x
  return out
