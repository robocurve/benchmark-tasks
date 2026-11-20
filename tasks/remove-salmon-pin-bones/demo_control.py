import numpy as np
from scipy.optimize import least_squares
class DemoControl:
 def __init__(self):
  import json
  from pathlib import Path
  self.plan=json.loads(Path(__file__).with_name('demo-plan.json').read_text())['frames'];self.offset=np.zeros(3);self.bone=0
 def action(self,k,wanted,m,observed,hands):
  from yam_arena import ik,station
  out=wanted.copy()
  for side,o in [('left',0),('right',7)]:
   target,R=ik.tcp_pose(wanted[o:o+6],station.BASES[side])
   if side=='right':
    from geometry import TIPS,AXIS
    stage=self.plan[k-1]['stage']
    if stage.startswith(('approach bone','place tweezers around bone','align tweezers on bone','grip bone')):
     i=int(stage.rsplit(' ',1)[1])-1
     if not stage.startswith('grip bone') and side not in m.grasp_state:self.offset=m.p(f'bone_tip_{i}',[0,0,.0025])-(TIPS[i]+AXIS*.0025)
     self.bone=i;target+=self.offset
    elif stage in ('draw bone along its direction','lift the extracted bone'):target+=self.offset
   seed=np.clip(observed[o:o+6],ik.LOWER+1e-6,ik.UPPER-1e-6)
   p0,R0=ik.tcp_pose(seed,station.BASES[side]);bias=hands[side]['tcp']-p0;rbias=hands[side]['rotation']@R0.T
   def residual(q):
    p,r=ik.tcp_pose(q,station.BASES[side]);return np.r_[p+bias-target,.10*ik._rot_error(R,rbias@r),.001*(q-seed)]
   out[o:o+6]=least_squares(residual,seed,bounds=(np.maximum(ik.LOWER,seed-.12),np.minimum(ik.UPPER,seed+.12)),max_nfev=25).x
  return out
