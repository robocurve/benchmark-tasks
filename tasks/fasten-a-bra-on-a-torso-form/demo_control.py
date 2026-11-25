"""Native tracking correction; outputs ordinary joint commands only."""
import numpy as np
from scipy.optimize import least_squares
class DemoControl:
 def __init__(self):
  import json
  from pathlib import Path
  self.plan=json.loads(Path(__file__).with_name('demo-plan.json').read_text())['frames'];self.offset={s:np.zeros(3) for s in ['left','right']};self.rot={s:np.eye(3) for s in ['left','right']};self.body_bias={s:np.zeros(3) for s in ['left','right']};self.release_pose={};self.release_reference={}
 def action(self,k,wanted,m,observed,hands):
  from yam_arena import ik,station
  out=wanted.copy()
  for side,o in [('left',0),('right',7)]:
   target,R=ik.tcp_pose(wanted[o:o+6],station.BASES[side])
   from geometry import X,Z,GAP
   stage=self.plan[k-1]['stage'];name=side+'_tab'
   if stage=='stretch the wings and put the hooks past the eyes' and len(m.pinch.held)!=2:raise RuntimeError('Demo pickup failed: both closure tapes must be held before stretching.')
   if stage in ['approach the open back closure','put fingers around the two closure tapes','pinch both tapes']:
    if stage!='pinch both tapes':
     sign=1 if side=='left' else -1;self.offset[side]=m.p(name,[0,sign*.010,0])-np.array([X,sign*(GAP+.010),Z]);self.rot[side]=m.orientation(name)
    target+=self.offset[side];R=self.rot[side]@R
   elif stage=='release the two tapes' and side in self.release_pose:
    target,R=self.release_pose[side]
   elif stage in ['withdraw from the closure','check the released fastening'] and side in self.release_pose:
    target=target-self.release_reference[side]+self.release_pose[side][0];R=self.release_pose[side][1]
   elif side in m.pinch.held:
    nominal=target.copy()
    if stage!='release the two tapes':self.body_bias[side]=np.clip(self.body_bias[side]+.10*(nominal-m.p(name)),-.008,.008)
    target=nominal+self.body_bias[side];grip=m.pinch.held[side];R=grip['local_rotation'].T;target-=R@grip['local_point']
    if stage=='release the two tapes':self.release_pose[side]=(target.copy(),R.copy());self.release_reference[side]=nominal.copy()
   seed=np.clip(observed[o:o+6],ik.LOWER+1e-6,ik.UPPER-1e-6)
   p0,R0=ik.tcp_pose(seed,station.BASES[side]);bias=hands[side]['tcp']-p0;rbias=hands[side]['rotation']@R0.T
   def residual(q):
    p,r=ik.tcp_pose(q,station.BASES[side]);return np.r_[p+bias-target,.10*ik._rot_error(R,rbias@r),.001*(q-seed)]
   out[o:o+6]=least_squares(residual,seed,bounds=(np.maximum(ik.LOWER,seed-.12),np.minimum(ik.UPPER,seed+.12)),max_nfev=25).x
  return out
