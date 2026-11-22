"""Measured material/gripper pose correction; outputs only ordinary joint actions."""
import numpy as np,json
from pathlib import Path
from scipy.optimize import least_squares
from geometry import *
class DemoControl:
 def __init__(self):
  self.frames=json.loads(Path(__file__).with_name('demo-plan.json').read_text())['frames'];self.offset={s:np.zeros(3) for s in ['left','right']}
 def action(self,k,wanted,m,observed,hands):
  from yam_arena import ik,station
  stage=self.frames[k-1]['stage'];out=wanted.copy()
  if ':' in stage:
   j=int(stage.split(':')[0]);phase=stage.split(':')[1];targets={'left':(j%2,[7,12,16][j]),'right':(2+j,N-1)}
   if phase in ['lift longitudinal','lift longitudinal high','lift cross strip','feed onto pie','draw across','lower cross strip'] and len(m.pinch.held)!=2:raise RuntimeError('Both strip ends must remain physically pinched through placement')
   if (phase in ['shift support','clear right hand','retreat right','park right','replace longitudinal']) and 'left' not in m.pinch.held:raise RuntimeError('Longitudinal strip slipped before replacement')
  else:phase=stage;targets={}
  for side,o in [('left',0),('right',7)]:
   target,R=ik.tcp_pose(wanted[o:o+6],station.BASES[side])
   if side in targets:
    s,i=targets[side];name=STRIPS[s][i];rest=ORIGINS[s]+ROTATIONS[s].apply([0,(i-(N-1)/2)*DX,0])
    if phase in ['align','pinch'] and side not in m.pinch.held:self.offset[side]=.8*self.offset[side]+.2*(m.p(name)-rest)
    if phase in ['align','pinch','shift support','lift longitudinal','lift longitudinal high','lift cross strip','replace longitudinal','release longitudinal']:
     target+=self.offset[side]
   seed=np.clip(observed[o:o+6],ik.LOWER+1e-6,ik.UPPER-1e-6);p0,R0=ik.tcp_pose(seed,station.BASES[side]);bias=hands[side]['tcp']-p0;rbias=hands[side]['rotation']@R0.T
   def f(q):
    p,r=ik.tcp_pose(q,station.BASES[side]);return np.r_[p+bias-target,.1*ik._rot_error(R,rbias@r),.001*(q-seed)]
   out[o:o+6]=least_squares(f,seed,bounds=(np.maximum(ik.LOWER,seed-.12),np.minimum(ik.UPPER,seed+.12)),max_nfev=25).x
  return out
