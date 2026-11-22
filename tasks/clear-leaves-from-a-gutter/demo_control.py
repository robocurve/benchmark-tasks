"""Scripted Cartesian feedback; writes only ordinary robot joint actions."""
import numpy as np,json
from pathlib import Path
from scipy.optimize import least_squares
from geometry import HANDLE,Rotation
GRIP_R=Rotation.from_euler("x",30,degrees=True).as_matrix()@__import__("common").down(0)
from common import down
class DemoControl:
 def __init__(self):self.frames=json.loads(Path(__file__).with_name('demo-plan.json').read_text())['frames'];self.attachment=None
 def action(self,k,wanted,m,observed,hands):
  from yam_arena import ik,station
  frame=self.frames[k-1];phase=frame['stage'];out=wanted.copy();side='right';o=7
  if side not in frame.get('targets',{}):return out
  spec=frame['targets'][side];target=np.array(spec['position']);R=np.array(spec['rotation']);Q=m.orientation('scoop');origin=m.p('scoop');h=hands[side]
  if phase in ['align','grip']:
   target=origin+Q@HANDLE;R=Q@GRIP_R
   self.attachment=(Q.T@(h['tcp']-origin),Q.T@h['rotation'])
  elif phase not in ['approach','release','clear']:
   if self.attachment is None:self.attachment=(Q.T@(h['tcp']-origin),Q.T@h['rotation'])
   desiredQ=R@GRIP_R.T;desiredP=target-desiredQ@HANDLE
   target=desiredP+desiredQ@self.attachment[0];R=desiredQ@self.attachment[1]
  seed=np.clip(observed[o:o+6],ik.LOWER+1e-6,ik.UPPER-1e-6);p0,R0=ik.tcp_pose(seed,station.BASES[side]);bias=h['tcp']-p0;rbias=h['rotation']@R0.T
  def f(q):
   p,r=ik.tcp_pose(q,station.BASES[side]);return np.r_[p+bias-target,.15*ik._rot_error(R,rbias@r),.003*(q-wanted[o:o+6])]
  out[o:o+6]=least_squares(f,seed,bounds=(np.maximum(ik.LOWER,seed-.12),np.minimum(ik.UPPER,seed+.12)),max_nfev=25).x
  return out
