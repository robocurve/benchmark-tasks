"""Scripted visual pose feedback; outputs only ordinary robot joint actions."""
import numpy as np,json
from pathlib import Path
from scipy.optimize import least_squares
from geometry import Q0,HANDLE,EDGE,Rotation
from common import down
class DemoControl:
 def __init__(self):
  self.frames=json.loads(Path(__file__).with_name('demo-plan.json').read_text())['frames'];self.attachments={}
 def action(self,k,wanted,m,observed,hands):
  from yam_arena import ik,station
  phase=self.frames[k-1]['stage'];out=wanted.copy();wood_grip=np.array([0,.055,0.]);held=np.array([.27,0,.7475+.12])
  for side,o,name,point,gripR in [('left',0,'stick',wood_grip,Q0@down(180)@Rotation.from_euler("x",40,degrees=True).as_matrix()),('right',7,'knife',HANDLE,Q0@down(-90)@Rotation.from_euler("x",40,degrees=True).as_matrix())]:
   target,R=ik.tcp_pose(wanted[o:o+6],station.BASES[side]);Q=m.orientation(name);origin=m.p(name);h=hands[side]
   localR=Q0.T@gripR;desiredQ=R@localR.T;desiredP=target-desiredQ@point
   if phase in ['align','grip']:
    target=origin+Q@point;R=Q@localR
   elif phase not in ['approach','release','clear','settle']:
    if side not in self.attachments:self.attachments[side]=(Q.T@(h['tcp']-origin),Q.T@h['rotation'])
    if side=='right' and any(phase.startswith(s) for s in ['approach cut','set edge','shave','clear edge']):
     W=m.orientation('stick');desiredP=m.p('stick')+W@Q0.T@(desiredP-held);desiredQ=W@Q0.T@desiredQ
    a,b=self.attachments[side];target=desiredP+desiredQ@a;R=desiredQ@b
   seed=np.clip(observed[o:o+6],ik.LOWER+1e-6,ik.UPPER-1e-6);p0,R0=ik.tcp_pose(seed,station.BASES[side]);bias=h['tcp']-p0;rbias=h['rotation']@R0.T
   def f(q):
    p,r=ik.tcp_pose(q,station.BASES[side]);return np.r_[p+bias-target,.15*ik._rot_error(R,rbias@r),.001*(q-seed)]
   out[o:o+6]=least_squares(f,seed,bounds=(np.maximum(ik.LOWER,seed-.12),np.minimum(ik.UPPER,seed+.12)),max_nfev=25).x
  return out
