"""Robot-only tracking correction; never changes task objects."""
import numpy as np
from scipy.optimize import least_squares
class DemoControl:
 def __init__(self):self.offset=None;self.hair_x=None;self.grip_names={};self.grip_offset={};self.band_offset_x=0.;self.loop_correction=0.;self.loop_correction_y=0.
 def action(self,k,wanted,m,observed,hands):
  if 150<=k<=500 and set(m.grasp_state)!={'left','right'}:
   raise RuntimeError('Scripted demonstration lost a required grip; stop this diagnostic before further motion.')
  if k==500 and m.measure()['secured_locks']<6:
   raise RuntimeError('Elastic did not enclose the hair; stop before release.')
  from yam_arena import ik,station
  if self.offset is None:
    from geometry import POINTS,M
    self.offset={s:m.nodes[i]-POINTS[i] for s,i in [('left',M+M//4),('right',M+3*M//4)]}
  out=wanted.copy()
  for side,o in [('left',0),('right',7)]:
   target,R=ik.tcp_pose(wanted[o:o+6],station.BASES[side])
   # The pickup follows the settled band. Fade that correction before
   # reaching the hair so its world height is not lowered by band sag.
   weight=min(1.,k/60)*max(0.,min(1.,(220-k)/80))
   offset=self.offset[side]
   if 80<=k<=100:
    if side not in self.grip_names:
     from geometry import M
     self.grip_names[side]=f'band_segment_{M//4 if side=="left" else 3*M//4:03d}'
    if side not in m.grasp_state:
     from geometry import M
     centre=m.nodes[M+(M//4 if side=='left' else 3*M//4)]
     self.grip_offset[side]=centre-R@np.array([0.,0.,-.001])-target
   if side in self.grip_offset:
    blend=float(np.clip((k-80)/20,0,1));offset=(1-blend)*offset+blend*self.grip_offset[side]
   target+=weight*offset
   if k>=220:
    from geometry import STRANDS,SEGMENTS
    if k<=300 or self.hair_x is None:
     centres=[]
     for strand in range(STRANDS):
      p=np.array([m.p(f'hair_{strand}_{i}') for i in range(SEGMENTS)])
      centres.append(p[np.argmin(abs(p[:,2]-target[2])),0])
     self.hair_x=float(np.clip((min(centres)+max(centres))/2,.27,.33))
    if side=='left' and 290<=k<=440:
     from geometry import M
     centre=(m.nodes[M:2*M,0].min()+m.nodes[M:2*M,0].max())/2
     self.loop_correction=float(np.clip(self.loop_correction+.05*(self.hair_x-centre),-.010,.010))
     centre_y=(m.nodes[M:2*M,1].min()+m.nodes[M:2*M,1].max())/2
     self.loop_correction_y=float(np.clip(self.loop_correction_y-.05*centre_y,-.010,.010))
    target[0]+=min(1.,(k-220)/80)*(self.hair_x-.289)+self.loop_correction
    target[1]+=self.loop_correction_y
   seed=np.clip(observed[o:o+6],ik.LOWER+1e-6,ik.UPPER-1e-6)
   p0,R0=ik.tcp_pose(seed,station.BASES[side]);bias=hands[side]['tcp']-p0;rbias=hands[side]['rotation']@R0.T
   def residual(q):
    pp,rr=ik.tcp_pose(q,station.BASES[side]);return np.r_[pp+bias-target,.10*ik._rot_error(R,rbias@rr),.001*(q-seed)]
   out[o:o+6]=least_squares(residual,seed,bounds=(np.maximum(ik.LOWER,seed-.12),np.minimum(ik.UPPER,seed+.12)),max_nfev=25).x
  return out
