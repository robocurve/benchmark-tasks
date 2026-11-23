"""Measured-pose tracking; all manipulation uses ordinary robot commands."""
import numpy as np
from scipy.optimize import least_squares
class DemoControl:
 def __init__(self):
  import json
  from pathlib import Path
  self.plan=json.loads(Path(__file__).with_name('demo-plan.json').read_text())['frames'];self.offset={s:np.zeros(3) for s in ['left','right']};self.tool_rotation=None;self.starts={};self.counts={}
  for i,f in enumerate(self.plan):self.starts.setdefault(f['stage'],i);self.counts[f['stage']]=self.counts.get(f['stage'],0)+1
 def action(self,k,wanted,m,observed,hands):
  from yam_arena import ik,station
  from geometry import FILM,PLATE,TOOL,NX,NY,DX,DY
  from common import down
  out=wanted.copy();stage=self.plan[k-1]['stage']
  from scipy.spatial.transform import Rotation
  roll_body=Rotation.from_euler('z',180,degrees=True).as_matrix()
  if stage=='lift film clear of release backing' and len(m.film_pinch.held)!=2:raise RuntimeError('Demo pickup failed: both film edges must be held.')
  if stage=='lift roller' and not m.tool_pinch.held:raise RuntimeError('Demo pickup failed: roller must be held.')
  for side,o in [('left',0),('right',7)]:
   target,R=ik.tcp_pose(wanted[o:o+6],station.BASES[side])
   if 'film edges' in stage and stage not in ['approach film edges','release film edges']:
    name=f'film_{NX//2}_{NY-1 if side=="left" else 0}';rest=FILM+[DX/2,(1 if side=='left' else -1)*(NY-1)/2*DY,0]
    if stage!='pinch film edges':self.offset[side]=m.p(name)-rest
    target+=self.offset[side]
   if side in m.film_pinch.held and stage not in ['release film edges','withdraw sideways from film']:
    grip=m.film_pinch.held[side];target+=R@np.array([0.,0.,.006]);R=grip['local_rotation'].T;target-=R@grip['local_point']
   if side=='right':
    if stage in ['put fingers around roller handle','grip roller handle']:
     if stage!='grip roller handle':self.offset[side]=m.p('roller_handle')-TOOL
     target+=self.offset[side]
    if stage in ['return roller above table','put roller on table','release roller']:target+=self.offset[side]
    if 'roller lane' in stage or 'rolling lane' in stage or 'roller from lane' in stage:target+=m.p('plate')-PLATE
    if side in m.tool_pinch.held and stage not in ['release roller','leave both hands clear','inspect the individual well seals']:
     grip=m.tool_pinch.held[side]
     if self.tool_rotation is None:self.tool_rotation=m.orientation('roller_handle').copy()
     body_R=roll_body
     if stage in ['grip roller handle','lift roller','put roller on table']:body_R=self.tool_rotation
     elif stage in ['return roller above table','approach rolling lane 0']:
      # Preserve the reachable joint-path orientation while moving between poses.
      # Independently interpolating tool rotation can demand an unreachable wrist pose.
      from geometry import TOOL_REST_R
      pickup=down(0)@Rotation.from_euler('x',50,degrees=True).as_matrix()
      body_R=R@pickup.T@TOOL_REST_R.as_matrix()
     R=body_R@grip['local_rotation'].T;target-=R@grip['local_point']
   seed=np.clip(observed[o:o+6],ik.LOWER+1e-6,ik.UPPER-1e-6);p0,R0=ik.tcp_pose(seed,station.BASES[side]);bias=hands[side]['tcp']-p0;rbias=hands[side]['rotation']@R0.T
   def residual(q):
    p,r=ik.tcp_pose(q,station.BASES[side]);return np.r_[p+bias-target,.10*ik._rot_error(R,rbias@r),.001*(q-seed)]
   out[o:o+6]=least_squares(residual,seed,bounds=(np.maximum(ik.LOWER,seed-.12),np.minimum(ik.UPPER,seed+.12)),max_nfev=25).x
  return out
