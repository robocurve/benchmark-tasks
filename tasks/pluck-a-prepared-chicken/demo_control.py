import numpy as np
from scipy.optimize import least_squares
class DemoControl:
 def __init__(self):
  import json
  from pathlib import Path
  self.plan=json.loads(Path(__file__).with_name('demo-plan.json').read_text())['frames'];self.offset=np.zeros(3);self.group=0;self.active=set()
 def action(self,k,wanted,m,observed,hands):
  from yam_arena import ik,station
  out=wanted.copy()
  stage=self.plan[k-1]['stage']
  if 'group ' in stage and stage.rsplit(' ',1)[1].isdigit():
   self.group=(int(stage.rsplit(' ',1)[1])-1)*3+1;self.active=set(range(self.group-1,self.group+2))
  if 'feather ' in stage and stage.rsplit(' ',1)[1].isdigit():
   self.group=int(stage.rsplit(' ',1)[1])-1;self.active={self.group}
  for side,o in [('left',0),('right',7)]:
   target,R=ik.tcp_pose(wanted[o:o+6],station.BASES[side])
   if side=='left':
    from geometry import C
    body_R=m.orientation('carcass')
    target=m.p('carcass')+body_R@(target-C)
    R=body_R@R
    # Once this bunch is out, stop pressing the recovering skin. Only command
    # the ordinary arm; material state and object poses are never changed.
    if self.active and self.active<=m.extracted:target+=np.array([0.,0.,.015])
   if side=='right':
    from geometry import TIPS,AXIS
    stage=self.plan[k-1]['stage']
    if stage.startswith(('approach group','place fingers around group','align group','grip group','approach feather','place fingers around feather','align feather','grip feather')):
     i=self.group
     if 'feather ' in stage and side not in m.grasp_state:
      measured=m.p(f'feather_{i}',[0,0,.0015])-(TIPS[i]+AXIS*.0015)
      # Damped visual centring avoids chasing a flexible tip at full gain.
      self.offset=.8*self.offset+.2*measured
     elif not stage.startswith('grip group') and side not in m.grasp_state:
      self.offset=m.p(f'feather_{i}',[0,0,.0015])-(TIPS[i]+AXIS*.0015)
     self.group=i;target+=self.offset
    elif stage in ('pluck along the quills','lift the freed feathers'):target+=self.offset
   from geometry import N,C
   if len(m.extracted)==N:
    # Drop the final freed feathers within the tray and withdraw support before
    # a held shaft can snag the other fingers. These are ordinary arm actions.
    out[o+6]=.7
    if side=='left':target=C+np.array([.015,-.08,.19])
   seed=np.clip(observed[o:o+6],ik.LOWER+1e-6,ik.UPPER-1e-6)
   p0,R0=ik.tcp_pose(seed,station.BASES[side]);bias=hands[side]['tcp']-p0;rbias=hands[side]['rotation']@R0.T
   def residual(q):
    p,r=ik.tcp_pose(q,station.BASES[side]);return np.r_[p+bias-target,.10*ik._rot_error(R,rbias@r),.001*(q-seed)]
   out[o:o+6]=least_squares(residual,seed,bounds=(np.maximum(ik.LOWER,seed-.12),np.minimum(ik.UPPER,seed+.12)),max_nfev=25).x
  return out
