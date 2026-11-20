"""Ordinary robot tracking and measured object-pose compensation."""
import numpy as np
from scipy.optimize import least_squares
class DemoControl:
 def __init__(self):
  import json
  from pathlib import Path
  self.plan=json.loads(Path(__file__).with_name('demo-plan.json').read_text())['frames'];self.offset={s:np.zeros(3) for s in ['left','right']};self.tail_rotation=None;self.tool_rotation=None;self.had_tool_grasp=False;self.press_offset=0.;self.starts={};self.counts={}
  for i,f in enumerate(self.plan):self.starts.setdefault(f["stage"],i);self.counts[f["stage"]]=self.counts.get(f["stage"],0)+1
 def action(self,k,wanted,m,observed,hands):
  from yam_arena import ik,station
  from geometry import TAIL,TOOL,C
  from common import down
  from scipy.spatial.transform import Rotation
  GRIP_R=down(0)@Rotation.from_euler("x",50,degrees=True).as_matrix()
  out=wanted.copy();stage=self.plan[k-1]['stage']
  if m.tool_pinch.held:self.had_tool_grasp=True
  if stage=='lift the scaler' and k-1==self.starts[stage] and (not m.tail_pinch.held or not self.had_tool_grasp):raise RuntimeError('Demo pickup failed: both initial grasps are required.')
  if stage=='lift the scaler' and k==self.starts[stage]+self.counts[stage] and m.p('scaler')[2]<TOOL[2]+.05:raise RuntimeError('Demo failed to lift the scaler clear of the table.')
  for side,o in [('left',0),('right',7)]:
   target,R=ik.tcp_pose(wanted[o:o+6],station.BASES[side]);name='fish' if side=='left' else 'scaler';rest=TAIL if side=='left' else TOOL;pinch=m.tail_pinch if side=='left' else m.tool_pinch
   exact=self.plan[k-1].get('tcp_goals',{}).get(side)
   if exact is not None:target=np.array(exact['position']);R=np.array(exact['rotation'])
   if stage in ['place fingers around tail and handle','grip tail and scaler handle']:
    if stage.startswith('place'):self.offset[side]=m.p(name,[0,.103,0] if side=='left' else [0,0,0])-rest
    target+=self.offset[side]
   elif side=='left' and 'release' not in stage and stage!='leave hands clear' and stage!='inspect remaining scales and intact skin':target+=self.offset[side]
   elif side=='right' and ('scraping lane' in stage or stage.startswith('scrape lane')):
    F=m.orientation('fish');target=m.p('fish')+F@(target-C);R=F@R
    if stage.startswith('load scraping') or stage.startswith('scrape lane'):
     from geometry import PATCHES
     load=max(700*max(0.,-float(np.dot(m.joint_gap('skin_bond_'+str(i)),m.orientation('skin_'+str(i))[:,2]))) for i in PATCHES)
     # Integrate measured pressure slowly rather than chase the nominal surface.
     self.press_offset=float(np.clip(self.press_offset+.00015*(load-.25),-.0003,.0025))
     target+=(R@GRIP_R.T)[:,2]*self.press_offset
    else:self.press_offset=0.

   if side in pinch.held and stage not in ['release scaler and fish','leave hands clear','inspect remaining scales and intact skin']:
    grip=pinch.held[side]
    if side=='left':
     if self.tail_rotation is None:self.tail_rotation=m.orientation('fish').copy()
     body_R=self.tail_rotation
    else:
     if self.tool_rotation is None:self.tool_rotation=m.orientation('scaler').copy()
     if stage in ['grip tail and scaler handle','lift the scaler']:body_R=self.tool_rotation
     elif stage=='approach scraping lane 0':
      from scipy.spatial.transform import Slerp
      from geometry import surface
      u=np.clip((k-1-self.starts[stage])/max(1,self.counts[stage]-1),0,1);goal=m.orientation('fish')@surface(-.022,.045)[1]
      body_R=Slerp([0,1],Rotation.from_matrix([self.tool_rotation,goal]))(float(u)).as_matrix()
     else:body_R=R@GRIP_R.T
    R=body_R@grip['local_rotation'].T;target-=R@grip['local_point']
    if side=='left':target-=body_R@np.array([0,.103,0])
   seed=np.clip(observed[o:o+6],ik.LOWER+1e-6,ik.UPPER-1e-6);p0,R0=ik.tcp_pose(seed,station.BASES[side]);bias=hands[side]['tcp']-p0;rbias=hands[side]['rotation']@R0.T
   def residual(q):
    p,r=ik.tcp_pose(q,station.BASES[side]);return np.r_[p+bias-target,.1*ik._rot_error(R,rbias@r),.001*(q-seed)]
   out[o:o+6]=least_squares(residual,seed,bounds=(np.maximum(ik.LOWER,seed-.12),np.minimum(ik.UPPER,seed+.12)),max_nfev=25).x
  return out
