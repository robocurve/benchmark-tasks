"""Native-pose feedback for cap alignment; outputs ordinary arm joint targets."""
import numpy as np
from scipy.spatial.transform import Rotation
class DemoControl:
 def __init__(self):self.start=None;self.handoff=None
 def action(self,k,wanted,model,hands,current):
  if k<260 or k>560:return wanted
  if k>480:
   if self.handoff is None:self.handoff=current[:6].copy()
   u=np.clip((k-480)/80,0,1);u=u*u*(3-2*u);result=wanted.copy();result[:6]=(1-u)*self.handoff+u*wanted[:6];return result
  from yam_arena import ik,station
  head=model.p('cigar',[0,0,.068]);tool=model.p('cutter');axis=-model.orientation('cutter')[:,2]
  if self.start is None:self.start=head.copy()
  u=np.clip((k-260)/60,0,1);u=u*u*(3-2*u);goal=self.start*(1-u)+tool*u
  if k>440:
   v=np.clip((k-440)/40,0,1);v=v*v*(3-2*v);goal=tool+np.array([0,0,.080*v])
  delta=(goal-head)*.55;delta*=min(1.,.002/max(np.linalg.norm(delta),1e-12))
  turn=Rotation.align_vectors([axis],[model.orientation('cigar')[:,2]])[0].as_rotvec();turn*=min(.5,np.radians(2)/max(np.linalg.norm(turn),1e-12))
  handR=Rotation.from_rotvec(turn).as_matrix()@hands['left']['rotation'];point=hands['left']['tcp']+delta
  q,pe,re=ik.solve(point,ik.matrix_to_quat_xyzw(handR),current[:6],base_pos=station.BASES['left'],iters=60)
  result=wanted.copy();result[:6]=q;return result
