"""Pose-feedback feasibility demo: ordinary arm targets, no object pose writes.

Native clasp poses are privileged demonstration feedback, never Dream input.
Both arms stop advancing when the measured pin reaches the catch depth.
"""
import numpy as np
from scipy.spatial.transform import Rotation
class DemoAlignment:
 def __init__(self):self.last=None;self.closed=None;self.target=None;self.orientation=None;self.withdraw_delta=None;self.withdraw_count=0;self.settle_count=0;self.settle_targets=None
 def action(self,wanted,observed,hands,model,stage):
  active=stage in ('align pin and socket','push pin into socket','seat the aligned pin','hold the closed clasp')
  if active and self.closed is None and self.last is not None and .0025<=model.entered<=.0035:
   self.closed=self.last.copy()
  if active and self.closed is not None:return self.closed.copy()
  if active:
   from yam_arena import ik,station
   hand=hands['right'];f,m=model.points[[0,-1]];a,b=model.axes[[0,-1]]
   a=a/np.linalg.norm(a);b=b/np.linalg.norm(b)
   v=np.cross(b,a);norm=np.linalg.norm(v);angle=float(np.arctan2(norm,np.clip(b@a,-1,1)))
   Q=np.eye(3) if norm<1e-9 else Rotation.from_rotvec(v/norm*min(.2*angle,np.radians(.5))).as_matrix()
   distance=float((f-m)@a);lateral=(f-m)-a*distance
   # Align without closing the separation; approach only when aligned.
   if stage=='align pin and socket':separation=distance
   elif np.linalg.norm(lateral)>.0005 or angle>np.radians(5):separation=distance
   else:separation=.0132
   correction=.2*(f-a*separation-m);length=np.linalg.norm(correction)
   if length>.001:correction*=.001/length
   if self.target is None:self.target=np.asarray(hand['tcp']).copy();self.orientation=hand['rotation'].copy()
   target=m+correction+Q@(self.target-m);R=Q@self.orientation
   self.target=target.copy();self.orientation=R.copy()
   q,pe,re=ik.solve(target,ik.matrix_to_quat_xyzw(R),np.asarray(observed)[7:13],base_pos=station.BASES['right'],iters=80)
   if pe>.002 or re>.05:raise RuntimeError('Demo insertion target exceeds reachable pose tolerance')
   out=np.array(wanted,copy=True);out[7:13]=q;self.last=out.copy();return out
  if stage=='lower the clasp onto the wrist' and self.last is not None:
   from yam_arena import ik,station
   if self.settle_targets is None:
    base=(self.closed if self.closed is not None else self.last).copy()
    self.settle_targets={s:ik.tcp_pose(base[o:o+6],station.BASES[s]) for s,o in [('left',0),('right',7)]}
   self.settle_count+=1;u=min(self.settle_count/40.,1.);u=u*u*(3-2*u)
   out=(self.closed if self.closed is not None else self.last).copy()
   for side,offset in [('left',0),('right',7)]:
    tcp,R=self.settle_targets[side];target=np.asarray(tcp)+[0.,0.,-.012*u]
    q,pe,re=ik.solve(target,ik.matrix_to_quat_xyzw(R),out[offset:offset+6],base_pos=station.BASES[side],iters=80)
    if pe>.002 or re>.05:raise RuntimeError('Demo lowering exceeds reachable pose tolerance')
    out[offset:offset+6]=q
   self.closed=out.copy();self.last=out.copy();return out
  if stage=='release clasp' and self.last is not None:
   out=(self.closed if self.closed is not None else self.last).copy();out[[6,13]]=np.asarray(wanted)[[6,13]];return out
  if stage=='withdraw' and self.last is not None:
   if self.withdraw_delta is None:self.withdraw_delta=(self.closed if self.closed is not None else self.last)-np.asarray(wanted)
   self.withdraw_count+=1;u=min(self.withdraw_count/40.,1.);u=u*u*(3-2*u)
   out=np.asarray(wanted)+self.withdraw_delta*(1-u);out[[6,13]]=np.asarray(wanted)[[6,13]];return out
  return wanted
