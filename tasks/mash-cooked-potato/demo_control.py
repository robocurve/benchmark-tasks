"""Use observed tool and food poses to guide ordinary robot-joint actions."""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from geometry import BODIES,JOINTS,BOWL
from common import down,TABLE

class DemoControl:
 def __init__(self):
  self.free=None;self.stage=None;self.offset=np.zeros(3);self.previous_offset=np.zeros(3);self.stage_start=0
  self.R0=Rotation.from_euler('x',45,degrees=True).as_matrix()@down(0)
  grip=np.array([.34,-.17,TABLE+.140])-.008*self.R0[:,2]
  self.local_head=self.R0.T@(np.array([.34,-.17,TABLE+.003])-grip)
  import json
  from pathlib import Path
  self.ends={f["stage"]:f["action14"] for f in json.loads(Path(__file__).with_name("demo-plan.json").read_text())["frames"]}
 def next_press(self,m):
  names=[n for n in m.names if n.startswith('food_')];parent={n:n for n in names}
  def root(n):
   while parent[n]!=n:n=parent[n]
   return n
  for j in JOINTS:
   if j['name'] not in m.broken and j['name'] not in m.disabled:parent[root(j['a'])]=root(j['b'])
  groups={}
  for n in names:groups.setdefault(root(n),[]).append(n)
  choices=[]
  for ns in groups.values():
   pts=np.array([m.p(n) for n in ns]);centre=pts.mean(axis=0);r=np.linalg.norm(centre[:2]-m.p('bowl')[:2])
   size=np.max(np.linalg.norm(pts[:,None]-pts[None,:],axis=-1))+.007
   if size>.009 and r<.09:choices.append((len(ns),size,centre))
  remaining=[n for ns in groups.values() if len(ns)>1 for n in ns if np.linalg.norm(m.p(n)[:2]-m.p('bowl')[:2])<.09]
  if remaining:
   pts=np.array([m.p(n) for n in remaining]);density=(np.linalg.norm(pts[:,None,:2]-pts[None,:,:2],axis=-1)<.027).sum(axis=1);target=pts[int(np.argmax(density))]
  else:target=m.p('bowl')
  delta=target[:2]-m.p('bowl')[:2];delta*=min(1.,.055/max(np.linalg.norm(delta),1e-6));return m.p('bowl')[:2]+delta
 def action(self,k,wanted,stage,m,hands,observed):
  if k<=200:return wanted
  # Compensate tool slip through ordinary hand motion; never reposition the tool.
  from yam_arena import ik,station
  h=hands['right']
  if stage in ['release masher','withdraw','released outcome']:
   if self.free is None:self.free=(h['tcp'].copy(),h['rotation'].copy(),k)
   pp,RR,start=self.free;target=pp.copy()
   if stage!='release masher':target[2]+=.09*np.clip((k-start-20)/40,0,1)
   seed=np.clip(observed[7:13],ik.LOWER+1e-6,ik.UPPER-1e-6)
   def residual(q):
    p,R=ik.tcp_pose(q,station.BASES['right']);return np.r_[p-target,.12*ik._rot_error(RR,R),.001*(q-seed)]
   out=wanted.copy();out[7:13]=least_squares(residual,seed,bounds=(np.maximum(ik.LOWER,seed-.12),np.minimum(ik.UPPER,seed+.12)),max_nfev=30).x;return out
  p,R=ik.tcp_pose(wanted[7:13],station.BASES['right']);head=p+R@self.local_head
  if stage!=self.stage:
   self.stage=stage;self.stage_start=k;self.previous_offset=self.offset.copy()
   if stage.startswith('position over potato'):
    ep,eR=ik.tcp_pose(np.asarray(self.ends[stage])[7:13],station.BASES['right']);end_head=ep+eR@self.local_head
    self.offset=np.r_[self.next_press(m)-end_head[:2],0.]
   elif stage=='return masher':self.offset=np.zeros(3)
  if stage.startswith('position over potato') or stage=='return masher':
   u=np.clip((k-self.stage_start+1)/20,0,1);u=u*u*(3-2*u);offset=(1-u)*self.previous_offset+u*self.offset
  else:offset=self.offset
  target=head+offset;desired=R@self.R0.T
  relative_head=h['rotation'].T@(m.p('masher')-h['tcp']);relative_R=h['rotation'].T@m.orientation('masher')
  seed=np.clip(observed[7:13],ik.LOWER+1e-6,ik.UPPER-1e-6)
  def residual(q):
   pp,rr=ik.tcp_pose(q,station.BASES['right']);return np.r_[pp+rr@relative_head-target,.12*ik._rot_error(desired,rr@relative_R),.001*(q-seed)]
  out=wanted.copy();out[7:13]=least_squares(residual,seed,bounds=(np.maximum(ik.LOWER,seed-.12),np.minimum(ik.UPPER,seed+.12)),max_nfev=30).x;return out
