"""Final four-corner wrap and two mechanical clamps; no arm strategy scoring."""
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation
ACTIVE=None

def retaining_overlap(pose, source_sign, end):
 """Length of the retaining edge behind a core ear, in metres.

 Transform the actual nose edge, not the nominal clamp origin. Clip it against
 the ear's inner half-space, longitudinal span and height. A recovered spring
 alone cannot establish retention when the whole bar is shifted off the ear.
 """
 from . import geometry as G
 edge=np.array([[-source_sign*.0095,end*.018,-.006],
                [-source_sign*.0095,end*.018,.006]])
 local=(Rotation.from_quat(pose[3:]).apply(edge)+pose[:3]-G.ORIGIN)@G.R
 # Both sides and both ends are equivalent; use the actual target quadrant.
 side=1 if local[:,0].mean()>=0 else -1
 longitudinal=1 if local[:,1].mean()>=0 else -1
 local[:,0]*=side;local[:,1]*=longitudinal
 low=np.array([.020,.012,-.0145]);high=np.array([.025,.019,-.0025])
 return segment_in_box(local,low,high)

def segment_in_box(local,low,high):
 """Length of a physical edge inside a mating region, in metres."""
 start,direction=local[0],local[1]-local[0];enter,leave=0.,1.
 for axis in range(3):
  if abs(direction[axis])<1e-12:
   if not low[axis]<start[axis]<high[axis]:return 0.
  else:
   a,b=sorted(((low[axis]-start[axis])/direction[axis],(high[axis]-start[axis])/direction[axis]))
   enter=max(enter,a);leave=min(leave,b)
 return max(0.,leave-enter)*float(np.linalg.norm(direction))

class Outcome:
 def __init__(self,paths,geometry,contacts):
  self.paths=paths;self.g=geometry;self.c=contacts;self.ids={p.rsplit('/',1)[-1]:i for i,p in enumerate(paths)};self.value=0.;self.history=deque(maxlen=3*geometry['physics_hz']+1);self.detail={};self.unsafe=False
 def update(self,p,robot,contacts,environment):
  from . import geometry as G
  if not np.isfinite(p).all():raise RuntimeError('Invalid mechanism state')
  self.unsafe|=bool(np.sum(environment[:,:18]) or np.sum(environment[:,-1]))
  pad=p[self.ids['foot_pad']];d=G.R.T@(pad[:3]-G.ORIGIN)
  half_turn=Rotation.from_euler('z',180,degrees=True).as_matrix()
  angular=min(float((Rotation.from_quat(pad[3:])*Rotation.from_matrix(G.R@sym).inv()).magnitude()) for sym in (np.eye(3),half_turn))
  hooks=[];hook_overlap=[]
  for i,(sign,y) in enumerate(G.FLAPS):
   pose=p[self.ids[f'foot_flap{i}']]
   edge=np.array([[sign*.0255,y-.007,-.0145],[sign*.0255,y+.007,-.0145]])
   local=(Rotation.from_quat(pose[3:]).apply(edge)+pose[:3]-G.ORIGIN)@G.R
   side=1 if local[:,0].mean()>=0 else -1
   end=1 if local[:,1].mean()>=0 else -1
   local[:,0]*=side;local[:,1]*=end
   # Actual ear underside and 6mm-deep hook. The 50um upper allowance
   # is numerical contact precision, not a physical measurement requirement.
   overlap=segment_in_box(local,np.array([.025,.012,-.0205]),np.array([.027,.019,-.01445]))
   hook_overlap.append(float(overlap*1000));hooks.append(bool(overlap>=.001))
  pad_supported=bool(np.sum(contacts[[self.ids['foot_pad']]+[self.ids[f'foot_flap{i}'] for i in range(4)],len(p):-1])>0)
  core_top=np.array([[x,y,-.0025] for x in (-.025,.025) for y in (-.016,.016)])
  in_cover=Rotation.from_quat(pad[3:]).inv().apply(core_top@G.R.T+G.ORIGIN-pad[:3])
  # The guide skirts physically enclose the core. These are mesh dimensions,
  # not arbitrary world-pose targets. 50um is the native contact allowance.
  guide_clearance=min(.0253-float(np.max(np.abs(in_cover[:,0]))),.0163-float(np.max(np.abs(in_cover[:,1]))),-.0025-float(np.max(in_cover[:,2])))
  guide_fit=guide_clearance>=-.00005-1e-12
  wrap=bool(pad_supported and all(hooks) and guide_fit)
  seated=[];errors=[];clip_recovered=[];engagement=[];assigned=[]
  for side in ['left','right']:
   idx=self.ids['foot_'+side+'_clamp'];pose=p[idx]
   target_side=min(G.CLAMP_TARGET,key=lambda key:np.linalg.norm(pose[:3]-G.CLAMP_TARGET[key]));assigned.append(target_side)
   err=np.linalg.norm(pose[:3]-G.CLAMP_TARGET[target_side])
   expected=G.R if target_side==side else G.R@half_turn
   angle=float((Rotation.from_quat(pose[3:])*Rotation.from_matrix(expected).inv()).magnitude())
   members=[idx,self.ids['foot_'+side+'_end0'],self.ids['foot_'+side+'_end1']]
   supported=bool(np.sum(contacts[members,len(p):-1])+np.sum(contacts[np.ix_(members,[self.ids['foot_pad']]+[self.ids[f'foot_flap{i}'] for i in range(4)])])>0)
   rc=Rotation.from_quat(pose[3:]);recovered=True;overlaps=[]
   for k in (0,1):
    ep=p[self.ids[f'foot_{side}_end{k}']];rel=rc.inv().apply(ep[:3]-pose[:3]);er=(rc.inv()*Rotation.from_quat(ep[3:])).magnitude()
    recovered &= bool(abs(rel[1])<.0005 and np.linalg.norm(rel[[0,2]])<.0002 and er<np.deg2rad(.3))
    overlaps.append(retaining_overlap(ep,1 if side=='left' else -1,-1 if k==0 else 1))
   clip_recovered.append(recovered)
   engagement.append([float(v*1000) for v in overlaps])
   seated.append(bool(supported and min(overlaps)>=.001));errors.append(float(err*1000))
  if len(set(assigned))!=2:seated=[False,False]
  released=bool(np.sum(robot)==0)
  valid=wrap and all(seated) and released and not self.unsafe
  measured=[self.ids['foot_pad'],self.ids['foot_left_clamp'],self.ids['foot_right_clamp']]
  self.history.append((p[measured].copy(),valid));stable=len(self.history)==self.history.maxlen and all(h[1] for h in self.history)
  drift=999.
  if stable:
   hist=np.array([h[0] for h in self.history]);drift=float(np.linalg.norm(hist[:,:,:3]-hist[0,:,:3],axis=-1).max());ang=max(float((Rotation.from_quat(hist[:,j,3:])*Rotation.from_quat(hist[0,j,3:]).inv()).magnitude().max()) for j in range(3))
   # Drift is diagnostic only. Retaining geometry, support and release must
   # persist; harmless motion within the installed fit does not lose credit.
  self.value=0. if self.unsafe else 1. if stable else .75 if wrap and all(seated) else .5 if wrap and any(seated) else .25 if wrap else 0.
  self.detail=dict(score=self.value,wrap=wrap,pad_supported=pad_supported,hooks=hooks,hook_overlap_mm=hook_overlap,guide_fit=guide_fit,guide_clearance_mm=guide_clearance*1000,clamps_seated=seated,clip_recovered=clip_recovered,retaining_overlap_mm=engagement,clamp_target_sides=assigned,clamp_error_mm=errors,pad_delta_mm=(d*1000).tolist(),pad_angle_deg=float(np.rad2deg(angular)),released=released,stable_3s=bool(stable),unsafe=self.unsafe,drift_mm=drift*1000)
