"""Isolated exact guard batching candidate. NOT installed in active coupon.

All315pieces×15OBBs are evaluated. Same narrowphase function, same threshold,
exceptions, min-pair traversal order and finite checks as frozen baseline.
Only independent far-box reductions are batched. Right FK cache requires exact
joint/finger VALUES; panel/cutter transforms always come from each fresh call.
"""
import numpy as np
from baseline_safety import ScoopSafety as Baseline,convex_box_gap
from yam_arena.urdf import quat_xyzw_to_matrix
from config import TOP

class ScoopSafety(Baseline):
 def __init__(self):
  super().__init__()
  self.counts=np.array([len(t.reshape(-1,3))for _,t in self.raw],dtype=np.int64)
  self.starts=np.r_[0,np.cumsum(self.counts)[:-1]]
  self.names=[n for n,_ in self.raw]
  self._right_key=None;self._right_bounds=None
  self.stats=dict(checks=0,all_piece_box_pairs=0,narrowphase_calls=0,min_tie_refinements=0,right_fk_builds=0,right_fk_exact_cache_hits=0)
 def _right(self,q,fingers):
  key=(np.asarray(q,dtype=np.float64).tobytes(),np.asarray(fingers,dtype=np.float64).tobytes())
  if key!=self._right_key:
   world=self.world(q,fingers,np.array([0,-.305,.75]));vertices=np.concatenate([t.reshape(-1,3)for _,t in world]);self._right_bounds=(np.minimum.reduceat(vertices,self.starts,axis=0),np.maximum.reduceat(vertices,self.starts,axis=0));self._right_key=key;self.stats['right_fk_builds']+=1
  else:self.stats['right_fk_exact_cache_hits']+=1
  return self._right_bounds
 def _pair_gaps(self,left,vertices,obstacles):
  gaps=np.empty((315,len(obstacles)+1))
  gaps[:,0]=np.minimum.reduceat(vertices[:,2],self.starts)-TOP
  for column,(n,p,R,size)in enumerate(obstacles,1):
   local=(vertices-p)@R;half=np.asarray(size)/2
   lo=np.minimum.reduceat(local,self.starts,axis=0);hi=np.maximum.reduceat(local,self.starts,axis=0)
   bound=np.maximum(lo-half,-half-hi).max(axis=1);gaps[:,column]=bound
   # The tiny dispatch band adds WORK only: the original narrowphase function
   # itself retains its exact >.0003 branch. No geometry/threshold is relaxed.
   for index in np.flatnonzero(bound<=.0003+1e-12):
    gaps[index,column]=convex_box_gap(left[index][1],p,R,size);self.stats['narrowphase_calls']+=1
  self.stats['all_piece_box_pairs']+=315*len(obstacles)
  return gaps
 def _minimum(self,gaps,mask,left,obstacles):
  values=np.where(mask,gaps,np.inf)
  if not np.isfinite(values).any():return 10.,None,None
  approximate=float(values.min())
  # Preserve the baseline's exact first-min pair even for coplanar/tied meshes.
  # Full-matrix transforms may differ by floating rounding; refine all possible
  # minima with the unchanged scalar function, in original piece/pair order.
  near=np.argwhere(mask & (gaps<=approximate+1e-12))
  best=(10.,None,None)
  for index,column in near:
   if column==0:value=float(left[index][1][:,:,2].min()-TOP);label='table'
   else:
    label,p,R,size=obstacles[column-1];value=convex_box_gap(left[index][1],p,R,size);self.stats['min_tie_refinements']+=1
   if value<best[0]:best=(value,self.names[index],label)
  return best
 def check(self,q,fingers,poses,cutter,right_q,right_fingers,*,contact=False):
  vectors=[q,fingers,cutter,right_q,right_fingers,*poses.values()]
  if not all(np.isfinite(v).all()for v in vectors):raise ValueError('nonfinite native scoop geometry')
  obs={n:(np.asarray(v[:3]),quat_xyzw_to_matrix(np.asarray(v[3:])),self.defs[n]['size'])for n,v in poses.items()}
  cr=quat_xyzw_to_matrix(np.asarray(cutter[3:]));cp=np.asarray(cutter[:3])
  for name,size,off in [('cutter_handle',(.1,.025,.025),(0,0,0)),('cutter_guard',(.02,.03,.02),(.052,0,.002)),('cutter_blade',(.014,.0005,.008),(.062,0,-.016))]:obs[name]=(cp+cr@off,cr,size)
  obstacles=[(n,p,R,size)for n,(p,R,size)in obs.items()]
  left=self.world(q,fingers,np.array([0,.305,.75]));vertices=np.concatenate([t.reshape(-1,3)for _,t in left]);gaps=self._pair_gaps(left,vertices,obstacles)
  intended_mask=np.zeros_like(gaps,dtype=bool)
  if contact:
   column=next((i+1 for i,(n,_,_,_)in enumerate(obstacles)if n=='north_bottom'),None)
   if column is not None:intended_mask[:,column]=np.array([n=='tip_right'for n in self.names])
  hard=self._minimum(gaps,~intended_mask,left,obstacles);intended=self._minimum(gaps,intended_mask,left,obstacles)if contact else(10.,None,None)
  lo=np.minimum.reduceat(vertices,self.starts,axis=0);hi=np.maximum.reduceat(vertices,self.starts,axis=0);rlo,rhi=self._right(right_q,right_fingers)
  pair_gaps=np.max(np.maximum(lo[:,None,:]-rhi[None,:,:],rlo[None,:,:]-hi[:,None,:]),axis=2);mutual=float(pair_gaps.min());self.stats['checks']+=1
  return dict(hard_gap_m=hard[0],hard_pair=list(hard[1:]),other_arm_aabb_gap_m=mutual,intended_target_gap_m=intended[0]if contact else None,intended_pair=list(intended[1:])if contact else None,safe=bool(hard[0]>=.0002 and mutual>=.0002),scope='actualorcommandFK315pieces,current12panels+cutter+table;onlytip_right/north_bottom contactexception;otherarmpositivepieceAABBcertificate')
