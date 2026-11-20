"""Measured geometry -> occluded, power-limited local convection.

Coarse capsules approximate locks. This is not CFD and ignores humidity and
inter-fibre capillary transport. All exposure comes from live body geometry.
"""
import math
import numpy as np
from config import *
from moisture import Patch

def add(a,b):return tuple(x+y for x,y in zip(a,b))
def sub(a,b):return tuple(x-y for x,y in zip(a,b))
def mul(a,s):return tuple(x*s for x in a)
def dot(a,b):return sum(x*y for x,y in zip(a,b))
def norm(a):return math.sqrt(dot(a,a))
def column(r,i):return tuple(row[i] for row in r)
def segment_distance(a,b,c,d):
 # Closest distance between finite segments, including parallel/degenerate cases.
 u=sub(b,a);v=sub(d,c);w=sub(a,c);aa=dot(u,u);bb=dot(u,v);cc=dot(v,v);dd=dot(u,w);ee=dot(v,w)
 if aa<1e-15 and cc<1e-15:return norm(w)
 if aa<1e-15:s=0.;t=max(0.,min(1.,ee/cc))
 elif cc<1e-15:t=0.;s=max(0.,min(1.,-dd/aa))
 else:
  den=aa*cc-bb*bb;s=max(0.,min(1.,(bb*ee-cc*dd)/den)) if den>1e-15 else 0.
  t=(bb*s+ee)/cc
  if t<0.:t=0.;s=max(0.,min(1.,-dd/aa))
  elif t>1.:t=1.;s=max(0.,min(1.,(bb-dd)/aa))
 return norm(sub(add(a,mul(u,s)),add(c,mul(v,t))))


def ray_segment_distances(a,b,starts,ends):
 """Exact batched counterpart of segment_distance for one ray, live capsules.

 Geometry is rebuilt every step; no frozen hair/occlusion cache is used.
 The degenerate/parallel/clamped cases follow the scalar reference exactly.
 """
 a=np.asarray(a,dtype=float);b=np.asarray(b,dtype=float)
 starts=np.asarray(starts,dtype=float);ends=np.asarray(ends,dtype=float)
 u=b-a;v=ends-starts;w=a-starts
 aa=float(u@u);bb=v@u;cc=np.einsum('ij,ij->i',v,v)
 dd=w@u;ee=np.einsum('ij,ij->i',v,w)
 s=np.zeros(len(starts));t=np.zeros(len(starts));valid=cc>=1e-15
 if aa<1e-15:
  t[valid]=np.clip(ee[valid]/cc[valid],0.,1.)
 else:
  s[~valid]=np.clip(-dd[~valid]/aa,0.,1.)
  den=aa*cc-bb*bb;nonparallel=valid&(den>1e-15)
  s[nonparallel]=np.clip((bb[nonparallel]*ee[nonparallel]-cc[nonparallel]*dd[nonparallel])/den[nonparallel],0.,1.)
  t[valid]=(bb[valid]*s[valid]+ee[valid])/cc[valid]
  low=valid&(t<0.);high=valid&(t>1.)
  t[low]=0.;s[low]=np.clip(-dd[low]/aa,0.,1.)
  t[high]=1.;s[high]=np.clip((bb[high]-dd[high])/aa,0.,1.)
 difference=w+s[:,None]*u-t[:,None]*v
 return np.sqrt(np.einsum('ij,ij->i',difference,difference))

class HairThermal:
 def __init__(self):
  area=2*math.pi*RADIUS*LENGTH/PATCHES_PER_SEGMENT
  self.patches=[Patch(area,DRY_MASS_PER_SEGMENT*1400/PATCHES_PER_SEGMENT,WATER_PER_PATCH) for _ in range(LOCKS*SEGMENTS*PATCHES_PER_SEGMENT)]
  self.max_temperature=20.;self.energy=0.
 def step(self,dt,segments,nozzle,axis,power,head_center=HEAD,patch_samples=False):
  if len(segments)!=LOCKS*SEGMENTS:raise ValueError('missing native hair segments')
  if not math.isfinite(power) or not 0<=power<=1:raise ValueError('invalid measured power slider')
  nn=norm(axis)
  if nn<.999 or nn>1.001:raise ValueError('dryer axis must be unit length')
  capsules=[(add(s['pos'],mul(column(s['rotation'],2),-LENGTH/2+RADIUS)),add(s['pos'],mul(column(s['rotation'],2),LENGTH/2-RADIUS))) for s in segments]
  capsule_starts=np.asarray([a for a,b in capsules]);capsule_ends=np.asarray([b for a,b in capsules])
  scalp_clearance=min(segment_distance(a,b,head_center,head_center)-RADIUS-HEAD_RADIUS for a,b in capsules)
  flux=[];wind=[0. for _ in segments];patch_geometry=[]
  for i,s in enumerate(segments):
   r=s['rotation'];normals=[column(r,0),mul(column(r,0),-1),column(r,1),mul(column(r,1),-1)]
   for quadrant,normal in enumerate(normals):
    point=add(s['pos'],mul(normal,RADIUS));ray=sub(point,nozzle);distance=norm(ray)
    h=0.;exposed=0.;occluder=None;incidence=0.
    if power>.5 and .08<=distance<=.30 and dot(ray,axis)/distance>=math.cos(math.radians(22.)):
     exposed=max(0.,dot(normal,mul(ray,-1/distance)));incidence=exposed
     # Test actual scalp sphere and every other measured capsule.
     if segment_distance(nozzle,point,head_center,head_center)<HEAD_RADIUS:exposed=0.;occluder='scalp'
     if exposed>0:
      end=add(point,mul(ray,-.0002/distance))
      blocked=ray_segment_distances(nozzle,end,capsule_starts,capsule_ends)<RADIUS-.0001
      blocked[i]=False
      indices=np.flatnonzero(blocked)
      if len(indices):
       k=int(indices[0]);exposed=0.;occluder='hair_'+str(k//SEGMENTS)+'_'+str(k%SEGMENTS)
     h=150.*(.15/distance)**2*power
    flux.append((h,exposed))
    if patch_samples:
     patch_geometry.append({'patch_index':i*PATCHES_PER_SEGMENT+quadrant,'lock':i//SEGMENTS,'segment':i%SEGMENTS,
                            'quadrant':quadrant,'point_m':list(point),'normal':list(normal),'distance_m':distance,
                            'incidence_fraction':incidence,'exposed_fraction':exposed,'occluder':occluder,
                            'h_W_m2_K':h,'scalp_surface_clearance_m':norm(sub(point,head_center))-HEAD_RADIUS})
    # Coarse pressure drag from this exposed surface quadrant.
    if exposed>0:wind[i]+=.5*1.2*1.1*(2*RADIUS*LENGTH/4)*(12.*(.15/distance)*power)**2*exposed
  heat=sum(h*p.area*f*max(0.,AIR_TEMPERATURE-p.temperature) for p,(h,f) in zip(self.patches,flux))
  scale=min(1.,HEATER_LIMIT/max(1e-12,heat));self.energy+=heat*scale*dt
  removed=0.
  for p,(h,f) in zip(self.patches,flux):removed+=p.step(dt,AIR_TEMPERATURE,h*scale,f)
  self.max_temperature=max(self.max_temperature,max(p.temperature for p in self.patches))
  wind_scale=min(1.,.25/max(1e-12,sum(wind)))
  forces=[list(mul(axis,f*wind_scale)) for f in wind]
  result=self.result()|{'wind_forces_N':forces,'water_removed_step_kg':removed,'convective_power_W':heat*scale,
                        'exposed_patches':sum(f>0 for _,f in flux),'min_scalp_capsule_clearance_m':scalp_clearance}
  if patch_samples:
   for sample,p in zip(patch_geometry,self.patches):
    sample.update({'temperature_C':p.temperature,'water_kg':p.water,'remaining_water_fraction':p.water/WATER_PER_PATCH,
                   'applied_h_W_m2_K':sample['h_W_m2_K']*scale})
   result['patch_samples']=patch_geometry
  return result
 def result(self):
  fractions=[p.water/WATER_PER_PATCH for p in self.patches]
  return {'dry_fraction':sum(f<=DRY_FRACTION for f in fractions)/len(fractions),'worst_remaining_water_fraction':max(fractions),'water_kg':sum(p.water for p in self.patches),'max_temperature_C':self.max_temperature,'delivered_energy_J':self.energy}
