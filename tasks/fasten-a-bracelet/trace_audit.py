"""Independent scoring of measured native bodies, not a re-simulation of physics."""
import numpy as np
from scipy.spatial.transform import Rotation
from bracelet_geometry import ANCHOR0,ANCHOR1

def audit(rows,report):
 latched=False;failed=False;release=0.;count=0;events=[];last=None
 for row in rows:
  assert row['step']==count,'Non-contiguous trace'
  command=np.asarray(row['action14'],float)
  assert command.shape==(14,) and np.isfinite(command).all()
  assert np.all((command[[6,13]]>=0)&(command[[6,13]]<=1))
  pose=np.asarray(row['state']['poses'],float);velocity=np.asarray(row['state']['velocities'],float)
  assert pose.shape==(41,7) and velocity.shape==(41,6)
  assert np.isfinite(pose).all() and np.isfinite(velocity).all()
  p=pose[:,:3];r=Rotation.from_quat(pose[:,3:]).as_matrix();axis=-r[:,:,0]
  gap=np.max(np.linalg.norm(p[:-1]+np.einsum('nij,nj->ni',r[:-1],ANCHOR0)-(p[1:]+np.einsum('nij,nj->ni',r[1:],ANCHOR1)),axis=1))
  rel=p[-1]+.011*axis[-1]-(p[0]-.005*axis[0]);depth=float(rel@axis[0])
  radial=float(np.linalg.norm(np.cross(rel,axis[0])));angle=float(np.degrees(np.arccos(np.clip(axis[0]@axis[-1],-1,1))))
  inserted=depth if radial<=.00055 and angle<=8 else 0.
  if count:
   if not latched and .0025<=inserted<=.0035:
    latched=True;events.append(dict(step=count,depth_mm=inserted*1000,radial_mm=radial*1000))
   failed |= bool(row['state'].get('native_catch_broken',False)) or gap>.004 or (latched and (depth<.001 or depth>.004 or radial>.0015 or angle>15))
   clear=all(h['opening']>.5 and min(np.linalg.norm(p-np.asarray(h['tcp']),axis=1))>.035 for h in row['hands'].values())
   release=release+.05 if clear else 0.
  a=np.arctan2(p[:,2]-.80,p[:,1]);turn=abs(np.sum(np.angle(np.exp(1j*np.diff(np.r_[a,a[0]])))))/(2*np.pi)
  around=bool(turn>.8);axial=float(max(abs(p[:,0]-.34))*1000)
  score=0. if failed or not around or axial>50 else (1. if release>=3 else .5) if latched else .25 if inserted>=.001 else 0.
  m=row['measurements']
  for k,v in [('clasp_closed',latched),('around_wrist',around),('failed',bool(failed))]:assert m[k]==v,(count,k,m[k],v)
  for k,v in [('inserted_mm',max(0,inserted)*1000),('radial_error_mm',radial*1000),('axis_error_deg',angle),('axial_offset_mm',axial),('released_seconds',release),('max_joint_gap_mm',gap*1000)]:
   if k=='axis_error_deg':
    # Native float32 quaternions differ slightly from SciPy's unit-normalized
    # rotations. Compare cosines near zero, where acos is ill-conditioned.
    agrees=np.isclose(np.cos(np.radians(m[k])),np.cos(np.radians(v)),atol=1e-6,rtol=0.)
   else:agrees=np.isclose(m[k],v,atol=2e-4,rtol=2e-4)
   assert agrees,(count,k,m[k],v)
  assert row['state']['latched']==latched
  last=m;count+=1
 assert count-1==report['steps'] and score==report['score']
 return dict(steps=count-1,score=score,score_agrees=True,native_body_samples=count*41,latch_entries=events,measurements=last,physics_resimulated=False,method='Independent geometry and release scoring from recorded native body poses; native dynamics require Isaac/PhysX.')
