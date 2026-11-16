"""Measured native contact pressure/dwell creates passive breakable adhesion.

No controller bond/unbond API and no pose writes. Native break notification is
not reliable on the pinned backend: separated bonds irreversibly fail closed;
coincident unloaded breakage remains a disclosed native validation limitation.
"""
import numpy as np
from bond_geometry import width_contacts,PATCHES,dot
from model import TAPE_WIDTH


class NativeAdhesion:
 def __init__(self,stage,parent,joint_factory=None):
  self.stage=stage;self.parent=parent;self.dwell={};self.bonds={};self.broken=set();self.joint_factory=joint_factory

 def _joint(self,fn,tn,fa,ta,rf,rt,fraction):
  if self.joint_factory:return self.joint_factory(fn,tn,fa,ta,rf,rt,fraction)
  from pxr import UsdPhysics,Gf
  from yam_arena.ik import matrix_to_quat_xyzw
  q=matrix_to_quat_xyzw(rt.T@rf)
  j=UsdPhysics.FixedJoint.Define(self.stage,self.parent+'/'+fn+'/adhesion_'+tn)
  j.CreateBody0Rel().SetTargets([self.parent+'/'+tn]);j.CreateBody1Rel().SetTargets([self.parent+'/'+fn])
  j.CreateLocalPos0Attr(Gf.Vec3f(*ta));j.CreateLocalPos1Attr(Gf.Vec3f(*fa))
  j.CreateLocalRot0Attr(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])))
  j.CreateBreakForceAttr(2.*fraction);j.CreateBreakTorqueAttr(.008*fraction)
  j.CreateExcludeFromArticulationAttr(True);j.CreateCollisionEnabledAttr(False)
  return j

 def step(self,dt,films,targets,contact_forces):
  if not 0<dt<=1/120:raise ValueError('adhesion requires physics substep telemetry')
  for fn,film in films.items():
   for tn,target in targets.items():
    key=(fn,tn)
    if key in self.broken:continue
    force=np.asarray(contact_forces.get(key,(0.,0.,0.)))
    if not np.isfinite(force).all():raise ValueError('nonfinite native contact force')
    if key not in self.bonds and np.linalg.norm(force)<.5:
     self.dwell[key]=0.;continue
    hits=width_contacts(film,target)
    if key in self.bonds:
     b=self.bonds[key];a=np.array(target['pos'])+np.array(target['rotation'])@b['target_anchor'];p=np.array(film['pos'])+np.array(film['rotation'])@b['film_anchor']
     b['separation']=float(np.linalg.norm(a-p))
     # Never count an adhesive patch that has moved off its physical face.
     b['active_bins']=b['width_bins']&{k for k,_,_ in hits}
     if b['separation']>.001 or b['joint'].GetJointEnabledAttr().Get() is False:
      b['joint'].GetJointEnabledAttr().Set(False);self.broken.add(key);b['active_bins']=set()
     continue
    valid=bool(hits) and dot(force,hits[0][2])>=.5
    self.dwell[key]=self.dwell.get(key,0.)+dt if valid else 0.
    if self.dwell[key]+1e-12<.1:continue
    point=np.mean([p for _,p,_ in hits],axis=0);rf=np.array(film['rotation']);rt=np.array(target['rotation'])
    fa=rf.T@(point-np.array(film['pos']));ta=rt.T@(point-np.array(target['pos']));bins={k for k,_,_ in hits}
    j=self._joint(fn,tn,fa,ta,rf,rt,len(hits)/PATCHES)
    self.bonds[key]={'joint':j,'width_bins':bins,'active_bins':bins.copy(),'target_anchor':ta,'film_anchor':fa,'separation':0.}

 def active_patches(self):
  return {key:b['active_bins'] for key,b in self.bonds.items() if key not in self.broken and b['active_bins']}

 def bonded_width(self,film):
  bins=set()
  for key,active in self.active_patches().items():
   if key[0]==film:bins|=active
  return len(bins)/PATCHES*TAPE_WIDTH
