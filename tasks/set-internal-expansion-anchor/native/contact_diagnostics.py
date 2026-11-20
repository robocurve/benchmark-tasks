"""Read-only finite native contact pairs. Unavailable is unknown, neverzero."""
import numpy as np
class Diagnostics:
 def __init__(self,rt):self.rt=rt;self.reset()
 def reset(self):self.parent=None;self.views={};self.paths={};self.errors={}
 def install(self,base):
  from isaaclab_physx.physics import PhysxManager
  from pxr import Usd,UsdPhysics
  self.parent=PhysxManager.get_physics_sim_view();root=base.scene.env_prim_paths[0]
  rigid=[str(p.GetPath())for p in base.scene.stage.Traverse(Usd.TraverseInstanceProxies())if p.HasAPI(UsdPhysics.RigidBodyAPI)]
  pairs=[('punch_wedge','punch','setting_wedge'),('punch_ring','punch','anchor_ring'),('hammer_punch','hammer','punch'),('ring_concrete','anchor_ring','concrete')]
  for k in range(4):pairs.extend([(f'petal{k}_wedge',f'petal_{k}','setting_wedge'),(f'petal{k}_concrete',f'petal_{k}','concrete')])
  for key,s,p in pairs:self.paths[key]=dict(sensor=root+'/'+s+'/body',partner=root+'/'+p+'/body')
  for side,arm,tar in [('left','LeftArm','punch'),('right','RightArm','hammer')]:
   for p in rigid:
    if p.startswith(root+'/'+arm+'/')and'tip_'in p:self.paths[side+'_'+p.rsplit('/',1)[-1]]=dict(sensor=root+'/'+tar+'/body',partner=p)
  for key,p in self.paths.items():
   try:
    v=self.parent.create_rigid_contact_view([p['sensor']],filter_patterns=[[p['partner']]],max_contact_data_count=256)
    if v is None:raise RuntimeError('Unbound actual contactview')
    v.get_contact_data(dt=float(base.physics_dt));self.views[key]=v
   except Exception as e:self.errors[key]=type(e).__name__+':'+str(e)
  self.rt.save('CONTACT-CONTRACT.json',dict(actual_pairs=self.paths,errors=self.errors,limitation='Raw installed API sample at last physics substep of20Hzcontrol; no force calibration claim. All missing/unbound pairs unknown.'))
 def observe(self,base):
  if self.parent is None:self.install(base)
  out={}
  for key,v in self.views.items():
   try:
    norm,point,direction,separation,count,start=[self.rt.arr(x)for x in v.get_contact_data(dt=float(base.physics_dt))];entries=[]
    for ix in np.ndindex(count.shape):
     for j in range(int(start[ix]),int(start[ix])+int(count[ix])):
      entries.append(dict(normal_force_N=float(np.asarray(norm[j]).reshape(-1)[0]),point_world_m=point[j].tolist(),normal_world=direction[j].tolist(),separation_m=np.asarray(separation[j]).tolist()))
    out[key]=dict(counts=count.tolist(),entries=entries)
   except Exception as e:self.errors[key]=str(e);out[key]=dict(unknown=str(e))
  return dict(pairs=out,unavailable_views=self.errors)
