"""Persistent native forces at body centres of mass, applied each physics step."""
import numpy as np
class NativeWrench:
 def __init__(self,model,names):
  import torch
  self.asset=model.base.scene['materials'];self.names=list(names)
  available=list(self.asset.body_names);self.ids=[available.index(n) for n in self.names]
  self.force=torch.zeros((1,len(names),3),dtype=torch.float32,device=self.asset.device)
  self.torque=torch.zeros_like(self.force)
  import warp as wp
  self.force_view=wp.from_torch(self.force,dtype=wp.vec3)
  self.torque_view=wp.from_torch(self.torque,dtype=wp.vec3)
 def set(self,forces):
  import torch
  v=np.asarray(forces,dtype=np.float32);assert v.shape==(len(self.names),3) and np.isfinite(v).all()
  self.force.copy_(torch.as_tensor(v[None],device=self.force.device))
  self.asset.permanent_wrench_composer.set_forces_and_torques_index(forces=self.force_view,torques=self.torque_view,body_ids=self.ids,is_global=True)
