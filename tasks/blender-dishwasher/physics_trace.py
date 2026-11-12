"""Read-only 240 Hz evidence for a fresh replay; never changes physics state."""
import json
from pathlib import Path
from demo_runtime import numpy

class PhysicsTrace:
 def __init__(self,reader,path,step):
  self.reader,self.step=reader,step;self.enabled=False;self.samples=0
  self.file=Path(path).open('w');self.original=reader.base.scene.update
  def update(dt):
   self.original(dt)
   if not self.enabled or dt<=0:return
   base=reader.base
   row=dict(sample=self.samples,control_step=int(step()),dt=float(dt),
    bodies={n:dict(pose=numpy(base.scene[n].data.root_link_pose_w)[0].tolist(),velocity=numpy(base.scene[n].data.root_com_vel_w)[0].tolist()) for n in reader.vertices},
    arms={side:dict(joint_pos=numpy(base.scene[side+'_robot'].data.joint_pos)[0].tolist(),
     net_normal_contact_forces_n=numpy(base.scene[side+'_contacts'].data.net_normal_forces_w)[0].tolist(),
     contact_body_names=list(base.scene[side+'_contacts'].body_names)) for side in ['left','right']})
   self.file.write(json.dumps(row)+'\n');self.samples+=1
  reader.base.scene.update=update
 def close(self):
  self.reader.base.scene.update=self.original;self.file.close()
