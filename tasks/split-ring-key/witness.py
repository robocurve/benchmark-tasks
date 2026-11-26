"""Fixed native witness camera; no tracked target, no policy/controller input."""
import numpy as np
EYE=np.array([.18,-.125,1.16])
TARGET=np.array([.40,.008,.992])
INSPECTION={
 'inspection_front':(np.array([.18,0.,.9875]),np.array([.40,0.,.9875]),125.,960,960),
 'inspection_edge':(np.array([.395,.115,1.007]),np.array([.40,.012,.992]),100.,960,540),
}

def view_rotation(eye,target):
 from scipy.spatial.transform import Rotation
 z=eye-target;z/=np.linalg.norm(z);x=np.cross([0.,0,1.],z);x/=np.linalg.norm(x);y=np.cross(z,x)
 return Rotation.from_matrix(np.stack([x,y,z],1)).as_quat()

def camera_cfg(name='witness_cam'):
 from isaaclab.sensors import CameraCfg
 import isaaclab.sim as sim_utils
 from yam_arena.embodiment import RENDERER
 eye,target,focal,width,height=INSPECTION.get(name,(EYE,TARGET,75.,960,540))
 rot=view_rotation(eye,target)
 return CameraCfg(prim_path='{ENV_REGEX_NS}/'+name,height=height,width=width,data_types=['rgb'],
  spawn=sim_utils.PinholeCameraCfg(focal_length=focal,horizontal_aperture=36.,vertical_aperture=36.*height/width,clipping_range=(.005,10.)),
  offset=CameraCfg.OffsetCfg(pos=tuple(map(float,eye)),rot=tuple(map(float,rot)),convention='opengl'),renderer_cfg=RENDERER)

class WitnessCamera:
 def __init__(self,base):
  self.base=base
  self.camera=base.scene['witness_cam']
  for _ in range(3):base.sim.render()
 def read(self):
  return self._read(self.camera)
 def _read(self,camera):
  camera.update(0.,force_recompute=True)
  value=camera.data.output['rgb']
  value=value.torch if hasattr(value,'torch') else value
  a=value[0,:,:,:3].detach().cpu().numpy()
  return np.ascontiguousarray(a,dtype=np.uint8)
 def read_all(self):
  return {'witness_cam':self.read(),**{name:self._read(self.base.scene[name]) for name in INSPECTION}}
 def setup_key_detail(self):
  """A second declared setup view; restore fixed ring view before actions."""
  eye=np.array([.29,-.28,1.17]);target=np.array([.355,-.140,.9675])
  self.camera.set_world_poses(eye[None],view_rotation(eye,target)[None],convention='opengl')
  for _ in range(3):self.base.sim.render()
  frame=self.read()
  self.camera.set_world_poses(EYE[None],view_rotation(EYE,TARGET)[None],convention='opengl')
  for _ in range(3):self.base.sim.render()
  return frame
