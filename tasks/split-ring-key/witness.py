"""Fixed native witness camera; no tracked target, no policy/controller input."""
import numpy as np
EYE=np.array([.18,-.125,1.16])
TARGET=np.array([.40,.008,.992])

def view_rotation(eye,target):
 from scipy.spatial.transform import Rotation
 z=eye-target;z/=np.linalg.norm(z);x=np.cross([0.,0,1.],z);x/=np.linalg.norm(x);y=np.cross(z,x)
 return Rotation.from_matrix(np.stack([x,y,z],1)).as_quat()

def camera_cfg():
 from isaaclab.sensors import CameraCfg
 import isaaclab.sim as sim_utils
 from yam_arena.embodiment import RENDERER
 rot=view_rotation(EYE,TARGET)
 return CameraCfg(prim_path='{ENV_REGEX_NS}/ThreadingWitness',height=540,width=960,data_types=['rgb'],
  spawn=sim_utils.PinholeCameraCfg(focal_length=75.,horizontal_aperture=36.,vertical_aperture=20.25,clipping_range=(.005,10.)),
  offset=CameraCfg.OffsetCfg(pos=tuple(map(float,EYE)),rot=tuple(map(float,rot)),convention='opengl'),renderer_cfg=RENDERER)

class WitnessCamera:
 def __init__(self,base):
  self.base=base
  self.camera=base.scene['witness_cam']
  for _ in range(3):base.sim.render()
 def read(self):
  self.camera.update(0.,force_recompute=True)
  value=self.camera.data.output['rgb']
  value=value.torch if hasattr(value,'torch') else value
  a=value[0,:,:,:3].detach().cpu().numpy()
  return np.ascontiguousarray(a,dtype=np.uint8)
 def setup_key_detail(self):
  """A second declared setup view; restore fixed ring view before actions."""
  eye=np.array([.29,-.28,1.17]);target=np.array([.355,-.140,.9675])
  self.camera.set_world_poses(eye[None],view_rotation(eye,target)[None],convention='opengl')
  for _ in range(3):self.base.sim.render()
  frame=self.read()
  self.camera.set_world_poses(EYE[None],view_rotation(EYE,TARGET)[None],convention='opengl')
  for _ in range(3):self.base.sim.render()
  return frame
