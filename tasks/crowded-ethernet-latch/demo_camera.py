"""Fixed review cameras, separate from the robot's normal policy cameras."""
NAMES=('review_top','review_left','review_right')
def add_cameras(instance):
 from isaaclab.sensors import CameraCfg
 import isaaclab.sim as sim
 from isaaclab_arena.utils.configclass import make_configclass
 from yam_arena.embodiment import RENDERER
 cfg=instance.scene_config
 fields=[(n,type(getattr(cfg,n)),getattr(cfg,n)) for n in cfg.__class__.__annotations__]
 for name in NAMES:
  c=CameraCfg(prim_path='{ENV_REGEX_NS}/'+name,height=270,width=480,data_types=['rgb'],spawn=sim.PinholeCameraCfg(focal_length=18.,clipping_range=(.03,20.)),renderer_cfg=RENDERER)
  fields.append((name,CameraCfg,c))
 instance.scene_config=make_configclass('WithFixedReviewCamerasCfg',fields)()
def place(base,target):
 import torch
 x,y,z=target
 eyes=[(x-.34,y-.24,z+.23),(x-.14,y+.086,z+.061),(x-.115,y-.085,z+.054)]
 for name,eye in zip(NAMES,eyes):base.scene[name].set_world_poses_from_view(torch.tensor([eye],device=base.device),torch.tensor([target],device=base.device))
def array(v):
 v=v.torch if hasattr(v,'torch') else v
 return v.detach().cpu().numpy()
def panels(base):
 import numpy as np
 return [array(base.scene[n].data.output['rgb'])[0,:,:,:3].astype(np.uint8) for n in NAMES]
