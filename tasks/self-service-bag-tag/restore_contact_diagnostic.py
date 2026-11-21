"""Explicit diagnostic-only initial-state reset. Never used for a success demo."""
import json,numpy as np
import torch
def restore(run,path,step):
 from isaaclab.sim.utils.stage import get_current_stage
 s=min(json.load(open(path))['trace'],key=lambda s:abs(s['step']-step))
 base=run.emb._env.unwrapped;stage=get_current_stage()
 for prim in list(stage.Traverse()):
  if prim.GetName()=='frangible_joint' and 'tag_panel_00' in str(prim.GetPath()): stage.RemovePrim(prim.GetPath())
 for i in range(10):
  a=base.scene[f'tag_panel_{i:02d}'];pose=torch.tensor([[*s['panel_xyz'][i],*s['panel_quat_xyzw'][i]]],device=base.device,dtype=torch.float32)
  a.write_root_pose_to_sim(pose);a.write_root_velocity_to_sim(torch.zeros((1,6),device=base.device))
 for side in ('left','right'):
  a=base.scene[side+'_robot'];q=torch.tensor([s['robots'][side]['joints']],device=base.device,dtype=torch.float32)
  a.write_joint_state_to_sim(q,torch.zeros_like(q));a.set_joint_position_target(q)
 base.scene.write_data_to_sim();base.scene.update(0.)
 run.trace.append(run.state(('staged restored initial condition','not a continuous demo')))
 (run.out/'STAGED_NOT_DEMO.json').write_text(json.dumps({'source':path,'step':s['step'],'object_state_reset':True},indent=2))
