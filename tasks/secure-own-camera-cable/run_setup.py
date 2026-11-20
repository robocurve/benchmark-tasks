from pathlib import Path
import argparse,json,time,traceback,subprocess
p=argparse.ArgumentParser();p.add_argument('--steps',type=int,default=140);p.add_argument('--out',required=True);a=p.parse_args();OUT=Path(a.out);OUT.mkdir(parents=True,exist_ok=True)
from isaaclab.app import AppLauncher
app=AppLauncher(headless=True,enable_cameras=True,device='cuda:0').app
try:
 import numpy as np,torch
 from PIL import Image
 from scipy.spatial.transform import Rotation
 import hardware
 hardware.install()
 from yam_arena import station,ik
 import yam_arena.scene as sm
 from yam_arena.constants import READY
 from yam_arena.embodiment import BimanualYamEmbodiment
 station.ITEMS={};orig=sm.station_assets
 def assets(cam):
  d=orig(cam);d.pop('bottle',None);return d
 sm.station_assets=assets
 emb=BimanualYamEmbodiment(enable_cameras=True,overview_cam=True,contact_sensors=False,robot_contact_offset=.0005,rig=station.rig(),cam_width=320,cam_height=176)
 emb.camera_config.overview_cam.width=800;emb.camera_config.overview_cam.height=600;emb.camera_config.overview_cam.spawn.focal_length=25.
 env=sm.make_env(emb,scene='station',control_hz=20,device='cuda:0');b=env.unwrapped;env.reset()
 pv=b.sim.physics_manager.get_physics_sim_view();cv=pv.create_rigid_body_view('/World/envs/env_0/service_cable/segment_*');fv=pv.create_rigid_body_view('/World/envs/env_0/service_flap/segment_*')

 def poses(v):
  x=v.get_transforms();return x.numpy() if hasattr(x,'numpy') else x.cpu().numpy()
 # Reset unmanaged task bodies once alongside robot reset, BEFORE any action.
 import warp as wp
 for v,key in [(cv,'cable_poses'),(fv,'flap_poses')]:
  desired=np.asarray(hardware.CREATED[key],dtype=np.float32);indices=wp.array(np.arange(v.count,dtype=np.int32),dtype=wp.int32,device=b.device)
  v.set_transforms(wp.array(desired,dtype=wp.float32,device=b.device),indices)
  v.set_velocities(wp.zeros((v.count,6),dtype=wp.float32,device=b.device),indices)
 (OUT/'initialization_check.json').write_text(json.dumps({'reset_once_before_action':True,'max_cable_position_error_m':float(np.linalg.norm(poses(cv)[:,:3]-np.array(hardware.CREATED['cable_poses'])[:,:3],axis=1).max())}))
 initial=json.loads(Path(__file__).with_name('initial_robot_state.json').read_text())['right_actual_q']
 for k in range(20):
  env.step(torch.tensor([[*READY,1.,*initial,1.]],device=b.device,dtype=torch.float32))
  if k in [0,19]:
   for name in ['top_cam','left_cam','right_cam']:
    rgb=b.scene[name].data.output['rgb'];rgb=rgb.torch if hasattr(rgb,'torch') else rgb;Image.fromarray(rgb[0,:,:,:3].cpu().numpy().astype('uint8')).save(OUT/f'{name}_{k:04d}.png')
 (OUT/'setup_check.json').write_text(json.dumps({'cable_bodies':cv.count,'strap_bodies':fv.count,'finite':bool(np.isfinite(poses(cv)).all() and np.isfinite(poses(fv)).all())}));env.close()
except Exception:
 (OUT/'error.txt').write_text(traceback.format_exc());raise
finally:app.close()
