from pathlib import Path
import argparse,json,time,traceback,subprocess
p=argparse.ArgumentParser();p.add_argument('--steps',type=int,default=1800);p.add_argument('--out',required=True);a=p.parse_args();OUT=Path(a.out);OUT.mkdir(parents=True,exist_ok=True)
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
 emb=BimanualYamEmbodiment(enable_cameras=True,overview_cam=False,contact_sensors=False,robot_contact_offset=.0005,rig=station.rig(),cam_width=320,cam_height=176)
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
 import importlib.util
 spec=importlib.util.spec_from_file_location('cable_dream_driver',Path(__file__).with_name('dream_driver.py'));module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);DreamDriver=module.DreamDriver
 print('Dream driver',module.__file__,'rows_per_request=4',flush=True)
 def mount_T():
  robot=b.scene['left_robot'];i=robot.body_names.index('link3');pos=robot.data.body_link_pos_w[0,i].cpu().numpy();q=robot.data.body_link_quat_w[0,i].cpu().numpy();T=np.eye(4);T[:3,3]=pos;T[:3,:3]=Rotation.from_quat(q).as_matrix();return T
 import importlib.util
 adhesive_spec=importlib.util.spec_from_file_location('cable_adhesive',Path(__file__).with_name('velcro_contact.py'));adhesive_module=importlib.util.module_from_spec(adhesive_spec);adhesive_spec.loader.exec_module(adhesive_module);VelcroContact=adhesive_module.VelcroContact
 adhesive=VelcroContact(pv,hardware);adhesive.bind();latched=False;rows=[]
 # Warm up before requesting a paid session; native images only.
 from yam_arena.constants import READY
 for _ in range(20):env.step(torch.tensor([[*READY,1.,*json.loads(Path(__file__).with_name('initial_robot_state.json').read_text())['right_actual_q'],1.]],device=b.device,dtype=torch.float32))
 instruction=json.loads((Path(__file__).with_name('task.json')).read_text())['instruction']
 (OUT/'instruction.txt').write_text(instruction)
 driver=DreamDriver(b,instruction,OUT);started=time.monotonic()
 for k in range(a.steps):
  action=driver.action()
  if latched:adhesive.maintain(mount_T())
  _,_,terminated,truncated,_=env.step(torch.tensor(action[None],device=b.device,dtype=torch.float32))
  if bool(terminated.any()) or bool(truncated.any()):raise RuntimeError('Unexpected environment termination or reset')
  T=mount_T();fp=poses(fv)
  latched=adhesive.update(fp,T,.05) or latched
  if k%5==0:
   for name in ['top_cam','left_cam','right_cam']:
    rgb=b.scene[name].data.output['rgb'];rgb=rgb.torch if hasattr(rgb,'torch') else rgb;Image.fromarray(rgb[0,:,:,:3].cpu().numpy().astype('uint8')).save(OUT/f'{name}_{k:04d}.png')
   rows.append(dict(step=k,left_actual_q=b.scene['left_robot'].data.joint_pos.torch[0,:6].cpu().tolist(),right_actual_q=b.scene['right_robot'].data.joint_pos.torch[0,:6].cpu().tolist(),right_actual_fingers=b.scene['right_robot'].data.joint_pos.torch[0,6:].cpu().tolist(),phase='Dream-1',latched=latched,cable=poses(cv).tolist(),flap=fp.tolist()))
  if k%100==0:print('Dream step',k,'latched',latched,'wall',time.monotonic()-started,flush=True);(OUT/'trace.json').write_text(json.dumps(rows))
 driver.close();(OUT/'trace.json').write_text(json.dumps(rows));(OUT/'result.json').write_text(json.dumps(dict(completed=True,steps=a.steps,continuous_session=(len({x['session_id'] for x in driver.request_audit})==1),latched=latched,remaining_contact_regions=adhesive.matches(poses(fv),mount_T()),wall_seconds=time.monotonic()-started),indent=2));env.close()
except Exception:
 (OUT/'error.txt').write_text(traceback.format_exc());print(traceback.format_exc(),flush=True)
finally:app.close()
