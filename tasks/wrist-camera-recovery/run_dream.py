"""Dream-1 wrist-camera recovery trial; no scripted policy assistance."""
import argparse,json,time
from pathlib import Path
import shutil
p=argparse.ArgumentParser();p.add_argument('--out',required=True);p.add_argument('--max-steps',type=int,default=1800);a=p.parse_args();out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
snapshot=out/'source';snapshot.mkdir(exist_ok=True)
for source in Path(__file__).parent.iterdir():
 if source.suffix in ('.py','.json'):
  shutil.copy2(source,snapshot/source.name)
from isaaclab.app import AppLauncher
app=AppLauncher(headless=True,enable_cameras=True,device='cuda:0').app
try:
 import numpy as np,torch
 from PIL import Image
 from scipy.spatial.transform import Rotation
 from pxr import UsdPhysics
 import wrist_adapter
 wrist_adapter.install()
 import asset_mirror
 (out/"asset_mirror.json").write_text(json.dumps(asset_mirror.install(),indent=2))
 import scene as spec
 from scorer import RecoveryScorer
 from yam_arena import station,task_file,ik
 from yam_arena.constants import READY,TCP_OFFSET
 from yam_arena.embodiment import BimanualYamEmbodiment
 from yam_arena.scene import make_env,object_states
 from yam_arena.demo import ArmPlan,grip_rotation
 from yam_arena.robot import asset_link_paths
 task=task_file.load(str(Path(__file__).with_name('scene.py')))
 emb=BimanualYamEmbodiment(enable_cameras=True,overview_cam=True,contact_sensors=False,rig=station.rig(),cam_width=320,cam_height=176)
 # Begin in original READY working posture; recipient actively presents mount.
 # Detached wrist sensor is world-rooted and follows the physical housing, never the wrist by default.
 emb.camera_config.left_cam.prim_path='{ENV_REGEX_NS}/DetachedLeftCam'
 emb.camera_config.overview_cam.offset.pos=(1.,-.8,1.5)
 # Set ROS look-at rotation for the diagnostic view only.
 eye=np.array([1.,-.8,1.5]);f=np.array([.25,0,.85])-eye;f/=np.linalg.norm(f);r=np.cross(f,[0,0,1]);r/=np.linalg.norm(r);d=np.cross(f,r)
 emb.camera_config.overview_cam.offset.rot=tuple(Rotation.from_matrix(np.column_stack([r,d,f])).as_quat());emb.camera_config.overview_cam.offset.convention='ros'
 env=make_env(emb,scene='task',task=task,control_hz=20);base=env.unwrapped;env.reset()
 import omni.usd
 stage=omni.usd.get_context().get_stage()
 parent='/World/envs/env_0/LeftArm'+asset_link_paths()['gripper']
 camera_paths=[str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI) and 'loose_camera' in str(p.GetPath())]
 if len(camera_paths)!=1:raise RuntimeError('Ambiguous camera rigid body '+str(camera_paths))
 assert stage.GetPrimAtPath(camera_paths[0]+'/shoe_core/geometry/mesh').HasAPI(UsdPhysics.CollisionAPI), 'Missing shoe collider'
 from latch_geometry import InsertionGate
 gate=InsertionGate()
 from motion import make_plan
 plan=make_plan();sensor=base.scene['left_cam']
 import isaaclab.sim as sim
 for name,size,position,color in [('lens_trim',(.018,.0002,.018),(0,.01811,0),(.15,.35,.55)),('lens',(.012,.0002,.012),(0,.01824,0),(.015,.02,.025))]:
  visual=sim.CuboidCfg(size=size,visual_material=sim.PreviewSurfaceCfg(diffuse_color=color))
  visual.func(camera_paths[0]+'/'+name,visual,translation=position)
 native=station.wrist_camera('left');mount=np.array(spec.LAYOUT['mount_position_in_gripper']);nativeR=Rotation.from_quat(native.rot_xyzw).as_matrix()
 # Housing is upright at insertion; same optical convention and orientation as native wrist when seated.
 presentR=np.array(spec.LAYOUT['present_rotation']);mountR=np.array(spec.LAYOUT['mount_rotation_in_gripper']);optical=np.array(spec.LAYOUT['optical_rotation_body']);lens_offset=np.array(spec.LAYOUT['lens_offset_body'])
 locked=json.loads(Path(__file__).with_name('native_camera_target.json').read_text())
 assert np.allclose(native.pos,locked['position_m'],atol=1e-10)
 assert np.allclose(mountR@optical,nativeR,atol=1e-10)
 assert np.allclose(mount+mountR@lens_offset,native.pos,atol=1e-10)
 rows=[];attached=False;hold=0.;scorer=RecoveryScorer();start=time.monotonic();latch_step=None
 from dream_driver import DreamDriver
 # Initialize loose-camera optical pose before the first policy observation.
 from dataclasses import replace
 def arr(x):
  x=x.torch if hasattr(x,'torch') else x
  return x[0].cpu().numpy()
 rb=base.scene.rigid_objects['loose_camera'].data
 initial=object_states(env,task.names)['loose_camera']
 initial=replace(initial,pos=arr(rb.root_pos_w),quat_xyzw=arr(rb.root_quat_w))
 initialR=Rotation.from_quat(initial.quat_xyzw).as_matrix()
 sensor.set_world_poses(positions=torch.tensor((initial.pos+initialR@lens_offset)[None],device=base.device,dtype=torch.float32),orientations=torch.tensor(Rotation.from_matrix(initialR@optical).as_quat()[None],device=base.device,dtype=torch.float32),convention='opengl')
 base.sim.render();sensor.update(0.,force_recompute=True)
 instruction=json.loads(Path(__file__).with_name('task.json').read_text())['instruction']
 driver=DreamDriver(base,instruction,out)
 for k in range(a.max_steps):
  label='Dream-1 control'
  env.step(torch.tensor(driver.action()[None],device=base.device,dtype=torch.float32))
  states=object_states(env,task.names);body=states['loose_camera']
  # Compound-shape AABB centroid is not the housing/optical reference frame.
  from dataclasses import replace
  rb=base.scene.rigid_objects['loose_camera'].data
  def arr(x):
   x=x.torch if hasattr(x,'torch') else x
   return x[0].cpu().numpy()
  body=replace(body,pos=arr(rb.root_pos_w),quat_xyzw=arr(rb.root_quat_w))
  R=Rotation.from_quat(body.quat_xyzw).as_matrix()
  ql=base.scene['left_robot'].data.joint_pos.torch[0,:6].cpu().numpy();tcp,gR=ik.tcp_pose(ql,station.rig().bases['left']);gp=tcp-gR@np.array(TCP_OFFSET)
  target=gp+gR@mount;targetR=gR@mountR
  error=float(np.linalg.norm(body.pos-target));angle=float(Rotation.from_matrix(targetR.T@R).magnitude())
  relative_offset=targetR.T@(body.pos-target)
  gate_speed=float(np.linalg.norm(relative_offset-np.array(gate.previous))/.05) if gate.previous is not None else float('inf')
  if not attached and gate.update(relative_offset,angle,gate_speed):
   wrist_adapter.latch(stage,parent,camera_paths[0],gp,gR,body.pos,R);attached=True;latch_step=k;wrist_adapter.engage_tooth(stage,parent)
  sensor.set_world_poses(positions=torch.tensor((body.pos+R@lens_offset)[None],device=base.device,dtype=torch.float32),orientations=torch.tensor(Rotation.from_matrix(R@optical).as_quat()[None],device=base.device,dtype=torch.float32),convention='opengl')
  base.sim.render();sensor.update(0.,force_recompute=True)
  qr=base.scene['right_robot'].data.joint_pos.torch[0,:6].cpu().numpy();rtcp,_=ik.tcp_pose(qr,station.rig().bases['right']);clear=float(np.linalg.norm(rtcp-body.pos))>.08
  valid=attached and error<.006 and angle<np.deg2rad(4) and clear
  hold=hold+.05 if valid else 0.
  actual_pos=sensor.data.pos_w.torch[0].cpu().numpy();actual_q=sensor.data.quat_w_opengl.torch[0].cpu().numpy()
  sensor_error=float(np.linalg.norm(actual_pos-(body.pos+R@lens_offset)));sensor_angle=float(Rotation.from_matrix((R@optical).T@Rotation.from_quat(actual_q).as_matrix()).magnitude())
  native_pos=gp+gR@np.array(native.pos);native_orientation=gR@nativeR
  native_error=float(np.linalg.norm(actual_pos-native_pos));native_angle=float(Rotation.from_matrix(native_orientation.T@Rotation.from_quat(actual_q).as_matrix()).magnitude())
  returned=bool(np.max(np.abs(ql-np.array(READY)))<.03)
  pixels=sensor.data.output['rgb'];pixels=pixels.torch if hasattr(pixels,'torch') else pixels
  image_std=float(pixels[0,...,:3].float().std().item())
  # Task scoring uses observable geometry and live frame delivery; precision sensor diagnostics remain separate.
  corners=np.array([[x,y,z] for x in [-.020,.020] for y in [-.027,.018] for z in [-.024,.014]])@R.T+body.pos
  nearest=np.linalg.norm(np.maximum(np.abs(R.T@(rtcp-body.pos))-np.array([.020,.027,.024]),0))
  held=bool(nearest<.05 and driver.actions[-1][13]<.5 and not attached)
  clear=bool(nearest>.08)
  live=bool(pixels.shape[-1]>=3 and np.isfinite(pixels[0,...,:3].cpu().numpy()).all())
  assessment=scorer.update(lifted=bool(corners[:,2].min()>spec.TOP+.020),held=held,latched=attached,position_error_m=native_error,rotation_error_rad=native_angle,other_hand_clear=clear,live_image=live,off_table=bool(body.pos[2]<spec.TOP-.05))
  if k%5==0:
   for name in ('top_cam','left_cam','right_cam','overview_cam'):
    rgb=base.scene[name].data.output['rgb'];rgb=rgb.torch if hasattr(rgb,'torch') else rgb
    Image.fromarray(rgb[0,...,:3].cpu().numpy().astype(np.uint8)).save(out/f'{name}_{k:04d}.png')
   rows.append(dict(step=k,phase=label,attached=attached,latch_step=latch_step,camera_pos=body.pos.tolist(),target=target.tolist(),error_m=error,angle_deg=float(np.rad2deg(angle)),right_clear=clear,retained_seconds=hold,sensor_error_m=sensor_error,sensor_angle_rad=sensor_angle,image_std=image_std,native_error_m=native_error,native_angle_deg=float(np.rad2deg(native_angle)),returned=returned,entry_verified=gate.entered,local_offset=(targetR.T@(body.pos-target)).tolist(),assessment=assessment,raw_speed=float(np.linalg.norm(body.lin_vel)),gate_speed=gate_speed))
   (out/'trace.json').write_text(json.dumps(rows,indent=2));print(rows[-1],flush=True)
 (out/'result.json').write_text(json.dumps(dict(completed=True,wall_seconds=time.monotonic()-start,final=rows[-1],retention_pass=bool(hold>=2),score=assessment["score"],scope='Dream-1 beta controls both arms; original optical target and scorer unchanged; 90-second trial.'),indent=2))
 driver.close()
 env.close()
except Exception:
 import traceback
 (out/'error.txt').write_text(traceback.format_exc());raise
finally:app.close()
