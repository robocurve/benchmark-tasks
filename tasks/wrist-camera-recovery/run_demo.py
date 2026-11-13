"""Physical wrist remount prototype with live input, not a Dream-1 trial."""
import argparse,json,time
from pathlib import Path
import shutil
p=argparse.ArgumentParser();p.add_argument('--out',required=True);a=p.parse_args();out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
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
 emb.camera_config.overview_cam.offset.pos=(-.08,-.16,1.04)
 # Set ROS look-at rotation for the diagnostic view only.
 eye=np.array([-.08,-.16,1.04]);f=np.array([.23,.05,.89])-eye;f/=np.linalg.norm(f);r=np.cross(f,[0,0,1]);r/=np.linalg.norm(r);d=np.cross(f,r)
 emb.camera_config.overview_cam.offset.rot=tuple(Rotation.from_matrix(np.column_stack([r,d,f])).as_quat());emb.camera_config.overview_cam.offset.convention='ros'
 env=make_env(emb,scene='task',task=task,control_hz=20);base=env.unwrapped;env.reset()
 import omni.usd
 stage=omni.usd.get_context().get_stage()
 parent='/World/envs/env_0/LeftArm'+asset_link_paths()['gripper']
 camera_paths=[str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI) and 'loose_camera' in str(p.GetPath())]
 if len(camera_paths)!=1:raise RuntimeError('Ambiguous camera rigid body '+str(camera_paths))
 assert stage.GetPrimAtPath(camera_paths[0]+'/shoe_core/geometry/mesh').HasAPI(UsdPhysics.CollisionAPI), "Missing shoe collider"
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
 rows=[];attached=False;hold=0.;scorer=RecoveryScorer();start=time.monotonic();latch_step=None;grasp_calibrated=False;dock_z=.035;dock_finish=None;dock_q=None;servo_audit={};servo_q=None
 for k in range(1500):
  t=(k-99)/20
  if t>=10.3:
   t=10.3 if dock_finish is None else 16.3+(k-dock_finish)/20
  if k==100:
   plan=make_plan(body.pos,R)
  if k>=100 and t>=7.3 and not grasp_calibrated:
   qr0=base.scene['right_robot'].data.joint_pos.torch[0,:6].cpu().numpy()
   tcp0,tcpR0=ik.tcp_pose(qr0,station.rig().bases['right'])
   grasp_offset=R.T@(tcp0-body.pos);grasp_rotation=R.T@tcpR0
   for w in plan.waypoints:
    w.pos=w.body+w.body_rotation@grasp_offset;w.rot=w.body_rotation@grasp_rotation
   grasp_calibrated=True
   (out/'grasp_calibration.json').write_text(json.dumps(dict(offset=grasp_offset.tolist(),rotation=grasp_rotation.tolist(),scope='Scripted reference controller only; measured physical grasp, no object pose writes'),indent=2))
  right,label=([*READY,1.],'settle') if k<100 else plan.action7(t)
  if k>=100 and t>=10.3 and not attached:
   # Scripted visual/pose servo: move joints, never teleport the camera.
   qnow=base.scene['right_robot'].data.joint_pos.torch[0,:6].cpu().numpy()
   pnow,Rnow=ik.tcp_pose(qnow,station.rig().bases['right'])
   if dock_z<.026 and not gate.entered:
    dock_z=.035
   local=targetR.T@(body.pos-target)
   if np.linalg.norm(local[:2])<.0006 and angle<np.deg2rad(2) and abs(local[2]-dock_z)<.002:
    dock_z=max(0.,dock_z-.0002)
   desired=target+targetR@np.array([0,0,dock_z])
   dp=.15*(desired-body.pos);dp*=min(1.,.0004/max(np.linalg.norm(dp),1e-9))
   rv=.15*Rotation.from_matrix(targetR@R.T).as_rotvec();rv*=min(1.,np.deg2rad(.15)/max(np.linalg.norm(rv),1e-9))
   deltaR=Rotation.from_rotvec(rv).as_matrix()
   if servo_q is None:servo_q=qnow.copy()
   pc,Rc=ik.tcp_pose(servo_q,station.rig().bases['right'])
   tcpgoal=pc+dp+(deltaR-np.eye(3))@(pnow-body.pos)
   qcmd,servo_pe,servo_re=ik.solve(tcpgoal,ik.matrix_to_quat_xyzw(deltaR@Rc),servo_q,base_pos=station.rig().bases['right'],iters=60)
   servo_q=qcmd.copy()
   right=np.r_[qcmd,0.]
   servo_audit=dict(ik_position_error=float(servo_pe),ik_rotation_error=float(servo_re),q=qnow.tolist(),qcmd=qcmd.tolist(),tcp=pnow.tolist(),tcpgoal=tcpgoal.tolist(),camera_goal=desired.tolist())
  if dock_finish is not None and k-dock_finish<24:
   right=np.r_[dock_q,min(1.,(k-dock_finish)/20)]
  fraction=ik.smoothstep(min(1.,max(0.,k/80)))
  left=np.array(READY)+(spec.PRESENT-np.array(READY))*fraction
  if k<100:label='present original wrist mount'
  if t>plan.duration+1:
   fraction=ik.smoothstep(min(1.,(t-plan.duration-1)/4))
   left=spec.PRESENT+(np.array(READY)-spec.PRESENT)*fraction;label='return to original working posture'
  env.step(torch.tensor([[*left,1.,*right]],device=base.device,dtype=torch.float32))
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
  # Insertion speed is relative to the moving receiver; use consecutive pose samples.
  gate_speed=float(np.linalg.norm(relative_offset-np.array(gate.previous))/.05) if gate.previous is not None else float('inf')
  if not attached and gate.update(relative_offset,angle,gate_speed):
   wrist_adapter.latch(stage,parent,camera_paths[0],gp,gR,body.pos,R);attached=True;latch_step=k;dock_finish=k;dock_q=base.scene['right_robot'].data.joint_pos.torch[0,:6].cpu().numpy().copy();wrist_adapter.engage_tooth(stage,parent)
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
  held=bool(nearest<.05 and right[6]<.5 and not attached)
  clear=bool(nearest>.08)
  live=bool(pixels.shape[-1]>=3 and np.isfinite(pixels[0,...,:3].cpu().numpy()).all())
  assessment=scorer.update(lifted=bool(corners[:,2].min()>spec.TOP+.020),held=held,latched=attached,position_error_m=native_error,rotation_error_rad=native_angle,other_hand_clear=clear,live_image=live,off_table=bool(body.pos[2]<spec.TOP-.05))
  if k%5==0:
   for name in ('top_cam','left_cam','right_cam','overview_cam'):
    rgb=base.scene[name].data.output['rgb'];rgb=rgb.torch if hasattr(rgb,'torch') else rgb
    Image.fromarray(rgb[0,...,:3].cpu().numpy().astype(np.uint8)).save(out/f'{name}_{k:04d}.png')
   rows.append(dict(step=k,phase=label,attached=attached,latch_step=latch_step,camera_pos=body.pos.tolist(),target=target.tolist(),error_m=error,angle_deg=float(np.rad2deg(angle)),right_clear=clear,retained_seconds=hold,sensor_error_m=sensor_error,sensor_angle_rad=sensor_angle,image_std=image_std,native_error_m=native_error,native_angle_deg=float(np.rad2deg(native_angle)),returned=returned,entry_verified=gate.entered,local_offset=(targetR.T@(body.pos-target)).tolist(),assessment=assessment,servo=servo_audit,raw_speed=float(np.linalg.norm(body.lin_vel)),gate_speed=gate_speed))
   (out/'trace.json').write_text(json.dumps(rows,indent=2));print(rows[-1],flush=True)
  if dock_finish is not None and t>plan.duration+8:break
 (out/'result.json').write_text(json.dumps(dict(completed=True,wall_seconds=time.monotonic()-start,final=rows[-1],retention_pass=bool(hold>=2),score=assessment["score"],scope='Original native camera-to-gripper calibration is immutable. No READY-return requirement. Physical keyed latch approximation; no Dream-1.'),indent=2))
 env.close()
except Exception:
 import traceback
 (out/'error.txt').write_text(traceback.format_exc());raise
finally:app.close()
