"""Native flexible-hair/dryer dynamics probe. No successful robot demo yet."""
import argparse,json,os,shutil,hashlib
from pathlib import Path
from config import *
def main():
 import time
 started=time.monotonic()
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);p.add_argument('--mode',choices=['orientation-check','dynamics','power-probe','sweep-probe','slider-coupon'],default='dynamics');p.add_argument('--no-video',action='store_true',help='State-only engineering capture; no RGB/media claim');a=p.parse_args();a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=False);(a.out/'source').mkdir()
 for f in [*Path(__file__).parent.glob('*.py'),Path(__file__).with_name('switch_point.json')]:shutil.copy2(f,a.out/'source'/f.name)
 (a.out/'source-sha256.json').write_text(json.dumps({f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in [*Path(__file__).parent.glob('*.py'),*Path(__file__).parent.glob('*point.json')]},indent=2))
 os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES')
 if a.mode=='slider-coupon':
  from slider_coupon import run_coupon
  run_coupon(a.out);return
 from isaaclab.app import AppLauncher
 app=AppLauncher(headless=True,enable_cameras=not a.no_video).app;video=None;detent=None
 try:
  import numpy as np,torch,cv2,imageio.v2 as imageio
  from yam_arena import inspect_embodiment as wrapper
  from yam_arena import scene as sc
  from yam_arena.urdf import quat_xyzw_to_matrix
  import isaaclab.sim as sim
  from native import export,export_hair,install_slider,install_skin_attachment
  from airflow import HairThermal
  import task
  wrapper._APP=app
  assets={s.name:(export_hair(a.out/'mannequin.usda') if s.name=='mannequin' else export(a.out/(s.name+'.usda'),s.name)) for s in task.OBJECTS if s.name not in ('mat','dryer_pad','stand')}
  from yam_arena.embodiment import BimanualYamEmbodiment
  from isaaclab.assets import ArticulationCfg
  from isaaclab.actuators import ImplicitActuatorCfg
  original_init=BimanualYamEmbodiment.__init__
  def with_hair(instance,*args,**kwargs):
   kwargs['cam_width']=320;kwargs['cam_height']=180;kwargs['enable_cameras']=not a.no_video
   original_init(instance,*args,**kwargs)
   spawncfg=sim.UsdFileCfg(usd_path=assets['mannequin']);spawnfunc=spawncfg.func
   def spawn_world_geometry(path,cfg,translation=None,orientation=None,**kwargs):return spawnfunc(path,cfg,translation=(0.,0.,0.),orientation=(0.,0.,0.,1.),**kwargs)
   spawncfg.func=spawn_world_geometry
   hair_cfg=ArticulationCfg(prim_path='{ENV_REGEX_NS}/mannequin',spawn=spawncfg,init_state=ArticulationCfg.InitialStateCfg(pos=HEAD,rot=HEAD_QUAT),actuators={'passive_bending':ImplicitActuatorCfg(joint_names_expr=['bend_.*'],stiffness=BEND_STIFFNESS,damping=BEND_DAMPING)})
   from isaaclab_arena.utils.configclass import make_configclass
   fields=[(n,type(getattr(instance.scene_config,n)),getattr(instance.scene_config,n)) for n in instance.scene_config.__class__.__annotations__]
   instance.scene_config=make_configclass('YamWithPassiveHairCfg',fields+[('hair_model',ArticulationCfg,hair_cfg)])()
   import review
   if not a.no_video:review.add_cameras(instance)
  BimanualYamEmbodiment.__init__=with_hair
  original=sc.task_object;original_dt=sc.PHYSICS_DT;sc.PHYSICS_DT=1/480
  def make(spec):
   if spec.name not in assets:return original(spec)
   cfg=sim.UsdFileCfg(usd_path=assets[spec.name])
   if spec.name in ('switch','scalp_collision'):
    spawn=cfg.func
    def with_joints(path,cfg,translation=None,orientation=None,**kwargs):
     prim=spawn(path,cfg,translation=translation,orientation=orientation,**kwargs);parent=str(prim.GetPath()).rsplit('/',1)[0]
     (install_slider if spec.name=='switch' else install_skin_attachment)(prim.GetStage(),parent);return prim
    cfg.func=with_joints
   return sc.Object(name=spec.name,object_type=sc.ObjectType.BASE if spec.name in ('mannequin','dryer_holder') else sc.ObjectType.RIGID,spawner_cfg=cfg,initial_pose=sc.Pose(position_xyz=spec.pos,rotation_xyzw=HEAD_QUAT if spec.name=='scalp_collision' else (0,0,0,1)))
  sc.task_object=make
  try:
   env=wrapper.YamArenaEmbodiment(task=str(Path(__file__).with_name('task.py')),object_jitter=0.,control_hz=20,headless=True,cam_width=320,cam_height=180);env._ensure_sim(None)
  finally:sc.task_object=original;sc.PHYSICS_DT=original_dt;BimanualYamEmbodiment.__init__=original_init
  base=env._base;env._env.reset();hair=base.scene['hair_model'];hair_ids={n:i for i,n in enumerate(hair.body_names)};env._instruction='Hair dynamics validation, unscored';env._t=0.
  import review
  if not a.no_video:review.place(base,(.36,0.,.99))
  def arr(v):
   v=v.torch if hasattr(v,'torch') else v
   return v.detach().cpu().numpy()
  def pose(name):
   if name in hair_ids:return arr(hair.data.body_link_pose_w)[0,hair_ids[name]]
   return arr(base.arena_world.get_pose_w(name))[0]
  pose_names=['head','dryer','switch']+[f'hair_{k}_{j}' for k in range(LOCKS) for j in range(SEGMENTS)]
  ordered_ids=[hair_ids['head']]+[hair_ids[f'hair_{k}_{j}'] for k in range(LOCKS) for j in range(SEGMENTS)]
  def all_poses():
   d=hair.data.body_link_pose_w;d=d.torch if hasattr(d,'torch') else d
   h=d[0,ordered_ids]
   ds=[]
   for name in ['dryer','switch']:
    v=base.arena_world.get_pose_w(name);ds.append(v.torch if hasattr(v,'torch') else v)
   return torch.cat([h[:1],*ds,h[1:]],dim=0).detach().cpu().numpy()
  import native_capture as capture
  from yam_arena.robot import asset_link_paths
  owners=capture.native_body_owners(base.scene,asset_link_paths(),['dryer','switch','scalp_collision'])
  for name,index in hair_ids.items():
   owners['/World/envs/env_0/mannequin/'+name]={'kind':'articulation','entity':'hair_model','body_name':name,'body_index':index,'source':'ArticulationData.body_link_pose_w','parent_body_owner':None}
  if len(owners)!=22+LOCKS*SEGMENTS:raise RuntimeError('unexpected actual robot/hair/object owner count')
  for path in owners:
   if not base.scene.stage.GetPrimAtPath(path):raise RuntimeError('missing native actor '+path)
  manifest=capture.capture_manifest(capture.export_scene(base.scene.stage,a.out/'capture-scene.usdc'),owners,json.loads((a.out/'source-sha256.json').read_text()),capture.virtual_review_views(sim.PinholeCameraCfg(focal_length=18.,clipping_range=(.03,20.)),target=(.36,0.,.99)),no_video=a.no_video)
  (a.out/'capture-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
  from switch_detent import NativeDetentBinding
  detent=NativeDetentBinding(base)
  def force(name,f):
   asset=hair if name in hair_ids else base.scene[name]
   kw={'body_ids':[hair_ids[name]]} if name in hair_ids else {}
   asset.permanent_wrench_composer.set_forces_and_torques_index(forces=torch.tensor([[f]],device=base.device,dtype=torch.float32),torques=torch.zeros((1,1,3),device=base.device),is_global=True,**kw)
  if not a.no_video:video=imageio.get_writer(str(a.out/'diagnostic.mp4'),fps=20,codec='libx264',pixelformat='yuv420p',macro_block_size=1,ffmpeg_params=['-movflags','+faststart'])
  samples=[];thermal=HairThermal();worst_sep=0.;max_motion=0.;worst_skin_attachment=0.;robot=None
  if a.mode in ('power-probe','sweep-probe'):
   from robot import DryerProbe,DryerSweepProbe,MAX_PROBE_STEPS
   robot=(DryerSweepProbe if a.mode=='sweep-probe' else DryerProbe)(env)
   if a.mode=='sweep-probe':MAX_PROBE_STEPS=2400
  from scoring import HairScorer,measured_head_motion,MAX_HEAD_TRANSLATION_M,MAX_HEAD_ROTATION_DEG
  from yam_arena import ik
  tracker=HairScorer() if robot else None;previous_dryer=None
  if tracker is not None:task._tracker=tracker;task._diagnostic=True
  active_wind=set();max_head_translation=0.;max_head_rotation=0.
  with (a.out/'trajectory.jsonl').open('w') as trace:
   for step in range(MAX_PROBE_STEPS if robot else 180):
    phase='passive_orientation_check' if a.mode=='orientation-check' else 'settle' if step<60 else 'lateral_force_probe' if step<100 else 'passive_return'
    control=None
    if robot:
     target,control=robot.command(step*.05);phase=control['phase'];action=torch.tensor(target[None,:],device=base.device,dtype=torch.float32)
    else:
     if a.mode=='dynamics':force(f'hair_{LOCKS//2}_{SEGMENTS-2}',(.004 if phase=='lateral_force_probe' else 0.,0,0))
     action=env._ready
    if control is not None and control.get('failure'):
     (a.out/'rejected-command.json').write_text(json.dumps({'time':env._t,'control':control,'not_issued':True},indent=2)+'\n');break
    before_steps,before_time=detent.detent.steps,detent.detent.sim_time
    env._env.step(action);env._t=(step+1)*.05
    detent.detent.check_interval(before_steps,before_time,.05)
    poses=all_poses();segments=[];sep=0.;headpose=poses[0]
    head_motion=measured_head_motion(headpose.tolist())
    if not head_motion['head_pose_valid']:raise RuntimeError('missing or nonfinite native head pose')
    max_head_translation=max(max_head_translation,head_motion['head_translation_m']);max_head_rotation=max(max_head_rotation,head_motion['head_rotation_deg'])
    headrot=quat_xyzw_to_matrix(headpose[3:])
    for k in range(LOCKS):
     previous=None
     for j in range(SEGMENTS):
      p0=poses[3+k*SEGMENTS+j];r=quat_xyzw_to_matrix(p0[3:]);pos=p0[:3];tip0=pos+r@np.array((0,0,LENGTH/2));tip1=pos+r@np.array((0,0,-LENGTH/2))
      sep=max(sep,float(np.linalg.norm(tip0-(headpose[:3]+headrot@np.array(root_local(k)) if previous is None else previous))));previous=tip1
      segments.append({'pos':pos.tolist(),'rotation':r.tolist()})
    skin=pose('scalp_collision');skin_gap=float(np.linalg.norm(skin[:3]-headpose[:3]));worst_skin_attachment=max(worst_skin_attachment,skin_gap)
    if skin_gap>.001:raise RuntimeError('scalp collision body detached from measured head')
    dryer=poses[1];switch=poses[2];r=quat_xyzw_to_matrix(dryer[3:]);local=r.T@(switch[:3]-dryer[:3]);power=float(np.clip((local[2]+.045)/SWITCH_TRAVEL,0,1))
    m=thermal.step(.05,segments,(dryer[:3]+r@np.array((.088,0,0))).tolist(),r[:,0].tolist(),power,headpose[:3].tolist(),patch_samples=bool(robot))
    current_wind={i for i,f in enumerate(m['wind_forces_N']) if sum(v*v for v in f)>1e-12}
    if robot and (current_wind or active_wind):
     force_array=torch.tensor([m['wind_forces_N']],device=base.device,dtype=torch.float32);zero=torch.zeros_like(force_array);hair.permanent_wrench_composer.set_forces_and_torques_index(forces=force_array,torques=zero,body_ids=ordered_ids[1:],is_global=True)
     recoil=-np.sum(m['wind_forces_N'],axis=0);force('dryer',tuple(float(v) for v in recoil));active_wind=current_wind
    worst_sep=max(worst_sep,sep);max_motion=max(max_motion,float(np.linalg.norm(np.array(segments[(LOCKS//2)*SEGMENTS+SEGMENTS-2]['pos'])-np.array(segment(LOCKS//2,SEGMENTS-2)))))
    measured_score=None
    if tracker is not None:
     hands=[]
     for side in ['left','right']:
      ids,_=env._ids[side+'_robot'];q=arr(base.scene[side+'_robot'].data.joint_pos)[0,ids];hands.append(ik.tcp_pose(q,env.rig.bases[side])[0])
     speed=0. if previous_dryer is None else float(np.linalg.norm(dryer[:3]-previous_dryer)/.05);previous_dryer=dryer[:3].copy()
     measured_score=tracker.update({'time':env._t,'head_pose':headpose.tolist(),'root_separation':sep,'min_height':float(min(dryer[2]-.11,min(s['pos'][2]-LENGTH/2 for s in segments))-TOP),'dry_fraction':m['dry_fraction'],'worst_remaining_water_fraction':m['worst_remaining_water_fraction'],'max_temperature_C':m['max_temperature_C'],'dryer_speed':speed,'power_on':power>.5,'water_removed_step_kg':m['water_removed_step_kg'],'dryer_parked':bool(np.linalg.norm(dryer[:3]-np.array(DRYER_START))<.025),'hands_clear':all(np.linalg.norm(h-dryer[:3])>.12 for h in hands)})
    if measured_score is not None and measured_score['invalid']:
     robot.failure=measured_score['invalid'];control['failure']=robot.failure
    row={'time':env._t,'phase':phase,'robot_control':control,'hair':segments,'head_pose':headpose.tolist(),'head_motion':head_motion,'nose_direction_world':(headrot@np.array((-1.,0.,0.))).tolist(),'dryer_pose':dryer.tolist(),'switch_pose':switch.tolist(),'power_slider_fraction':power,'slider_detent':detent.snapshot(),'crease_separation':sep,'min_scalp_capsule_clearance_m':m['min_scalp_capsule_clearance_m'],'skin_attachment_error_m':skin_gap,'thermal':m,'score_metrics':measured_score}
    row['native_owner_poses']=capture.native_body_world_poses(base.scene,owners,{'dryer':dryer,'switch':switch,'scalp_collision':skin})
    row['action_14d']=arr(action)[0].tolist()
    trace.write(json.dumps(row)+'\n');samples.append(row)
    if not np.isfinite(dryer).all() or sep>.03:raise RuntimeError('unstable native hair dynamics')
    if not a.no_video:
     panels=review.panels(base);canvas=np.zeros((594,960,3),dtype=np.uint8);canvas[36:306,240:720]=panels[0];canvas[306:576,:480]=panels[1];canvas[306:576,480:]=panels[2]
     cv2.putText(canvas,('ROBOT DRYER POWER/PICKUP | ' if robot else 'NATIVE FLEXIBLE HAIR | ')+phase,(12,25),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1,cv2.LINE_AA);cv2.putText(canvas,('Ordinary robot commands; incomplete drying attempt. Actual moisture measured.' if robot else 'Diagnostic force test, not robot drying. Actual moisture/geometry measured.'),(12,590),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,205,120),1,cv2.LINE_AA);video.append_data(canvas)
     if step in (59,99,139,179,239,319,379,459,619):imageio.imwrite(a.out/f'step-{step}.png',canvas)
    if control is not None and (control.get('failure') or control.get('complete')):break
  result={'elapsed_wall_s':time.monotonic()-started,'no_video':a.no_video,'state_only_diagnostic':a.no_video,'media_eligible':False,'hair_discretization':{'locks':LOCKS,'segments_per_lock':SEGMENTS,'passive_bending_dofs':LOCKS*SEGMENTS,'patches':LOCKS*SEGMENTS*PATCHES_PER_SEGMENT,'capsule_length_m':LENGTH,'capsule_radius_m':RADIUS},'head_rest_quaternion_xyzw':HEAD_QUAT,'probe_mode':a.mode,'diagnostic':True,'task_score':None,'robot_demo':False,'max_joint_separation_m':worst_sep,'max_lock_motion_m':max_motion,'thermal':thermal.result(),'max_skin_attachment_error_m':worst_skin_attachment,'checks':{'scalp_contact_respected':all(s['thermal']['min_scalp_capsule_clearance_m']>=-.0002 for s in samples),'joints_intact':worst_sep<.005,'locks_move':max_motion>.005,'power_off_no_drying':all(s['thermal']['water_removed_step_kg']==0 for s in samples if s['power_slider_fraction']<.5)}}
  result['max_head_translation_m']=max_head_translation;result['max_head_rotation_deg']=max_head_rotation;result['checks']['head_fixed']=max_head_translation<=MAX_HEAD_TRANSLATION_M and max_head_rotation<=MAX_HEAD_ROTATION_DEG
  result['slider_detent']=detent.snapshot();result['checks']['detent_callback_cadence_verified']=True
  if robot:result|={'robot_power_probe':a.mode=='power-probe','robot_sweep_probe':a.mode=='sweep-probe','diagnostic':True,'task_score':None,'diagnostic_score_readback':tracker.value,'score_invalid':tracker.invalid,'controller_complete':bool(control and control.get('complete')),'controller_failure':None if control is None else control.get('failure'),'final_measured_power_fraction':samples[-1]['power_slider_fraction'],'final_dryer_pose':samples[-1]['dryer_pose']}
  (a.out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
 except BaseException as error:
  import traceback;traceback.print_exc();(a.out/'error.json').write_text(json.dumps({'error':str(error),'type':type(error).__name__},indent=2));raise
 finally:
  if detent is not None:
   try:detent.write_trace(a.out/'slider-crossings.json')
   finally:detent.close()
  if video is not None:video.close()
  app.close()
if __name__=='__main__':main()
