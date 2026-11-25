"""Native latch/cable mechanics probe. External test forces are NOT a robot demo."""
import argparse,os,json,hashlib,time,shutil
from pathlib import Path
from config import *

def main():
 started=time.monotonic();timings={key:0. for key in ('setup_s','ensure_sim_s','controller_build_s','policy_s','physics_step_s','telemetry_s','video_s')}
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--mode',choices=['fit-probe','retention-probe','clip-constraint-probe','grasp-probe','release-probe','press-tracking-probe','clip-press-probe'],default='retention-probe');p.add_argument('--out',type=Path,required=True);p.add_argument('--no-video',action='store_true',help='State-only engineering run; final media require the default video mode');p.add_argument('--pinch-spec',type=Path);p.add_argument('--batch-size',type=int,default=1);p.add_argument('--tune',type=Path,help='JSON list of search candidates for fresh native resets');p.add_argument('--pinch-plan',type=int,help='Right-hand shell and clip pinch candidate index');a=p.parse_args()
 if a.pinch_plan is not None and a.mode!='release-probe':p.error('--pinch-plan requires a scored release-probe')
 isolated_mode=a.mode in ('press-tracking-probe','clip-press-probe')
 a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=False)
 (a.out/'source').mkdir()
 for f in [*Path(__file__).parent.glob('*.py'),*Path(__file__).parent.glob('*point.json'),*Path(__file__).parent.glob('pinch_plans.json'),*Path(__file__).parent.glob('boot_prefix.npz')]:shutil.copy2(f,a.out/'source'/f.name)
 (a.out/'source-sha256.json').write_text(json.dumps({f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in [*Path(__file__).parent.glob('*.py'),*Path(__file__).parent.glob('*point.json'),*Path(__file__).parent.glob('pinch_plans.json'),*Path(__file__).parent.glob('boot_prefix.npz')]},indent=2))
 os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES')
 from isaaclab.app import AppLauncher
 app=AppLauncher(headless=True,enable_cameras=not a.no_video).app
 video=None
 try:
  import numpy as np,torch,cv2,imageio.v2 as imageio
  from yam_arena import inspect_embodiment as wrapper
  from yam_arena import scene as sc
  from yam_arena.urdf import quat_xyzw_to_matrix
  import isaaclab.sim as sim
  from native import export,install_joints
  import task
  from diagnostics import bench_command,connector_fit,fit_summary,robot_snapshot,offcenter_clip_wrench,clip_constraint_summary
  wrapper._APP=app
  assets={s.name:export(a.out/(s.name+'.usda'),s.name) for s in task.OBJECTS if s.name!='mat'}
  original=sc.task_object
  def make(spec):
   if spec.name not in assets:return original(spec)
   cfg=sim.UsdFileCfg(usd_path=assets[spec.name])
   if spec.name==f'wire_{USED[-1]}_{LINKS-1}':
    spawn=cfg.func
    def with_joints(path,cfg,translation=None,orientation=None,**kwargs):
     prim=spawn(path,cfg,translation=translation,orientation=orientation,**kwargs)
     install_joints(prim.GetStage(),str(prim.GetPath()).rsplit('/',1)[0]);return prim
    cfg.func=with_joints
   return sc.Object(name=spec.name,object_type=sc.ObjectType.BASE if spec.name=='switch' else sc.ObjectType.RIGID,spawner_cfg=cfg,initial_pose=sc.Pose(position_xyz=spec.pos,rotation_xyzw=(0,0,0,1)))
  from yam_arena.embodiment import BimanualYamEmbodiment
  import review
  original_init=BimanualYamEmbodiment.__init__
  def with_reviews(instance,*args,**kwargs):
   kwargs['cam_width']=320;kwargs['cam_height']=180;kwargs['enable_cameras']=not a.no_video;original_init(instance,*args,**kwargs)
   if not a.no_video:review.add_cameras(instance)
  BimanualYamEmbodiment.__init__=with_reviews
  sc.task_object=make
  ensure_started=time.monotonic()
  try:
   if a.tune is not None and a.batch_size>1:
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    def make_search_env(embodiment, *, scene, device, **kw):
     assert scene=='task'
     cfg=ArenaEnvBuilderCfg(num_envs=a.batch_size,solve_relations=False,device=device)
     return ArenaEnvBuilder(sc.build_task_environment(embodiment,**kw),cfg).make_registered()
    sc.make_env=make_search_env
   env=wrapper.YamArenaEmbodiment(task=str(Path(__file__).with_name('task.py')),object_jitter=0.,control_hz=20,headless=True,cam_width=320,cam_height=180)
   env._ensure_sim(None)
  finally:
   timings['ensure_sim_s']=time.monotonic()-ensure_started
   sc.task_object=original;BimanualYamEmbodiment.__init__=original_init
  base=env._base;env._env.reset();env._instruction='Untouched native fit; no task score' if a.mode=='fit-probe' else 'Native retention force probe; no task score';env._t=0.
  if a.tune is not None:
   if a.mode!='release-probe' or not a.no_video:raise ValueError('tuning requires a camera-free release run')
   from tune import run_trials
   run_trials(env,a.out,json.loads(a.tune.read_text()))
   return
  if not a.no_video:review.place(base,(.41,0.,.96))
  from physics_manifest import manifest
  from yam_arena import YAM_URDF
  physical=manifest(base.scene.stage,assets,YAM_URDF,sc.PHYSICS_DT,20)
  (a.out/'physics-manifest.json').write_text(json.dumps(physical,indent=2)+'\n')
  from pxr import PhysxSchema
  solver_readback={}
  for prim in base.scene.stage.Traverse():
   if prim.GetName() not in {f'{kind}_{i}' for kind in ('plug','clip') for i in USED}:continue
   api=PhysxSchema.PhysxRigidBodyAPI(prim)
   solver_readback[str(prim.GetPath())]={'position':api.GetSolverPositionIterationCountAttr().Get(),
                                      'velocity':api.GetSolverVelocityIterationCountAttr().Get()}
  if len(solver_readback)!=2*len(USED) or any(v!={'position':64,'velocity':16} for v in solver_readback.values()):
   raise RuntimeError('native plug/clip solver iteration readback does not match 64/16: '+str(solver_readback))
  (a.out/'native-solver-readback.json').write_text(json.dumps(solver_readback,indent=2)+'\n')
  def arr(v):
   v=v.torch if hasattr(v,'torch') else v
   return v.detach().cpu().numpy()
  actuator_readback={}
  for side in ('left','right'):
   arm=base.scene[side+'_robot']
   actuator_readback[side]={'joint_names':list(arm.joint_names)}
   for field in ('joint_stiffness','joint_damping','joint_friction_coeff','joint_dynamic_friction_coeff','joint_viscous_friction_coeff'):
    value=getattr(arm.data,field,None)
    actuator_readback[side][field]=None if value is None else arr(value)[0].tolist()
  actuator_readback['semantics']='Installed isaaclab_physx: static/dynamic friction are efforts for Isaac Sim >=5.0; viscous friction is effort per velocity. Read only; no parameter changes.'
  (a.out/'native-actuator-readback.json').write_text(json.dumps(actuator_readback,indent=2)+'\n')
  def pose(name):return arr(base.arena_world.get_pose_w(name))[0]
  def force(name,f,torque=(0.,0.,0.)):
   base.scene[name].permanent_wrench_composer.set_forces_and_torques_index(forces=torch.as_tensor(np.asarray(f)[None,None,:],device=base.device,dtype=torch.float32),torques=torch.as_tensor(np.asarray(torque)[None,None,:],device=base.device,dtype=torch.float32),is_global=True)
  if not a.no_video:video=imageio.get_writer(str(a.out/'diagnostic.mp4'),fps=20,codec='libx264',pixelformat='yuv420p',macro_block_size=1,ffmpeg_params=['-movflags','+faststart'])
  samples=[]
  body_names=[n for i in USED for n in (f'plug_{i}',f'clip_{i}')]+[f'wire_{i}_{k}' for i in USED for k in range(LINKS)]
  def native_body_snapshot():
   # One device-to-host transfer keeps every pose in the same physics frame.
   values=[]
   for name in body_names:
    data=base.arena_world.get_pose_w(name);values.append(data.torch if hasattr(data,'torch') else data)
   rows=arr(torch.cat(values,dim=0))
   return {name:rows[j] for j,name in enumerate(body_names)}
  timings['setup_s']=time.monotonic()-started-timings['ensure_sim_s']
  build_started=time.monotonic();robot=None
  if a.mode in ('grasp-probe','release-probe','press-tracking-probe','clip-press-probe'):
   from robot import GraspProbe,ReleaseProbe,PressTrackingProbe,MAX_RELEASE_STEPS
   if a.pinch_plan is not None or a.pinch_spec is not None:
    from pinch import make_policy
    robot=make_policy(env,json.loads(a.pinch_spec.read_text()) if a.pinch_spec is not None else a.pinch_plan)
   else:robot={'grasp-probe':GraspProbe,'release-probe':ReleaseProbe,'press-tracking-probe':PressTrackingProbe,'clip-press-probe':PressTrackingProbe}[a.mode](env,pose(f'plug_{SERVICE}')[:3])
  timings['controller_build_s']=time.monotonic()-build_started
  scorer=None;previous_plug=None
  if a.mode=='release-probe':
   from feedback import retaining_face,clearance
   from scoring import EthernetScorer
   scorer=EthernetScorer();task._tracker=scorer;task._diagnostic=False
  step_limit=MAX_RELEASE_STEPS if a.mode=='release-probe' else 1000 if isolated_mode else 600 if robot else 120 if a.mode=='fit-probe' else 180
  with (a.out/'trajectory.jsonl').open('w') as trace:
   for step in range(step_limit):
    policy_started=time.monotonic()
    phase,pull,press=bench_command(step,a.mode)
    if phase=='pull_pressed' and pose(f'plug_{SERVICE}')[0]<PLUG_X-.035:pull=0.
    control=None
    if robot:
     target,control=robot.command(step*.05);phase=control['phase'];pull=press=0.
     if a.mode=='press-tracking-probe' and robot.phase=='depress_clip' and not control.get('failure'):
      control['complete']=True;control['diagnostic_stop']='prepress complete; stopped before clip contact'
     if a.mode=='clip-press-probe' and robot.phase=='withdraw_with_clip_pressed' and not control.get('failure'):
      control['complete']=True;control['diagnostic_stop']='measured clip release sustained; stopped before withdrawal'
     action=torch.tensor(target[None,:],device=base.device,dtype=torch.float32)
    else:
     if a.mode in ('retention-probe','clip-constraint-probe'):
      force(f'plug_{SERVICE}',(pull,0,0))
      if a.mode=='clip-constraint-probe':
       load,moment=offcenter_clip_wrench(pose(f'plug_{SERVICE}'),pose(f'clip_{SERVICE}'),press)
       force(f'clip_{SERVICE}',load,moment)
      else:force(f'clip_{SERVICE}',(0,0,press))
     action=env._ready
    timings['policy_s']+=time.monotonic()-policy_started
    step_started=time.monotonic();env._env.step(action);env._t=(step+1)*.05
    timings['physics_step_s']+=time.monotonic()-step_started
    telemetry_started=time.monotonic()
    bodies=native_body_snapshot();p0=bodies[f'plug_{SERVICE}'];c=bodies[f'clip_{SERVICE}']
    from yam_arena import ik
    native_hands=robot_snapshot(env,arr)
    row={'time':env._t,'phase':phase,'robot_control':control,'plug_pose':p0.tolist(),'clip_pose':c.tolist(),
         'pull_N':pull,'press_N':press,'action_14d':arr(action)[0].tolist(),'native_hands':native_hands,
         'plug_poses':{str(i):bodies[f'plug_{i}'].tolist() for i in USED},
         'clip_poses':{str(i):bodies[f'clip_{i}'].tolist() for i in USED},
         'wire_body_poses':{f'wire_{i}_{k}':bodies[f'wire_{i}_{k}'].tolist() for i in USED for k in range(LINKS)},
         'connector_fit':{str(i):connector_fit(bodies[f'plug_{i}'],bodies[f'clip_{i}'],i) for i in USED},
         'neighbours':{str(i):bodies[f'plug_{i}'].tolist() for i in USED if i!=SERVICE}}
    if scorer is not None:
     rp=quat_xyzw_to_matrix(p0[3:]);local_clip=rp.T@(c[:3]-p0[:3]);travel=float(CLIP_Z-local_clip[2]);tip=p0[:3]+rp@np.array((.012,0,0))
     def insertion(i):
      center=np.array(port(i));lateral=np.linalg.norm(p0[1:3]-center[1:3]);return float(tip[0]-(.412-.027/2)) if lateral<.007 and rp[0,0]>.8 else -.020
     target=np.array(port(TARGET));face=retaining_face(c,TARGET);hand_positions=[]
     from yam_arena import ik
     for side in ('left','right'):
      ids,_=env._ids[side+'_robot'];d=arr(base.scene[side+'_robot'].data.joint_pos)[0,ids];hand_positions.append(ik.tcp_pose(d,env.rig.bases[side])[0])
     tcp_clear=all(np.linalg.norm(h-p0[:3])>.070 for h in hand_positions)
     mesh_clearance=None
     if tcp_clear:
      parts=[]
      for hand in native_hands:
       parts.extend(a@np.array(hand['tcp_rotation']).T+np.array(hand['tcp']) for _,a in robot.geometry.pieces(hand['finger_joints']))
      state=({i:bodies[f'plug_{i}'] for i in USED},{i:bodies[f'clip_{i}'] for i in USED},
             {(i,k):bodies[f'wire_{i}_{k}'] for i in USED for k in range(LINKS)})
      mesh_clearance=clearance(parts,state)
     row['actual_retaining_face']=face;row['hands_mesh_clearance']=mesh_clearance
     sample={'time':env._t,'clip_travel':travel,'source_insertion':insertion(SERVICE),'target_insertion':insertion(TARGET),'target_lateral_error':float(np.linalg.norm(p0[1:3]-target[1:3])),'target_angle':float(np.arccos(np.clip(rp[0,0],-1,1))),'speed':0. if previous_plug is None else float(np.linalg.norm(p0[:3]-previous_plug)/.05),'height':float(p0[2]),'neighbour_median_depth':float(np.median([bodies[f'plug_{i}'][0]+quat_xyzw_to_matrix(bodies[f'plug_{i}'][3:])[0,0]*.012-(.412-.027/2) for i in USED if i!=SERVICE])),'max_neighbour_withdrawal':max(max(0.,PLUG_X-v[0]) for v in row['neighbours'].values()),'hands_clear':bool(tcp_clear and mesh_clearance is not None and mesh_clearance['minimum_gap_m']>=.005),'hook_retained':face['retained']}
     previous_plug=p0[:3].copy();row['score_sample']=sample;row['metrics']=scorer.update(sample)
    if not np.isfinite([*p0,*c]).all():raise RuntimeError('nonfinite rigid body state')
    trace.write(json.dumps(row)+'\n');samples.append(row)
    timings['telemetry_s']+=time.monotonic()-telemetry_started
    if video is not None:
     video_started=time.monotonic()
     panels=review.panels(base)
     canvas=np.zeros((594,960,3),dtype=np.uint8);canvas[36:306,240:720]=panels[0];canvas[306:576,:480]=panels[1];canvas[306:576,480:]=panels[2]
     cv2.putText(canvas,(('ETHERNET ROBOT RELEASE ATTEMPT | ' if scorer else 'ETHERNET RIGHT ARM CONTACT PROBE | ' if a.mode=='clip-press-probe' else 'ETHERNET RIGHT ARM TRACKING PROBE | ' if isolated_mode else 'ETHERNET ROBOT GRASP PROBE | ') if robot else 'ETHERNET UNTOUCHED FIT | ' if a.mode=='fit-probe' else 'ETHERNET NATIVE FORCE PROBE | ')+phase,(12,25),cv2.FONT_HERSHEY_SIMPLEX,.60,(255,255,255),1,cv2.LINE_AA)
     cv2.putText(canvas,(('Ordinary robot commands. Measured score: '+str(scorer.value) if scorer else 'Ordinary robot commands; isolated diagnostic only. Task score: unscored') if robot else 'No external force or robot motion; all seven connector fits measured.' if a.mode=='fit-probe' else 'External test forces, not a robot demonstration. Task score: unscored'),(12,590),cv2.FONT_HERSHEY_SIMPLEX,.46,(255,205,120),1,cv2.LINE_AA);video.append_data(canvas)
     if step in (39,59,79,119,179,259,299,359,459,539,619,719,819):imageio.imwrite(a.out/f'step-{step}.png',canvas)
     timings['video_s']+=time.monotonic()-video_started
    if control is not None and (control.get('failure') or control.get('complete')):break
  wall_to_stop=time.monotonic()-started
  # Fit is evaluated before any bench load. Robot runs never call force().
  rest=[r for r in samples if 1.5<=r['time']<=2.000001] if a.mode!='fit-probe' else samples[-20:]
  baseline=fit_summary(rest)
  result={'diagnostic':True,'task_score':None,'robot_demo':False,'untouched_fit':baseline,
          'probe_mode':a.mode,'external_bench_forces':a.mode in ('retention-probe','clip-constraint-probe'),'no_video':a.no_video,
          'state_only_diagnostic':a.no_video,'media_eligible':not a.no_video and not isolated_mode,
          'elapsed_wall_s_to_stop':wall_to_stop,'timings_s':timings,
          'timing_scope':'physics_step_s measures env._env.step, including internal sensor rendering when enabled; video_s measures readback/encoding. Setup includes app startup. Shutdown excluded.',
          'physics_manifest':{key:physical[key] for key in ('physical_input_sha256','body_joint_mass_sha256','counts')}}
  result['native_solver_readback']=solver_readback
  result['native_actuator_readback']=actuator_readback
  if a.mode in ('retention-probe','clip-constraint-probe'):
   x0=samples[39]['plug_pose'][0];xe=samples[59]['plug_pose'][0];xr=min(r['plug_pose'][0] for r in samples[80:120])
   cliptravel=max(r['connector_fit'][str(SERVICE)]['clip_travel_m'] for r in samples[60:120])
   tips=[r['plug_pose'][0]+float((quat_xyzw_to_matrix(np.array(r['plug_pose'][3:]))@np.array((.012,0,0)))[0]) for r in samples[40:60]]
   settled_span=float(np.ptp([r['plug_pose'][0] for r in samples[50:60]]))
   result|={'engaged_pull_displacement_m':x0-xe,'pressed_withdrawal_m':x0-xr,'max_clip_travel_m':cliptravel,
            'engaged_tip_min_x_m':min(tips),'engaged_end_x_span_m':settled_span,
            'checks':{'untouched_all_connectors_fit':baseline['pass'],
                      'retains_engaged':min(tips)>(.412-.027/2)+.005 and settled_span<.0015,
                      'releases_pressed':x0-xr>.020,'clip_depresses':cliptravel>RELEASE_TRAVEL}}
  result['clip_joint_constraints']=clip_constraint_summary(samples)
  if a.mode=='clip-constraint-probe':
   result['checks']['clip_joint_constraints']=result['clip_joint_constraints']['pass']
   result['clip_joint_constraints_by_phase']={phase:clip_constraint_summary([s for s in samples if s['phase']==phase])
                                             for phase in dict.fromkeys(s['phase'] for s in samples)}
   result['offcenter_load']={'force_plug_frame_N':[0,0,-.5],'application_point_clip_frame_m':[-.010,-.0015,.0005],
                           'nominal_torque_plug_frame_Nm':[.00075,-.005,0],'torque_scope':'actual clip-frame lever cross actual plug-frame force'}
  if robot:
   tracking_by_arm={side:max(arm['position_error'] for r in samples for arm in r['robot_control']['arms'] if arm['side']==side) for side in ('left','right')}
   result|={'robot_grasp_probe':not isolated_mode,'robot_press_tracking_probe':a.mode=='press-tracking-probe','robot_clip_press_probe':a.mode=='clip-press-probe','max_tracking_error_m':max(tracking_by_arm.values()),'max_tracking_error_by_arm_m':tracking_by_arm,
            'final_plug_pose':samples[-1]['plug_pose'],'controller_failure':samples[-1]['robot_control'].get('failure'),
            'controller_complete':samples[-1]['robot_control'].get('complete',False),
            'probe_step_count':len(samples),
            'control_step_limit':step_limit,
            'control_time_limit_s':.05*step_limit,
            'probe_end_reason':'controller_completed' if samples[-1]['robot_control'].get('complete') else 'controller_failure' if samples[-1]['robot_control'].get('failure') else 'control_step_limit'}
   if isolated_mode:
    handoff=samples[-1]['robot_control'].get('functional_handoff') or {}
    error=handoff.get('max_mesh_error_5_frames_m')
    completed=bool(samples[-1]['robot_control'].get('complete') and handoff.get('stage')=='PREPRESS' and handoff.get('accepted'))
    result['isolated_tracking']={'left_hand':'READY; no source grasp','prepress_completed':completed,
                                'stopped_before_clip_press_stage':not any(s['phase']=='depress_clip' for s in samples),
                                'max_abs_measured_clip_travel_m':max(abs(s['connector_fit'][str(SERVICE)]['clip_travel_m']) for s in samples),
                                'handoff_stage':handoff.get('stage'),'max_mesh_error_5_frames_m':error,'loaded_022_useful_error_limit_m':.000291,
                                'meets_loaded_022_error_target':bool(completed and error is not None and error<=.000291),
                                'scope':'Unscored arm tracking only. Full loaded grasp, release and insertion still require native validation.'}
    if a.mode=='clip-press-probe':
     consecutive=maximum_consecutive=0
     for sample in samples:
      consecutive=consecutive+1 if sample['phase']=='depress_clip' and sample['connector_fit'][str(SERVICE)]['clip_travel_m']>=RELEASE_TRAVEL else 0
      maximum_consecutive=max(maximum_consecutive,consecutive)
     result['isolated_clip_press']={'clip_release_threshold_m':RELEASE_TRAVEL,
                                   'release_measured_for_at_least_five_control_frames':bool(samples[-1]['robot_control'].get('complete') and maximum_consecutive>=5),
                                   'maximum_consecutive_measured_release_frames':maximum_consecutive,
                                   'stopped_before_withdrawal':not any(s['phase']=='withdraw_with_clip_pressed' for s in samples),
                                   'task_score':None,'scope':'Left READY; actual clip pressure only. No unplug, transfer or insertion has been demonstrated.'}
  if scorer is not None:result|={'diagnostic':False,'privileged_robot_attempt':True,'task_score':scorer.value,'metrics':samples[-1]['metrics']}
  (a.out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
 except BaseException as error:
  import traceback
  traceback.print_exc()
  (a.out/'error.json').write_text(json.dumps({'error':str(error),'type':type(error).__name__},indent=2))
  raise
 finally:
  if video is not None:video.close()
  app.close()
if __name__=='__main__':main()
