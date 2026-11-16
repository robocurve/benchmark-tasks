"""Native free-carton crease/stability probe. No robot completion is claimed."""
import argparse,os,json,shutil,hashlib
from pathlib import Path
from config import *
from scene import layout
from model import WIDTH,HEIGHT
from crease import physical_bottom_angle,joint_parameters

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);p.add_argument('--mode',choices=['dynamics','grip-probe','cut-probe','open-probe','turnover-probe','turn-and-cut-probe','setup-check'],default='dynamics');p.add_argument('--bottom-up-start',action='store_true',help='Legacy bottom-up engineering setup, not the revised full task');p.add_argument('--unsealed',action='store_true',help='Explicit unscored starting condition for folding diagnostics');p.add_argument('--no-video',action='store_true',help='State-only engineering run: omit camera sensors/rendering and record exact native poses, without eligible media');a=p.parse_args();
 if a.mode=='cut-probe' and not a.bottom_up_start:p.error('legacy cut-probe requires explicit --bottom-up-start; full task starts upright')
 if a.mode=='open-probe' and not a.unsealed:p.error('open-probe requires --unsealed and cannot count as task completion')
 a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=False);(a.out/'source').mkdir()
 for f in Path(__file__).parent.glob('*.py'):shutil.copy2(f,a.out/'source'/f.name)
 (a.out/'source-sha256.json').write_text(json.dumps({f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in Path(__file__).parent.glob('*.py')},indent=2))
 os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES')
 from isaaclab.app import AppLauncher
 app=AppLauncher(headless=True,enable_cameras=not a.no_video).app;video=None
 try:
  import numpy as np,torch
  if not a.no_video:import cv2,imageio.v2 as imageio
  from yam_arena import inspect_embodiment as wrapper
  from yam_arena import scene as sc
  from yam_arena.urdf import quat_xyzw_to_matrix
  import isaaclab.sim as sim
  from native import export,install_joints
  from scoring import BreakdownScorer
  from yam_arena import ik
  from yam_arena.constants import FINGER_OPEN
  from pxr import UsdPhysics
  import task
  wrapper._APP=app;parts,hs=layout();byname={s['name']:s for s in parts};assets={s.name:export(a.out/(s.name+'.usda'),s.name) for s in task.OBJECTS if s.name!='cutter_pad'};original=sc.task_object
  def make(spec):
   if spec.name not in assets:return original(spec)
   cfg=sim.UsdFileCfg(usd_path=assets[spec.name]);pos,q=starting_pose(byname[spec.name],upright=not a.bottom_up_start) if spec.name in byname else (spec.pos,(1,0,0,0))
   if spec.name=='cutter':
    spawn=cfg.func
    def with_joints(path,cfg,translation=None,orientation=None,**kwargs):
     prim=spawn(path,cfg,translation=translation,orientation=orientation,**kwargs);install_joints(prim.GetStage(),str(prim.GetPath()).rsplit('/',1)[0],sealed=not a.unsealed);return prim
    cfg.func=with_joints
   return sc.Object(name=spec.name,object_type=sc.ObjectType.RIGID,spawner_cfg=cfg,initial_pose=sc.Pose(position_xyz=pos,rotation_xyzw=(*q[1:],q[0])))
  from yam_arena.embodiment import BimanualYamEmbodiment
  import review
  import capture
  original_init=BimanualYamEmbodiment.__init__
  def with_reviews(instance,*args,**kwargs):
   capture.configure_embodiment(instance,args,kwargs,original_init,review,no_video=a.no_video)
  BimanualYamEmbodiment.__init__=with_reviews
  sc.task_object=make
  try:
   env=wrapper.YamArenaEmbodiment(task=str(Path(__file__).with_name('task.py')),object_jitter=0.,control_hz=20,headless=True,cam_width=320,cam_height=180);env._ensure_sim(None)
  finally:sc.task_object=original;BimanualYamEmbodiment.__init__=original_init
  base=env._base
  capture.suppress_state_only_rendering(base,no_video=a.no_video)
  env._env.reset();env._instruction='Native carton mechanics probe';env._t=0.
  from native_angle_binding import install as install_angle_observer
  angle_observer=install_angle_observer(base)
  from native_angle_binding import PINS
  (a.out/'angle-tracking-reset.json').write_text(json.dumps(dict(version=__import__('angle_tracking').VERSION,source_pins=PINS,caps_deg_s=angle_observer.tracker.caps,initial_state=angle_observer.initial_state),indent=2)+'\n')
  if not a.no_video:review.place(base,(.32,0.,.82))
  from yam_arena.robot import asset_link_paths
  owners=capture.native_body_owners(base.scene,asset_link_paths(),[part['name'] for part in parts])
  for path in owners:
   if not base.scene.stage.GetPrimAtPath(path):raise RuntimeError('missing native capture owner '+path)
  view_cfg=sim.PinholeCameraCfg(focal_length=18.,clipping_range=(.03,20.)) if a.no_video else base.scene[review.NAMES[0]].cfg.spawn
  manifest=capture.capture_manifest(capture.export_scene(base.scene.stage,a.out/'capture-scene.usdc'),owners,
      json.loads((a.out/'source-sha256.json').read_text()),capture.virtual_review_views(view_cfg),no_video=a.no_video)
  (a.out/'capture-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
  (a.out/'mode.json').write_text(json.dumps({'mode':a.mode,'unsealed_start':a.unsealed,'state_only':a.no_video,'camera_observations':not a.no_video,'media_eligible':False,'control_hz':20,'cut_probe_max_control_steps':4800,'cut_probe_limit_s':240.},indent=2)+'\n')
  def arr(v):
   v=v.torch if hasattr(v,'torch') else v
   return v.detach().cpu().numpy()
  def pose(name):return arr(base.arena_world.get_pose_w(name))[0]
  if not a.no_video:video=imageio.get_writer(str(a.out/'diagnostic.mp4'),fps=20,codec='libx264',pixelformat='yuv420p',macro_block_size=1,ffmpeg_params=['-movflags','+faststart'])
  worst=0.;minz=10.;bestflat=10.;robot=None
  if a.mode in ('grip-probe','cut-probe','open-probe','turnover-probe','turn-and-cut-probe'):
   from robot import CartonGripProbe,CartonCutProbe,CartonOpenProbe
   from turnover import CartonTurnoverProbe
   robot=({'cut-probe':CartonCutProbe,'open-probe':CartonOpenProbe,'turnover-probe':CartonTurnoverProbe,'turn-and-cut-probe':CartonTurnoverProbe}.get(a.mode,CartonGripProbe))(env)
  tracker=BreakdownScorer(require_turnover=a.mode=='turn-and-cut-probe') if a.mode in ('cut-probe','turn-and-cut-probe') else None;released=False;previous_positions=None;previous_cutter=None
  if tracker is not None:task._tracker=tracker;task._diagnostic=False
  with (a.out/'trajectory.jsonl').open('w') as trace:
   for step in range(2400 if a.mode=='open-probe' else 6000 if a.mode=='turn-and-cut-probe' else 4800 if a.mode=='cut-probe' else 1600 if a.mode=='turnover-probe' else 440 if robot else 180):
    phase='settle' if step<60 else 'wall_load' if step<120 else 'release'
    f=(0.,-.25,0.) if phase=='wall_load' and not robot and a.mode!='setup-check' else (0.,0.,0.)
    base.scene['east'].permanent_wrench_composer.set_forces_and_torques_index(forces=torch.tensor([[f]],device=base.device),torques=torch.zeros((1,1,3),device=base.device),is_global=True)
    control=None
    if robot:
     target,control=robot.command(step*.05-(cut_started if "cut_started" in locals() else 0.));phase=control[0]['phase'];action=torch.tensor(target[None,:],device=base.device,dtype=torch.float32)
    else:action=env._ready
    if robot is not None and getattr(robot,'failure',None):
     (a.out/'rejected-command.json').write_text(json.dumps({'time':env._t,'control':control,'failure':robot.failure},indent=2)+'\n');break
    angle_observer.begin_control()
    env._env.step(action);env._t=(step+1)*.05
    if len(base._reset_env_ids):angle_observer.tracker.invalidate('native episode reset')
    angle_evidence=angle_observer.end_control()
    if not angle_evidence['valid']:raise RuntimeError(angle_evidence['failure'])
    pp={s['name']:pose(s['name']) for s in parts};rr={n:quat_xyzw_to_matrix(v[3:]) for n,v in pp.items()};sep=0.
    angles={}
    for h in hs:
     pa=pp[h['a']][:3]+rr[h['a']]@np.array(h['anchor_a']);pb=pp[h['b']][:3]+rr[h['b']]@np.array(h['anchor_b']);sep=max(sep,float(np.linalg.norm(pa-pb)))
     rel=rr[h['a']].T@rr[h['b']]
     raw=float(np.degrees(np.arctan2(rel[2,1],rel[1,1]) if h['axis']=='X' else np.arctan2(rel[1,0],rel[0,0])))
     angles[h['name']]=float(angle_observer.tracker.angles[['north','south','east','west'].index(h['a'])]) if h['b'].endswith('_bottom') else raw
    vertices=[]
    for s in parts:
     for x in [-1,1]:
      for y in [-1,1]:
       for z in [-1,1]:vertices.append(pp[s['name']][:3]+rr[s['name']]@(.5*np.array(s['size'])*np.array((x,y,z))))
    vertices=np.array(vertices);low=float(vertices[:,2].min()-TOP);high=float(vertices[:,2].max()-TOP);worst=max(worst,sep);minz=min(minz,low);bestflat=min(bestflat,high-low)
    native_hands=[]
    for side in ['left','right']:
     ids,fids=env._ids[side+'_robot'];qnative=arr(base.scene[side+'_robot'].data.joint_pos)[0];tcpnative,rnative=ik.tcp_pose(qnative[ids],env.rig.bases[side]);native_hands.append({'side':side,'joints':qnative[ids].tolist(),'finger_joints':qnative[fids].tolist(),'tcp':tcpnative.tolist(),'tcp_rotation':rnative.tolist(),'opening':float(np.clip(qnative[fids].mean()/FINGER_OPEN,0,1))})
    row={'time':env._t,'phase':phase,'robot_control':control,'action_14d':arr(action)[0].tolist(),'native_hands':native_hands,'cutter_pose':pose('cutter').tolist(),'poses':{n:v.tolist() for n,v in pp.items()},'angles_deg':angles,'max_crease_separation_m':sep,'lowest_vertex_m':low,'highest_vertex_m':high,'force_N':f}
    row['angle_evidence']=angle_evidence
    row['angle_substeps']=angle_observer.samples
    row['native_owner_poses']=capture.native_body_world_poses(base.scene,owners,{**pp,'cutter':row['cutter_pose']})
    if tracker is not None:
     cutter=pose('cutter');cr=quat_xyzw_to_matrix(cutter[3:]);tip=cutter[:3]+cr@np.array((.062,0,-.020));reference=rr['north'];origin=pp['north'][:3]-reference@np.array((0,WIDTH/2,HEIGHT/2));localtip=reference.T@(tip-origin);hands=[];held=False
     for side in ['left','right']:
      ids,fids=env._ids[side+'_robot'];q=arr(base.scene[side+'_robot'].data.joint_pos)[0];tcp,rotation=ik.tcp_pose(q[ids],env.rig.bases[side]);opening=float(np.clip(q[fids].mean()/FINGER_OPEN,0,1));hands.append(tcp);held|=np.linalg.norm(tcp-cutter[:3])<.035 and .08<opening<.38
     bottom=[angles[n+'_bottom_hinge'] for n in ['north','south','east','west']];top=[(-angles[n+'_top_hinge'])%360 for n in ['north','south','east','west']];top=[0. if x>350 else x for x in top]
     speed=0. if previous_positions is None else max(float(np.linalg.norm(pp[n][:3]-previous_positions[n])/ .05) for n in pp);cspeed=0. if previous_cutter is None else float(np.linalg.norm(cutter[:3]-previous_cutter)/.05);previous_positions={n:p[:3].copy() for n,p in pp.items()};previous_cutter=cutter[:3].copy();vmin=vertices.min(0);vmax=vertices.max(0)
     sample={'body_up_dot':float(reference[2,2]),'angle_evidence':angle_evidence,'time':env._t,'blade_tip_local':localtip.tolist(),'blade_alignment':float(-np.dot(cr[:,2],reference[:,2])),'cutter_held':bool(held),'bottom_angles':bottom,'top_angles':top,'crease_separation':sep,'min_height':low,'max_height':high,'carton_speed':speed,'cutter_height':float(cutter[2]-TOP),'cutter_parked':bool(np.linalg.norm(cutter[:3]-np.array(CUTTER_START))<.05 and cspeed<.02),'hands_clear':all(np.linalg.norm(h-np.clip(h,vmin,vmax))>.07 and np.linalg.norm(h-cutter[:3])>.065 for h in hands)}
     row['score_sample']=sample;row['metrics']=tracker.update(sample);row['cutter_pose']=cutter.tolist()
     if tracker.coverage.complete and not released:
      for name in ['north','south']:
       joint=UsdPhysics.RevoluteJoint(base.scene.stage.GetPrimAtPath(f'/World/envs/env_0/{name}_bottom/{name}_bottom_hinge'));parameters=joint_parameters(next(h for h in hs if h['name']==name+'_bottom_hinge'));joint.GetLowerLimitAttr().Set(parameters['low']);joint.GetUpperLimitAttr().Set(parameters['high'])
      released=True
    trace.write(json.dumps(row)+'\n')
    if not np.isfinite(vertices).all() or sep>.03:raise RuntimeError('unstable carton/crease failure')
    if not a.no_video:
     panels=review.panels(base);canvas=np.zeros((594,960,3),dtype=np.uint8);canvas[36:306,240:720]=panels[0];canvas[306:576,:480]=panels[1];canvas[306:576,480:]=panels[2]
     cv2.putText(canvas,('CARTON ROBOT GRIP PROBE | ' if robot else 'NATIVE CARTON | ')+phase+(' | unsealed diagnostic' if a.unsealed else ' | sealed bottom'),(12,25),cv2.FONT_HERSHEY_SIMPLEX,.57,(255,255,255),1,cv2.LINE_AA);cv2.putText(canvas,(('Measured cutting score: '+str(tracker.value)+' | incomplete task attempt' if tracker else 'Ordinary robot commands. Rim grip/lift diagnostic, not task completion.') if robot else 'Free rigid panels and physical creases. Force probe, not robot completion.'),(12,590),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,205,120),1,cv2.LINE_AA);video.append_data(canvas)
     if step in (59,119,179,279,399,599,799,999,1199):imageio.imwrite(a.out/f'step-{step}.png',canvas)
    if a.mode=='turn-and-cut-probe' and isinstance(robot,CartonTurnoverProbe) and robot.done:
     robot=CartonCutProbe(env);robot.previous=arr(action)[0].copy();cut_started=env._t
    elif robot is not None and (getattr(robot,"failure",None) or getattr(robot,"done",False)):break
    if a.mode=='open-probe' and robot.index>=4:break
  result={'diagnostic':True,'task_score':None,'unsealed_start':a.unsealed,'max_crease_separation_m':worst,'lowest_vertex_m':minz,'smallest_height_span_m':bestflat,'checks':{'creases_intact':worst<.004,'above_table':minz>-.004}}
  if a.mode=='turnover-probe':result|={'controller_failure':robot.failure,'turnover_completed':robot.done,'final_roll_angle_deg':robot.angle}
  if a.mode=='open-probe':result|={'controller_failure':robot.failure,'flaps_completed':robot.index,'final_angles_deg':angles}
  if tracker is not None:result|={'diagnostic':False,'task_score':tracker.value,'privileged_robot_attempt':True,'metrics':tracker.result(),'bottom_up_start':a.bottom_up_start,'controller_failure':robot.failure,'controller_completed_cut_attempt':robot.done}
  result|={'state_only':a.no_video,'media_eligible':False,'capture_manifest':'capture-manifest.json'}
  if a.no_video:result|={'diagnostic':True,'diagnostic_reason':'camera-free engineering run; measured scorer values retained, no final media'}
  (a.out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
 except BaseException as error:
  import traceback;traceback.print_exc();(a.out/'error.json').write_text(json.dumps({'error':str(error),'type':type(error).__name__},indent=2));raise
 finally:
  if video is not None:video.close()
  app.close()
if __name__=='__main__':main()
