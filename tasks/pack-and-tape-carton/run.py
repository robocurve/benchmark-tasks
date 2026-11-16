"""Native wound-ribbon bench probe; fixture/drive diagnostic, never robot credit."""
import argparse,os,json,shutil,hashlib,math
from pathlib import Path
from config import *
from carton_scene import layout

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);p.add_argument('--mode',choices=['bench','closure-probe','adhesion-probe','settle-probe','roll-pickup-probe'],default='settle-probe');p.add_argument('--no-video',action='store_true');a=p.parse_args();a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=False);(a.out/'source').mkdir()
 for f in Path(__file__).parent.glob('*.py'):shutil.copy2(f,a.out/'source'/f.name)
 (a.out/'source-sha256.json').write_text(json.dumps({f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in Path(__file__).parent.glob('*.py')},indent=2));os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES')
 from isaaclab.app import AppLauncher
 app=AppLauncher(headless=True,enable_cameras=not a.no_video).app;video=None;contacts=None
 try:
  import numpy as np,torch,cv2,imageio.v2 as imageio
  from yam_arena import inspect_embodiment as wrapper
  from yam_arena import scene as sc
  from yam_arena.urdf import quat_xyzw_to_matrix
  from pxr import UsdPhysics,Gf
  import isaaclab.sim as sim
  from carton_native import export as cexport
  from tape_native import export as texport,install_joints
  import carton_scene
  carton_scene.INITIAL_TOP_ANGLE=160. if a.mode=='closure-probe' else 0.
  import task
  task_path=Path(__file__).with_name('task.py')
  coupon_size=(.10,.10,.04);coupon_pos=(.15,.12,TOP+.022);coupon_start=np.array((.15,.12,TOP+.042+TAPE_THICKNESS/2+.0002))
  if a.mode=='adhesion-probe':
   task_path=a.out/'probe_task.py';task_path.write_text(Path(__file__).with_name('task.py').read_text()+"\nOBJECTS.append(Box('adhesion_coupon',pos="+repr(coupon_pos)+",size="+repr(coupon_size)+",mass=5.,color=(.3,.6,.8)))\n")
  wrapper._APP=app;parts,hs=layout();byname={p['name']:p for p in parts};assets={}
  for spec in task.OBJECTS:
   if spec.name in byname or spec.name=='cutter':assets[spec.name]=cexport(a.out/(spec.name+'.usda'),spec.name)
   elif spec.name=='roll' or spec.name.startswith('tape_'):assets[spec.name]=texport(a.out/(spec.name+'.usda'),spec.name)
  original=sc.task_object;original_dt=sc.PHYSICS_DT;original_callback=sc.env_cfg_callback
  def contact_callback(hz=None):
   original_configure=original_callback(hz)
   def configure(cfg):
    cfg=original_configure(cfg)
    if hasattr(cfg.sim,'disable_contact_processing'):cfg.sim.disable_contact_processing=False
    return cfg
   return configure
  sc.env_cfg_callback=contact_callback
  sc.PHYSICS_DT=.0005 if a.mode in ('bench','adhesion-probe') else 1/240
  def make(spec):
   if spec.name not in assets:return original(spec)
   q=byname[spec.name]['quat'] if spec.name in byname else (math.cos(spec.yaw/2),0,0,math.sin(spec.yaw/2))
   cfg=sim.UsdFileCfg(usd_path=assets[spec.name])
   if spec.name=='cutter':
    spawn=cfg.func
    def with_joints(path,cfg,translation=None,orientation=None,**kwargs):
     prim=spawn(path,cfg,translation=translation,orientation=orientation,**kwargs);s=prim.GetStage();parent=str(prim.GetPath()).rsplit('/',1)[0];install_joints(s,parent)
     if a.mode=='adhesion-probe':
      for name in ['wound_tack','film_bend']:UsdPhysics.Joint(s.GetPrimAtPath(parent+f'/tape_{CELLS-1}/'+name)).GetJointEnabledAttr().Set(False)
     # Explicit test-bench fixtures only. Not present in a real task episode.
     if a.mode!='bench':return prim
     j=UsdPhysics.FixedJoint.Define(s,parent+'/roll/bench_fixture');j.CreateBody1Rel().SetTargets([parent+'/roll']);j.CreateLocalPos0Attr(Gf.Vec3f(*ROLL_START));j.CreateLocalPos1Attr(Gf.Vec3f(0,0,0));j.CreateExcludeFromArticulationAttr(True)
     point,yaw=tape_cell(CELLS-1);j=UsdPhysics.PrismaticJoint.Define(s,parent+f'/tape_{CELLS-1}/bench_peel');j.CreateBody1Rel().SetTargets([parent+f'/tape_{CELLS-1}']);j.CreateAxisAttr('X');j.CreateLocalPos0Attr(Gf.Vec3f(*point));j.CreateLocalPos1Attr(Gf.Vec3f(0,0,0));j.CreateLocalRot1Attr(Gf.Quatf(math.cos(yaw/2),Gf.Vec3f(0,0,-math.sin(yaw/2))));j.CreateLowerLimitAttr(0.);j.CreateUpperLimitAttr(.07);j.CreateExcludeFromArticulationAttr(True)
     d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'linear');d.CreateTypeAttr('force');d.CreateStiffnessAttr(30.);d.CreateDampingAttr(.1);d.CreateMaxForceAttr(3.);d.CreateTargetPositionAttr(0.)
     return prim
    cfg.func=with_joints
   pos=tuple(u+v for u,v in zip(CENTER,byname[spec.name]['pos'])) if spec.name in byname else spec.pos
   if a.mode=='adhesion-probe' and spec.name==f'tape_{CELLS-1}':pos=tuple(float(v) for v in coupon_start);q=(math.sqrt(.5),-math.sqrt(.5),0.,0.)
   return sc.Object(name=spec.name,object_type=sc.ObjectType.RIGID,spawner_cfg=cfg,initial_pose=sc.Pose(position_xyz=pos,rotation_xyzw=(*q[1:],q[0])))
  from yam_arena.embodiment import BimanualYamEmbodiment
  import review
  original_init=BimanualYamEmbodiment.__init__
  def with_reviews(instance,*args,**kwargs):
   kwargs['cam_width']=320;kwargs['cam_height']=180;kwargs['enable_cameras']=not a.no_video;original_init(instance,*args,**kwargs)
   if not a.no_video:review.add_cameras(instance)
   from contact_runtime import add_contact_sensor
   add_contact_sensor(instance,[*[p['name'] for p in parts],*(['adhesion_coupon'] if a.mode=='adhesion-probe' else []),'cutter'])
   if a.mode=='roll-pickup-probe':
    from roll_contacts import add_sensor
    add_sensor(instance)
  BimanualYamEmbodiment.__init__=with_reviews
  sc.task_object=make
  try:
   env=wrapper.YamArenaEmbodiment(task=str(task_path),object_jitter=0.,control_hz=20,headless=True,cam_width=320,cam_height=180);env._ensure_sim(None)
  finally:sc.task_object=original;sc.PHYSICS_DT=original_dt;sc.env_cfg_callback=original_callback;BimanualYamEmbodiment.__init__=original_init
  base=env._base
  if a.no_video:base.sim.render=lambda *args,**kwargs:None
  env._env.reset();env._instruction='Finite wound-ribbon material probe';env._t=0.
  if not a.no_video:review.place(base,(.30,0.,.82))
  import native_capture as capture
  from yam_arena.robot import asset_link_paths
  capture_names=[s.name for s in task.OBJECTS if s.name!='mat']
  owners=capture.native_body_owners(base.scene,asset_link_paths(),capture_names)
  for path in owners:
   if not base.scene.stage.GetPrimAtPath(path):raise RuntimeError('missing native capture owner '+path)
  view_cfg=sim.PinholeCameraCfg(focal_length=18.,clipping_range=(.03,20.))
  manifest=capture.capture_manifest(capture.export_scene(base.scene.stage,a.out/'capture-scene.usdc'),owners,json.loads((a.out/'source-sha256.json').read_text()),capture.virtual_review_views(view_cfg),no_video=a.no_video)
  (a.out/'capture-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
  from contact_runtime import ContactSubsteps
  from seal_geometry import bonded_coverage,strip_component
  target_sizes={p['name']:p['size'] for p in parts}
  if a.mode=='adhesion-probe':target_sizes['adhesion_coupon']=coupon_size
  contacts=ContactSubsteps(base,target_sizes,enabled=a.mode!='bench')
  drive=UsdPhysics.DriveAPI(base.scene.stage.GetPrimAtPath(f'/World/envs/env_0/tape_{CELLS-1}/bench_peel'),'linear') if a.mode=='bench' else None
  def arr(v):
   v=v.torch if hasattr(v,'torch') else v
   return v.detach().cpu().numpy()
  def pose(name):return arr(base.arena_world.get_pose_w(name))[0]
  # Keep exact native break payloads for interpretation; never infer from phase.
  events=[];subscription=None
  try:
   import omni.physx
   stream=omni.physx.get_physx_interface().get_simulation_event_stream_v2()
   def event(e):events.append({'time':env._t,'type':int(e.type),'payload':str(e.payload)})
   subscription=stream.create_subscription_to_pop(event)
  except Exception as error:events.append({'subscription_error':str(error)})
  if not a.no_video:video=imageio.get_writer(str(a.out/'diagnostic.mp4'),fps=20,codec='libx264',pixelformat='yuv420p',macro_block_size=1,ffmpeg_params=['-movflags','+faststart'])
  worst=0.;max_unwound=0.;allposes=[];initial_displacement=0.;robot=None;tracker=None;previous_items=None;previous_parts=None
  if a.mode in ('closure-probe','settle-probe','roll-pickup-probe'):
   from robot import ClosureProbe
   from scoring import PackingScorer
   from yam_arena import ik
   robot=ClosureProbe(env) if a.mode=='closure-probe' else None;tracker=PackingScorer();task._tracker=tracker;task._diagnostic=False
  contact_reader=None
  if a.mode=='roll-pickup-probe':
   from roll_probe import RollPickupProbe
   from roll_contacts import Reader
   contact_reader=Reader(base,env);robot=RollPickupProbe(env,contact_reader)
  steps_completed=0
  with (a.out/'trajectory.jsonl').open('w') as trace:
   for step in range(1200 if a.mode=='roll-pickup-probe' else 3600 if a.mode=='closure-probe' else 40 if a.mode=='settle-probe' else 120 if a.mode=='adhesion-probe' else 100):
    phase='preloaded_stability' if a.mode=='settle-probe' else 'settle' if step<20 else 'peel_bench' if step<60 else 'hold'
    target=.06*min(1.,max(0.,(step-20)/40))
    control=None
    if robot:
     action,control=robot.command(step*.05);phase=control['phase'];action=torch.tensor(action[None,:],device=base.device,dtype=torch.float32)
    elif a.mode=='adhesion-probe':
     phase='press_coupon' if step<20 else 'release_pressure' if step<40 else 'retention_pull' if step<60 else 'overload_break' if step<80 else 'released_coupon'
     f=-.7 if step<20 else .5 if 40<=step<60 else 3. if 60<=step<80 else 0.
     base.scene[f'tape_{CELLS-1}'].permanent_wrench_composer.set_forces_and_torques_index(forces=torch.tensor([[[0.,0.,f]]],device=base.device),torques=torch.zeros((1,1,3),device=base.device),is_global=True);action=env._ready
    elif a.mode=='bench':drive.GetTargetPositionAttr().Set(target);action=env._ready
    else:action=env._ready
    if control is not None and control.get('failure'):
     (a.out/'rejected-command.json').write_text(json.dumps({'time':env._t,'not_issued':True,'control':control},indent=2)+'\n');break
    env._env.step(action);env._t=(step+1)*.05;steps_completed+=1
    roll=pose('roll');rr=quat_xyzw_to_matrix(roll[3:]);tensors=[base.arena_world.get_pose_w(f'tape_{i}') for i in range(CELLS)];states=torch.cat([v.torch if hasattr(v,'torch') else v for v in tensors],dim=0).detach().cpu().numpy();local=[rr.T@(v[:3]-roll[:3]) for v in states];displaced=[];sep=0.;anchor_errors={}
    for i,v in enumerate(local):
     pos,_=tape_cell(i);displaced.append(float(np.linalg.norm(v-(np.array(pos)-np.array(ROLL_START)))))
     if i and not (a.mode=='adhesion-probe' and i==CELLS-1):
      # Recompute the actual authored anchors in each measured film frame.
      p0,y0=tape_cell(i-1);p1,y1=tape_cell(i);anchor=(np.array(p0)+p1)/2
      def localpoint(origin,yaw):
       r=np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
       return r.T@(anchor-np.array(origin))
      a0=states[i-1][:3]+quat_xyzw_to_matrix(states[i-1][3:])@localpoint(p0,y0);a1=states[i][:3]+quat_xyzw_to_matrix(states[i][3:])@localpoint(p1,y1);anchor_errors[i]=float(np.linalg.norm(a0-a1))
      if i not in contacts.blade.cuts:sep=max(sep,anchor_errors[i])
    if step<20:initial_displacement=max(initial_displacement,max(displaced[:-1] if a.mode=='adhesion-probe' else displaced))
    if step==19 and initial_displacement>.0005:raise RuntimeError('tape shifted before peel loading; initial wrap invalid')
    worst=max(worst,sep);unwound=sum(d>.003 for d in displaced)*CELL_LENGTH;max_unwound=max(max_unwound,unwound)
    row={'time':env._t,'phase':phase,'robot_control':control,'bench_target_m':target,'roll_pose':roll.tolist(),'film_poses':[s.tolist() for s in states],'native_cell_displacement_m':displaced,'material_anchor_separation_m':sep,'unwound_length_estimate_m':unwound,'native_contact':contacts.telemetry()}
    if robot:
     native_hands=[]
     for side in ['left','right']:
      ids,fids=env._ids[side+'_robot'];native=arr(base.scene[side+'_robot'].data.joint_pos)[0];tcp,rotation=ik.tcp_pose(native[ids],env.rig.bases[side]);native_hands.append({'side':side,'joints':native[ids].tolist(),'finger_joints':native[fids].tolist(),'tcp':tcp.tolist(),'tcp_rotation':rotation.tolist()})
     row.update(action_14d=arr(action)[0].tolist(),native_hands=native_hands)
    if a.mode=='adhesion-probe':row['coupon_probe']={'film_height_m':float(states[-1,2]-coupon_start[2]),'external_force_z_n':f,'active_bond':(f'tape_{CELLS-1}','adhesion_coupon') in contacts.adhesion.bonds and (f'tape_{CELLS-1}','adhesion_coupon') not in contacts.adhesion.broken}
    if tracker is not None:
     pp={p['name']:pose(p['name']) for p in parts};rr={n:quat_xyzw_to_matrix(p[3:]) for n,p in pp.items()};reference=rr['north'];origin=pp['north'][:3]-reference@np.array((0,CARTON_WIDTH/2,CARTON_HEIGHT/2+BOARD/2));crease=0.
     for h in hs:
      crease=max(crease,float(np.linalg.norm(pp[h['a']][:3]+rr[h['a']]@np.array(h['anchor_a'])-pp[h['b']][:3]-rr[h['b']]@np.array(h['anchor_b']))))
     item_states=[];current_items={}
     for name,_,size,_,_ in PRODUCTS:
      body=pose(name);r=quat_xyzw_to_matrix(body[3:]);corners=[]
      for x in [-1,1]:
       for y in [-1,1]:
        for z in [-1,1]:corners.append(reference.T@(body[:3]+r@(np.array(size)*np.array((x,y,z))/2)-origin))
      corners=np.array(corners);speed=0. if previous_items is None else float(np.linalg.norm(body[:3]-previous_items[name])/.05);current_items[name]=body[:3].copy();item_states.append({'local_min':corners.min(0).tolist(),'local_max':corners.max(0).tolist(),'speed':speed})
     carton_vertices={}
     for p in parts:
      vertices=[]
      for x in [-1,1]:
       for y in [-1,1]:
        for z in [-1,1]:vertices.append(reference.T@(pp[p['name']][:3]+rr[p['name']]@(np.array(p['size'])*np.array((x,y,z))/2)-origin))
      carton_vertices[p['name']]=np.array(vertices)
     inner_under=max(carton_vertices[n+'_top'][:,2].max() for n in ['east','west'])<=min(carton_vertices[n+'_top'][:,2].min() for n in ['north','south'])+.0005
     vmin=np.min(np.concatenate(list(carton_vertices.values())),axis=0);vmax=np.max(np.concatenate(list(carton_vertices.values())),axis=0);hands=[]
     for side in ['left','right']:
      ids,_=env._ids[side+'_robot'];q=arr(base.scene[side+'_robot'].data.joint_pos)[0,ids];hands.append(reference.T@(ik.tcp_pose(q,env.rig.bases[side])[0]-origin))
     hand_clear=all(np.linalg.norm(h-np.clip(h,vmin,vmax))>.07 for h in hands);tool=pose('cutter');tools_parked=np.linalg.norm(tool[:3]-np.array(CUTTER_START))<.05 and np.linalg.norm(roll[:3]-np.array(ROLL_START))<.05
     angles=[]
     for n in ['north','south','east','west']:
      r=rr[n].T@rr[n+'_top'];angles.append(float(np.degrees(np.arctan2(r[2,1],r[1,1]))))
     speed=0. if previous_parts is None else max(float(np.linalg.norm(pp[n][:3]-previous_parts[n])/.05) for n in pp);previous_parts={n:p[:3].copy() for n,p in pp.items()};previous_items=current_items
     peeled={i for i,d in enumerate(displaced) if d>.003};payout=max(tracker.tape.paid_out,len(peeled)*CELL_LENGTH);film_length=payout+sum(e for i,e in anchor_errors.items() if i in peeled and i-1 in peeled and i not in contacts.blade.cuts)
     films={f'tape_{i}':{'pos':body[:3],'rotation':quat_xyzw_to_matrix(body[3:])} for i,body in enumerate(states)}
     component=strip_component(contacts.blade.cuts,anchor_errors)
     seam_bond,end_bond=bonded_coverage(films,contacts.adhesion.active_patches(),component,origin,reference)
     detached=bool(component) and component<=peeled
     cut_coordinate=contacts.cut_coordinates[0] if len(contacts.cut_coordinates)==1 else None
     sample={'time':env._t,'payout':payout,'ribbon_length':film_length,'flap_angles':angles,'items':item_states,'seam_bond':seam_bond,'end_bond':end_bond,'carton_speed':speed,'crease_separation':crease,'max_strain':sep/CELL_LENGTH,'canary_min_height':min(min(i['local_min'][2]+origin[2]-TOP for i in item_states),min(v[:,2].min()+origin[2]-TOP for v in carton_vertices.values())),'inner_flaps_under_outer':bool(inner_under),'continuous_material':bool(component) and len(component)*CELL_LENGTH>=.055,'single_strip':bool(len(contacts.blade.cuts)==1 and len(component)*CELL_LENGTH>=MIN_STRIP),'strip_still_on_roll':not detached,'hands_clear':bool(hand_clear),'tools_parked':bool(tools_parked)}
     if cut_coordinate is not None:sample|={'cut_coordinate':cut_coordinate,'blade_crossing':True}
     row['score_sample']=sample;row['metrics']=tracker.update(sample)
    captured={name:pose(name) for name in capture_names}
    row['action_14d']=arr(action)[0].tolist()
    if contact_reader is not None:row['roll_grip_contact']=contact_reader.latest
    row['native_body_world_poses']=capture.native_body_world_poses(base.scene,owners,captured)
    trace.write(json.dumps(row)+'\n')
    if not np.isfinite(states).all() or sep>.02:raise RuntimeError('unstable ribbon or disconnected material')
    if not a.no_video:
     panels=review.panels(base);canvas=np.zeros((594,960,3),dtype=np.uint8);canvas[36:306,240:720]=panels[0];canvas[306:576,:480]=panels[1];canvas[306:576,480:]=panels[2]
     cv2.putText(canvas,(('ROBOT FLAP CLOSURE | ' if a.mode=='closure-probe' else 'ROBOT PRODUCT PACKING | ') if robot else 'NATIVE WOUND TAPE | ')+phase,(12,25),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1,cv2.LINE_AA);cv2.putText(canvas,('Actual robot commands. Packing score: '+str(tracker.value)+' | sealing incomplete' if robot else 'Clamped material bench, not robot packing. No tape pose animation.'),(12,590),cv2.FONT_HERSHEY_SIMPLEX,.46,(255,205,120),1,cv2.LINE_AA);video.append_data(canvas)
     if step in (19,59,99,179,259,339,419,499,579,659,719) or (a.mode=='closure-probe' and step%200==199):imageio.imwrite(a.out/f'step-{step}.png',canvas)
    if robot is not None and (getattr(robot,'failure',None) or getattr(robot,'done',False)):break
  result={'diagnostic':True,'robot_demo':False,'task_score':None,'bench_fixtures':True,'physics_dt_s':float(base.sim.get_physics_dt()),'explicit_material_length_m':ACTIVE_LENGTH,'initial_wrap_displacement_m':initial_displacement,'max_anchor_separation_m':worst,'max_displaced_material_estimate_m':max_unwound,'native_contact':contacts.telemetry(),'contact_integration_native_validated':False,'checks':{'stays_wound_before_pull':initial_displacement<.0005,'material_connected':worst<CELL_LENGTH*MAX_STRAIN,'material_peels':max_unwound>.025}}
  if a.mode=='adhesion-probe':result|={'diagnostic':True,'task_score':None,'coupon_probe':True,'bench_fixtures':True,'final_coupon_film_height_m':float(states[-1,2]-coupon_start[2])}
  if tracker is not None:result|={'diagnostic':False,'bench_fixtures':False,'privileged_robot_attempt':robot is not None,'task_score':tracker.value,'metrics':tracker.result(),'controller_failure':getattr(robot,'failure',None),'controller_completed':getattr(robot,'done',False),'control_steps':steps_completed,'elapsed_s':env._t}
  if a.mode=='roll-pickup-probe':
   result|={'diagnostic':True,'scorer_readback':result.get('task_score'),'task_score':None,'scope':'Unscored loose-roll acquisition checkpoint; no peeling/sealing demonstration'}
  result|={'no_video':a.no_video,'state_only_diagnostic':a.no_video,'media_eligible':False}
  (a.out/'native-events.json').write_text(json.dumps(events,indent=2)+'\n');(a.out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
 except BaseException as error:
  import traceback;traceback.print_exc();(a.out/'error.json').write_text(json.dumps({'error':str(error),'type':type(error).__name__},indent=2));raise
 finally:
  if contacts is not None:contacts.close()
  if video is not None:video.close()
  app.close()
if __name__=='__main__':main()
