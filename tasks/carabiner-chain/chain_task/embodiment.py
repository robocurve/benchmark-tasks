"""Native-camera YAM environment with spring gates and physics-step link audit."""
import atexit,hashlib,json,os
from pathlib import Path
from dataclasses import replace
import numpy as np
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.urdf import quat_xyzw_to_matrix
from chain_task.geometry import *
from chain_task.scoring import Body,History,curves_and_closed,segments
from chain_task.topology import CrossingAudit,RodAudit
ROOT=Path(__file__).resolve().parent.parent
FEEDBACK={}

def array(v):
 v=v.torch if hasattr(v,'torch') else v
 if hasattr(v,'detach'):return v.detach().cpu().numpy()
 return np.asarray(v.numpy() if hasattr(v,'numpy') else v)

class ChainEmbodiment(YamArenaEmbodiment):
 def __init__(self,layout_seed=17,**kwargs):
  kwargs.setdefault('object_jitter',0.);super().__init__(**kwargs)
  if self.object_jitter!=0:raise ValueError('Use explicit layout_seed for joint-consistent randomization; object_jitter must be0')
  self.layout_seed=int(layout_seed);self.layout_data=layout(self.layout_seed);self.joints={};self.audit=CrossingAudit();self.rod_audit=RodAudit();self._audit_enabled=False;self._hardware_invalid=False;self._writer=None;self._epoch=0;self._original_step=None;self._access_result=None
  h=hashlib.sha256()
  for p in sorted(ROOT.rglob('*')):
   if p.is_file() and p.suffix in ['.py','.usda','.toml']:h.update(str(p.relative_to(ROOT)).encode()+b'\0'+p.read_bytes())
  self.info=replace(self.info,name='yam_carabiner_chain',environment_revision=h.hexdigest())
 def _ensure_sim(self,goal):
  if self._env is not None:self._restore_joints();return
  from yam_arena import inspect_embodiment as base
  if base._APP is None:
   from isaaclab.app import AppLauncher
   os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES');base._APP=AppLauncher(headless=self.headless,device=self.device,enable_cameras=True).app;atexit.register(base._close_app_at_exit)
  from yam_arena import scene,embodiment as robots
  from isaaclab_arena.assets.object import Object
  from isaaclab_arena.assets.object_type import ObjectType
  from isaaclab_arena.utils.pose import Pose
  from isaaclab.sensors import CameraCfg
  from yam_arena.ik import matrix_to_quat_xyzw
  import isaaclab.sim as sim
  original_object,original_callback,original_robot=scene.task_object,scene.env_cfg_callback,robots.BimanualYamEmbodiment
  def spawn(spec):
   if spec.name not in self.task.names:return original_object(spec)
   if spec.name=='stand':name='stand';p=STAND_POS;r=np.eye(3)
   else:
    i=int(spec.name[-1]);d=self.layout_data['link'+str(i)];r=d['rotation'];p=d['position']+(r@GATE_OFFSET if spec.name.startswith('gate') else 0);name='gate' if spec.name.startswith('gate') else 'frame'+str(i)
   props=dict(physics_material=sim.RigidBodyMaterialCfg(static_friction=.6,dynamic_friction=.4),collision_props=sim.CollisionPropertiesCfg(contact_offset=.0001,rest_offset=0.))
   if spec.free:props['rigid_props']=sim.RigidBodyPropertiesCfg(solver_position_iteration_count=SOLVER_POSITION_ITERATIONS,solver_velocity_iteration_count=SOLVER_VELOCITY_ITERATIONS,max_depenetration_velocity=.5,angular_damping=BODY_ANGULAR_DAMPING,linear_damping=.05)
   return Object(name=spec.name,usd_path=str(ROOT/'assets'/(name+'.usda')),object_type=ObjectType.RIGID if spec.free else ObjectType.BASE,initial_pose=Pose(position_xyz=tuple(map(float,p)),rotation_xyzw=tuple(map(float,matrix_to_quat_xyzw(r)))),spawn_cfg_addon=props)
  def callback(hz=None):
   configure=original_callback(hz)
   def configured(cfg):cfg=configure(cfg);cfg.sim.physics.enable_ccd=True;return cfg
   return configured
  class DiagnosticRobot(original_robot):
   def __init__(robot,**options):
    options['overview_cam']=True;super().__init__(**options);eye=np.array([.85,-.55,1.28]);back=eye-np.array([.42,0,.91]);back/=np.linalg.norm(back);right=np.cross([0,0,1.],back);right/=np.linalg.norm(right);q=matrix_to_quat_xyzw(np.column_stack([right,np.cross(back,right),back]));cfg=robot.camera_config.overview_cam;cfg.offset=CameraCfg.OffsetCfg(pos=tuple(eye),rot=tuple(q),convention='opengl');cfg.spawn.focal_length=18.
  scene.task_object,scene.env_cfg_callback,robots.BimanualYamEmbodiment=spawn,callback,DiagnosticRobot
  try:super()._ensure_sim(goal)
  finally:scene.task_object,scene.env_cfg_callback,robots.BimanualYamEmbodiment=original_object,original_callback,original_robot
  from pxr import UsdPhysics
  for i in range(3):self.joints[i]=UsdPhysics.RevoluteJoint.Define(self._base.sim.stage,'/World/envs/env_0/GateHinge'+str(i))
  self._restore_joints()
  from chain_task.sensors import hand_geometry
  self._robot_geometry=hand_geometry()
  import omni.physx
  self._physx=omni.physx.get_physx_interface();self._original_step=self._base.sim.step
  def physics_step(*args,**kwargs):
   result=self._original_step(*args,**kwargs)
   if self._audit_enabled:self._substep_audit()
   return result
  self._base.sim.step=physics_step
 def _restore_joints(self):
  from pxr import UsdPhysics,Sdf,Gf
  angle=np.deg2rad(GATE_REFERENCE_DEG)
  for i in range(3):
   for kind,mass in [('link',FRAME_MASS),('gate',GATE_MASS)]:
    prim=self._base.sim.stage.GetPrimAtPath('/World/envs/env_0/'+kind+str(i));rb=UsdPhysics.RigidBodyAPI(prim);rb.CreateRigidBodyEnabledAttr(True);rb.CreateKinematicEnabledAttr(False);UsdPhysics.MassAPI(prim).CreateMassAttr(mass);prim.GetAttribute('physxRigidBody:angularDamping').Set(BODY_ANGULAR_DAMPING);prim.GetAttribute('physxRigidBody:linearDamping').Set(.05);prim.GetAttribute('physxRigidBody:solverPositionIterationCount').Set(SOLVER_POSITION_ITERATIONS);prim.GetAttribute('physxRigidBody:solverVelocityIterationCount').Set(SOLVER_VELOCITY_ITERATIONS)
  for i,j in self.joints.items():
   j.CreateBody0Rel().SetTargets([Sdf.Path('/World/envs/env_0/link'+str(i))]);j.CreateBody1Rel().SetTargets([Sdf.Path('/World/envs/env_0/gate'+str(i))]);j.CreateAxisAttr('Z');j.CreateLocalPos0Attr(Gf.Vec3f(*HINGE));j.CreateLocalPos1Attr(Gf.Vec3f(*GATE_ANCHOR));j.CreateLocalRot0Attr(Gf.Quatf(float(np.cos(angle/2)),0.,0.,float(np.sin(angle/2))));j.CreateLocalRot1Attr(Gf.Quatf(1.,0.,0.,0.));j.CreateLowerLimitAttr(-GATE_REFERENCE_DEG);j.CreateUpperLimitAttr(GATE_MAX_DEG-GATE_REFERENCE_DEG);j.CreateJointEnabledAttr(True);j.CreateCollisionEnabledAttr(False);j.CreateExcludeFromArticulationAttr(True)
   d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');d.CreateTypeAttr('force');d.CreateStiffnessAttr(SPRING_K);d.CreateDampingAttr(SPRING_D);d.CreateMaxForceAttr(SPRING_CAP);d.CreateTargetPositionAttr(-GATE_REFERENCE_DEG);d.CreateTargetVelocityAttr(0.)
 def _direct_poses(self):
  poses,rots={},{}
  for name in self.task.names:
   if name=='stand':poses[name]=STAND_POS.copy();rots[name]=np.eye(3);continue
   value=self._physx.get_rigidbody_transformation('/World/envs/env_0/'+name)
   if not value.get('ret_val',False):raise RuntimeError('Live PhysX pose unavailable for '+name)
   poses[name]=np.array(value['position'],dtype=float);rots[name]=quat_xyzw_to_matrix(np.array(value['rotation'],dtype=float))
  return poses,rots
 def _substep_audit(self):
  try:
   p,r=self._direct_poses();b={n:Body(p[n],r[n],np.zeros(3),np.zeros(3)) for n in p};curves,closed=curves_and_closed(b);self.audit.update(curves,closed)
   rods={'eye':[(a,c,EYE_RADIUS) for a,c in zip(curves['eye'][:-1],curves['eye'][1:])],**{'link'+str(i):segments(b,i) for i in range(3)}}
   groups={n:(np.array([(u,v) for u,v,_ in shapes]),np.array([radius for _,_,radius in shapes]),np.array([n!='eye' and i==len(shapes)-1 for i in range(len(shapes))])) for n,shapes in rods.items()}
   if not self.rod_audit.update(groups):
    reason=dict(self.rod_audit.reason);label=reason.pop('reason');self._invalidate(label,**reason)
   for i in range(3):
    f,g='link'+str(i),'gate'+str(i);error=np.linalg.norm(p[f]+r[f]@HINGE-p[g]-r[g]@GATE_ANCHOR)
    if error>.0008:self._invalidate('gate_anchor',index=i,error_m=float(error))
   self._last_direct=(p,r)
   self._max_gate_angles={str(i):max(self._max_gate_angles.get(str(i),0.),abs(gate_angle(r['link'+str(i)],r['gate'+str(i)]))) for i in range(3)}
  except Exception as exc:self._invalidate('physics_step_readback',error=str(exc));raise
 def _invalidate(self,reason,**values):
  if not self._hardware_invalid:print('CHAIN_INVALID',self._t,reason,json.dumps(values),flush=True)
  self._hardware_invalid=True
 def reset(self,scene,*,seed=None):
  self._audit_enabled=False;self._close_writer();self.audit=CrossingAudit();self.rod_audit=RodAudit();self._max_gate_angles={};self._hardware_invalid=False;FEEDBACK.clear();self.task.module.HISTORY=History();observation=super().reset(scene,seed=seed);self._read()
  if self._access_result is None:
   from chain_task.access import validate_layout
   self._access_result=validate_layout(ROOT,FEEDBACK['poses'],FEEDBACK['rotations'],self.layout_data);print('CHAIN_ACCESS_PREFLIGHT',json.dumps(self._access_result),flush=True)
  self._audit_enabled=True
  self._substep_audit();print('CHAIN_RESET',json.dumps(dict(engine_seed=seed,layout_seed=self.layout_seed,layout={k:dict(position=v['position'].tolist(),rotation=v['rotation'].tolist(),yaw=v['yaw'],face_down=v['face_down']) for k,v in self.layout_data.items()})),flush=True)
  destination=os.environ.get('CHAIN_DIAGNOSTIC_DIR')
  if destination:
   import imageio_ffmpeg
   out=Path(destination);out.mkdir(parents=True,exist_ok=True);self._writer=imageio_ffmpeg.write_frames(str(out/f'diagnostic-e{self._epoch}.mp4'),(self.cam_width,self.cam_height),fps=self.info.control_hz,codec='libx264',pix_fmt_out='yuv420p',macro_block_size=1,output_params=['-crf','18']);self._writer.send(None);self._epoch+=1;self._record();self._metadata(out/f'runtime-metadata-e{self._epoch-1}.json',observation)
  return observation
 def _read(self):
  poses,rots={},{}
  for n in self.task.names:
   value=array(self._base.arena_world.get_pose_w(n))[0];poses[n]=value[:3];rots[n]=quat_xyzw_to_matrix(value[3:])
  from yam_arena import ik,station
  hands={}
  for side,key in [('left','left_robot'),('right','right_robot')]:
   arm,fingers=self._ids[key];q=array(self._base.scene[key].data.joint_pos)[0];p,r=ik.tcp_pose(q[arm],station.BASES[side]);hands[side]=dict(tcp=p.tolist(),rotation=r.tolist(),joints=q[arm].tolist(),fingers=q[fingers].tolist())
  FEEDBACK.update(poses=poses,rotations=rots,hands=hands,angles={str(i):gate_angle(rots['link'+str(i)],rots['gate'+str(i)]) for i in range(3)})
 def _integrity(self):
  from pxr import UsdPhysics,Gf
  expected={str(j.GetPath()) for j in self.joints.values()}
  for i,j in self.joints.items():
   if not j.GetJointEnabledAttr().Get() or j.GetAxisAttr().Get()!='Z' or j.GetCollisionEnabledAttr().Get():self._invalidate('gate_joint_flags',index=i)
   if list(map(str,j.GetBody0Rel().GetTargets()))!=['/World/envs/env_0/link'+str(i)] or list(map(str,j.GetBody1Rel().GetTargets()))!=['/World/envs/env_0/gate'+str(i)]:self._invalidate('gate_joint_bodies',index=i)
   if abs(j.GetLowerLimitAttr().Get()+GATE_REFERENCE_DEG)>1e-5 or abs(j.GetUpperLimitAttr().Get()-(GATE_MAX_DEG-GATE_REFERENCE_DEG))>1e-5:self._invalidate('gate_joint_limits',index=i)
   angle=np.deg2rad(GATE_REFERENCE_DEG);q0=np.array(j.GetLocalRot0Attr().Get().GetImaginary());w0=float(j.GetLocalRot0Attr().Get().GetReal());q1=j.GetLocalRot1Attr().Get()
   if np.linalg.norm(np.array(j.GetLocalPos0Attr().Get())-HINGE)>1e-7 or np.linalg.norm(np.array(j.GetLocalPos1Attr().Get())-GATE_ANCHOR)>1e-7 or np.linalg.norm(q0-[0,0,np.sin(angle/2)])>1e-7 or abs(w0-np.cos(angle/2))>1e-7 or abs(q1.GetReal()-1)>1e-7 or np.linalg.norm(q1.GetImaginary())>1e-7:self._invalidate('gate_joint_frames',index=i)
   for kind,mass in [('link',FRAME_MASS),('gate',GATE_MASS)]:
    prim=self._base.sim.stage.GetPrimAtPath('/World/envs/env_0/'+kind+str(i));rb=UsdPhysics.RigidBodyAPI(prim);actual=UsdPhysics.MassAPI(prim).GetMassAttr().Get()
    if not rb.GetRigidBodyEnabledAttr().Get() or rb.GetKinematicEnabledAttr().Get() or abs(actual-mass)>1e-7 or abs(prim.GetAttribute('physxRigidBody:angularDamping').Get()-BODY_ANGULAR_DAMPING)>1e-6:self._invalidate('body_physics',index=i,kind=kind)
   if np.linalg.norm(FEEDBACK['rotations']['link'+str(i)][:,2]-FEEDBACK['rotations']['gate'+str(i)][:,2])>.01:self._invalidate('gate_axis_alignment',index=i)
   d=UsdPhysics.DriveAPI(j.GetPrim(),'angular')
   for a,v in [(d.GetStiffnessAttr(),SPRING_K),(d.GetDampingAttr(),SPRING_D),(d.GetMaxForceAttr(),SPRING_CAP),(d.GetTargetPositionAttr(),-GATE_REFERENCE_DEG),(d.GetTargetVelocityAttr(),0.)]:
    if abs(a.Get()-v)>max(abs(v)*1e-5,1e-9):self._invalidate('gate_spring_settings',index=i)
   physical=FEEDBACK['angles'][str(i)]
   if not np.deg2rad(-2.)<=physical<=np.deg2rad(GATE_MAX_DEG+2.):self._invalidate('gate_travel',index=i,angle_deg=float(np.degrees(physical)))
  free={f'/World/envs/env_0/{kind}{i}' for kind in ['link','gate'] for i in range(3)}
  for prim in self._base.sim.stage.Traverse():
   if prim.IsA(UsdPhysics.Joint):
    j=UsdPhysics.Joint(prim);targets={str(p) for rel in [j.GetBody0Rel(),j.GetBody1Rel()] for p in rel.GetTargets()}
    if targets&free and str(j.GetPath()) not in expected:self._invalidate('unexpected_constraint',path=str(j.GetPath()))
 def _task_step(self):
  self._read();self._integrity();b={}
  for n in self.task.names:
   v=w=np.zeros(3)
   if n!='stand':v=array(self._base.arena_world.get_root_linear_velocity_w(n)[0]);w=array(self._base.arena_world.get_root_angular_velocity_w(n)[0])
   b[n]=Body(FEEDBACK['poses'][n],FEEDBACK['rotations'][n],v,w)
  from chain_task.sensors import hand_bounds
  h=self.task.module.HISTORY;h.update(b,hand_bounds(self._base,self._robot_geometry),self._base.step_dt,not self._hardware_invalid,self.audit.invalid);FEEDBACK['score_reasons']=h.reasons;FEEDBACK['score']=h.score;FEEDBACK['audit_steps']=self.audit.steps
  result=super()._task_step();self._record()
  if round(self._t*self.info.control_hz)%20==0:print('CHAIN_STATE',round(self._t,3),json.dumps(dict(positions={k:v.tolist() for k,v in FEEDBACK['poses'].items()},rotations={k:v.tolist() for k,v in FEEDBACK['rotations'].items()},angles=FEEDBACK['angles'],hands=FEEDBACK['hands'],score=h.score,checks=h.reasons,audit_steps=self.audit.steps,audit_invalid=self.audit.invalid,max_physics_step_gate_angles=self._max_gate_angles,gate_narrow_checks=self.rod_audit.narrow_checks)),flush=True)
  return result
 def _metadata(self,path,observation):
  import hppfcl
  bodies={}
  for i in range(3):
   for kind in ['link','gate']:
    n=kind+str(i);d=self._base.scene[n].data;m=array(d.default_mass)[0].reshape(-1);inertia=array(d.default_inertia)[0].reshape(3,3);eig=np.linalg.eigvalsh(inertia)
    if np.any(m<=0) or np.any(eig<=0):raise RuntimeError('Invalid mass/inertia')
    bodies[n]=dict(mass_kg=m.tolist(),inertia_eigenvalues=eig.tolist(),com_in_body_m=array(d.body_com_pos_b)[0,0].tolist(),authored_physics={key:self._base.sim.stage.GetPrimAtPath('/World/envs/env_0/'+n).GetAttribute('physxRigidBody:'+key).Get() for key in ['angularDamping','linearDamping','solverPositionIterationCount','solverVelocityIterationCount','maxDepenetrationVelocity']})
  cams={}
  for n in ['top_cam','left_cam','right_cam','overview_cam']:
   c=self._base.scene[n];cfg=c.cfg;cams[n]=dict(position=cfg.offset.pos,rotation=cfg.offset.rot,convention=cfg.offset.convention,width=cfg.width,height=cfg.height,intrinsic_matrix=array(c.data.intrinsic_matrices)[0].tolist(),policy_observation=n!='overview_cam')
  path.write_text(json.dumps(dict(environment_revision=self.info.environment_revision,layout_seed=self.layout_seed,access_preflight=self._access_result,hppfcl_version=hppfcl.__version__,bodies=bodies,cameras=cams,observation_shapes={k:list(v.shape) for k,v in observation.images.items()},physics_dt=self._base.cfg.sim.dt,control_dt=self._base.step_dt,device=str(self._base.device),ccd=self._base.cfg.sim.physics.enable_ccd,audit='Live PhysX get_rigidbody_transformation after every simulation substep'),indent=2)+'\n')
 def _record(self):
  if self._writer is not None:self._writer.send(np.ascontiguousarray(array(self._base.scene['overview_cam'].data.output['rgb'])[0,:,:,:3],dtype=np.uint8))
 def _close_writer(self):
  if self._writer is not None:self._writer.close();self._writer=None
 def close(self):
  self._audit_enabled=False;self._close_writer()
  if self._original_step is not None and self._base is not None:self._base.sim.step=self._original_step
  super().close()

def make_embodiment(**kwargs):return ChainEmbodiment(**kwargs)
