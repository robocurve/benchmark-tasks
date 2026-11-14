"""Visible silicone liner, geometric engagement and uncalibrated joint retention."""
from pathlib import Path
import os,atexit,math,hashlib,json
import numpy as np
from yam_arena.inspect_embodiment import YamArenaEmbodiment
BODY_POSITION_ITERATIONS=128
BODY_VELOCITY_ITERATIONS=16
from pen_capping.contact_tracker import BackingContacts
FEEDBACK={}
class PenEmbodiment(YamArenaEmbodiment):
 def __init__(self,**kwargs):
  super().__init__(**kwargs)
  from dataclasses import replace
  root=Path(__file__).resolve().parent.parent;h=hashlib.sha256()
  for p in sorted(root.rglob('*')):
   if p.is_file() and p.suffix in ('.py','.usdc','.png','.toml') and '__pycache__' not in p.parts:h.update(str(p.relative_to(root)).encode()+b'\0'+p.read_bytes())
  self.info=replace(self.info,environment_revision=h.hexdigest());self._pairs={};self._insertions={};self._joint_reference={};self._backing_contacts=BackingContacts()
 def _ensure_sim(self,goal):
  from yam_arena import inspect_embodiment as base
  if base._APP is None:
   from isaaclab.app import AppLauncher
   os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES');base._APP=AppLauncher(headless=self.headless,device=self.device,enable_cameras=True).app;atexit.register(base._close_app_at_exit)
  from yam_arena import scene
  from isaaclab_arena.assets.object import Object
  from isaaclab_arena.assets.object_type import ObjectType
  from isaaclab_arena.utils.pose import Pose
  import isaaclab.sim as sim
  original=scene.task_object
  def spawn(spec):
   if spec.name.startswith('area_'):
    return Object(name=spec.name,usd_path=str(Path(__file__).resolve().parent.parent/'assets'/('area-'+spec.name[5:]+'.usdc')),object_type=ObjectType.BASE,initial_pose=Pose(position_xyz=tuple(spec.pos)))
   if spec.name.startswith(('pen_','cap_')):
    kind,index=spec.name.split('_');color=self.task.module.COLORS[int(index)];asset=kind+'-'+color+'.usdc'
    return Object(name=spec.name,usd_path=str(Path(__file__).resolve().parent.parent/'assets'/asset),object_type=ObjectType.RIGID,initial_pose=Pose(position_xyz=tuple(spec.pos),rotation_xyzw=(0.,0.,math.sin(spec.yaw/2),math.cos(spec.yaw/2))),spawn_cfg_addon={'physics_material':scene._grippy(),'collision_props':sim.CollisionPropertiesCfg(contact_offset=.0002,rest_offset=0.),'rigid_props':sim.RigidBodyPropertiesCfg(solver_position_iteration_count=BODY_POSITION_ITERATIONS,solver_velocity_iteration_count=BODY_VELOCITY_ITERATIONS)})
   return original(spec)
  scene.task_object=spawn
  try:super()._ensure_sim(goal)
  finally:scene.task_object=original
  self._base.sim.set_setting('/physics/disableContactProcessing',False)
  self._contacts_seen=set()
  if not hasattr(self,'_contact_subscription'):
   import omni.physx
   self._contact_subscription=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(self._contacts)
 def _contacts(self,headers,data):
  from pxr import PhysicsSchemaTools
  from omni.physx.bindings._physx import ContactEventType
  for h in headers:
   paths=[str(PhysicsSchemaTools.intToSdfPath(getattr(h,n))).split('/env_0/')[-1] for n in ('collider0','collider1')]
   event='lost' if h.type==ContactEventType.CONTACT_LOST else ('found' if h.type==ContactEventType.CONTACT_FOUND else 'persists')
   if h.num_contact_data and any('cap_' in p for p in paths) and any('pen_' in p for p in paths):self._contacts_seen.add(tuple(paths))
   if not any('/LinerBacking' in p for p in paths) or not any('/Barrel' in p for p in paths):continue
   points=[]
   for k in range(h.contact_data_offset,min(h.contact_data_offset+h.num_contact_data,h.contact_data_offset+2)):
    c=data[k];points.append(dict(position_m=list(map(float,c.position)),normal=list(map(float,c.normal)),impulse=list(map(float,c.impulse)),separation_m=float(c.separation)))
   impulse_norm_sum=sum(math.sqrt(sum(float(v)**2 for v in data[k].impulse)) for k in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data))
   self._backing_contacts.record(paths,event,float(self._t),points,impulse_norm_sum=impulse_norm_sum)
 def reset(self,scene,*,seed=None):
  from pen_capping.scoring import History,RestMotion
  if self._base is not None:
   stage=self._base.sim.stage
   for pen,cap in self._pairs.items():stage.RemovePrim(f'/World/envs/env_0/CapRetention{pen}_{cap}')
  self._pairs={};self._insertions={};self._joint_reference={};self._backing_contacts=BackingContacts();self._contacts_seen=set();FEEDBACK.clear();self.task.module.HISTORY=History(self.task.module.COLORS);self._rest_motion=RestMotion()
  super().reset(scene,seed=seed)
  for _ in range(48):self._env.step(self._ready)
  for name in ('top_cam','left_cam','right_cam'):self._base.scene[name].reset()
  from pen_capping.solver_audit import collect_solver_settings
  names=[f'{kind}_{i}' for i in range(len(self.task.module.COLORS)) for kind in ('pen','cap')]
  cfg=getattr(self._base.sim.cfg,'physx',None)
  self._solver_audit=collect_solver_settings(self._base.sim.stage,cfg,names,BODY_POSITION_ITERATIONS,BODY_VELOCITY_ITERATIONS)
  print('PEN_SOLVER_AUDIT',json.dumps(self._solver_audit,sort_keys=True),flush=True)
  if self._solver_audit['issues']:raise RuntimeError('Solver settings mismatch or global clamp: '+str(self._solver_audit['issues']))
  FEEDBACK['solver_audit']=self._solver_audit
  self._read();return self._observation()
 def _read(self):
  from yam_arena.urdf import quat_xyzw_to_matrix
  poses={};rotations={}
  for i in range(len(self.task.module.COLORS)):
   for kind in ('pen','cap'):
    name=f'{kind}_{i}';p=self._base.arena_world.get_pose_w(name)[0].detach().cpu().numpy();poses[name]=p;rotations[name]=quat_xyzw_to_matrix(p[3:])
  FEEDBACK.update(poses=poses,rotations=rotations,pairs=dict(self._pairs),colors=self.task.module.COLORS);return poses,rotations
 def _retain(self,i,j,poses,rots):
  from pxr import UsdPhysics,Gf,Sdf
  from yam_arena.ik import matrix_to_quat_xyzw
  from pen_capping.retention import engagement
  pn,cn=f'pen_{i}',f'cap_{j}';relative=rots[pn].T@(poses[cn][:3]-poses[pn][:3])
  if not engagement(relative,rots[pn].T@rots[cn])['eligible']:raise ValueError('Retention requires actual liner engagement')
  q=matrix_to_quat_xyzw(rots[pn].T@rots[cn]);joint=UsdPhysics.FixedJoint.Define(self._base.sim.stage,f'/World/envs/env_0/CapRetention{i}_{j}')
  joint.CreateBody0Rel().SetTargets([Sdf.Path('/World/envs/env_0/'+pn)]);joint.CreateBody1Rel().SetTargets([Sdf.Path('/World/envs/env_0/'+cn)]);joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float,relative)));joint.CreateLocalPos1Attr(Gf.Vec3f(0,0,0));joint.CreateLocalRot0Attr(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3]))));joint.CreateLocalRot1Attr(Gf.Quatf(1,0,0,0));joint.CreateExcludeFromArticulationAttr(True)
  from yam_arena.urdf import quat_xyzw_to_matrix
  stored_q=np.array([q[0],q[1],q[2],q[3]],dtype=np.float32).astype(float);stored_q/=np.linalg.norm(stored_q)
  self._joint_reference[(i,j)]=dict(position=relative.astype(np.float32).astype(float),rotation=quat_xyzw_to_matrix(stored_q))
  self._pairs[i]=j
  print('CAP_SEATED',round(self._t,2),i,j,'relative',relative.tolist(),'liner',engagement(relative,rots[pn].T@rots[cn]),'approximation','uncalibrated_fixed_joint',flush=True)
 def _task_step(self):
  from pen_capping.scoring import Insertion,hands_clear
  from yam_arena import ik,station
  poses,rots=self._read();count=len(self.task.module.COLORS)
  backing=self._backing_contacts.snapshot(float(self._t));FEEDBACK['backing_contacts']=backing
  for event in self._backing_contacts.drain():print('CAP_BACKING_EVENT',json.dumps(event,sort_keys=True),flush=True)
  if any(row['active'] or row['recent'] for row in backing.values()):print('CAP_BACKING_STATE',json.dumps(dict(time_s=float(self._t),pairs=backing),sort_keys=True),flush=True)
  from pen_capping.retention import engagement
  attempts={}
  for i,j in list(self._pairs.items()):
   pn,cn=f'pen_{i}',f'cap_{j}';rel=rots[pn].T@(poses[cn][:3]-poses[pn][:3]);alignment=float(rots[pn][:,0]@rots[cn][:,0])
   metrics=engagement(rel,rots[pn].T@rots[cn])
   reference=self._joint_reference[(i,j)];delta_rotation=(rots[pn].T@rots[cn])@reference['rotation'].T
   skew=np.array([delta_rotation[2,1]-delta_rotation[1,2],delta_rotation[0,2]-delta_rotation[2,0],delta_rotation[1,0]-delta_rotation[0,1]])*.5
   residual=rel-reference['position']
   state=dict(time_s=float(self._t),pen=i,cap=j,relative_m=rel.tolist(),captured_joint_local0_m=reference['position'].tolist(),constraint_position_residual_m=residual.tolist(),constraint_position_residual_norm_m=float(np.linalg.norm(residual)),constraint_rotation_residual_deg=math.degrees(math.atan2(float(np.linalg.norm(skew)),float((np.trace(delta_rotation)-1)*.5))),metrics=metrics)
   print('CAP_RETAINED_STATE',json.dumps(state,sort_keys=True),flush=True)
   if not metrics['eligible']:
    loss=dict(time_s=float(self._t),pen=i,cap=j,relative_m=rel.tolist(),cap_rotation_in_pen=(rots[pn].T@rots[cn]).tolist(),metrics=metrics)
    FEEDBACK.setdefault('retention_losses',{})[str(i)]=loss
    print('CAP_RETENTION_LOSS_STATE',json.dumps(loss,sort_keys=True),flush=True)
    self._base.sim.stage.RemovePrim(f'/World/envs/env_0/CapRetention{i}_{j}');del self._pairs[i];print('CAP_RETENTION_LOST',i,j,flush=True)
  for i in range(count):
   if i in self._pairs:continue
   for j in range(count):
    if j in self._pairs.values():continue
    pn,cn=f'pen_{i}',f'cap_{j}';relative=rots[pn].T@(poses[cn][:3]-poses[pn][:3]);alignment=float(rots[pn][:,0]@rots[cn][:,0]);history=self._insertions.setdefault((i,j),Insertion())
    seated=history.update(relative,alignment,rots[pn].T@rots[cn])
    snapshot=dict(time_s=float(self._t),pen=i,cap=j,relative_m=relative.tolist(),cap_rotation_in_pen=(rots[pn].T@rots[cn]).tolist(),approach_seen=history.approach,eligible_steps=history.count,metrics=history.metrics)
    attempts[f'{i}_{j}']=snapshot
    if history.approach and -.110<float(relative[0])<-.070 and np.linalg.norm(relative[1:])<.010:print('CAP_ENGAGEMENT',json.dumps(snapshot,sort_keys=True),flush=True)
    if seated:self._retain(i,j,poses,rots);break
  tcps=[]
  for side,key in (('left','left_robot'),('right','right_robot')):
   arm,_=self._ids[key];q=self._base.scene[key].data.joint_pos.torch[0];tcp,_=ik.tcp_pose(q[arm].cpu().numpy(),station.BASES[side]);tcps.append(tcp)
  pairs=[];positions=[];axes=[];speeds=[];clear=[]
  for i in range(count):
   name=f'pen_{i}';j=self._pairs.get(i);pairs.append(None if j is None else self.task.module.COLORS[j]);positions.append(poses[name][:3]);axes.append(rots[name][:,0]);v=self._base.arena_world.get_root_linear_velocity_w(name)[0].detach().cpu().numpy();w=self._base.arena_world.get_root_angular_velocity_w(name)[0].detach().cpu().numpy();speeds.append(self._rest_motion.speed(name,poses[name][:3],rots[name],self._base.step_dt))
   points=[poses[name][:3]]+([] if j is None else [poses[f'cap_{j}'][:3]]);clear.append(hands_clear(tcps,poses[name][:3],rots[name][:,0],None if j is None else poses[f'cap_{j}'][:3]))
  h=self.task.module.HISTORY;h.update(pairs,positions,axes,speeds,clear,self.task.module.TOP,self.task.module.DESTINATIONS)
  FEEDBACK.update(pairs=dict(self._pairs),placed=list(h.placed),capped=list(h.capped),attempts=attempts,liner={f'{i}_{j}':history.metrics for (i,j),history in self._insertions.items()})
  result=super()._task_step()
  if round(self._t*self.info.control_hz)%20==0:
   relative=rots['pen_0'].T@(poses['cap_0'][:3]-poses['pen_0'][:3]);alignment=float(rots['pen_0'][:,0]@rots['cap_0'][:,0]);print('PEN_STATE',round(self._t,2),'pen_position',np.round(poses['pen_0'][:3],4).tolist(),'cap_position',np.round(poses['cap_0'][:3],4).tolist(),'relative',np.round(relative,4).tolist(),'alignment',round(alignment,4),'contacts',sorted(self._contacts_seen),'all_positions',{name:np.round(pose[:3],4).tolist() for name,pose in poses.items()},'all_relatives',[np.round(rots[f'pen_{i}'].T@(poses[f'cap_{i}'][:3]-poses[f'pen_{i}'][:3]),4).tolist() for i in range(count)],'liner',{f'{i}_{j}':engagement(rots[f'pen_{i}'].T@(poses[f'cap_{j}'][:3]-poses[f'pen_{i}'][:3]),rots[f'pen_{i}'].T@rots[f'cap_{j}']) for i,j in self._pairs.items()},'pairs',self._pairs,'capped',h.capped,'placed',h.placed,'clear',clear,'resolved_motion_speed_m_s',speeds,'score',result.info.get('score'),flush=True)
  return result

def make_embodiment(**kwargs):return PenEmbodiment(**kwargs)
