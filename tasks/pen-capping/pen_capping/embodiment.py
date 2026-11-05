"""Hollow cap insertion, measured seating and a pose-preserving retention joint."""
from pathlib import Path
import os,atexit,math,hashlib
import numpy as np
from yam_arena.inspect_embodiment import YamArenaEmbodiment
FEEDBACK={}
class PenEmbodiment(YamArenaEmbodiment):
 def __init__(self,**kwargs):
  super().__init__(**kwargs)
  from dataclasses import replace
  root=Path(__file__).resolve().parent.parent;h=hashlib.sha256()
  for p in sorted(root.rglob('*')):
   if p.is_file() and p.suffix in ('.py','.usdc','.png','.toml') and '__pycache__' not in p.parts:h.update(str(p.relative_to(root)).encode()+b'\0'+p.read_bytes())
  self.info=replace(self.info,environment_revision=h.hexdigest());self._pairs={};self._insertions={}
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
    return Object(name=spec.name,usd_path=str(Path(__file__).resolve().parent.parent/'assets'/asset),object_type=ObjectType.RIGID,initial_pose=Pose(position_xyz=tuple(spec.pos),rotation_xyzw=(0.,0.,math.sin(spec.yaw/2),math.cos(spec.yaw/2))),spawn_cfg_addon={'physics_material':scene._grippy(),'collision_props':sim.CollisionPropertiesCfg(contact_offset=.0002,rest_offset=0.),'rigid_props':sim.RigidBodyPropertiesCfg(solver_position_iteration_count=32,solver_velocity_iteration_count=4)})
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
  for h in headers:
   if not h.num_contact_data:continue
   paths=[str(PhysicsSchemaTools.intToSdfPath(getattr(h,n))) for n in ('collider0','collider1')]
   if any('/cap_0/' in p for p in paths) and any('/pen_0/' in p for p in paths):self._contacts_seen.add(tuple(p.split('/env_0/')[-1] for p in paths))
 def reset(self,scene,*,seed=None):
  from pen_capping.scoring import History
  if self._base is not None:
   stage=self._base.sim.stage
   for pen,cap in self._pairs.items():stage.RemovePrim(f'/World/envs/env_0/CapRetention{pen}_{cap}')
  self._pairs={};self._insertions={};FEEDBACK.clear();self.task.module.HISTORY=History(self.task.module.COLORS)
  super().reset(scene,seed=seed)
  for _ in range(48):self._env.step(self._ready)
  for name in ('top_cam','left_cam','right_cam'):self._base.scene[name].reset()
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
  pn,cn=f'pen_{i}',f'cap_{j}';relative=rots[pn].T@(poses[cn][:3]-poses[pn][:3]);q=matrix_to_quat_xyzw(rots[pn].T@rots[cn]);joint=UsdPhysics.FixedJoint.Define(self._base.sim.stage,f'/World/envs/env_0/CapRetention{i}_{j}')
  joint.CreateBody0Rel().SetTargets([Sdf.Path('/World/envs/env_0/'+pn)]);joint.CreateBody1Rel().SetTargets([Sdf.Path('/World/envs/env_0/'+cn)]);joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float,relative)));joint.CreateLocalPos1Attr(Gf.Vec3f(0,0,0));joint.CreateLocalRot0Attr(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3]))));joint.CreateLocalRot1Attr(Gf.Quatf(1,0,0,0));joint.CreateExcludeFromArticulationAttr(True)
  self._pairs[i]=j;print('CAP_SEATED',round(self._t,2),i,j,'relative',relative.tolist(),flush=True)
 def _task_step(self):
  from pen_capping.scoring import Insertion,hands_clear
  from yam_arena import ik,station
  poses,rots=self._read();count=len(self.task.module.COLORS)
  from pen_capping.scoring import SEATED_X
  for i,j in list(self._pairs.items()):
   pn,cn=f'pen_{i}',f'cap_{j}';rel=rots[pn].T@(poses[cn][:3]-poses[pn][:3]);alignment=float(rots[pn][:,0]@rots[cn][:,0])
   if np.linalg.norm(rel-np.array([SEATED_X,0,0]))>.006 or alignment<math.cos(math.radians(8)):
    self._base.sim.stage.RemovePrim(f'/World/envs/env_0/CapRetention{i}_{j}');del self._pairs[i];print('CAP_RETENTION_LOST',i,j,flush=True)
  for i in range(count):
   if i in self._pairs:continue
   for j in range(count):
    if j in self._pairs.values():continue
    pn,cn=f'pen_{i}',f'cap_{j}';relative=rots[pn].T@(poses[cn][:3]-poses[pn][:3]);alignment=float(rots[pn][:,0]@rots[cn][:,0]);history=self._insertions.setdefault((i,j),Insertion())
    if history.update(relative,alignment):self._retain(i,j,poses,rots);break
  tcps=[]
  for side,key in (('left','left_robot'),('right','right_robot')):
   arm,_=self._ids[key];q=self._base.scene[key].data.joint_pos.torch[0];tcp,_=ik.tcp_pose(q[arm].cpu().numpy(),station.BASES[side]);tcps.append(tcp)
  pairs=[];positions=[];axes=[];speeds=[];clear=[]
  for i in range(count):
   name=f'pen_{i}';j=self._pairs.get(i);pairs.append(None if j is None else self.task.module.COLORS[j]);positions.append(poses[name][:3]);axes.append(rots[name][:,0]);v=self._base.arena_world.get_root_linear_velocity_w(name)[0].detach().cpu().numpy();w=self._base.arena_world.get_root_angular_velocity_w(name)[0].detach().cpu().numpy();speeds.append(float(np.linalg.norm(v)+.06*np.linalg.norm(w)))
   points=[poses[name][:3]]+([] if j is None else [poses[f'cap_{j}'][:3]]);clear.append(hands_clear(tcps,poses[name][:3],rots[name][:,0],None if j is None else poses[f'cap_{j}'][:3]))
  h=self.task.module.HISTORY;h.update(pairs,positions,axes,speeds,clear,self.task.module.TOP,self.task.module.DESTINATIONS)
  FEEDBACK.update(placed=list(h.placed),capped=list(h.capped))
  result=super()._task_step()
  if round(self._t*self.info.control_hz)%20==0:
   relative=rots['pen_0'].T@(poses['cap_0'][:3]-poses['pen_0'][:3]);alignment=float(rots['pen_0'][:,0]@rots['cap_0'][:,0]);print('PEN_STATE',round(self._t,2),'pen_position',np.round(poses['pen_0'][:3],4).tolist(),'cap_position',np.round(poses['cap_0'][:3],4).tolist(),'relative',np.round(relative,4).tolist(),'alignment',round(alignment,4),'contacts',sorted(self._contacts_seen),'all_positions',{name:np.round(pose[:3],4).tolist() for name,pose in poses.items()},'all_relatives',[np.round(rots[f'pen_{i}'].T@(poses[f'cap_{i}'][:3]-poses[f'pen_{i}'][:3]),4).tolist() for i in range(count)],'pairs',self._pairs,'capped',h.capped,'placed',h.placed,'clear',clear,'score',result.info.get('score'),flush=True)
  return result

def make_embodiment(**kwargs):return PenEmbodiment(**kwargs)
