"""Original station with native passive maintenance parts; private feasibility evidence."""
import atexit,json
import numpy as np
from . import geometry as G
def install(out,diagnostic_no_images=False,allow_demo_evidence=False):
 from yam_arena import inspect_embodiment as official
 ensure0=official.YamArenaEmbodiment._ensure_sim;reset0=official.YamArenaEmbodiment.reset;observe0=official.YamArenaEmbodiment._observation
 joints=[];sample_indices=[];diagnostic=None;pending=[];shapes=None
 installed=False;view=None;simulation=None;paths=[];q=[];vel=[];robot_forces=[];robot_counts=[];contacts=None;pair_view=None;pair_forces=[];pair_counts=[];robot_environment=None;env_forces=[];env_counts=[]
 def ensure(self,goal):
  nonlocal installed,view,simulation,paths,contacts,pair_view,robot_environment,diagnostic
  if self.rig!=official.station.rig('calibrated') or (self.cam_width,self.cam_height)!=(320,176) or self.info.control_hz!=20:raise RuntimeError('Original rig required')
  if not installed:
   from isaaclab.app import AppLauncher
   assert official._APP is None
   official._APP=AppLauncher(headless=True,device=self.device,enable_cameras=not diagnostic_no_images,multi_gpu=False,kit_args="--/UJITSO/geometry=false").app
   atexit.register(official._close_app_at_exit)
   if diagnostic_no_images:
    # This explicit physics-only branch never exports valid submission media.
    # Disable camera sensors/observation terms as well as image recording, while
    # preserving native robot geometry, drives, contacts and fixed-step physics.
    from yam_arena.embodiment import BimanualYamEmbodiment
    constructor=BimanualYamEmbodiment.__init__
    def without_cameras(instance,*args,**kwargs):
     kwargs['enable_cameras']=False
     constructor(instance,*args,**kwargs)
    BimanualYamEmbodiment.__init__=without_cameras
   from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
   compose=ArenaEnvBuilder.compose_manager_cfg
   def compact(builder):
    cfg,kw=compose(builder)
    cfg.sim.dt=1/G.PHYSICS_HZ;cfg.decimation=G.PHYSICS_HZ//20;cfg.sim.render_interval=G.PHYSICS_HZ//20
    cfg.sim.physics.solve_articulation_contact_last=True
    if diagnostic_no_images:cfg.sim.render_interval=1000000
    for k,v in dict(gpu_found_lost_pairs_capacity=262144,gpu_found_lost_aggregate_pairs_capacity=262144,gpu_total_aggregate_pairs_capacity=262144,gpu_max_rigid_contact_count=262144,gpu_max_rigid_patch_count=65536).items():setattr(cfg.sim.physics,k,v)
    return cfg,kw
   ArenaEnvBuilder.compose_manager_cfg=compact
   from yam_arena import scene
   from isaaclab.sim.utils import clone
   from isaaclab.sim.spawners.shapes.shapes import spawn_cuboid
   old=scene.task_object
   def task_object(spec):
    obj=old(spec)
    if spec.name=='build_anchor':
     @clone
     def spawn(path,cfg,*args,**kwargs):
      prim=spawn_cuboid.__wrapped__(path,cfg,*args,**kwargs);stage=prim.GetStage()
      from pxr import UsdPhysics,Sdf
      for sp in stage.Traverse():
       if sp.IsA(UsdPhysics.Scene):sp.CreateAttribute('physxScene:solveArticulationContactLast',Sdf.ValueTypeNames.Bool).Set(True)
      G.build(stage)
      return prim
     obj.spawner_cfg.func=spawn;obj.object_cfg.spawn=obj.spawner_cfg
    return obj
   scene.task_object=task_object;installed=True
  ensure0(self,goal)
  if view is None:
   from isaaclab.sim.utils.stage import get_current_stage_id
   from pxr import Usd
   import omni.usd,omni.physics.tensors as tensors
   simulation=tensors.create_simulation_view('torch',stage_id=get_current_stage_id());simulation.set_subspace_roots('/')
   view=simulation.create_rigid_body_view([G.ROOT+'/'+G.PREFIX+n for n in G.BODY_NAMES]);paths=list(view.prim_paths)
   assert len(paths)==len(G.BODY_NAMES),paths
   stage=omni.usd.get_context().get_stage()
   sensors=[str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(G.ROOT)) if any('/'+a+'/' in str(p.GetPath()) for a in ['LeftArm','RightArm']) and 'PhysxContactReportAPI' in p.GetAppliedSchemas()]
   assert len(sensors)==18
   contacts=simulation.create_rigid_contact_view(sensors,[paths]*18,max_contact_data_count=65536)
   rail=G.FIXED
   for path in rail:assert 'PhysicsCollisionAPI' in stage.GetPrimAtPath(path).GetAppliedSchemas(),path
   table=G.ROOT+'/table/geometry/mesh'
   staging=[str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(G.ROOT)) if 'PhysicsCollisionAPI' in p.GetAppliedSchemas() and any('/'+name in str(p.GetPath()) for name in ('cup_rest','core_rest','pad_rest','left_clamp_rest','right_clamp_rest'))]
   filters=paths+rail+[table]
   pair_view=simulation.create_rigid_contact_view(paths,[filters]*len(paths),max_contact_data_count=65536)
   robot_environment=simulation.create_rigid_contact_view(sensors,[sensors+rail+staging+[table]]*18,max_contact_data_count=65536)
   (out/'contact-contract.json').write_text(json.dumps(dict(robot_sensors=sensors,robot_object_filters=paths,object_sensors=paths,object_filters=filters,robot_environment_filters=sensors+rail+staging+[table],rail_paths=rail,table_path=table,capacity=65536),indent=2))
   (out/'mechanism-contract.json').write_text(json.dumps(dict(**G.contract(),numerical_solver=dict(solve_articulation_contact_last=True,diagnostic_no_images=diagnostic_no_images),paths=paths,robot_sensors=sensors),indent=2))
   from . import scoring as outcome
   diagnostic=outcome.Outcome(paths,G.contract(),json.loads((out/'contact-contract.json').read_text()))
   outcome.ACTIVE=diagnostic
   # Script-only getter; no data is added to the model observation.
   from . import controller
   controller.BODY_EVIDENCE=(lambda: (list(paths),q[-1].copy()) if q else None) if allow_demo_evidence else None
   stage.GetRootLayer().Export(str(out/'scene.usda'))
   from isaaclab_physx.physics.physx_manager import PhysxManager
   step0=PhysxManager.step
   def step(cls):
    nonlocal shapes
    step0()
    # Clone every substep on the GPU, then transfer a complete control interval
    # together. No contact sample is skipped; this avoids repeated CPU/GPU stalls.
    tensors_=[view.get_transforms(),view.get_velocities(),contacts.get_contact_data(dt=1/G.PHYSICS_HZ)[4],contacts.get_contact_force_matrix(dt=1/G.PHYSICS_HZ),pair_view.get_contact_force_matrix(dt=1/G.PHYSICS_HZ),pair_view.get_contact_data(dt=1/G.PHYSICS_HZ)[4],robot_environment.get_contact_force_matrix(dt=1/G.PHYSICS_HZ),robot_environment.get_contact_data(dt=1/G.PHYSICS_HZ)[4]]
    import torch
    if shapes is None:shapes=[tuple(t.shape) for t in tensors_]
    pending.append(torch.cat([t.reshape(-1).float() for t in tensors_]))
    if len(pending)>=G.PHYSICS_HZ//20:flush()
   PhysxManager.step=classmethod(step)
 def flush():
  if not pending:return
  import torch
  batch=torch.stack(pending).cpu().numpy();pending.clear()
  for row in batch:
   values=[];off=0
   for shape in shapes:
    size=int(np.prod(shape));values.append(row[off:off+size].reshape(shape).copy());off+=size
   poses,v,counts,forces,pf,pc,ef,ec=values
   if not all(np.isfinite(a).all() for a in values) or any(int(a.sum())>=65536 for a in (counts,pc,ec)):raise RuntimeError('Invalid native substep evidence')
   q.append(poses);vel.append(v);robot_counts.append(counts.astype(np.int32));robot_forces.append(forces)
   pair_forces.append(pf);pair_counts.append(pc.astype(np.int32));env_forces.append(ef);env_counts.append(ec.astype(np.int32))
   diagnostic.update(poses,counts,pc,ec)
 def reset(self,*args,**kwargs):
  obs=reset0(self,*args,**kwargs)
  import cv2
  for n,im in obs.images.items():cv2.imwrite(str(out/('setup_'+n+'.png')),cv2.cvtColor(im,cv2.COLOR_RGB2BGR))
  return obs
 def observe(self):
  if diagnostic_no_images:
   from inspect_robots.types import Observation
   from inspect_robots_yam.packing import STATE_KEY,pack
   per_arm=[]
   for name in official.ARMS:
    arm_ids,finger_ids=self._ids[name];positions=self._base.scene[name].data.joint_pos.torch[0]
    angles=positions[arm_ids].cpu().numpy().astype(np.float64)
    opening=float(np.clip(positions[finger_ids].mean().item()/official.FINGER_OPEN,0.,1.))
    per_arm.append(np.r_[angles,opening])
   obs=Observation(images={},state={STATE_KEY:pack(*per_arm)},instruction=self._instruction,image_times={},state_time=self._t)
  else:obs=observe0(self)
  from inspect_robots_yam.packing import STATE_KEY
  joints.append(np.asarray(obs.state[STATE_KEY],float).copy());sample_indices.append(len(q)-1)
  if len(joints)%20==1:
   (out/'live-physics.json').write_text(json.dumps(dict(frames=len(joints),samples=len(q),paths=paths,
       poses=q[-1].tolist(),joint_pos=joints[-1].tolist(),robot_contact_peak_N=np.linalg.norm(robot_forces[-(G.PHYSICS_HZ//20):],axis=-1).max(axis=0).tolist(),
       object_contact_peak_N=np.linalg.norm(pair_forces[-(G.PHYSICS_HZ//20):],axis=-1).max(axis=0).tolist(),
       robot_environment_peak_N=np.linalg.norm(env_forces[-(G.PHYSICS_HZ//20):],axis=-1).max(axis=0).tolist(),
       prototype_outcome=diagnostic.detail)))
  return obs
 official.YamArenaEmbodiment._observation=observe
 official.YamArenaEmbodiment._ensure_sim=ensure;official.YamArenaEmbodiment.reset=reset
 def save():
  flush()
  if diagnostic is not None:(out/'prototype-outcome.json').write_text(json.dumps(diagnostic.detail,indent=2))
  if joints:np.savez_compressed(out/'joint-observations.npz',joint_pos=joints,physics_sample=sample_indices)
  if q:np.savez_compressed(out/'physics-substeps.npz',poses=q,velocities=vel,paths=paths,robot_contact_counts=robot_counts,robot_contact_forces=robot_forces,object_contact_forces=pair_forces,object_contact_counts=pair_counts,robot_environment_forces=env_forces,robot_environment_counts=env_counts,physics_hz=G.PHYSICS_HZ)
 return save
