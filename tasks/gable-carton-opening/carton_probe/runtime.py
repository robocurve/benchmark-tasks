"""Private physics evidence and original-camera snapshots, never extra policy inputs."""
import atexit
import json
import numpy as np
from . import geometry as G
from . import scoring


def install(out):
    from yam_arena import inspect_embodiment as official
    original_ensure=official.YamArenaEmbodiment._ensure_sim
    original_reset=official.YamArenaEmbodiment.reset
    original_observe=official.YamArenaEmbodiment._observation
    view=None;simulation=None;installed=False;resets=0;subscription=None
    poses=[];velocities=[];events=[];paths=[];contact_forces=[];contact_counts=[];contact_view=None
    environment_view=None;support_view=None
    environment_forces=[];environment_counts=[];support_forces=[];support_counts=[]
    score_trace=[]
    material=None;material_order=None;material_targets=[];material_rest=[]
    camera_records=[];joint_records=[];camera_samples=[];camera_mounts=[];robot_pose_view=None

    def ensure(self,goal):
        nonlocal view,simulation,installed,subscription,paths,contact_view,environment_view,support_view
        nonlocal material,material_order,material_targets,camera_mounts,robot_pose_view
        if self.rig!=official.station.rig('calibrated') or (self.cam_width,self.cam_height)!=(320,176):
            raise RuntimeError('Original rig/cameras required')
        if self.info.control_hz!=20 or self.object_jitter!=0:raise RuntimeError('Fixed original 20 Hz required')
        if not installed:
            from isaaclab.app import AppLauncher
            assert official._APP is None
            # Process-local RTX geometry residency: no camera/lighting/backend changes.
            # Streaming produced missing-scene frames under shared GPU load. The
            # original USD meshes remain fully resident; persistent settings untouched.
            official._APP=AppLauncher(headless=True,device=self.device,enable_cameras=True,multi_gpu=False,kit_args="--/UJITSO/geometry=false").app
            atexit.register(official._close_app_at_exit)
            import carb.settings
            settings=carb.settings.get_settings()
            (out/'renderer-settings.json').write_text(json.dumps({key:settings.get(key) for key in [
                '/UJITSO/geometry','/persistent/UJITSO/geometry','/renderer/scenePartitioning/enabled',
                '/rtx/hydra/readTransformsFromFabricInRenderDelegate']},indent=2))
            # Single-scene contact buffers, not a solver/time-step/geometry change.
            # Default multi-environment allocations exceed available shared-GPU memory.
            # Overflow remains a fatal SDK error in the evidence validator.
            from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
            compose=ArenaEnvBuilder.compose_manager_cfg
            def compact_buffers(builder):
                cfg,kwargs=compose(builder);physics=cfg.sim.physics
                allocations=dict(gpu_found_lost_pairs_capacity=262144,
                    gpu_found_lost_aggregate_pairs_capacity=262144,
                    gpu_total_aggregate_pairs_capacity=262144,
                    gpu_max_rigid_contact_count=262144,gpu_max_rigid_patch_count=65536)
                changes={}
                for key,value in allocations.items():
                    if not hasattr(physics,key):raise RuntimeError('Unknown official contact allocator '+key)
                    changes[key]=dict(original=getattr(physics,key),allocated=value)
                    setattr(physics,key,value)
                (out/'contact-allocation.json').write_text(json.dumps(changes,indent=2)+'\n')
                return cfg,kwargs
            ArenaEnvBuilder.compose_manager_cfg=compact_buffers
            from yam_arena import scene
            from isaaclab.sim.utils import clone
            from isaaclab.sim.spawners.shapes.shapes import spawn_cuboid
            original_object=scene.task_object
            def task_object(spec):
                obj=original_object(spec)
                if spec.name in G.CENTRES:
                    if G.MECHANICAL_ONLY:
                        from isaaclab_arena.assets.object import Object
                        from isaaclab_arena.utils.pose import Pose
                        initial=G.initial_poses()[spec.name]
                        obj=Object(name=spec.name,object_type=obj.object_type,spawner_cfg=obj.spawner_cfg,
                                   initial_pose=Pose(position_xyz=tuple((G.ORIGIN+initial['center']).tolist()),
                                                     rotation_xyzw=tuple(initial['quat_xyzw'])))
                    @clone
                    def spawn(path,cfg,*args,**kwargs):
                        prim=spawn_cuboid.__wrapped__(path,cfg,*args,**kwargs)
                        G.decorate(prim,spec.name)
                        if spec.name==list(G.FACETS)[-1]:G.finish_filters(prim.GetStage())
                        return prim
                    obj.spawner_cfg.func=spawn;obj.object_cfg.spawn=obj.spawner_cfg
                return obj
            scene.task_object=task_object
            import omni.physx
            from omni.physx.bindings._physx import SimulationEvent
            from pxr import PhysicsSchemaTools
            def event(e):
                if e.type==int(SimulationEvent.JOINT_BREAK):
                    events.append(dict(sample=len(poses),path=str(PhysicsSchemaTools.decodeSdfPath(*e.payload['jointPath']))))
            subscription=omni.physx.get_physx_interface().get_simulation_event_stream_v2().create_subscription_to_pop(event)
            installed=True
        original_ensure(self,goal)
        if view is None:
            from isaaclab.sim.utils.stage import get_current_stage_id
            import omni.physics.tensors as tensors
            simulation=tensors.create_simulation_view('torch',stage_id=get_current_stage_id())
            simulation.set_subspace_roots('/')
            view=simulation.create_rigid_body_view(G.ROOT+'/carton_*')
            paths=list(view.prim_paths)
            if set(p.rsplit('/',1)[-1] for p in paths)!=set(G.CENTRES):raise RuntimeError('Incomplete carton recording')
            (out/'mechanism-contract.json').write_text(json.dumps(dict(**G.contract(),paths=paths),indent=2)+'\n')
            if scoring.ENABLED:scoring.ACTIVE=scoring.Outcome(G.contract(),paths)
            from pxr import Usd
            import omni.usd
            stage=omni.usd.get_context().get_stage()
            if G.MECHANICAL_ONLY:
                material=G.crease_memory()
                material_order=[paths.index(G.ROOT+'/'+name) for name in G.CENTRES]
                material_targets=material.native_drives(stage,G.ROOT+'/carton_body/joints')
            sensors=[str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(G.ROOT))
                     if any('/'+arm+'/' in str(p.GetPath()) for arm in ('LeftArm','RightArm'))
                     and 'PhysxContactReportAPI' in p.GetAppliedSchemas()]
            if len(sensors)!=18:raise RuntimeError('Original robot contact coverage mismatch')
            # Record physical link poses plus original static camera extrinsics.
            # Never re-access camera.data after the official RGB read.
            from pxr import UsdGeom,UsdPhysics
            cache=UsdGeom.XformCache()
            robot_pose_view=simulation.create_rigid_body_view(sensors)
            robot_paths=list(robot_pose_view.prim_paths)
            for camera in ('top_cam','left_cam','right_cam'):
                prim=self._base.scene[camera]._sensor_prims[0].GetPrim()
                parent=prim.GetParent()
                while parent and not parent.HasAPI(UsdPhysics.RigidBodyAPI):parent=parent.GetParent()
                if parent:
                    matrix,_=cache.ComputeRelativeTransform(prim,parent)
                    body_index=robot_paths.index(str(parent.GetPath()))
                else:
                    matrix=cache.GetLocalToWorldTransform(prim);body_index=None
                qq=matrix.ExtractRotationQuat();pos=matrix.ExtractTranslation()
                camera_mounts.append(dict(name=camera,body_index=body_index,
                    body_path=None if body_index is None else robot_paths[body_index],
                    pos=list(pos),quat=list(qq.GetImaginary())+[qq.GetReal()]))
            (out/'camera-extrinsics.json').write_text(json.dumps(camera_mounts,indent=2)+'\n')
            contact_view=simulation.create_rigid_contact_view(sensors,[paths]*len(sensors),max_contact_data_count=65536)
            (out/'contact-contract.json').write_text(json.dumps(dict(sensors=list(contact_view.sensor_paths),
                filters=list(contact_view.filter_paths),capacity=65536),indent=2))
            # Private evidence only: no sensor data reaches the action policy.
            table=G.ROOT+'/table/geometry/mesh'
            if 'PhysicsCollisionAPI' not in stage.GetPrimAtPath(table).GetAppliedSchemas():
                raise RuntimeError('Original table collider missing')
            environment_view=simulation.create_rigid_contact_view(sensors,[sensors+[table]]*len(sensors),max_contact_data_count=65536)
            support_view=simulation.create_rigid_contact_view(paths,[[table]]*len(paths),max_contact_data_count=65536)
            (out/'environment-contact-contract.json').write_text(json.dumps(dict(
                robot_sensors=list(environment_view.sensor_paths),robot_filters=list(environment_view.filter_paths),
                carton_sensors=list(support_view.sensor_paths),carton_filters=list(support_view.filter_paths),
                table=table,capacity=65536),indent=2)+'\n')
            from isaaclab_physx.physics.physx_manager import PhysxManager
            original_step=PhysxManager.step
            def step(cls):
                original_step()
                p=view.get_transforms().detach().cpu().numpy().copy()
                v=view.get_velocities().detach().cpu().numpy().copy()
                poses.append(p);velocities.append(v)
                if material is not None:
                    rest=material.update(p[material_order]);material_rest.append(rest)
                    for attr,value in zip(material_targets,rest):attr.Set(float(value))
                contact_forces.append(contact_view.get_contact_force_matrix(dt=1/240).cpu().numpy().copy())
                counts=contact_view.get_contact_data(dt=1/240)[4].cpu().numpy()
                contact_counts.append(int(counts.sum()))
                if contact_counts[-1]>=65536:raise RuntimeError('Robot contact capacity exhausted')
                for sensor,force_rows,count_rows in ((environment_view,environment_forces,environment_counts),
                                                      (support_view,support_forces,support_counts)):
                    force=sensor.get_contact_force_matrix(dt=1/240).cpu().numpy().copy()
                    count=sensor.get_contact_data(dt=1/240)[4].cpu().numpy().copy()
                    if not np.isfinite(force).all() or int(count.sum())>=65536:
                        raise RuntimeError('Invalid or saturated private environment contact evidence')
                    force_rows.append(force);count_rows.append(count)
                if scoring.ENABLED:
                    scoring.ACTIVE.update(p,contact_counts[-1],support_counts[-1],broken=bool(events))
                    if len(poses)%12==0:score_trace.append(dict(sample=len(poses)-1,**scoring.ACTIVE.detail))
                if not np.isfinite(p).all() or not np.isfinite(v).all():raise RuntimeError('Nonfinite carton physics')
            PhysxManager.step=classmethod(step)
    def reset(self,*args,**kwargs):
        nonlocal resets
        if resets:raise RuntimeError('Fresh process required: broken-bond reconstruction not yet validated')
        resets+=1
        obs=original_reset(self,*args,**kwargs)
        import cv2,omni.usd
        for name,rgb in obs.images.items():
            np.save(out/f'setup_{name}.npy',rgb)
            cv2.imwrite(str(out/f'setup_{name}.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
        (out/'scene.usda').write_text(omni.usd.get_context().get_stage().GetRootLayer().ExportToString())
        return obs
    def observe(self):
        observation=original_observe(self)
        from inspect_robots_yam.packing import STATE_KEY
        from scipy.spatial.transform import Rotation
        robot_poses=robot_pose_view.get_transforms().cpu().numpy()
        cameras=[]
        for mount in camera_mounts:
            if mount['body_index'] is None:cameras.append(np.r_[mount['pos'],mount['quat']])
            else:
                body=robot_poses[mount['body_index']];rotation=Rotation.from_quat(body[3:])
                cameras.append(np.r_[body[:3]+rotation.apply(mount['pos']),
                    (rotation*Rotation.from_quat(mount['quat'])).as_quat()])
        camera_records.append(np.array(cameras))
        joint_records.append(np.asarray(observation.state[STATE_KEY],float).copy())
        camera_samples.append(len(poses)-1)
        if len(camera_records)%20==1:
            (out/'live-physics.json').write_text(json.dumps(dict(frames=len(camera_records),samples=len(poses),
                poses=poses[-1].tolist(),paths=paths,score=None if scoring.ACTIVE is None else scoring.ACTIVE.detail)))
        return observation
    official.YamArenaEmbodiment._ensure_sim=ensure;official.YamArenaEmbodiment.reset=reset
    official.YamArenaEmbodiment._observation=observe
    def save():
        (out/'break-events.json').write_text(json.dumps(events,indent=2)+'\n')
        if camera_records:np.savez_compressed(out/'camera-poses.npz',poses=camera_records,joint_pos=joint_records,
            physics_sample=camera_samples,cameras=['top_cam','left_cam','right_cam'],
            quaternion_convention='xyzw; native USD OpenGL camera axes; measured rigid-body pose times unchanged original camera mount')
        if material is not None:
            (out/'material-state.json').write_text(json.dumps(dict(plastic_work_J=material.plastic_work_J,
                final_rest_deg=material.rest.tolist(),plastic_rotation_deg=material.plastic_rotation_deg.tolist()),indent=2)+'\n')
        if scoring.ENABLED and scoring.ACTIVE is not None:
            (out/'live-score.json').write_text(json.dumps(dict(final=scoring.ACTIVE.detail,trace=score_trace),indent=2)+'\n')
        if poses:np.savez_compressed(out/'physics-substeps.npz',poses=np.stack(poses),velocities=np.stack(velocities),
            robot_contact_forces=np.array(contact_forces),contact_counts=contact_counts,contact_capacity=65536,paths=paths,physics_hz=240,
            material_rest_deg=np.array(material_rest),
            environment_contact_forces=np.array(environment_forces),environment_contact_counts=np.array(environment_counts),
            support_contact_forces=np.array(support_forces),support_contact_counts=np.array(support_counts))
    return save
