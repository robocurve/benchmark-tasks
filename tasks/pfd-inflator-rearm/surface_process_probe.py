"""Install the inert workcell and sample native PhysX contact at 240 Hz."""
import atexit
import json
import numpy as np
WORLD='/World/envs/env_0'
BODY_ITERATIONS=(16,1)
SCENE_CCD=False
SPECULATIVE_CCD=False
PHYSICS_HZ=240

def box(stage,path,center,size,color=(.5,.5,.5),collision=True,opacity=1.):
    from pxr import UsdGeom,Gf,UsdShade,Sdf,UsdPhysics,PhysxSchema
    from yam_arena.station import linear_rgb
    shape=UsdGeom.Cube.Define(stage,path);shape.CreateSizeAttr(1.)
    shape.AddTranslateOp().Set(Gf.Vec3d(*map(float,center)))
    shape.AddScaleOp().Set(Gf.Vec3f(*map(float,size)))
    mat=UsdShade.Material.Define(stage,path+'/appearance');s=UsdShade.Shader.Define(stage,path+'/appearance/surface');s.CreateIdAttr('UsdPreviewSurface')
    s.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(color)))
    s.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.4)
    s.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(float(opacity))
    mat.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(mat)
    if collision:
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim());pc=PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim());pc.CreateContactOffsetAttr(.00002);pc.CreateRestOffsetAttr(0.)
        pm=UsdShade.Material.Define(stage,path+'/friction');m=UsdPhysics.MaterialAPI.Apply(pm.GetPrim());m.CreateStaticFrictionAttr(.5);m.CreateDynamicFrictionAttr(.4);m.CreateRestitutionAttr(0.)
        UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(pm,materialPurpose='physics')
    return shape

def body(stage,path,position,mass):
    from pxr import UsdGeom,UsdPhysics,PhysxSchema,Gf
    p=UsdGeom.Xform.Define(stage,path).GetPrim();UsdGeom.Xformable(p).AddTranslateOp().Set(Gf.Vec3d(*map(float,position)))
    UsdPhysics.RigidBodyAPI.Apply(p);UsdPhysics.MassAPI.Apply(p).CreateMassAttr(float(mass))
    rb=PhysxSchema.PhysxRigidBodyAPI.Apply(p);rb.CreateSolverPositionIterationCountAttr(BODY_ITERATIONS[0]);rb.CreateSolverVelocityIterationCountAttr(BODY_ITERATIONS[1])
    rb.CreateMaxDepenetrationVelocityAttr(.1);rb.CreateEnableCCDAttr(True)
    rb.CreateEnableSpeculativeCCDAttr(SPECULATIVE_CCD)
    PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.)
    return p


def install(geometry, out, plan, stock_order):
    import disposition_score as grading
    from yam_arena import inspect_embodiment as official
    initial=official.YamArenaEmbodiment._ensure_sim;observe0=official.YamArenaEmbodiment._observation
    installed=False;view=None;contact=None;object_contact=None;sim=None
    poses=[];vels=[];counts=[];forces=[];ocounts=[];oforces=[];onets=[];joints=[];obs_samples=[];pending=[];shapes=None
    contact_capacity=getattr(geometry,'CONTACT_CAPACITY',65536)
    paths=[]; sensors=[]; filters=[]; sample_count=0
    grader=None
    def flush():
        if not pending:return
        import torch
        batch=torch.stack(pending).cpu().numpy();pending.clear()
        for row in batch:
            values=[];start=0
            for shape in shapes:
                end=start+int(np.prod(shape));values.append(row[start:end].reshape(shape).copy());start=end
            if not all(np.isfinite(v).all() for v in values):raise RuntimeError('Non-finite native physics')
            a,b,c,d,e,f,g=values
            if c.sum()>=contact_capacity or e.sum()>=contact_capacity:raise RuntimeError('Contact buffer full')
            poses.append(a);vels.append(b);counts.append(c);forces.append(d);ocounts.append(e);oforces.append(f);onets.append(g)
    def ensure(self,goal):
        nonlocal installed,view,contact,object_contact,sim,paths,sensors,filters,grader
        assert self.rig==official.station.rig('calibrated') and (self.cam_width,self.cam_height)==(320,176)
        assert self.info.control_hz==20 and self.object_jitter==0
        if not installed:
            from isaaclab.app import AppLauncher
            assert official._APP is None
            official._APP=AppLauncher(headless=True,device=self.device,enable_cameras=True,multi_gpu=False,kit_args='--/UJITSO/geometry=false').app
            atexit.register(official._close_app_at_exit)
            from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
            compose=ArenaEnvBuilder.compose_manager_cfg
            def cfg(builder):
                c,k=compose(builder);c.sim.dt=1/PHYSICS_HZ;c.decimation=PHYSICS_HZ//20;c.sim.render_interval=PHYSICS_HZ//20
                c.sim.physics.solve_articulation_contact_last=True
                c.sim.physics.enable_ccd=SCENE_CCD
                # Native thin-object contact resolution. Robot, control and cameras unchanged.
                c.sim.physics.friction_correlation_distance=.0001;c.sim.physics.friction_offset_threshold=.00005
                for name,val in dict(gpu_found_lost_pairs_capacity=262144,gpu_found_lost_aggregate_pairs_capacity=262144,gpu_total_aggregate_pairs_capacity=262144,gpu_max_rigid_contact_count=262144,gpu_max_rigid_patch_count=65536).items():setattr(c.sim.physics,name,val)
                (out/'physics-settings.json').write_text(json.dumps(dict(dt=c.sim.dt,decimation=c.decimation,solver_type=c.sim.physics.solver_type,enable_stabilization=c.sim.physics.enable_stabilization,enable_ccd=SCENE_CCD,body_iterations=BODY_ITERATIONS),indent=2)+'\n')
                return c,k
            ArenaEnvBuilder.compose_manager_cfg=cfg
            from yam_arena import scene
            from isaaclab.sim.utils import clone
            from isaaclab.sim.spawners.shapes.shapes import spawn_cuboid
            old=scene.task_object
            def task_object(spec):
                obj=old(spec)
                if spec.name=='build_anchor':
                    @clone
                    def spawn(path,cfg,*a,**kw):
                        p=spawn_cuboid.__wrapped__(path,cfg,*a,**kw);geometry.build(p.GetStage(), stock_order);return p
                    obj.spawner_cfg.func=spawn;obj.object_cfg.spawn=obj.spawner_cfg
                return obj
            scene.task_object=task_object;installed=True
        initial(self,goal)
        if view is None:
            import omni.physics.tensors as tensors,omni.usd
            from isaaclab.sim.utils.stage import get_current_stage_id
            from pxr import Usd
            sim=tensors.create_simulation_view('torch',stage_id=get_current_stage_id());sim.set_subspace_roots('/')
            paths=[WORLD+'/'+n for n in geometry.BODIES];view=sim.create_rigid_body_view(paths);paths=list(view.prim_paths)
            assert len(paths)==len(geometry.BODIES)
            stage=omni.usd.get_context().get_stage()
            sensors=[str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(WORLD)) if any('/'+a+'/' in str(p.GetPath()) for a in ['LeftArm','RightArm']) and 'PhysxContactReportAPI' in p.GetAppliedSchemas()]
            assert len(sensors)==18
            filters=paths+[WORLD+'/'+n for n in geometry.FIXED]+[WORLD+'/table/geometry/mesh']+sensors
            contact=sim.create_rigid_contact_view(sensors,[filters]*len(sensors),max_contact_data_count=contact_capacity)
            object_contact=sim.create_rigid_contact_view(paths,[filters]*len(paths),max_contact_data_count=contact_capacity)
            grader=grading.CURRENT=grading.Grader(stage,paths,filters)
            (out/'contact-contract.json').write_text(json.dumps(dict(paths=paths,sensors=sensors,filters=filters,physics_hz=PHYSICS_HZ),indent=2)+'\n')
            (out/'native-masses.json').write_text(json.dumps(dict(paths=paths,masses_kg=view.get_masses().cpu().numpy().tolist()),indent=2)+'\n')
            stage.GetRootLayer().Export(str(out/'scene.usda'))
            from isaaclab_physx.physics.physx_manager import PhysxManager
            step0=PhysxManager.step
            def step(cls):
                nonlocal shapes,sample_count
                step0()
                sample_count+=1
                arrays=[view.get_transforms(),view.get_velocities(),contact.get_contact_data(dt=1/PHYSICS_HZ)[4],contact.get_contact_force_matrix(dt=1/PHYSICS_HZ),object_contact.get_contact_data(dt=1/PHYSICS_HZ)[4],object_contact.get_contact_force_matrix(dt=1/PHYSICS_HZ),object_contact.get_net_contact_forces(dt=1/PHYSICS_HZ)]
                import torch
                if shapes is None:shapes=[tuple(a.shape) for a in arrays]
                pending.append(torch.cat([a.reshape(-1).float() for a in arrays]))
                if len(pending)>=12:flush()
            PhysxManager.step=classmethod(step)

    def observation(self):
        flush()
        obs=observe0(self);flush()
        from inspect_robots_yam.packing import STATE_KEY
        joints.append(np.asarray(obs.state[STATE_KEY],float));obs_samples.append(len(poses)-1)
        if len(joints)==1:
            import cv2
            for k,v in obs.images.items():cv2.imwrite(str(out/f'setup_{k}.png'),cv2.cvtColor(v,cv2.COLOR_RGB2BGR))
        if len(joints)%20==1 and poses:
            (out/'progress.json').write_text(json.dumps(dict(frame=len(joints)-1)))
        return obs

    native_task_step=official.YamArenaEmbodiment._task_step
    def scored_task_step(self):
        flush()
        start=obs_samples[-1]+1
        if len(poses)-start != 12:
            raise RuntimeError("Expected twelve physics samples per action")
        batch=[np.asarray(rows[start:]) for rows in (poses,counts,ocounts,oforces)]
        grader.update(*batch,None)
        with (out/'handoff-trace.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(action=len(joints),**grader.metrics))+'\n')
        return native_task_step(self)

    official.YamArenaEmbodiment._task_step=scored_task_step
    official.YamArenaEmbodiment._ensure_sim=ensure
    official.YamArenaEmbodiment._observation=observation
    inspection=None
    if plan is not None:
        from yam_arena.scripted import YamScriptedPolicy
        from inspect_robots.types import Action, ActionChunk
        from inspect_robots_yam.packing import pack
        from stock_policy import StockInspection
        inspection=StockInspection()
        index=0
        def act(self,obs):
            nonlocal index
            row=plan[index]
            index+=1
            a=inspection.command(row,obs.images,row['action'])
            return ActionChunk(actions=[Action(data=pack(a[:7],a[7:]))],control_hz=20)
        YamScriptedPolicy.act=act

    def save():
        flush()
        if grader is not None:
            (out/'handoff-result.json').write_text(json.dumps(dict(final=grader.metrics),indent=2))
        if inspection is not None:
            inspection.save(out)
        if poses:
            np.savez_compressed(out/'physics.npz',poses=poses,velocities=vels,robot_contact_counts=counts,robot_contact_forces=forces,object_contact_counts=ocounts,object_contact_forces=oforces,object_net_contact_forces=onets,paths=paths,joint_pos=joints,observation_samples=obs_samples,physics_hz=PHYSICS_HZ)
    return save
