"""Fixed YAM task runtime and optional known-layout demonstration controller."""
import atexit, json
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

def install(geometry,out,plan=None):
    from yam_arena import inspect_embodiment as official
    initial=official.YamArenaEmbodiment._ensure_sim;observe0=official.YamArenaEmbodiment._observation
    installed=False;view=None;contact=None;object_contact=None;sim=None
    poses=[];vels=[];counts=[];forces=[];ocounts=[];oforces=[];onets=[];joints=[];obs_samples=[];pending=[];shapes=None
    contact_capacity=getattr(geometry,'CONTACT_CAPACITY',65536)
    paths=[];sensors=[];filters=[];sample_count=0;
    applied_repair_actions=[];executed_indices=[]
    adaptive=plan is not None and any(r.get("label")!="Reset and hold" for r in plan)
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
        if hasattr(geometry,'after_contact_batch') and len(batch):
            geometry.after_contact_batch(poses[-len(batch):],oforces[-len(batch):],paths,filters,len(poses),PHYSICS_HZ)
    def ensure(self,goal):
        nonlocal installed,view,contact,object_contact,sim,paths,sensors,filters
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
                        p=spawn_cuboid.__wrapped__(path,cfg,*a,**kw);geometry.build(p.GetStage());return p
                    obj.spawner_cfg.func=spawn;obj.object_cfg.spawn=obj.spawner_cfg
                return obj
            scene.task_object=task_object;installed=True
        initial(self,goal)
        if view is None:
            import omni.physics.tensors as tensors, omni.usd
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
            (out/'contact-contract.json').write_text(json.dumps(dict(paths=paths,sensors=sensors,filters=filters,physics_hz=PHYSICS_HZ),indent=2)+'\n')
            (out/'native-masses.json').write_text(json.dumps(dict(paths=paths,masses_kg=view.get_masses().cpu().numpy().tolist()),indent=2)+'\n')
            stage.GetRootLayer().Export(str(out/'scene.usda'))
            from isaaclab_physx.physics.physx_manager import PhysxManager
            if hasattr(geometry,'install_proof_load'):
                geometry.install_proof_load(sim)
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
            label=plan[min(len(joints)-1,len(plan)-1)]['label'] if plan is not None else 'policy episode'
            if executed_indices:label=plan[executed_indices[-1]]['label']
            progress=json.dumps(dict(frame=len(joints)-1,poses=poses[-1].tolist(),paths=paths,stage=label,task_metrics=geometry.debug_metrics() if hasattr(geometry,'debug_metrics') else None))+'\n'
            (out/'progress.json').write_text(progress)
            if hasattr(geometry,'debug_metrics'):
                with (out/'progress.jsonl').open('a') as trace:trace.write(progress)
        return obs
    official.YamArenaEmbodiment._ensure_sim=ensure;official.YamArenaEmbodiment._observation=observation
    from yam_arena.scripted import YamScriptedPolicy
    from inspect_robots.types import Action,ActionChunk
    from inspect_robots_yam.packing import pack
    from yam_arena import ik
    policy_reset=YamScriptedPolicy.reset
    def reset(self,scene):
        policy_reset(self,scene);self._probe_i=0;self._feedback_transition=False
        self._joint_hold=None;self._tcp_retreat=None
    def act(self,obs):
        index=min(self._probe_i,len(plan)-1)
        from tool_return import PREFIX_ACTIONS
        if adaptive and self._probe_i>=PREFIX_ACTIONS:
            from inspect_robots_yam.packing import STATE_KEY
            from tool_return import ReturnDriver
            if not hasattr(self,'driver_return'):
                self.driver_return=ReturnDriver(applied_repair_actions[-1])
            phase=self.driver_return.phase
            bodies={p.rsplit('/',1)[-1]:v for p,v in zip(paths,poses[-1])}
            if phase in ('Align lifting handle above rack','Seat lifting handle behind posts','Release lifting handle','Clear lifting handle'):
                from repair_plan import correct_tool
                base=plan[self.repair_schedule.index]
                left=correct_tool(self,base,obs,np.array(base['action']),poses[-1],paths)
                self.driver_return.action[:7]=left[:7]
            row=self.driver_return.step(np.asarray(obs.state[STATE_KEY]),bodies,geometry.debug_metrics())
            plan.append(dict(label=phase,action=row.tolist()))
            executed_indices.append(len(plan)-1)
            applied_repair_actions.append(row.copy())
            self._probe_i+=1
            return ActionChunk(actions=[Action(data=pack(row[:7],row[7:]))],control_hz=20)
        if adaptive:
            from repair_plan import RepairSchedule
            if not hasattr(self,'repair_schedule'):self.repair_schedule=RepairSchedule(plan)
            index=self.repair_schedule.next_index(geometry.debug_metrics())
            executed_indices.append(index)
        entry=plan[index]
        row=np.array(entry['action'],float);self._probe_i+=1
        if 'park_tool' in entry:
            from repair_plan import park_joints
            row=park_joints(self,entry,obs,row,poses[-1],paths)
        if entry.get('hold_tool') and hasattr(self,'drive_command'):row[:6]=self.drive_command
        if 'stock_feed' in entry:
            from repair_plan import feed_recorded_stock
            row=feed_recorded_stock(self,entry,obs,row,poses[-1],paths)
        if 'stock_target' in entry:
            from repair_plan import track_stock
            row=track_stock(self,entry,obs,row,poses[-1],paths)
        if entry.get('hold_wood') and hasattr(self,'wood_q'):row[7:13]=self.wood_q
        if 'clear_stock' in entry:
            from repair_plan import clear_stock
            row=clear_stock(self,entry,obs,row)
        if 'repair_pose' in entry:
            from repair_plan import align_patch
            row=align_patch(self,entry,obs,row,poses[-1],paths)
        if ('stock_target' in entry or 'stock_feed' in entry or 'repair_pose' in entry) and self._probe_i%10==0 and hasattr(self,'carry_diagnostics'):
            with (out/'carry-servo.jsonl').open('a') as trace:trace.write(json.dumps(dict(action=self._probe_i,**self.carry_diagnostics))+'\n')
        if 'unhook' in entry:
            from repair_plan import unhook
            row=unhook(self,entry,obs,row,poses[-1],paths)
        if 'repair_drive' in entry:
            from repair_plan import correct_tool
            if not hasattr(self,'pickup_q'):
                from inspect_robots_yam.packing import STATE_KEY
                self.pickup_q=np.asarray(obs.state[STATE_KEY])[:6].copy()
            row=correct_tool(self,entry,obs,row,poses[-1],paths)
            if self._probe_i%5==0:
                with (out/'tool-servo.jsonl').open('a') as trace:
                    trace.write(json.dumps(dict(action=self._probe_i,**self.tool_diagnostics))+'\n')
        if hasattr(geometry,'debug_metrics'):applied_repair_actions.append(row.copy())
        return ActionChunk(actions=[Action(data=pack(row[:7],row[7:]))],control_hz=20)
    if plan is not None:
        YamScriptedPolicy.reset=reset;YamScriptedPolicy.act=act
    def save():
        flush()
        if applied_repair_actions:np.save(out/'applied-actions.npy',np.array(applied_repair_actions))
        if executed_indices:
            (out/'scheduled-plan.json').write_text(json.dumps(dict(steps=plan))+'\n')
            (out/'plan.json').write_text(json.dumps(dict(steps=[plan[i] for i in executed_indices[:len(applied_repair_actions)]]))+'\n')
            np.save(out/'executed-indices.npy',np.asarray(executed_indices[:len(applied_repair_actions)]))
        if hasattr(geometry,"save_probe"):
            geometry.save_probe(out)
        if poses:np.savez_compressed(out/'physics.npz',poses=poses,velocities=vels,robot_contact_counts=counts,robot_contact_forces=forces,object_contact_counts=ocounts,object_contact_forces=oforces,object_net_contact_forces=onets,paths=paths,joint_pos=joints,observation_samples=obs_samples,physics_hz=PHYSICS_HZ)
        (out/'probe-summary.json').write_text(json.dumps(dict(observations=len(joints),physics_samples=len(poses),controller_installed=plan is not None,material_response_implemented=hasattr(geometry,'film'),final_score=geometry.debug_metrics().get('score',0.),physics_hz=PHYSICS_HZ),indent=2)+'\n')
    return save
