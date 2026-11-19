"""Actual native-fluid diagnostic, without robot policy or task-success claims.

A hollow glass under the nozzle receives a measured inflow, then settles
for two seconds. The diagnostic table can be raised together with the glass. This isolates GPU fluid/readback/collision behavior before the
full robot demonstration. The supplied task design review covers this probe.
"""
import argparse
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import socket
import shutil
import subprocess
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    parser.add_argument('--device',default='cuda:0')
    parser.add_argument('--glass-lift',type=float,default=0.,help='Raise the entire diagnostic table and glass together, metres')
    parser.add_argument('--flow-ml-s',type=float,default=18.,help='Fixed diagnostic inlet rate, not a hand-pump validation')
    parser.add_argument('--pour-ml',type=float,default=36.,help='Nominal inflow volume followed by two seconds of settling')
    parser.add_argument('--initial-ml',type=float,default=0.,help='Initial fixed-mass liquid lattice, followed by --pour-ml additional inflow')
    parser.add_argument('--prefill',action='store_true',help='Also author --pour-ml as initial liquid before attach; no pouring')
    parser.add_argument('--physics-hz',type=int,choices=(240,480),help='Native physics rate; defaults to the task physics_hz')
    parser.add_argument('--mass-scale',type=float,default=1.,help='Particle-mass metrology multiplier; scaled runs are not beer demonstrations')
    parser.add_argument('--explicit-mass',action='store_true',help='Use fixed initial total mass instead of normalized set density; requires --prefill')
    parser.add_argument('--rigid-position-iterations',type=int,choices=(4,16,64),help='Diagnostic glass solver position iterations, before attach')
    parser.add_argument('--rigid-velocity-iterations',type=int,choices=(1,4,16),help='Diagnostic glass solver velocity iterations, before attach')
    parser.add_argument('--solver-iterations',type=int,choices=(4,16,64),help='Diagnostic native PBD position iterations, before attach')
    parser.add_argument('--cohesion',type=float,help='Diagnostic PBD cohesion override, before native attach')
    parser.add_argument('--surface-tension',type=float,help='Diagnostic PBD surface-tension override, before native attach')
    parser.add_argument('--stop-on-spill',action='store_true',help='Stop a failed native diagnostic prefix; never report it as a completed endpoint')
    parser.add_argument('--pvd',action='store_true',help='Record native OmniPVD debug data in output/pvd for independent particle-mass inspection')
    args=parser.parse_args()
    try:
        if not args.device.startswith('cuda:'):raise ValueError
        physics_gpu=int(args.device.split(':')[1])
        if physics_gpu<0:raise ValueError
    except (ValueError,IndexError):parser.error('Expected a native GPU renderer/physics device cuda:N')
    if (not math.isfinite(args.glass_lift) or not 0 <= args.glass_lift <= .20
            or not math.isfinite(args.flow_ml_s) or not 0 < args.flow_ml_s <= 1000
            or not math.isfinite(args.pour_ml) or not 0 < args.pour_ml <= 4000
            or not math.isfinite(args.initial_ml) or not 0 <= args.initial_ml <= 4000
            or not math.isfinite(args.mass_scale) or args.mass_scale<=0):
        parser.error('Expected lift in [0,.20], flow in (0,1000] mL/s, pour in (0,4000] mL, and initial volume in [0,4000] mL')
    if (args.explicit_mass or args.mass_scale != 1.) and not args.prefill:
        parser.error('Amplified or fixed-mass metrology requires --prefill')
    if any(v is not None and (not math.isfinite(v) or v<0) for v in (args.cohesion,args.surface_tension)):
        parser.error('Expected nonnegative finite material diagnostic coefficients')
    args.output=args.output.resolve()
    task_directory=Path(__file__).resolve().parent
    if args.output==task_directory or task_directory in args.output.parents:
        parser.error('Keep calibration outputs outside task sources')
    from tools.run_recovery_pilot import git_output
    source_root=Path(git_output.__code__.co_filename).resolve().parents[1]
    revision=(git_output(source_root,'rev-parse','HEAD') or b'').decode().strip() or None
    args.output.mkdir(parents=True,exist_ok=False)
    receipt=dict(scope='Native particle/fluid metrology, not a beer or task demonstration',hostname=socket.gethostname(),
        gpu=subprocess.check_output(['nvidia-smi','--query-gpu=index,name,uuid','--format=csv,noheader'],text=True),
        git_revision=revision,
        source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                      for p in Path(__file__).parent.glob('*.py') if not p.name.startswith('._')},
        versions={k:importlib.metadata.version(k) for k in ('isaacsim','isaaclab')},status='starting')
    app=None
    pvd_previous=None
    started=time.monotonic()
    try:
        from isaaclab.app import AppLauncher
        app=AppLauncher(headless=True,device=args.device,enable_cameras=False,multi_gpu=False).app
        import carb
        import numpy as np
        import warp as wp
        import omni.usd
        from omni.physx import get_physx_interface
        from omni.physx.bindings import _physx as pb
        from isaaclab.sim import SimulationContext, SimulationCfg, UsdFileCfg, CuboidCfg, CollisionPropertiesCfg
        from resist_isaac.runtime import _task_module
        from resist_isaac.usd_assets import create_asset
        from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,PhysicsSchemaTools,Sdf,Vt
        if args.pvd:
            settings=carb.settings.get_settings()
            pvd_previous={key:settings.get(key) for key in
                (pb.SETTING_OMNIPVD_ENABLED,pb.SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY)}
            pvd_dir=args.output/'pvd';pvd_dir.mkdir()
            receipt['pvd']=dict(status='recording_requested',directory='pvd',previous_settings=pvd_previous)
            settings.set(pb.SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY,str(pvd_dir.resolve())+'/')
            settings.set(pb.SETTING_OMNIPVD_ENABLED,True)
        module,_=_task_module(Path(__file__).parent)
        from importlib import import_module
        fluid_module=import_module(module.__package__+'.fluid')
        score_module=import_module(module.__package__+'.scoring')
        hz=args.physics_hz or module.DraftBeer.physics_hz
        if hz not in (240,480):raise ValueError('Expected task physics_hz of 240 or 480')
        initial_ml=args.initial_ml+(args.pour_ml if args.prefill else 0.)
        has_initial=initial_ml>0
        baseline_steps=round(1.5*hz)
        receipt['physics_hz']=hz
        updates=(pb.SETTING_UPDATE_TO_USD,pb.SETTING_UPDATE_PARTICLES_TO_USD,pb.SETTING_UPDATE_VELOCITIES_TO_USD)
        for key in updates:carb.settings.get_settings().set(key,True)
        sim=SimulationContext(SimulationCfg(dt=1/hz,device='cpu',use_fabric=False))
        # Match the validated task: CPU readback with explicitly selected GPU PBD.
        carb.settings.get_settings().set('/physics/cudaDevice',physics_gpu)
        carb.settings.get_settings().set(pb.SETTING_SUPPRESS_READBACK,False)
        carb.settings.get_settings().set('/physics/disableContactProcessing',False)
        for key in updates:carb.settings.get_settings().set(key,True)
        receipt['physics_settings_before_attach']={k:carb.settings.get_settings().get(k)
            for k in (pb.SETTING_SUPPRESS_READBACK,'/physics/disableContactProcessing',*updates)}
        stage=omni.usd.get_context().get_stage()
        native_scene=PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(sim.cfg.physics_prim_path))
        native_scene.CreateEnableGPUDynamicsAttr(True)
        native_scene.CreateBroadphaseTypeAttr('GPU')
        glass=create_asset(module.geometry()['glass'],args.output/'glass.usda')
        asset_stage=Usd.Stage.Open(str(glass))
        for prim in asset_stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
                # Sleeping bodies stop reporting support impulses during tare.
                body=PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
                body.CreateSleepThresholdAttr(0.)
                if args.rigid_position_iterations is not None:
                    body.CreateSolverPositionIterationCountAttr(args.rigid_position_iterations)
                if args.rigid_velocity_iterations is not None:
                    body.CreateSolverVelocityIterationCountAttr(args.rigid_velocity_iterations)
                mass=UsdPhysics.MassAPI(prim)
                receipt['authored_glass_mass']=dict(path=str(prim.GetPath()),
                    mass_kg=mass.GetMassAttr().Get(),density=mass.GetDensityAttr().Get(),
                    sleep_threshold=body.GetSleepThresholdAttr().Get(),
                    solver_position_iterations=body.GetSolverPositionIterationCountAttr().Get(),
                    solver_velocity_iterations=body.GetSolverVelocityIterationCountAttr().Get())
        asset_stage.GetRootLayer().Save()
        cfg=UsdFileCfg(usd_path=str(glass))
        cfg.func('/World/glass',cfg,translation=(*module.NOZZLE[:2],.7475+args.glass_lift))
        table=CuboidCfg(size=(1.,1.,.05),collision_props=CollisionPropertiesCfg(collision_enabled=True))
        table.func('/World/table',table,translation=(*module.NOZZLE[:2],.7225+args.glass_lift))
        root=UsdGeom.Xform.Define(stage,'/World/beer')
        fluid_module.create_fluid(stage,str(root.GetPath()))
        from omni.physx.scripts import particleUtils
        particleUtils.add_pbd_particle_material(stage,str(root.GetPath())+'/material',
            cohesion=args.cohesion,surface_tension=args.surface_tension)
        material=stage.GetPrimAtPath(str(root.GetPath())+'/material')
        receipt['native_pbd_material']={a.GetName():a.Get() for a in material.GetAttributes()
            if a.GetName().startswith('physxPBDMaterial:')}
        system=PhysxSchema.PhysxParticleSystem.Get(stage,str(root.GetPath())+'/system')
        system.CreateSimulationOwnerRel().SetTargets([Sdf.Path(sim.cfg.physics_prim_path)])
        if args.solver_iterations is not None:
            system.CreateSolverPositionIterationCountAttr(args.solver_iterations)
        receipt['native_particle_position_iterations']=system.GetSolverPositionIterationCountAttr().Get()
        fluid=fluid_module.NativeBeerFluid(stage,str(root.GetPath()))
        mpu=UsdGeom.GetStageMetersPerUnit(stage)
        mass=UsdPhysics.MassAPI(fluid.points.GetPrim())
        initial_points=None
        if has_initial:
            n=math.floor(initial_ml/fluid_module.PARTICLE_ML)
            if n<1 or n>fluid_module.MAX_PARTICLES:raise ValueError('Prefill volume exceeds the native particle capacity')
            spacing=fluid_module.SPACING
            radius=score_module.INNER_RADIUS-score_module.REST_OFFSET
            layers=[]
            remaining=n;layer=0
            while remaining:
                sites=fluid_module.packed_layer(radius,layer)
                take=min(remaining,len(sites))
                positions=np.tile(np.asarray([*module.NOZZLE[:2],.7475+args.glass_lift]),(take,1))
                positions[:,:2]+=sites[:take]
                positions[:,2]+=score_module.BASE+score_module.REST_OFFSET+layer*fluid_module.LAYER_SPACING
                layers.append(positions)
                remaining-=take;layer+=1
            initial_points=np.concatenate(layers)
            if initial_points[:,2].max()>.7475+args.glass_lift+score_module.RIM-score_module.REST_OFFSET:
                raise ValueError('Prefill lattice exceeds the glass interior')
            # Initial calibration configuration, before native physics attaches.
            fluid.points.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(initial_points.astype(np.float32)))
            fluid.points.GetVelocitiesAttr().Set(Vt.Vec3fArray.FromNumpy(np.zeros((n,3),dtype=np.float32)))
            fluid.points.GetWidthsAttr().Set(Vt.FloatArray([spacing]*n))
            fluid.count=n
            fluid._surface_enabled.Set(True)
            np.save(args.output/'initial-prefill.npy',initial_points,allow_pickle=False)
        expected_particle_mass=fluid_module.PARTICLE_MASS_KG*args.mass_scale
        mass.GetMassAttr().Set(fluid.count*expected_particle_mass if args.explicit_mass else 0.)
        mass.GetDensityAttr().Set(fluid_module.NATIVE_SET_DENSITY*args.mass_scale)
        receipt['mass_metrology']=dict(mode='explicit_mass' if args.explicit_mass else 'normalized_set_density',
            scale=args.mass_scale,expected_particle_mass_kg=expected_particle_mass,
            expected_initial_mass_kg=fluid.count*expected_particle_mass,
            pbd_material_density_kg_m3=fluid_module.DENSITY_KG_M3)
        receipt['authored_particle_mass']=dict(mass_kg=mass.GetMassAttr().Get(),
            density=mass.GetDensityAttr().Get())
        receipt['units']=dict(stage_meters_per_unit=mpu,gravity_cfg_m_s2=list(sim.cfg.gravity),
            vendor_water_viscosity=.0000017/mpu**3,vendor_water_surface_tension=.0074*mpu**3)
        sim.reset()
        carb.settings.get_settings().set('/physics/disableContactProcessing',False)
        support_path='/World/table'
        support_roots=[support_path]
        support=sim.physics_manager.get_physics_sim_view().create_rigid_contact_view(
            '/World/glass/glass',filter_patterns=[],max_contact_data_count=256)
        bindings=dict(sensors=list(support.sensor_paths),sensor_count=support.sensor_count,
            allowed_support_roots=support_roots,actual_support_actor_paths=[])
        receipt['weighing_bindings']=bindings
        if bindings['sensors']!=['/World/glass/glass'] or bindings['sensor_count']!=1:
            raise RuntimeError('Native weighing contact sensor did not bind to glass')
        glass_body=sim.physics_manager.get_physics_sim_view().create_rigid_body_view('/World/glass/glass')
        if glass_body.count!=1:raise RuntimeError('Native weighing glass pose did not bind')
        def glass_pose():
            value=glass_body.get_transforms()
            if hasattr(value,'numpy'):value=value.numpy()
            value=np.asarray(value)
            if value.shape!=(1,7) or not np.isfinite(value).all():
                raise RuntimeError('Invalid native weighing glass transform')
            get_physx_interface().update_transformations(True,True,True,False)
            matrix=UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath('/World/glass/glass'))
            if abs(float(value[0,2])-(.7475+args.glass_lift))>.002:
                raise RuntimeError('Diagnostic glass left its declared table height')
            return dict(native_xyz_quat_xyzw=value[0].tolist(),usd_world_matrix=np.asarray(matrix).tolist())
        receipt['glass_pose_initial']=glass_pose()
        stage.GetRootLayer().Export(str(args.output/'initial-stage.usda'))
        carb.settings.get_settings().set(pb.SETTING_SUPPRESS_READBACK,False)
        for key in updates:carb.settings.get_settings().set(key,True)
        receipt['physics_settings']={k:carb.settings.get_settings().get(k)
            for k in (pb.SETTING_SUPPRESS_READBACK,'/physics/disableContactProcessing',*updates)}
        scene=PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath(sim.cfg.physics_prim_path))
        physics_dt=float(sim.get_physics_dt())
        if not math.isclose(physics_dt,1/hz,rel_tol=0.,abs_tol=1e-10):
            raise RuntimeError('Native diagnostic physics timestep changed')
        receipt['physics_hz']=1/physics_dt
        receipt['physics_gpu_index']=int(carb.settings.get_settings().get('/physics/cudaDevice'))
        if receipt['physics_gpu_index']!=physics_gpu:raise RuntimeError('Native particle GPU selection changed')
        receipt['native_scene']=dict(path=sim.cfg.physics_prim_path,
            enable_gpu_dynamics=scene.GetEnableGPUDynamicsAttr().Get(),
            broadphase=scene.GetBroadphaseTypeAttr().Get(),
            time_steps_per_second=scene.GetTimeStepsPerSecondAttr().Get(),
            physics_dt_s=physics_dt,
            owner=[str(x) for x in system.GetSimulationOwnerRel().GetTargets()])
        receipt['probe_options']=dict(glass_lift_m=args.glass_lift,flow_ml_s=args.flow_ml_s,
            pour_ml=args.pour_ml,initial_ml=initial_ml,prefill=args.prefill,physics_hz=hz,
            mass_scale=args.mass_scale,explicit_mass=args.explicit_mass,
            cohesion_override=args.cohesion,surface_tension_override=args.surface_tension,
            solver_iterations_override=args.solver_iterations,stop_on_spill=args.stop_on_spill,
            rigid_position_iterations=args.rigid_position_iterations,
            rigid_velocity_iterations=args.rigid_velocity_iterations)
        source=args.output/'source';source.mkdir()
        for p in Path(__file__).parent.glob('*.py'):
            if not p.name.startswith('._'):shutil.copy2(p,source/p.name)
        stats_interface=pb.acquire_physx_statistics_interface()
        receipt['native_statistics']=[]

        def statistics(step):
            stats=pb.PhysicsSceneStats()
            try:
                valid=stats_interface.get_physx_scene_statistics(omni.usd.get_context().get_stage_id(),
                    PhysicsSchemaTools.sdfPathToInt(Sdf.Path(sim.cfg.physics_prim_path)),stats)
                values={k:getattr(stats,k) for k in dir(stats) if ('particle' in k or k=='nb_dynamic_rigids')}
                receipt['native_statistics'].append(dict(step=step,valid=valid,**values))
            except Exception as error:receipt['native_statistics'].append(dict(step=step,error=str(error)))

        scorer=score_module.BeerScore()
        moved=False
        statistics(0)
        pour_s=0. if args.prefill else args.pour_ml/args.flow_ml_s
        steps=math.ceil((pour_s+2.)*hz)
        tare_forces=[];loaded_forces=[];actors={}
        previous_points=fluid.positions().copy()
        previous_pose=receipt['glass_pose_initial']
        escape_saved=False
        peak_clear_horizontal_speed=0.
        bulk_window=[]
        def support_force():
            values=[np.array(a.numpy(),copy=True) for a in support.get_raw_contact_data(dt=1.)]
            if len(values)!=7:raise RuntimeError('Unexpected native weighing contact buffers')
            impulse,point,normal,separation,count,start,ids=values
            if count.size!=1 or start.size!=1:raise RuntimeError('Unexpected native weighing contact ranges')
            n,first=int(count.reshape(-1)[0]),int(start.reshape(-1)[0])
            if n<0 or n>=len(ids) or (n and (first<0 or first+n>len(ids))):
                raise RuntimeError('Invalid or saturated native weighing contact range')
            active=range(first,first+n)
            unknown=sorted({int(ids[i]) for i in active}-actors.keys())
            paths=support.get_other_actor_paths_from_ids(wp.array(unknown,dtype=wp.uint64,device='cpu')) if unknown else []
            if len(paths)!=len(unknown):raise RuntimeError('Native support counterpart unresolved')
            for identity,path in zip(unknown,paths):
                path=str(path)
                if (not stage.GetPrimAtPath(path).IsValid()
                        or not any(path==root or path.startswith(root+'/') for root in support_roots)):
                    raise RuntimeError(f'Unexpected native weighing contact actor: {path}')
                actors[identity]=path
            bindings['actual_support_actor_paths']=sorted(set(actors.values()))
            force=np.zeros(3);rows=[]
            for i in active:
                if not np.isfinite(np.r_[impulse[i],point[i],normal[i],separation[i]]).all():
                    raise RuntimeError('Nonfinite native support contact')
                if abs(float(normal[i,2]))<=.99:raise RuntimeError('Native weighing support contact is not horizontal')
                force[2]+=abs(float(impulse[i,0]*normal[i,2]))*hz
                rows.append(dict(other=actors[int(ids[i])],impulse_Ns=float(impulse[i,0]),
                    point_m=point[i].tolist(),normal=normal[i].tolist(),separation_m=float(separation[i,0])))
            net=support.get_net_contact_forces(dt=1/hz)
            if hasattr(net,'numpy'):net=net.numpy()
            net=np.asarray(net)
            if net.shape!=(1,3) or not np.isfinite(net).all():
                raise RuntimeError('Invalid native support force readback')
            return force,net[0].copy(),rows
        with (args.output/'fluid-trace.jsonl').open('w') as trace, (args.output/'support-trace.jsonl').open('w') as weights:
            def sample_fluid(phase,step,physics_step):
                nonlocal previous_points,previous_pose,escape_saved,moved,peak_clear_horizontal_speed
                pose=glass_pose()
                points=fluid.positions()
                common=min(len(points),len(previous_points))
                if common:moved |= bool(np.any(np.linalg.norm(points[:common]-previous_points[:common],axis=1)>1e-6))
                if has_initial:moved |= bool(np.any(np.linalg.norm(points[:len(initial_points)]-initial_points,axis=1)>1e-6))
                matrix=np.asarray(pose['usd_world_matrix'])
                old_spills=np.pad(scorer.spilled,(0,len(points)-len(scorer.spilled)))
                metrics=scorer.update(points,matrix[3,:3],matrix[:3,:3].T)
                escaped=np.flatnonzero(scorer.spilled & ~old_spills)
                if len(escaped) and not escape_saved:
                    np.savez_compressed(args.output/'first-escape.npz',
                        current_positions_m=points,previous_positions_m=previous_points,
                        current_ids=np.arange(len(points)),previous_ids=np.arange(len(previous_points)),
                        escaped_ids=escaped,
                        current_glass_native_xyz_quat_xyzw=np.asarray(pose['native_xyz_quat_xyzw']),
                        previous_glass_native_xyz_quat_xyzw=np.asarray(previous_pose['native_xyz_quat_xyzw']),
                        current_glass_usd_world_matrix=matrix,
                        previous_glass_usd_world_matrix=np.asarray(previous_pose['usd_world_matrix']),
                        physics_step=physics_step,physics_hz=hz)
                    receipt['first_escape']=dict(path='first-escape.npz',phase=phase,step=step,
                        physics_step=physics_step,escaped_ids=escaped.tolist())
                    escape_saved=True
                local=points[:common]-np.asarray(pose['native_xyz_quat_xyzw'][:3])
                clear=(np.linalg.norm(local[:,:2],axis=1)<score_module.INNER_RADIUS-.003)&(local[:,2]>score_module.BASE+.003)
                speed=np.linalg.norm((points[:common,:2]-previous_points[:common,:2])*hz,axis=1)
                clear_speed=float(speed[clear].max()) if np.any(clear) else 0.
                peak_clear_horizontal_speed=max(peak_clear_horizontal_speed,clear_speed)
                bulk_ml=None
                core=(np.linalg.norm(local[:,:2],axis=1)>.010)&(np.linalg.norm(local[:,:2],axis=1)<.030)&(local[:,2]>=score_module.BASE)&(local[:,2]<=score_module.RIM)
                if np.count_nonzero(core)>20:
                    surface=float(np.quantile(local[core,2],.99)+score_module.REST_OFFSET)
                    bulk_ml=max(0.,surface-score_module.BASE)*math.pi*score_module.INNER_RADIUS**2*1e6
                    rms=float(np.sqrt(np.mean(np.sum(((points[:common]-previous_points[:common])*hz)**2,axis=1))))
                    bulk_window.append((bulk_ml,rms));del bulk_window[:-hz]
                sim_points=PhysxSchema.PhysxParticleSetAPI(fluid.points.GetPrim()).GetSimulationPointsAttr().Get()
                trace.write(json.dumps(dict(phase=phase,step=step,physics_step=physics_step,
                    first_particle=points[0].tolist() if len(points) else None,
                    max_horizontal_speed_clear_of_glass_m_s=clear_speed,bulk_volume_estimate_ml=bulk_ml,
                    first_simulation_point=list(sim_points[0]) if sim_points is not None and len(sim_points) else None,**metrics))+'\n')
                previous_points=points.copy();previous_pose=pose
                if args.stop_on_spill and metrics['spill_latched']:
                    receipt['failed_prefix_metrics']=metrics
                    receipt['failed_prefix_physics_step']=physics_step
                    raise RuntimeError('Native diagnostic stopped after irreversible spill; requested endpoint not completed')
                return points,metrics

            # Discard half a second of settling, then measure one second of tare.
            for step in range(baseline_steps):
                sim.step(render=False)
                force,net,contacts=support_force()
                phase='prefill_initial_settle' if has_initial else 'empty'
                weights.write(json.dumps(dict(phase=phase,step=step+1,force_N=force.tolist(),
                    net_force_N=net.tolist(),contacts=contacts))+'\n')
                if has_initial:points,metrics=sample_fluid(phase,step+1,step+1)
                elif step>=hz//2:tare_forces.append(force)
            receipt['glass_pose_tare']=glass_pose()
            previous_pose=receipt['glass_pose_tare']
            for step in range(steps):
                if not args.prefill:
                    opening=args.flow_ml_s*min(1.,max(0.,(pour_s-step/hz)*hz))
                    fluid.emit_volume(opening/hz,1/hz)
                sim.step(render=False)
                force,net,contacts=support_force()
                phase='prefill_final_settle' if args.prefill else 'pour_and_settle'
                weights.write(json.dumps(dict(phase=phase,step=step+1,force_N=force.tolist(),
                    net_force_N=net.tolist(),contacts=contacts))+'\n')
                if step>=steps-hz:loaded_forces.append(force)
                points,metrics=sample_fluid(phase,step+1,baseline_steps+step+1)
                if step in (math.ceil(hz/12)-1,math.ceil(pour_s*hz)-1,steps-1):statistics(step+1)
        receipt['glass_pose_final']=glass_pose()
        stage.GetRootLayer().Export(str(args.output/'final-stage.usda'))
        tare=np.asarray(tare_forces);loaded=np.asarray(loaded_forces)
        if has_initial:
            native_mass=glass_body.get_masses()
            if hasattr(native_mass,'numpy'):native_mass=native_mass.numpy()
            native_mass=np.asarray(native_mass).reshape(-1)
            if len(native_mass)!=1 or not np.isfinite(native_mass).all():
                raise RuntimeError('Invalid native dry glass mass property')
            empty_mass=float(native_mass[0])
        else:empty_mass=float(tare[:,2].mean()/9.81)
        loaded_mass=float(loaded[:,2].mean()/9.81)
        if empty_mass<=0:raise RuntimeError('Native support reported no positive empty-glass weight')
        receipt['authored_particle_sets_final']=[dict(path=str(p.GetPath()),
            mass_kg=UsdPhysics.MassAPI(p).GetMassAttr().Get(),
            density=UsdPhysics.MassAPI(p).GetDensityAttr().Get(),
            particles=len(UsdGeom.Points(p).GetPointsAttr().Get()))
            for p in stage.Traverse() if p.IsA(UsdGeom.Points)
            and p.GetPath().HasPrefix(root.GetPath())]
        receipt['particle_count_final']=fluid.count
        receipt['inlet_pending_ml']=fluid._remainder*fluid_module.PARTICLE_ML
        receipt['inlet_accounted_ml']=(fluid.count+fluid._remainder)*fluid_module.PARTICLE_ML
        if len(points)>100:
            local=points-np.asarray(receipt['glass_pose_final']['native_xyz_quat_xyzw'][:3])
            radial=np.linalg.norm(local[:,:2],axis=1)
            core=(radial>.010)&(radial<.030)&(local[:,2]>=score_module.BASE)&(local[:,2]<=score_module.RIM)
            if np.count_nonzero(core)>20:
                surface=float(np.quantile(local[core,2],.99)+score_module.REST_OFFSET)
                receipt['bulk_volume_estimate']=dict(method='Flat surface inferred from core-particle q99 plus particle radius; approximate, not a mass measurement',
                    surface_height_local_m=surface,
                    cylinder_volume_ml=max(0.,surface-score_module.BASE)*math.pi*score_module.INNER_RADIUS**2*1e6,
                    retained_particle_volume_ml=metrics['retained_ml'])
        if bulk_window:
            values=np.asarray(bulk_window)
            receipt['bulk_last_second']=dict(samples=len(values),mean_ml=float(values[:,0].mean()),
                std_ml=float(values[:,0].std()),first_ml=float(values[0,0]),last_ml=float(values[-1,0]),
                mean_particle_rms_velocity_m_s=float(values[:,1].mean()))
        receipt['weighing']=dict(support_path=support_path,baseline_steps=baseline_steps,mean_window_steps=hz,
            tare_basis='native_mass_property' if has_initial else 'measured_empty_support',
            empty_force_mean_N=None if has_initial else tare.mean(axis=0).tolist(),
            empty_force_std_N=None if has_initial else tare.std(axis=0).tolist(),
            loaded_force_mean_N=loaded.mean(axis=0).tolist(),loaded_force_std_N=loaded.std(axis=0).tolist(),
            empty_mass_kg=empty_mass,loaded_mass_kg=loaded_mass,added_mass_kg=loaded_mass-empty_mass,
            nominal_liquid_mass_kg=(args.initial_ml+args.pour_ml)*1e-6*fluid_module.DENSITY_KG_M3*args.mass_scale,
            emitted_particle_mass_kg=fluid.count*expected_particle_mass,
            retained_liquid_mass_kg=metrics['retained_ml']*1e-6*fluid_module.DENSITY_KG_M3*args.mass_scale)
        receipt.update(status='completed',steps=steps,actual_device=str(sim.device),
                       native_readback_moved=moved,final_metrics=metrics,
                       peak_horizontal_speed_clear_of_glass_m_s=peak_clear_horizontal_speed)
        if not moved:raise RuntimeError('Native particle readback did not move')
        np.save(args.output/'final-particles.npy',points,allow_pickle=False)
        sim.stop()
    except Exception as error:
        import traceback
        receipt.update(status='error',error=traceback.format_exc())
        if 'stage' in locals():
            try:
                if 'glass_pose' in locals():receipt['glass_pose_error']=glass_pose()
                stage.GetRootLayer().Export(str(args.output/'error-stage.usda'))
            except Exception as export_error:receipt['error_stage_export_error']=str(export_error)
        raise
    finally:
        pvd_error=None
        if pvd_previous is not None:
            try:
                if 'sim' in locals():sim.stop()
                from omni.physx import get_physx_simulation_interface
                native=get_physx_simulation_interface()
                if native.get_attached_stage():native.detach_stage()
                # NVIDIA's PVD test detaches the stage to close and serialize OVD.
                settings.set(pb.SETTING_OMNIPVD_ENABLED,False)
                files=[p for p in pvd_dir.glob('*.ovd') if not p.name.endswith('tmp.ovd') and p.stat().st_size]
                receipt['pvd'].update(status='recorded',files=[dict(path=str(p.relative_to(args.output)),
                    bytes=p.stat().st_size) for p in sorted(files)])
                if not files:raise RuntimeError('Requested native PVD recording produced no completed, nonempty OVD file')
            except Exception as error:
                receipt['pvd'].update(status='error',error=str(error))
                if receipt['status']=='completed':
                    receipt.update(status='error',error=str(error))
                    pvd_error=error
            finally:
                for key,value in pvd_previous.items():
                    if value is None:settings.destroy_item(key)
                    else:settings.set(key,value)
        receipt['wall_s']=time.monotonic()-started
        (args.output/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
        if app:app.close()
        if pvd_error:raise pvd_error


if __name__=='__main__':main()
