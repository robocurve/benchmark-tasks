"""Bounded native YAM panel-paper preview; no task success claim."""
import argparse
import gzip
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import time
import traceback
import signal
import shutil
import numpy as np


def native_audit(stage,quasistatic=False):
    from pxr import UsdPhysics
    from arena_hardware import BODY_NAMES
    from arena_panels import SHELLS
    attachments=[];fixed=[];hardware=[];papers={}
    for prim in stage.Traverse():
        path=str(prim.GetPath());schemas=prim.GetAppliedSchemas()
        if any('Attachment' in x for x in schemas) or 'Attachment' in prim.GetTypeName():attachments.append(path)
        if '/hardware/' in path and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            if UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Get() and not (quasistatic and '/staple/' in path):raise RuntimeError('Unexpected kinematic hardware')
            hardware.append(path)
        if '/hardware/staple/' in path and prim.IsA(UsdPhysics.Joint):
            if not prim.GetRelationship('physics:body0').GetTargets() or not prim.GetRelationship('physics:body1').GetTargets():fixed.append(path)
    for name,shell in SHELLS.items():
        for path in shell['bodies']:
            prim=stage.GetPrimAtPath(path);shape=stage.GetPrimAtPath(path+'/Shape')
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI) or UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Get():raise RuntimeError('Paper panel must be free/dynamic')
            if shape.GetTypeName()!='Cube' or abs(shape.GetAttribute('xformOp:scale').Get()[2]-shell['material'].thickness_m)>1e-10:raise RuntimeError('Paper thickness does not match its declared material')
        for path in shell['joints']:
            prim=stage.GetPrimAtPath(path)
            if not prim.GetRelationship('physics:body0').GetTargets() or not prim.GetRelationship('physics:body1').GetTargets():fixed.append(path)
        papers[name]={'dynamic_panels':len(shell['bodies']),'passive_hinges':len(shell['joints']),
                      'surface_area_m2':shell['mesh'].audit()['surface_area_m2']}
    if len(papers)!=2 or len(hardware)!=len(BODY_NAMES) or attachments or fixed:raise RuntimeError('Incomplete native paper or hidden attachment')
    return dict(paper=papers,hardware_bodies=len(hardware),quasistatic_wire=quasistatic,attachments=attachments,fixed_material_joints=fixed)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--steps',type=int,default=10)
    p.add_argument('--physics-hz',type=int,choices=(240,480,2000,5000,10000),default=10000)
    p.add_argument('--quasistatic',action='store_true',help='Contact-driven clinch bending approximation, identical for demo and Dream')
    p.add_argument('--paper-profile',choices=('office-80gsm','heavy-200gsm-candidate'),default='office-80gsm')
    p.add_argument('--panel-spacing-m',type=float,choices=(.012,.04),default=.012)
    p.add_argument('--plan',type=Path)
    p.add_argument('--dream',action='store_true',help='Ordinary Dream policy; mutually exclusive with scripted feedback')
    p.add_argument('--dream-rows-per-request',type=int,choices=(2,4,8),default=8)
    p.add_argument('--judging-views',action='store_true',help='Fixed close review cameras, never policy inputs')
    p.add_argument('--joint-tracking',action='store_true',help='Bounded ordinary-joint integral tracking, without object feedback')
    p.add_argument('--joint-velocity-feedforward-from',type=int,help='From this scripted action, compensate native drive damping through the same bounded ordinary joint targets')
    p.add_argument('--tool-pose-servo',action='store_true',help='Disclosed scripted remover-pose feedback through ordinary joint targets; never Dream')
    p.add_argument('--tool-velocity-damping',action='store_true',help='Bounded ordinary position feedback opposes measured tool velocity during contact; no force or state writes')
    p.add_argument('--position-iterations',type=int,choices=(32,64,128,255),default=128)
    p.add_argument('--paper-gap-envelope-um',type=int,choices=(25,100),default=25,
        help='100 is an explicitly unscored continuity diagnostic; it does not change contact or forces')
    p.add_argument('--geometry-aware-continuity',action='store_true',
        help='Unscored150um empty-aperture diagnostic; preserve raw endpoint errors and native fracture events')
    p.add_argument('--minimum-panel-mass-mg',type=int,choices=(0,5),default=0,
        help='Explicit numerical mass floor for tiny paper panels; geometry, stiffness, strength and crossing checks remain unchanged')
    p.add_argument('--paper-aperture-um',type=int,choices=(150,250),default=150,
        help='Explicit numerical panel-gap bound.250um remains below the300um minimum steel-wire thickness; independent swept crossing and fracture checks remain active')
    p.add_argument('--solver-type',choices=('TGS','PGS'),default='TGS')
    p.add_argument('--recenter-at-paper',action='store_true',help='Fixed global origin shift including robot bases and fixed cameras before construction')
    p.add_argument('--reset-profile',choices=('pocket-support-candidate',),default='pocket-support-candidate')
    p.add_argument('--mesh-profile',choices=('clearance-candidate',),default='clearance-candidate')
    p.add_argument('--refined-contact-diagnostic',action='store_true')
    p.add_argument('--max-depenetration-m-s',type=float)
    a=p.parse_args()
    if a.dream and (a.plan or a.joint_tracking or a.tool_pose_servo or a.paper_gap_envelope_um!=25):p.error('Dream cannot use scripted feedback or a diagnostic endpoint envelope')
    if a.paper_aperture_um!=150 and not a.geometry_aware_continuity:p.error('Explicit aperture bound requires direct geometric continuity')
    if a.joint_tracking and not a.plan:p.error('Joint tracking requires a plan')
    if a.joint_velocity_feedforward_from is not None and (not a.joint_tracking or not 1<=a.joint_velocity_feedforward_from<=a.steps):p.error('Feedforward needs a bounded scripted joint-tracking start')
    if a.tool_pose_servo and (not a.plan or not a.joint_tracking):p.error('Tool-pose servo requires a plan and bounded joint tracking')
    if a.tool_velocity_damping and not a.tool_pose_servo:p.error('Tool velocity damping requires scripted pose feedback')
    if not 1<=a.steps<=(600 if a.plan or a.dream else 100):p.error('Bounded preview step count exceeded')
    import runtime_config
    import coordinate_frame
    coordinate_frame.configure(a.recenter_at_paper)
    runtime_config.configure(a.reset_profile,a.mesh_profile,a.refined_contact_diagnostic,a.max_depenetration_m_s)
    runtime_config.PAPER_PROFILE=a.paper_profile
    runtime_config.PANEL_SPACING_M=a.panel_spacing_m
    plan=actions=None
    if a.plan:
        from motion_plan import load
        from yam_arena import YAM_URDF
        plan,actions=load(a.plan)
        if a.steps>len(actions):p.error('Requested steps exceed plan')
        if plan['reset_profile']!=a.reset_profile or plan['mesh_profile']!=a.mesh_profile:p.error('Plan and native reset must match')
        if plan.get('paper_profile','office-80gsm')!=a.paper_profile:p.error('Plan and physical paper grade must match')
        if plan.get('panel_spacing_m',.012)!=a.panel_spacing_m:p.error('Plan and panel spacing must match')
        if hashlib.sha256(Path(YAM_URDF).read_bytes()).hexdigest()!=plan['urdf_sha256']:raise RuntimeError('Plan URDF mismatch')
        for name,digest in plan['source_sha256'].items():
            if a.quasistatic:continue # New material variant is fully snapshotted below, not represented as the old physics.
            if hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()!=digest:raise RuntimeError('Stale plan source: '+name)
    if not version('isaacsim').startswith('6.1.'):raise RuntimeError('Use the pinned Isaac6.1 environment')
    a.out.mkdir(parents=True,exist_ok=False);started=time.monotonic()
    frozen=a.out/'task-source';frozen.mkdir()
    for source in Path(__file__).parent.glob('*.py'):shutil.copyfile(source,frozen/source.name)
    if a.plan:shutil.copyfile(a.plan,frozen/'fixed-plan.json')
    report={'kind':'staple_panel_yam_integration_preview','status':'running','score':None,'robot_demo':False,
        'joint_tracking':a.joint_tracking,'solver_type':a.solver_type,'position_iterations':a.position_iterations,
        'tool_pose_servo':a.tool_pose_servo,
        'diagnostic_only':a.paper_gap_envelope_um!=25 or a.geometry_aware_continuity,
        'paper_gap_envelope_m':a.paper_gap_envelope_um*1e-6,
        'geometry_aware_continuity':a.geometry_aware_continuity,
        'empty_aperture_limit_m':a.paper_aperture_um*1e-6 if a.geometry_aware_continuity else None,
        'minimum_panel_mass_mg':a.minimum_panel_mass_mg,
        'dream_requests':0,'requested_steps':a.steps,'control_hz':20,'phase':'launch',
        'policy':('Dream with ordinary camera/joint observations' if a.dream else
                  'State-assisted scripted tool-pose servo; ordinary bounded joint commands' if a.tool_pose_servo else
                  'ordinary fixed joint commands; no object state feedback'),
        'paper_model':'native analytic box panels with passive bending/strength hinges',
        'quasistatic_wire':a.quasistatic,
        'paper_profile':a.paper_profile,'panel_spacing_m':a.panel_spacing_m,
        'world_coordinate_offset_m':coordinate_frame.OFFSET_M.tolist(),
        'plan_sha256':hashlib.sha256(a.plan.read_bytes()).hexdigest() if a.plan else None,
        'reset_profile':a.reset_profile,'mesh_profile':a.mesh_profile,
        'refined_contact':a.refined_contact_diagnostic,'max_depenetration_m_s':a.max_depenetration_m_s,
        'source_sha256':{f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in Path(__file__).parent.glob('*.py')}}
    def checkpoint():
        report['wall_seconds']=time.monotonic()-started
        temporary=a.out/'preview.tmp.json';temporary.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');temporary.replace(a.out/'preview.json')
    def terminated(signum,frame):raise TimeoutError('Preview stopped by external watchdog')
    signal.signal(signal.SIGTERM,terminated);checkpoint()
    app=video=trace=dream_policy=break_observer=wire_model=None;monitors=[];request_audit=[];session_audit={'session_count':0}
    try:
        from isaaclab.app import AppLauncher
        app=AppLauncher(headless=True,enable_cameras=True,device='cpu').app
        from joint_break_observer import JointBreakObserver
        break_observer=JointBreakObserver()
        import imageio.v2 as imageio
        import torch
        from PIL import Image
        import omni.usd
        import arena_panels,arena_hardware
        arena_hardware.FUNCTIONAL_MODEL=a.quasistatic
        arena_hardware.PANEL_CONTACT_CANDIDATE=True
        arena_hardware.PANEL_POSITION_ITERATIONS=a.position_iterations
        arena_panels.POSITION_ITERATIONS=a.position_iterations
        arena_panels.MAXIMUM_HINGE_GAP_M=a.paper_gap_envelope_um*1e-6
        arena_panels.GEOMETRIC_APERTURE_LIMIT_M=a.paper_aperture_um*1e-6 if a.geometry_aware_continuity else None
        arena_panels.MINIMUM_PANEL_MASS_KG=a.minimum_panel_mass_mg*1e-6
        from yam_arena import inspect_embodiment as wrapper,scene as adapter
        wrapper._APP=app;report['phase']='build'
        emb=wrapper.YamArenaEmbodiment(task=str(Path(__file__).with_name('task_panels.py')),object_jitter=0.,control_hz=20,cam_width=224,cam_height=224,device='cpu')
        original_dt=adapter.PHYSICS_DT;adapter.PHYSICS_DT=1/a.physics_hz
        original_configuration=adapter.env_cfg_callback
        def configure_callback(*args,**kwargs):
            configure=original_configuration(*args,**kwargs)
            def override(cfg):
                cfg=coordinate_frame.shift_scene_config(configure(cfg))
                from isaaclab_physx.physics import PhysxCfg
                if cfg.sim.physics is None:cfg.sim.physics=PhysxCfg()
                cfg.sim.physics.solver_type=1 if a.solver_type=='TGS' else 0
                cfg.sim.physics.enable_ccd=True
                cfg.sim.physics.enable_external_forces_every_iteration=a.solver_type=='TGS'
                return cfg
            return override
        adapter.env_cfg_callback=configure_callback
        original_make_env=adapter.make_env
        if a.judging_views:
            from review_views import configure
            def make_env(embodiment,**kwargs):
                configure(embodiment)
                return original_make_env(embodiment,**kwargs)
            adapter.make_env=make_env
        try:emb._ensure_sim(None)
        finally:
            adapter.PHYSICS_DT=original_dt
            adapter.env_cfg_callback=original_configuration
            adapter.make_env=original_make_env
        report['coordinate_configuration_audit']=list(coordinate_frame.CONFIG_AUDIT)
        base=emb._base;stage=omni.usd.get_context().get_stage()
        report['phase']='reset';break_observer.context('reset');emb._env.reset()
        report['native_audit']=native_audit(stage,a.quasistatic)
        if a.quasistatic:
            from functional_staple import install
            wire_model=install(base,next(iter(arena_hardware.ASSEMBLIES.values()))['metadata'])
            report['wire_fidelity']='Causal fork engagement and quasistatic plastic unbending; kinematic steel segments, no bending-force feedback. Native flexible paper and remover. Sideways extraction damages paper.'
        else:
            arena_panels.install_material_hook(base,stage)
            monitors=[x['monitor'] for x in arena_hardware.ASSEMBLIES.values()]
        report['physics_dt_s']=float(base.sim.get_physics_dt())
        report['effective_device']=str(base.device)
        if report['effective_device']!='cpu':raise RuntimeError('Expected CPU native panel dynamics')
        from pxr import UsdPhysics,PhysxSchema
        scenes=[prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
        if len(scenes)!=1:raise RuntimeError('Expected one native physics scene')
        phys=PhysxSchema.PhysxSceneAPI(scenes[0])
        report['native_physics']={'solver_type':str(phys.GetSolverTypeAttr().Get()),
            'gpu':bool(phys.GetEnableGPUDynamicsAttr().Get()),'ccd':bool(phys.GetEnableCCDAttr().Get()),
            'external_forces_every_iteration':bool(phys.GetEnableExternalForcesEveryIterationAttr().Get())}
        if report['native_physics']!={'solver_type':a.solver_type,'gpu':False,'ccd':True,'external_forces_every_iteration':a.solver_type=='TGS'}:
            raise RuntimeError('Native settings differ from the passing panel-paper control')
        report['wire_segment_lengths_m']=next(iter(arena_hardware.ASSEMBLIES.values()))['metadata']['staple']['lengths_m']
        report['judging_schema']='staple-judging-v4'
        stage.GetRootLayer().Export(str(a.out/'initial.usda'))
        trace=gzip.open(a.out/'trace.jsonl.gz','wt')
        video=imageio.get_writer(a.out/'preview.mp4',fps=20,codec='libx264',pixelformat='yuv420p',ffmpeg_params=['-movflags','+faststart'],macro_block_size=1)
        report['phase']='record'
        report['control_step_wall_seconds']=[]
        tracker=tool_servo=None
        observation=emb._observation()
        issued=emb._ready.detach().cpu().numpy()[0].copy()
        from yam_arena.constants import SHOULDER,WRIST,GRIPPER,FINGER_OPEN
        limits=np.tile(np.array([SHOULDER['velocity']/20]*3+[WRIST['velocity']/20]*3+[GRIPPER['velocity']/abs(FINGER_OPEN)/20]),2)
        if a.dream:
            from inspect_robots import Scene
            from dream_policy import make_policy
            instruction=json.loads(Path(__file__).with_name('task.json').read_text())['instruction']
            emb._instruction=instruction;observation=emb._observation()
            dream_policy=make_policy(request_audit,session_audit,rows_per_request=a.dream_rows_per_request)
            dream_policy.bind(emb.info);dream_policy.reset(Scene(id='remove-a-staple',instruction=instruction))
            report['dream_model']=dream_policy.info.checkpoint
            report['dream_rows_per_request']=a.dream_rows_per_request
        if a.joint_tracking:
            from joint_tracking import JointTracker
            tracker=JointTracker(issued,emb._low,emb._high,limits)
            report['tracking_disclosure']='Only robot joint observations; no native object poses, attachments, forces or actuator-effort changes.'
        if a.tool_pose_servo:
            from tool_pose_servo import RemoverPoseServo
            tool_servo=RemoverPoseServo(plan,velocity_damping_s=.1 if a.tool_velocity_damping else 0.)
            report['tool_servo_disclosure']='The scripted demo reads native remover pose and observed robot joints to compensate grasp slip after pickup. It issues only ordinary bounded joint targets; no object pose, force, material, attachment or effort-limit writes. Dream cannot use this controller.'
        hardware=arena_hardware.hardware_state(emb._env,'hardware')
        for step in range(a.steps+1):
            break_observer.context('episode',step)
            requested=None
            if step:
                if dream_policy is not None:
                    from dream_policy import bounded_action
                    chunk=dream_policy.act(observation)
                    if len(chunk.actions)!=1:raise RuntimeError('Expected one ordinary Dream control action')
                    requested=np.asarray(chunk.actions[0].data,float)
                    issued=bounded_action(requested,issued,emb._low,emb._high,limits)
                else:
                    nominal=emb._ready.detach().cpu().numpy()[0] if actions is None else actions[step-1]
                    target=nominal
                    if tool_servo is not None:
                        tool_pose=hardware.body_poses[list(hardware.body_names).index('remover')]
                        tool_velocity=hardware.body_velocities[list(hardware.body_names).index('remover')]
                        target=tool_servo.command(step,nominal,np.asarray(observation.state['joint_pos']),tool_pose,tool_velocity)
                    if tracker is not None and a.joint_velocity_feedforward_from is not None and step>=a.joint_velocity_feedforward_from:
                        tracker.damping_over_stiffness_s[:]=np.tile(
                            [SHOULDER['damping']/SHOULDER['stiffness']]*3+
                            [WRIST['damping']/WRIST['stiffness']]*3+[0.],2)
                    issued=target if tracker is None else tracker.command(target,np.asarray(observation.state['joint_pos']))
                action=torch.as_tensor(issued[None],device=base.device,dtype=torch.float32)
                tick=time.perf_counter();emb._env.step(action)
                report['control_step_wall_seconds'].append(time.perf_counter()-tick)
                emb._t=step/20
            # Observe every final substep too; the material callback inspects
            # preceding states before all intermediate native substeps.
            for monitor in monitors:monitor.inspect(update_material=False)
            observation=emb._observation()
            paper=[arena_panels.state(emb._env,'paper_'+str(i)) for i in range(2)]
            hardware=arena_hardware.hardware_state(emb._env,'hardware')
            trace.write(json.dumps({'step':step,'time_s':step/20,'joint_pos':np.asarray(observation.state['joint_pos']).tolist(),
                'quasistatic_wire':None if wire_model is None else wire_model.record(),
                'commanded_action14':None if step==0 else issued.tolist(),
                'dream_requested_action14':None if requested is None else requested.tolist(),
                'joint_tracking':None if tracker is None else tracker.last,
                'tool_pose_servo':None if tool_servo is None else tool_servo.last,
                'native_joint_break_events':break_observer.summary(),
                'paper_nodes':[x.nodes.tolist() for x in paper],
                # Preserve exact native solids, not only averaged shared nodes,
                # so successful trials can receive the same independent contact
                # audit as a saved first-invalid snapshot. These poses are
                # already read by state(); recording adds no physics operation.
                'paper_facet_poses':[x.facet_poses.tolist() for x in paper],
                'paper_facet_velocities':[x.facet_velocities.tolist() for x in paper],
                'paper_continuity':[x.continuity for x in paper],'hardware_names':list(hardware.body_names),
                'hardware_poses':hardware.body_poses.tolist(),'hardware_velocities':hardware.body_velocities.tolist(),'material':[m.last for m in monitors]})+'\n')
            rgb=base.scene['top_cam'].data.output['rgb'];rgb=getattr(rgb,'torch',rgb)
            frame=rgb.detach().cpu().numpy()[0,:,:,:3].astype(np.uint8)
            if a.judging_views:
                from review_views import composite_frame
                frame=composite_frame(base)
            video.append_data(frame)
            if step in (0,a.steps):Image.fromarray(frame).save(a.out/('setup.png' if not step else 'final.png'))
            report['completed_steps']=step
            if wire_model is not None:report['quasistatic_state']=wire_model.record()
            if step%5==0:trace.flush();checkpoint()
        if a.judging_views:
            from review_views import capture_final
            report['judging_views']=capture_final(base,a.out)
        report.update(status='complete',meaning='Unscored integration only; no successful extraction demonstrated')
    except BaseException as error:
        report.update(status='invalid_model_domain' if type(error).__name__=='MaterialDomainError' else 'failed',
            error=f'{type(error).__name__}: {error}',traceback=traceback.format_exc(),material=[m.last for m in monitors])
        for index,monitor in enumerate(monitors):
            if monitor.failure_state is not None:
                np.savez_compressed(a.out/f'first-invalid-state-{index}.npz',**monitor.failure_state)
        raise
    finally:
        if trace:trace.close()
        if video:video.close()
        if dream_policy is not None:
            from dream_policy import public_transcript
            (a.out/'dream-transcript.json').write_text(json.dumps(public_transcript(dream_policy),indent=2)+'\n')
        report.update(dream_requests=len(request_audit),dream_session_audit=session_audit)
        if break_observer is not None:
            report['native_joint_break_events']=break_observer.summary()
            break_observer.close()
        if request_audit:(a.out/'dream-requests.json').write_text(json.dumps(request_audit,indent=2)+'\n')
        report['wall_seconds']=time.monotonic()-started
        checkpoint()
        if app:app.close()


if __name__=='__main__':main()
