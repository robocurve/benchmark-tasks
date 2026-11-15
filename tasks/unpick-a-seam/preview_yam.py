"""Unscored YAM seam diagnostic with every-substep material/contact audits.

Ordinary ready targets or a frozen joint plan drive the robot. An explicitly
selected scripted-entry option compensates the measured tool grasp using only
bounded robot joint targets. Native state also feeds assessment and cutting.
"""
import argparse
from dataclasses import asdict
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import time
import traceback
import numpy as np

HERE=Path(__file__).resolve().parent


def array(value):
    return getattr(value,'torch',value).detach().cpu().numpy().copy()


def rotation(q):
    x,y,z,w=np.asarray(q,float)/np.linalg.norm(q)
    return np.array(((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),
                     (2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),
                     (2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y))))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--steps',type=int,default=20)
    p.add_argument('--plan',type=Path,help='Frozen action14 JSON produced by plan.py')
    p.add_argument('--dream',action='store_true',help='Run Dream with standard camera and joint observations only')
    p.add_argument('--dream-rows-per-request',type=int,choices=(1,2,4,8),default=4)
    p.add_argument('--pose-assisted-entry',action='store_true',help='Disclosed scripted demo: after the proven140-action pickup, filter measured tool-in-gripper pose to convert the fixed blade path into ordinary bounded joint targets')
    p.add_argument('--judging-views',action='store_true',help='Fixed native seam close-ups for recording only, excluded from policy inputs')
    p.add_argument('--physics-hz',type=int,choices=(240,480,960,1920,3840),default=960)
    p.add_argument('--coarse-thread',action='store_true')
    p.add_argument('--hole-mesh-spacing',type=float)
    p.add_argument('--resting-seam',action='store_true',help='Lower the sewn assembly so the bottom thread starts tangent to the tabletop; records the fixed layout change')
    p.add_argument('--pre-draped-seam',action='store_true',help='Fixed initial drape over the resting thread, with unchanged flat intrinsic rest shape; requires coarse thread and quality mesh')
    p.add_argument('--drape-shape',choices=('gaussian','cylindrical'),default='gaussian',help='Cylindrical bends preserve material arc length; applies only to the pre-draped reset')
    p.add_argument('--quality-mesh',action='store_true',help='Opt-in graded 0.4 mm hole-boundary triangulation; requires coarse thread and no hole-mesh-spacing')
    p.add_argument('--sewing-clamp',action='store_true',help='Vertical swatches in an ordinary fixed bench clamp; requires coarse thread and 0.4 mm hole refinement, without table-reset options')
    p.add_argument('--passive-clamp',action='store_true',help='Replace the incompatible fixed-gap jaws with the declared force-limited passive spring jaw; requires --sewing-clamp')
    p.add_argument('--supported-clamp',action='store_true',help='Lower the fixture so the lower patch edge rests on the real tabletop; requires --passive-clamp')
    p.add_argument('--facet-cloth',action='store_true',help='Opt-in inextensible native rigid-facet shell with passive bending hinges; requires backed clamp and once-per-step forces; default remains FEM')
    p.add_argument('--box-panels',action='store_true',help='Use native rectangular boxes for the facet candidate, avoiding failed convex hull cooking')
    p.add_argument('--geometric-panel-audit',action='store_true',help='Diagnostic: stop on 0.20 mm empty shared-edge aperture or lost overlap; retain 0.10 mm Euclidean residual telemetry and every thread, strain and blade guard')
    p.add_argument('--panel-passage-diagnostic',action='store_true',help='Unscored native geometry diagnostic: retain 0.20 mm aperture warning, stop at 0.40 mm aperture (half thread diameter), audit actual box midplanes every substep; no change to physics, strain or swept crossing checks')
    p.add_argument('--cpu-ccd',action='store_true',help='Opt-in CPU native panel/thread candidate with sweep CCD; renderer remains on GPU')
    p.add_argument('--solver',choices=('TGS','PGS'),default='TGS')
    p.add_argument('--recenter-at-seam',action='store_true',help='Preconstruction translation of the whole world, including robot bases and fixed cameras')
    p.add_argument('--clear-thread-sampling',action='store_true',help='Use the same centreline with24 capsules per loop')
    p.add_argument('--thread-long-side-links',type=int,choices=(4,10),default=10,
        help='Explicit12-link or24-link closed-thread discretization; radius, centreline and density unchanged')
    p.add_argument('--thread-inertia-scale',type=float,choices=(1.,10.),default=1.,
        help='Explicit numerical mass approximation:10x makes each loop0.175g; geometry/contact/cutting unchanged, gravity scales too')
    p.add_argument('--passive-thread-bending',action='store_true',help='Disclosed passive straight-rest D6 thread bending; no task-aware forces')
    p.add_argument('--sharp-edge-cut',action='store_true',help='Explicit sharp-edge approximation: measured inward contact force of50mN for10ms severs the contacted thread joint; no cloth/contact guard changes')
    p.add_argument('--rigid-panel-strain-telemetry',action='store_true',help='Use actual panel apertures and contact damage as the domain checks; averaged render-node strain is telemetry because rigid panels cannot stretch')
    p.add_argument('--boundary-contact-envelope',action='store_true',help='Explicit0.1mm numerical tolerance at actual outer/needle-hole boundaries only; deeper and interior crossings remain invalid')
    p.add_argument('--convex-capsules-diagnostic',action='store_true',help='Matched scaled convex thread/guard colliders; preserves dimensions and mass, avoids native tiny-capsule contact fallback')
    p.add_argument('--external-forces-once',action='store_true',help='Configure TGS external forces once per physics step before scene construction; measured wheel control motivates this diagnostic')
    p.add_argument('--velocity-iterations',type=int,choices=(1,32),default=None,
                   help='Explicit scene-level velocity iteration count, including robot articulations; preserves ordinary joint/action limits')
    p.add_argument('--backed-clamp',action='store_true',help='Add the physical rear sewing board and front guide rails outside the open seam window; requires --supported-clamp')
    p.add_argument('--keep-constructor-state',action='store_true',help='Unsupported historical diagnostic; rejected before simulator startup because Gym requires an episode reset')
    p.add_argument('--contact-refinement',choices=('baseline','iterations','speculative','combined'),default='baseline')
    p.add_argument('--cloth-contact-offset',type=float,choices=(.00025,.00075),default=.00025,
                   help='Opt-in native contact generation distance in metres; rest offset, thickness, material and geometric guards are unchanged')
    p.add_argument('--max-depenetration-m-s',type=float,help='Bound native deformable contact-correction velocity (0.01–1 m/s); no change to object velocity or scoring limits')
    p.add_argument('--maximum-stretch',type=float,choices=(1.02,1.05),default=1.02,
                   help='Explicit engineering domain; 2%% telemetry remains recorded, neither limit is a calibrated tear law')
    p.add_argument('--crossing-backend',choices=('cpu','warp-shadow','warp'),default='cpu')
    p.add_argument('--audit-junctions',action='store_true',help='Also audit the closed native-joint connectors and disjoint end caps; force-qualified severed joints are excluded')
    p.add_argument('--profile-steps',action='store_true',help='CPU call profile around the bounded control steps; does not alter physics or audit cadence')
    p.add_argument('--continue-after-strain',action='store_true',
                   help='Failure analysis only: retain invalid status but observe ready-hold settling for at most 0.2 s; crossings and blade damage still stop the run')
    p.add_argument('--contact-forensics',action='store_true',
                   help='Unscored ready hold only, at most 1 s: record every-step crossing depth while retaining invalidity; no cutting, and strain, blade damage or disjoint joints still stop the run')
    p.add_argument('--loose-tacking',action='store_true',help='Declared loose basting loops: front bridge3.8mm above the fabric midplane')
    p.add_argument('--practical-material',action='store_true',help='Disclosed coarse panel/contact model:2mm cloth continuity bound, physical blade damage, native hinge breaks, and1mm interior thread-crossing bound; numerical contacts remain logged')
    p.add_argument('--functional-tacks',action='store_true',help='Quasistatic closed tacks with native fabric connectors, released only by blade contact')
    a=p.parse_args()
    if a.dream and (a.plan is not None or a.pose_assisted_entry):p.error('Dream cannot use scripted actions or tool-pose feedback')
    import os
    os.environ['ROBOCURVE_SEAM_FUNCTIONAL_TACKS']='1' if a.functional_tacks else '0'
    if a.functional_tacks and not a.practical_material:p.error('Functional tacks require practical native panel material')
    os.environ['ROBOCURVE_SEAM_LOOSE_TACKING']='1' if a.loose_tacking else '0'
    os.environ['ROBOCURVE_SEAM_PRACTICAL_MATERIAL']='1' if a.practical_material else '0'
    os.environ['ROBOCURVE_SEAM_CONVEX_CAPSULE_DIAGNOSTIC']='1' if a.convex_capsules_diagnostic else '0'
    if a.practical_material and not (a.box_panels and a.cpu_ccd and a.rigid_panel_strain_telemetry):p.error('Practical material requires the native CPU panel model')
    if a.keep_constructor_state:
        p.error('Keeping constructor state is unsupported: Gym requires env.reset() before env.step(). Omit --keep-constructor-state for the normal episode lifecycle.')
    if not 1<=a.steps<=2400:p.error('Use 1 to 2400 control steps')
    if a.quality_mesh and (not a.coarse_thread or a.hole_mesh_spacing is not None):
        p.error('Quality mesh requires --coarse-thread and omitting --hole-mesh-spacing')
    if a.pre_draped_seam and not (a.coarse_thread and a.quality_mesh):
        p.error('Pre-drape requires --coarse-thread and --quality-mesh')
    if a.sewing_clamp and (not a.coarse_thread or (not a.quality_mesh and not a.facet_cloth and a.hole_mesh_spacing!=.0004) or a.resting_seam or a.pre_draped_seam):
        p.error('Sewing clamp requires --coarse-thread and either --hole-mesh-spacing .0004 or --quality-mesh, without table reset options')
    if a.passive_clamp and not a.sewing_clamp:p.error('Passive clamp requires --sewing-clamp')
    if a.supported_clamp and not a.passive_clamp:p.error('Supported reset requires --passive-clamp')
    if a.backed_clamp and not a.supported_clamp:p.error('Repair backing requires --supported-clamp')
    if a.facet_cloth and (not a.backed_clamp or not a.external_forces_once or a.quality_mesh or a.hole_mesh_spacing is not None or a.cloth_contact_offset!=.00025 or a.contact_refinement!='baseline' or a.max_depenetration_m_s is not None):
        p.error('Facet shell requires backed clamp, once-per-step forces and its own mesh; no quality-mesh/hole refinement/contact-offset override')
    if a.box_panels and not a.facet_cloth:p.error('Box panels require --facet-cloth')
    if a.geometric_panel_audit and not a.box_panels:p.error('Geometric panel audit requires native box panels')
    if a.panel_passage_diagnostic and not (a.geometric_panel_audit and a.coarse_thread and a.audit_junctions):
        p.error('Panel passage diagnostic requires geometric box-panel audit, 0.8 mm thread and junction audits')
    if a.rigid_panel_strain_telemetry and not a.panel_passage_diagnostic:
        p.error('Averaged strain is telemetry only when actual box-panel apertures, thread passage and junctions are all checked')
    if a.boundary_contact_envelope and not (a.panel_passage_diagnostic and a.crossing_backend=='cpu'):
        p.error('Boundary envelope requires all direct box-panel and CPU swept-contact audits')
    if a.cpu_ccd and (not a.box_panels or a.crossing_backend!='cpu'):
        p.error('CPU CCD requires box panels and CPU geometric audits')
    if (a.solver!='TGS' or a.recenter_at_seam) and not a.cpu_ccd:
        p.error('Solver/origin comparison requires the CPU candidate')
    if a.clear_thread_sampling and not a.coarse_thread:p.error('Clear sampling requires coarse thread')
    if a.thread_long_side_links!=10 and not a.clear_thread_sampling:p.error('Reduced thread links require clear sampling')
    if a.passive_thread_bending and not (a.cpu_ccd and a.clear_thread_sampling):
        p.error('Passive thread bending requires the clear-sampled CPU candidate')
    if a.drape_shape!='gaussian' and not a.pre_draped_seam:
        p.error('Drape shape requires --pre-draped-seam')
    if a.max_depenetration_m_s is not None and not .01<=a.max_depenetration_m_s<=1.:
        p.error('Diagnostic depenetration velocity must be 0.01 to 1 m/s')
    if a.plan is None and not a.dream and a.steps>200:p.error('Ready-hold preview limited to 200 steps')
    if a.pose_assisted_entry and (a.plan is None or a.steps<=140):
        p.error('Pose-assisted entry requires a frozen plan extending beyond the140-action pickup')
    if a.continue_after_strain and (a.plan is not None or a.steps>4):
        p.error('Strain continuation is limited to four ready-hold steps and cannot run a policy')
    if a.contact_forensics and (a.plan is not None or a.steps>20 or a.continue_after_strain):
        p.error('Contact forensics requires at most20 ready-hold steps, no plan, and no strain continuation')
    if a.contact_forensics and not a.audit_junctions:
        p.error('Contact forensics requires --audit-junctions to retain native joint-continuity checks')
    actions=None;entry_stop=None
    if a.plan is not None:
        plan=json.loads(a.plan.read_text())
        if plan.get('control_hz')!=20 or plan.get('no_object_feedback') is not True:
            raise ValueError('Expected a frozen 20 Hz joint plan without object feedback')
        if plan.get('requires_pre_draped_seam') and not a.pre_draped_seam:
            raise ValueError('This frozen cutting plan requires the declared pre-draped seam reset')
        if plan.get('requires_junction_audit') and not a.audit_junctions:
            raise ValueError('This frozen cutting plan requires the closed-junction audit')
        if bool(plan.get('requires_sewing_clamp'))!=a.sewing_clamp:
            raise ValueError('Frozen plan does not match the selected sewing-clamp layout')
        if plan.get('requires_passive_clamp') and not a.passive_clamp:
            raise ValueError('This frozen cutting plan requires the passive spring jaw')
        if bool(plan.get('requires_supported_clamp'))!=a.supported_clamp:
            raise ValueError('Frozen cutting plan does not match the supported-cloth layout')
        actions=np.asarray([row['action14'] for row in plan['frames']],float)
        if actions.ndim!=2 or actions.shape[1]!=14 or not np.isfinite(actions).all() or len(actions)<a.steps:
            raise ValueError('Incomplete/invalid fixed action sequence')
        if a.pose_assisted_entry:
            from tool_entry import assistance_stop_step
            entry_stop=assistance_stop_step(plan)
    from native_cloth import ContactRefinement
    import runtime_config
    import coordinate_frame
    coordinate_frame.configure(a.recenter_at_seam)
    refined=a.contact_refinement in ('iterations','combined')
    settings=ContactRefinement(a.contact_refinement in ('speculative','combined'),4 if refined else 1,4 if refined else 1,a.max_depenetration_m_s)
    runtime_config.CONFIG=runtime_config.SeamRuntime(
        .0004 if a.coarse_thread else .0002,.0008 if a.coarse_thread else .0006,
        a.hole_mesh_spacing,a.physics_hz,settings,a.maximum_stretch,a.quality_mesh,
        .0024-(.0004 if a.coarse_thread else .0002) if (a.resting_seam or a.pre_draped_seam) else 0.,
        a.pre_draped_seam,a.drape_shape,a.sewing_clamp,a.passive_clamp,a.supported_clamp,a.cloth_contact_offset,a.backed_clamp,a.facet_cloth,a.box_panels,
        a.cpu_ccd,a.solver,a.clear_thread_sampling,a.passive_thread_bending,
        thread_long_side_links=a.thread_long_side_links,thread_inertia_scale=a.thread_inertia_scale)
    a.out.mkdir(parents=True,exist_ok=False)
    frozen=a.out/'task-source';frozen.mkdir()
    hashes={}
    for source in HERE.glob('*.py'):
        compile(source.read_bytes(),str(source),'exec')
        hashes[source.name]=hashlib.sha256(source.read_bytes()).hexdigest()
        shutil.copyfile(source,frozen/source.name)
    if a.plan is not None:
        shutil.copyfile(a.plan,frozen/'fixed-plan.json')
        hashes['fixed-plan.json']=hashlib.sha256(a.plan.read_bytes()).hexdigest()
    started=time.monotonic()
    report={'status':'running','robot_demo':False,'score':None,'dream_requests':0,
            'requested_steps':a.steps,'completed_steps':0,'control_hz':20,
            'crossing_backend':a.crossing_backend,
            'practical_material':a.practical_material,'functional_tacks':a.functional_tacks,
            'audit_junctions':a.audit_junctions,
            'continue_after_strain':a.continue_after_strain,
            'contact_forensics':a.contact_forensics,
            'geometric_panel_audit':a.geometric_panel_audit,
            'panel_passage_diagnostic':a.panel_passage_diagnostic,
            'reset_mode':'fresh-constructor-state' if a.keep_constructor_state else 'explicit-state-reset',
            'constructor_prefix_audited':False,
            'runtime':asdict(runtime_config.CONFIG),'source_sha256':hashes,
            'world_coordinate_offset_m':coordinate_frame.OFFSET_M.tolist(),
            'policy':'ordinary ready targets' if actions is None else 'frozen joint targets; no object feedback'}
    if a.pose_assisted_entry:
        report['policy']='scripted feasibility demo: frozen pickup then fixed blade path with filtered native tool-in-gripper pose compensation; ordinary bounded joint actions'
        report['pose_assistance_actions']={'first':141,'last':entry_stop,
            'after_last':'frozen joint targets with ordinary position/rate limits; no live tool-pose compensation'}
    app=video=trace=emb=break_observer=None
    dream=None;request_audit=[];session_audit={'session_count':0}
    profiler=None
    if a.profile_steps:
        import cProfile
        profiler=cProfile.Profile()
    def save():
        report['wall_seconds']=time.monotonic()-started
        (a.out/'preview.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    try:
        from isaaclab.app import AppLauncher
        device='cpu' if a.cpu_ccd else 'cuda:0'
        app=AppLauncher(headless=True,enable_cameras=True,device=device).app
        import carb
        import torch
        import imageio.v2 as imageio
        from PIL import Image
        import omni.physics.tensors as tensors
        import omni.usd
        from pxr import UsdPhysics,PhysxSchema,UsdUtils
        if a.facet_cloth:
            from joint_break_observer import JointBreakObserver
            break_observer=JointBreakObserver()
        from yam_arena import inspect_embodiment as wrapper,scene as adapter
        wrapper._APP=app
        adapter.PHYSICS_DT=1/a.physics_hz
        emb=wrapper.YamArenaEmbodiment(task=str(HERE/'scene.py'),object_jitter=0.,
                                      control_hz=20,cam_width=224,cam_height=224,device=device)
        original_callback=adapter.env_cfg_callback
        if a.external_forces_once or a.velocity_iterations is not None or a.cpu_ccd:
            def callback(*args,**kwargs):
                configure=original_callback(*args,**kwargs)
                def override(cfg):
                    cfg=configure(cfg)
                    if a.recenter_at_seam:cfg=coordinate_frame.shift_scene_config(cfg)
                    if cfg.sim.physics is None:
                        from isaaclab_physx.physics import PhysxCfg
                        cfg.sim.physics=PhysxCfg()
                    if a.external_forces_once:cfg.sim.physics.enable_external_forces_every_iteration=False
                    if a.cpu_ccd:
                        cfg.sim.physics.enable_ccd=True
                        cfg.sim.physics.solver_type=1 if a.solver=='TGS' else 0
                    if a.velocity_iterations is not None:
                        cfg.sim.physics.min_velocity_iteration_count=a.velocity_iterations
                        cfg.sim.physics.max_velocity_iteration_count=a.velocity_iterations
                    return cfg
                return override
            adapter.env_cfg_callback=callback
        original_make_env=adapter.make_env
        if a.judging_views:
            from review_views import configure as configure_views
            def make_env(embodiment,**kwargs):
                configure_views(embodiment)
                return original_make_env(embodiment,**kwargs)
            adapter.make_env=make_env
        try:
            emb._ensure_sim(None)
        finally:
            adapter.env_cfg_callback=original_callback
            adapter.make_env=original_make_env
        base=emb._base;stage=omni.usd.get_context().get_stage()
        report['coordinate_configuration_audit']=list(coordinate_frame.CONFIG_AUDIT)
        report['native_device']=str(base.device)
        native_scenes=[prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
        if len(native_scenes)!=1:raise RuntimeError('Expected one native physics scene')
        native_scene=PhysxSchema.PhysxSceneAPI(native_scenes[0])
        report['native_scene_physics']=dict(gpu=bool(native_scene.GetEnableGPUDynamicsAttr().Get()),
            ccd=bool(native_scene.GetEnableCCDAttr().Get()),solver=str(native_scene.GetSolverTypeAttr().Get()),
            broadphase=str(native_scene.GetBroadphaseTypeAttr().Get()))
        if a.cpu_ccd and (str(base.device)!='cpu' or report['native_scene_physics']!={
                'gpu':False,'ccd':True,'solver':a.solver,'broadphase':'MBP'}):
            raise RuntimeError('CPU scene does not match the selected native contact configuration')
        effective_force_mode=bool(native_scene.GetEnableExternalForcesEveryIterationAttr().Get())
        report['external_forces_once_requested']=a.external_forces_once
        report['effective_external_forces_every_iteration']=effective_force_mode
        if a.external_forces_once and effective_force_mode:
            raise RuntimeError('Required once-per-step force mode did not reach native scene')
        report['scene_velocity_iteration_range']=[int(native_scene.GetMinVelocityIterationCountAttr().Get()),
                                                  int(native_scene.GetMaxVelocityIterationCountAttr().Get())]
        if a.velocity_iterations is not None and report['scene_velocity_iteration_range']!=[a.velocity_iterations]*2:
            raise RuntimeError('Requested velocity iteration count did not reach native scene')
        # SimulationContext sets this to True while constructing the scene.
        # Use the same post-construction setting as IsaacLab's ContactSensor
        # before creating our directly filtered native sensor view. Setting it
        # before _ensure_sim is overwritten; this flag alone is not evidence
        # that GPU contact forces are absent (legacy CPU-only behavior).
        from isaaclab.app.settings_manager import get_settings_manager
        report['contact_processing_setting_before_sensor']=carb.settings.get_settings().get('/physics/disableContactProcessing')
        get_settings_manager().set_bool('/physics/disableContactProcessing',False)
        namespace=adapter.task_object.__globals__
        roots,cloth=namespace['ROOTS'],namespace['MESHES']
        from arena_thread import SPAWNED
        threads=[SPAWNED[f'stitch_{i}'] for i in range(6)]
        if a.passive_thread_bending:report['thread_bending']=[t['bending'] for t in threads]
        from cutting import ThreadCutter,CutParameters,DamageAudit
        from inspection import StrainAudit,inspect_removed_threads
        from contact_audit import ThreadClothAudit,capsule_endpoints
        from native_stitches import sever_contacted_joints,restore_separated_cut_contacts
        all_paths=[path for thread in threads for path in thread['bodies']]
        for prim in stage.Traverse():
            if 'Attachment' in prim.GetTypeName():raise RuntimeError('Unexpected attachment in seam scene')
        for path in all_paths:
            if not a.functional_tacks and UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path)).GetKinematicEnabledAttr().Get():
                raise RuntimeError('Task thread must remain dynamic')
        envroot=roots['ripper_handle'].rsplit('/',1)[0]
        facet_shells=None
        if a.facet_cloth:
            from arena_facet import SPAWNED as facet_spawned,FacetView
            facet_shells=[facet_spawned[name] for name in ('cloth_left','cloth_right')]
            report['cloth_model']='inextensible finite rigid facets with passive bending hinges; candidate, not FEM'
            report['minimum_panel_mass_kg']=.000005 if a.practical_material else 0.
            report['facet_body_counts']=[len(shell['bodies']) for shell in facet_shells]
            for shell in facet_shells:
                for path in shell['bodies']:
                    if UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path)).GetKinematicEnabledAttr().Get():
                        raise RuntimeError('Every fabric facet must remain native dynamic')
            if a.cpu_ccd:
                paths=all_paths+[path for shell in facet_shells for path in shell['bodies']]
                report['native_task_ccd']=[dict(path=path,
                    sweep=bool(PhysxSchema.PhysxRigidBodyAPI(stage.GetPrimAtPath(path)).GetEnableCCDAttr().Get()),
                    speculative=bool(PhysxSchema.PhysxRigidBodyAPI(stage.GetPrimAtPath(path)).GetEnableSpeculativeCCDAttr().Get()))
                    for path in paths]
                if not all(row['sweep'] and not row['speculative'] for row in report['native_task_ccd']):
                    raise RuntimeError('CPU cloth/thread CCD differs from the selected native candidate')
        else:
            for name in ('cloth_left','cloth_right'):
                prim=stage.GetPrimAtPath(envroot+'/'+name)
                if 'OmniPhysicsSurfaceDeformableSimAPI' not in prim.GetAppliedSchemas() or prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    raise RuntimeError('Cloth is not native surface FEM')
            contact_names=('physxCollision:restOffset','physxCollision:contactOffset',
                           'physxDeformableBody:selfCollisionFilterDistance',
                           'physxDeformableBody:maxDepenetrationVelocity',
                           'physxDeformableBody:maxLinearVelocity',
                           'physxDeformableBody:enableSpeculativeCCD',
                           'physxDeformableBody:collisionIterationMultiplier',
                           'physxDeformableBody:collisionPairUpdateFrequency')
            report['native_surface_contact']=[{key:stage.GetPrimAtPath(envroot+'/'+name).GetAttribute(key).Get()
                                               for key in contact_names}
                                              for name in ('cloth_left','cloth_right')]
            for row in report['native_surface_contact']:
                if not np.isclose(row['physxCollision:contactOffset'],a.cloth_contact_offset,rtol=0.,atol=1e-9):
                    raise RuntimeError('Authored cloth contact generation offset differs from the selected configuration')
                if not np.isclose(row['physxCollision:restOffset'],.0002,rtol=0.,atol=1e-9):
                    raise RuntimeError('Physical cloth rest offset changed')
                for key,value in row.items():
                    if isinstance(value,float) and not np.isfinite(value):row[key]=str(value)
        stage_cache=UsdUtils.StageCache.Get()
        stage_id=stage_cache.GetId(stage).ToLongInt()
        if stage_id<0:
            stage_cache.Insert(stage);stage_id=stage_cache.GetId(stage).ToLongInt()
        sim=tensors.create_simulation_view('torch',stage_id);sim.set_subspace_roots('/')
        report['native_tensor_stage_id']=stage_id
        report['native_tensor_device']=str(sim.device)
        if a.cpu_ccd and str(sim.device)!='cpu':raise RuntimeError('Expected CPU native tensors')
        blade=sim.create_rigid_body_view(roots['ripper_blade'])
        handle=sim.create_rigid_body_view(roots['ripper_handle'])
        moving_jaw=None
        if a.passive_clamp:
            from passive_clamp import description,MOVING_NAME
            report['passive_clamp']=description()
            moving_jaw=sim.create_rigid_body_view(roots[MOVING_NAME])
            if moving_jaw.count!=1:raise RuntimeError('Missing dynamic spring jaw')
        if a.backed_clamp:
            from repair_backing import description as backing_description
            report['repair_backing']=backing_description()
        thread_view=sim.create_rigid_body_view(all_paths)
        native_paths=list(thread_view.prim_paths)
        order=np.asarray([native_paths.index(path) for path in all_paths])
        views=([FacetView(sim,shell,geometry_aperture_m=(.005 if a.practical_material else .0004 if a.panel_passage_diagnostic else .0002) if a.geometric_panel_audit else None,
                        boundary_tolerance_m=.0001 if a.boundary_contact_envelope else 0.,
                        require_edge_overlap=not a.functional_tacks)
                for shell in facet_shells] if a.facet_cloth else
               [sim.create_surface_deformable_body_view(envroot+'/'+name) for name in ('cloth_left','cloth_right')])
        if blade.count!=1 or handle.count!=1 or any(v.count!=1 for v in views):
            raise RuntimeError('Missing native seam body')
        contact=sim.create_rigid_contact_view([roots['ripper_blade']],filter_patterns=[all_paths],max_contact_data_count=4096)
        if contact.sensor_count!=1 or contact.filter_count!=len(all_paths):
            raise RuntimeError('Blade force matrix does not cover every thread link')
        filter_paths=list(contact.filter_paths)
        if filter_paths and isinstance(filter_paths[0],(list,tuple)):filter_paths=filter_paths[0]
        force_order=np.asarray([filter_paths.index(path) for path in all_paths])
        dt=float(base.physics_dt)
        if abs(dt-1/a.physics_hz)>1e-10:raise RuntimeError('Requested physics timestep was not applied')
        report.update(physics_dt_s=dt,contact_processing_disabled=carb.settings.get_settings().get('/physics/disableContactProcessing'))
        if report['contact_processing_disabled'] is True:raise RuntimeError('Native contact reporting is disabled')
        from scene_snapshot import authored_scene,native_state,state_summary
        report['authored_scene']=authored_scene(stage,roots)
        stage.GetRootLayer().Export(str(a.out/'initial.usda'))
        report['initial_stage']='initial.usda'
        report['reset_observations']={}
        def snapshot_reset(label):
            row={'physics_step_count':int(base.sim._physics_step_count)}
            try:
                data=native_state(views,cloth,thread_view,order)
                np.savez_compressed(a.out/f'{label}-reset-state.npz',**data)
                row.update(state_summary(data,namespace['INITIAL']))
                row['maximum_thread_speed_m_s']=float(np.linalg.norm(data['thread_velocity'][:,:3],axis=1).max())
                row['maximum_thread_angular_speed_rad_s']=float(np.linalg.norm(data['thread_velocity'][:,3:],axis=1).max())
                if moving_jaw is not None:
                    row['moving_jaw_pose']=array(moving_jaw.get_transforms())[0].tolist()
                    row['moving_jaw_velocity']=array(moving_jaw.get_velocities())[0].tolist()
            except Exception as exc:
                row['snapshot_error']=f'{type(exc).__name__}: {exc}'
            report['reset_observations'][label]=row
        snapshot_reset('before')
        if not a.keep_constructor_state:
            if break_observer:break_observer.context('reset')
            emb._env.reset()
        snapshot_reset('after')
        grip=None
        if a.functional_tacks:
            from tool_grip import install as install_grip
            grip=install_grip(stage,base,handle)
        tacks=None
        if a.functional_tacks:
            from static_fabric_clamp import install as install_fabric_clamp
            report['fabric_clamp_grip']=install_fabric_clamp(stage,facet_shells[0],views[0])
            from functional_tacks import Tacks
            tacks=Tacks(stage,base,threads,facet_shells,views,thread_view,order)
            report['tack_law']='Closed loops follow upper cloth, with native interlayer needle-hole connectors. Native inward blade contact releases the stitch and its thread. Thread/fibre sliding and knot friction are approximated.'
        if a.facet_cloth:
            for view in views:view.reset_audit()
        # Every following substep is audited. Constructor-internal startup
        # steps are outside this hook and explicitly reported as unobserved.
        # Keeping their state requires a freshly constructed scene per trial;
        # no material/body state is rewound during control execution.
        cut_parameters=CutParameters(thread_radius_m=runtime_config.CONFIG.thread_radius_m,
            minimum_edge_force_n=.005 if a.sharp_edge_cut else .25,
            sustained_seconds=dt if a.sharp_edge_cut else .040)
        report['cutting_law']={'minimum_edge_force_n':cut_parameters.minimum_edge_force_n,
            'sustained_seconds':cut_parameters.sustained_seconds,
            'calibration':'Explicit engineering approximation of a sharp thread-cutting edge; not measured tool calibration. Only native contact directed into the blade qualifies.'}
        report['averaged_panel_strain_role']='telemetry_only_rigid_facets' if a.rigid_panel_strain_telemetry else 'domain_stop'
        cutter=ThreadCutter([len(t['bodies']) for t in threads],cut_parameters)
        damage=DamageAudit();strain=[StrainAudit(m.rest,m.faces,maximum_stretch=a.maximum_stretch) for m in cloth]
        boundary_tolerance=.0001 if a.boundary_contact_envelope else 0.
        report['boundary_contact_tolerance_m']=boundary_tolerance
        crossing=[ThreadClothAudit(m.faces,boundary_tolerance_m=boundary_tolerance) for m in cloth]
        lengths=np.concatenate([t['lengths'] for t in threads])
        from thread_integrity import ThreadIntegrity,ClosedJunctionCrossings
        integrity=ThreadIntegrity([len(t['lengths']) for t in threads],runtime_config.CONFIG.thread_radius_m)
        junctions=ClosedJunctionCrossings([m.faces for m in cloth],len(lengths),boundary_tolerance_m=boundary_tolerance) if a.audit_junctions else None
        contact_history=None
        if a.contact_forensics:
            from contact_history import ContactHistory
            contact_history=[ContactHistory(m.faces,runtime_config.CONFIG.thread_radius_m,dt) for m in cloth]
        gpu_crossing=None
        if a.crossing_backend!='cpu':
            from warp_contact import PersistentCrossingAudit
            gpu_crossing=PersistentCrossingAudit(cloth,len(lengths),dt,device=base.device)
        from tool_geometry import blade_edge_samples
        edge_local=blade_edge_samples()
        cut_contact_events=[]
        state={'steps':0,'invalid':False,'non_strain_invalid':False,'hard_invalid':False,'previous_edge':None,
               'last_physics_step':int(base.sim._physics_step_count),'max_edge_force_n':0.,
               'timings_s':dict(readback=0.,strain=0.,crossing=0.,junctions=0.,crossing_poll=0.,blade_damage=0.,cutting=0.,control_steps_total=0.,native_step_total=0.,scene_write_total=0.,scene_update_total=0.)}
        def panel_state():
            data={}
            if a.facet_cloth:
                for index,view in enumerate(views):
                    poses,velocity=view.native_state()
                    data[f'facet_poses{index}']=poses
                    data[f'facet_velocity{index}']=velocity
            return data
        def measured():
            poses=array(thread_view.get_transforms())[order]
            nodes=[array(view.get_simulation_nodal_positions())[0,:len(mesh.rest)] for view,mesh in zip(views,cloth)]
            pose=array(blade.get_transforms())[0];R=rotation(pose[3:])
            return poses,nodes,edge_local@R.T+pose[:3],R[:,0]
        def assess_step(apply_cut=True):
            start=time.perf_counter()
            poses,nodes,edge,normal=measured()
            segments=capsule_endpoints(poses,lengths);t=state['steps']*dt
            native_solid_invalid=False
            if a.panel_passage_diagnostic and not a.functional_tacks:
                for view in views:
                    native_solid_invalid=view.audit_thread_solids(segments,runtime_config.CONFIG.thread_radius_m,t) or native_solid_invalid
                if any(view.first_aperture_warning for view in views) and 'first_panel_aperture_warning_substep' not in report:
                    report['first_panel_aperture_warning_substep']=state['steps']
                    np.savez_compressed(a.out/'first-panel-aperture-warning.npz',left=nodes[0],right=nodes[1],
                        thread=poses,thread_lengths=lengths,left_faces=cloth[0].faces,right_faces=cloth[1].faces,
                        handle=array(handle.get_transforms())[0],blade=array(blade.get_transforms())[0],
                        time_s=t,**panel_state())
            for index in restore_separated_cut_contacts(stage,threads,segments):
                cut_contact_events.append(dict(time_s=t,joint=index,event='separated_cut_pair_contact_restored'))
            state['timings_s']['readback']+=time.perf_counter()-start
            start=time.perf_counter()
            if not a.functional_tacks or state['steps'] % round(1/(20*dt)) == 0:
                for j,points in enumerate(nodes):
                    strain[j].update(points,t)
            state['timings_s']['strain']+=time.perf_counter()-start
            start=time.perf_counter()
            if a.crossing_backend!='warp' and not a.functional_tacks:
                for j,points in enumerate(nodes):crossing[j].update(points,segments,t)
            if gpu_crossing is not None:gpu_crossing.update(nodes,segments,state['steps'])
            if contact_history is not None:
                for history,points in zip(contact_history,nodes,strict=True):history.update(points,segments,t)
            state['timings_s']['crossing']+=time.perf_counter()-start
            junction_invalid=False
            if junctions is not None and not a.functional_tacks:
                start=time.perf_counter()
                connectors=integrity.update(segments,cutter.severed,t)
                junction_invalid=junctions.update(nodes,connectors,cutter.severed,t) or integrity.first_failure is not None
                state['timings_s']['junctions']+=time.perf_counter()-start
            start=time.perf_counter()
            damage.update(edge if state['previous_edge'] is None else state['previous_edge'],edge,
                          nodes,[m.faces for m in cloth],t)
            state['timings_s']['blade_damage']+=time.perf_counter()-start
            state['previous_edge']=edge.copy()
            if a.practical_material and not a.functional_tacks:
                # Coarse-panel contact: tangential hole-edge penetration up to
                #1mm is an explicit numerical envelope. Deep solid crossings
                # and a2mm shared-edge opening remain failures. No pose is edited.
                from panel_diagnostics import solid_contacts
                interior=[]
                for j,view in enumerate(views):
                    c=solid_contacts(view.shell['model'],view.last_poses,segments,
                        radius=runtime_config.CONFIG.thread_radius_m,thickness=.0004)
                    interior += [dict(layer=j,**row) for row in c['native_midplane_intersections']
                        if row['nearest_panel_edge_m']>.001]
                report['practical_contact']={'time_s':t,'deep_panel_crossings':interior}
                native_solid_invalid=bool(interior)
                junction_invalid=False
            if a.functional_tacks:
                native_solid_invalid=False;junction_invalid=False
            native_fracture=bool(break_observer and break_observer.summary()['cloth_hinge_break_count'])
            facet_invalid=native_fracture or native_solid_invalid or (a.facet_cloth and any(view.first_failure for view in views))
            state['non_strain_invalid']=bool(facet_invalid or junction_invalid or damage.crossed_faces or (not a.practical_material and any(s.first_failure for s in crossing)))
            strain_invalid=not a.rigid_panel_strain_telemetry and any(s.first_excursion for s in strain)
            state['hard_invalid']=bool(facet_invalid or junction_invalid or damage.crossed_faces or strain_invalid)
            invalid=bool(state['non_strain_invalid'] or strain_invalid)
            if invalid:
                state['invalid']=True
                if 'first_invalid_substep' not in report:
                    report['first_invalid_substep']=state['steps']
                    np.savez_compressed(a.out/'first-invalid.npz',left=nodes[0],right=nodes[1],
                        thread=poses,left_faces=cloth[0].faces,right_faces=cloth[1].faces,time_s=t,
                        thread_lengths=lengths,handle=array(handle.get_transforms())[0],
                        blade=array(blade.get_transforms())[0],**panel_state())
            if apply_cut and not invalid and not a.contact_forensics:
                start=time.perf_counter()
                forces=array(contact.get_contact_force_matrix(dt=dt))[0,force_order]
                state['max_edge_force_n']=max(state['max_edge_force_n'],float(np.max(-forces@normal)))
                cut_joints=cutter.update(forces,normal,dt)
                if tacks is not None:tacks.cut(cut_joints,t)
                else:sever_contacted_joints(stage,threads,cut_joints)
                state['timings_s']['cutting']+=time.perf_counter()-start
            state.update(poses=poses,nodes=nodes,segments=segments)
        def crossing_summaries():
            if gpu_crossing is None:return [audit.summary() for audit in crossing]
            summaries=gpu_crossing.summaries()
            if a.crossing_backend=='warp-shadow':
                for cpu,gpu in zip(crossing,summaries,strict=True):
                    f,g=cpu.first_failure,gpu['first_failure']
                    if (f is None)!=(g is None):raise RuntimeError('Full-substep CPU/Warp crossing mismatch')
                    if f is not None and any(f[key]!=g[key] for key in ('time_s','static_intersections','swept_intersections')):
                        raise RuntimeError('CPU/Warp first-failure diagnostics mismatch')
            return summaries
        def check_crossing_boundary():
            start=time.perf_counter()
            try:summaries=crossing_summaries()
            finally:state['timings_s']['crossing_poll']+=time.perf_counter()-start
            failures=[s['first_failure'] for s in summaries if not s['valid']]
            if failures and not a.practical_material:
                state['invalid']=True
                if gpu_crossing is not None:
                    first=min(f['native_substep'] for f in failures)
                    report['first_invalid_substep']=min(first,report.get('first_invalid_substep',first))
                    gpu_crossing.save_failure(a.out/'first-crossing-pair.npz')
                if not a.contact_forensics:raise RuntimeError('Native thread/fabric crossing detected')
        assess_step(False)
        check_crossing_boundary()
        original_update=base.scene.update
        if a.profile_steps:
            def timed(function,key):
                def wrapped(*args,**kwargs):
                    begin=time.perf_counter()
                    try:return function(*args,**kwargs)
                    finally:state['timings_s'][key]+=time.perf_counter()-begin
                return wrapped
            # Explicit wall timers retain native/C++ wait time that is absent
            # from some profiler callbacks. These never change call cadence.
            base.sim.step=timed(base.sim.step,'native_step_total')
            base.scene.write_data_to_sim=timed(base.scene.write_data_to_sim,'scene_write_total')
            original_update=timed(original_update,'scene_update_total')
        def after_physics(*args,**kwargs):
            result=original_update(*args,**kwargs)
            count=int(base.sim._physics_step_count)
            if count!=state['last_physics_step']:
                state['last_physics_step']=count;state['steps']+=1;assess_step()
                stop=(state['hard_invalid'] if a.contact_forensics else
                      state['invalid'] and (not a.continue_after_strain or state['non_strain_invalid']))
                if stop:
                    raise RuntimeError('Native seam material/contact validity boundary crossed')
            return result
        base.scene.update=after_physics
        ready=array(emb._ready)[0]
        from yam_arena.constants import SHOULDER,WRIST,GRIPPER,FINGER_OPEN
        limits=np.tile(np.array([SHOULDER['velocity']/20]*3+[WRIST['velocity']/20]*3+
                               [GRIPPER['velocity']/abs(FINGER_OPEN)/20]),2)
        if actions is not None:
            from yam_arena.constants import SHOULDER,WRIST,GRIPPER,FINGER_OPEN
            limits=np.tile(np.array([SHOULDER['velocity']/20]*3+[WRIST['velocity']/20]*3+
                                   [GRIPPER['velocity']/abs(FINGER_OPEN)/20]),2)
            if np.any(actions<emb._low) or np.any(actions>emb._high) or np.any(np.abs(np.diff(np.vstack((ready,actions)),axis=0))>limits*1.001):
                raise ValueError('Fixed actions exceed ordinary joint/rate limits')
        video=imageio.get_writer(a.out/'preview.mp4',fps=20,codec='libx264',pixelformat='yuv420p',
                                ffmpeg_params=['-movflags','+faststart'],macro_block_size=1)
        trace=gzip.open(a.out/'trace.jsonl.gz','wt')
        right_robot=base.scene['right_robot']
        right_gripper_index=right_robot.body_names.index('gripper')
        from yam_arena.constants import TCP_OFFSET
        entry=None
        if a.pose_assisted_entry:
            from tool_entry import ToolEntryCompensation
            from yam_arena import ik,station
            from plan import DOWN
            from plan_cut import GRASP_OFFSET
            entry=ToolEntryCompensation(TCP_OFFSET,alpha=.1)
        def snapshot(step,action):
            hand=array(right_robot.data.body_link_pose_w)[0,right_gripper_index]
            hand_rotation=rotation(hand[3:]);handle_pose=array(handle.get_transforms())[0]
            rgb=array(base.scene['top_cam'].data.output['rgb'])[0,:,:,:3].astype(np.uint8)
            if a.judging_views:
                from review_views import composite_frame
                rgb=composite_frame(base)
            video.append_data(rgb)
            if step==0:Image.fromarray(rgb).save(a.out/'setup.png')
            row={'step':step,'time_s':step/20,'action14':action.tolist(),'physics_substeps':state['steps'],
                 'handle_pose_xyzw':handle_pose.tolist(),
                 'blade_pose_xyzw':array(blade.get_transforms())[0].tolist(),
                 'right_gripper_pose_xyzw':hand.tolist(),
                 'right_tcp_position_m':(hand[:3]+hand_rotation@np.asarray(TCP_OFFSET)).tolist(),
                 'handle_position_in_gripper_m':(hand_rotation.T@(handle_pose[:3]-hand[:3])).tolist(),
                 'handle_rotation_in_gripper':(hand_rotation.T@rotation(handle_pose[3:])).tolist(),
                 'blade_pose_xyzw':array(blade.get_transforms())[0].tolist(),
                 'thread_poses_xyzw':state['poses'].tolist(),
                 'thread_velocity_m_s_rad_s':array(thread_view.get_velocities())[order].tolist(),
                 'handle_velocity_m_s_rad_s':array(handle.get_velocities())[0].tolist(),
                 'blade_velocity_m_s_rad_s':array(blade.get_velocities())[0].tolist(),
                 'cloth_nodes':[n.tolist() for n in state['nodes']],
                 'cut_events':cutter.events,'invalid':state['invalid']}
            if grip is not None:row['tool_grip']=dict(grip)
            if tacks is not None:row['tacks']={'closed':list(tacks.closed),'events':list(tacks.events)}
            if a.facet_cloth:
                row['facet_poses_xyzw']=[view.last_poses.tolist() for view in views]
                row['facet_velocity_m_s_rad_s']=[array(view.view.get_velocities())[view.order].tolist() for view in views]
                row['facet_consistency']=[view.last_check for view in views]
                row['native_joint_breaks']=break_observer.summary()
            if moving_jaw is not None:
                row['moving_jaw_pose_xyzw']=array(moving_jaw.get_transforms())[0].tolist()
                row['moving_jaw_velocity_m_s_rad_s']=array(moving_jaw.get_velocities())[0].tolist()
            trace.write(json.dumps(row,allow_nan=False)+'\n');trace.flush()
        snapshot(0,ready);save()
        if state['invalid']:raise RuntimeError('Authored seam reset fails material/contact audit')
        if a.dream:
            from inspect_robots import Scene
            from dream_policy import make_policy
            spec_path=HERE/('task.json' if (HERE/'task.json').exists() else 'task.draft.json')
            instruction=json.loads(spec_path.read_text())['instruction']
            emb._instruction=instruction
            dream=make_policy(request_audit,session_audit,rows_per_request=a.dream_rows_per_request)
            dream.bind(emb.info);dream.reset(Scene(id='unpick-a-seam',instruction=instruction))
            report.update(dream_model=dream.info.checkpoint,dream_rows_per_request=a.dream_rows_per_request,
                policy='Dream: standard cameras and proprioception only; identical material model')
        from joint_tracking import JointTracking
        tracking=JointTracking(side='right')
        alignment=None
        if entry is not None and tacks is not None:
            from tack_alignment import TackAlignment
            alignment=TackAlignment(state['poses'],[len(t['bodies']) for t in threads])
        previous_action=ready.copy()
        for step in range(1,a.steps+1):
            action=ready.copy() if actions is None else actions[step-1].copy()
            if dream is not None:
                chunk=dream.act(emb._observation())
                if len(chunk.actions)!=1:raise RuntimeError('Expected one normal Dream action')
                action=np.asarray(chunk.actions[0].data,float)
                if action.shape!=(14,) or not np.isfinite(action).all():raise ValueError('Invalid Dream joint action')
                action=np.clip(action,emb._low,emb._high)
                action=previous_action+np.clip(action-previous_action,-limits,limits)
            if entry is not None and 140<step<=entry_stop:
                nominal_tcp,nominal_hand_r=ik.tcp_pose(action[7:13],station.BASES['right'])
                nominal_tool_r=nominal_hand_r@DOWN.T
                nominal_tool_p=nominal_tcp-nominal_tool_r@GRASP_OFFSET
                if alignment is not None:
                    nominal_tool_p=alignment.apply(nominal_tool_p,plan['frames'][step-1].get('phase',''),state['poses'],tacks.closed,step)
                    report['stitch_alignment']=alignment.records
                measured_hand=array(right_robot.data.body_link_pose_w)[0,right_gripper_index]
                measured_tool=array(handle.get_transforms())[0]
                # IK bases and frozen plan remain in station coordinates.
                # Only readback coordinates are converted; no body pose writes.
                measured_hand[:3]=coordinate_frame.to_station(measured_hand[:3])
                measured_tool[:3]=coordinate_frame.to_station(measured_tool[:3])
                goal_p,goal_r=entry.target(nominal_tool_p,nominal_tool_r,measured_hand,measured_tool)
                q,position_error,rotation_error=ik.solve(goal_p,ik.matrix_to_quat_xyzw(goal_r),
                    previous_action[7:13],station.BASES['right'],iters=200,tol=1e-6)
                if position_error>.001 or rotation_error>np.deg2rad(.5):
                    raise RuntimeError('Pose-assisted entry target is not reachable within the ordinary robot limits')
                action[7:13]=q
                action=tracking.action(action,np.asarray(emb._observation().state['joint_pos'],float),enabled=True)
            if entry is not None and step>140:
                # Also retain rate limits when the plan deliberately releases
                # the handle and returns to frozen, unassisted targets.
                action=np.clip(action,emb._low,emb._high)
                action=np.clip(action,previous_action-limits,previous_action+limits)
            start=time.perf_counter()
            try:
                if break_observer:break_observer.context('episode',step)
                if profiler is not None:profiler.enable()
                emb._env.step(torch.as_tensor(action[None],device=base.device,dtype=torch.float32))
            finally:
                if profiler is not None:profiler.disable()
                state['timings_s']['control_steps_total']+=time.perf_counter()-start
            check_crossing_boundary()
            emb._t=step/20
            snapshot(step,action);report['completed_steps']=step
            previous_action=action.copy()
            if step%20==0:save()
        if state['steps']!=round(a.steps*a.physics_hz/20):raise RuntimeError('Every-substep audit coverage mismatch')
        if gpu_crossing is not None and gpu_crossing.updates!=state['steps']+1:
            raise RuntimeError('GPU crossing audit missed a native substep')
        if junctions is not None and not a.functional_tacks and any(audit.steps!=state['steps']+1 for audit in junctions.audits):
            raise RuntimeError('Closed-junction audit missed a native substep')
        if a.panel_passage_diagnostic and not a.functional_tacks and any(view.solid_checks!=state['steps']+1 for view in views):
            raise RuntimeError('Actual native panel audit missed a substep')
        pieces=[];start=0
        for t in threads:
            end=start+len(t['lengths']);pieces.append(state['segments'][start:end]);start=end
        removed=inspect_removed_threads(pieces,state['nodes'],[m.faces for m in cloth],radius=runtime_config.CONFIG.thread_radius_m)
        for name in ('top_cam','left_cam','right_cam'):
            Image.fromarray(array(base.scene[name].data.output['rgb'])[0,:,:,:3].astype(np.uint8)).save(a.out/f'final-{name}.png')
        if a.judging_views:
            from review_views import capture_final
            report['judging_views']=capture_final(base,a.out)
        report.update(status='invalid_model_domain' if state['invalid'] else 'complete',thread_inspection=removed,
                      note='Unscored native robot diagnostic, not full task acceptance evidence.')
    except BaseException as exc:
        report.update(status='failed',error=f'{type(exc).__name__}: {exc}',traceback=traceback.format_exc())
        raise
    finally:
        if profiler is not None:
            import pstats
            profiler.disable()
            entries=[]
            for (filename,line,function),(primitive,calls,own,total,callers) in pstats.Stats(profiler).stats.items():
                entries.append(dict(filename=filename,line=line,function=function,primitive_calls=primitive,calls=calls,
                                    self_seconds=own,cumulative_seconds=total))
            (a.out/'control-profile.json').write_text(json.dumps(sorted(entries,key=lambda e:-e['cumulative_seconds']),indent=2)+'\n')
            report['control_profile']='control-profile.json'
        if 'state' in locals():
            if 'entry' in locals() and entry is not None:report['pose_assisted_entry']=entry.summary()
            try:
                cross_report=crossing_summaries()
                failed_steps=[row['first_failure']['native_substep'] for row in cross_report
                              if row.get('first_failure') and 'native_substep' in row['first_failure']]
                if failed_steps:report['first_invalid_substep']=min(min(failed_steps),report.get('first_invalid_substep',min(failed_steps)))
                if gpu_crossing is not None:
                    report['gpu_crossing_updates']=gpu_crossing.updates
                    if any(not s['valid'] for s in cross_report):gpu_crossing.save_failure(a.out/'first-crossing-pair.npz')
            except BaseException as exc:
                state['invalid']=True;report['status']='failed'
                report['crossing_backend_error']=f'{type(exc).__name__}: {exc}'
                cross_report=[audit.summary() for audit in crossing]
            report.update(audited_physics_substeps=state['steps'],maximum_edge_force_n=state['max_edge_force_n'],
                measured_wall_timings_s=state['timings_s'],
                material_invalid=state['invalid'],cut_events=cutter.events,damage_events=damage.events,
                cut_contact_events=cut_contact_events,
                pending_cut_contacts=[sorted(thread.get('pending_cut_contacts',set())) for thread in threads],
                contact_audit=cross_report,
                thread_joint_integrity=integrity.summary() if a.audit_junctions else None,
                closed_junction_contact_audit=junctions.summary() if junctions is not None else None,
                strain=[{'maximum_principal_stretch':s.maximum_observed,'first_validity_excursion':s.first_excursion,
                         'first_2pct_warning':s.first_warning,'maximum_stretch_limit':s.limit} for s in strain])
            if a.facet_cloth:report['facet_consistency']=[view.summary() for view in views]
            if contact_history is not None:
                report['forensic_contact_history']=[h.summary() for h in contact_history]
                report['forensic_contact_history_file']='contact-history.json'
                (a.out/'contact-history.json').write_text(json.dumps(
                    {'score':None,'original_invalidity_retained':True,
                     'layers':[{'summary':h.summary(),'steps':h.rows} for h in contact_history]},indent=2)+'\n')
            # Preserve the actual terminal geometry and velocities even for a
            # failed diagnostic. This is evidence only, never a reset target
            # or a declaration that a material-domain violation was repaired.
            try:
                velocities=[array(view.get_simulation_nodal_velocities())[0,:len(mesh.rest)]
                            for view,mesh in zip(views,cloth)]
                thread_velocity=array(thread_view.get_velocities())[order]
                np.savez_compressed(a.out/'last-measured-state.npz',
                    left=state['nodes'][0],right=state['nodes'][1],
                    left_velocity=velocities[0],right_velocity=velocities[1],
                    thread=state['poses'],thread_velocity=thread_velocity,
                    left_rest=cloth[0].rest,right_rest=cloth[1].rest,
                    left_faces=cloth[0].faces,right_faces=cloth[1].faces,
                    time_s=state['steps']*dt,material_invalid=state['invalid'],thread_lengths=lengths,**panel_state())
                report['last_measured_state']='last-measured-state.npz'
                report['final_maximum_cloth_speed_m_s']=[float(np.linalg.norm(v,axis=1).max()) for v in velocities]
                if moving_jaw is not None:
                    report['final_moving_jaw_pose']=array(moving_jaw.get_transforms())[0].tolist()
                    report['final_moving_jaw_velocity']=array(moving_jaw.get_velocities())[0].tolist()
            except BaseException as exc:
                report['last_measured_state_error']=f'{type(exc).__name__}: {exc}'
        if trace:trace.close()
        if video:video.close()
        if break_observer:
            report['native_joint_breaks']=break_observer.summary()
            break_observer.close()
        if dream is not None:
            from dream_policy import public_transcript
            (a.out/'dream-transcript.json').write_text(json.dumps(public_transcript(dream),indent=2)+'\n')
            (a.out/'dream-requests.json').write_text(json.dumps(request_audit,indent=2)+'\n')
            report.update(dream_requests=len(request_audit),dream_session_audit=session_audit)
        save()
        if app:app.close()


if __name__=='__main__':main()
