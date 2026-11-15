"""Bounded YAM/Arena smoke recorder. No task demo, Dream call or scored success.

Smoke mode holds the ordinary ready joint targets. Motion-probe mode replays a
frozen ordinary joint trajectory. The explicitly selected state-assisted grasp
uses native reset cloth geometry to plan ordinary finger targets; this scripted
feasibility mode is disclosed separately from Dream. No prop state is written.
"""
import argparse
from dataclasses import fields,is_dataclass
from datetime import datetime,timezone
import gzip
import hashlib
from importlib.metadata import version,PackageNotFoundError
import json
import os
from pathlib import Path
import shutil
import signal
import time
import numpy as np

HERE=Path(__file__).resolve().parent
CONTROL_HZ=20
MAX_SMOKE_STEPS=200


def encode(value):
    if isinstance(value,np.ndarray):
        return value.tolist()
    if isinstance(value,np.generic):
        return value.item()
    if is_dataclass(value):
        return {f.name:encode(getattr(value,f.name)) for f in fields(value)}
    if isinstance(value,dict):
        return {k:encode(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):
        return [encode(v) for v in value]
    return value


def sources():
    result={}
    for p in sorted(HERE.iterdir()):
        if p.suffix in ('.py','.json','.md') or p.name=='LICENSE':
            if p.suffix=='.py':
                compile(p.read_bytes(),str(p),'exec')
            elif p.suffix=='.json':
                json.loads(p.read_text())
            result[p.name]=hashlib.sha256(p.read_bytes()).hexdigest()
    return result


def package_versions():
    values={}
    for name in ('isaacsim','isaaclab','yam-arena','inspect-robots','inspect-robots-yam','numpy'):
        try:values[name]=version(name)
        except PackageNotFoundError:values[name]=None
    return values


def native_audit(stage,cloth_names,peg_names,allow_functional=False):
    """Fail closed if a custom asset silently becomes a rigid/proxy object."""
    from pxr import UsdPhysics
    result={'cloth':{},'pegs':{},'attachments':[],'physics_scenes':[]}
    for prim in stage.Traverse():
        path=str(prim.GetPath())
        schemas=prim.GetAppliedSchemas()
        if prim.IsA(UsdPhysics.Scene):
            result['physics_scenes'].append(dict(path=path,settings={name:prim.GetAttribute('physxScene:'+name).Get() for name in ('enableGPUDynamics','enableEnhancedDeterminism','enableExternalForcesEveryIteration','enableStabilization','enableCCD','solverType','frictionOffsetThreshold','frictionCorrelationDistance','gpuMaxSoftBodyContacts')}))
        if 'Attachment' in prim.GetTypeName() or any('Attachment' in s for s in schemas):
            result['attachments'].append(path)
        for name in cloth_names:
            if path.endswith('/'+name) and 'OmniPhysicsDeformableBodyAPI' in schemas:
                result['cloth'][name]={'path':path,'schemas':list(schemas),
                     'rigid_body':prim.HasAPI(UsdPhysics.RigidBodyAPI),
                     'self_collision':prim.GetAttribute('physxDeformableBody:selfCollision').Get(),
                     'speculative_ccd':prim.GetAttribute('physxDeformableBody:enableSpeculativeCCD').Get(),
                     'collision_pair_update_frequency':prim.GetAttribute('physxDeformableBody:collisionPairUpdateFrequency').Get(),
                     'rest_bend_angles_default':prim.GetAttribute('omniphysics:restBendAnglesDefault').Get(),
                     'solver_position_iterations':prim.GetAttribute('physxDeformableBody:solverPositionIterationCount').Get()}
        for name in peg_names:
            if '/'+name+'/' in path:
                entry=result['pegs'].setdefault(name,{'bodies':[],'joints':[],'kinematic_bodies':[]})
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    entry['bodies'].append(path)
                    if UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Get():
                        entry['kinematic_bodies'].append(path)
                if prim.IsA(UsdPhysics.Joint):
                    entry['joints'].append({'path':path,'type':prim.GetTypeName(),
                        'body0':[str(p) for p in prim.GetRelationship('physics:body0').GetTargets()],
                        'body1':[str(p) for p in prim.GetRelationship('physics:body1').GetTargets()]})
    unexpected=[path for path in result['attachments'] if not (allow_functional and path.endswith(('_cloth_grip','_dry_pinch')))]
    if unexpected:
        raise RuntimeError('Unexpected attachment in this attachment-free scene')
    for name in cloth_names:
        item=result['cloth'].get(name)
        if not item or item['rigid_body'] or item['self_collision'] is not True:
            raise RuntimeError('Missing native self-contact cloth: '+name)
    for name in peg_names:
        item=result['pegs'].get(name,{})
        if len(item.get('bodies',[]))!=2 or len(item.get('joints',[]))!=1:
            raise RuntimeError('Peg must have exactly two free jaws and one hinge: '+name)
        if item.get('kinematic_bodies') and not allow_functional:
            raise RuntimeError('Task peg jaw is kinematic: '+name)
        joint=item['joints'][0]
        if joint['type']!='PhysicsRevoluteJoint' or len(joint['body0'])!=1 or len(joint['body1'])!=1:
            raise RuntimeError('Peg has a world fixture or unexpected constraint: '+name)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',choices=('smoke','motion-probe','dream'),default='smoke')
    p.add_argument('--steps',type=int,default=None)
    p.add_argument('--adaptive-clip',action='store_true',help='Permit bounded extra ordinary alignment actions before closing a peg')
    p.add_argument('--plan',type=Path,help='Precomputed ordinary joint commands; required for motion-probe')
    p.add_argument('--seed',type=int,default=0)
    p.add_argument('--state-assisted-grasp',action='store_true',help='Explicit scripted feasibility demo: measure reset cloth once to place ordinary fingers; never used by Dream')
    p.add_argument('--contact-refined',action='store_true',help='Use the explicitly refined cloth contact band for every policy reset')
    p.add_argument('--regular-refined',action='store_true',help='Use the regular2.5mm cloth grid for every policy reset, preserving1.6mm seam width')
    p.add_argument('--pad-refined',action='store_true',help='Conforming1.25mm contact refinement at the two hem grips; unchanged material')
    p.add_argument('--no-turn-lift-diagnostic',action='store_true',help='380-action pickup with vertical lift and right release, preserving pinch orientation')
    p.add_argument('--raised-support-diagnostic',action='store_true',help='Use an explicitly155mm laundry support and a measured lateral grasp; otherwise100mm')
    p.add_argument('--continuous-pad-diagnostic',action='store_true',help='Reobserve the garment at action100 and target the continuous finger pads with ordinary joint commands')
    p.add_argument('--surface-contact-diagnostic',action='store_true',help='Native speculative cloth CCD and eight collision-pair updates; unchanged cloth material and thickness')
    p.add_argument('--contact-pinch-diagnostic',action='store_true',help='Ordinary jaw relief from measured pad penetration during the short cloth pickup; no attachments or state writes')
    p.add_argument('--lift-turn-diagnostic',action='store_true',help='Keep the pinch plane during withdrawal, then turn it slowly while lifting through ordinary joint actions')
    p.add_argument('--center-peg-diagnostic',action='store_true',help='Explicit reset with the first peg at line centre and the spare200mm to its left')
    p.add_argument('--reset-diagnostic',action='store_true',help='Read-only early reset native nodes and topology; unchanged reset physics')
    p.add_argument('--binary-cloth-trace',action='store_true',help='Preserve all native float32 cloth states in binary arrays, reducing diagnostic JSON overhead')
    p.add_argument('--solver-iterations',type=int,choices=(32,128),default=32,help='Native cloth numerical convergence diagnostic; material unchanged')
    p.add_argument('--physics-hz',type=int,choices=(120,240,480,960),default=480,help='Native timestep; material and control rate unchanged')
    p.add_argument('--edge-patch-diagnostic',action='store_true',help='Use a shorter approach and left hem patch130mm from centre for the finer-timestep reset')
    p.add_argument('--wide-insertion-diagnostic',action='store_true',help='Keep ordinary fingers fully open during measured cloth insertion before the unchanged pinch')
    p.add_argument('--external-forces-once-diagnostic',action='store_true',help='Use the ordinary once-per-step TGS force integration option for a controlled comparison')
    p.add_argument('--state-assisted-clip',action='store_true',help='Explicit scripted demo: center free peg jaws with ordinary robot actions before closure')
    p.add_argument('--functional-pegs',action='store_true',help='Disclosed contact-driven peg and ideal dry fabric-grip model')
    p.add_argument('--dream-rows-per-request',type=int,choices=(1,2,4,8),default=4)
    p.add_argument('--joint-feedback-from',type=int,help='Enable bounded right-arm PI tracking after this many actions, using ordinary joint observations only')
    p.add_argument('--judging-views',action='store_true',help='Record additional native960×720 front/rear assessor videos')
    p.add_argument('--out',type=Path)
    p.add_argument('--check-only',action='store_true',help='Compile source and check JSON without loading a simulator')
    args=p.parse_args()
    os.environ['ROBOCURVE_SHIRT_FUNCTIONAL_PEGS']='1' if args.functional_pegs else '0'
    os.environ['ROBOCURVE_SHIRT_RAISED_SUPPORT']='1' if args.raised_support_diagnostic else '0'
    os.environ['ROBOCURVE_SHIRT_SURFACE_CONTACT_DIAGNOSTIC']='1' if args.surface_contact_diagnostic else '0'
    os.environ['ROBOCURVE_SHIRT_CENTER_PEG_DIAGNOSTIC']='1' if args.center_peg_diagnostic else '0'
    if args.regular_refined and args.contact_refined:p.error('Select one garment mesh')
    if args.pad_refined and not args.regular_refined:p.error('Pad refinement requires the regular base mesh')
    os.environ['ROBOCURVE_SHIRT_PAD_REFINED']='1' if args.pad_refined else '0'
    fixed_plan=fixed_actions=None
    if args.mode=='motion-probe':
        if args.plan is None:
            p.error('motion-probe requires a precomputed --plan')
        from fixed_motion import load
        fixed_plan,fixed_actions=load(args.plan)
        if bool(fixed_plan.get('requires_center_peg_reset'))!=args.center_peg_diagnostic:
            p.error('Frozen motion and explicit peg reset must match')
        if fixed_plan.get('do_not_run_reason'):
            p.error('Blocked diagnostic plan: '+fixed_plan['do_not_run_reason'])
        if fixed_plan.get('line_pegs_required'):
            expected=hashlib.sha256(Path(__file__).with_name('line_pegs.py').read_bytes()).hexdigest()
            if fixed_plan.get('line_peg_source_sha256')!=expected or not 'LINE_PEGS_ENABLED=True' in Path(__file__).with_name('task.py').read_text():
                p.error('Motion requires the matching passive pegs-on-line reset')
        if fixed_plan.get('peg_rests_required') or fixed_plan.get('peg_rests_candidate'):
            expected=hashlib.sha256(Path(__file__).with_name('peg_rests.py').read_bytes()).hexdigest()
            if fixed_plan.get('peg_rest_source_sha256')!=expected:
                p.error('Peg motion does not match the current rest source; recompile and audit first')
            if not 'PEG_RESTS_ENABLED=True' in Path(__file__).with_name('task.py').read_text():
                p.error('Peg motion requires the declared raised-rest task reset')
        from scene import SUPPORT_BOARD_THICKNESS
        if fixed_plan.get('fixture_board_thickness_m')!=SUPPORT_BOARD_THICKNESS:
            p.error('Frozen motion was not compiled for the current support thickness; recompile and audit first')
        args.steps=len(fixed_actions) if args.steps is None else args.steps
        if not 1<=args.steps<=len(fixed_actions):
            p.error('--steps exceeds the frozen action sequence')
    else:
        if args.plan is not None:
            p.error('smoke mode has no movement plan')
        maximum=2400 if args.mode=='dream' else MAX_SMOKE_STEPS
        args.steps=maximum if args.mode=='dream' and args.steps is None else 60 if args.steps is None else args.steps
        if not 1<=args.steps<=maximum:p.error(f'This mode permits1–{maximum} control steps')
    if fixed_plan is not None:
        if bool(fixed_plan.get('requires_pad_refinement'))!=args.pad_refined:
            p.error('Frozen motion and contact mesh refinement must match')
        if fixed_plan.get('required_cloth_mesh')=='regular_2p5mm' and not args.regular_refined:
            p.error('This motion requires the validated regular 2.5 mm cloth mesh')
        expected_iterations=fixed_plan.get('required_solver_iterations')
        if expected_iterations is not None and args.solver_iterations!=expected_iterations:
            p.error('This motion requires its recorded cloth solver iteration count')
    if args.adaptive_clip and not (args.mode=='motion-probe' and args.state_assisted_clip and args.steps<=2400):p.error('Adaptive clip is a bounded ordinary demo controller only')
    if args.mode=='dream' and (args.state_assisted_grasp or args.state_assisted_clip or args.joint_feedback_from is not None):p.error('Dream receives ordinary observations and its own actions only')
    if args.joint_feedback_from is not None and (args.mode!='motion-probe' or not 0<=args.joint_feedback_from<args.steps):
        p.error('Joint feedback requires a motion probe and a start before its last action')
    if fixed_plan is not None and fixed_plan.get('requires_joint_feedback_from')!=args.joint_feedback_from:
        p.error('Joint feedback option must exactly match the frozen plan')
    if args.state_assisted_grasp and (args.mode!='motion-probe' or args.steps<200):
        p.error('State-assisted reset grasp requires at least200 scripted motion actions')
    if args.continuous_pad_diagnostic and not (args.state_assisted_grasp and args.raised_support_diagnostic):
        p.error('Continuous-pad diagnostic requires the raised-support scripted grasp')
    if args.edge_patch_diagnostic and not (args.continuous_pad_diagnostic and args.lift_turn_diagnostic and (args.physics_hz==960 or args.functional_pegs)):
        p.error('Edge patch requires the finer-timestep measured lift-turn diagnostic')
    if args.wide_insertion_diagnostic and not args.continuous_pad_diagnostic:
        p.error('Wide insertion requires the measured continuous-pad diagnostic')
    if args.contact_pinch_diagnostic and not (args.continuous_pad_diagnostic and args.surface_contact_diagnostic and args.steps==320):
        p.error('Contact pinch is restricted to the320-action surface-contact pickup diagnostic')
    if args.lift_turn_diagnostic and not (args.continuous_pad_diagnostic and args.surface_contact_diagnostic and args.steps>=320 and not args.contact_pinch_diagnostic):
        p.error('Lift-turn comparison requires a complete surface-contact pickup without jaw relief')
    if args.no_turn_lift_diagnostic and not (args.lift_turn_diagnostic and args.steps==380 and not args.state_assisted_clip):
        p.error('No-turn lift requires the380-action measured pickup diagnostic')
    if args.state_assisted_clip and (args.mode!='motion-probe' or args.steps<940 or (not args.functional_pegs and args.joint_feedback_from!=700)):
        p.error('State-assisted clip requires the tracked first-clip sequence through at least940 actions')
    if not 0<=args.seed<2**32:
        p.error('Seed must be an unsigned 32-bit integer')
    source_hashes=sources()
    if args.check_only:
        print(json.dumps({'status':'local_syntax_and_json_passed','source_sha256':source_hashes,
                          'native_runtime':'not_tested','score':None},indent=2))
        return
    if args.out is None:
        p.error('--out is required for a native smoke run')
    versions=package_versions()
    if not (versions.get('isaacsim') or '').startswith('6.1.'):
        raise RuntimeError('Use the retained Isaac Sim 6.1 environment; this runner has no CPU fallback')
    args.out.mkdir(parents=True,exist_ok=False)
    frozen=args.out/'task-source'
    frozen.mkdir()
    for name in source_hashes:
        shutil.copyfile(HERE/name,frozen/name)
    if args.plan is not None:
        shutil.copyfile(args.plan,frozen/'fixed-motion.json')
        source_hashes['fixed-motion.json']=hashlib.sha256(args.plan.read_bytes()).hexdigest()
    draft=json.loads((HERE/('task.json' if (HERE/'task.json').exists() else 'task.draft.json')).read_text())
    started=time.monotonic()
    report={'kind':'diagnostic_yam_'+args.mode+'_not_submission_evidence','status':'initializing',
            'score':None,'demo':False,'dream_requests':0,'started_utc':datetime.now(timezone.utc).isoformat(),
            'source_sha256':source_hashes,'versions':versions,'seed':args.seed,
            'control_hz':CONTROL_HZ,'requested_steps':args.steps,'completed_steps':0,
            'policy':('constant adapter-ready joint targets' if fixed_plan is None else 'frozen fixed joint trajectory')+'; no object/assessor feedback',
            'instruction':draft['instruction'],'joint_feedback_from':args.joint_feedback_from,'state_assisted_grasp':args.state_assisted_grasp,'contact_refined':args.contact_refined,'regular_refined':args.regular_refined,'support_height_m':.155 if args.raised_support_diagnostic else .10,'solver_iterations':args.solver_iterations,'external_forces_once_diagnostic':args.external_forces_once_diagnostic}
    report.update(pad_refined=args.pad_refined,no_turn_lift_diagnostic=args.no_turn_lift_diagnostic)
    if args.joint_feedback_from is not None:
        report['policy']='frozen joint reference with bounded right-arm PI from ordinary joint observations; no object/assessor feedback'
    if args.state_assisted_grasp:
        report['policy']='Explicit state-assisted scripted feasibility demonstration: native reset cloth selects ordinary grasp targets; normal joint tracking thereafter. This is not a Dream policy run.'
    if args.state_assisted_clip:
        report['state_assisted_clip']=True
        report['policy']+=' Free-jaw poses also guide ordinary peg-mouth alignment before release.'
    def save():
        report['wall_seconds']=time.monotonic()-started
        temporary=args.out/'run.json.tmp'
        temporary.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        temporary.replace(args.out/'run.json')
    save()
    app=video=trace=emb=dense_trace=dream=None
    request_audit=[];session_audit={'session_count':0}
    judging_videos={}
    def terminate(signum,frame):
        raise SystemExit('SIGTERM: bounded diagnostic interrupted')
    previous_sigterm=signal.signal(signal.SIGTERM,terminate)
    try:
        os.environ['ROBOCURVE_SHIRT_CONTACT_REFINED']='1' if args.contact_refined else '0'
        os.environ['ROBOCURVE_SHIRT_REGULAR_REFINED']='1' if args.regular_refined else '0'
        os.environ['ROBOCURVE_SHIRT_SOLVER_ITERATIONS']=str(args.solver_iterations)
        os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES')
        from isaaclab.app import AppLauncher
        app=AppLauncher(headless=True,enable_cameras=True).app
        import torch
        import imageio.v2 as imageio
        from PIL import Image
        import omni.usd
        import arena_cloth
        from arena_props import PEG_SPECS
        from yam_arena import inspect_embodiment as wrapper,scene
        # The normal callback derives decimation from this timestep and20 Hz.
        scene.PHYSICS_DT=1/args.physics_hz
        if args.external_forces_once_diagnostic:
            original_cfg_callback=scene.env_cfg_callback
            def configure_callback(*values,**kwargs):
                configure=original_cfg_callback(*values,**kwargs)
                def override(cfg):
                    cfg=configure(cfg)
                    if cfg.sim.physics is None:
                        from isaaclab_physx.physics import PhysxCfg
                        cfg.sim.physics=PhysxCfg()
                    cfg.sim.physics.enable_external_forces_every_iteration=False
                    return cfg
                return override
            scene.env_cfg_callback=configure_callback
        if args.judging_views:
            from review_views import configure,metadata
            original_make_env=scene.make_env
            def with_review_cameras(embodiment,**kwargs):
                configure(embodiment)
                return original_make_env(embodiment,**kwargs)
            scene.make_env=with_review_cameras
            report['judging_cameras']=metadata()
        np.random.seed(args.seed)
        torch.manual_seed(args.seed)
        wrapper._APP=app
        emb=wrapper.YamArenaEmbodiment(task=str(HERE/'task.py'),object_jitter=0.,
                                      control_hz=CONTROL_HZ,cam_width=224,cam_height=224)
        emb._ensure_sim(None)
        base=emb._base
        emb._instruction=draft['instruction']
        from reset_scene import reset_and_settle
        report['reset']={'method':'raw reset plus five seconds of fixed native settling','status':'requested'}
        save()
        reset_diagnostic=None
        if args.reset_diagnostic:
            from reset_diagnostic import ResetDiagnostic
            reset_diagnostic=ResetDiagnostic(emb,args.out)
            report['native_scene_before_reset']=native_audit(omni.usd.get_context().get_stage(),list(arena_cloth.SPECS),list(PEG_SPECS))
        def reset_observer(label,elapsed):
            report['reset_progress']={'phase':label,'simulated_seconds':elapsed,
                                      'target_simulated_seconds':5.}
            save()
            if reset_diagnostic is not None:reset_diagnostic(label,elapsed)
        report['reset']=reset_and_settle(emb,observer=reset_observer)
        stage=omni.usd.get_context().get_stage()
        names=list(arena_cloth.SPECS)+list(PEG_SPECS)
        if not names:
            raise RuntimeError('Task custom assets were not registered')
        mechanisms=pinches=None
        if args.functional_pegs:
            from functional_pegs import install
            from pinch_constraint import install as install_pinch
            mechanisms=install(base);pinches=install_pinch(base,'tshirt')
        report['native_scene_audit']=native_audit(stage,list(arena_cloth.SPECS),list(PEG_SPECS),allow_functional=args.functional_pegs)
        if args.surface_contact_diagnostic:
            actual=report['native_scene_audit']['cloth']['tshirt']
            if actual['speculative_ccd'] is not True or actual['collision_pair_update_frequency']!=8:
                raise RuntimeError('Requested contact settings did not reach the native garment')
        if report['native_scene_audit']['cloth']['tshirt']['solver_position_iterations']!=args.solver_iterations:
            raise RuntimeError('Cloth iteration request did not reach native scene')
        if args.external_forces_once_diagnostic and any(p['settings']['enableExternalForcesEveryIteration'] for p in report['native_scene_audit']['physics_scenes']):
            raise RuntimeError('Force integration diagnostic did not reach native scene')
        report['physics_dt_s']=float(base.sim.get_physics_dt())
        report['initial_physics_step_count']=int(base.sim._physics_step_count)
        report['initial_snapshot']='After one raw reset and the declared five-second native cloth/peg settling period'
        if args.raised_support_diagnostic:
            from audit_self_contact import crossing_pairs
            from fixture_clearance import fixture_intersections
            from scene import support_geometry,rack_geometry,TABLE
            actual=arena_cloth.state(emb._env,'tshirt').nodes
            spec=arena_cloth.SPECS['tshirt']
            # Sub-thickness contact error is an explicit numerical approximation.
            # Keep ordinary cloth folds and fixture rejection meaningful without
            # treating micrometre solver noise as material destruction.
            np.savez_compressed(args.out/'raised-reset-state.npz',nodes=actual,rest=spec.mesh.rest,faces=spec.mesh.faces)
            pairs=crossing_pairs(actual,spec.mesh.faces,tolerance=.001,cell=.005)
            fixtures=support_geometry()|rack_geometry()
            fixtures['Table']=dict(kind='box',center=(.36,0.,TABLE-.03),size=(1.1,1.3,.06))
            hits=fixture_intersections(actual,spec.mesh.faces,fixtures,margin=-.001)
            report['raised_reset_integrity']=dict(transverse_pairs=len(pairs),fixture_hits={k:len(v) for k,v in hits.items()},self_contact_tolerance_m=.001,fixture_tolerance_m=.001)
            save()
            if pairs or any(hits.values()):raise RuntimeError('Raised-support reset fails actual material/fixture integrity')
        if args.state_assisted_grasp:
            if args.raised_support_diagnostic:
                from lateral_grasp import calibrate
            else:
                from scripted_grasp import calibrate
            from audit_motion import audit
            reset_cloth=arena_cloth.state(emb._env,'tshirt')
            np.savez_compressed(args.out/'measured-reset-cloth.npz',
                nodes=reset_cloth.nodes,rest=arena_cloth.SPECS['tshirt'].mesh.rest,
                faces=arena_cloth.SPECS['tshirt'].mesh.faces)
            options=([{'edge_patch':True,'clear_height':.035,'left_closing_tilt':tilt} for tilt in (0.,-10.,10.)]+
                [{'edge_patch':True,'clear_height':height,'left_closing_tilt':-10.} for height in (0.,.02,.05)]
                if args.edge_patch_diagnostic else [{}])
            route_checks=[]
            for option in options:
                try:
                    candidate,calibration=calibrate(fixed_plan,arena_cloth.SPECS['tshirt'].mesh.rest,reset_cloth.nodes,**option)
                    rejection=audit(dict(candidate,frames=candidate['frames'][:200]))
                    route_checks.append(dict(options=option,violations=len(rejection['violations'])))
                    if rejection['passed_rejection_check']:
                        fixed_plan=candidate
                        break
                except ValueError as exc:
                    route_checks.append(dict(options=option,error=str(exc)))
            else:
                (args.out/'reset-grasp-route-checks.json').write_text(json.dumps(route_checks,indent=2)+'\n')
                raise RuntimeError('No measured reset grasp route passed unchanged reach and fixture checks')
            (args.out/'reset-grasp-route-checks.json').write_text(json.dumps(route_checks,indent=2)+'\n')
            # Only the first200 ordinary reference actions change. The rest
            # stays byte-for-byte identical to the reviewed fixed reference.
            prefix=dict(fixed_plan,frames=fixed_plan['frames'][:200])
            rejection=audit(prefix)
            (args.out/'reset-grasp-hull-audit.json').write_text(json.dumps(rejection,indent=2)+'\n')
            if not rejection['passed_rejection_check']:
                raise RuntimeError('State-assisted grasp failed the rigid fixture/hull rejection check')
            (frozen/'fixed-motion.json').rename(frozen/'requested-fixed-motion.json')
            (frozen/'fixed-motion.json').write_text(json.dumps(fixed_plan,indent=2)+'\n')
            fixed_actions=np.asarray([f['action14'] for f in fixed_plan['frames']],float)
            report['scripted_reset_grasp']=calibration
            report['executed_plan_sha256']=hashlib.sha256((frozen/'fixed-motion.json').read_bytes()).hexdigest()
            save()
        (args.out/'meshes.json').write_text(json.dumps({name:{'rest':spec.mesh.rest.tolist(),
            'faces':spec.mesh.faces.tolist(),'markers':spec.mesh.markers} for name,spec in arena_cloth.SPECS.items()})+'\n')
        video=imageio.get_writer(args.out/'diagnostic.mp4',fps=CONTROL_HZ,codec='libx264',pixelformat='yuv420p',
                                  ffmpeg_params=['-movflags','+faststart'],macro_block_size=1)
        if args.judging_views:
            for camera in report['judging_cameras']:
                judging_videos[camera['name']]=imageio.get_writer(
                    args.out/(camera['name'].replace('_','-')+'.mp4'),fps=CONTROL_HZ,
                    codec='libx264',pixelformat='yuv420p',ffmpeg_params=['-movflags','+faststart'],macro_block_size=1)
        trace=gzip.open(args.out/'trace.jsonl.gz','wt',compresslevel=1)
        if args.binary_cloth_trace:
            from trace_data import DenseClothTrace
            dense_trace=DenseClothTrace(args.out,arena_cloth.SPECS,(2400 if args.adaptive_clip else args.steps)+1)
            report['cloth_trace_format']='All control-step float32 nodes and velocities in named .npy arrays, referenced by trace JSON; no state sampling or precision reduction'
        ready=emb._ready.detach().cpu().numpy()[0].copy()
        if ready.shape!=(14,) or not np.isfinite(ready).all():
            raise RuntimeError('Invalid ready action')
        if np.any(ready<emb._low) or np.any(ready>emb._high):
            raise RuntimeError('Ready action exceeds normal adapter limits')
        report['initial_action14']=ready.tolist()
        if fixed_actions is not None and (np.any(fixed_actions<emb._low) or np.any(fixed_actions>emb._high)):
            raise ValueError('Fixed trajectory exceeds ordinary adapter action limits')
        from yam_arena.constants import SHOULDER,WRIST,GRIPPER,FINGER_OPEN
        delta_limit=np.tile(np.array([SHOULDER['velocity']/CONTROL_HZ]*3+
            [WRIST['velocity']/CONTROL_HZ]*3+[GRIPPER['velocity']/abs(FINGER_OPEN)/CONTROL_HZ]),2)
        if fixed_actions is not None:
            if np.any(np.abs(np.diff(np.vstack((ready,fixed_actions)),axis=0))>delta_limit*1.001):
                raise ValueError('Fixed trajectory exceeds normal joint/gripper speed limits')
        from joint_tracking import JointTracking
        tracking=JointTracking()
        from contact_pinch import ContactPinch
        pinch=ContactPinch() if args.contact_pinch_diagnostic else None
        from scripted_clip import PegMouthServo
        clip_servos=[]
        if args.state_assisted_clip:
            configurations=fixed_plan.get('clip_servos') or [dict(
                window=fixed_plan.get('clip_servo_window',[820,880]),peg='left_peg')]
            for config in configurations:
                window=config['window']
                clip_servos.append((config['peg'],PegMouthServo(start=window[0],end=window[1],
                    target=config.get('target',(.52,.115,1.09)),side=config.get('side','right'),
                    align_y=config.get('align_y',False),functional_jaws=args.functional_pegs)))
        report['joint_tracking_parameters']={k:getattr(tracking,k) for k in ('kp','ki','limit','slew')}
        active_action=ready.copy();reference_action=ready.copy();requested_action=ready.copy()
        def arr(value):
            return getattr(value,'torch',value).detach().cpu().numpy()
        def snapshot(step):
            observation=emb._observation()
            state=scene.object_states(emb._env,names)
            recorded_state=dense_trace.record(state,step,encode) if dense_trace is not None else encode(state)
            record={'step':step,'time_s':step/CONTROL_HZ,'action14':active_action.tolist(),
                    'measured_joint_pos':encode(np.asarray(observation.state['joint_pos'])),
                    'reference_action14':reference_action.tolist(),'requested_action14':requested_action.tolist(),
                    'right_tracking_bias6':tracking.bias.tolist(),
                    'objects':recorded_state,'physics_step_count':int(base.sim._physics_step_count),
                    'robot_links':{name:{'names':list(base.scene[name].body_names),
                         'poses_xyzw':arr(base.scene[name].data.body_link_pose_w)[0].tolist()}
                         for name in ('left_robot','right_robot')}}
            if mechanisms:
                record['functional_pegs']={name:dict(held=p.held,clipped=p.clipped,opening_rad=p.opening,captured_vertices=p.indices,attachment_error_m=getattr(p,'attachment_error_m',None),events=p.events) for name,p in mechanisms['pegs'].items()}
                record['pinches']={name:dict(indices=p.indices.tolist(),events=p.events,correction_m=p.last_correction) for name,p in pinches.items()}
            trace.write(json.dumps(record,allow_nan=False)+'\n')
            trace.flush()
            report['last_recorded_native_step']=step
            image=np.asarray(observation.images['top_cam'],dtype=np.uint8)
            video.append_data(image)
            if args.judging_views:
                from review_views import images
                for camera,pixels in images(base,warnings=report.setdefault('judging_frame_warnings',[]),step=step).items():
                    judging_videos[camera].append_data(pixels)
                    if step in (0,args.steps):
                        label='start' if step==0 else 'final'
                        Image.fromarray(pixels).save(args.out/f'{camera.replace("_","-")}-{label}.png')

            if step in (0,args.steps):
                label='start' if step==0 else 'final'
                for camera,rgb in observation.images.items():
                    Image.fromarray(np.asarray(rgb,dtype=np.uint8)).save(args.out/f'diagnostic-{label}-{camera}.png')
        snapshot(0)
        if args.mode=='dream':
            from inspect_robots import Scene
            from dream_policy import make_policy
            dream=make_policy(request_audit,session_audit,rows_per_request=args.dream_rows_per_request)
            dream.bind(emb.info);dream.reset(Scene(id='peg-out-a-tshirt',instruction=draft['instruction']))
            report.update(dream_model=dream.info.checkpoint,policy='Dream: ordinary camera/joint observations only, no material-state or scripted controller input')
        report['status']='running'
        # Kit may replace process handlers during startup. Re-arm after setup
        # so a bounded trial preserves its partial evidence on termination.
        signal.signal(signal.SIGTERM,terminate)
        save()
        command=torch.as_tensor(ready[None,:],device=base.device,dtype=torch.float32)
        for step in range(1,(2400 if args.adaptive_clip else args.steps)+1):
            if step>args.steps:break
            if args.adaptive_clip:
                from adaptive_clip import extend_alignment
                from scripted_clip import line_capture
                from arena_props import peg_state
                for peg_name,servo in clip_servos:
                    if step==servo.end+1 and not line_capture(peg_state(emb._env,peg_name).body_poses,line_x=servo.target[0],line_z=servo.target[2])['can_attempt_closure']:
                        extend_alignment(fixed_plan,clip_servos,servo,step)
                        args.steps=len(fixed_plan['frames']);fixed_actions=np.asarray([f['action14'] for f in fixed_plan['frames']],float)
                        report['requested_steps']=args.steps
                        report.setdefault('extra_alignment_actions',[]).append(dict(before_step=step,peg=peg_name,actions=20))
                        (frozen/'fixed-motion.json').write_text(json.dumps(fixed_plan,indent=2)+'\n')
                        report['executed_plan_sha256']=hashlib.sha256((frozen/'fixed-motion.json').read_bytes()).hexdigest();save()
            if step==101 and args.continuous_pad_diagnostic:
                from lateral_pad_candidate import reobserve
                from audit_motion import audit
                observed=np.asarray(emb._observation().state['joint_pos'],float)
                actual=arena_cloth.state(emb._env,'tshirt').nodes
                candidate=reobserve(fixed_plan,observed,arena_cloth.SPECS['tshirt'].mesh.rest,actual,
                    lift_turn=args.lift_turn_diagnostic,edge_patch=args.edge_patch_diagnostic,
                    wide_insertion=args.wide_insertion_diagnostic,no_turn_lift=args.no_turn_lift_diagnostic)
                if args.functional_pegs:
                    # Re-observe only the grasp; retain the separately planned
                    # lift/carry after action200. Commands remain rate limited.
                    candidate['frames'][200:]=fixed_plan['frames'][200:]
                    from lateral_grasp import blend_lift
                    blend_lift(candidate)
                rejection=audit(dict(candidate,frames=candidate['frames'][:380 if args.no_turn_lift_diagnostic else 320]))
                (args.out/'continuous-pad-hull-audit.json').write_text(json.dumps(rejection,indent=2)+'\n')
                if not rejection['passed_rejection_check']:
                    raise RuntimeError('Continuous-pad candidate fails rigid fixture/hull screening')
                fixed_plan=candidate
                fixed_actions=np.asarray([f['action14'] for f in fixed_plan['frames']],float)
                if np.any(fixed_actions<emb._low) or np.any(fixed_actions>emb._high):
                    raise RuntimeError('Continuous-pad candidate exceeds ordinary joint bounds')
                (frozen/'fixed-motion.json').write_text(json.dumps(fixed_plan,indent=2)+'\n')
                report['continuous_pad_diagnostic']=candidate['pad_candidate_calibration']
                report['executed_plan_sha256']=hashlib.sha256((frozen/'fixed-motion.json').read_bytes()).hexdigest()
                save()
            if step==321 and args.state_assisted_grasp and args.steps>320 and not args.no_turn_lift_diagnostic:
                from scripted_grasp import held_patch_geometry
                actual_cloth=scene.object_states(emb._env,names)['tshirt'].nodes
                robot=base.scene['left_robot']
                pose=arr(robot.data.body_link_pose_w)[0,robot.body_names.index('gripper')]
                geometry=held_patch_geometry(arena_cloth.SPECS['tshirt'].mesh.rest,actual_cloth,pose)
                report['pickup_retention_guard']=geometry
                if geometry['plainly_lost']:
                    raise RuntimeError('Hem has left the hand after pickup; later peg actions withheld')
            if fixed_actions is not None:
                reference_action=fixed_actions[step-1].copy()
                observed=np.asarray(emb._observation().state['joint_pos'],float)
                if pinch is not None and step>=141:
                    cloth_nodes=arena_cloth.state(emb._env,'tshirt').nodes
                    for index,side in enumerate(('left','right')):
                        robot=base.scene[side+'_robot']
                        poses=dict(zip(robot.body_names,arr(robot.data.body_link_pose_w)[0]))
                        joint=7*index+6
                        releasing=side=='right' and step>260
                        reference_action[joint]=pinch.action(side,reference_action[joint],observed[joint],
                            cloth_nodes,poses,step,release=releasing)
                    report['contact_pinch_feedback']=pinch.records
                for peg_name,clip_servo in clip_servos:
                    from arena_props import peg_state
                    body_poses=peg_state(emb._env,peg_name).body_poses if clip_servo.start<step<=clip_servo.end+1 else None
                    reference_action=clip_servo.action(step,reference_action,observed,body_poses)
                    report.setdefault('peg_alignments',{})[peg_name]=clip_servo.records
                requested_action=tracking.action(reference_action,observed,enabled=args.joint_feedback_from is not None and step>args.joint_feedback_from)
                bounded=np.clip(requested_action,emb._low,emb._high)
                active_action=active_action+np.clip(bounded-active_action,-delta_limit,delta_limit)
                command=torch.as_tensor(active_action[None,:],device=base.device,dtype=torch.float32)
            elif dream is not None:
                chunk=dream.act(emb._observation())
                if len(chunk.actions)!=1:raise RuntimeError('Expected one ordinary Dream action')
                reference_action=np.asarray(chunk.actions[0].data,float)
                if reference_action.shape!=(14,) or not np.isfinite(reference_action).all():raise ValueError('Invalid Dream joint action')
                requested_action=reference_action.copy();bounded=np.clip(requested_action,emb._low,emb._high)
                active_action=active_action+np.clip(bounded-active_action,-delta_limit,delta_limit)
                command=torch.as_tensor(active_action[None,:],device=base.device,dtype=torch.float32)
            emb._env.step(command)
            emb._t=step/CONTROL_HZ
            snapshot(step)
            report['completed_steps']=step
            if step%20==0:
                save()
        if args.judging_views:
            issues=report.get('judging_frame_warnings',[])
            report['judging_recording_quality']=dict(flagged_frames=len(issues),
                final_frame_flagged=any(x['step']==args.steps for x in issues),
                requires_visual_review=True,
                note='All returned native frames are retained. Optional camera occlusion does not alter physics or imply task success; final scoring still requires meaningful visible evidence.')
        report['status']='complete'
        report['meaning']='Bounded diagnostic execution completed; no task success or contact fidelity certified'
    except BaseException as exc:
        report.update(status='interrupted' if isinstance(exc,SystemExit) else 'failed',error_type=type(exc).__name__,error_message=str(exc))
        raise
    finally:
        # Preserve status even if an interrupted encoder cannot finish closing.
        save()
        if dense_trace is not None:
            dense_trace.flush()
            report['recorded_cloth_frames']=dense_trace.count
        if trace is not None:trace.close()
        if video is not None:video.close()
        for recorder in judging_videos.values():recorder.close()
        if dream is not None:
            from dream_policy import public_transcript
            (args.out/'dream-transcript.json').write_text(json.dumps(public_transcript(dream),indent=2)+'\n')
        report.update(dream_requests=len(request_audit),dream_session_audit=session_audit)
        if request_audit:(args.out/'dream-requests.json').write_text(json.dumps(request_audit,indent=2)+'\n')
        save()
        if app is not None:app.close()
        signal.signal(signal.SIGTERM,previous_sigterm)


if __name__=='__main__':main()
