"""Scheduled shared task runner: scripted, offline interface check, or live Dream-1.

Live inference is explicit and requires a passed local interface check, full
scripted demonstration and a create-once single-attempt ledger. No media uploads.
"""
import argparse
import hashlib
import importlib.metadata
import json
import linecache
import os
from pathlib import Path
import shutil
import sys
import time
import traceback

ROOT=Path(__file__).resolve().parent
# Keep traceback lines aligned with the loaded runner while later versions are
# developed in the shared task folder; controllers execute the per-run snapshot.
_entry_source=Path(__file__).read_text()
linecache.cache[__file__]=(len(_entry_source),None,_entry_source.splitlines(True),__file__)


def compatible_gate(gate_path,spec):
    """Budget and public-transport revisions leave the strict G1 test unchanged."""
    if not gate_path:return False
    gate_path=Path(gate_path)
    gate=json.loads(gate_path.read_text())
    prior=json.loads((gate_path.parent/'spec-used.json').read_text())
    def normalize(value):
        value=json.loads(json.dumps(value))
        for key in ('max_steps','one_pad_max_steps'):value['control'].pop(key,None)
        # These fields govern only the four-pad public transport witness. Keep
        # every G1 geometry, contact, load, release and drift threshold exact.
        for key in ('transport_foot_center_offset_m','transport_height_tolerance_m',
                    'transport_upright_tolerance_deg'):
            value['score'].pop(key,None)
        return value
    # The saved source spec is byte-identical to the spec hashed at run startup.
    source_path=gate_path.parent/'spec-source.json'
    if not source_path.exists():source_path=gate_path.parent/'task-bundle'/'spec.json'
    source=source_path.read_bytes()
    return bool(gate.get('one_pad_gate_pass') and gate.get('device',{}).get('verified')
                and hashlib.sha256(source).hexdigest()==gate.get('spec_sha256')
                and json.loads(source)==prior and normalize(prior)==normalize(spec))


def digest(directory=ROOT):
    h=hashlib.sha256()
    for p in sorted(directory.iterdir()):
        if p.is_file(): h.update(p.name.encode()+b'\0'+p.read_bytes()+b'\0')
    return h.hexdigest()


def diagnostic_reset(mode,path):
    if mode!='rear-debug':
        if path:raise ValueError('Initialized reset state is restricted to explicitly labeled rear-debug runs')
        return None
    if not path:raise ValueError('rear-debug requires --reset-state from a recorded released two-pad state')
    raw=Path(path).read_bytes();record=json.loads(raw);m=record['measurement']
    import math
    if set(m['pads'])!={f'slider_{i}' for i in range(4)}:
        raise ValueError('Rear diagnostic requires all four pad poses')
    if any(len(p)!=7 or not all(math.isfinite(x) for x in p) for p in [m['furniture'],*m['pads'].values()]):
        raise ValueError('Invalid diagnostic initial pose')
    return raw


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=['preview','contact-debug','one-pad','two-pad','turn',
                                         'continuation-debug','turn-debug','rear-debug','full','lift',
                                         'interface-check','dream'],default='preview')
    parser.add_argument('--max-steps',type=int,default=1200)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--video',action='store_true')
    parser.add_argument('--spec',type=Path)
    parser.add_argument('--width',type=int,default=640)
    parser.add_argument('--height',type=int,default=352)
    parser.add_argument('--seed',type=int,default=0)
    parser.add_argument('--gate-evidence',type=Path,help='Successful one-pad result with the same specification for --full')
    parser.add_argument('--reset-state',type=Path,help='Recorded released state for rear-debug only; never valid for full')
    parser.add_argument('--demo-evidence',type=Path,help='Audited continuous full demonstration for policy modes')
    parser.add_argument('--preflight-evidence',type=Path,help='Passed offline interface check required before live Dream')
    parser.add_argument('--attempt-ledger',type=Path,help='Create-once reservation for one hosted episode; never overwrite')
    args=parser.parse_args()
    policy_mode=args.mode in ('interface-check','dream')
    if policy_mode and (not args.video or not args.demo_evidence):
        raise ValueError('Policy runs require --video and --demo-evidence')
    if args.mode=='dream' and (not args.preflight_evidence or not args.attempt_ledger):
        raise ValueError('Live Dream requires --preflight-evidence and --attempt-ledger')
    reset_source=diagnostic_reset(args.mode,args.reset_state)
    os.environ.pop('FURNITURE_SLIDERS_RESET_STATE',None)
    # No simulator or CUDA imports above this fail-closed reservation check.
    from scheduled_runtime import reservation,native_startup,verify_runtime
    assignment=reservation()
    if not policy_mode and args.mode != 'preview' and os.environ.get('ROBOCURVE_GPU_MODE')!='exclusive':
        raise RuntimeError('Numerical/controller debugging requires scheduler --mode exclusive')
    if args.spec: os.environ['FURNITURE_SLIDERS_SPEC']=str(args.spec.resolve())
    os.environ['FURNITURE_SLIDERS_PAD_COUNT']='1' if args.mode in ('one-pad','contact-debug') else '4'
    os.environ['FURNITURE_SLIDERS_CONTACT_DEBUG']='1' if args.mode=='contact-debug' else '0'
    preinstalled={'contact-debug':1,'continuation-debug':1,'turn-debug':2,'rear-debug':2}.get(args.mode,0)
    os.environ['FURNITURE_SLIDERS_PREINSTALLED_PAD_COUNT']=str(preinstalled)
    # Extra view is for human evidence only; native observations remain the
    # three unchanged stock camera streams in inspect_embodiment.DEFAULT_CAMERAS.
    os.environ['FURNITURE_SLIDERS_OBSERVER']='1' if args.video else '0'
    os.environ['FURNITURE_SLIDERS_OBSERVER_SIZE']=f'{args.width}x{args.height}'
    from config import S,SPEC_PATH,HZ,validation_blockers
    spec_source=SPEC_PATH.read_bytes()
    if json.loads(spec_source)!=S:raise RuntimeError('Specification changed during startup')
    spec_hash=hashlib.sha256(spec_source).hexdigest()
    limit=S['control']['one_pad_max_steps' if args.mode=='one-pad' else 'max_steps']
    if not 0<args.max_steps<=limit: raise ValueError(f'Step limit must be 1..{limit}')
    if args.mode=='dream' and args.max_steps!=S['control']['max_steps']:
        raise ValueError('The live attempt must receive the full declared action budget')
    demo_proof=None
    if policy_mode:
        from dream_control import validate_demo,validate_preflight,source_hashes
        demo_proof=validate_demo(args.demo_evidence,ROOT,S)
        if args.mode=='dream':validate_preflight(args.preflight_evidence,ROOT,S)
    if args.mode in ('two-pad','turn','continuation-debug','turn-debug','rear-debug','full'):
        if not compatible_gate(args.gate_evidence,S):
            raise RuntimeError('Four-pad controller requires a passed one-pad gate with identical physical spec and verified device')
    out=args.out.resolve()
    out.mkdir(parents=True,exist_ok=False)
    emb=None
    policy=None;session=None;interface=None;ctx=None
    writers={}
    began=time.monotonic()
    result={'mode':args.mode,'spec_sha256':spec_hash,'bundle_sha256':digest(),
            'calibration_blockers':validation_blockers(),'physical_validation':False,
            'assignment':assignment,'argv':list(sys.argv),'scheduler_job_id':os.environ['ROBOCURVE_JOB_ID'],
            'scheduler_job_dir':os.environ['ROBOCURVE_JOB_DIR'],'completed':False,'one_pad_gate_pass':False,
            'diagnostic_preinstalled_pads':preinstalled,'full_task_demonstration':False}
    result['recording_only_observer_camera']=bool(args.video)
    if args.video:
        from observer import camera_metadata
        result['evidence_cameras']=camera_metadata()
        result['evidence_cameras'].update(width=args.width,height=args.height)
    if policy_mode:
        result.update(evaluation_source_sha256=source_hashes(ROOT),demo_evidence=demo_proof,
                      requested_hosted_evaluation=args.mode=='dream',scripted_assistance=False,
                      seed=args.seed,declared_action_budget=S['control']['max_steps'])
    if reset_source is not None:
        reset_copy=out/'reset-source.json';reset_copy.write_bytes(reset_source)
        os.environ['FURNITURE_SLIDERS_RESET_STATE']=str(reset_copy)
        result['diagnostic_reset_state_sha256']=hashlib.sha256(reset_source).hexdigest()
    (out/'metadata.json').write_text(json.dumps(result,indent=2)+'\n')
    shutil.copytree(ROOT,out/'task-bundle',ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    runtime_root=out/'task-bundle'
    # Fingerprint the actual frozen files, including the runner source loaded
    # at process startup, even if documentation is edited while admission waits.
    (runtime_root/'run.py').write_text(_entry_source)
    result['bundle_sha256']=digest(runtime_root)
    if policy_mode:result['evaluation_source_sha256']=source_hashes(runtime_root)
    (out/'metadata.json').write_text(json.dumps(result,indent=2)+'\n')
    # Deferred task/controller imports execute the frozen per-run snapshot.
    sys.path.insert(0,str(runtime_root))
    (out/'spec-used.json').write_text(json.dumps(S,indent=2)+'\n')
    (out/'spec-source.json').write_bytes(spec_source)
    try:
        from inspect_robots import Scene
        from inspect_robots.types import Action
        from yam_arena.inspect_embodiment import yam_arena_embodiment
        from yam_arena.constants import READY
        import numpy as np
        import imageio.v2 as imageio
        emb=yam_arena_embodiment(task=str(runtime_root/'task.py'),object_jitter=0,control_hz=HZ,
                                 cam_width=args.width,cam_height=args.height,device='cuda:0')
        with native_startup():
            obs=emb.reset(Scene(id='furniture-sliders',instruction=emb.task.module.__doc__.strip()),seed=args.seed)
        import adapter
        if adapter.monitor.detail['matches'] and not preinstalled:
            raise RuntimeError('Task must reset with all feet bare and the sliders loose')
        # Native reset already advances physics and renders; warm up actual cameras again.
        for _ in range(5): emb._base.sim.render()
        result['device']=verify_runtime(emb,out)
        from native_audit import capture
        native=capture(emb._base)
        (out/'native-robot.json').write_text(json.dumps(native,indent=2)+'\n')
        result['stock_robot_verified']=native['verified_stock_properties']
        result['actual_physics_dt']=emb._base.physics_dt
        result['actual_control_dt']=emb._base.step_dt
        from observer import recording_images
        images=recording_images(emb,obs)
        imageio.imwrite(out/'setup.png',np.concatenate(list(images.values()),axis=1))
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Sdf,Usd
        stage=get_current_stage()
        asset_stage=Usd.Stage.CreateNew(str(out/'assets.usda'))
        for name in ('furniture','floor_panel',*adapter.PAD_NAMES):
            Sdf.CopySpec(stage.GetRootLayer(),'/World/envs/env_0/'+name,asset_stage.GetRootLayer(),'/'+name)
        asset_stage.GetRootLayer().Save()
        if args.video:
            for name,img in images.items():
                writers[name]=imageio.get_writer(out/(name+'.mp4'),fps=HZ,codec='libx264',macro_block_size=1,
                    ffmpeg_params=['-pix_fmt','yuv420p','-movflags','+faststart'])
                writers[name].append_data(img)
        if policy_mode:
            from dream_control import make_policy,policy_observation,action_from_chunk,terminal_reason,InterfaceAudit,integration_hashes
            instruction=emb.task.module.__doc__.strip()
            result['instruction']=instruction
            result['integration_source_sha256']=integration_hashes()
            policy,session=make_policy(out,args.mode=='dream',args.attempt_ledger)
            if 'controller' in sys.modules:raise RuntimeError('Scripted controller must not be loaded during model evaluation')
            policy.bind(emb.info)
            policy.reset(Scene(id='furniture-sliders',instruction=instruction))
            interface=InterfaceAudit(emb,instruction)
            policy_obs=policy_observation(obs,instruction)
            interface.observe(policy_obs,adapter.last_record['diagnostics'])
        elif args.mode not in ('preview','contact-debug'):
            from controller import Context,script
            ctx=Context(emb)
            gen=script(ctx,args.mode)
        previous_stage='reset'
        steps=0
        reason='max_steps' if policy_mode else 'step_limit'
        with (out/'measurements.jsonl').open('w') as log:
            log.write(json.dumps({**adapter.last_record,'stage':'reset','action':None})+'\n')
            for steps in range(1,args.max_steps+1):
                if policy_mode:
                    chunk=policy.act(policy_obs)
                    action,terminal=action_from_chunk(chunk)
                    if terminal is not None:
                        reason=terminal.kind;result['termination_detail']=terminal.reason;steps-=1;break
                elif ctx:
                    try: next(gen)
                    except StopIteration:
                        reason='script_complete'; steps-=1; break
                    action=Action(data=ctx.action().tolist())
                else: action=Action(data=[*READY,1.,*READY,1.])
                final=emb.step(action)
                stage=('dream_preroll' if steps<=policy.preroll_steps else 'dream') if policy_mode else (ctx.stage if ctx else 'preview')
                executed=interface.action(action.data) if policy_mode else list(action.data)
                record={**adapter.last_record,'stage':stage,'action':executed}
                if policy_mode:
                    record.update(policy_action=np.asarray(action.data).tolist(),policy_observation_step=steps-1,
                                  inference_latency_s=chunk.inference_latency_s)
                log.write(json.dumps(record)+'\n')
                obs=final.observation
                if policy_mode:
                    policy_obs=policy_observation(obs,instruction)
                    interface.observe(policy_obs,adapter.last_record['diagnostics'])
                images=recording_images(emb,obs)
                if record['stage']!=previous_stage:
                    imageio.imwrite(out/(f'{steps:04d}-'+record['stage']+'.png'),np.concatenate(list(images.values()),axis=1))
                    previous_stage=record['stage']
                for name,writer in writers.items(): writer.append_data(images[name])
                if steps%20==0:
                    log.flush()
                    print(json.dumps({'step':steps,'stage':record['stage'],
                                      **{k:v for k,v in adapter.monitor.detail.items() if k!='foot_corners'}}),flush=True)
                if policy_mode:
                    stop=terminal_reason(final,action)
                    if stop:
                        reason=stop
                        if action.terminal:result['termination_detail']=action.terminal.reason
                        break
                elif final.terminated:
                    reason=final.termination_reason; break
        imageio.imwrite(out/'final.png',np.concatenate(list(images.values()),axis=1))
        result.update(completed=True,steps=steps,termination_reason=reason,scoring=adapter.monitor.detail,
                      one_pad_gate_pass=bool(adapter.monitor.gate_pass and args.mode=='one-pad'),
                      full_task_demonstration=bool(args.mode=='full' and adapter.monitor.success),
                      physical_validation=not validation_blockers())
        if policy_mode:
            result['interface_validation']=interface.summary(session,offline=args.mode=='interface-check')
            if args.mode=='interface-check' and not result['interface_validation']['passed']:
                raise RuntimeError('Offline interface check did not exercise state/action/clock contract')
    except BaseException:
        result['completed']=False
        if policy_mode:result['termination_reason']='policy_or_interface_error'
        result['traceback']=traceback.format_exc()
        if 'adapter' in locals():
            result.update(scoring=adapter.monitor.detail,
                          steps=adapter.monitor.previous['step'] if adapter.monitor.previous else 0,
                          failed_stage=ctx.stage if 'ctx' in locals() and ctx else ('policy' if policy_mode and policy is not None else 'startup'))
        if 'obs' in locals():
            failure_images=images if 'images' in locals() else obs.images
            imageio.imwrite(out/'failure.png',np.concatenate(list(failure_images.values()),axis=1))
        (out/'failure.json').write_text(json.dumps(result,indent=2)+'\n')
        traceback.print_exc()
        raise
    finally:
        if policy is not None:
            result['policy_transcript']=policy.transcript()
            (out/'policy-transcript.json').write_text(json.dumps(result['policy_transcript'],indent=2)+'\n')
            result['hosted_inference_requests']=session.requests if session.hosted else 0
            result['hosted_inference_replies']=session.replies if session.hosted else 0
            result['genuine_hosted_evaluation']=bool(session.hosted and session.replies)
            try:policy.close()
            except Exception as error:result['policy_close_error']=type(error).__name__
        if policy_mode:result['scripted_controller_loaded']='controller' in sys.modules
        for writer in writers.values(): writer.close()
        if 'ctx' in locals() and ctx: result['controller_events']=ctx.events
        result['wall_seconds']=time.monotonic()-began
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print('RESULT '+json.dumps(result),flush=True)
        if emb: emb.close()


if __name__=='__main__': main()
