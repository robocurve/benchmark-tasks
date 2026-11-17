"""Pinned original-rig launcher. Config-only never starts a simulator or model client."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import zipfile

ROOT=Path(__file__).resolve().parent
MAX_STEPS=2400
DREAM_MODEL='dreamlabs/dream-1-beta'
MODES=('setup','demo','dream')


def cli_args(mode,steps,output):
    if mode not in MODES:raise ValueError('Unknown mode')
    if type(steps) is not int or not 1<=steps<=MAX_STEPS:raise ValueError('Budget must be 1..2400 actions')
    from assembly.instruction import INSTRUCTION
    argv=['inspect-robots','run','--embodiment','yam_arena','--policy','dream' if mode=='dream' else 'yam_scripted',
          '-E',f'task={ROOT/"task.py"}','-E','scene=station','-E','object_jitter=0','-E','headless=true',
          '-E','control_hz=20','-E','cam_width=320','-E','cam_height=176',
          '--instruction',INSTRUCTION,'--scorer','task_score','--max-steps',str(steps),
          '--epochs','1','--fail-on-error','1','--store-frames','--no-prompt','--log-dir',str(output)]
    if mode=='dream':
        argv+=['-P',f'model={DREAM_MODEL}','-P','record=false','-P','dry_run=false',
               '-P','rows_per_request=48','-P','preroll_steps=32','-P','timeout_s=300']
    else:argv+=['-P',f'hold_s={steps/20+5}']
    return argv


def select_gpu(choice, minimum_free_mib=8192):
    raw=subprocess.check_output(['nvidia-smi','--query-gpu=index,uuid,memory.free',
                                 '--format=csv,noheader,nounits'],text=True,timeout=10)
    cards=[dict(index=i.strip(),uuid=u.strip(),free_mib=int(m)) for i,u,m in
           (line.split(',') for line in raw.splitlines())]
    candidates=cards if choice=='auto' else [c for c in cards if c['index']==choice]
    if not candidates:raise RuntimeError('Selected GPU is unavailable')
    gpu=max(candidates,key=lambda c:c['free_mib'])
    if gpu['free_mib']<minimum_free_mib:raise RuntimeError('Insufficient free GPU memory; wait for capacity. No process was stopped.')
    os.environ['CUDA_VISIBLE_DEVICES']=gpu['uuid']
    return gpu


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',choices=MODES,required=True)
    p.add_argument('--log-dir',type=Path,required=True)
    p.add_argument('--steps',type=int,default=MAX_STEPS)
    p.add_argument('--plan',type=Path)
    p.add_argument('--gpu',choices=('auto','0','1'),default='auto')
    p.add_argument('--config-only',action='store_true')
    p.add_argument('--min-free-mib',type=int,default=8192)
    a=p.parse_args();out=a.log_dir.resolve()
    if out.is_relative_to(ROOT):p.error('Keep run evidence outside the task source directory')
    if out.exists() and any(out.iterdir()):p.error('Use a new empty output directory; preserve old evidence')
    if a.mode in ('dream','setup') and a.plan:p.error('This mode accepts no scripted plan')
    os.environ.update(OMNI_KIT_ACCEPT_EULA='YES',ACCEPT_EULA='Y',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    try:argv=cli_args(a.mode,a.steps,out)
    except ValueError as exc:p.error(str(exc))
    from assembly import geometry,scoring
    from assembly.instruction import INSTRUCTION
    # Score the ending state over a fixed budget in every mode. No demo-only
    # success delay or controller logic is present on the Dream path.
    scoring.AUTO_TERMINATE=False
    plan=None
    if a.mode=='demo':
        path=a.plan or ROOT/'demo-plan.json'
        plan=json.loads(path.read_text())
        import numpy as np
        from yam_arena import ik
        actions=np.asarray([r['action'] for r in plan['steps']],dtype=float)
        if actions.ndim!=2 or actions.shape[1]!=14 or not np.isfinite(actions).all():p.error('Invalid 14D plan')
        for offset in (0,7):
            if np.any(actions[:,offset:offset+6]<ik.LOWER) or np.any(actions[:,offset:offset+6]>ik.UPPER):p.error('Joint limit violation')
            if np.any(actions[:,offset+6]<0) or np.any(actions[:,offset+6]>1):p.error('Invalid gripper command')
    out.mkdir(parents=True,exist_ok=True)
    source={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in ROOT.rglob('*')
            if f.is_file() and f.suffix in ('.py','.md','.json') and not any(
                s in f.parts for s in ('__pycache__','.pytest_cache'))}
    (out/'source_manifest.json').write_text(json.dumps(source,indent=2)+'\n')
    with zipfile.ZipFile(out/'source.zip','w',zipfile.ZIP_DEFLATED) as z:
        for name in source:z.write(ROOT/name,name)
    if plan is not None:(out/'action-plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    config=dict(argv=argv,mode=a.mode,steps=a.steps,paid=a.mode=='dream',config_only=a.config_only,
                physics_only_diagnostic=False,scoring_implemented=True,continue_after_success=True,
                canonical_instruction=INSTRUCTION,controller_installed=a.mode=='demo',
                scripted_plan_supplied=plan is not None,control_hz=20,physics_hz=geometry.PHYSICS_HZ,camera_resolution=[320,176],
                observations=['top_cam RGB','left_cam RGB','right_cam RGB','joint_pos'],
                image_integrity_guard=True,extra_cloud_recording=False,
                purpose='Dream evaluation' if a.mode=='dream' else 'Free known-layout feasibility or setup')
    (out/'configuration.json').write_text(json.dumps(config,indent=2)+'\n')
    from runtime_check import check
    (out/'runtime_contract.json').write_text(json.dumps(check(),indent=2)+'\n')
    if a.config_only:return 0
    if a.min_free_mib<6144:p.error('Use at least a 6GiB launch floor')
    gpu=select_gpu(a.gpu,a.min_free_mib);(out/'gpu-at-launch.json').write_text(json.dumps(gpu,indent=2)+'\n')
    from assembly.image_health import install as install_images
    from assembly.runtime import install
    save_images=install_images(out);save=install(out,allow_demo_evidence=a.mode=='demo')
    if a.mode=='demo':
        from assembly.controller import install as install_controller
        install_controller(plan['steps'])
    sys.argv=argv
    try:
        from inspect_robots.cli import main as cli
        return cli()
    finally:
        save();save_images()


if __name__=='__main__':raise SystemExit(main())
