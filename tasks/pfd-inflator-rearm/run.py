"""Run the task in the official walkthrough environment."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from zipfile import ZipFile, ZIP_DEFLATED

ROOT = Path(__file__).resolve().parent
BUDGET = json.loads((ROOT/'budget.json').read_text())['max_actions']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('demo','dream','setup'), required=True)
    parser.add_argument('--log-dir', type=Path, required=True)
    parser.add_argument('--gpu', choices=('auto','0','1'), default='auto')
    parser.add_argument('--stock-order', choices=('intact-first','pierced-first'), default='intact-first',
                        help='Identity of the cartridge at stand A. Visible to the scene builder only.')
    parser.add_argument('--config-only', action='store_true')
    args = parser.parse_args()
    out = args.log_dir.resolve()
    if out.is_relative_to(ROOT):
        parser.error('Select a new output directory outside the task folder.')
    out.mkdir(parents=True,exist_ok=False)
    os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    from runtime_check import check
    (out/'runtime-contract.json').write_text(json.dumps(check(),indent=2))
    from yam_arena.constants import READY
    instruction=json.loads((ROOT/'task.json').read_text())['instruction']
    steps=40 if args.mode=='setup' else BUDGET
    plan=None
    if args.mode=='demo':
        import numpy as np
        with np.load(ROOT/'demo.npz',allow_pickle=False) as d:
            steps=len(d['action'])
            if not 0<steps<=BUDGET:
                raise ValueError('Demo exceeds action limit')
            plan=[]
            for action,alternate,choose,view in zip(d['action'],d['alternate'],d['choose'],d['view'],strict=True):
                row=dict(action=action)
                if choose: row['alternate_action']=alternate
                if view: row['stock_view']=str(view)
                plan.append(row)
    elif args.mode=='setup':
        plan=[dict(action=[*READY,1.,*READY,1.])]*steps
    policy='dream' if args.mode=='dream' else 'yam_scripted'
    argv=['inspect-robots','run','--embodiment','yam_arena','--policy',policy,
          '-E',f'task={ROOT/"task.py"}','-E','scene=station','-E','object_jitter=0',
          '-E','headless=true','-E','control_hz=20','-E','cam_width=320','-E','cam_height=176',
          '--instruction',instruction,'--scorer','task_score','--max-steps',str(steps),'--epochs','1',
          '--fail-on-error','1','--store-frames','--no-prompt','--log-dir',str(out)]
    if args.mode=='dream':
        argv+=['-P','model=dreamlabs/dream-1-beta','-P','record=false','-P','dry_run=false',
               '-P','rows_per_request=48','-P','preroll_steps=32','-P','timeout_s=300']
    else:
        argv+=['-P',f'hold_s={steps/20+5}']
    config=dict(mode=args.mode,stock_order=args.stock_order,steps=steps,canonical_instruction=instruction,argv=argv,
                config_only=args.config_only,controller_installed=plan is not None,physics_hz=240,control_hz=20)
    (out/'configuration.json').write_text(json.dumps(config,indent=2))
    with ZipFile(out/'source.zip','w',ZIP_DEFLATED) as archive:
        manifest={}
        for f in sorted(ROOT.iterdir()):
            if f.is_file() and f.suffix in ('.py','.json','.md','.scad','.svg','.txt','.npz'):
                archive.write(f,f.name)
                manifest[f.name]=hashlib.sha256(f.read_bytes()).hexdigest()
    (out/'source-manifest.json').write_text(json.dumps(manifest,indent=2))
    if args.config_only:
        return
    cards=subprocess.check_output(['nvidia-smi','--query-gpu=index,uuid,memory.free','--format=csv,noheader,nounits'],text=True,timeout=15)
    eligible=[(int(free),uid.strip(),index.strip()) for index,uid,free in (s.split(',') for s in cards.splitlines())
              if args.gpu=='auto' or index.strip()==args.gpu]
    free,uid,index=max(eligible)
    if free < 6500:
        raise RuntimeError('Insufficient GPU memory for the renderer.')
    os.environ.update(CUDA_VISIBLE_DEVICES=uid,OMNI_KIT_ACCEPT_EULA='YES',ACCEPT_EULA='Y',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    (out/'gpu-at-launch.json').write_text(json.dumps(dict(index=index,free_mib=free)))
    import probe_geometry as geometry
    from surface_process_probe import install
    from image_health import install as guard_cameras
    save=install(geometry,out,plan,args.stock_order)
    save_images=guard_cameras(out)
    sys.argv=argv
    try:
        from inspect_robots.cli import main as cli
        try:
            cli()
        except SystemExit as exc:
            if exc.code not in (0,None):
                raise
    finally:
        save()
        save_images()
    final,=[f for f in out.glob('adhoc*.json') if '.live.' not in f.name]
    result=json.loads(final.read_text())
    if result['status']!='success' or result['results'].get('errored_trials'):
        raise RuntimeError('Inspect did not finish the episode successfully.')


if __name__=='__main__':
    main()
