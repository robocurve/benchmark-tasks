"""Original-rig launcher. Dream is paid; --config-only never starts a client."""
import argparse,hashlib,json,sys,zipfile
from pathlib import Path
from guide_task import rules

ROOT=Path(__file__).resolve().parent
DREAM_MODEL='dreamlabs/dream-1-beta'
MODES=('setup','stability','demo','negative','fixed_view','right_only','lift_probe','dream')

def cli_args(mode,steps,output):
    if mode not in MODES:raise ValueError('unknown mode')
    if not isinstance(steps,int) or not 1<=steps<=rules.MAX_STEPS:raise ValueError('budget must be1..2400 actions')
    instruction=json.loads((ROOT/'task.json').read_text())['instruction']
    argv=['inspect-robots','run','--embodiment','yam_arena','--policy','dream' if mode=='dream' else 'yam_scripted',
          '-E',f'task={ROOT/"task.py"}','-E','scene=station','-E','object_jitter=0','-E','headless=true',
          '-E','control_hz=20','-E','cam_width=320','-E','cam_height=176','--instruction',instruction,
          '--scorer','task_score','--max-steps',str(steps),'--epochs','1','--fail-on-error','1',
          '--store-frames','--no-prompt','--log-dir',str(output)]
    if mode=='dream':
        argv+=['-P',f'model={DREAM_MODEL}','-P','record=false','-P','dry_run=false',
               '-P','rows_per_request=48','-P','preroll_steps=32','-P','timeout_s=300']
    else:argv+=['-P',f'hold_s={steps/20+5}']
    return argv

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',choices=MODES,default='setup')
    p.add_argument('--steps',type=int,default=rules.MAX_STEPS)
    p.add_argument('--log-dir',required=True,type=Path)
    p.add_argument('--config-only',action='store_true')
    a=p.parse_args()
    try:argv=cli_args(a.mode,a.steps,a.log_dir)
    except ValueError as exc:p.error(str(exc))
    a.log_dir.mkdir(parents=True,exist_ok=True)
    files={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in ROOT.rglob('*')
           if f.is_file() and f.suffix in ('.py','.json','.md','.txt','.sh') and '__pycache__' not in f.parts}
    (a.log_dir/'source_manifest.json').write_text(json.dumps(files,indent=2))
    with zipfile.ZipFile(a.log_dir/'task-source-at-run.zip','w',zipfile.ZIP_DEFLATED) as z:
        for relative in files:z.write(ROOT/relative,relative)
    instruction=argv[argv.index('--instruction')+1]
    (a.log_dir/'configuration.json').write_text(json.dumps(dict(mode=a.mode,steps=a.steps,argv=argv,
        controller='Dream-1' if a.mode=='dream' else 'known-layout scripted route with RGB-gated progress' if a.mode=='demo' else 'scripted diagnostic',
        dream=a.mode=='dream',scoring_version=rules.VERSION,canonical_instruction=instruction,
        controller_installed=a.mode!='dream',config_only=a.config_only,
        demo_record_until_s=49. if a.mode=='demo' else None,
        image_integrity_guard=True,
        physics_hz=240,control_hz=20,cameras=[320,176]),indent=2))
    if a.config_only:return 0
    from guide_task.image_health import install as install_image_health
    save_image_health=install_image_health(a.log_dir)
    controller=None
    if a.mode!='dream':
        from guide_task import controller
        controller.MODE='setup' if a.mode=='stability' else a.mode
        controller.install(a.log_dir)
    from guide_task.runtime import install
    save=install(a.log_dir,record_until_s=49. if a.mode=='demo' else None);sys.argv=argv
    try:
        from inspect_robots.cli import main as cli
        return cli()
    finally:
        save()
        save_image_health()
        if controller is not None:(a.log_dir/'controller_trace.json').write_text(json.dumps(controller.LOG,indent=2))

if __name__=='__main__':raise SystemExit(main())
