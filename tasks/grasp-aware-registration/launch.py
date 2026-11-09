"""Original-rig launcher. Dream is paid; config-only never creates a client."""
import argparse,hashlib,json,sys,zipfile
from pathlib import Path
from prototype import layout as L
from prototype.scoring import SCORING_REVISION
ROOT=Path(__file__).resolve().parent
MODES=('setup','view','central','end','end_visual','regrasp_visual','adaptive_visual','dream')
DREAM_MODEL='dreamlabs/dream-1-beta'
MAX_STEPS=2400

def cli_args(mode,steps,output):
    if mode not in MODES:raise ValueError('unknown mode')
    if type(steps) is not int or not 1<=steps<=MAX_STEPS:raise ValueError('budget must be1..2400 actions')
    argv=['inspect-robots','run','--embodiment','yam_arena','--policy','dream' if mode=='dream' else 'yam_scripted',
          '-E',f'task={ROOT/"task.py"}','-E','scene=station','-E','object_jitter=0','-E','headless=true',
          '-E','control_hz=20','-E','cam_width=320','-E','cam_height=176',
          '--instruction',L.INSTRUCTION,'--scorer','task_score','--max-steps',str(steps),
          '--epochs','1','--fail-on-error','1','--store-frames','--no-prompt','--log-dir',str(output)]
    if mode=='dream':
        argv+=['-P',f'model={DREAM_MODEL}','-P','record=false','-P','dry_run=false',
               '-P','rows_per_request=48','-P','preroll_steps=32','-P','timeout_s=300']
    else:argv+=['-P',f'hold_s={steps/20+5}']
    return argv

def main():
    p=argparse.ArgumentParser();p.add_argument('--mode',choices=MODES,required=True)
    p.add_argument('--initial-grasp',choices=['central','end'],default='central')
    p.add_argument('--steps',type=int,default=600);p.add_argument('--log-dir',type=Path,required=True)
    p.add_argument('--observer-json',type=Path);p.add_argument('--config-only',action='store_true');a=p.parse_args()
    try:argv=cli_args(a.mode,a.steps,a.log_dir)
    except ValueError as exc:p.error(str(exc))
    if a.log_dir.exists() and any(a.log_dir.iterdir()):p.error('Use a new empty output directory')
    if a.mode not in ('setup','dream') and not a.observer_json:p.error('A checked observer pose JSON is required')
    if a.mode=='dream' and a.observer_json:p.error('Dream does not receive a scripted observer pose')
    observer=json.loads(a.observer_json.read_text())['q'] if a.observer_json else None
    a.log_dir.mkdir(parents=True,exist_ok=True)
    source={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in ROOT.rglob('*') if f.is_file() and f.suffix in ['.py','.md','.json'] and '__pycache__' not in f.parts and '.pytest_cache' not in f.parts}
    (a.log_dir/'source_manifest.json').write_text(json.dumps(source,indent=2)+'\n')
    with zipfile.ZipFile(a.log_dir/'task-source-at-run.zip','w',zipfile.ZIP_DEFLATED) as z:
        for n in source:z.write(ROOT/n,n)
    initial_grasp=(a.initial_grasp if a.mode=='adaptive_visual' else 'central' if a.mode in ('central','regrasp_visual') else 'end' if a.mode in ('end','end_visual') else None)
    (a.log_dir/'configuration.json').write_text(json.dumps(dict(scene_revision=L.SCENE_REVISION,scoring_revision=SCORING_REVISION,mode=a.mode,initial_grasp=initial_grasp,steps=a.steps,argv=argv,observer_q=observer,paid=a.mode=='dream',prototype=True,config_only=a.config_only,controller_installed=a.mode!='dream',canonical_instruction=L.INSTRUCTION,image_integrity_guard=True,control_hz=20,physics_hz=240,cameras=[320,176]),indent=2)+'\n')
    from runtime_check import check
    (a.log_dir/'runtime_contract.json').write_text(json.dumps(check(),indent=2)+'\n')
    if a.config_only:return 0
    from prototype.image_health import install as install_image_health
    save_images=install_image_health(a.log_dir)
    controller=None
    if a.mode!='dream':
        from prototype import controller
        controller.MODE=a.mode;controller.INITIAL_GRASP=a.initial_grasp;controller.install(a.log_dir,observer)
    from prototype.runtime import install
    save=install(a.log_dir);sys.argv=argv
    try:
        from inspect_robots.cli import main as cli
        return cli()
    finally:
        save();save_images()
        if controller is not None:(a.log_dir/'controller_trace.json').write_text(json.dumps(controller.LOG,indent=2)+'\n')

if __name__=='__main__':raise SystemExit(main())
