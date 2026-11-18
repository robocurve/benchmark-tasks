"""Pinned local task launcher. --config-only never starts a model or simulator."""
import argparse,hashlib,json,os,subprocess,sys,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parent
MAX_STEPS=2400

def cli_args(mode,steps,out,instruction):
    if mode not in ('setup','demo','dream') or type(steps) is not int or not 1<=steps<=MAX_STEPS:raise ValueError('Invalid mode/budget')
    a=['inspect-robots','run','--embodiment','yam_arena','--policy','dream' if mode=='dream' else 'yam_scripted','-E',f'task={ROOT/"task.py"}','-E','scene=station','-E','object_jitter=0','-E','headless=true','-E','control_hz=20','-E','cam_width=320','-E','cam_height=176','--instruction',instruction,'--scorer','task_score','--max-steps',str(steps),'--epochs','1','--fail-on-error','1','--store-frames','--no-prompt','--log-dir',str(out)]
    if mode=='dream':a+=['-P','model=dreamlabs/dream-1-beta','-P','record=false','-P','dry_run=false','-P','rows_per_request=48','-P','preroll_steps=32','-P','timeout_s=300']
    else:a+=['-P',f'hold_s={steps/20+5}']
    return a

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--mode',choices=['setup','demo','dream'],required=True);p.add_argument('--log-dir',type=Path,required=True);p.add_argument('--steps',type=int);p.add_argument('--gpu',choices=['auto','0','1'],default='auto');p.add_argument('--config-only',action='store_true');p.add_argument('--plan',type=Path)
    a=p.parse_args();cfg=json.loads((ROOT/'scene.json').read_text());steps=a.steps or (40 if a.mode=='setup' else cfg['demo_actions'] if a.mode=='demo' else MAX_STEPS);out=a.log_dir.resolve()
    if out.is_relative_to(ROOT) or (out.exists() and any(out.iterdir())):p.error('Use a new output directory outside task source')
    if a.plan and a.mode!='demo':p.error('Plans are allowed only in scripted demos')
    argv=cli_args(a.mode,steps,out,cfg['instruction'])
    os.environ.update(OMNI_KIT_ACCEPT_EULA='YES',ACCEPT_EULA='Y',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    from runtime_check import check
    contract=check();plan=None
    if a.mode in ('setup','demo'):
        import numpy as np
        from yam_arena import ik
        from yam_arena.constants import READY
        plan=json.loads((a.plan or ROOT/'demo-plan.json').read_text())['steps'] if a.mode=='demo' else [dict(label='initial state',action=list(READY)+[1.]+list(READY)+[1.])]*steps
        q=np.asarray([r['action'] for r in plan]);assert q.ndim==2 and q.shape[1]==14 and np.isfinite(q).all()
        for off in [0,7]:
            assert np.all(q[:,off:off+6]>=ik.LOWER) and np.all(q[:,off:off+6]<=ik.UPPER)
            assert np.all((q[:,off+6]>=0)&(q[:,off+6]<=1))
    out.mkdir(parents=True,exist_ok=True)
    source={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in ROOT.rglob('*') if f.is_file() and not any(x in f.parts for x in ['__pycache__','.pytest_cache'])}
    (out/'source-manifest.json').write_text(json.dumps(source,indent=2)+'\n')
    with zipfile.ZipFile(out/'source.zip','w',zipfile.ZIP_DEFLATED) as z:
        for n in source:z.write(ROOT/n,n)
    config=dict(argv=argv,mode=a.mode,paid=a.mode=='dream',config_only=a.config_only,steps=steps,control_hz=20,physics_hz=cfg['physics_hz'],controller_installed=plan is not None,canonical_instruction=cfg['instruction'],film_response=cfg['kind'],body_iterations=cfg['body_iterations'],camera_resolution=[320,176],extra_cloud_recording=False)
    (out/'configuration.json').write_text(json.dumps(config,indent=2)+'\n');(out/'runtime-contract.json').write_text(json.dumps(contract,indent=2)+'\n')
    if plan is not None:(out/'plan.json').write_text(json.dumps(dict(steps=plan),indent=2)+'\n')
    if a.config_only:return 0
    raw=subprocess.check_output(['nvidia-smi','--query-gpu=index,uuid,memory.free','--format=csv,noheader,nounits'],text=True,timeout=10);cards=[(i.strip(),u.strip(),int(m)) for i,u,m in (l.split(',') for l in raw.splitlines())];cards=[c for c in cards if a.gpu=='auto' or c[0]==a.gpu];gpu=max(cards,key=lambda c:c[2])
    if gpu[2]<10000:raise RuntimeError('Wait for 10,000 MiB free GPU memory; no other process is stopped')
    os.environ['CUDA_VISIBLE_DEVICES']=gpu[1];(out/'gpu-at-launch.json').write_text(json.dumps(dict(index=gpu[0],free_mib=gpu[2]))+'\n')
    import surface_runtime as runtime,geometry
    runtime.PHYSICS_HZ=cfg['physics_hz'];runtime.BODY_ITERATIONS=tuple(cfg['body_iterations']);runtime.SPECULATIVE_CCD=cfg['speculative_ccd']
    for k,v in cfg['flags'].items():setattr(geometry,k,v)
    if cfg['kind']=='stencil_paste':
        from stencil_paste_model import StencilPaste,PasteDisplay
        geometry.film=StencilPaste(geometry.DATA);display=PasteDisplay
    else:
        from surface_material_models import WetFilm,SpreadFilm,FilmDisplay
        geometry.film=WetFilm() if cfg['kind']=='wet_mount' else SpreadFilm();display=FilmDisplay
    build0=geometry.build
    def build(stage):
        build0(stage);geometry.film_display=display(stage,runtime.WORLD+'/finite_liquid',geometry.film);geometry.film_display.update()
    geometry.build=build
    save=runtime.install(geometry,out,plan)
    from image_health import install as install_images
    save_images=install_images(out)
    sys.argv=argv
    try:
        from inspect_robots.cli import main as cli
        try:cli()
        except SystemExit as exc:
            if exc.code not in (None,0):raise
    finally:save();save_images()
    logs=[f for f in out.glob('adhoc*.json') if '.live.' not in f.name]
    if len(logs)!=1:raise RuntimeError('Missing final official result')
    r=json.loads(logs[0].read_text())
    if r['status']!='success' or r['results'].get('errored_trials'):raise RuntimeError('Official run did not complete successfully')
    return 0

if __name__=='__main__':raise SystemExit(main())
