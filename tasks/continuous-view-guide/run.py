"""Portable pinned-runtime entry point. Default modes are free; Dream is paid."""
import argparse,json,os,signal,subprocess,sys
from pathlib import Path
from launch import MODES,cli_args
ROOT=Path(__file__).resolve().parent

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',choices=MODES,required=True)
    p.add_argument('--steps',type=int)
    p.add_argument('--log-dir',required=True,type=Path)
    p.add_argument('--config-only',action='store_true')
    p.add_argument('--gpu',choices=('auto','0','1'),help='Choose a physical GPU; auto requires at least8GiB free. Default preserves CUDA_VISIBLE_DEVICES or GPU0.')
    a=p.parse_args();steps=a.steps if a.steps is not None else {'setup':40,'lift_probe':300,'negative':500,'fixed_view':430}.get(a.mode,2400)
    try:cli_args(a.mode,steps,a.log_dir)
    except ValueError as exc:p.error(str(exc))
    out=a.log_dir.resolve()
    if out.exists() and any(out.iterdir()):p.error('Use a new empty log directory; preserve previous evidence')
    out.mkdir(parents=True,exist_ok=True)
    from runtime_check import check
    (out/'runtime_contract.json').write_text(json.dumps(check(),indent=2))
    command=[sys.executable,'-u',str(ROOT/'launch.py'),'--mode',a.mode,'--steps',str(steps),'--log-dir',str(out)]
    if a.config_only:return subprocess.call(command+['--config-only'])
    environment=dict(os.environ,OMNI_KIT_ACCEPT_EULA='YES',ACCEPT_EULA='Y')
    if a.gpu is not None:
        raw=subprocess.check_output(['nvidia-smi','--query-gpu=index,memory.free','--format=csv,noheader,nounits'],text=True,timeout=10)
        free={int(row.split(',')[0]):int(row.split(',')[1]) for row in raw.splitlines()}
        selected=max(free,key=free.get) if a.gpu=='auto' else int(a.gpu)
        if free.get(selected,0)<8192:p.error('Selected GPU has less than8GiB free; wait for capacity')
        environment['CUDA_VISIBLE_DEVICES']=str(selected)
        (out/'gpu_selection.json').write_text(json.dumps(dict(selected=selected,free_mib=free,mode=a.gpu),indent=2)+'\n')
    else:environment.setdefault('CUDA_VISIBLE_DEVICES','0')
    environment.setdefault('OMP_NUM_THREADS','1');environment.setdefault('OPENBLAS_NUM_THREADS','1')
    limit=2400 if a.mode=='dream' else 900
    print(f'{a.mode}: {steps} actions, wall cap{limit}s; logs:{out}',flush=True)
    with (out/'console.log').open('w') as log:
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=environment,start_new_session=True)
        try:code=process.wait(timeout=limit)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait()
            code=124
    (out/'process.exit').write_text(str(code)+'\n')
    checked=subprocess.call([sys.executable,str(ROOT/'validate_run.py'),str(out)])
    return 0 if code==0 and checked==0 else 1

if __name__=='__main__':raise SystemExit(main())
