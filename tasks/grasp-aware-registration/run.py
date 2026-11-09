"""Portable entry point; free modes by default, explicit Dream mode is paid."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
from launch import MODES, cli_args

ROOT=Path(__file__).resolve().parent

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',choices=MODES,required=True)
    p.add_argument('--initial-grasp',choices=('central','end'),default='central')
    p.add_argument('--steps',type=int)
    p.add_argument('--log-dir',type=Path,required=True)
    p.add_argument('--gpu',choices=('auto','0','1'),default='auto')
    p.add_argument('--config-only',action='store_true')
    a=p.parse_args()
    steps=a.steps if a.steps is not None else {'setup':40,'view':120}.get(a.mode,2400)
    out=a.log_dir.resolve()
    try:cli_args(a.mode,steps,out)
    except ValueError as exc:p.error(str(exc))
    if out.exists() and any(out.iterdir()):p.error('Use a new empty log directory; preserve old evidence')
    command=[sys.executable,'-u',str(ROOT/'launch.py'),'--mode',a.mode,'--steps',str(steps),
             '--log-dir',str(out),'--initial-grasp',a.initial_grasp]
    if a.mode not in ('setup','dream'):
        observer='central' if a.mode=='central' else 'end'
        command+=['--observer-json',str(ROOT/'observers'/f'{observer}.json')]
    if a.config_only:return subprocess.call(command+['--config-only'])
    raw=subprocess.check_output(['nvidia-smi','--query-gpu=index,memory.free','--format=csv,noheader,nounits'],text=True,timeout=10)
    free={int(row.split(',')[0]):int(row.split(',')[1]) for row in raw.splitlines()}
    gpu=max(free,key=free.get) if a.gpu=='auto' else int(a.gpu)
    if free.get(gpu,0)<8192:p.error('Selected GPU has less than8GiB free; wait for capacity')
    environment=dict(os.environ,CUDA_VISIBLE_DEVICES=str(gpu),OMNI_KIT_ACCEPT_EULA='YES',ACCEPT_EULA='Y',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
    out.parent.mkdir(parents=True,exist_ok=True)
    print(f'{a.mode}: {steps} actions; GPU{gpu}; evidence:{out}',flush=True)
    with out.with_suffix('.console.log').open('w') as output:
        process=subprocess.Popen(command,stdout=output,stderr=subprocess.STDOUT,env=environment,start_new_session=True)
        try:code=process.wait(timeout=2400 if a.mode=='dream' else 900)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=30)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
            code=124
    if not out.exists():return 1
    (out/'process.exit').write_text(str(code)+'\n')
    (out/'gpu_selection.json').write_text(json.dumps(dict(physical_gpu=gpu,free_mib=free),indent=2)+'\n')
    checked=subprocess.call([sys.executable,str(ROOT/'validate_run.py'),str(out)])
    return 0 if code==0 and checked==0 else 1

if __name__=='__main__':raise SystemExit(main())
