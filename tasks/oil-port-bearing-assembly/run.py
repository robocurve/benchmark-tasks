"""Portable runner. Dream is paid; --config-only is an offline configuration check."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys

ROOT=Path(__file__).resolve().parent


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',choices=('setup','demo','dream'),required=True)
    p.add_argument('--steps',type=int)
    p.add_argument('--gpu',choices=('auto','0','1'),default='auto')
    p.add_argument('--log-dir',type=Path,required=True)
    p.add_argument('--config-only',action='store_true')
    a=p.parse_args();steps=a.steps if a.steps is not None else 40 if a.mode=='setup' else 2400
    if not 1<=steps<=2400:p.error('Budget must be 1..2400 actions')
    out=a.log_dir.resolve()
    if out.is_relative_to(ROOT):p.error('Keep evidence outside the task source directory')
    if out.exists() and any(out.iterdir()):p.error('Use a new empty output directory')
    command=[sys.executable,'-u',str(ROOT/'launch.py'),'--mode',a.mode,'--steps',str(steps),
             '--gpu',a.gpu,'--log-dir',str(out)]
    env=dict(os.environ,OMNI_KIT_ACCEPT_EULA='YES',ACCEPT_EULA='Y',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    if a.config_only:return subprocess.call(command+['--config-only'],env=env)
    out.parent.mkdir(parents=True,exist_ok=True)
    console=out.parent/(out.name+'.console.log')
    print(f'{a.mode}: at most {steps} actions; GPU {a.gpu}; console: {console}',flush=True)
    with console.open('x') as log:
        proc=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:code=proc.wait(timeout=3600 if a.mode=='dream' else 1800)
        except (subprocess.TimeoutExpired,KeyboardInterrupt) as exc:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=30)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
            code=124 if isinstance(exc,subprocess.TimeoutExpired) else 130
    out.mkdir(exist_ok=True);(out/'process.exit').write_text(str(code)+'\n')
    checked=subprocess.call([sys.executable,str(ROOT/'validate_run.py'),str(out)],env=env)
    return 0 if code==0 and checked==0 else 1


if __name__=='__main__':raise SystemExit(main())
