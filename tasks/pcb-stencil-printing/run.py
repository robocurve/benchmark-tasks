"""Run one bounded task episode; configuration-only never calls Dream."""
import argparse,json,os,signal,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--mode',choices=['setup','demo','dream'],required=True);p.add_argument('--log-dir',type=Path,required=True);p.add_argument('--steps',type=int);p.add_argument('--gpu',choices=['auto','0','1'],default='auto');p.add_argument('--config-only',action='store_true');a=p.parse_args()
    out=a.log_dir.resolve()
    if out.is_relative_to(ROOT) or (out.exists() and any(out.iterdir())):p.error('Use a new output directory outside task source')
    cmd=[sys.executable,'-u',str(ROOT/'launch.py'),'--mode',a.mode,'--gpu',a.gpu,'--log-dir',str(out)]
    if a.steps is not None:cmd+=['--steps',str(a.steps)]
    if a.config_only:return subprocess.call(cmd+['--config-only'])
    out.parent.mkdir(parents=True,exist_ok=True);console=out.parent/(out.name+'.console.log')
    with console.open('x') as f:
        proc=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        try:code=proc.wait(timeout=7200)
        except (subprocess.TimeoutExpired,KeyboardInterrupt) as exc:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=30)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
            code=124 if isinstance(exc,subprocess.TimeoutExpired) else 130
    out.mkdir(exist_ok=True);(out/'process.exit').write_text(str(code)+'\n')
    if code:return code
    return subprocess.call([sys.executable,str(ROOT/'validate_run.py'),str(out)])
if __name__=='__main__':raise SystemExit(main())
