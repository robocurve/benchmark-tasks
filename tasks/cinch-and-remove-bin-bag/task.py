"""YAM removal-task launcher using the verified Inspect Robots interface by default."""
import argparse,json,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parent
def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['setup','demo']);p.add_argument('--out',type=Path,required=True);p.add_argument('--device',default='cuda:0');p.add_argument('--interface',choices=['inspect'],default='inspect');a=p.parse_args()
    out=a.out.resolve()
    if out.exists():p.error('Use a fresh output directory')
    driver='inspect_run_removal.py' if a.interface=='inspect' else 'run_removal.py'
    rc=subprocess.call([sys.executable,'-u',str(ROOT/driver),'--mode','setup' if a.mode=='setup' else 'full','--camera','--run',str(out),'--device',a.device])
    if a.mode=='setup':return int(rc!=0 or not (out/'setup_result.json').exists() or (out/'trial_error.txt').exists())
    if (out/'trace.json').exists():subprocess.call([sys.executable,str(ROOT/'evaluate_removal.py'),str(out)])
    f=out/'verified_result.json'
    return int(rc!=0 or not f.exists() or not json.loads(f.read_text()).get('full_task_success',False) or any((out/n).exists() for n in ['error.txt','trial_error.txt']))
if __name__=='__main__':sys.exit(main())

