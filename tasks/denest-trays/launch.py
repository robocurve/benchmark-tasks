"""Run this task on any selected host GPU, with no Brev or viewer dependency."""
import argparse
import os
from pathlib import Path
import subprocess
import uuid
ROOT=Path(__file__).resolve().parent

def command(args):
    if args.gpu < 0: raise ValueError('--gpu must be nonnegative')
    if args.policy=='dream' and args.episodes!=1: raise ValueError('Dream runs one episode')
    out=args.output.resolve()/f'gpu{args.gpu}'; home=out/'home'
    home.mkdir(parents=True,exist_ok=True);(home/'tmp').mkdir(exist_ok=True)
    cmd=['docker','run','--rm','--name','denest-trays-'+uuid.uuid4().hex[:8],
         '--gpus',f'device={args.gpu}','--network','host','--shm-size','8g',
         '--user',f'{os.getuid()}:{os.getgid()}',
         '-v','/etc/passwd:/etc/passwd:ro','-v','/etc/group:/etc/group:ro',
         '-v',f'{ROOT}:/task:ro','-v',f'{out}:/outputs:rw','-v',f'{home}:/tmp/task-home:rw',
         '-e','HOME=/tmp/task-home','-e','TMPDIR=/tmp/task-home/tmp',
         '-e',f'USER={os.environ.get("USER","robot")}',
         '-e','OMNI_KIT_ACCEPT_EULA=YES','-e','ACCEPT_EULA=Y','-e','NVIDIA_DRIVER_CAPABILITIES=all']
    for group in sorted(set(os.getgroups()) | {1234}):cmd+=['--group-add',str(group)]
    if args.policy=='dream':
        if args.credentials is None or args.budget is None: raise ValueError('Dream requires --credentials and --budget (an existing shared ledger)')
        secret=args.credentials.resolve(); budget=args.budget.resolve()
        if not secret.is_file() or secret.stat().st_mode&0o077: raise ValueError('Credentials must be a mode-600 file')
        from dream_budget import Ledger
        Ledger(budget).status('denest-trays')
        cmd+=['-v',f'{secret}:/run/secrets/dream.env:ro','-e','DREAM_CREDENTIALS=/run/secrets/dream.env',
              '-v',f'{budget.parent}:/dream-budget:rw']
    cmd+=[args.image,'/task/run.py','--policy',args.policy,'--episodes',str(args.episodes),
          '--livestream','0','--viz','none','--kit_args',
          '--portable-root /tmp/task-home/kit --/plugins/carb.tasking.plugin/threadCount=16']
    if args.policy=='dream':cmd+=['--budget-path','/dream-budget/'+args.budget.name]
    return cmd

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--gpu',type=int,required=True)
    p.add_argument('--policy',choices=('scripted','dream'),default='scripted')
    p.add_argument('--episodes',type=int,default=1)
    p.add_argument('--image',default='denest-trays-runtime:6.1')
    p.add_argument('--output',type=Path,default=ROOT/'.runs')
    p.add_argument('--credentials',type=Path)
    p.add_argument('--budget',type=Path)
    p.add_argument('--dry-run',action='store_true')
    a=p.parse_args()
    if a.episodes<1:p.error('--episodes must be positive')
    try:cmd=command(a)
    except (ValueError,RuntimeError) as e:p.error(str(e))
    if a.dry_run:
        import shlex
        print(shlex.join(cmd))
    else:subprocess.run(cmd,check=True)
