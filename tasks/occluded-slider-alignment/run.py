"""Portable entry point in the official Arena Python environment; dream mode is paid."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

ROOT = Path(__file__).resolve().parent


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode', choices=('setup','stability','demo','contact-probe','dream'), required=True)
    p.add_argument('--log-dir', type=Path, required=True)
    p.add_argument('--steps', type=int)
    p.add_argument('--epochs', type=int, default=1)
    p.add_argument('--config-only', action='store_true')
    args = p.parse_args()
    from launch import cli_args
    steps = args.steps if args.steps is not None else {'setup':20,'contact-probe':150}.get(args.mode,2400)
    cli_args(args.mode,steps,args.log_dir,args.epochs)  # fail before any client or physics
    out = args.log_dir.resolve()
    if out.exists() and any(out.iterdir()):
        p.error('Use a new empty log directory; prior evidence is never overwritten')
    out.mkdir(parents=True,exist_ok=True)
    from runtime_check import check
    (out/'runtime_contract.json').write_text(json.dumps(check(),indent=2)+'\n')
    command=[sys.executable,'-u',str(ROOT/'launch.py'),'--mode',args.mode,
             '--steps',str(steps),'--epochs',str(args.epochs),'--log-dir',str(out)]
    if args.config_only:
        return subprocess.call(command+['--config-only'])
    if args.mode=='dream':
        # Local checks first; this script makes only a metadata GET before inference.
        access=subprocess.run([sys.executable,str(ROOT/'check_dream_access.py'),
                               '--output',str(out/'access.json')],timeout=60)
        if access.returncode:
            return access.returncode
    environment=dict(os.environ, OMNI_KIT_ACCEPT_EULA='YES', ACCEPT_EULA='Y')
    wall_limit=1800 if args.mode=='dream' else 600
    print(f'{args.mode}: {steps} actions/epoch, {args.epochs} epoch(s), wall cap{wall_limit}s; logs: {out}',flush=True)
    with (out/'console.log').open('w') as log:
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,
                                 env=environment,start_new_session=True)
        try:
            code=process.wait(timeout=wall_limit)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGINT)
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL)
                process.wait()
            code=124
    (out/'process.exit').write_text(str(code)+'\n')
    checked=subprocess.call([sys.executable,str(ROOT/'validate_run.py'),str(out)])
    return 0 if code==0 and checked==0 else 1


if __name__=='__main__':
    raise SystemExit(main())
