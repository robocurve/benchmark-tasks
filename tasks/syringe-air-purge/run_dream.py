"""Run an actual Dream-1 trial; keep the API key out of files and logs."""
from __future__ import annotations
import argparse
import ast
from datetime import datetime, timezone
import getpass
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

CORE = ('task.py', 'fluid.py', 'seal.py', 'sample_visuals.py', 'endpoint_hold.py', 'rack.py', 'mechkit.py', 'demokit.py', 'task.json')

def instruction(task):
    for node in ast.parse(task.read_text()).body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'INSTRUCTION' for t in node.targets):
            return ast.literal_eval(node.value)
    raise ValueError('Task has no literal INSTRUCTION')

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--worker', type=Path)
    ap.add_argument('--cli', default='/data/IsaacLab-Arena/.venv/bin/inspect-robots')
    args = ap.parse_args()
    task_dir = Path(__file__).resolve().parent
    for name in CORE:
        if not (task_dir / name).is_file():
            raise SystemExit(f'Missing task source: {name}')
    if not Path(args.cli).is_file():
        raise SystemExit('Inspect Robots executable is missing; specify --cli.')
    prompt = instruction(task_dir / 'task.py')
    if args.check:
        subprocess.run([args.cli, 'list', 'policies'], check=True)
        print('Preflight passed: source and CLI present; seed 1; 1800 actions; 20 Hz; cameras 320x176.')
        return
    if args.worker:
        run = args.worker
        config = json.loads((run / 'run_config.json').read_text())
        try:
            completed = subprocess.run(config['command'], check=False)
            video_codes = []
            ffmpeg = shutil.which('ffmpeg')
            if not ffmpeg:
                import imageio_ffmpeg
                ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
            for log in sorted((run / 'eval').glob('*.json')):
                if log.name.endswith('.live.json'):
                    continue
                video_codes.append(subprocess.run([args.cli, 'video', str(log), '--out', str(run / 'videos'), '--fps', '20', '--ffmpeg', ffmpeg], check=False).returncode)
            (run / 'runner_status.json').write_text(json.dumps({'run_exit_code': completed.returncode, 'video_exit_codes': video_codes, 'finished_at': datetime.now(timezone.utc).isoformat()}, indent=2) + '\n')
            print('Trial process finished; inspect evaluation score and runner_status.json.', flush=True)
        except Exception as error:
            # Do not serialize environment or credentials.
            (run / 'runner_status.json').write_text(json.dumps({'error_type': type(error).__name__}) + '\n')
            raise
        return
    key = os.environ.get('DREAM_API_KEY', '').strip()
    if not key:
        if not sys.stdin.isatty():
            raise SystemExit('Open a terminal to enter the Dream API key privately.')
        key = getpass.getpass('Paste existing Dream API key (hidden; used for this trial only): ').strip()
    if not key:
        raise SystemExit('No key entered. No model run started.')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    run = Path.home() / 'yam-eval' / ('syringe-dream-v16-' + stamp)
    source = run / 'source'
    source.mkdir(parents=True, exist_ok=False)
    hashes = {}
    for name in CORE:
        shutil.copy2(task_dir / name, source / name)
        hashes[name] = hashlib.sha256((source / name).read_bytes()).hexdigest()
    (source / 'sha256.json').write_text(json.dumps(hashes, indent=2) + '\n')
    command = [args.cli, 'run', '--embodiment', 'yam_arena', '--policy', 'dream',
               '-E', 'task=' + str(source / 'task.py'), '-E', 'object_jitter=0',
               '-E', 'control_hz=20', '-E', 'cam_width=320', '-E', 'cam_height=176',
               '--instruction', prompt, '--scorer', 'task_score', '--max-steps', '1800',
               '--seed', '1', '--store-frames', '--no-prompt', '--log-dir', str(run / 'eval')]
    (run / 'run_config.json').write_text(json.dumps({'command': command, 'instruction': prompt, 'source_sha256': hashes}, indent=2) + '\n')
    env = os.environ.copy()
    env['DREAM_API_KEY'] = key
    env['PYTHONUNBUFFERED'] = '1'
    with (run / 'runner.log').open('w') as stream:
        process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--worker', str(run), '--cli', args.cli], stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    print(f'Dream-1 trial started. PID: {process.pid}\nResults: {run}\nProgress: tail -f {run}/runner.log')

if __name__ == '__main__':
    main()
