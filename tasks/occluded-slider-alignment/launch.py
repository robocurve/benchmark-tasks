"""Task launcher on the original YAM rig; dream mode performs paid remote inference."""

import argparse
import hashlib
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parent
DREAM_MODEL = 'dreamlabs/dream-1-beta'
MAX_STEPS = 2400


def cli_args(mode, steps, log_dir, epochs=1):
    """Build the actual CLI without importing Isaac Sim or creating a model client."""
    if not 1 <= steps <= MAX_STEPS:
        raise ValueError('steps must be between 1 and 2400')
    if not 1 <= epochs <= 3 or (mode == 'dream' and epochs != 1):
        raise ValueError('free reset checks allow 1..3 epochs; Dream allows one trial only')
    instruction = json.loads((ROOT / 'task.json').read_text())['instruction']
    task = ROOT / ('stability.py' if mode in ('setup', 'stability') else 'task.py')
    argv = ['inspect-robots', 'run', '--embodiment', 'yam_arena', '--policy',
            'dream' if mode == 'dream' else 'yam_scripted',
            '-E', f'task={task}', '-E', 'scene=station', '-E', 'object_jitter=0',
            '-E', 'headless=true', '-E', 'control_hz=20', '-E', 'cam_width=320', '-E', 'cam_height=176',
            '--instruction', instruction, '--scorer', 'task_score', '--max-steps', str(steps),
            '--epochs', str(epochs), '--fail-on-error', '1', '--store-frames', '--no-prompt', '--log-dir', str(log_dir)]
    if mode == 'dream':
        argv += ['-P', f'model={DREAM_MODEL}', '-P', 'record=false', '-P', 'dry_run=false',
                 '-P', 'rows_per_request=48', '-P', 'preroll_steps=32', '-P', 'timeout_s=300']
    elif mode != 'demo':
        argv += ['-P', f'hold_s={steps / 20 + 5}']
    return argv


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--log-dir', type=Path, required=True)
    p.add_argument('--steps', type=int, default=MAX_STEPS)
    p.add_argument('--epochs', type=int, default=1)
    p.add_argument('--mode', choices=('setup', 'stability', 'demo', 'contact-probe', 'dream'), default='setup')
    p.add_argument('--config-only', action='store_true', help='Save resolved configuration without simulator/model/network')
    args = p.parse_args()
    try:
        argv = cli_args(args.mode, args.steps, args.log_dir, args.epochs)
    except ValueError as error:
        p.error(str(error))
    args.log_dir.mkdir(parents=True, exist_ok=True)
    root = ROOT
    sources = {str(f.relative_to(root)): hashlib.sha256(f.read_bytes()).hexdigest()
               for f in sorted(root.rglob('*')) if f.suffix in ('.py', '.json', '.usda', '.md')}
    (args.log_dir / 'source_manifest.json').write_text(json.dumps(sources, indent=2)+'\n')
    (args.log_dir / 'effective_config.json').write_text(json.dumps({
        'mode': args.mode, 'argv': argv, 'steps': args.steps, 'epochs': args.epochs,
        'scoring_version': 'visual-contact-v2', 'formal_run_limit': MAX_STEPS,
        'control_hz': 20, 'physics_hz': 240, 'camera_rgb': [320, 176],
        'canonical_instruction': json.loads((root / 'task.json').read_text())['instruction'],
        'oracle_enabled': args.mode in ('demo', 'contact-probe'),
        'demo_record_until_s': 14.0 if args.mode == 'demo' else None,
        'config_only': args.config_only,
    }, indent=2)+'\n')
    if args.config_only:
        return 0
    if args.mode in ('demo', 'contact-probe'):
        from slider_task.oracle import install_inspection_plan
        candidate = Path(__file__).with_name('scene') / 'inspection_candidate.json'
        if not candidate.is_file():
            p.error('Run offline_diagnostics.py first to generate the inspection candidate')
        install_inspection_plan(json.loads(candidate.read_text()))
        if args.mode == 'contact-probe':
            from slider_task import oracle
            oracle.manipulation_waypoints = oracle.contact_probe_waypoints
    from slider_task.runtime import install
    # Record the scheduled release/retreat and final rest even if the outcome
    # first qualifies earlier. Score still comes from the final evaluated state.
    save = install(args.log_dir, record_until_s=14.0 if args.mode == 'demo' else None)
    sys.argv = argv
    try:
        from inspect_robots.cli import main as cli
        return cli()
    finally:
        save()


if __name__ == '__main__':
    raise SystemExit(main())
