"""Run the baby-bottle assembly on the original station through Inspect Robots."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import numpy as np
from zipfile import ZipFile, ZIP_DEFLATED

ROOT = Path(__file__).resolve().parent
INSTRUCTION = "Fit the teat and screw the collar onto the bottle, then release the assembly."


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--gpu', choices=('0','1'), required=True)
    parser.add_argument('--mode', choices=('demo', 'setup', 'dream'), default='demo')
    parser.add_argument('--config-only', action='store_true')
    parser.add_argument('--collar-only-demo', action='store_true', help='End the scripted example after the fitted pair is released, before mounting it on the bottle.')
    args = parser.parse_args()
    if args.collar_only_demo and args.mode != 'demo':
        parser.error('--collar-only-demo requires --mode demo')
    args.out.mkdir(parents=True,exist_ok=False)
    source_hashes = {}
    with ZipFile(args.out/'source.zip','w',ZIP_DEFLATED) as zip_file:
        for path in sorted(ROOT.iterdir()):
            if path.is_file() and path.suffix in ('.py','.md','.npz','.json'):
                zip_file.write(path,path.name)
                source_hashes[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
    (args.out/'source-manifest.json').write_text(json.dumps(source_hashes,indent=2)+'\n')
    os.environ.update(CUDA_VISIBLE_DEVICES=args.gpu, OMNI_KIT_ACCEPT_EULA='YES', ACCEPT_EULA='Y')
    from inspect_robots.types import Action, ActionChunk
    from inspect_robots_yam.packing import pack
    from yam_arena.scripted import YamScriptedPolicy
    from monitor import install, save
    from scoring import evaluate, read_window
    from load_exam import register, grade
    from inspect_robots.cli import main as inspect_main
    policy_name = 'dream' if args.mode == 'dream' else 'yam_scripted'
    steps = 2400 if args.mode == 'dream' else 40
    actions = np.load(ROOT/'demo-actions.npz')['actions'] if args.mode == 'demo' else None
    if args.collar_only_demo:
        actions = actions[:962]  # Last rest action before the collar is picked up for threading.
    if actions is not None:
        steps = len(actions)
    argv = ['inspect-robots','run','--embodiment','yam_arena','--policy',policy_name,
            '-E',f'task={ROOT/"task.py"}','-E','scene=station','-E','object_jitter=0',
            '-E','cam_width=320','-E','cam_height=176','-E','headless=true','-E','control_hz=20',
            '--instruction',INSTRUCTION,'--grader','collar_load_exam','--scorer','collar_retention',
            '--max-steps',str(steps),'--epochs','1','--fail-on-error','1','--store-frames',
            '--no-prompt','--log-dir',str(args.out)]
    if args.mode=='dream':
        argv += ['-P','model=dreamlabs/dream-1-beta','-P','record=false','-P','dry_run=false',
                 '-P','rows_per_request=48','-P','preroll_steps=32','-P','timeout_s=300']
    from runtime_check import check
    (args.out/'configuration.json').write_text(json.dumps(dict(mode=args.mode, steps=steps,
        argv=argv, runtime=check(), demo_actions_loaded=actions is not None),indent=2)+'\n')
    if args.config_only:
        return
    if actions is not None and (actions.ndim != 2 or actions.shape[1] != 14 or not np.isfinite(actions).all()):
        raise ValueError("Invalid fixed-layout joint targets")
    def reset(policy,scene):
        policy.index = 0
    def act(policy, observation):
        if actions is None:
            from inspect_robots_yam.packing import STATE_KEY
            row = observation.state[STATE_KEY]
        else:
            row = actions[policy.index]
        policy.index += 1
        return ActionChunk(actions=[Action(data=pack(row[:7],row[7:]))],control_hz=20)
    if args.mode != "dream":
        YamScriptedPolicy.reset = reset
        YamScriptedPolicy.act = act
    install()
    register(args.out)
    from image_health import install as guard
    save_health = guard(args.out)
    sys.argv = argv
    try:
        inspect_main()
    except SystemExit as exc:
        if exc.code not in (None,0):
            raise
    finally:
        save_health()
    log, = [p for p in args.out.glob('adhoc*.json') if '.live.' not in p.name]
    record = json.loads(log.read_text())
    if record['status'] != 'success' or record['results'].get('errored_trials'):
        raise RuntimeError(f"Inspect did not finish the trial. See {log}")
    save(args.out)
    replay = evaluate(read_window(args.out))
    trace = json.loads((args.out/'score-trace.json').read_text())
    if len(trace) != steps or replay['score'] != trace[-1]:
        raise RuntimeError("Released assembly trace and independent endpoint disagree")
    proof = None
    if replay['score']:
        with np.load(args.out/'collar-exam.npz') as raw:
            proof = grade(raw)
    score = min(replay['score'], proof['score']) if proof else 0.0
    if score != record['results']['metrics']['collar_retention']:
        raise RuntimeError("Stored judgement and mechanical replay disagree")
    (args.out/'endpoint.json').write_text(json.dumps(dict(score=score, released_assembly=replay,
        collar_load_exam=proof),indent=2)+'\n')


if __name__ == '__main__':
    main()
