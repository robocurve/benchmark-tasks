"""Bounded official Inspect evaluation of the common Arena scene."""
import argparse
import fcntl
import hashlib
import json
import shutil
from pathlib import Path
import sys
import traceback


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--policy', choices=['hold','demo','dream'], default='hold')
    p.add_argument('--steps', type=int, default=200)
    p.add_argument('--epochs', type=int, default=1)
    p.add_argument('--budget-ledger', type=Path)
    p.add_argument('--check-reset', action='store_true', help='Audit a fresh reset and 20-step hold after the local trial')
    a = p.parse_args()
    if not 1<=a.steps<=2000 or a.epochs not in (1,2):
        p.error('Use 1..2000 control steps and 1 or 2 epochs')
    if a.output.exists():
        p.error('Use a new output directory')
    if a.policy == 'dream' and (a.budget_ledger is None or a.epochs!=1):
        p.error('Dream requires one epoch and a persistent --budget-ledger shared by all paid runs')
    if a.policy == 'dream' and a.check_reset:
        p.error('Use the local demonstration for the reset audit')
    lock = open('/tmp/robocurve-shared-gpu.lock','a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    a.output.mkdir(parents=True)
    root = Path(__file__).resolve().parent
    source_files = [*root.glob('*.py'),root/'task.json',root/'DEPENDENCIES.json']
    (a.output/'source-manifest.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in source_files},indent=2)+'\n')
    (a.output/'source').mkdir()
    for path in source_files:
        shutil.copyfile(path,a.output/'source'/path.name)
    # Evaluate the archived source itself so later development edits cannot
    # change a running episode or invalidate its source provenance.
    sys.path.insert(0,str((a.output/'source').resolve()))
    arena = None
    policy = None
    result = {'policy':a.policy,'steps_limit':a.steps,'episodes':a.epochs,
              'control_hz':20,'object_jitter':0,'seed':0,'exit_code':1}
    try:
        import spec
        from inspect_robots import eval
        from inspect_robots.scene import Scene
        from inspect_robots.task import Task
        from inspect_task import Trolley, TrolleyScore, RGBDemo, Hold, INSTRUCTION
        arena = Trolley(a.output/'evidence',robot_state=a.policy!='demo')
        task = Task(name='hand_truck',scenes=[Scene(id='hand_truck',instruction=INSTRUCTION,init_seed=0)],
                    scorer=TrolleyScore(),max_steps=a.steps,epochs=a.epochs)
        if a.policy=='dream':
            from dream_budget import make_policy
            policy = make_policy(a.budget_ledger)
        else:
            policy = RGBDemo() if a.policy=='demo' else Hold()
        logs = eval(task,policy,arena,seed=0,
                    store_frames=True,fail_on_error=True,log_dir=str(a.output/'inspect'))
        result['statuses'] = [log.status for log in logs]
        if not logs or any(log.status=='error' for log in logs):
            raise RuntimeError('Inspect evaluation failed; inspect its recorded error')
        if a.check_reset:
            import numpy as np
            from inspect_robots import Action
            previous = arena.world.output
            observation = arena.reset(Scene(id='reset-check',instruction=INSTRUCTION),seed=0)
            world = arena.world
            assert world.steps == 0 and not world.history
            assert world.scorer.score == 0 and world.scorer.dwell == 0 and not world.scorer.invalid
            for filename in ['reset-state.json','reset-robot-state.json','reset-cameras.json']:
                assert json.loads((previous/filename).read_text()) == json.loads((world.output/filename).read_text()), filename
            assert set(observation.images) == {'top_cam','left_cam','right_cam'}
            assert set(observation.image_times.values()) == {0.}
            # Hold the published starting command; no live object/robot state
            # is used to generate these regression-test actions.
            command = world.initial_command
            target = np.concatenate([np.r_[command[s][:6],command[s][6]/world.constants.FINGER_OPEN]
                                     for s in ['left','right']])
            for _ in range(20):
                step = arena.step(Action(target))
                assert step.reward == 0 and not step.info['invalid']
            (a.output/'reset-check.json').write_text(json.dumps({'poses_and_velocities_match':True,'counters_cleared':True,'initial_camera_bytes_match':True,'hold_steps':20,'hold_score':0.},indent=2)+'\n')
        result['exit_code'] = 0
    except BaseException as error:
        traceback.print_exc()
        result['error_type'] = type(error).__name__
        result['error'] = str(error)
    finally:
        if policy is not None and callable(getattr(policy, 'close', None)):
            policy.close()
        if arena is not None:
            arena.close()
        (a.output/'process-result.json').write_text(json.dumps(result,indent=2)+'\n')
        sys.stdout.flush()
        sys.stderr.flush()
        import arena_runtime
        if arena_runtime.APP is not None:
            arena_runtime.APP.close(exit_code=result['exit_code'])
    return result['exit_code']


if __name__=='__main__':
    raise SystemExit(main())
