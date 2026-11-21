#!/usr/bin/env python3
"""Evaluate stock Dream-1 on one frozen JHU task, preserving native evidence."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib
from importlib.metadata import version
import json
import math
import os
from pathlib import Path
import re
import stat
import sys
import time
import traceback

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODEL = 'dreamlabs/dream-1-beta'
TASKS = {'empty-a-toaster-crumb-tray': ('rdb_toaster_tray', 'toaster_tray_score')}
CAMERAS = ('top_cam', 'left_cam', 'right_cam')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(type(value).__name__)


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, default=json_value, allow_nan=False) + '\n')


def redact(value):
    return re.sub(r'dl_live_[A-Za-z0-9_-]+', '[REDACTED_DREAM_KEY]', str(value))


def observation_row(observation, tick):
    if set(observation.images) != set(CAMERAS) or set(observation.state) != {'joint_pos'}:
        raise ValueError('Model observation must contain only three stock cameras and joint_pos')
    for name in CAMERAS:
        frame = np.asarray(observation.images[name])
        if frame.shape != (176, 320, 3) or frame.dtype != np.uint8:
            raise ValueError(f'Unexpected model camera {name}: {frame.shape}/{frame.dtype}')
    measured = np.asarray(observation.state['joint_pos'], dtype=float)
    if measured.shape != (14,) or not np.isfinite(measured).all():
        raise ValueError('Expected actual finite 14-D measured joint state')
    return dict(tick=tick, time_s=observation.state_time, joint_pos=measured.tolist())


def policy_hashes():
    import dream
    import inspect_robots_dream
    files = [Path(__file__), Path(__file__).with_name('dream_model_audit.py')]
    integration = Path(inspect_robots_dream.__file__).parent
    sdk = Path(dream.__file__).parent
    rows = {str(p.relative_to(ROOT)): digest(p) for p in files}
    rows.update({'inspect_robots_dream/' + p.name: digest(p) for p in sorted(integration.glob('*.py'))})
    rows.update({'dream/' + p.name: digest(p) for p in sorted(sdk.glob('*.py'))})
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', choices=TASKS, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--credential-file', type=Path,
                        default=Path(os.environ.get('DREAM_CREDENTIALS', Path.home() / '.config/api/credentials.env')))
    parser.add_argument('--timeout-s', type=float, default=300.)
    parser.add_argument('--request-cap', type=int,
                        help='Bounded attempt allowance; successful chunks still use the normal action budget')
    args = parser.parse_args()
    os.umask(0o077)
    credential_file = args.credential_file.expanduser().resolve(strict=True)
    for path, required in [(credential_file.parent, 0o700), (credential_file, 0o600)]:
        if path.stat().st_uid != os.getuid() or stat.S_IMODE(path.stat().st_mode) != required:
            raise RuntimeError(f'Private credential permissions required: {path}')
    # Only this process reads the shared dotenv. Unrelated provider keys are never exported.
    os.environ['DREAM_CREDENTIALS'] = str(credential_file)
    os.environ.pop('DREAM_API_KEY', None)
    import dream
    from inspect_robots import Scene, eval as inspect_eval
    from inspect_robots.guard import ClampGuard, DeltaLimitGuard, GuardChain
    from inspect_robots.logging import JsonLogSink
    from inspect_robots.task import Task
    from inspect_robots_dream.policy import DreamPolicy
    from inspect_robots_dream.session import DreamSession
    from dream_model_audit import AuditedDreamClient, DreamAudit

    module, scorer_name = TASKS[args.task]
    candidate = ROOT
    sys.path.insert(0, str(candidate))
    F = importlib.import_module(module + '.fixed_scene')
    adapter = importlib.import_module(module + '.adapter')
    grading = importlib.import_module(module + '.grading')
    task_runner = importlib.import_module(module + '.run')
    if F.CONTROL_HZ != 20:
        raise RuntimeError('Dream baseline requires the frozen 20 Hz task')
    args.out = args.out.resolve()
    args.out.mkdir(parents=True, exist_ok=False, mode=0o700)
    started = time.monotonic()
    start_utc = datetime.now(timezone.utc).isoformat()
    initial_sources = task_runner.source_hashes()
    initial_policy = policy_hashes()
    # Real service sessions can expire between chunks while native physics runs.
    # Allow retries without shortening the task or changing any successful row.
    maximum_requests = (args.request_cap if args.request_cap is not None else
                        3 * math.ceil((F.MAX_STEPS - 32) / 48) + 16)
    if maximum_requests < 1:
        parser.error('--request-cap must be positive')
    source_snapshot = args.out / 'policy-source'
    source_snapshot.mkdir(mode=0o700)
    for name in ('evaluate_dream.py', 'dream_model_audit.py'):
        path = Path(__file__).with_name(name)
        (source_snapshot / name).write_bytes(path.read_bytes())
    write_json(args.out / 'CONFIG.json', dict(
        task=args.task, module=module, model=MODEL, policy_variant='stock_inspect_robots_dream',
        instruction=F.INSTRUCTION, seed=0, model_sampling_seed=None,
        max_steps=F.MAX_STEPS, control_hz=20, rows_per_request=48, preroll_steps=32,
        timeout_s=args.timeout_s, record=False, custom_blending=False, custom_smoother_reset=False,
        request_cap=maximum_requests,
        credential_file=str(credential_file), credential_variable='DREAM_API_KEY',
        source_sha256=initial_sources, policy_source_sha256=initial_policy,
        guards=['clamp', 'delta-limit:max_delta=1.0'], started_at_utc=start_utc,
        gpu_selection=os.environ.get('CUDA_VISIBLE_DEVICES'),
    ))
    actions = (args.out / 'actions.jsonl').open('x')
    observations = (args.out / 'measured-observations.jsonl').open('x')
    policy_actions = (args.out / 'policy-actions.jsonl').open('x')
    audit = DreamAudit(args.out)
    client = AuditedDreamClient(dream.DreamClient(), maximum_requests, audit)
    rig = adapter.make_embodiment(media_dir=args.out / 'media')
    raw_targets = {}
    guard_changes = []

    class ObservedDream(DreamPolicy):
        def act(self, observation):
            observation_row(observation, rig.tick)
            chunk = super().act(observation)
            if len(chunk.actions) != 1:
                raise RuntimeError('Stock Dream must return exactly one row per step')
            target = np.asarray(chunk.actions[0].data, float)
            if target.shape != (14,) or not np.isfinite(target).all():
                raise RuntimeError('Invalid Dream action')
            raw_targets[rig.tick + 1] = target.copy()
            policy_actions.write(json.dumps(dict(tick=rig.tick + 1, command=target.tolist()), allow_nan=False) + '\n')
            return chunk

    policy = ObservedDream(session=DreamSession(make_client=lambda: client), model=MODEL,
                           record=False, timeout_s=args.timeout_s)
    original_reset, original_step = rig.reset, rig.step

    def logged_reset(scene, *, seed=None):
        observation = original_reset(scene, seed=seed)
        observations.write(json.dumps(observation_row(observation, 0), allow_nan=False) + '\n')
        observations.flush()
        return observation

    def logged_step(action):
        result = original_step(action)
        tick = rig.tick
        actual = np.asarray(action.data, float)
        raw = raw_targets.pop(tick)
        if not np.array_equal(actual, raw):
            guard_changes.append(dict(tick=tick, dimensions=np.flatnonzero(actual != raw).tolist(),
                                      maximum_change=float(np.max(np.abs(actual - raw)))))
        actions.write(json.dumps(dict(tick=tick, time_s=result.observation.state_time,
                                      command=actual.tolist()), allow_nan=False) + '\n')
        observations.write(json.dumps(observation_row(result.observation, tick), allow_nan=False) + '\n')
        if tick % 100 == 0 or result.terminated:
            print(json.dumps(dict(kind='progress', task=args.task, tick=tick,
                                  score=rig.final_grade['score'], void=rig.final_grade.get('void'),
                                  dream_requests=client.calls)), flush=True)
            actions.flush(); observations.flush(); policy_actions.flush()
        return result

    rig.reset, rig.step = logged_reset, logged_step
    error = None
    evaluation = None
    native_samples = 0
    result_report = None
    try:
        guard = GuardChain(ClampGuard(rig.info.action_space),
                           DeltaLimitGuard(rig.info.action_space, max_delta=1.),
                           names=['clamp', 'delta-limit'])
        task = Task(args.task, scenes=[Scene(id='fixed-start', instruction=F.INSTRUCTION)],
                    scorer=grading.make_scorer(), max_steps=F.MAX_STEPS)
        evaluation = inspect_eval(task, policy, rig, log_dir=str(args.out / 'inspect'), guard=guard,
                                  sinks=[JsonLogSink(args.out / 'inspect'), audit], seed=0,
                                  grader=None, store_frames=False, store_records=False, fail_on_error=True)[0]
        if evaluation.status != 'success' or not evaluation.samples:
            raise RuntimeError(redact(evaluation.error or evaluation.status))
        sample = evaluation.samples[0]
        score = sample.reduced.get(scorer_name)
        if score is None or rig.final_grade is None:
            raise RuntimeError('Native scorer produced no terminal score')
        if float(score) != float(rig.final_grade['score']):
            raise RuntimeError('Inspect score differs from actual native grade')
        if rig.tick != F.MAX_STEPS and not rig.final_grade['finished']:
            raise RuntimeError('Model run stopped before its full budget or native terminal state')
        if client.successful_replies < 1:
            raise RuntimeError('Run contains no real Dream inference')
        if initial_sources != task_runner.source_hashes() or initial_policy != policy_hashes():
            raise RuntimeError('Task or policy source changed during the model trial')
        native_samples = rig.recorder.physics_samples
        if native_samples != rig.tick * round(1 / rig.physics_dt / 20):
            raise RuntimeError('Incomplete native scoring trace')
        result_report = dict(
            policy='dream', model=MODEL, policy_variant='stock_inspect_robots_dream',
            execution_status='completed', candidate_slug=args.task, module=module, scorer=scorer_name,
            score=float(score), executed_steps=rig.tick, step_limit=F.MAX_STEPS,
            declared_step_limit=F.MAX_STEPS, control_hz=20, seed=0, model_sampling_seed=None,
            smoke_only=False, source_sha256=initial_sources, source_unchanged=True,
            policy_source_sha256=initial_policy, policy_source_unchanged=True,
            physics_hz=1 / rig.physics_dt, physics_samples=native_samples,
            observations=dict(cameras=list(CAMERAS), state=['joint_pos'], rgb_size=[320, 176]),
            scripted_reads_simulator_state=False, model_reads_simulator_state=False,
            dream_requests=client.calls, dream_successful_replies=client.successful_replies,
            request_cap=maximum_requests, guard_modified_steps=len(guard_changes),
            guard_changes=guard_changes, final_grade=rig.final_grade,
            termination_reasons=list(sample.termination_reasons),
            dream_transcript=policy.transcript(), diagnostics=rig.diagnostics,
            runtime={p: version(p) for p in ('isaacsim', 'inspect-robots', 'inspect-robots-yam',
                     'yam-arena', 'dreamlabs-client', 'inspect-robots-dream')},
            started_at_utc=start_utc, ended_at_utc=datetime.now(timezone.utc).isoformat(),
            elapsed_s=round(time.monotonic() - started, 2),
        )
    except BaseException as exc:
        error = exc
        write_json(args.out / 'ERROR.json', dict(
            policy='dream', model=MODEL, execution_status='infrastructure_error',
            candidate_slug=args.task, error_type=type(exc).__name__, message=redact(exc),
            traceback=redact(traceback.format_exc()), source_sha256=initial_sources,
            executed_steps=rig.tick, step_limit=F.MAX_STEPS, partial_grade=rig.final_grade,
            dream_requests=client.calls, dream_successful_replies=client.successful_replies,
            counts_as_completed_model_trial=False,
        ))
    finally:
        try:
            rig.close()
        finally:
            try:
                policy.close()
            finally:
                audit.close()
                actions.close(); observations.close(); policy_actions.close()
        write_json(args.out / 'DREAM_USAGE.json', dict(
            action_requests=client.calls, successful_replies=client.successful_replies,
            request_cap=maximum_requests, model=MODEL, account_recording=False,
            policy_variant='stock_inspect_robots_dream',
        ))
    if error is not None:
        print(json.dumps(dict(status='infrastructure_error', task=args.task,
                              error_type=type(error).__name__, output=str(args.out))), flush=True)
        return 2
    write_json(args.out / 'RESULT.json', result_report)
    print(json.dumps({k: v for k, v in result_report.items()
                     if k not in ('diagnostics', 'source_sha256', 'policy_source_sha256',
                                  'guard_changes', 'dream_transcript')}, default=json_value), flush=True)
    # A completed score-zero/partial model trial is a valid measured outcome.
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
