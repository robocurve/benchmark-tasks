#!/usr/bin/env python3
"""Run hosted Dream-1 against native Isaac on visnu; retain real scoring evidence."""
import argparse
from dataclasses import asdict
import hashlib
import importlib.metadata
import json
from pathlib import Path
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--device', default='cuda:1')
    args = parser.parse_args()
    if socket.gethostname() != 'visnu':
        raise RuntimeError('Native task evaluation must run on verified visnu')

    from inspect_robots import Scene, Task, eval as evaluate
    from inspect_robots.cli import _build_guardrails
    from inspect_robots.logging.sink import NullSink
    from inspect_robots.logging import JsonLogSink
    from inspect_robots_dream import dream_policy
    from resist_isaac.runtime import ResistanceEmbodiment
    from tools.run_recovery_pilot import source_snapshot
    from yam_arena.station import top_camera

    task = Path(__file__).parent
    cfg = json.loads((task / 'pilot.json').read_text())
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    hashes = source_snapshot(task, output)
    receipt = dict(status='starting', hostname=socket.gethostname(), policy='dream',
        model='dreamlabs/dream-1-beta', instruction=cfg['instruction'],
        max_steps=cfg['max_steps'], seed=0, source_files_sha256=hashes,
        git_revision=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        gpu=subprocess.check_output(['nvidia-smi', '--query-gpu=index,name,uuid,driver_version',
                                     '--format=csv,noheader'], text=True),
        versions={p: importlib.metadata.version(p) for p in
                  ('isaacsim', 'isaaclab', 'isaaclab-arena', 'yam-arena',
                   'inspect-robots', 'inspect-robots-dream', 'dreamlabs-client')},
        policy_scope='Hosted Dream-1, camera images and robot joints only; bounds and delta guards',
        steps=0)

    def save():
        (output / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')

    class EvidenceSink(NullSink):
        def log_step(self, t, observation, action, result):
            base = emb._base
            if receipt['steps'] == 0:
                receipt.update(physics_dt_s=base.physics_dt, control_dt_s=base.step_dt,
                    native_devices=dict(requested=str(emb.device), environment=str(base.device),
                        simulation_config=str(base.sim.cfg.device),
                        physics_manager=str(base.sim.physics_manager.get_device()),
                        robot_tensors={side: str(base.scene[side + '_robot'].data.joint_pos.device)
                                       for side in ('left', 'right')}))
                devices = receipt['native_devices']
                assert all(devices[k] == devices['requested'] for k in
                           ('environment', 'simulation_config', 'physics_manager'))
                assert all(v == devices['requested'] for v in devices['robot_tensors'].values())
            receipt['steps'] += 1
            receipt['final_metrics'] = dict(result.info)
            trace.write(json.dumps(dict(step=receipt['steps'], time_s=receipt['steps']/20,
                action=action.data.tolist(), metrics=dict(result.info))) + '\n')
            trace.flush()
            save()

    emb = None
    policy = None
    started = time.monotonic()
    save()
    try:
        emb = ResistanceEmbodiment(task=str(task), device=args.device, control_hz=20,
                                   cam_width=1280, cam_height=720, headless=True)
        assert asdict(emb.rig.top_cam) == asdict(top_camera('calibrated'))
        emb._mechanism.set_evidence_directory(output)
        policy = dream_policy(model=receipt['model'], record=False)
        trial = Task(name=task.name, scenes=[Scene(id='scene-0', instruction=cfg['instruction'])],
                     scorer='task_score', max_steps=cfg['max_steps'])
        guards, active_guards, guard_warnings = _build_guardrails(
            emb.info.action_space, max_action_delta=None, embodiment=emb)
        receipt.update(guardrails=active_guards, guardrail_warnings=guard_warnings)
        with (output / 'trace.jsonl').open('w') as trace:
            evaluate(trial, policy, emb, log_dir=str(output / 'evaluation'),
                     sinks=[JsonLogSink(str(output / "evaluation")), EvidenceSink()], seed=0, fail_on_error=True,
                     approver=guards, store_frames=True, grader=None)
        logs = [p for p in (output / 'evaluation').glob('*.json') if '.live.' not in p.name]
        assert len(logs) == 1, 'Expected one completed evaluation log'
        log = json.loads(logs[0].read_text())
        assert log['status'] == 'success' and log['results']['errored_trials'] == 0, log['status']
        assert log['stats']['total_steps'] == receipt['steps'] <= cfg['max_steps']
        transcript = log['samples'][0]['policy_transcripts'][0]
        assert transcript['model'] == receipt['model'] and not transcript['dry_run']
        assert transcript['requests'], 'No actual Dream inference was recorded'
        receipt['dream_requests'] = len(transcript['requests'])
        receipt['evaluation_log'] = str(logs[0].relative_to(output))
        cli = Path(sys.executable).with_name('inspect-robots')
        subprocess.run([str(cli), 'video', str(logs[0]), '--out', str(output / 'videos'),
                        '--fps', '20'], check=True)
        for camera in ('top_cam', 'left_cam', 'right_cam'):
            (output / 'videos' / ('scene-0-e0_' + camera + '.mp4')).rename(output / (camera + '.mp4'))
        receipt['status'] = 'completed'
    except BaseException as error:
        receipt.update(status='error', error=f'{type(error).__name__}: {error}')
        raise
    finally:
        try:
            if policy is not None:
                policy.close()
            if emb is not None:
                emb.close()
        finally:
            receipt['elapsed_wall_s'] = time.monotonic() - started
            receipt['source_unchanged'] = all(
                hashlib.sha256((ROOT / p).read_bytes()).hexdigest() == h for p, h in hashes.items())
            receipt['evidence_sha256'] = {}
            for p in output.iterdir():
                if p.is_file() and p.suffix in ('.jsonl', '.mp4'):
                    with p.open('rb') as stream:
                        receipt['evidence_sha256'][p.name] = hashlib.file_digest(stream, 'sha256').hexdigest()
            save()
    print(json.dumps({k: receipt[k] for k in ('status', 'steps', 'dream_requests', 'final_metrics')}))


if __name__ == '__main__':
    main()
