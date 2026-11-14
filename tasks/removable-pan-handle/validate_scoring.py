#!/usr/bin/env python3
"""Replay a completed native trial and require agreement with its live scores.

Run from the repository root with PYTHONPATH=. in the native environment:
  python tasks/removable-pan-handle/validate_scoring.py OUTPUT --expected 1
This validates recorded evidence; it does not run or replace a simulation.
"""
import argparse
import hashlib
import importlib
import json
from pathlib import Path

PanScore = importlib.import_module('tasks.removable-pan-handle.scoring').PanScore


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def validate(directory, expected, rescore=False):
    receipt = json.loads((directory/'receipt.json').read_text())
    assert receipt['hostname'] == 'visnu' and receipt['status'] == 'completed'
    assert receipt['source_unchanged']
    devices = receipt['native_devices']
    assert all(devices[k] == devices['requested'] for k in ('environment','simulation_config','physics_manager'))
    assert all(v == devices['requested'] for v in devices['robot_tensors'].values())
    assert devices['requested'].startswith('cuda:')
    for name in ('native-pan-contacts.jsonl','trace.jsonl','top_cam.mp4'):
        assert sha256(directory/name) == receipt['evidence_sha256'][name], name
    task = Path(__file__).parent
    for name in ('task.py','geometry.py','contacts.py'):
        key = 'tasks/removable-pan-handle/'+name
        assert sha256(task/name) == receipt['source_files_sha256'][key], 'Source differs: '+name
    original_scorer = receipt['source_files_sha256']['tasks/removable-pan-handle/scoring.py']
    evaluated_scorer = sha256(task/'scoring.py')
    assert rescore or original_scorer == evaluated_scorer, 'Scorer differs; use --rescore explicitly'
    trace = [json.loads(line) for line in (directory/'trace.jsonl').read_text().splitlines()]
    assert len(trace) == receipt['steps'] <= receipt['max_steps']
    with (directory/'native-pan-contacts.jsonl').open() as stream:
        scorer = PanScore(json.loads(next(stream)))
        batch = []; controls = 0; changed_steps = []
        for line in stream:
            batch.append(json.loads(line))
            if len(batch) != round(receipt['control_dt_s']/receipt['physics_dt_s']):
                continue
            result = scorer.update(batch); batch = []
            row = trace[controls]
            assert row['step'] == controls+1
            if any(result[k] != row['metrics'][k] for k in ('score','success')):
                changed_steps.append(row['step'])
                assert rescore, ('Live/replay mismatch', row['step'])
            controls += 1
        assert not batch and controls == len(trace)
    assert result['score'] == expected, (result, expected)
    return dict(recording=str(directory), controls=controls, expected_score=expected,
                final=result, original_live_score=trace[-1]['metrics']['score'],
                live_replay_agreement=not changed_steps, changed_steps=changed_steps,
                original_scorer_sha256=original_scorer, evaluated_scorer_sha256=evaluated_scorer,
                scope='Rescoring unchanged native evidence' if rescore else 'Exact native live-score verification')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--expected', type=float, choices=(0.,.5,1.), required=True)
    parser.add_argument('--rescore', action='store_true',
                        help='Allow a changed read-only scorer; report differences without rewriting the receipt')
    args = parser.parse_args()
    print(json.dumps(validate(args.recording, args.expected, args.rescore), indent=2))
