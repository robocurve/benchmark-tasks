"""Read native trial outcomes without rewriting the original Inspect report.

A recorded Dream safety refusal is a distinct outcome. Its score comes from
the last independently scored physical state, not from a made-up report metric.
Other execution errors, including exhausted credits, remain errors.
"""
import json
import math
from pathlib import Path

REFUSAL_PREFIX = (
    'PolicyError: DreamAPI refused the inference request: '
    'DA-4000 BAD_REQUEST (malformed): unsafe instruction in this scene, refused: '
)


def trial_outcome(trial, report, *, allow_recorded_refusal=False):
    if report['status'] == 'success' and report['results']['errored_trials'] == 0:
        value = float(report['results']['metrics']['task_score'])
        assert math.isfinite(value) and 0 <= value <= 1
        return {'metrics': {'task_score': value}, 'outcome': 'scored_trial'}
    assert allow_recorded_refusal, 'execution error; recorded refusals require explicit review'
    assert len(report['samples']) == 1 and report['results']['errored_trials'] == 1
    sample = report['samples'][0]
    error = sample.get('error', '')
    assert error.startswith(REFUSAL_PREFIX) and error[len(REFUSAL_PREFIX):].strip(), 'not a verified Dream safety refusal'
    assert any(t.get('model') == 'dreamlabs/dream-1-beta' and t.get('dry_run') is False
               and t.get('requests') for t in sample.get('policy_transcripts', [])), 'missing actual model inference'
    trial = Path(trial)
    invocation = json.loads((trial/'invocation.json').read_text())
    assert invocation['policy'] == 'dream'
    count = 0
    last = None
    with (trial/'states.jsonl').open() as recording:
        for line in recording:
            state = json.loads(line)
            value = float(state['score_info']['score'])
            assert math.isfinite(value) and 0 <= value <= 1
            count += 1
            last = state
    assert last is not None and 0 < count <= invocation['max_steps'], 'missing recorded physical outcome'
    return {'metrics': {'task_score': float(last['score_info']['score'])},
            'outcome': 'recorded_model_refusal', 'model_refusal': error,
            'score_basis': 'last independently scored native physical state before refusal',
            'scored_state_time_s': last['time_s'], 'recorded_steps': count}


def score_description(demo_score, model):
    value = model['metrics']['task_score']
    if model.get('outcome') == 'recorded_model_refusal':
        return (f'Dream-1 beta refused after {model["recorded_steps"]} recorded steps; '
                f'last native state score: {value:.2f}; scripted native-contact demo: {demo_score:.2f}')
    return f'Dream-1 beta run: {value:.2f}; scripted native-contact demo: {demo_score:.2f}'
