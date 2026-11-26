"""Verify evaluator output and post-physics evidence from all three scripted trials."""

import argparse
import hashlib
import json
from pathlib import Path

from geometry import ACTION_LIMIT, RESET_ANGLE
from scoring import HOLD_S, MIN_EDGE_LOAD, MIN_TOTAL_EDGE_LOAD


def load(directory, control, expected_score):
    path, = directory.glob('adhoc_*.json')
    log = json.loads(path.read_text(encoding='utf-8'))
    with directory.joinpath('trace.jsonl').open(encoding='utf-8') as stream:
        setup, *actions = map(json.loads, stream)
    assert log['status'] == 'success' and log['results']['errored_trials'] == 0
    assert log['results']['total_trials'] == 1 and log['eval']['policy'] == 'yam_scripted'
    assert setup['control'] == control and abs(setup['hinge_deg'] - RESET_ANGLE) < 1.
    assert 0 < len(actions) == log['stats']['total_steps'] <= ACTION_LIMIT
    score = log['results']['metrics']['task_score']
    assert score == actions[-1]['score'] == expected_score
    substeps = setup['physics_hz'] // setup['control_hz']
    assert all(len(row['samples']) == substeps for row in actions)
    record = dict(
        score=score, actions=len(actions), evaluator_seconds=round(log['stats']['duration_s'], 2),
        simulated_seconds=len(actions) / setup['control_hz'], run_id=path.stem,
        physical_gpu=int(setup['physical_gpu']), termination=log['samples'][0]['termination_reasons'][0],
        initial_hinge_deg=setup['hinge_deg'], physics_hz=setup['physics_hz'], control_hz=setup['control_hz'],
        friction=setup['friction'], final=actions[-1]['metrics'],
    )
    return record, actions


def verify_normal(directory):
    record, actions = load(directory, 'normal', 1.)
    assert record['termination'] == 'success'
    end = record['final']
    assert end['hold_s'] >= HOLD_S and end['bearing'] and end['released']
    assert min(end['edge_load_N']) >= MIN_EDGE_LOAD and sum(end['edge_load_N']) >= MIN_TOTAL_EDGE_LOAD
    crossed = next(row for row in actions if row['metrics']['hinge_deg'] > 180.)
    two_hands = [row for row in actions if row['stage'] in ('pass the hinge through flat', 'form the inverted V')
                 and row['metrics']['arm_normal_N_by_link'][0][0] > 1.
                 and row['metrics']['arm_normal_N_by_link'][1][1] > 1.]
    assert two_hands and two_hands[-1]['metrics']['hinge_deg'] > 238.
    regrasp = [row for row in actions if row['stage'] == 'roll the open right gripper'
               and row['metrics']['arm_normal_N_by_link'][0][0] > 1.
               and sum(row['metrics']['arm_normal_N_by_link'][1]) < 0.05]
    assert regrasp
    transferred = next(row for row in actions if row['stage'] in ('lower the display edge', 'transfer the weight to the desk')
                       and row['metrics']['bearing'])
    record.update(first_past_flat_action=crossed['action'], two_hand_fold_last_action=two_hands[-1]['action'],
                  supported_regrasp_actions=len(regrasp), desk_bearing_before_release_action=transferred['action'],
                  desk_bearing_before_release_N=transferred['metrics']['edge_load_N'])
    return record


def verify_flat(directory):
    record, actions = load(directory, 'flat', 0.)
    end = record['final']
    assert abs(end['hinge_deg'] - 180.) < 7.
    assert max(abs(h) for edge in end['edge_height_mm'] for h in edge) < 20.
    assert end['robot_force_N'] < .05 and not end['shape']
    assert all(row['score'] == 0. for row in actions)
    return record


def verify_held(directory):
    record, actions = load(directory, 'held', 0.)
    end = record['final']
    assert end['shape'] and not end['released'] and end['robot_force_N'] > 1.
    assert sum(end['edge_load_N']) < MIN_TOTAL_EDGE_LOAD
    held = [row for row in actions if row['stage'] == 'keep the tent supported with the grippers'
            and row['metrics']['shape'] and row['metrics']['robot_force_N'] > 1.]
    assert len(held) >= record['control_hz']
    assert all(row['score'] == 0. for row in actions)
    record['supported_tent_actions'] = len(held)
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for option in ('normal', 'flat', 'held', 'out'):
        parser.add_argument(f'--{option}', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    files = ('task.py', 'arena_adapter.py', 'demo.py', 'geometry.py', 'physics_monitor.py', 'scoring.py',
             'assets/laptop.usda')
    results = dict(
        seed=0, action_limit=ACTION_LIMIT,
        normal=verify_normal(args.normal), flat=verify_flat(args.flat), held=verify_held(args.held),
        sha256={name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in files},
    )
    args.out.write_text(json.dumps(results, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    for name in ('normal', 'flat', 'held'):
        print(name, results[name]['score'], results[name]['actions'])


if __name__ == '__main__':
    main()
