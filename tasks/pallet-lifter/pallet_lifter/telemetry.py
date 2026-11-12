"""CPU-only control-step diagnostics, using the current scorer without changes.

Contact reports describe events received since the preceding sample, not a
continuous support-force measurement. Heights are sampled at control rate.
"""

import json
import sys

import numpy as np

from pallet_lifter.scoring import RAIL_HEIGHT, evaluate, parked, seated

FORMER_GATE = .060


def sample(states, history, top, *, step, time_s, contacts, score,
           held_steps=0, terminated=False, termination_reason=None):
    objects = {}
    for name in ('pallet', 'lifter'):
        s = states[name]
        objects[name] = {
            key: np.asarray(getattr(s, key)).tolist()
            for key in ('pos', 'low', 'high', 'quat_xyzw', 'lin_vel', 'ang_vel')
        }
        objects[name].update(bottom_height_m=float(s.low[2] - top),
                             resting=bool(s.resting()))
    bottom = objects['pallet']['bottom_height_m']
    return {
        'kind': 'step', 'step': step, 'time_s': float(time_s),
        'objects': objects, 'contacts_since_previous_sample': list(contacts),
        'history': dict(vars(history)), 'score': float(score),
        'current_scorer_score': evaluate(states, history, top),
        'pallet_seated': seated(states['pallet'], top),
        'lifter_parked': parked(states['lifter'], top),
        'rail_clearance_m': bottom - RAIL_HEIGHT,
        'below_former_gate': bool(states['pallet'].low[2] < top + FORMER_GATE),
        'held_steps': int(held_steps), 'terminated': bool(terminated),
        'termination_reason': termination_reason,
    }


def summarize(rows):
    rows = [r for r in rows if r.get('kind') == 'step']
    if not rows:
        raise ValueError('No control-step telemetry found')
    final = rows[-1]
    bottoms = np.array([r['objects']['pallet']['bottom_height_m'] for r in rows], dtype=float)
    finite = bool(np.all(np.isfinite(bottoms)))
    return {
        'sampling': 'reset plus control steps; no between-step height guarantee',
        'contact_semantics': 'reported events, not continuous support force',
        'samples': len(rows), 'final_step': final['step'],
        'contiguous_from_reset': [r['step'] for r in rows] == list(range(len(rows))),
        'max_sampled_pallet_bottom_m': float(bottoms.max()) if finite else None,
        'all_sampled_bottoms_below_60_mm': finite and all(r['below_former_gate'] for r in rows),
        'observed_rail_clearance': any(r['rail_clearance_m'] > 0 for r in rows),
        'tool_contact_history': final['history']['tool_contact'],
        'invalid_ever': any(r['history']['invalid'] for r in rows),
        'scores_match_current_scorer': all(r['score'] == r['current_scorer_score'] for r in rows),
        'final_score': final['score'], 'final_seated': final['pallet_seated'],
        'final_parked': final['lifter_parked'], 'final_open_clear': final['history']['open_clear'],
        'final_held_steps': final['held_steps'],
        'runtime_success': final['terminated'] and final['termination_reason'] == 'success',
    }


if __name__ == '__main__':
    with open(sys.argv[1]) as stream:
        print(json.dumps(summarize(json.loads(line) for line in stream), indent=2))
