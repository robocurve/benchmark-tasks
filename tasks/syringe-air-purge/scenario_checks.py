"""Controlled fluid-model comparisons, NOT robot rollouts or hardware trials.

python scenario_checks.py --output validation/scenario_results.json
All variants start from Fluid(), use measured-input-shaped updates, and call the
unchanged production scorer. Endpoint booleans are explicit test assumptions.
"""
import argparse
import hashlib
import gzip
import json
from pathlib import Path
from fluid import Fluid, AREA, INITIAL_AIR, INITIAL_SAMPLE

EXPECTED = {'full': 1., 'horizontal': 0., 'overpush': 0., 'partial': .3,
            'medium_partial': .6, 'retract_repurge': 1., 'no_return': 0.,
            'no_release': 0., 'no_lift': 0., 'none': 0.}


def run(mode, dt=.05):
    f = Fluid()
    trace = []
    t = 0.

    def update(q, up):
        nonlocal t
        f.update(q, up, dt)
        t += dt
        assert abs(f.sample + f.lost_sample - INITIAL_SAMPLE) < 1e-8
        assert abs(f.air + f.expelled_air - f.aspirated - INITIAL_AIR) < 1e-8
        trace.append(dict(t=round(t, 6), piston_ml=q*AREA, upward=up,
                          air_ml=f.air, lost_ml=f.lost_sample,
                          aspirated_ml=f.aspirated, failed=bool(f.failed)))

    up = 0. if mode in {'horizontal', 'none'} else .766
    for _ in range(round(2/dt)):
        update(0., up)
    if mode != 'none':
        target = {'overpush': 4., 'partial': 1., 'medium_partial': 1.25}.get(mode, 1.55)

        def stroke(to_ml):
            start = f.last_q
            n = round(2/dt)
            for k in range(1, n+1):
                update(start + (to_ml/AREA-start)*k/n, up)

        stroke(target)
        if mode == 'retract_repurge':
            stroke(1.15)
            stroke(1.55)
    gates = dict(released=mode!='no_release', parked=mode!='no_return', lifted=mode!='no_lift')
    score = f.score(**gates)
    assert score == EXPECTED[mode], (mode, score, EXPECTED[mode])
    if mode == 'retract_repurge':
        assert f.air <= .05 and f.lost_sample <= .30 and f.aspirated > .20 and not f.failed
    return dict(mode=mode, expected_score=EXPECTED[mode], measured_model_score=score,
                passed=True, evidence_level='fluid_model_only', endpoint_gate_assumptions=gates,
                final=dict(air_ml=f.air, sample_loss_ml=f.lost_sample,
                           aspirated_ml=f.aspirated, failed=bool(f.failed)), trace=trace)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    result = dict(evidence_level='fluid_model_only', control_hz=20,
                  limitation='No robot motion, contacts, lift/return/release measurement, video, or hardware validation. Endpoint gates are supplied by the test.',
                  source_sha256={p: hashlib.sha256((root/p).read_bytes()).hexdigest()
                                 for p in ['fluid.py', 'scenario_checks.py']},
                  cases=[run(mode) for mode in EXPECTED])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    for case in result['cases']:
        trace_path = args.output.parent / (case['mode'] + '.model_trace.json.gz')
        trace_bytes = gzip.compress(json.dumps(case.pop('trace'), separators=(',', ':')).encode(), mtime=0)
        trace_path.write_bytes(trace_bytes)
        case['trace_file'] = trace_path.name
        case['trace_sha256'] = hashlib.sha256(trace_bytes).hexdigest()
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    for case in result['cases']:
        print(f"{case['mode']}: {case['measured_model_score']} (fluid model only)")


if __name__ == '__main__':
    main()
