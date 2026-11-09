"""Replay a free prototype's archived scorer; this does not certify task novelty."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import zipfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    run = args.run.resolve()
    manifest = json.loads((run/'source_manifest.json').read_text())
    archive = run/'task-source-at-run.zip'
    with zipfile.ZipFile(archive) as z:
        assert all(hashlib.sha256(z.read(name)).hexdigest() == digest
                   for name, digest in manifest.items())
    sys.path.insert(0, str(archive))
    from prototype.scoring import Evaluator
    trace = json.loads((run/'evaluation_trace.json').read_text())
    native = [p for p in run.glob('adhoc_*.json') if '.live.' not in p.name]
    assert len(native) == 1
    result = json.loads(native[0].read_text())
    assert result['status'] == 'success' and not result['error']
    assert result['results']['errored_trials'] == 0
    assert trace['monitor_ready'] and trace['monitor_error'] is None
    health = json.loads((run/'image_health.json').read_text())
    assert health['valid'] and health['error'] is None
    evaluator = Evaluator()
    monitor = SimpleNamespace(events=[], samples=0, ready=True, error=None)
    events = trace['contacts']
    assert all(a['physics_sample'] <= b['physics_sample'] for a, b in zip(events, events[1:]))
    cursor = 0
    full_steps = []
    for i, row in enumerate(trace['steps']):
        assert row['step'] == i
        assert row['monitor_valid'], (i, 'invalid scoring/contact clock; do not report task performance')
        monitor.samples = row['contact_samples']
        while cursor < len(events) and events[cursor]['physics_sample'] <= monitor.samples:
            monitor.events.append(events[cursor])
            cursor += 1
        replay = evaluator.update(row['pose'], monitor)
        for key in ('score', 'complete', 'monitor_valid', 'still', 'release_support'):
            assert replay[key] == row[key], (i, key, replay[key], row[key])
        if row['score'] == 1:
            full_steps.append(i + 1)
    steps = len(trace['steps'])
    frames = {cam: len(list((run/'frames').rglob(f'*_{cam}_*.npy')))
              for cam in ('top_cam', 'left_cam', 'right_cam')}
    assert all(n == steps + 1 for n in frames.values())
    assert health['frames'] == steps + 1
    final = trace['steps'][-1]
    assert final['score'] == result['results']['metrics']['task_score']
    config = json.loads((run/'configuration.json').read_text())
    visual = config['mode'].endswith('_visual')
    report = dict(valid_free_prototype=config['mode']!='dream', valid_run=True, run=str(run), steps=steps,
                  simulation_s=steps/20, frames_per_camera=frames,
                  archived_source_files_verified=len(manifest), raw_scorer_replay=True,
                  final_score=final['score'], final_error_mm=1000*final['distance_m'],
                  first_full_action=full_steps[0] if full_steps else None,
                  contact_samples=trace['contact_samples'],
                  monitored_violation_reports=len(trace['violations']),
                  contact_monitor_valid=True, image_health_valid=True,
                  final_release_support=final['release_support'], final_still=final['still'],
                  controller_mode=config['mode'],
                  initial_grasp=config.get('initial_grasp'),
                  scoring_revision=final.get('scoring_revision','outcome-v1'),
                  scene_revision=config.get('scene_revision','historical-open-board'),
                  caveats=['Prototype rubric; not final submission validation.',
                           ('Known-layout approach plus specialized RGB translation feedback; not a general visual policy.'
                            if visual else 'Known-layout open-loop controller, not visual feedback.'),
                           'Contact coverage is diagnostic, not a complete safety certificate.'])
    if config['mode']=='dream':
        assert config['controller_installed'] is False and config['observer_q'] is None
        assert not (run/'controller_trace.json').exists()
        assert 'record=false' in config['argv'] and 'dry_run=false' in config['argv']
        report['caveats'][1]='Real Dream policy rollout; no scripted controller or observer pose installed.'
    if config['mode']=='adaptive_visual':
        control=json.loads((run/'controller_trace.json').read_text())
        decisions=[r for r in control if r.get('view_decision',{} ) and r['view_decision']['choice']]
        expected='recover' if config['initial_grasp']=='central' else 'continue'
        chosen=decisions[0]['selected_branch'] if decisions else None
        report['route_decision']=dict(branch=chosen,
                                      at_wall_s=decisions[0]['wall_time_s'] if decisions else None,
                                      evidence=decisions[0]['view_decision'] if decisions else None,
                                      diagnostic_expected_branch=expected,expectation_matched=chosen==expected,
                                      choice_consistent=all(r['selected_branch'] in (None,chosen) for r in control))
        report['caveats'][1]='Known-layout routes selected by specialized RGB evidence; not autonomous grasp-pose planning.'
    (run/'probe-validation.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
