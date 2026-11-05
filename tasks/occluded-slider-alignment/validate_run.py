"""Validate real physics evidence, including failed policies and repeated resets."""
import argparse
import json
from pathlib import Path


def validate(root):
    def read(name):
        return json.loads((root/name).read_text())
    finals = [p for p in root.glob('adhoc_*.json') if not p.name.endswith('.live.json')]
    assert len(finals) == 1, 'expected one final Inspect log'
    run = json.loads(finals[0].read_text())
    cfg, trace = read('effective_config.json'), read('evaluation_trace.json')
    assert run['status'] in ('success', 'completed') and not run.get('error'), 'execution error'
    assert not run['results']['errored_trials']
    assert len(run['samples']) == 1
    sample = run['samples'][0]
    assert sample['instruction'] == cfg['canonical_instruction'], 'instruction mismatch'
    assert not cfg['config_only'] and cfg['scoring_version'] == 'visual-contact-v2'
    episodes = trace['episodes']
    assert len(episodes) == cfg['epochs'] == run['results']['total_trials']
    assert 'GPU contact filter for collider' not in (root/'console.log').read_text()
    total, results = 0, []
    for index, ep in enumerate(episodes):
        steps = ep['steps']
        assert ep['epoch'] == index and ep['physics_executed'] and 0 < len(steps) <= cfg['steps']
        assert ep['contact_callback_error'] is None and ep['contact_callback_count'] >= 12*len(steps)
        assert all(s['measurement']['instrumented'] for s in steps)
        reasons = sorted({reason for s in steps for reason in s['invalid_reasons']})
        assert not {'missing_contact_instrumentation', 'nonfinite_state'} & set(reasons)
        score = steps[-1]['score']
        assert sample['epochs'][index]['task_score'] == score
        actions = list((root/'actions').rglob(f'*-e{index}.jsonl'))
        assert len(actions) == 1 and len(actions[0].read_text().splitlines())-1 == len(steps)
        frames = {c: len(list((root/'frames').rglob(f'*-e{index}_{c}_*.npy')))
                  for c in ('top_cam', 'left_cam', 'right_cam')}
        assert all(n == len(steps)+1 for n in frames.values())
        reset = read(f'resets/epoch-{index:03d}/setup_capture.json')
        assert reset['action_count'] == 0
        assert all(c['policy'] == [176, 320, 3] for c in reset['cameras'].values())
        if cfg['mode'] == 'demo':
            assert score == 1 and steps[-1]['complete'], 'demo did not complete'
            assert not reasons and ep['robot_slider_reports'] > 0
            assert all(s['measurement']['released'] and s['error_m'] <= .008 and
                       s['measurement']['speed'] <= .005 for s in steps[-20:])
        elif cfg['mode'] in ('setup', 'stability'):
            assert len(steps) == cfg['steps'] and score == 0 and not reasons
            assert all(abs(s['measurement']['displacement']) <= .001 and
                       s['measurement']['speed'] <= .005 for s in steps)
        elif cfg['mode'] == 'contact-probe':
            assert score == 0 and 'prohibited_collision' in reasons
        elif cfg['mode'] == 'dream':
            assert run['eval']['policy'] == 'dream' and not cfg['oracle_enabled']
            assert cfg['epochs'] == 1 and ep['demo_record_until_s'] is None
            transcript = sample['policy_transcripts'][index]
            assert transcript['requests'] and not transcript['dry_run']
            assert transcript['model'] == 'dreamlabs/dream-1-beta'
            assert transcript['control_hz'] == 20 and transcript['rows_per_request'] == 48
            # A partial run is evidence, but not a completed formal-budget trial.
            assert score == 1 or len(steps) == cfg['steps'], 'model stopped before its budget'
        results.append(dict(epoch=index, score=score, steps=len(steps), frames=frames,
                            final_error_mm=steps[-1]['error_m']*1000, safety_reasons=reasons,
                            reset_slider_pose=reset['slider_pose'], reset_joint_pos=reset['joint_pos']))
        total += len(steps)
    assert total == run['stats']['total_steps']
    if len(results) > 1:
        for r in results[1:]:
            assert max(abs(a-b) for a,b in zip(r['reset_slider_pose'],results[0]['reset_slider_pose'])) < 1e-5
            assert max(abs(a-b) for a,b in zip(r['reset_joint_pos'],results[0]['reset_joint_pos'])) < 1e-3
    return dict(valid=True, mode=cfg['mode'], total_steps=total, episodes=results,
                scoring_version=cfg['scoring_version'], wall_seconds=run['stats']['duration_s'],
                formal_dream=(cfg['mode']=='dream' and cfg['steps']==2400))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path)
    args=p.parse_args()
    try:
        result=validate(args.run)
    except Exception as error:
        result=dict(valid=False,error_type=type(error).__name__)
        if isinstance(error,AssertionError):
            result['check']=str(error)
    (args.run/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if result['valid'] else 1


if __name__=='__main__':
    raise SystemExit(main())
