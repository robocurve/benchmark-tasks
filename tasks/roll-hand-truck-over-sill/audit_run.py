"""Verify recorded scores, resets, source snapshots and delivered camera frames."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scorer import Evidence, Scorer


def audit(root, require_full=False):
    process = json.loads((root/'process-result.json').read_text())
    assert process['exit_code'] == 0 and 'error' not in process['statuses'], 'Run failed'
    for name, digest in json.loads((root/'source-manifest.json').read_text()).items():
        assert hashlib.sha256((root/'source'/name).read_bytes()).hexdigest() == digest, name
    frame_dir, = (root/'inspect/frames').iterdir()
    reports = []
    previous_reset = None
    episodes = sorted((root/'evidence').glob('episode-*'))[:process.get('episodes',999)]
    for number, episode in enumerate(episodes):
        rows = [json.loads(line) for line in (episode/'trace.jsonl').read_text().splitlines()]
        grade = Scorer()
        for i, row in enumerate(rows):
            assert row['step'] == i, 'Missing or reordered step'
            score = grade.update(Evidence(**row['evidence']))
            assert (score,grade.dwell,grade.invalid) == (row['score'],row['dwell'],row['invalid']), 'Grade mismatch'
        assert rows and rows[0]['score'] == 0 and rows[0]['dwell'] == 0, 'Unearned initial score'
        result = json.loads((episode/'result.json').read_text())
        assert (len(rows),grade.score,grade.invalid) == (result['steps'],result['score'],result['invalid'])
        if require_full:
            assert grade.score == 1. and grade.dwell >= 10 and not grade.invalid, 'Full score missing'
        reset = json.loads((episode/'reset-state.json').read_text())
        assert all(np.count_nonzero(v['velocity']) == 0 for v in reset.values()), 'Nonzero reset velocity'
        if previous_reset is not None:
            assert reset == previous_reset, 'Prop reset differs between episodes'
        previous_reset = reset
        robot = json.loads((episode/'reset-robot-state.json').read_text())
        assert all(np.count_nonzero(v[key]) == 0 for v in robot.values()
                   for key in ['root_velocity','joint_velocity']), 'Nonzero robot reset velocity'
        cameras = {'top_cam':'overhead_cam','left_cam':'left_wrist_cam','right_cam':'right_wrist_cam'}
        for sdk, camera in cameras.items():
            native = sorted((episode/camera).glob('*.png'))
            assert len(native) == len(rows)+1, 'Incomplete native frame sequence'
            expected = {f'{i:05d}.png' for i in range(len(rows)+1)}
            assert {p.name for p in native} == expected, 'Missing native frame'
            frames = sorted(frame_dir.glob(f'hand_truck-e{number}_{sdk}_*.npy'))
            assert len(frames) == len(rows)+1, 'Incomplete policy frame sequence'
            for i in sorted({0,min(140,len(rows)-1),len(rows)-1,len(rows)}):
                delivered = np.load(frame_dir/f'hand_truck-e{number}_{sdk}_{i:06d}.npy')
                resized = np.asarray(Image.open(episode/camera/f'{i:05d}.png').resize((320,176),Image.Resampling.BILINEAR))
                assert delivered.shape == (176,320,3) and np.array_equal(delivered,resized), 'Policy frame mismatch'
        reports.append({'episode':number, 'steps':len(rows), 'terminal_score':grade.score,
                        'maximum_score':max(row['score'] for row in rows), 'invalid':grade.invalid,
                        'dwell':grade.dwell, 'max_cart_pitch_deg':max(abs(row['evidence']['pitch_deg']) for row in rows),
                        'reset_velocities_zero':True, 'policy_frame_samples_match_native':True})
    assert reports, 'No episode evidence'
    reset_check = json.loads((root/'reset-check.json').read_text()) if (root/'reset-check.json').exists() else None
    return {'episodes':reports,'source_snapshots_verified':True,'all_recorded_grades_match':True,
            'reset_check':reset_check}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run',type=Path)
    parser.add_argument('--require-full',action='store_true')
    args = parser.parse_args()
    print(json.dumps(audit(args.run,args.require_full),indent=2))
