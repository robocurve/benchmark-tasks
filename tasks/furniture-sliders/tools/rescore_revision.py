"""Rescore saved raw measurements with the current scorer, without simulation.

Reject changed physical specifications and scene/reset helpers. A recording-only
camera change requires an explicit flag and is reported with both source hashes.
Never overwrite the original result, measurements or evaluated source snapshot.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys


TASK = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rescore(run, allow_recording_camera_change=False):
    saved = json.loads((run / 'spec-used.json').read_text())
    current = json.loads((TASK / 'spec.json').read_text())
    if {k:v for k,v in saved.items() if k!='score'} != {k:v for k,v in current.items() if k!='score'}:
        raise ValueError('Physical specification changed; cannot reuse this recording')
    unchanged = ('assets.py', 'adapter.py', 'config.py', 'partkit.py', 'floor_geometry.py',
                 'task.py', 'native_audit.py', 'scheduled_runtime.py', 'controller.py',
                 'control_utils.py', 'motion_plan.py')
    for name in unchanged:
        if sha(TASK / name) != sha(run / 'task-bundle' / name):
            raise ValueError('Recorded physical runtime/controller differs: ' + name)
    camera_sources = {'recorded':sha(run/'task-bundle'/'observer.py'),
                      'current':sha(TASK/'observer.py')}
    camera_changed = camera_sources['recorded'] != camera_sources['current']
    if camera_changed and not allow_recording_camera_change:
        raise ValueError('Recording cameras changed; review the diff and explicitly use '
                         '--allow-recording-camera-change to rescore preserved evidence')
    os.environ['FURNITURE_SLIDERS_SPEC'] = str(TASK / 'spec.json')
    sys.path.insert(0, str(TASK))
    import numpy as np
    from scoring import Scorer, points
    from config import FOOT_CENTERS, FLOOR_Z, PAD_H
    result = json.loads((run / 'result.json').read_text())
    scorer = Scorer()
    history = []
    first = None
    stationary_matches_unchanged = True
    with (run / 'measurements.jsonl').open() as f:
        for line in f:
            row = json.loads(line)
            m = row['measurement']
            if first is None:
                first = row
                if m['step'] != 0 or row['action'] is not None:
                    raise ValueError('Recording does not start at reset')
            d = scorer.update(m)
            stationary_matches_unchanged &= [list(p) for p in d['matches']] == row['scoring']['matches']
            feet = points(m['furniture'], FOOT_CENTERS)
            pairs = d['transport_matches']
            history.append({
                'step':m['step'], 'retained':d['transport_retained'], 'transport_m':d['transport_m'],
                'center_offset_m':max((float(np.linalg.norm(feet[i,:2]-m['pads'][p][:2])) for i,p in pairs), default=0.),
                'foot_height_error_m':float(np.max(np.abs(feet[:,2]-(FLOOR_Z+PAD_H)))),
                'pad_height_error_m':max(abs(p[2]-FLOOR_Z) for p in m['pads'].values()),
            })
    if not first or len(history) != result['steps']+1:
        raise ValueError('Recording length does not match declared action count')
    start = scorer.detail['transport_start_step']
    segment = [h for h in history if start is not None and h['step']>=start]
    detail = {k:v for k,v in scorer.detail.items() if k!='foot_corners'}
    return {
        'run':run.name, 'observations':len(history), 'actions':result['steps'],
        'recorded_score':result['scoring']['score'], 'rescored_score':scorer.score,
        'same_final_score':scorer.score==result['scoring']['score'],
        'scoring':detail, 'physical_specification_unchanged':True,
        'unchanged_physical_runtime_files':list(unchanged),
        'recording_camera_source_changed':camera_changed,
        'recording_camera_sha256':camera_sources,
        'camera_change_acknowledged':bool(camera_changed and allow_recording_camera_change),
        'stationary_matches_unchanged_at_every_frame':bool(stationary_matches_unchanged),
        'bare_start':not first['scoring']['matches'] and len(first['measurement']['pads'])==4,
        'qualified_transport_frames':sum(h['retained'] for h in history),
        'final_transport_segment':None if not segment else {
            'start_step':start, 'last_step':history[-1]['step'],
            'first_150mm_step':next((h['step'] for h in segment if h['transport_m']>=.15),None),
            'max_foot_pad_center_offset_m':max(h['center_offset_m'] for h in segment),
            'max_foot_height_error_m':max(h['foot_height_error_m'] for h in segment),
            'max_pad_height_error_m':max(h['pad_height_error_m'] for h in segment),
        },
        'source_sha256':{name:sha(TASK/name) for name in ('scoring.py','spec.json','tools/rescore_revision.py')},
        'original_evidence_sha256':{name:sha(run/name) for name in ('measurements.jsonl','result.json','spec-used.json')},
        'new_simulator_launches':0, 'new_hosted_calls':0,
    }


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--allow-recording-camera-change', action='store_true',
                        help='Acknowledge a reviewed evidence-camera-only diff; physical helpers remain exact.')
    args = parser.parse_args()
    answer = rescore(args.run.resolve(), args.allow_recording_camera_change)
    encoded = json.dumps(answer, indent=2)+'\n'
    if args.out:
        with args.out.open('x') as f:
            f.write(encoded)
    print(encoded)
