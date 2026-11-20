"""Replay recorded raw native substeps; never infer history from wrapped logs."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from angle_tracking import ContinuousBottomAngles,VERSION,evidence_error,SUBSTEPS


def audit(folder):
    folder=Path(folder)
    hashes=json.loads((folder/'source-sha256.json').read_text())
    for name,digest in hashes.items():
        if hashlib.sha256((folder/'source'/name).read_bytes()).hexdigest()!=digest:
            raise ValueError('captured source hash mismatch: '+name)
    for name in ('angle_tracking.py','angle_step_observer.py','native_angle_binding.py','run.py'):
        if name not in hashes:raise ValueError('angle capture source missing: '+name)
        if hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()!=hashes[name]:
            raise ValueError('auditor module does not match capture: '+name)
    reset=json.loads((folder/'angle-tracking-reset.json').read_text())
    if reset['version']!=VERSION:raise ValueError('wrong capture version')
    t=ContinuousBottomAngles(reset['caps_deg_s']);initial=reset['initial_state']
    if not t.initialize(initial['counter'],initial['time_s'],initial['poses'],initial['velocities']):
        raise ValueError(t.failure)
    rows=0
    with (folder/'trajectory.jsonl').open() as f:
        for rows,line in enumerate(f,1):
            row=json.loads(line);samples=row['angle_substeps']
            if len(samples)!=SUBSTEPS:raise ValueError('incomplete raw substeps')
            for sample in samples:
                if not t.advance(sample['counter'],sample['time_s'],sample['poses'],sample['velocities']):
                    raise ValueError(t.failure)
                np.testing.assert_allclose(t.angles,sample['angles_deg'],atol=1e-9,rtol=0.)
            angles=[row['angles_deg'][n+'_bottom_hinge'] for n in ('north','south','east','west')]
            np.testing.assert_allclose(t.angles,angles,atol=1e-9,rtol=0.)
            error=evidence_error(row['angle_evidence'],angles,row['time'],rows)
            if error:raise ValueError(error)
            if 'score_sample' in row and row['score_sample']['angle_evidence']!=row['angle_evidence']:
                raise ValueError('score evidence differs from trajectory evidence')
    if not rows:raise ValueError('no actual native rows')
    return dict(valid=True,rows=rows,substeps=t.counter,final_angles_deg=t.angles.tolist(),
                trace_sha256=hashlib.sha256((folder/'trajectory.jsonl').read_bytes()).hexdigest(),
                scope='Continuous-angle capture integrity only; not task completion or native visual evidence.')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('folder',type=Path)
    print(json.dumps(audit(parser.parse_args().folder),indent=2))
