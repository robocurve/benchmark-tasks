"""Validate the official result, raw native replay, frame set, and run-time camera monitor."""
import argparse
import json
from pathlib import Path
import numpy as np
from replay import replay
from layout import PHYSICS_HZ


def verify(run):
    read=lambda name: json.loads((run/name).read_text())
    cfg=read('configuration.json')
    final,=[p for p in run.glob('adhoc*.json') if '.live.' not in p.name]
    result=read(final.name)
    assert result['status']=='success' and result['results']['total_trials']==1 and result['results']['errored_trials']==0
    assert result['eval']['seed']==0 and result['eval']['embodiment']=='yam_arena'
    assert cfg['physics_hz']==PHYSICS_HZ and result['eval']['embodiment_info']['control_hz']==20
    assert result['samples'][0]['instruction']==cfg['canonical_instruction']
    evidence=replay(run/'physics.npz')
    assert evidence['actions']==cfg['steps']
    assert evidence['final']==result['results']['metrics']['task_score']
    health=read('image_health.json')
    n=cfg['steps']+1
    assert health['valid'] and health['frames']==n
    frame_dir,=(run/'frames').iterdir()
    for cam in ('top','left','right'):
        sequence=sorted(frame_dir.glob(f'*_{cam}_cam_*.npy'))
        assert len(sequence)==n and [int(p.stem.rsplit('_',1)[1]) for p in sequence]==list(range(n))
        for file in sequence:
            image=np.load(file,mmap_mode='r',allow_pickle=False)
            assert image.shape==(176,320,3) and image.dtype==np.uint8
    with np.load(run/'physics.npz',allow_pickle=False) as trace:
        assert trace['counts'].shape==(n,12,22) and trace['poses'].shape==(n,12,7)
        peak=int(trace['counts'].sum(axis=(1,2)).max())
    if cfg['mode']=='demo':
        votes=[r['heading'] for r in read('wrist-read.json')[-6:]]
        assert votes.count(1)>=5 and evidence['final']==1
    return dict(**evidence,official_score=result['results']['metrics']['task_score'],mode=cfg['mode'],
                frames_per_camera=n,links_checked_for_release=18,peak_contacts_at_sample=peak,
                min_overview_spatial_std=health['minimum_spatial_std'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path)
    print(json.dumps(verify(p.parse_args().run),indent=2))
