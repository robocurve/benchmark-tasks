"""Replay native contact/pose evidence and optionally export the original camera views."""
import argparse
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile
import numpy as np
from pxr import Usd
from disposition_score import Grader


def load(path):
    return json.loads(path.read_text())


def verify(run, media=None):
    cfg = load(run / 'configuration.json')
    report, = [p for p in run.glob('adhoc_*.json') if '.live.' not in p.name]
    official = load(report)
    assert official['status'] == 'success' and official['results']['errored_trials'] == 0
    n = official['stats']['total_steps']
    assert n == cfg['steps'] and n > 0
    manifest = load(run / 'source-manifest.json')
    with ZipFile(run / 'source.zip') as source:
        assert all(hashlib.sha256(source.read(k)).hexdigest() == v for k, v in manifest.items())
        assert json.loads(source.read('task.json'))['instruction'] == cfg['canonical_instruction']
    for name in ('disposition_layout.py', 'disposition_score.py'):
        assert hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest() == manifest[name]
    native = load(run / 'contact-contract.json')
    assert len(native['sensors']) == 18 and native['physics_hz'] == 240
    stage = Usd.Stage.Open(str(run / 'scene.usda'), load=Usd.Stage.LoadNone)
    grader = Grader(stage, native['paths'], native['filters'])
    with np.load(run / 'physics.npz', allow_pickle=False) as physics:
        index = physics['observation_samples']
        streams = [physics[k] for k in ('poses','robot_contact_counts','object_contact_counts','object_contact_forces')]
        assert physics['physics_hz'] == 240 and list(physics['paths']) == native['paths']
        assert physics['joint_pos'].shape == (n+1,14) and np.isfinite(physics['joint_pos']).all()
    assert len(index) == n+1 and (np.diff(index) == 12).all()
    assert all(len(x) == index[-1]+1 and np.isfinite(x).all() for x in streams)
    trace = [json.loads(line) for line in (run / 'handoff-trace.jsonl').read_text().splitlines()]
    assert len(trace) == n
    milestones = {}
    for i, (a,b) in enumerate(zip(index[:-1]+1,index[1:]+1)):
        grader.update(*(x[a:b] for x in streams),None)
        assert trace[i] == dict(action=i+1, **grader.metrics), f'Score replay diverged at action {i+1}'
        milestones.setdefault(str(grader.score),i+1)
    assert grader.score == official['results']['metrics']['task_score']
    assert grader.metrics == load(run / 'handoff-result.json')['final']
    actions_file, = (run / 'actions').rglob('*.jsonl')
    header,*commands = [json.loads(x) for x in actions_file.read_text().splitlines()]
    assert header['action_dim'] == 14 and len(commands) == n and all(row['t'] == i for i,row in enumerate(commands))
    actions = np.asarray([row['action'] for row in commands])
    assert actions.shape == (n,14) and np.isfinite(actions).all()
    metrics = {}
    for i, path in enumerate(native['paths']):
        count = streams[1][index[0]+1:,:,native['filters'].index(path)]
        touched = count.reshape(n,12,18).any(axis=(1,2))
        xyz = streams[0][index[0]:,i,:3]
        metrics[path.rsplit('/',1)[-1]] = dict(robot_contact_actions=int(touched.sum()),
                max_base_motion_mm=round(1000*float(np.linalg.norm(xyz-xyz[0],axis=1).max()),3))
    health = load(run / 'image_health.json')
    assert health['valid'] and health['frames'] == n+1
    camera_stats = {}
    for cam in ('top','left','right'):
        frames = sorted((run / 'frames').rglob(f'*_{cam}_cam_*.npy'))
        assert len(frames) == n+1
        minimum = 255.
        for i,f in enumerate(frames):
            assert f.stem.endswith(f'_{i:06d}')
            rgb = np.load(f,allow_pickle=False)
            assert rgb.shape == (176,320,3) and rgb.dtype == np.uint8
            minimum = min(minimum,float(rgb.std(axis=(0,1)).mean()))
        camera_stats[cam] = round(minimum,2)
    model = None
    if cfg['mode'] == 'dream':
        assert not cfg['controller_installed'] and 'record=false' in cfg['argv']
        assert not (run / 'stock-decision.json').exists()
        transcript, = official['samples'][0]['policy_transcripts']
        assert transcript['model']=='dreamlabs/dream-1-beta' and transcript['dry_run'] is False
        assert (transcript['control_hz'], transcript['rows_per_request'], transcript['preroll_steps']) == (20,48,32)
        requests = transcript['requests']
        assert [r['step'] for r in requests] == list(range(32,n,48))
        model = dict(name=transcript['model'],chunks=len(requests),attempts=sum(r['attempts'] for r in requests))
    result = dict(mode=cfg['mode'],score=grader.score,actions=n,simulation_seconds=n/20,frames_per_camera=n+1,
                  physics_samples=len(streams[0]),first_action_by_score=milestones,final=grader.metrics,
                  motion_and_contact=metrics,minimum_spatial_std_per_camera=camera_stats,model=model)
    if media:
        from media import image_at, export_video
        media.mkdir(parents=True,exist_ok=True)
        if cfg['mode'] == 'setup':
            image_at(run,0).save(media / 'setup.png')
        else:
            export_video(run,media/('dream.mp4' if model else 'fullscore.mp4'),n+1)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--media-dir',type=Path)
    args = parser.parse_args()
    result = verify(args.run.resolve(),args.media_dir)
    print(json.dumps(result,indent=2))
