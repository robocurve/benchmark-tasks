"""Replay a recorded full-score robot action sequence in the unchanged scene."""
import argparse
import hashlib
import json
from pathlib import Path

import imageio.v2 as imageio
import numpy as np


def replay(recording,output,video=True):
    manifest=json.loads((recording/'replay_manifest.json').read_text())
    source_dir=Path(__file__).parent
    for name,expected in manifest['source_sha256'].items():
        path=source_dir/name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
            raise RuntimeError(f'Scene or controller source changed: {name}')
    with np.load(recording/'actions.npz',allow_pickle=False) as saved:
        actions=saved['actions']
    if actions.shape!=(manifest['steps'],14):
        raise RuntimeError(f'Invalid action shape: {actions.shape}')

    from inspect_robots import Scene
    from inspect_robots.types import Action
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    import yam_arena.inspect_embodiment as ie
    from isaaclab.app import AppLauncher
    if ie._APP is None:
        ie._APP=AppLauncher(headless=True,device='cuda:0',enable_cameras=True).app
        __import__('atexit').register(ie._close_app_at_exit)
    from yam_arena.embodiment import BimanualYamEmbodiment
    old_init=BimanualYamEmbodiment.__init__
    def sensor_init(self,*args,**kwargs):
        kwargs['contact_sensors']=True
        old_init(self,*args,**kwargs)
    BimanualYamEmbodiment.__init__=sensor_init

    output.mkdir(parents=True,exist_ok=False)
    emb=yam_arena_embodiment(task=str((source_dir/'single_screw_task.py').resolve()),
        object_jitter=manifest['object_jitter'],control_hz=manifest['control_hz'],
        cam_width=640,cam_height=352,top_cam='demo')
    obs=emb.reset(Scene(id='single_screw_repair',instruction=(
        'Inspect the loose drawer-knob screw. Fit the matching bit into the switched-off powered screwdriver, '
        'tighten the screw with that tool, then release the stopped tool in its stand.')))
    imageio.imwrite(output/'initial_top_cam.png',np.asarray(obs.images['top_cam']))
    frames=[];trace=[];task=emb.task.module
    for step,command in enumerate(actions,1):
        obs=emb.step(Action(data=np.asarray(command,dtype=np.float32))).observation
        if video and step%3==0:
            frames.append(np.asarray(obs.images['top_cam']))
        if step%5==0 or step==len(actions):
            m=task.MECHANISM
            trace.append({'step':step,'score':float(task.score(None)),
                'motor_enabled':bool(m.get('motor_enabled',False)),
                'pressure_m':float(m.get('pressure_m',0.)),
                'screw_feed_lower_m':float(m.get('screw_feed_lower_m',0.)),
                'powered_lower_m':float(m.get('powered_lower_m',0.))})
    result={'scope':'Recorded robot-action replay from reset','steps':len(actions),
            'reference_score':manifest['score'],'score':float(task.score(None)),
            'success':bool(task.success(None)),'source_hashes_match':True}
    (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    with (output/'score_trace.jsonl').open('w') as f:
        for row in trace:f.write(json.dumps(row)+'\n')
    if video:imageio.mimsave(output/'replay.mp4',frames,fps=20/3,codec='libx264',output_params=['-pix_fmt','yuv420p'])
    print(json.dumps(result),flush=True)
    if not result['success']:raise RuntimeError('Recorded actions did not reproduce full score')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('recording',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--no-video',action='store_true')
    args=parser.parse_args()
    replay(args.recording,args.output,video=not args.no_video)
