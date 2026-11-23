#!/usr/bin/env python3
"""Record the ordinary scripted demo, followed by a camera-only inspection.

The trial and its score end before the inspection. Only the rendering camera is
moved afterwards; robot and material states remain unchanged.
"""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import sys
import time

import cv2
import numpy as np
from PIL import Image

HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE))
from inspect_robots import Scene
from demo import WoodDrillDemo
from runtime import WoodDrillEmbodiment, array


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log-dir',type=Path,required=True)
    args=parser.parse_args()
    args.log_dir.mkdir(parents=True,exist_ok=True)
    spec=json.loads((HERE/'task.json').read_text())
    scene=Scene(id='drilling-demo-inspection',instruction=spec['instruction'])
    robot=WoodDrillEmbodiment(task=str(HERE/'task.py'),headless=True,cam_width=320,cam_height=176,
                              control_hz=20,object_jitter=0)
    robot._diagnostics=args.log_dir/'measurements.jsonl'
    hashes={p.name:sha256(p.read_bytes()).hexdigest() for p in HERE.glob('*.py')}
    hashes['tools/inspection_demo.py']=sha256(Path(__file__).read_bytes()).hexdigest()
    (args.log_dir/'source_manifest.json').write_text(json.dumps(hashes,indent=2)+'\n')
    encoder=None
    started=time.monotonic()
    try:
        obs=robot.reset(scene,seed=0)
        policy=WoodDrillDemo()
        policy.bind(robot.info)
        policy.reset(scene)
        movie=args.log_dir/'demo-with-inspection.mp4'
        encoder=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y',
            '-f','rawvideo','-pixel_format','rgb24','-video_size','640x528','-framerate','20',
            '-i','pipe:0','-an','-c:v','libx264','-threads','2','-preset','medium','-crf','20',
            '-pix_fmt','yuv420p','-movflags','+faststart',str(movie)],stdin=subprocess.PIPE)
        def write(observation):
            top=cv2.resize(observation.images['top_cam'],(640,352))
            wrists=np.hstack([observation.images[name] for name in ('left_cam','right_cam')])
            encoder.stdin.write(np.ascontiguousarray(np.vstack([top,wrists]),dtype=np.uint8).tobytes())
        write(obs)
        success=False
        for steps in range(1,2401):
            result=robot.step(policy.act(obs).actions[0])
            obs=result.observation
            write(obs)
            if result.terminated:
                success=result.termination_reason=='success'
                break
        summary={'policy':'wood_drill_demo','score':1. if success else float(result.info.get('score',0.)),
                 'steps':steps,'simulated_seconds':robot._t,'elapsed_seconds':time.monotonic()-started,
                 'final_measurements':robot.latest,'source_hashes':hashes,
                 'post_trial_inspection_frames':0}
        if success:
            # The task has already terminated successfully. Freeze physics and
            # inspect from the open side of the bench using only rendering.
            pose_before=array(robot._base.scene['drill'].data.root_link_pose_w).copy()
            depth_before=robot._material.depth
            camera=robot._base.scene['top_cam']
            camera.set_world_poses_from_view([[.36,.02,.90]],[[.26,-.08,.7775]])
            for _ in range(6):
                robot._base.sim.render()
            camera.update(.05,force_recompute=True)
            rgb=array(camera.data.output['rgb'])[0,...,:3]
            Image.fromarray(rgb).save(args.log_dir/'post-trial-inspection.png')
            inspection=robot._observation()
            assert np.array_equal(pose_before,array(robot._base.scene['drill'].data.root_link_pose_w))
            assert robot._material.depth==depth_before
            for _ in range(80):
                write(inspection)
            summary['post_trial_inspection_frames']=80
            summary['inspection_changes_only_camera']=True
        encoder.stdin.close()
        assert encoder.wait()==0
        encoder=None
        (args.log_dir/'inspection_result.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(json.dumps({k:v for k,v in summary.items() if k not in ('final_measurements','source_hashes')},indent=2),flush=True)
    finally:
        if encoder is not None:
            encoder.stdin.close()
            encoder.wait()
        robot.close()


if __name__=='__main__':
    main()
