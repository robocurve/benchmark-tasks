"""Export a fully recorded Inspect episode as anonymous H.264 review media."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('episode', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--name',choices=['demo','model_run'],default='demo')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output must be a new directory')
    result = json.loads((args.episode / 'result.json').read_text())
    cameras = ['overhead_cam', 'left_wrist_cam', 'right_wrist_cam']
    expected = {f'{step:05d}.png' for step in range(result['steps']+1)}
    for camera in cameras:
        if {p.name for p in (args.episode / camera).glob('*.png')} != expected:
            parser.error('Media export requires every frame: use the Inspect runner with every frame saved')
    args.output.mkdir(parents=True)
    command = ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-nostdin',
               '-filter_complex_threads', '1']
    for camera in cameras:
        command += ['-framerate', str(result['control_hz']), '-start_number', '0',
                    '-i', str(args.episode / camera / '%05d.png')]
    video = args.output / (args.name+'.mp4')
    command += ['-filter_complex', '[0:v][1:v][2:v]vstack=inputs=3[v]', '-map', '[v]',
                '-an', '-c:v', 'libx264', '-threads', '2', '-crf', '23', '-pix_fmt', 'yuv420p',
                '-movflags', '+faststart', str(video)]
    subprocess.run(command, check=True)
    setup = args.output / 'setup.png'
    shutil.copyfile(args.episode / 'overhead_cam' / '00000.png', setup)
    if video.stat().st_size > 50_000_000 or setup.stat().st_size > 10_000_000:
        raise RuntimeError('Media exceeds upload limits')
    print(json.dumps({'video_bytes': video.stat().st_size, 'setup_bytes': setup.stat().st_size}))


if __name__ == '__main__':
    main()
