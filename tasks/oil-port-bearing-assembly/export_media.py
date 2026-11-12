"""Export saved original camera frames; no added camera or reconstructed scene."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import cv2
import numpy as np

CAMERAS = ('top_cam', 'left_cam', 'right_cam')
WIDTH, HEIGHT = 640, 576


def compose_frame(images, index):
    """Large overview above two wrists, preserving full RGB fields and aspect ratios."""
    assert all(rgb.shape == (176, 320, 3) and rgb.dtype == np.uint8
               for rgb in images.values())
    canvas = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    # Exact 2x pixel replication changes display size, not the recorded observation.
    canvas[24:376] = np.repeat(np.repeat(images['top_cam'], 2, axis=0), 2, axis=1)
    canvas[400:, :320] = images['left_cam']
    canvas[400:, 320:] = images['right_cam']
    for label, position in [(f'Third-person   {index/20:.2f}s', (8, 18)),
                            ('Left wrist', (8, 394)), ('Right wrist', (328, 394))]:
        cv2.putText(canvas, label, position, cv2.FONT_HERSHEY_SIMPLEX,
                    .5, (255, 255, 255), 1, cv2.LINE_AA)
    return canvas


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--name', default='trial')
    p.add_argument('--epoch', type=int, default=0)
    args = p.parse_args()
    cams = CAMERAS
    files = {cam: sorted((args.run/'frames').rglob(f'*-e{args.epoch}_{cam}_*.npy')) for cam in cams}
    count = len(files[cams[0]])
    assert count and all(len(f) == count for f in files.values())
    args.output.mkdir(parents=True, exist_ok=True)
    video = args.output/(args.name+'.mp4')
    ffmpeg = subprocess.Popen(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
        '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{WIDTH}x{HEIGHT}', '-r', '20', '-i', '-',
        '-an', '-c:v', 'libx264', '-crf', '18', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(video)],
        stdin=subprocess.PIPE)
    snapshots = {0, min(32,count-1), count//4, count//2, 3*count//4, count-1}
    try:
        for i in range(count):
            assert len({files[cam][i].stem.rsplit('_', 1)[-1] for cam in cams}) == 1
            canvas = compose_frame({cam: np.load(files[cam][i]) for cam in cams}, i)
            ffmpeg.stdin.write(canvas.tobytes())
            if i in snapshots:
                assert cv2.imwrite(str(args.output/f'{args.name}-frame-{i:04d}.png'), cv2.cvtColor(canvas,cv2.COLOR_RGB2BGR))
    finally:
        ffmpeg.stdin.close()
    assert ffmpeg.wait() == 0
    report = dict(source_run=str(args.run.resolve()), frames_per_camera=count, fps=20,
                  video=str(video.resolve()), bytes=video.stat().st_size,
                  sha256=hashlib.sha256(video.read_bytes()).hexdigest(),
                  layout='overview-above-wrists', resolution=[WIDTH, HEIGHT],
                  content_rects_xywh={'top_cam': [0,24,640,352],
                                      'left_cam': [0,400,320,176],
                                      'right_cam': [320,400,320,176]},
                  note='Original RGB: overview2x pixel replication above native wrists. '
                       'No crop, aspect-ratio change or temporal edits; labels outside images; includes initial frame.')
    (args.output/(args.name+'-origin.json')).write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
