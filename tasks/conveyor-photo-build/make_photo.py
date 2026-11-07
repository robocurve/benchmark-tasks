"""Takes the photo on the card: the finished model, photographed in the simulator from in front (studio.py).

    python tasks/conveyor-photo-build/make_photo.py /path/to/logs     (Isaac Sim, about 1 min on an RTX GPU)

With CONVEYOR_STUDIO=1 the task spawns the four model parts already assembled on the white marks (the extra
parts in the tote, the card blank) and moves the overhead camera to the studio camera; one scripted step renders
it at 1920 x 1080. This script then crops the model, with a margin, to 4:3 by projecting the model's corners
through that camera, and writes photo.png (640 x 480) next to it. The photo is committed, so trials do not need
to run this.
"""
from pathlib import Path
import json
import os
import subprocess
import sys

import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from layout import (BEAM_TOP, GATE_X, GATE_Y, MARK_TOP, MARKS, PARTS, PHOTO_FILE)  # noqa: E402

WIDTH, HEIGHT = 1920, 1080
OUT = (640, 480)
MARGIN = 0.45  # added around the model's projected box, as a share of its size


def model_points() -> np.ndarray:
    """Corners of the box around the finished model and its marks (world, m)."""
    half_beam = PARTS['beam'][0][1] / 2
    xs = (GATE_X - 0.03, GATE_X + 0.03)
    ys = (min(GATE_Y - half_beam, MARKS[1][1] - 0.03), max(GATE_Y + half_beam, MARKS[0][1] + 0.03))
    zs = (MARK_TOP, BEAM_TOP + PARTS['cap'][0][2])
    return np.array([(x, y, z) for x in xs for y in ys for z in zs])


def crop_box(points: np.ndarray) -> tuple[int, int, int, int]:
    """Pixel box (left, top, right, bottom) of the points in the studio camera at WIDTH x HEIGHT, grown, made 4:3."""
    from yam_arena.urdf import quat_xyzw_to_matrix

    from studio import camera

    cam = camera()
    assert cam.convention == 'opengl', cam.convention  # looks along -z, y up
    lens = cam.lens.resized(WIDTH, HEIGHT)
    rot = quat_xyzw_to_matrix(np.asarray(cam.rot_xyzw))
    uv = []
    for p in points:
        q = rot.T @ (p - np.asarray(cam.pos))
        uv.append((lens.cx + lens.fx * q[0] / -q[2], lens.cy - lens.fy * q[1] / -q[2]))
    uv = np.array(uv)
    (u0, v0), (u1, v1) = uv.min(0), uv.max(0)
    cu, cv = (u0 + u1) / 2, (v0 + v1) / 2
    w, h = (u1 - u0) * (1 + MARGIN), (v1 - v0) * (1 + MARGIN)
    w, h = max(w, h * 4 / 3), max(h, w * 3 / 4)
    box = (int(round(cu - w / 2)), int(round(cv - h / 2)), int(round(cu + w / 2)), int(round(cv + h / 2)))
    assert box[0] >= 0 and box[1] >= 0 and box[2] <= WIDTH and box[3] <= HEIGHT, box
    return box


def main():
    out_dir = Path(sys.argv[1] if len(sys.argv) > 1 else 'photo-studio').resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    cli = Path(sys.executable).parent / 'inspect-robots'
    env = dict(os.environ, CONVEYOR_STUDIO='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    subprocess.run([str(cli), 'run', '--embodiment', 'yam_arena', '--policy', 'yam_scripted',
                    '-E', f'task={HERE / "task.py"}', '-E', 'device=cpu', '-E', 'object_jitter=0', '-E', 'control_hz=20',
                    '-E', f'cam_width={WIDTH}', '-E', f'cam_height={HEIGHT}', '--instruction', 'photo studio',
                    '--scorer', 'task_score', '--grader', 'none', '--no-prompt', '--max-steps', '1', '--seed', '0',
                    '--store-frames', '--log-dir', str(out_dir)], env=env, check=True)
    log = sorted(out_dir.glob('adhoc_*.json'), key=os.path.getmtime)[-1]
    frames = Path(json.loads(log.read_text())['stats']['frames_dir'])
    first = sorted(frames.glob('*top_cam_*.npy'))[0]
    image = Image.fromarray(np.load(first)[..., :3].astype(np.uint8))
    assert image.size == (WIDTH, HEIGHT), image.size
    image.save(out_dir / 'studio_top_cam.png')
    photo = image.crop(crop_box(model_points())).resize(OUT, Image.LANCZOS)
    photo.save(HERE / PHOTO_FILE)
    print(f'wrote {HERE / PHOTO_FILE} from {first}')


if __name__ == '__main__':
    main()
