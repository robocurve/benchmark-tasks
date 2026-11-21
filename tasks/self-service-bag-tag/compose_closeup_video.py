"""Compose the verified continuous rollout with wrist close-ups (recording only).

Left: unmodified top camera. Right column: magnified centre-bottom crop of the right
wrist camera (where the adhesive ends are squeezed) and the full left wrist view.
Every panel comes from the same simulation step; no frame is synthesised or reordered.
Run finalize_demo.py first: this refuses runs that have not passed final review.
"""
import json, sys, subprocess, hashlib
from pathlib import Path
import numpy as np
import imageio.v2 as imageio
import imageio_ffmpeg
from PIL import Image, ImageDraw

run = Path(sys.argv[1]).resolve()
review = json.loads((run / 'final-review.json').read_text())
assert review['passed'], 'Final review did not pass'
frames = run / 'frames'
tops = sorted(frames.glob('top_*.png'))
steps = [int(f.stem.split('_')[-1]) for f in tops]
assert all(b - a == 4 for a, b in zip(steps, steps[1:])), 'Noncontinuous frames'
for s in steps:
    for cam in ('rightwrist', 'leftwrist'):
        assert (frames / f'{cam}_{s:05d}.png').exists(), f'missing {cam} frame {s}'
trace = {r['step']: r for r in json.loads((run / 'result.json').read_text())['trace']}
bond_step = min((k for k, r in trace.items() if r['closure'].get('bonded')), default=None)
bond_n = trace[bond_step]['closure'].get('bond_press_n') if bond_step else None

out = run / 'composite'; out.mkdir(exist_ok=True)
W, H = 640, 352; SW, SH = 320, 176
def label(img, text):
    d = ImageDraw.Draw(img); d.rectangle((0, 0, img.width, 14), fill=(0, 0, 0)); d.text((4, 2), text, fill=(255, 255, 0))
for s in steps:
    top = Image.open(frames / f'top_{s:05d}.png').convert('RGB').resize((W, H))
    rw = Image.open(frames / f'rightwrist_{s:05d}.png').convert('RGB'); w, h = rw.size
    zoom = rw.crop((int(w * .3), int(h * .5), int(w * .7), h)).resize((SW, SH))
    lw = Image.open(frames / f'leftwrist_{s:05d}.png').convert('RGB').resize((SW, SH))
    label(top, f'top camera   step {s}   t={s / 20:.1f} s')
    status = 'sealed' if bond_step is not None and s >= bond_step else 'not sealed'
    label(zoom, f'right wrist (zoomed)  {status}'); label(lw, 'left wrist')
    canvas = Image.new('RGB', (W + SW, H)); canvas.paste(top, (0, 0)); canvas.paste(zoom, (W, 0)); canvas.paste(lw, (W, SH))
    canvas.save(out / f'comp_{s:05d}.png')
comp = sorted(out.glob('comp_*.png'))
manifest = run / 'composite-frames.txt'
manifest.write_text(''.join(f"file '{p}'\nduration 0.2\n" for p in comp))
video = run / 'successful-demo-closeup.mp4'
subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), '-y', '-loglevel', 'error', '-f', 'concat', '-safe', '0', '-i', str(manifest),
                '-r', '5', '-c:v', 'libx264', '-crf', '24', '-maxrate', '2M', '-bufsize', '4M', '-map_metadata', '-1',
                '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(video)], check=True)
reader = imageio_ffmpeg.read_frames(str(video), pix_fmt='rgb24'); meta = next(reader); n = 0; last = None
for px in reader:
    n += 1; last = np.frombuffer(px, np.uint8).reshape(meta['size'][1], meta['size'][0], 3)
assert n == len(comp), (n, len(comp))
err = float(np.mean(np.abs(last.astype(float) - imageio.imread(comp[-1]).astype(float))))
assert err < 8., 'Encoded terminal frame differs from composed terminal frame'
imageio.imwrite(run / 'closeup-encoded-final-frame.png', last)
(run / 'closeup-video-audit.json').write_text(json.dumps(dict(
    decoded_frames=n, expected_frames=len(comp), duration_seconds=n / meta['fps'], size=meta['size'],
    terminal_frame_mean_pixel_error=err, bond_step=bond_step, bond_press_n=bond_n,
    sha256=hashlib.sha256(video.read_bytes()).hexdigest()), indent=2) + '\n')
print(video)
