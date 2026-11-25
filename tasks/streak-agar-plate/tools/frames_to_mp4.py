"""Tile a yam_arena run's stored camera frames (left_cam, top_cam, right_cam) into an MP4, with close-up panels
(latch_cam.py) from directories of <step>.npy images if given: one goes beside the three, several in a row under them
(a directory of subdirectories counts as one per subdirectory, in name order).
  python frames_to_mp4.py <frames dir of one run> <out.mp4> [fps] [extra frames dir ...]"""
import glob
import os
import subprocess
import sys
import tempfile

import numpy as np
from PIL import Image, ImageDraw, ImageFont

src, out = sys.argv[1], sys.argv[2]
fps = sys.argv[3] if len(sys.argv) > 3 else "10"
dirs = []
for d in sys.argv[4:]:
    subs = sorted(p for p in glob.glob(f"{d}/*") if os.path.isdir(p))
    dirs += subs if subs else [d]
extras = [f for f in (sorted(glob.glob(f"{d}/*.npy")) for d in dirs) if f]
# LABELS=1: each extra panel captioned with its directory's name (a leading "<n>_" for ordering dropped)
labels = [os.path.basename(d.rstrip('/')).split('_', 1)[-1].replace('_', ' ') for d in dirs if glob.glob(f"{d}/*.npy")]
LABELS = os.environ.get("LABELS") == "1"


def caption(img, text):
    im = Image.fromarray(img)
    try:
        font = ImageFont.load_default(size=max(14, im.height // 18))
    except TypeError:
        font = ImageFont.load_default()
    d = ImageDraw.Draw(im)
    d.rectangle((0, 0, d.textlength(text, font=font) + 12, font.size + 10 if hasattr(font, 'size') else 22), fill=(0, 0, 0))
    d.text((6, 4), text, fill=(255, 255, 255), font=font)
    return np.asarray(im)
cams = ("left_cam", "top_cam", "right_cam")
files = {c: sorted(glob.glob(f"{src}/*~{c}_*.npy")) for c in cams}
n = min(len(v) for v in files.values())


def fit(img, w, h):
    return img if img.shape[:2] == (h, w) else np.asarray(Image.fromarray(img).resize((w, h)))


with tempfile.TemporaryDirectory() as tmp:
    for k in range(n):
        panels = [np.load(files[c][k]) for c in cams]
        h, w = panels[0].shape[:2]
        extra = [np.load(e[min(k, len(e) - 1)]) for e in extras]
        if LABELS:
            extra = [caption(e, t) for e, t in zip(extra, labels)]
        if len(extra) == 1:
            frame = np.concatenate(panels + [fit(extra[0], w, h)], axis=1)
        else:
            frame = np.concatenate(panels, axis=1)
            if extra:
                ew = frame.shape[1] // len(extra)
                eh = int(round(ew * h / w)) // 2 * 2
                frame = np.concatenate([frame, np.concatenate([fit(e, ew, eh) for e in extra], axis=1)[:, :frame.shape[1]]], axis=0)
        Image.fromarray(frame).save(f"{tmp}/{k:05d}.png")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", fps, "-i", f"{tmp}/%05d.png", "-vf",
                    "pad=ceil(iw/2)*2:ceil(ih/2)*2", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart", out], check=True)
print("MP4", out, n, "frames")
