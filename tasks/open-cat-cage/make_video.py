"""Join one run's camera videos into one H.264 video: the top camera large, the wrist cameras beside it, without text.

First write the per-camera videos: inspect-robots video LOG.json
Usage (in the Arena environment): python make_video.py FRAMES_DIR OUT.mp4 [--setup SETUP.png]
FRAMES_DIR holds the run's *top_cam.mp4, *left_cam.mp4 and *right_cam.mp4. --setup also writes the top
camera's first frame, the setup screenshot.
"""

from __future__ import annotations

import argparse
import glob
import os

import imageio.v2 as imageio
import numpy as np
from PIL import Image

TOP, WRIST = (854, 480), (426, 240)  # 1280 x 480 in all


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("frames_dir")
    ap.add_argument("out")
    ap.add_argument("--setup")
    ap.add_argument("--fps", type=int, default=20)
    a = ap.parse_args()
    readers = [imageio.get_reader(glob.glob(os.path.join(a.frames_dir, f"*{c}.mp4"))[0]) for c in ("top_cam", "left_cam", "right_cam")]
    writer = imageio.get_writer(a.out, fps=a.fps, codec="libx264", pixelformat="yuv420p", macro_block_size=None,
                                ffmpeg_params=["-movflags", "+faststart", "-crf", "20"])
    n = 0
    for top, left, right in zip(*readers):
        if n == 0 and a.setup:
            Image.fromarray(top).save(a.setup)
        big = np.asarray(Image.fromarray(top).resize(TOP, Image.LANCZOS))
        side = [np.asarray(Image.fromarray(f).resize(WRIST, Image.LANCZOS)) for f in (left, right)]
        writer.append_data(np.concatenate([big, np.concatenate(side, axis=0)], axis=1))
        n += 1
    writer.close()
    print(f"{a.out}: {n} frames, {n / a.fps:.1f} s, {os.path.getsize(a.out) / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
