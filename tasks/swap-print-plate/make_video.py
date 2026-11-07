"""Join one run's camera videos into one H.264 video: the top camera large, the wrist cameras beside it, without text.

First write the per-camera videos: inspect-robots video LOG.json
Usage (in the Arena environment): python make_video.py FRAMES_DIR OUT.mp4 [--setup SETUP.png]
FRAMES_DIR holds scene-0-e0_top_cam.mp4, scene-0-e0_left_cam.mp4 and scene-0-e0_right_cam.mp4. --setup also writes the top
camera's first frame, the setup screenshot.
"""

from __future__ import annotations

import argparse
import os

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

TOP, WRIST = (854, 480), (426, 240)  # 1280 x 480 in all


def _label(frame: np.ndarray, text: str) -> np.ndarray:
    """A small speed label in the top left corner."""
    image = Image.fromarray(frame)
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default(size=30)
    draw.rounded_rectangle((12, 12, 74, 56), radius=8, fill=(20, 20, 24))
    draw.text((22, 15), text, font=font, fill=(255, 255, 255))
    return np.asarray(image)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("frames_dir")
    ap.add_argument("out")
    ap.add_argument("--setup")
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--fast-until", type=float, default=0.0, help="play the run's first seconds at 2x, with a 2x label")
    a = ap.parse_args()
    readers = [imageio.get_reader(os.path.join(a.frames_dir, f"scene-0-e0_{c}.mp4")) for c in ("top_cam", "left_cam", "right_cam")]
    writer = imageio.get_writer(a.out, fps=a.fps, codec="libx264", pixelformat="yuv420p", macro_block_size=None,
                                ffmpeg_params=["-movflags", "+faststart", "-crf", "20"])
    n = 0
    frames = 0
    for top, left, right in zip(*readers):
        fast = frames < a.fast_until * a.fps
        frames += 1
        if fast and frames % 2 == 0:
            continue
        if n == 0 and a.setup:
            Image.fromarray(top).save(a.setup)
        big = np.asarray(Image.fromarray(top).resize(TOP, Image.LANCZOS))
        side = [np.asarray(Image.fromarray(f).resize(WRIST, Image.LANCZOS)) for f in (left, right)]
        frame = np.concatenate([big, np.concatenate(side, axis=0)], axis=1)
        if fast:
            frame = _label(frame, "2x")
        writer.append_data(frame)
        n += 1
    writer.close()
    print(f"{a.out}: {n} frames, {n / a.fps:.1f} s, {os.path.getsize(a.out) / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
