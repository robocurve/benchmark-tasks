"""Write stills from a run's camera videos, for judging the look: top and both wrist cameras at the run's size and at Dream-1's.

First write the per-camera videos: inspect-robots video LOG.json
Usage (in the Arena environment): python stills.py FRAMES_DIR OUT_DIR [--name NAME] [--at SECONDS ...] [--fps 20]
"""

from __future__ import annotations

import argparse
import os

import imageio.v2 as imageio
from PIL import Image

CAMERAS = ("top_cam", "left_cam", "right_cam")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("frames_dir")
    ap.add_argument("out_dir")
    ap.add_argument("--name", default="start")
    ap.add_argument("--at", type=float, nargs="*", default=[0.0])
    ap.add_argument("--fps", type=int, default=20)
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    for cam in CAMERAS:
        reader = imageio.get_reader(os.path.join(a.frames_dir, f"scene-0-e0_{cam}.mp4"))
        frames = {round(t * a.fps): t for t in a.at}
        for i, frame in enumerate(reader):
            if i in frames:
                tag = f"{a.name}_{frames[i]:05.1f}s_{cam}"
                im = Image.fromarray(frame)
                im.save(os.path.join(a.out_dir, f"{tag}_{im.width}x{im.height}.png"))
                im.resize((320, 176), Image.LANCZOS).save(os.path.join(a.out_dir, f"{tag}_320x176.png"))
            if i > max(frames):
                break


if __name__ == "__main__":
    main()
