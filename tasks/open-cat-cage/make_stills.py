"""Write stills from a run's stored frames: each camera at its run size and at Dream-1's 320 x 176, and a contact sheet.

Usage: python make_stills.py FRAMES_DIR OUT_DIR PREFIX STEP [STEP ...]     (FRAMES_DIR holds the run's <...>_<camera>_<step>.npy files)
"""

from __future__ import annotations

import glob
import os
import sys

import numpy as np
from PIL import Image

CAMERAS = ("top_cam", "left_cam", "right_cam")


def main() -> None:
    frames, out, prefix, steps = sys.argv[1], sys.argv[2], sys.argv[3], [int(s) for s in sys.argv[4:]]
    os.makedirs(out, exist_ok=True)
    rows = []
    for step in steps:
        row = []
        for cam in CAMERAS:
            hits = glob.glob(os.path.join(frames, f"*{cam}_{step:06d}.npy"))
            if not hits:
                print(f"no frame for {cam} at step {step}")
                continue
            image = Image.fromarray(np.load(hits[0])[..., :3].astype(np.uint8))
            small = image.resize((320, 176), Image.LANCZOS) if image.size != (320, 176) else image
            image.save(os.path.join(out, f"{prefix}_{cam}_{step:06d}_{image.width}x{image.height}.png"))
            if small is not image:
                small.save(os.path.join(out, f"{prefix}_{cam}_{step:06d}_320x176.png"))
            row.append((image, small))
        rows.append(row)
    w, h = rows[0][0][0].size
    sheet = Image.new("RGB", (3 * w, len(rows) * h), "white")
    for r, row in enumerate(rows):
        for c, (image, _) in enumerate(row):
            sheet.paste(image, (c * w, r * h))
    sheet.save(os.path.join(out, f"{prefix}_sheet.jpg"), quality=90)
    print(os.path.join(out, f"{prefix}_sheet.jpg"))


if __name__ == "__main__":
    main()
