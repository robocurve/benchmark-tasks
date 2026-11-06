"""Write stills from a run's stored frames: each camera at its run size and at Dream-1's 320 x 176, and a contact sheet.

Usage: python make_stills.py FRAMES_DIR OUT_DIR STEP [STEP ...]     (FRAMES_DIR holds the run's <...>_<camera>_<step>.npy files)
"""

from __future__ import annotations

import glob
import os
import sys

import numpy as np
from PIL import Image

CAMERAS = ("top_cam", "left_cam", "right_cam")


def main() -> None:
    frames, out, steps = sys.argv[1], sys.argv[2], [int(s) for s in sys.argv[3:]]
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
            small = image.resize((320, 176), Image.LANCZOS)
            image.save(os.path.join(out, f"{cam}_{step:06d}_{image.width}x{image.height}.png"))
            small.save(os.path.join(out, f"{cam}_{step:06d}_320x176.png"))
            row.append((image, small))
        rows.append(row)
    w, h = rows[0][0][0].size
    sheet = Image.new("RGB", (3 * w + 320, sum(1 for _ in rows) * h), "white")
    for r, row in enumerate(rows):
        for c, (image, small) in enumerate(row):
            sheet.paste(image, (c * w, r * h))
        if row:
            sheet.paste(row[0][1], (3 * w, r * h))
    sheet.save(os.path.join(out, "sheet.jpg"), quality=88)
    print(os.path.join(out, "sheet.jpg"))


if __name__ == "__main__":
    main()
