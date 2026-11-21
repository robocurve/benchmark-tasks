"""Save the stored frames of chosen steps as PNGs (top, left wrist and right wrist cameras side by side).
usage: frame_png.py <run folder> <out prefix> <step> [<step> ...]"""
import glob
import struct
import sys
import zlib

import numpy as np

run, prefix = sys.argv[1], sys.argv[2]


def files(cam):
    return sorted(glob.glob(f"{run}/frames/*/*{cam}_*.npy"), key=lambda f: int(f.rsplit("_", 1)[1][:-4]))


cams = [files(c) for c in ("top_cam", "left_cam", "right_cam")]
for s in [int(a) for a in sys.argv[3:]]:
    s = min(s, min(len(c) for c in cams) - 1)
    img = np.hstack([np.load(c[s])[..., :3].astype(np.uint8) for c in cams])
    raw = b"".join(b"\x00" + row.tobytes() for row in img)

    def chunk(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d))

    with open(f"{prefix}{s}.png", "wb") as fh:
        fh.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", img.shape[1], img.shape[0], 8, 2, 0, 0, 0))
                 + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))
    print(f"{prefix}{s}.png", img.shape)
