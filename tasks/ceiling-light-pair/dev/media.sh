#!/bin/bash
# dev/media.sh RUN: media/demo.mp4 (the six station cameras in a 2x3 grid, plus the overview) and media/setup.png
# (the overview, and A's and B's top cameras below it) from dev/logs/RUN, a --store-frames run, in the active
# Isaac Lab-Arena environment. Step 0 is left out: the overview camera is placed at the reset, after it.
D=$(cd "$(dirname "$0")/.." && pwd)
RUN=$1
T=$(mktemp -d)
mkdir -p "$D/media" "$T/frames/run"
for f in "$D"/dev/logs/"$RUN"/frames/*/*.npy; do
  case "$f" in *_000000.npy) ;; *) ln -s "$f" "$T/frames/run/" ;; esac
done
yam-pair-video "$T/frames" "$D/media/demo.mp4"
python3 - "$T/frames/run" "$D/media/setup.png" <<'PY'
import glob, sys
import numpy as np
from PIL import Image
src, out = sys.argv[1], sys.argv[2]
f = lambda cam, k: np.load(sorted(glob.glob(f"{src}/*_{cam}_{k:06d}.npy"))[0])[..., :3].astype(np.uint8)
ov = Image.fromarray(f("overview_cam", 1))
tops = [Image.fromarray(f(c, 1)) for c in ("a_top_cam", "b_top_cam")]   # A left, B right, as in the overview
w = ov.width
tops = [t.resize((w // 2, round(t.height * (w // 2) / t.width))) for t in tops]
canvas = Image.new("RGB", (w, ov.height + tops[0].height), "white")
canvas.paste(ov, (0, 0))
canvas.paste(tops[0], (0, ov.height))
canvas.paste(tops[1], (w // 2, ov.height))
canvas.save(out)
print(out, canvas.size)
PY
rm -rf "$T"
ls -la "$D/media"
