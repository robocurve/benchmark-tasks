#!/usr/bin/env bash
# Render a run's cameras to H.264 MP4 and save a setup screenshot (created 11_15_26): dev/media.sh <label> <out-dir>
set -euo pipefail
source ~/IsaacLab-Arena/yam.env
dir=~/yam-eval/cookie-rack-"$1"; out=$2; mkdir -p "$out"
for log in "$dir"/*.json; do inspect-robots video "$log"; done
for f in $(find "$dir" -name "*.mp4"); do
  cam=$(basename "$f" .mp4 | grep -oE "(top|left|right)_cam" || echo cam)
  ffmpeg -y -loglevel error -i "$f" -c:v libx264 -pix_fmt yuv420p -movflags +faststart "$out/cookie-rack-$1-$cam.mp4"
done
python - "$dir" "$out/cookie-rack-$1-setup.png" <<'PY'
import glob, sys
import cv2, numpy as np
first = sorted(glob.glob(sys.argv[1] + "/frames/*/*top_cam_000000.npy"))[0]
cv2.imwrite(sys.argv[2], np.load(first)[..., ::-1])
PY
ls -la "$out"
