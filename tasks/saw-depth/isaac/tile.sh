#!/usr/bin/env bash
# Tile three camera videos of one trial side by side (same timing): one H.264 video plus the first (setup) and last
# (final) frames as PNG. Needs ffmpeg/ffprobe on the host; reads the videos only (no change to any run directory).
#   tile.sh <out_dir> <name> <top.mp4> <left_wrist.mp4> <right_wrist.mp4>
# Output: <out_dir>/<name>_tiled_top_left_right.mp4, setup_<name>_tiled_top_left_right.png,
#         final_<name>_tiled_top_left_right.png
set -euo pipefail
out=$1; name=$2; top=$3; left=$4; right=$5
mkdir -p "$out"
base="${name}_tiled_top_left_right"
timeout 1800 ffmpeg -loglevel error -y -i "$top" -i "$left" -i "$right" \
  -filter_complex "[0:v][1:v][2:v]hstack=inputs=3,pad=ceil(iw/2)*2:ceil(ih/2)*2[v]" -map "[v]" \
  -c:v libx264 -pix_fmt yuv420p -crf 20 -movflags +faststart "$out/$base.mp4"
timeout 300 ffmpeg -loglevel error -y -i "$out/$base.mp4" -frames:v 1 "$out/setup_$base.png"
timeout 300 ffmpeg -loglevel error -y -sseof -0.2 -i "$out/$base.mp4" -frames:v 1 -update 1 "$out/final_$base.png"
ffprobe -v error -select_streams v:0 -show_entries stream=codec_name,width,height,nb_frames -of csv=p=0 "$out/$base.mp4"
