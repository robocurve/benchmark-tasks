#!/usr/bin/env bash
# make_video.sh <run_dir> <views_dir> <mode: demo|dream> [slow window "t0-t1" in s] [hold_s]
# Builds <run_dir>/<mode>.mp4 from the run's first trial (lot A), using only frames the run's cameras recorded:
#   1. a 3 x 2 grid at real speed: top camera | side view 2x (level with the seat) | view from beyond the motor /
#      left wrist | right wrist | side view;
#   2. the slow window, if given, at half speed: side view 2x | view from beyond the motor;
#   3. the last frame of the side view 2x and the view from beyond the motor, held hold_s seconds.
# It also writes <run_dir>/setup.png (top camera, side view 2x, view from beyond the motor, at the first frame) when
# mode is demo. The stock files are kept as <mode>_stock.mp4 and setup_stock.png. Nothing is drawn in but labels.
set -euo pipefail
run=$1; views=$2; mode=$3; slow=${4:-}; hold=${5:-3}
v=$views/trial0
top=$(find "$run/log" -name '*top_cam.mp4' | sort | head -1)
left=${top%top_cam.mp4}left_cam.mp4
right=${top%top_cam.mp4}right_cam.mp4
[ -f "$run/${mode}_stock.mp4" ] || cp "$run/$mode.mp4" "$run/${mode}_stock.mp4"
tmp=$(mktemp -d)
for c in side_cam back_cam; do
  ffmpeg -loglevel error -y -framerate 20 -i "$v/$c/%05d.jpg" -c:v libx264 -crf 14 -pix_fmt yuv420p "$tmp/$c.mp4"
done
lab() { echo "drawtext=text='$1':x=8:y=8:fontsize=20:fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=4"; }
T=640x360
ZOOM="crop=640:360:320:180"  # the middle of the 1280 x 720 side view: twice the scale
ffmpeg -loglevel error -y -i "$top" -i "$tmp/side_cam.mp4" -i "$tmp/back_cam.mp4" -i "$left" -i "$right" \
  -filter_complex "
[0:v]fps=20,scale=$T,setsar=1,$(lab 'top camera')[a];
[1:v]split=2[s1][s2];
[s1]$ZOOM,setsar=1,$(lab 'seat, side view 2x')[b];
[2:v]scale=$T,setsar=1,$(lab 'from beyond the motor')[c];
[3:v]fps=20,scale=$T,setsar=1,$(lab 'left wrist')[d];
[4:v]fps=20,scale=$T,setsar=1,$(lab 'right wrist')[e];
[s2]scale=$T,setsar=1,$(lab 'side view')[f];
[a][b][c]hstack=3[r1];[d][e][f]hstack=3[r2];[r1][r2]vstack=2,pad=1920:1080:0:180:black,setsar=1,format=yuv420p[out]" \
  -map "[out]" -shortest -c:v libx264 -crf 20 -pix_fmt yuv420p "$tmp/grid.mp4"
parts=("$tmp/grid.mp4")
if [ -n "$slow" ]; then
  t0=${slow%-*}; t1=${slow#*-}
  ffmpeg -loglevel error -y -ss "$t0" -to "$t1" -i "$tmp/side_cam.mp4" -ss "$t0" -to "$t1" -i "$tmp/back_cam.mp4" \
    -filter_complex "
[0:v]$ZOOM,scale=960:540,setsar=1,setpts=2*PTS,$(lab "seat, side view 2x - half speed, from ${t0} s")[a];
[1:v]scale=960:540,setsar=1,setpts=2*PTS,$(lab 'from beyond the motor - half speed')[b];
[a][b]hstack=2,pad=1920:1080:0:270:black,fps=20,setsar=1,format=yuv420p[out]" \
    -map "[out]" -shortest -c:v libx264 -crf 20 -pix_fmt yuv420p "$tmp/slow.mp4"
  parts+=("$tmp/slow.mp4")
fi
last=$(ls "$v/side_cam" | sort | tail -1)
ffmpeg -loglevel error -y -loop 1 -t "$hold" -framerate 20 -i "$v/side_cam/$last" \
  -loop 1 -t "$hold" -framerate 20 -i "$v/back_cam/$last" -filter_complex "
[0:v]$ZOOM,scale=960:540,setsar=1,$(lab 'end of the run - seat, side view 2x')[a];
[1:v]scale=960:540,setsar=1,$(lab 'from beyond the motor')[b];
[a][b]hstack=2,pad=1920:1080:0:270:black,setsar=1,fps=20,format=yuv420p[out]" \
  -map "[out]" -c:v libx264 -crf 20 -pix_fmt yuv420p "$tmp/hold.mp4"
parts+=("$tmp/hold.mp4")
inputs=(); filt=""
for k in "${!parts[@]}"; do inputs+=(-i "${parts[$k]}"); filt+="[$k:v]"; done
ffmpeg -loglevel error -y "${inputs[@]}" -filter_complex "${filt}concat=n=${#parts[@]}:v=1[out]" \
  -map "[out]" -c:v libx264 -pix_fmt yuv420p -movflags +faststart -crf 21 "$run/$mode.mp4"
if [[ $mode == demo ]]; then
  [ -f "$run/setup_stock.png" ] || cp "$run/setup.png" "$run/setup_stock.png"
  first=$(ls "$v/side_cam" | sort | head -1)
  ffmpeg -loglevel error -y -i "$run/setup_stock.png" -i "$v/side_cam/$first" -i "$v/back_cam/$first" \
    -i "$v/side_cam/$first" -filter_complex "
[0:v]scale=$T,setsar=1,$(lab 'top camera')[a];
[1:v]$ZOOM,setsar=1,$(lab 'seat, side view 2x')[b];
[2:v]scale=$T,setsar=1,$(lab 'from beyond the motor')[c];
[3:v]scale=$T,setsar=1,$(lab 'side view')[d];
[a][b]hstack=2[r1];[c][d]hstack=2[r2];[r1][r2]vstack=2[out]" -map "[out]" -frames:v 1 "$run/setup.png"
fi
rm -rf "$tmp"
ls -la "$run/$mode.mp4"; ffprobe -v error -show_entries format=duration -of csv=p=0 "$run/$mode.mp4"
