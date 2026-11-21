#!/bin/bash
# film.sh <lock: a|b|c>: the filmed demo on the task file itself (not a logging copy), --seed 0 (README Seeds), frames stored, 1,200 steps (the run limit); the top camera rendered after.
set -eu
cd "$(dirname "$0")"
source "${YAM_ENV:-/data/IsaacLab-Arena/yam.env}" >/dev/null 2>&1 || true   # YAM_ENV: the sim host's env file
TASK=$(cd .. && pwd); O=$PWD/out/film
case $1 in a) L=${LOCKS:-$HOME/locks}/sim.lock;; b) L=${LOCKS:-$HOME/locks}/sim.lock.b;; c) L=${LOCKS:-$HOME/locks}/sim.lock.c;; *) echo "lock a|b|c"; exit 2;; esac
[ -e "$O" ] && { echo "REFUSED: $O exists (move it away to film again)"; exit 4; }
mkdir -p "$O"
I=$(python3 -c "import json;print(json.load(open('$TASK/task.json'))['instruction'])")
( cd "$TASK" && sha256sum lancet_fingerstick.py mechanism.py task.json ) > "$O/files.sha256"
while [ "$(nvidia-smi --query-compute-apps=pid --format=csv,noheader | wc -l)" -ge "${SIMCAP:-3}" ]; do sleep 20; done
inspect-robots --version > "$O/version.txt" 2>&1 || true
flock "$L" inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$TASK/lancet_fingerstick.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --seed 0 --max-steps 1200 --store-frames --log-dir "$O" > "$O/run.log" 2>&1 || true
for log in "$O"/*.json; do case $log in *.live.json) ;; *) inspect-robots video "$log" --out "$O/cams" >> "$O/run.log" 2>&1 || true;; esac; done
echo "film $(grep -h task_score "$O/run.log" | tail -1) steps $(python3 -c "import json,glob;f=[x for x in sorted(glob.glob('$O/*.json')) if not x.endswith('.live.json')];print(json.load(open(f[-1]))['stats']['total_steps'])" 2>&1 | tail -1)"
echo RUNDONE film
