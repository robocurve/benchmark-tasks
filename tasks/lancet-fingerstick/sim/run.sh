#!/bin/bash
# run.sh <stem> <max steps> <lock: a|b|c> [cli seed, default 0]: one yam_scripted run of sim/arm/<stem>.py (a start-delay copy made by mk.sh),
# no frames. Refuses a copy older than ../lancet_fingerstick.py (rerun mk.sh).
set -eu
cd "$(dirname "$0")"
source "${YAM_ENV:-/data/IsaacLab-Arena/yam.env}" >/dev/null 2>&1 || true   # YAM_ENV: the sim host's env file
J=$(cd arm && pwd); F=$1; M=$2; S=${4:-0}; O=${F}_s$S
case $3 in a) L=${LOCKS:-$HOME/locks}/sim.lock;; b) L=${LOCKS:-$HOME/locks}/sim.lock.b;; c) L=${LOCKS:-$HOME/locks}/sim.lock.c;; *) echo "lock a|b|c"; exit 2;; esac
[ "$J/$F.py" -nt ../lancet_fingerstick.py ] || { echo "STALE: $F.py older than lancet_fingerstick.py; rerun mk.sh"; exit 3; }
I=$(python3 -c "import json;print(json.load(open('$J/task.json'))['instruction'])")
mkdir -p "$PWD/out"
while [ "$(nvidia-smi --query-compute-apps=pid --format=csv,noheader | wc -l)" -ge "${SIMCAP:-3}" ]; do sleep 20; done
T0=$(date +%s)
flock "$L" inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=$J/$F.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "$I" --scorer task_score \
  --seed $S --max-steps $M --log-dir $PWD/out/$O > $PWD/out/$O.log 2>&1 || true
echo "$O $(grep -h task_score out/$O.log | tail -1) steps $(python3 -c "import json,glob;print(json.load(open(sorted(glob.glob('out/$O/*.json'))[-1]))['stats']['total_steps'])" 2>&1 | tail -1) wall $(( $(date +%s) - T0 )) s"
echo RUNDONE $O
