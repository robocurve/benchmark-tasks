#!/usr/bin/env bash
# Double-hook probe: repeats and controls on rams_probe.usda. Usage: bash rams_batch.sh
set -u
cd "$(dirname "$0")"
source ~/IsaacLab-Arena/yam.env
run() { local name=$1; shift; python rams_run.py --out "out/rams/$name" "$@" > "out/rams/$name.log" 2>&1; }
mkdir -p out/rams
md5sum rams_probe.usda rams_model.py rams_run.py rams_build.py > out/rams/inputs.md5
run guards-b --case guards &
run guards-swing+3 --case guards --swing 3 &
run guards-swing-3 --case guards --swing -3 &
wait
run noguards-a --case noguards &
run noguards-swing+3 --case noguards --swing 3 &
run bottomless-a --case bottomless &
wait
echo BATCH-DONE
