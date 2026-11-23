#!/usr/bin/env bash
# Created 11_16_26 (America/New_York). One scripted trial on a cloud host through new-task-drafts/cloud_run.sh.
#   dev/cloud.sh <host> <label> [ENV=VALUE ...]   (run from the repository root)
# Writes the full per-step trace to ~/yam-eval/bikecage-<label>.txt on the host (copied back with the logs).
set -euo pipefail
HOST=$1; LABEL=$2; shift 2
ENVS="$*"
INSTR="Put the bottle in the bike's bottle cage so it stays put."
new-task-drafts/cloud_run.sh "$HOST" put-a-bottle-in-a-side-entry-bike-cage \
  "env BIKE_DEBUG=1 $ENVS flock \$SIM_LOCK inspect-robots run --embodiment yam_bike_cage -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --policy yam_scripted --no-prompt --instruction \"$INSTR\" --scorer task_score --max-steps 1400 --seed 0 --store-frames --log-dir ~/yam-eval/bikecage-$LABEL > ~/yam-eval/bikecage-$LABEL.txt 2>&1; grep -E 'task_score|Traceback|Error' ~/yam-eval/bikecage-$LABEL.txt | tail -5"
