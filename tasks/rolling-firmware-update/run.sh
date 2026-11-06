#!/usr/bin/env bash
# Runs one trial of the rolling firmware update task in the simulator.
#
#   ./run.sh demo           scripted bimanual rolling update (predicted 1.00)
#   ./run.sh no_switch      attempts to update active unit A -> OUTAGE (predicted 0.00)
#   ./run.sh release_early  door released after 2.0 s -> update cancelled (predicted 0.25)
#   ./run.sh door_only      door held open >= 18 mm for 5 s, no update press (predicted 0.25)
#   ./run.sh settle         parts rest at home, checks stability (predicted 0.00)
#   ./run.sh dream          Dream-1 baseline policy
#
# Output goes to $OUT/<mode> (default ~/yam-eval).
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="Update both units to the new firmware without ever turning the load lamp red: a unit updates only while its orange service door is held open and its yellow UPDATE button has been pressed, and the white SWITCH button moves the load between units."

if ! command -v inspect-robots >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source /data/IsaacLab-Arena/yam.env 2>/dev/null || source "$HOME/IsaacLab-Arena/yam.env"
fi

case "$MODE" in
  settle) export DEMO_SETTLE_ONLY=1; POLICY=(--policy yam_scripted -P hold_s=8); STEPS=400 ;;
  demo) POLICY=(--policy yam_scripted); STEPS=1200 ;;
  no_switch) export DEMO_NO_SWITCH=1; POLICY=(--policy yam_scripted); STEPS=600 ;;
  release_early) export DEMO_RELEASE_EARLY=1; POLICY=(--policy yam_scripted); STEPS=600 ;;
  door_only) export DEMO_DOOR_ONLY=1; POLICY=(--policy yam_scripted); STEPS=600 ;;
  dream) POLICY=(--policy dream); STEPS=2400 ;;
  *) echo "usage: $0 demo|no_switch|release_early|door_only|settle|dream" >&2; exit 2 ;;
esac

mkdir -p "$OUT/$MODE"
export TASK_TRACE="$OUT/$MODE/trace.jsonl"
rm -f "$TASK_TRACE"

EXTRA=()
if [ "${WATCH:-0}" = "1" ]; then
  EXTRA=(-E headless=false)
  export DISPLAY="${DISPLAY:-:1}"
fi

inspect-robots run --embodiment yam_arena "${POLICY[@]}" ${EXTRA[@]+"${EXTRA[@]}"} \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score \
  --max-steps "$STEPS" --store-frames --log-dir "$OUT/$MODE"

echo
echo "trace: $TASK_TRACE ($(wc -l < "$TASK_TRACE") lines); last line:"
tail -n 1 "$TASK_TRACE"
