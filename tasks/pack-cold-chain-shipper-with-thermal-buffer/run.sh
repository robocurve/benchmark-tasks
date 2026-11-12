#!/usr/bin/env bash
# Runs one trial of the pack-cold-chain-shipper-with-thermal-buffer task in the simulator.
#
#   ./run.sh demo                   scripted demo (full pack)
#   ./run.sh no_buffer_violation    ablation: gel brick without the buffer layer
#   ./run.sh brick_only             ablation: only the gel brick is packed
#   ./run.sh buffer_without_carton  ablation: buffer placed without the carton
#   ./run.sh settle                 parts rest, arms hold at home 8 s, predicted 0.00
#   ./run.sh dream                  Dream-1 baseline policy, predicted 0.00
# Set TASK_VARIANT=A|B for the other layout. Output goes to $OUT/<mode> (default ~/yam-eval).
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="Pack the medication shipper for transport."

if ! command -v inspect-robots >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source /data/IsaacLab-Arena/yam.env 2>/dev/null || source "$HOME/IsaacLab-Arena/yam.env"
fi

case "$MODE" in
  demo) POLICY=(--policy yam_scripted); STEPS=2600 ;;
  no_buffer_violation) export DEMO_NO_BUFFER_VIOLATION=1; POLICY=(--policy yam_scripted); STEPS=2600 ;;
  brick_only) export DEMO_BRICK_ONLY=1; POLICY=(--policy yam_scripted); STEPS=2600 ;;
  buffer_without_carton) export DEMO_BUFFER_WITHOUT_CARTON=1; POLICY=(--policy yam_scripted); STEPS=2600 ;;
  settle) export DEMO_SETTLE_ONLY=1; POLICY=(--policy yam_scripted -P hold_s=8); STEPS=400 ;;
  dream) POLICY=(--policy dream); STEPS=2500 ;;
  *) echo "usage: $0 demo|no_buffer_violation|brick_only|buffer_without_carton|settle|dream" >&2; exit 2 ;;
esac

mkdir -p "$OUT/$MODE"
export TASK_TRACE="$OUT/$MODE/trace.jsonl"
rm -f "$TASK_TRACE"

inspect-robots run --embodiment yam_arena "${POLICY[@]}" \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score \
  --max-steps "$STEPS" --store-frames --log-dir "$OUT/$MODE"

echo
echo "trace: $TASK_TRACE ($(wc -l < "$TASK_TRACE") lines); last line:"
tail -n 1 "$TASK_TRACE"
