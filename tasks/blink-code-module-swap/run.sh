#!/usr/bin/env bash
# Runs one trial of the blink-code module swap task in the simulator.
#
#   ./run.sh demo          the scripted demo (both arms), reaches 1.0
#   ./run.sh wrong_socket  swaps module 2 instead of module 3 (scores 0.0 zero-gate)
#   ./run.sh no_diag       swaps module 3 with spare without ever pressing DIAG (scores 0.75)
#   ./run.sh no_verify     initial DIAG press and swap, skips verify press (scores 0.75)
#   ./run.sh no_swap       presses DIAG only, no module swap (scores 0.25)
#   ./run.sh settle        parts rest for a few seconds, checks stability (scores 0.0)
#   ./run.sh dream         Dream-1 baseline policy
#
# Output goes to $OUT/<mode> (default ~/yam-eval): log, frames, and trace.jsonl.
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="While you hold the white DIAG button the status lamp blinks the number of the failed socket (count the dots beside each socket); replace that module with the teal spare, then hold DIAG again until the lamp stays green."

if ! command -v inspect-robots >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source /data/IsaacLab-Arena/yam.env 2>/dev/null || source "$HOME/IsaacLab-Arena/yam.env"
fi

case "$MODE" in
  settle) export DEMO_SETTLE_ONLY=1; POLICY=(--policy yam_scripted -P hold_s=8); STEPS=400 ;;
  demo) POLICY=(--policy yam_scripted); STEPS=1500 ;;
  wrong_socket) export DEMO_WRONG_SOCKET=1; POLICY=(--policy yam_scripted); STEPS=1500 ;;
  no_diag) export DEMO_NO_DIAG=1; POLICY=(--policy yam_scripted); STEPS=1500 ;;
  no_verify) export DEMO_NO_VERIFY=1; POLICY=(--policy yam_scripted); STEPS=1500 ;;
  no_swap) export DEMO_NO_SWAP=1; POLICY=(--policy yam_scripted); STEPS=1500 ;;
  dream) POLICY=(--policy dream); STEPS=3000 ;;
  *) echo "usage: $0 demo|wrong_socket|no_diag|no_verify|no_swap|settle|dream" >&2; exit 2 ;;
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
