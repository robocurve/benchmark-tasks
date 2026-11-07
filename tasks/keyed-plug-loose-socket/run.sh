#!/usr/bin/env bash
# Runs one trial of this task in the simulator and prints its score.
#
#   ./run.sh demo        the scripted demo (both arms), expected to reach 1.0
#   ./run.sh nobrace     the same demo with the left arm idle: shows the block slides away without bracing
#   ./run.sh settle      nothing moves for a few seconds: checks the parts settle and stay put
#   ./run.sh dream       Dream-1 (needs DREAM_API_KEY in ~/.dreamlabs/credentials)
#   WATCH=1 ./run.sh demo   any mode, with the Isaac Sim window shown on the VM's browser desktop (port 6080)
#
# Output goes to $OUT/<mode> (default ~/yam-eval): the run log, camera frames, and trace.jsonl, one line per scorer
# call with the plug and block poses, the plunger, the grippers and the score components. Run it from any folder inside
# the Arena environment (source <ARENA_DIR>/yam.env first).
set -euo pipefail
cd "$(dirname "$0")"

MODE="${1:-demo}"
OUT="${OUT:-$HOME/yam-eval}"
INSTRUCTION="Plug the connector into the socket on the loose block, then let go of both. Do not let the block slide."

if ! command -v inspect-robots >/dev/null 2>&1; then
  # shellcheck disable=SC1090
  source /data/IsaacLab-Arena/yam.env 2>/dev/null || source "$HOME/IsaacLab-Arena/yam.env"
fi

case "$MODE" in
  settle) export DEMO_HOLD_ONLY=1; POLICY=(--policy yam_scripted -P hold_s=8); STEPS=400 ;;
  demo) POLICY=(--policy yam_scripted); STEPS=600 ;;
  nobrace) export DEMO_NO_BRACE=1; POLICY=(--policy yam_scripted); STEPS=600 ;;
  dream) POLICY=(--policy dream); STEPS=1200 ;;
  *) echo "usage: $0 demo|nobrace|settle|dream" >&2; exit 2 ;;
esac

mkdir -p "$OUT/$MODE"
export TASK_TRACE="$OUT/$MODE/trace.jsonl"
rm -f "$TASK_TRACE"

# WATCH=1 draws the Isaac Sim window on the desktop that ./setup.sh --desktop installed (browser, port 6080).
EXTRA=()
if [ "${WATCH:-0}" = "1" ]; then
  EXTRA=(-E headless=false)
  export DISPLAY="${DISPLAY:-:1}"  # the launchable's desktop (Xvfb :1)
fi

inspect-robots run --embodiment yam_arena "${POLICY[@]}" ${EXTRA[@]+"${EXTRA[@]}"} \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score \
  --max-steps "$STEPS" --store-frames --log-dir "$OUT/$MODE"

echo
echo "trace: $TASK_TRACE ($(wc -l < "$TASK_TRACE") lines); last line:"
tail -n 1 "$TASK_TRACE"
