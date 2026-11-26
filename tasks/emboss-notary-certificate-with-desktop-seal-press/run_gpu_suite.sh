#!/usr/bin/env bash
TASK_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$TASK_DIR"

source /home/yaoyao/IsaacLab-Arena/yam.env >/dev/null 2>&1

INSTR="Emboss the official notary seal on the certificate card."
EVAL_BASE="$TASK_DIR/eval_results"
rm -rf "$EVAL_BASE"
mkdir -p "$EVAL_BASE"

echo "=== 1. Running Demo 1 (seed 0, 640x352) ==="
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task="$TASK_DIR/emboss_notary_certificate_with_desktop_seal_press.py" \
  -E device=cuda:0 -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=352 \
  --instruction "$INSTR" --scorer task_score --max-steps 450 --seed 0 \
  --no-prompt --store-frames --log-dir "$EVAL_BASE/demo_1" > "$EVAL_BASE/demo_1.log" 2>&1
D1_RC=$?
echo "Demo 1 Exit Code: $D1_RC"
tail -n 25 "$EVAL_BASE/demo_1.log"

echo "=== 2. Running Demo 2 (seed 1, 640x352) ==="
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task="$TASK_DIR/emboss_notary_certificate_with_desktop_seal_press.py" \
  -E device=cuda:0 -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=352 \
  --instruction "$INSTR" --scorer task_score --max-steps 450 --seed 1 \
  --no-prompt --store-frames --log-dir "$EVAL_BASE/demo_2" > "$EVAL_BASE/demo_2.log" 2>&1
D2_RC=$?
echo "Demo 2 Exit Code: $D2_RC"
tail -n 25 "$EVAL_BASE/demo_2.log"

echo "=== 3. Running Dream 1 (seed 0, 400 steps) ==="
ARENA_DIR="/home/yaoyao/IsaacLab-Arena"
"$ARENA_DIR/.venv/bin/inspect-robots-dream-bridge" --warm --port 8768 > "$EVAL_BASE/bridge.log" 2>&1 &
BR_PID=$!
trap 'kill $BR_PID 2>/dev/null || true' EXIT

for i in $(seq 1 60); do
  if grep -q "listening" "$EVAL_BASE/bridge.log" 2>/dev/null; then break; fi
  sleep 1
done

inspect-robots run --embodiment yam_arena --policy dream \
  -P server_url=http://127.0.0.1:8768 \
  -E task="$TASK_DIR/emboss_notary_certificate_with_desktop_seal_press.py" \
  -E device=cuda:0 -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 400 --seed 0 \
  --no-prompt --store-frames --log-dir "$EVAL_BASE/dream_1" > "$EVAL_BASE/dream_1.log" 2>&1
DR_RC=$?

kill $BR_PID 2>/dev/null || true

echo "Dream 1 Exit: $DR_RC"
tail -n 25 "$EVAL_BASE/dream_1.log"

echo "=== Qualification runs completed! ==="
