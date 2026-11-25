#!/bin/bash
# dev/run.sh NAME STEPS [VARIANT] [extra inspect-robots args...]: one scripted-demo run in the active Isaac Lab-Arena
# environment: the demo, or a scorer check with CLP_VARIANT=VARIANT. Output goes to dev/NAME.log and the eval log to
# dev/logs/NAME; prints the monitor lines when done. CLP_* variables pass through. RUN_PREFIX, if set, is put in front
# of the command (for example a GPU job queue).
NAME=$1; STEPS=$2; VARIANT=${3:-}; shift 3
D=$(cd "$(dirname "$0")/.." && pwd)
cd "$D" || exit 1
INSTR=$(python3 -c "import json;print(json.load(open('task.json'))['instruction'])")
CLP_DEBUG=${CLP_DEBUG:-1} CLP_VARIANT="$VARIANT" $RUN_PREFIX inspect-robots run --embodiment yam_pair -E task=task.py \
    --policy yam_pair_scripted --instruction "$INSTR" --scorer pair_task_score --max-steps "$STEPS" \
    --log-dir "$D/dev/logs/$NAME" "$@" > "$D/dev/$NAME.log" 2>&1
grep -E "^\[clp|^\[scr|^\[drv|Traceback|Error|error|score|Exception" "$D/dev/$NAME.log" \
    | grep -v "Error during demuxing\|DeprecationWarning" | cut -c1-900 | tail -"${TAILN:-40}"
