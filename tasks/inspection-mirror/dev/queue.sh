#!/usr/bin/env bash
# Run each "label steps args..." line (e.g. dev/all_runs.txt) of a queue file one at a time, each only once at most one other sim holds the GPU.
q=$1
here=$(cd "$(dirname "$0")/.." && pwd)
while IFS= read -r line; do
  [[ -z $line || $line == \#* ]] && continue
  while [[ $(nvidia-smi --query-compute-apps=pid --format=csv,noheader | wc -l) -gt 1 ]]; do sleep 20; done
  echo "=== $(date -u +%H:%M:%S) start: $line"
  # shellcheck disable=SC2086
  bash "$here/dev/run.sh" $line 2>&1 | grep -E "\[inspect|OUT=|task_score|Error|error|Traceback|score" | grep -v "moving or" | tail -400
  echo "=== $(date -u +%H:%M:%S) done: $line"
done < "$q"
