#!/bin/bash
# The proof runs on the final code, one at a time: demo seeds 0 (video) and 1, the plug-pulled run, the careless and
# part-done variants, nothing done, then Dream-1 once (inside flock ~/dream.lock, via ~/dream-retry.sh).
# Usage: bash dev/queue.sh <prefix>
p=$1
cd "$(dirname "$0")/.." || exit 1
FRAMES=1 bash dev/run.sh "$p-a" yam_scripted 0 800
bash dev/run.sh "$p-unplug" yam_scripted 0 1000 DEMO_VARIANT=unplug
bash dev/run.sh "$p-b2" yam_scripted 1 800
bash dev/run.sh "$p-careless" yam_scripted 0 800 DEMO_VARIANT=careless
bash dev/run.sh "$p-cordonly" yam_scripted 0 800 DEMO_VARIANT=cordonly
bash dev/run.sh "$p-nothing" yam_scripted 0 800 DEMO_VARIANT=nothing
bash ~/dream-retry.sh "$PWD" kettle-cord-off-edge 1200 3
