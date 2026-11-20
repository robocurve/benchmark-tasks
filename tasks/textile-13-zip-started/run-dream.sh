#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
source /data/IsaacLab-Arena/yam.env
out=${1:-runs/dream-new}
if [ -e "$out" ]; then
    printf 'Choose a new output directory: %s already exists.\n' "$out" >&2
    exit 2
fi
mkdir -p "$out"
result=0
timeout --signal=INT --kill-after=20s 25m python -u task.py --dream --max-steps 1200 --output "$out" >"$out/console.log" 2>&1 || result=$?
printf '%s\n' "$result" > "$out/exit.txt"
shopt -s nullglob
logs=("$out"/textile-*.json)
if (( ${#logs[@]} != 1 )); then
    tail -30 "$out/console.log" | cut -c1-230
    exit 1
fi
inspect-robots video "${logs[0]}" --out "$out/replay" --fps 20 >>"$out/console.log" 2>&1
top=("$out/replay"/*~top_cam.mp4)
left=("$out/replay"/*~left_cam.mp4)
right=("$out/replay"/*~right_cam.mp4)
ffmpeg -v error -y -i "${top[0]}" -i "${left[0]}" -i "${right[0]}" \
    -filter_complex '[0:v]scale=640:-2[t];[1:v]scale=320:-2[l];[2:v]scale=320:-2[r];[l][r]hstack[w];[t][w]vstack' \
    -c:v libx264 -pix_fmt yuv420p -movflags +faststart "$out/dream.mp4"
ffmpeg -v error -y -i "${top[0]}" -frames:v 1 "$out/setup.png"
mv "${logs[0]}" "$out/dream-eval.json"
python - "$out" <<'PY'
from pathlib import Path
import json,sys
folder=Path(sys.argv[1])
evaluation=json.loads((folder/"dream-eval.json").read_text())
sample,=evaluation["samples"]
name,score=next(iter(sample["reduced"].items()))
result={
    "score":score, "goal_met":score == 1.0,
    "status":evaluation["status"],"sample_status":sample["status"],
    "steps":evaluation["stats"]["total_steps"],"control_hz":evaluation["eval"]["embodiment_info"]["control_hz"],
    "instruction":sample["instruction"],"termination_reasons":sample["termination_reasons"],
    "model":evaluation["eval"].get("policy_checkpoint"),
}
(folder/"dream-score.json").write_text(json.dumps(result,indent=2)+"\n")
print(json.dumps(result,indent=2))
PY
exit "$result"
