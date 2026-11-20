# Seat paired floating blind-mate coolant ports on a cold compute tray

Use a cold, dry tray on a rail. Each guide jaw floats independently across the lateral axis under spring force. Male pin and guide jaw dimensions are original bench geometry. The initial pitch is 48 mm and offset is 2.2 mm, with ±4 mm float and 38 mm total rail travel. These dimensions are test assumptions. [Manufacturer reference](https://developer.nvidia.com/blog/?p=90182)

## Physical measurements

Attach a dial indicator to each floating jaw and an axial indicator to the tray. At 20 mm or more insertion, award 0.25 per jaw with lateral error at most 0.5 mm. Award 0.75 if both are aligned and the depth gap to the bench seat reference is at most 0.5 mm. Full credit requires that state for one second with the hand clear. Mark the ±4 mm limits on both indicators and watch through insertion. Reset the tray to zero and re-center both floating carriers.

## Run on Lambda

From this task folder, activate the environment and read the instruction and control-step limit.

```bash
source ~/IsaacLab-Arena/.venv/bin/activate
source ~/IsaacLab-Arena/yam.env
instruction=$(jq -er .instruction task.json)
steps=$(jq -er '.run_limit | split(" ")[0] | gsub(",";"") | tonumber' task.json)
```

### Scripted demo

```bash
inspect-robots run --policy yam_scripted -P hold_s=3 --embodiment yam_arena \
  --scorer task_score -E task=arena_task.py -E headless=true -E object_jitter=0 \
  -E control_hz=20 -E cam_width=640 -E cam_height=352 --seed 0 \
  --instruction "$instruction" --max-steps "$steps" --no-prompt --store-frames --log-dir logs/demo
```

### Dream-1

```bash
inspect-robots run --policy dream --embodiment yam_arena --scorer task_score \
  -E task=arena_task.py -E headless=true -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --seed 0 --instruction "$instruction" \
  --max-steps "$steps" --no-prompt --store-frames --log-dir logs/dream
```

Read the final `task_score` under `metrics` in the saved evaluation. Run `python task.py --ik` for the synthetic scoring and reach checks.

## Remaining hardware validation

Confirm the coupler model, manifold pitch, nominal float and load-depth curve on a documented GB200 assembly. The simulation records float limits and binding through travel. An insertion-load comparison needs calibration. [CPC UQD reference](https://www.cpcworldwide.com/Liquid-Cooling/Products/Universal-Quick-Disconnects-UQDs)

The scripted Lambda trial scored **1.00** in `adhoc_19d4eea7.json`.

Dream-1 scored **0.0** in the saved evaluation `adhoc_7d73fd66.json`.
