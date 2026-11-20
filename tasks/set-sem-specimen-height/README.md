# Set and lock a mounted chip cross-section below a NeoScope height gauge

Use the 15398 base and 18 mm adjustable pillar, M6 nut, and M4 specimen mount on a cold bench. Use a dummy mounted silicon cross-section. The original USD uses an 11 × 4 × 3.5 mm coupon, a gauge face 50 mm above the bench datum, and steadying and wrench handles at +103 and −104 mm Y. The custom slide stop is at 8 mm. The lock is modeled as a physical brake force that holds the measured height after the nut turns with both controls touched. [Manufacturer reference](https://www.tedpella.com/JEOL-neoscope_html/Specimen-mounts-and-holders.aspx)

## Physical measurements

Zero a depth gauge or dial height stand from the orange reference face. Award 0.35 when the highest coupon point lies 0.5–1.0 mm below it. Film the nut angle against a witness mark during tightening. Award 0.65 when the nut turns at least 16.6° during support and the height shifts at most 0.25 mm. Release and record the height stand for one continuous second for full credit. Repeat after returning the pillar to its low stop and the nut to its open stop. Record the settled gap before approaching.

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

Update the measured fixture registration and actual nut torque before an OEM comparison. The proposed gap and angle are task thresholds.

The scripted Lambda trial scored **1.00** in `adhoc_752e4965.json`.

Dream-1 scored **0.0** in the saved evaluation `adhoc_6f2e6c6a.json`.
