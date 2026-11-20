# Recover a depressurized ATI QC-001 tool plate with support

Use a depressurized QC-001 with the unlock elbow fitting removed. The CAD surrogate has a horizontal guided 1.5 mm stem, a left-side tool grip, and a separate floor-and-tail stand. The piston stroke is a bench choice. The simulated latch applies holding force to the plate until the measured piston crosses 7 mm. The captive key assembly exercises the press and simultaneous support phases. [Manufacturer reference](https://ati.novanta.com/product/qc-1/)

## Physical measurements

Film the tool and the key at 60 fps or faster against a millimetre reference. Award 0.25 for continuous support for at least 0.20 s before release. Award 0.65 when the key crosses the calibrated release line and a side camera or displacement probe measures at most 2 mm downward movement in the next 0.30 s. Unlocking without support or exceeding the drop limit gives zero. Award 1.00 when the released plate is within 9 mm horizontally and 3 mm vertically of the stand datum and stationary for one second. Reset by recoupling the tool with its body supported.

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

Confirm the port orientation, piston stroke and uncoupling force on a released service fixture. [ATI manual 9610-20-1955](https://ati.novanta.com/wp-content/uploads/sites/2/2025/05/9610-20-1955.pdf)

The scripted Lambda trial scored **1.00** in `adhoc_3451baea.json`.

Dream-1 scored **0.0** in the saved evaluation `adhoc_316f6a1e.json`.
