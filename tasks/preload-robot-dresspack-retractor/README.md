# Set a two-link preload and check travel of a robot dresspack retractor

The original fixture has a 70 mm spring-loaded guide and six visual carrier links on the rigid moving section. The modeled pitch is 13 mm. The orange protector slides on a captive guide. The blue test paddle is on an extension at Y −101 mm. The bench spring sets a new neutral position after measured protector closure on the correct preload. [Manufacturer reference](https://www.igus.com/robot-dress-pack/triflex-rse-linear-assembly-instructions)

## Physical measurements

Measure system zero at the start and mark two link pitches on the carrier. Award 0.25 at two pitches ±2.5 mm on a ruler. Award 0.50 when the protector reaches its seated witness line at 9 mm of its benchmark stroke while the carrier is held there. Award 0.75 after at least 24 mm additional motion without exhausting the last 10 mm of the stroke. Award full credit when the released mount returns within 5 mm of the preloaded mark for one second. An overtravel outside the reserve invalidates the trial. Reset both sliders to zero.

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

Select the RSE part and matching carrier and protector, then measure actual pitch, preload, usable range and return force. The moving carrier is a rigid surrogate.

The scripted Lambda trial scored **1.00** in `adhoc_01935dca.json`.

Dream-1 scored **0.0** in the saved evaluation `adhoc_61350169.json`.
