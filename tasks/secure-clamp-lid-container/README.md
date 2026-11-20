# Latch a Snapware Pyrex leftovers container

Put the lid on the glass leftovers container. Press down on the lid while closing all four tabs, then let go.

## Bench layout

A transparent rounded 147 mm-rim glass vessel. The lid starts off the container on a separate rest, all four tabs at −57°. A removable silicone lift grip sits at the center.

## What the demo does

The lid starts on a separate low rest. Left places the lid using a small removable silicone pickup grip. Right closes the near two tabs while left presses the opposite edge. The hands exchange roles for the last pair so there is wrist clearance. Capture both the lid transfer and the hand bracing it.

## Scoring

0.25 for seating within 4 mm laterally and −3 to +4 mm vertically with level aligned lid. Each latch contributes 0.15 when it closes within 7° of its stop and its measured catch overlaps the underside of the rim. Full score after all four are engaged, with a stable lid and no robot contact for 1 s.

## Reset

Return lid to its side rest, put all four tabs at the open stop, and keep the glass vessel on its nonslip mat.

## Fidelity

Transparent glass and four latch geometry follow Snapware 1109304. A removable low silicone grip allows lid transfer. Hinges and gasket compression are approximations. The scorer measures mechanical closure and stability over all four tabs.

[Pyrex Snapware 1109304](https://pyrexhome.com/products/snapware-total-solution-pyrex-glass-4-cup-square-food-storage-with-plastic-lid)

The clear vessel is rendered as a continuous glass shell with separate floor, wall and rim colliders. Reset initializes all four physical hinge states to the open angle. During the demo the joints move under contact and an over-center detent torque.

## Run the trials in YAM Arena

From this task folder, activate the existing Arena environment and load the instruction and limit from the task specification. Use a clean reset, seed 0 and zero object jitter. Both trials run at 20 Hz with the `task_score` scorer.

```bash
source ~/IsaacLab-Arena/.venv/bin/activate
source ~/IsaacLab-Arena/yam.env
instruction=$(jq -er .instruction task.json)
steps=$(jq -er '.run_limit | split(" ")[0] | gsub(",";"") | tonumber' task.json)
```

### Scripted demo

The recorded demo used the graphical display and 640×352 frames. Set `-E headless=true` to run without a display.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -P hold_s=3 \
  -E task=arena_task.py -E object_jitter=0 -E headless=false -E control_hz=20 \
  -E cam_width=640 -E cam_height=352 --instruction "$instruction" \
  --scorer task_score --seed 0 --max-steps "$steps" --no-prompt --store-frames \
  --log-dir logs/demo
```

### Dream-1

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=arena_task.py -E object_jitter=0 -E headless=true -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
  --scorer task_score --seed 0 --max-steps "$steps" --no-prompt --store-frames \
  --log-dir logs/model_run
```

Read the final `task_score` under `metrics` in `inspect-robots inspect <saved-eval.json>`.

The scripted demo scored **1.0** in the saved evaluation `adhoc_2907eae8.json`.

The recorded demo finished with all four tab catches under the rim. The sampled final catch overlaps were 4.0, 1.5, 1.7, and 4.0 mm with the lid released.

Dream-1 scored **0.0** in the saved evaluation `adhoc_af496a9a.json`.
