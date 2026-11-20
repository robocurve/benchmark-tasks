# Align three 200 mm wafer notches in a cassette using a common roller

Three 200 mm dummy wafers sit on captive axes 27 mm apart. The robot presses a counterweighted crank, whose measured rotation drives the shared 48 mm diameter edge roller. The model transfers torque to each wafer according to roller speed and stops it at its notch with a simulated detent. The fixed slots constrain each wafer to its own axis. The visual notches share a circular collision envelope. [Manufacturer reference](https://www.sps-international.com/content/net_products/13013-WHS-A4_Manual_Notch_Aligner_datasheet.pdf)

## Physical measurements

Take calibrated side views along each wafer axis and mark the comb alignment line. Award 0.25 for each final notch within ±2° of the line. Film the loading and rotation from the side and above. Award the last 0.25 only if all wafers retain their slot assignment, no hand touches any wafer face during the trial, and the final release is steady for one second. Reset initial errors to 4°, 7° and 9° and zero the crank.

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

Confirm the WHS-A4 cassette and comb geometry. The current catch abstracts the detailed spring-loaded contact at the notches.

The scripted Lambda trial scored **1.00** in `adhoc_45a1d9a3.json`.

The hands-off rollout retained the initial notch errors of 4°, 7°, and 9° and scored 0.00 in `adhoc_022126a2.json`.

Dream-1 scored **0.0** in the saved evaluation `adhoc_94b0d334.json`.
