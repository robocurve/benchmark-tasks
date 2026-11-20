# Hold the field-tool release while extracting two MTP guide pins

Use a disconnected cord. The production procedure seats the connector and exchanger before holding both tools through withdrawal. The current cold bench starts seated and models a pair of 0.70 mm pins at 4.60 mm pitch using separate prismatic bodies. The black control and green grip have lateral reach extensions. Damped pin drives follow the measured exchanger travel while both controls are held and maintain the coupling after capture. [Field-tool instructions](https://afl-delivery.stylelabs.cloud/api/public/content/bc6728d0aed54e5f9a569bfc27a64de0?v=d92f5f44) · [US Conec ferrule dimensions](https://www.usconec.com/products/mt-ferrule-72f-multimode-pps)

## Physical measurements

Film both handle movements and the pin exchanger from above. Check the black witness reaches 7 mm before withdrawal and stays engaged through 31 mm. Count a pin only if it travels past 29 mm and stays within 2 mm of its extractor witness. Count full credit only when two pins remain captured after releasing both controls for one second. After the simultaneous hold ends, any captured pin drifting more than 5 mm from the exchanger invalidates the trial. A calibrated witness on the dummy ferrule is needed to check whole-trial face contact. The simulated sensors check both robot and exchanger contact with the face witness. Confirm stock access and the actual latch stroke before using the result as a stock tool evaluation.

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

Confirm the bill of materials, OEM fixture interfaces and allowable forces on the physical test piece.

The scripted Lambda trial scored **1.00** in `adhoc_a342a0f2.json`.

The scripted rollout retained both guide pins after release. Omitting the black release hold scored 0.00.

Dream-1 scored **0.0** in the saved evaluation `adhoc_5611cb59.json`.
