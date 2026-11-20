# Fit and verify a robot linear-axis homing flag

Fit the homing flag to the carriage. Slide the carriage toward the sensor to verify that the flag triggers home, then let go.

A linear axis on a robotic workcell needs a repeatable home datum. Installing the flag in its carrier and sweeping it through a real-size fork tests mechanical alignment and commissioning.

## Setup

Fixed 141 × 181 mm aluminum bed holding a 155 mm guide, 63 mm passive Y carriage and 5 mm slot optical fork. A 2 mm steel flag with blue polymer handle starts on a 22 × 36 × 23 mm open-sided rest. Relative to the top of the table, the fixture origin is (278, −119, 0) mm and the flag foot starts at (229, −224, 23) mm. The fork center is 50 mm along the carriage travel.

## Scoring

0.30 with the flag seated within 4.5 mm in the table plane and 4 mm vertically, with tilt under 9.3°. A new optical crossing at 36–61 mm of carriage travel raises the score to 0.65. A crossing while unseated or outside that travel window sets the score to zero. Full credit after one continuous second beyond 54 mm with the flag seated and neither the flag nor the carriage touched.

## Reset

Return carriage to zero, move the flag back to the rest and clear the trigger history.

## Physical replication

Use an EE-SX671 sensor, 150 mm linear guide, cut 2 mm sheet-metal flag, fixed brackets made to the supplied stage geometry and a ruler along the carriage. Record the first edge on the sensor indicator and its position. Fix the flag orientation and mechanical stop from the setup scene.

Zero a ruler against the carriage at the open stop. Place the flag foot flat in the keyway and verify the lateral and vertical seating tolerances before starting the sweep. Use the sensor output LED or a continuity logger to identify the first rising edge and read the ruler at that edge. Check the 36–61 mm trigger window. Release the carriage and verify it remains past 54 mm for one second. Restore the carriage to zero, return the flag to its rest, and confirm the sensor is clear.

## Fidelity

A buildable bench fixture pairs an Omron slot geometry with a working physical prismatic carriage. The optical output is calculated from metal flag pose. It does not simulate electronics. Carriage and mounting brackets are custom and the source generator fixes their dimensions.

[Omron EE-SX671 sensor](https://automation.omron.com/en/us/products/family/EE-SX47-67/ee-sx671)

## Run in YAM Arena

Run from this task folder. The recorded settings use seed 0, 20 Hz, zero object jitter, and the step limit and instruction in `task.json`.

```bash
source ~/IsaacLab-Arena/.venv/bin/activate
source ~/IsaacLab-Arena/yam.env
instruction=$(jq -er .instruction task.json)
steps=$(jq -er '.run_limit | split(" ")[0] | gsub(",";"") | tonumber' task.json)
inspect-robots run --embodiment yam_arena --policy yam_scripted -P hold_s=3 \
  -E task=arena_task.py -E headless=true -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=352 --instruction "$instruction" \
  --seed 0 --scorer task_score --max-steps "$steps" --no-prompt --store-frames \
  --log-dir "logs/demo-$(date -u +%Y%m%dT%H%M%SZ)"
```

Use `inspect-robots inspect <saved-eval.json>` to read the final `task_score` and verify the released end state in the saved frames.

The Lambda scripted demo reached **1.00** in `adhoc_d7d29d85.json`. The script centers the flag with up to 8 mm of XY trim from the measured flag pose, releases it, then pushes the carriage with closed jaws. The beam was interrupted at a sampled 46.0 mm of travel. The flag remained seated 1 mm from each planar datum at a final released travel of 62.9 mm.

### Dream-1

From the task folder with the same environment, instruction, and limit:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=arena_task.py -E headless=true -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
  --seed 0 --scorer task_score --max-steps "$steps" --no-prompt --store-frames \
  --log-dir "logs/dream-$(date -u +%Y%m%dT%H%M%SZ)"
```

Dream-1 scored **0.0** in the saved evaluation `adhoc_4963dff7.json`.
