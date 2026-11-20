# Set and verify a parallel gripper's closing stop

Fit the blue closing-stop block. Close the spring-loaded jaw to the stop, then let it reopen.

Gripper setup often limits travel to stop overclosure into the part or fixture. Choose the correct gauge, fit it as a hard stop, and test both the closing stroke and spring return.

## Setup

De-energized 151 × 98 mm guide bed with a passive jaw, keyed stop shelf and visible stroke scale. The blue gauge stem is 26 × 22 × 24 mm, and the gray stem is 14 × 22 × 24 mm. The bed origin is (294, −82, 0) mm relative to the tabletop. The blue and gray blocks start upright at (221, −209, 0) and (362, −209, 0) mm.

## Scoring

0.30 for the blue block within 4 mm in the table plane and 4 mm vertically with tilt under 8.9°. 0.65 after the robot touches the moving jaw while it is at 26–32 mm of measured stroke and PhysX reports contact with the seated block. Full credit with the block still seated and the jaw below 3 mm of stroke, with neither touched for one second. Travel beyond 33 mm with blue seated sets the score to zero.

## Reset

Open the jaw to zero and return both blocks to their marked starts.

## Physical replication

Machine the blocks and pocket to their generated dimensions and fit a 132 N/m return spring. Calibrate a dial indicator against the retracted jaw before the run. Record both the stop position and the unloaded return.

Zero a dial indicator with the jaw fully open. Check that the blue gauge is seated in the pocket within 4 mm of its horizontal and vertical datums. Move the jaw against the gauge and confirm that the indicator reads 26–32 mm at the mechanical stop. A reading beyond 33 mm with blue installed invalidates the trial. Release and check that residual stroke is under 3 mm for one second. Reset the jaw and both gauges to their starting marks.

## Fidelity

This de-energized fabrication fixture models a removable hard stop and manual spring-return carriage. It represents documented industrial travel adjustment. The replaceable block is a custom bench mechanism and does not reproduce SMC's screw adjuster.

[SMC MHF2 adjustable opening and closing options](https://static.smc.eu/pdf/MHF2_EU.pdf)

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

The Lambda scripted demo reached **1.00** in `adhoc_0f7767a0.json`. The robot seated the blue block, then pushed the moving jaw with its closed fingertips. PhysX recorded direct contact with the stop in the scoring window. The jaw returned to 0.0 mm under its 132 N/m return force, and the block stayed seated after release.

### Dream-1

From the task folder with the same environment, instruction, and limit:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=arena_task.py -E headless=true -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
  --seed 0 --scorer task_score --max-steps "$steps" --no-prompt --store-frames \
  --log-dir "logs/dream-$(date -u +%Y%m%dT%H%M%SZ)"
```

Dream-1 scored **0.0** in the saved evaluation `adhoc_a564b4fe.json`.
