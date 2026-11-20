# Load a paint roller evenly

**Societal relatability.** Rolling the sleeve through a shallow paint well and over the ribbed ramp spreads paint around the sleeve and removes excess. The motion is preparatory loading with a rotating sleeve. [Panel painting #1530](https://github.com/robocurve/benchmark-tasks/pull/1530)

Use a stock 100 mm mini-roller frame and sleeve with a nominal 42 mm outside diameter. The simulated frame is 115 g and the sleeve 49 g. The axle starts at (324, −225, 22) mm relative to the tabletop, with the handle raised 22° on its cradle. The tray is centered at (345, −47) mm. Its well is 157 mm long and its ribbed ramp rises 31 mm over 150 mm. The starting grip has molded flats and a thumb rest. A shallow parking pocket and separate block support the roller after release. A paper witness sheet is at (347, 197) mm. A water-based washable substitute is appropriate for a repeatable bench check. [Cottam 100 mm roller frame](https://cottambrush.com/product/short-roller-frame-4-inch-100-mm/)

**Scoring.** The scorer uses sleeve rotation, position and height to estimate contact with the well. Each of four width bands earns 0.20 when at least ten of twelve circumferential sectors become coated. The last 0.20 requires at least 69 mm equivalent rolling on the ramp after coating and five seconds of release on the rest. During the release, the roller center must remain within 2 mm and the frame within 3° of the start of that hold. Sliding the frame with a fixed sleeve adds no coverage. The scorer clears all coverage, conditioning and rest history at reset and restores the sleeve appearance.

**Before submitting.** Calibrate the paint level, friction and the amount of roller travel that leaves a usable coat. In a real trial, use a single full revolution onto a witness strip and define the qualifying coverage and drip check. Compare a rolled load with a skid or a bare sleeve. Simulated coating and excess are contact/rotation proxies. The task does not require modeling liquid flow.

## Run the trials

From the submitted `tasks/load-paint-roller` folder, activate the YAM Arena environment and read the instruction from `task.json`. Both trials use seed 0, zero object jitter, and a limit of 1,400 control steps at 20 Hz (70 seconds of simulated time).

```bash
source ~/IsaacLab-Arena/yam.env
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
```

### Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -P hold_s=3 \
  -E task=arena_task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=640 -E cam_height=352 \
  --instruction "$instruction" --scorer task_score --seed 0 --max-steps 1400 \
  --no-prompt --store-frames --log-dir logs/demo
```

The demo lifts straight up, lowers inside the front rim, rolls over the ramp twice, then parks. Use `frame_contact`, `pitch_deg`, `axle_deg`, `covered` and `ramp_mm` to check each phase.

### Dream-1

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=arena_task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$instruction" --scorer task_score --seed 0 --max-steps 1400 \
  --no-prompt --store-frames --log-dir logs/model_run
```

## Read the final score

Pass the saved eval JSON printed by the run to `inspect-robots inspect`. Read `task_score` under `metrics`. For example, from the directory containing the saved demo log, run:

```bash
inspect-robots inspect adhoc_ae59520c.json
```

- Scripted demo (`adhoc_ae59520c.json`) scored **1.0** and ended successfully after 726 steps, with 94 seconds of recorded wall time.
- Dream-1 (`adhoc_64ffe083.json`) scored **0.0** and reached the 1,400-step limit, with 154 seconds of recorded wall time.
