# Load a paint roller evenly

**Societal relatability.** Rolling the sleeve through a shallow paint well and over the ribbed ramp spreads paint around the sleeve and removes excess. The motion is preparatory loading with a rotating sleeve. [Panel painting #1530](https://github.com/robocurve/benchmark-tasks/pull/1530)

Use a stock 100 mm mini-roller frame and sleeve with a nominal 42 mm outside diameter. The simulated frame is 115 g and the sleeve 49 g. The axle starts at (324, −225, 22) mm relative to the tabletop, with the handle raised 22° on its cradle. The tray is centered at (345, −47) mm. Its well is 157 mm long and its ribbed ramp rises 31 mm over 150 mm. The starting grip has molded flats and a thumb rest. A shallow parking pocket and separate block support the roller after release. The simulated paper witness sheet is at (347, 197) mm relative to the station origin. [Cottam 100 mm roller frame](https://cottambrush.com/product/short-roller-frame-4-inch-100-mm/)

**Simulation scoring.** The scorer uses sleeve rotation, position and height to estimate contact with the well. Each of four width bands earns 0.20 when at least ten of twelve circumferential sectors become coated. The last 0.20 requires at least 69 mm equivalent rolling on the ramp after coating and five seconds of release on the rest. During the release, the roller center must remain within 2 mm and the frame within 3° of the start of that hold. Sliding the frame with a fixed sleeve adds no coverage. The scorer clears all coverage, conditioning and rest history at reset and restores the sleeve appearance.

## Physical trial setup and reset

- **Roller and coating.** Use a Cottam PPA00067 6 mm mini-roller frame and a Cottam PPR00037 100 mm microfibre sleeve. Start each trial with an unused, dry sleeve of that model. Use undiluted blue Crayola Artista II Washable Tempera Paint, 32 fl oz (946 mL), at 20–24 °C. [Cottam frame](https://cottambrush.com/product/short-roller-frame-4-inch-100-mm/) · [Cottam sleeve](https://cottambrush.com/product/microfibre-roller-sleeve-4-inch-100-mm/) · [Crayola paint](https://www.crayola.com/products/paint-and-brushes/artista-ii-washable-tempera-paint-32-oz-choose-your-color-543132)
- **Tray and rests.** Use the tray dimensions in `assets/paint_tray.usda` to build a watertight tray from 6 mm rigid PVC sheet with sealed seams. The flat well is 157 × 139 mm inside, and the front wall extends 25 mm above the floor. The ramp starts flush with the well floor and rises 31 mm over 150 mm of horizontal run. Bond rounded PVC ribs about 2 mm high at 17 mm intervals across the width. Fit a level 54 × 139 mm parking shelf at the top with low lips to retain the sleeve. The starting cradle is centered at (221, −243) mm and the parking handle support is centered at (6, −65) mm. Use the two rest assets for their shapes. Set the starting handle pitch to 22° with the sleeve touching the tabletop. Shim the parking support so the sleeve axis is within 5° of horizontal when parked. Use the roller and tray x/y positions above. The actual start axle height is half the installed sleeve diameter.
- **Well fill.** Fix the tray so the well floor is level in both directions. Stir the paint gently for 30 s, fill the well, then wait 30 s for the paint to settle. Set the depth to **8 ± 1 mm** above the well floor, measured before each trial at the center of the flat well with a millimetre depth gauge. Remove the gauge before starting. Empty and wipe the well, ramp, shelf and handle supports before refilling for the next trial. Use fresh witness and drip cards for every trial.
- **Protect the rig.** Fix the tray, supports and witness board inside a lipped spill pan without changing their relative positions. Keep electrical hardware out of the spill pan. Fit lab-approved removable pad covers or dedicated cleanable pads and check the dry handle grip before running. Keep the gripping surfaces dry and grasp only the handle. If coating reaches shared hardware or a gripper pad, stop the run, clean and reset. Record an interrupted trial separately from a scored trial.

## Physical scoring

Record the 70 s trial from fixed overhead and side views. Use millimetre rulers in the planes being measured and contrasting pose marks on the dry frame. Mark the rotating sleeve end cap so its angle can be tracked independently. Record the release and compare the sleeve center and frame orientation with their poses at release. A qualifying park on the specified rest lasts **5 continuous seconds** with no robot contact, a center displacement of at most **2 mm**, and a frame rotation of at most **3°**. Measure post-loading ramp travel after the last roll through the well, counting only segments where the sleeve turns in rolling contact. For each segment use the smaller of the center travel along the ramp and the distance implied by the sleeve rotation using the measured rolling circumference below. A roller dropped below the tabletop scores **0**.

After the trial, the evaluator performs the drip test first, then the witness transfer. Prepare both stations before the run.

1. **Drip test.** Within 5 s of the trial ending, lift by the dry handle and place the roller without shaking it in a prepared support that holds the handle with the axle horizontal and the sleeve free of contact. Center a fresh white uncoated 160 g/m² card at least 140 × 80 mm under the sleeve, **20 ± 2 mm** below its lowest point. Start the timer when the roller is stationary in the support and leave the card in place for **5 s**. Count new blue spots on the card that measure **2 mm or more** at their widest point with a millimetre ruler. The drip test passes with **zero** such spots. Repeat the trial with a fresh setup if the card touches the sleeve during measurement.
2. **One-revolution witness.** Begin within 10 s after the drip test. Fix a fresh sheet of white uncoated 160 g/m² card to a flat board on a tared bench scale. Roll in one direction, at **25 ± 5 mm/s**, with the axle horizontal and a scale reading of **0.50 ± 0.10 kg** above tare (about 5 ± 1 N). Use the end-cap mark to stop at exactly one revolution without a return pass. Align the start and end of the 100 mm working width on the card. Before trials, use a dry sleeve of the same model on the same board and at the same load to measure the distance traveled in one revolution. Mark that start and end distance on the witness board as a check against sliding.
3. **Coverage and score.** Overlay a grid on the witness photo with four **25 mm** bands across the sleeve width and **12 equal columns** between the start and end of the measured revolution. In each cell, sample four **2 mm diameter** circles centered at the quarter and three-quarter positions along both axes. A circle counts when blue covers at least half its area. Count the cell as coated if at least **3 of 4** circles count. Each band earns **0.20** for **10 of 12** or more coated cells. Award the final **0.20** only if all four bands pass, at least **69 mm** of qualifying ramp travel is recorded, the released park passes, and the drip test passes. For example, four passing bands with a 3 mm drip score **0.80**.

The physical protocol measures paint transfer and drips. The simulation estimates coverage from sleeve contact and rotation, and represents excess removal with the post-loading ramp distance. Its recorded trial scores are simulated.

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
