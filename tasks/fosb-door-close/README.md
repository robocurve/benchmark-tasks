# Close a 300 mm wafer shipping box in the maker's order: left edge in and latched, right side pressed flush and latched

A 300 mm wafer shipping box (a front-opening shipping box, FOSB) is closed the way the box maker's manual gives: the door is carried by its two D-ring handles, its pivot edge (the robot's left) is set against the opening first and latched with a quarter turn, the far side is pressed flush against the wafer cushion's spring and latched, and both D-rings are folded down.

**What this task adds.** Hold-and-turn tasks use one hand to hold a part still while the other hand turns a latch. Here the order is set by the manual, the door is a free 1 kg body carried by two handles, and the far side springs back out until it is held flush, so the second hand has to press against a spring while its own latch turns. Latching the far side first, one latch only, a door left cocked or rings left up all score less.

**How the demo does it.** Both hands pinch the two legs of a lifted D-ring (jaws across the ring's plane), lift the door off its rest and carry it, yawed 10 degrees, so that its left edge meets the opening's rim. The door's own pose steers both hands every step. The left edge goes to the rim, the door is swung about it until the cushion pads meet the wafers, the left hand turns its ring a quarter turn (the right hand holds the far side), the right hand presses the far side in 5 mm and turns its ring, both hands let go and back off, and each hand folds its ring down with closed fingertips.

## Requirements

`yam-isaac-lab-arena` with Isaac Lab-Arena 0.3.1 (`source <ARENA_DIR>/yam.env`). `mechkit.py` and `demokit.py` are in this folder. Importing `mechkit` hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` without changing them on disk.

## Run a trial

From this folder, the scripted demo (the README command):

```bash
python demokit.py task.py OUT --mode full                                      # the full-score reference, filmed (640 x 360)
python demokit.py task.py OUT --no-record --mode none,rightfirst,oneside,crooked,ringsup   # failure checks, below
python demokit.py task.py OUT --mode full --jitter 0.01 --seed 1               # free parts shifted by up to 1 cm
```

`FOSB_TRACE=1` prints the gaps, pad compressions, ring angles and latch state every 10 steps. The demo reads the door's and the box's poses from the simulator (as `demokit` allows); a policy gets the cameras only.

**Result** (scripted, GPU physics on three lab PCs, 20 Hz): see the table. 560 steps (28 s of simulated time) to 1.0, against a limit of 3,000.

| Run | Setting | Score |
|---|---|---|
| README command, 3 runs | no jitter, 640 x 360, three lab PCs | 1.0, 1.0, 1.0 (560 steps each) |
| Jittered, 4 runs | `--jitter 0.01 --seed 1..4` (free parts shifted up to 1 cm) | 1.0 on all 4 |
| RTX 5090 | README command, GPU to itself | 1.0 |

The slowest 30 steps took 9 s at 640 x 360 on a lab PC (about 0.3 s a step): with Dream-1's 8 s idle limit, send 12 or fewer steps per request. At 320 x 176 (Dream-1's size, `--store-frames` on a fast GPU alone) the slowest 30 steps took 5.3 s (0.18 s a step): `rows_per_request=24` takes 4.3 s plus 0.5 s of round trip, under 5 s.

| Failure check | What it does | Score |
|---|---|---|
| `none` | the arms wait | 0 |
| `rightfirst` | presses the far side and latches the right ring first, then the left | 0 (the order rule) |
| `oneside` | latches the left ring only, then folds both rings | 0.25 |
| `crooked` | swings the door in only to 4 degrees out of square, latches the left, and turns the right ring without pressing the far side in | 0.25 (the right latch never engages: its side is not flush) |
| `ringsup` | does everything but leave the D-rings up | 0.75 |

## Building it for a real trial

- **Box:** a used Entegris SB300 FOSB (385 x 333 x 337 mm), or any 300 mm FOSB, on the printed plate (the maker's three-pin coupling; the simulation holds the box by friction only).
- **Wafers: 25 aluminium discs in place of silicon** (below). No silicon wafer is needed.
- **Door:** the box's own door (363.7 x 258.0 mm, the manual), with its own wafer cushion. The simulation's 25 mm thickness, 1.0 kg and the D-ring bar are estimates.
- **Plate and rest:** printed; the rest is a 75 mm block with two 35 mm rails 28 mm apart. A pencil outline of the box on the plate reads the 5 mm movement limit.

### The wafer stack: 25 aluminium discs

Real silicon is out of the budget: the cheapest listed 300 mm dummy wafers are 18 EUR each in units of 50 (900 EUR), or 38 EUR each in a boxed unit of 25 (950 EUR) (MicroChemicals, CZ-Si wafer 12 inch 755 um and 750 um DSP, microchemicals.com). The stack is therefore cut from 0.032 in aluminium sheet.

| | Silicon wafer (SEMI M1) | Substitute disc |
|---|---|---|
| Material | silicon, 2.33 g/cm3, E 130 GPa | 5052 aluminium, 2.68 g/cm3, E 70 GPa |
| Diameter | 300 mm | 300 mm (+/- 0.5 mm, checked with a tape across two diameters) |
| Thickness | 0.775 mm | 0.81 mm (0.032 in) |
| Mass | 128 g | 154 g |
| Sag under its own weight, against silicon | 1 | about 2 (it scales with density / (E x thickness squared)); a 0.76 mm polycarbonate disc would sag about 30 times silicon's, so polycarbonate is not used |
| Edge | rounded, notched | filed round and sanded (220 grit), so it cannot cut the cushion or a finger; no notch |
| Count and arrangement | 25, one per slot, 10 mm pitch | the same: one disc in each of the box's 25 slots, pushed back to the slots' back stops as a wafer sits |

**Source and cost.** Zoro Select 5052 aluminium sheet, 24 x 24 in x 0.032 in (Zoro G702643793, 22.35 USD each). A 610 mm sheet takes four 300 mm discs, so seven sheets give 28 discs (25 and 3 spares) for 156 USD, with aviation snips (about 15 USD) and a file. Each disc is scribed round a 300 mm template (printed, or cut once from MDF), cut with the snips, then filed to the line. The discs do not break, so the stack is reused for every trial.

**Why it reproduces the cushion contact and the closing resistance.**
- The cushion touches only the wafers' front edges. Seated against the slots' back stops, a disc's front edge stands where a wafer's does, because the diameter sets it: a 0.5 mm diameter error moves the edge 0.5 mm, and changes the simulator's cushion force by 0.9 N of 14 N.
- The thickness, 0.81 mm against 0.775 mm, seats each disc in its slot and in the cushion's groove as a wafer is seated. Its sag is about twice silicon's, a fraction of a millimetre, so its front edge stays at the groove's height. Check it on the first trial: with the box open, the front-edge droop of the top and bottom discs, read with a steel rule against the slot, must be under 1 mm.
- The closing resistance is the cushion's own spring pressed against the 25 edges. The cushion is the real box's, so the resistance is the real one, once the edges stand where the wafers' would.
- The mass does not enter: the cushion pushes the edges horizontally, and the scene's wafers are fixed in the box, so the simulation uses no wafer mass.

### Real cushion force and latch torque, against the simulator

Measure each on the real box with the disc stack in place, once before the first trial, and write the real value in the last column. No score tier depends on either: the same rule (latch order, flush to a 0.5 mm feeler at four corners, the door's angle at first contact) scores the simulation and the real trial. The arms push about 20 N and grip about 40 N a finger, so the trial is feasible while the measured push at the right edge is under 20 N and the cam turns under a pinch of the ring. A value outside the simulator's estimate is a reason to re-run the scene with the measured figure, which the scene's constants (`PAD_K`, `CAM_NM` in `task.py`) take directly.

| Value | Simulator (estimate) | How to measure it on the real box | Real |
|---|---|---|---|
| Cushion's push at flush, whole door | 14 N (two 875 N/m pads, 8 mm) | Both latches open. Push the door's centre with a 50 N digital push-pull gauge and its flat tip (SDF-50 class, about 112 USD) until a 0.5 mm feeler (a feeler gauge set, about 10 USD) enters at none of the four corners. Read the peak. | |
| Push at the right edge, left latched | 7 N (the same pads, about the left edge) | Left latch engaged. Push the right edge with the gauge until that side is flush. | |
| Cam torque | 1 N m (the manual's maximum is 4.4 N m) | Tie a string loop to the end of the D-ring's bar, 26 mm from its stem in the simulator (measure the real arm with a ruler). Pull it at right angles to the bar with a digital luggage scale (50 kg x 10 g, 10 to 25 USD) while the ring turns. Torque = reading x arm: 1 N m reads 38 N (3.9 kg) at 26 mm. | |
| Door mass | 1.0 kg | A kitchen scale | |
| Does the cam draw the door in by itself | assumed not (the latch only holds) | Turn the cam with the door held 5 mm proud | |

### Checking the door's angle at first cushion contact

- **Camera.** A phone on a stand, 50 cm above the box's top, lens over the door's top edge and looking straight down, filming at 60 frames a second.
- **Angle mark.** A printed paper protractor (1 degree marks, the 5 degree line in red, 150 mm radius) taped to the box's top. Its centre is on the opening's left (pivot) corner, and its 0 line runs along the rim's face.
- **First contact.** The top disc's front edge is marked with a red paint pen. The first frame in which the cushion meets the red edge, seen through the clear top, is the frame of first contact. Read the door's top edge against the protractor in that frame. Past the 5 degree line scores 0.
- **Resolution.** On a 1080p frame about 700 mm wide (a phone's main lens at 50 cm), 1 degree at 150 mm is 2.6 mm, about 7 pixels. The demo swings the door 11 degrees in 2.5 s, about 0.07 degrees a frame.
- **The simulator's rule.** The same rule: the cushion touches the wafers with the door more than 5 degrees out of square. The simulator takes 1 mm of pad travel as touching. A door swung about its seated left edge first touches at about 3 degrees, because the pads stand 8 mm from flush, 137 and 227 mm from the pivot.

### Protecting the gripper pads, and handling

- **Where the hands touch.** The demo touches the door only at its D-rings. Both hands pinch a lifted ring to carry the door, press the far side in through the ring and turn it. Each then folds its ring down with closed fingertips on top of the bar. The rings stand 36 mm proud of the door's face and 32 mm in from its side edges, so no finger meets a door edge.
- **Pad cover.** Wrap each fingertip pad in one layer of cloth gaffer tape (about 8 USD a roll), changed every session, or slip a 2 mm TPU 95A sleeve over each finger (printed, a few grams of filament). Either keeps a ring's moulding marks or a stray door edge off the pads.
- **Handling.** The discs have no fragile material. Wear gloves while cutting and filing them, and file every edge round before a disc goes into the box. If silicon wafers are ever used instead, handle them with a wafer wand or gloves by the edge only; a chipped wafer's edge is sharp.

## Size table and size check

Every dimension a hand or the scorer touches, the scene value, and its source. Manual: Entegris SB300 FOSB user manual, https://www.entegris.com/content/dam/product-assets/sb300fullpitchfrontopeningshippingboxesfosbs/manual-sb300-fosb-for-300mm-wafers-7429.pdf. Wafer: SEMI M1 / SEMI M31 (300 mm wafers and FOSB), https://store-us.semi.org.

| Dimension | Scene | Source |
|---|---|---|
| Box outside, W x D x H | 385 x 333 x 337 mm | Manual |
| Wafer diameter and thickness | 300 mm, 0.775 mm | SEMI M1 (the 300 mm wafer) |
| Wafer count, pitch, first slot above the datum | 25, 10 mm, 44 mm | Manual |
| Door, W x H | 363.7 x 258.0 mm | Manual |
| Door thickness, mass | 25 mm, 1.0 kg | Estimate |
| Box mass | 7 kg | Estimate |
| Opening, W x H, centre above the datum | 340 x 250 mm, 184 mm | Estimate (the manual gives no opening size; the door overlaps it by 11.9 mm a side and 4 mm top and bottom) |
| Rim depth; wafers' front edge behind the rim | 12 mm; 18 mm | Estimate |
| D-ring: bar, legs, height proud | 12 mm square x 52 mm, legs 40 mm apart, 36 mm | Estimate; the manual shows the D-ring handles without dimensions |
| D-ring spacing on the door | 300 mm (y = +/-15 cm) | Estimate |
| Cam torque (latch) | 1 N m | Estimate, within the manual's 4.4 N m maximum |
| Wafer cushion | two pads, 875 N/m each, 14 N together at flush (10 to 18 N) | Estimate; the manual gives no force |
| Plate under the box | 20 mm | Estimate (a stand for the real kinematic coupling, which is not modelled) |

Size check, printed from the constants in `task.py` (`POD_W`, `POD_D`, `POD_H`, `WIN_W`, `WIN_H`, `WIN_Z`, `DOOR_W`, `DOOR_H`, `WAFER_R`, `WAFER_T`, `WAFER_PITCH`, `WAFER_Z0`, `NWAF`) against the published figures; no simulator run is needed:

```
box      385 x 333 x 337 mm   published 385 x 333 x 337    ok
door     363.7 x 258.0 mm     published 363.7 x 258.0      ok
wafer    300 mm x 0.775 mm    published 300 x 0.775        ok
pitch    10 mm, first 44 mm, 25 wafers, last at 284 mm (top of box 337 mm)   published 10 / 44 / 25   ok
opening  340 x 250 mm         estimate; door overlaps it 11.85 mm a side, 4.0 mm top and bottom
```

## Simplifications, stated

- A latch is a rule, not a cam and hook: it engages when its ring is turned 80 degrees or more while its side is within 1 mm of flush, then holds that side to the rim with 12 N. The robot's contact turns the ring and presses the door.
- The wafers are fixed in the box's colliders (25 discs, 0.775 mm). The cushion is two spring pads that press on them; "the door strikes the wafers" is scored as the pads meeting them with the door more than 5 degrees out of square.
- The box and door are hand-built from published sizes. No vendor CAD was used; none is distributed here.

## Safety for a real trial

The door is light but its moulded edges can cut. Keep people clear of the arms' reach while the robot runs and keep an emergency stop in reach. The aluminium discs do not shatter and their edges are filed round; cover the gripper pads as above.

## Earlier feedback

The review of this pull request asked for the points below; each is answered in the task itself.

- **"Specify an inexpensive, repeatable substitute for the 25 full-size silicon wafers, including its material, dimensions and arrangement, and explain how it reproduces the cushion contact and closing resistance."** 25 aluminium discs, 300 mm across and 0.81 mm thick (sag about twice silicon's), one in each slot, cut from seven sheets for 156 USD and reused every trial. The diameter puts each front edge where a wafer's is, and the box's own cushion gives the closing resistance (The wafer stack, above; `setup`).
- **"Protect the gripper pads from the door's sharp edges and document safe handling of any fragile wafer material."** The demo touches only the D-rings, 32 mm in from the door's side edges. The pads are taped or sleeved, and the discs are not fragile (Protecting the gripper pads, above).
- **"Check and document the real cushion force and latch torque against the simulation estimates."** A push-pull gauge reads the cushion's push (14 N, and 7 N at the right edge, in the simulator), and a luggage scale on the D-ring's bar reads the cam torque (1 N m). The table has a column for the real values, and no score tier depends on either (Real cushion force and latch torque, above).
- **"Clarify how the real-trial video checks the door's angle when the cushion first contacts the wafers."** An overhead phone at 60 frames a second, a printed protractor on the box's top centred on the pivot corner, and the top disc's front edge marked red. The frame in which the cushion meets the red edge is read against the 5 degree line (Checking the door's angle, above; `scoring`).

## Scoring

See `scoring` in `task.json`, and `Trial` in `task.py`. The scorer reads the door's four inner corners against the box's rim, the D-ring angles, the pads' compression and the box's position.

## Trademarks and sources

Entegris and SB300 are trademarks of Entegris, Inc. They are used here only to name the product whose published dimensions the box follows; this task is not affiliated with or endorsed by Entegris. Sizes and the procedure quoted in the task text come from the SB300 FOSB user manual (entegris.com, manual-sb300-fosb-for-300mm-wafers-7429.pdf).

## Licence

MIT. Copyright (c) 2026 the task's authors.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: the above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED.
