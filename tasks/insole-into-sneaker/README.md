# Put a foam insole back in a laced sneaker: bend it through the opening, push it home to the hidden toe, seat it flat

Category: Societal relatability. Instruction, setup and scoring: `task.json`.

People take the insoles out of their sneakers to wash, dry or replace them, and then put them back with the shoe
still laced. A full-length insole (267 mm) is about twice as long as the collar opening (130 mm). It goes in toe
first, bent down behind the tongue, and slides along the bottom under the laced vamp, where it disappears. The
padded tongue leans back over the front of the opening and is held forward out of the way with the other hand. The
hand cannot follow into the toe box, so the insole is pushed home from its heel end until the heel drops behind the
heel counter into the heel cup and the whole insole lies flat. Held level, the insole bridges the opening and never
goes in; pushed too little, its toe stops short and its heel stays up on the counter.

The shoe is a Brooks Ghost 18, men's US 9, built to its published inside length, stack heights (10 mm heel-to-toe
drop), toe box height and tongue padding (sources below); what is not published is listed, with how to measure it,
in `build/README.md`.

## Files
- `isaac/task.py`: yam_arena task file (objects, instruction, live score, success on 1.0 held 4 s).
- `isaac/geom.py`: insole outline and stiffness parameter, the shoe (bed with drop, heel cup and toe spring; upper,
  collar, eyelets, laces, sole), the tongue, start layout. Published numbers are marked (P), numbers to measure (M).
- `isaac/insolekit.py`: the insole (a chain of 18 foam slices joined by constant bending, twisting and in-plane
  springs) and the tongue (a chain of 4 padded slices jointed to the shoe at the top eyelets).
- `isaac/insole_scoring.py`: the scorer, a pure function of measured geometry; `python insole_scoring.py` runs its
  engine-free checks (15 cases).
- `isaac/partkit.py`: rigid parts made of boxes (the shoe; boxes can be drawn without a collider), step and reset
  hooks, gripper points.
- `isaac/demo/run_demo.py`: scripted demo and comparison runs; `isaac/demo/official_replay.py`: replays the demo's
  recorded actions through `inspect-robots run`.
- `build/`: parts list, published numbers, measurements to take, the insole stiffness test and `fit_stiffness.py`.
- `LICENSE`: MIT.

## How to run (Isaac Lab-Arena + yam_arena + Inspect Robots environment, from `isaac/demo/`)
```sh
python run_demo.py demo OUT          # scripted demo: writes OUT/actions.json, run.mp4, result.json
python run_demo.py idle OUT          # also: nobend, short, nopress, tongue_free (comparison runs)
python official_replay.py --actions OUT/actions.json --hold-s 1 -- run --embodiment yam_arena \
  --policy insole_replay -E task=../task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "<task.json instruction>" --scorer task_score --max-steps 2400 --store-frames --log-dir OFFICIAL
inspect-robots video OFFICIAL/<log>.json
```
One trial per process, fixed layout (object_jitter 0), 20 Hz control. Environment knobs for sensitivity runs only
(defaults are the task): `INSOLE_E_SCALE` (multiplies the insole's E), `INSOLE_P0` (entry pitch, degrees, default
20), `INSOLE_TONGUE_K_SCALE` (multiplies the tongue's bending springs), `INSOLE_CAM=WxH` (camera size; `shot` stage
averages 40 frames for a clean setup picture). Runtime on an A6000: Kit start and reset about 2.5 min, then about
0.3 s per control step (the demo takes about 6 minutes).

## Scoring
The scorer runs after every control step on measured poses. The value at the end of the trial counts, and the trial
ends once 1.0 has held for 4 s.
- 0 unless the shoe stands upright on the table (within 10 degrees) and both grippers are at least 3 cm clear of
  the shoe (with its tongue) and the insole.
- 0 in a real trial if at any time the robot unties or loosens the laces (whole-trial video: the bow comes undone,
  a lace end is pulled, or a lace leaves its eyelet). In the simulation the laces are tied and fixed.
- 0.25: the insole is inside the shoe: within its walls, no part of it higher than the top edge of the upper (collar, then vamp),
  not folded back, and the point 20 mm from its toe tip forward of the top eyelets (under the laced vamp).
- 0.5: also home: heel end within 6 mm of the heel counter (the trimmed insole is 4 mm shorter than the inside) with
  its edge down on the bed (no more than 5 mm above it), toe within 10 mm of the end of the toe box.
- 1.0: also flat: the insole's underside within 3 mm of the bed at its heel, arch and ball marks (30, 100 and
  185 mm from its heel end).

Real trial (simple tools): video for the hands and the shoe; a ruler laid across the collar for "below the top
edge"; a pen line 20 mm from the insole's toe that must not be visible looking straight down into the opening (tongue
held forward); a steel ruler between the heel counter's lining and the insole's heel edge; a pen line 150 mm from the
toe tip, at least the inside length minus 160 mm (111 mm) from the heel counter; and, for "flat", a pencil's eraser
end pressed lightly on the heel, arch and ball dots (the ball dot reached through the opening): the insole moves
down by no more than 3 mm at each, read against a ruler held upright beside the pencil.

## Results
All runs: Isaac Lab-Arena yam_arena, one Kit process per run, object_jitter 0, 20 Hz control, 40 N finger cap, the
final scene. Scores are what the scorer reported at the end of each run.

| Run | What the robot does | Measured end state | Score |
|---|---|---|---|
| Scripted demo | left hand folds the tongue forward onto the laces and holds it; right hand pinches the insole's heel end, hangs it over the shoe, puts the toe down behind the throat at 20 degrees and pushes forward and down along 7 points; opens fully in place, lifts straight up; presses heel, arch, heel; both hands let go and both arms go to READY | toe 2.4 mm from the toe box, heel 2.1 mm from the counter, heel edge 3.4 mm up; underside 1.4 / 0.5 / 2.9 mm above the bed at the heel / arch / ball marks; shoe moved 1.6 cm | 1.0 (first 1.0 at step 1024, success at 1103) |
| Official replay of the demo | same actions through `inspect-robots run --scorer task_score` | same | task_score 1 (success, 1103 steps) |
| Do nothing | arms hold READY for 600 steps | insole on the box lid | 0 |
| No bend | insole held level, laid straight down over the opening, let go | the insole bridges the opening and stays draped over the open gripper, its heel end 200 mm up | 0 |
| Short push | entry and 2 of the 7 push points, let go, same presses | toe 100 mm short of the toe box, heel up on the counter (110 mm) | 0 |
| No presses | the demo without the three presses | same end state as the demo: released from low over the heel, the insole drops flat by itself | 1.0 (success at 854) |
| Tongue left alone | the demo with the left hand idle | the toe lands on the tongue leaning over the opening, never gets under the vamp; the insole ends outside the shoe | 0 |
| Near-rigid insole (E x 1000) | the demo | toe stalls at 160 mm (30 mm past the throat) and never reaches the toe box | 0 |
| Dream-1 (dreamlabs/dream-1-beta, through `inspect-robots run --policy dream`, task_score scorer, the instruction in task.json) | the right arm reaches over to the insole on the box lid; the insole is not picked up | the insole still on the box lid, the shoe untouched | 0 (2400 steps, run limit) |


## How the tongue was handled
The tongue is a soft padded flap sewn in under the laces: four 12.5 mm slices (52 mm wide, 9 mm thick, 12 g in
total) jointed to the shoe at the top eyelets, with bending springs of 0.015 N m/rad per joint. At rest it leans back
over the front of the opening, 30 degrees from upright, and sags a little under its own weight. In the demo the left
hand pushes it forward from behind with closed fingertips, folds it onto the laced vamp and holds it there while the
right hand feeds and pushes the insole, then lets it spring back before both arms return to READY. The laces stay as
they are. Left alone (the same run with the left hand idle), the insole's toe lands on the tongue where it leans over
the opening, never gets under the vamp, and the insole ends outside the shoe (score 0); with the tongue held forward
the same push seats it (1.0). The earlier version's stall at 160 mm with a rigid block tongue was not reproduced:
with the jointed tongue made near-rigid (its springs x 1000) the toe still reached the toe box, so the old stall's exact cause is not known.

## Physics notes
- Insole: 18 rigid foam slices 14.8 mm long, each as wide as the insole outline there and 7 mm thick, 40 g in total,
  joined by D6 joints whose translations are locked. Bending through the thickness, twist and in-plane bending have
  linear springs k = E w t^3/12/s, G w t^3/3/s and E t w^3/12/s from one modulus E = 4 MPa (EI about 8.1e-3 N m^2).
  E is a named parameter, not a measurement: `build/` has a 5-minute cantilever test that measures it. The springs
  are constant for the whole trial: nothing stiffens to help seating. Lateral curling across the width and
  compression of the foam are not modelled.
- Check of the bending model: held level by its toe, the chain's free heel end hung 253 mm out and 81 mm down; a
  heavy elastica with the same E, outline and mass gives 256 / 77 mm (checked on the 270 mm version of the insole).
- Shoe: one free rigid body of 0.27 kg made of boxes. The bed under the insole follows the published 36 / 26 mm stack
  less the 5 mm original insole (31 mm above the table at the heel, 21 mm at the ball, a cosine blend between 35 and
  180 mm), with a toe spring and a heel cup whose sides rise 8 mm. Walls follow the inside outline; the collar is
  padded; the laced vamp is a rigid roof 58 mm above the bed at the top eyelets and 31 mm at the toe box. Eyelets,
  laces and the bow are drawn on the vamp without colliders (the laces stay tied, so the vamp under them is rigid).
  The upper does not flex. Because it is rigid, its inside above the arch is wider than the insole by up to 15 mm per
  side (insole / inside width 65 / 85 mm at 72 mm from the heel, 62 / 92 at 97 mm, 64 / 94 at 121 mm, 75 / 95 at
  145 mm; 40 / 60 at the heel end, 92 / 100 at the ball), where a real insole fits the bed outline within about 1-2 mm
  per side: rigid walls following the outline blocked the 92 mm forefoot on its way in. Friction 0.8 on the shoe and
  the insole.
- YAM fingers capped at 40 N each. The heel-end pinch through the 7 mm foam holds at about 17 N per finger.

## Stiffness: measure it on the real insole
No published bending stiffness for a foam insole was found, so the task keeps it as a named parameter (`geom.E`).
`build/README.md` gives a 5-minute cantilever test (heel end over a table edge at 80, 120 and 160 mm, a ruler, an
optional 5 g coin) and `build/fit_stiffness.py`, which turns the measured drops into E. With E = 4 MPa the heel tip
should drop 0.8 / 4.5 / 12.9 mm.

## Real trials
Bought items only: a Brooks Ghost 18 (men's US 9) with its own insole taken out, laced to a marked lacing; a Spenco
PolySorb Walker/Runner insole (size 3) trimmed to the shoe's own insole; the shoe box. Measure the shoe and the
insole before the first trial (`build/README.md`). Reset: pull the insole out by its heel, lay it on the box lid at
its marks, put the shoe back on its tape marks, push the tongue back over the opening. Nothing is used up.

## Sources
- Brooks Ghost 18, removable insole. Zappos product page https://www.zappos.com/product/10041249/color/683936 lists
  "PDAC A5500 Approved", which it explains is "awarded to footwear ... that is available in three or more widths,
  with a removable insole (sock-liner)". RunRepeat lab review https://www.runrepeat.com/brooks-ghost-18: "The insole
  is fully removable", "Removable insole: Yes".
- Brooks Ghost 18 numbers. Running Warehouse https://www.runningwarehouse.com/Brooks_Ghost_18/descpage-B18GHM0.html:
  "Heel Stack: 36 mm", "Forefoot Stack: 26 mm", "Heel-Toe Offset: 10 mm", "10.0 oz | 283 g" (men's size 9),
  "soft, flexible, flat-knit tongue". RunRepeat (same page as above), lab measurements: internal length 271.0 mm,
  toebox height 25.8 mm, "We measured the Ortholite X-60 insole at 5.0 mm thick", "We measured 9.0 mm of tongue
  thickness", gusset type "None", midsole width 115.7 mm forefoot / 95.9 mm heel.
- Spenco PolySorb Walker/Runner replacement insole, Cat. No. 38-385, maker's product sheet: "Thickness at Heel:
  7.50mm [0.30"]", "Thickness at Forefoot: 6.80mm [0.27"]", "Best For Use In: Athletic, running or walking shoes with
  removable insoles", "Use the original insole as a guide to trace and trim", "Care: Hand wash in warm water. Air
  dry.", size 3 = U.S. Men's 8/9. https://cdn.shopify.com/s/files/1/0316/3533/files/Spenco-WalkerRunner.pdf

## Third-party assets and licenses
The SDK is yam-isaac-lab-arena at 0059d02a1a91bc6834c1d0846022891df0c78730, the program's official dependency,
installed by the user's environment. Nothing from it is redistributed here. Licenses are given as each source states
them.

| Asset | Used for | Source | License as stated by the source | Bundled |
|---|---|---|---|---|
| YAM arm and gripper (URDF, meshes, collision hulls, converted USD) | both arms | SDK `yam_arena/assets/yam/`, from i2rt (SOURCE_COMMIT 120c3c81400171174604e503943f8d1ebc891058) | MIT, Copyright (c) I2RT Robotics (`LICENSE.i2rt` there) | no |
| link2/link3 meshes split into black and white parts | station arm colours | SDK `yam_arena/assets/yam/dream/`, from the dreamlabs 1.0.23 wheel | "MIT, see ../LICENSE.i2rt" (SDK `dream/SOURCE`) | no |
| Station floor and table | room | SDK `station.py`/`scene.py`: boxes with flat colours generated in code; no mesh or texture file loaded | SDK README: "**Placeholder.** This repository has no license yet: most of it is ported from YAM-inspect-Isaac by Thomas Pierson, which has none." | no |
| Top-camera stand | room | SDK `station.py`: shapes generated in code | as the row above | no |
| Sky dome light and lamp | lighting | SDK `scene.py`, using Isaac Lab-Arena's DomeLight and an Isaac Lab sphere light | as the row above | no |
| top_cam, left_cam, right_cam | policy RGB | SDK pinhole camera sensors at the calibrated poses; no camera mesh | as the row above | no |
| Station poses, sizes and colours | numbers in the SDK code | SDK README: "Dream Labs' calibrated YAM station, from Dream's MuJoCo replica (dreamlabs 1.0.23, `yam_mujoco`) and the calibration it follows (yamlab at `ec0455d`)" | license not stated by the SDK | no |
| Station fruit, bowl, plate (Arena object library) and water bottle | not used: this task's own OBJECTS replace them | - | - | no |
| Inspect Robots, inspect-robots-yam | runner, scorer, video | inspect-robots 1.0.0, inspect-robots-yam 0.38.0 | as stated by each package | no |
| `isaac/partkit.py` | part builder and hooks | adapted from a part-builder helper released with an earlier benchmark task under the MIT License (Copyright (c) 2026 The task authors) | MIT | yes |
| Sneaker, tongue, insole, shoe box, scorer, demo, build files | task | this folder | authors' own work, MIT (`LICENSE`) | yes |

Isaac Lab-Arena release/0.3.1 (c7edf3a9d5d8276565208f72422f8f8a0806d6cf) ships the Apache License 2.0 in its
LICENSE.md. Isaac Sim and Isaac Lab are runtime dependencies under their own licenses. None of them is redistributed
here. All task-specific code and geometry in this folder is the authors' own work under the MIT License.
