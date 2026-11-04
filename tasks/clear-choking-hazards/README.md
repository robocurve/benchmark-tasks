# Clear choking hazards from a toddler's play mat

A bimanual YAM task in Isaac Lab Arena. Seven toys stand in a row on a play mat. Four are small enough to be
choking hazards under the US small-parts rule for children under 3 (16 CFR 1501), and three are not. The robot
has to judge each toy by its size and shape against the rule stated in the instruction, put the hazards in a
grey bin, leave the safe toys on the mat, and let go. Colours carry no information. Two toys are deliberately
close calls: a thin peg that is narrow enough for the cylinder but too long, and a block that fits only when it
stands on its square face.

The whole task is one task file, [`clear_choking_hazards.py`](clear_choking_hazards.py), for the `yam_arena`
embodiment from [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (Isaac Sim 6, Isaac Lab
3, Isaac Lab Arena 0.3). The scene is built from boxes, cylinders and spheres only, so nothing is downloaded.
[`small_parts_fit.py`](small_parts_fit.py) checks every toy against the small-parts test fixture (see below).

## Run a trial

From this folder, in the Arena environment:

```bash
INSTRUCTION="A toddler under 3 will play on this mat. Under the US small-parts rule (a part is a choking hazard if it fits entirely inside a cylinder 31.7 mm wide and 25.4 to 57.1 mm deep), put every choking hazard in the grey bin and leave every safe toy on the mat. Then let go."

# Dream-1 (needs DREAM_API_KEY in ~/.dreamlabs/credentials)
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=clear_choking_hazards.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir runs/dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=clear_choking_hazards.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" --scorer task_score --max-steps 2400 --store-frames --log-dir runs/demo

# One MP4 per camera, in runs/<policy>/frames/<run>/
inspect-robots video runs/dream/*.json

# Check the hazard / safe calls against the test fixture (about 45 s on a CPU)
python small_parts_fit.py
```

The run prints `task_score` at the end and keeps it in the log.

- **Run limit:** 2,400 control steps (2 minutes at 20 Hz).
- **Seeds:** none. `-E object_jitter=0` starts every toy exactly where the task file puts it, the layout in the
  screenshot. The scripted demo is open-loop and expects that layout.
- **Runtime per trial:** the scripted demo's trial is about 1,040 steps (52 s simulated) and took about 3 minutes
  of wall time on an L40S. The Dream-1 trial ran the full 2,400 steps in about 6.5 minutes. Add a few minutes for
  Isaac Sim's first start on a fresh machine.

## Scene

Positions are in cm, from the midpoint between the two arm bases: x forward, y left, heights above the table
top. The arm bases are at y = +30 (left) and y = -30 (right).

| Object | Type and material | Size | Position (centre) | Mass | |
|---|---|---|---|---|---|
| Play mat | box, pale-green foam | 16 (x) x 60 (y) x 1 cm | x 17, y 0 (spans x 9 to 25, y -30 to +30) | | fixed |
| Toy bin: floor | box, grey plastic | 14 x 14 x 1 cm | x 34, y 0, on the table | | fixed |
| Toy bin: walls | four boxes, grey plastic, 1 cm thick, 7 cm tall | 14 x 14 cm inside, 16 x 16 cm outside | around the floor; the rim is 6 cm above the bin floor | | fixed |
| Rubber ball | sphere, orange rubber | 34 mm diameter | x 17, y -24, on the mat | 23 g | safe |
| Wooden peg | cylinder, purple wood, standing on an end | 28 mm diameter x 60 mm | x 17, y -10, on the mat | 26 g | safe |
| Wooden cube | box, red wood | 20 x 20 x 20 mm | x 17, y -3, on the mat | 5.6 g | **hazard** |
| Wooden block | box, blue wood | 60 x 60 x 60 mm | x 17, y +4, on the mat | 150 g | safe |
| Plastic block | box, teal plastic, standing on a square face | 20.9 x 20.9 x 24 mm | x 17, y +11, on the mat | 11 g | **hazard** |
| Wooden spool | cylinder, pink wood, standing on an end | 29.6 mm diameter x 22 mm | x 17, y +18, on the mat | 10.6 g | **hazard** |
| Steel ball | sphere, steel | 16 mm diameter | x 17, y +25, on the mat | 16.8 g | **hazard** |

All box toys are square to the table edges. Hazards and safe toys alternate along the row, and every toy has
its own colour, so neither position nor colour tells which is which.

## Which toys are choking hazards

16 CFR 1501.4 tests a part with a cylinder of 31.7 mm (1.25 in) inside diameter whose floor slopes, so the depth
below the rim runs from 25.4 mm (1.0 in) on one side to 57.1 mm (2.25 in) on the other. A part that fits
entirely inside it in **any** orientation, without being compressed, is a choking hazard for children under 3.

`small_parts_fit.py` models that fixture exactly and searches about 800 orientations of each toy (random ones
plus every axis-aligned one and tilts about them), with a coarse then a 0.02 mm fine search over positions,
lifting the toy as high as the rim allows. The constraints (inside the circle, below the rim, above the sloped
floor) are convex, so checking points on each toy's surface is exact for these shapes. "Slack" is the smallest
margin by which the best orientation satisfies all three; negative means the toy cannot fit.

| Toy | Fits? | Best slack | Why |
|---|---|---|---|
| 16 mm steel ball | yes | +7.9 mm | 16 mm across, well under 31.7 mm |
| 20 mm cube | yes | +1.7 mm | its 28.3 mm face diagonal fits the circle; 20 mm tall is under the 25.4 mm shallow side |
| 29.6 x 22 mm spool | yes | +1.05 mm | 29.6 mm across, 22 mm tall |
| 20.9 x 20.9 x 24 mm block | yes | +1.07 mm | only standing on its square face: that face's 29.6 mm diagonal fits and 24 mm is under 25.4 mm; on any other face its 31.8 mm diagonal does not |
| 34 mm rubber ball | no | -1.15 mm | 34 mm is wider than the 31.7 mm cylinder |
| 28 x 60 mm peg | no | -14.2 mm | narrow enough, but 60 mm is longer than the 57.1 mm deepest point, and tilting it to use the deep side makes it too wide |
| 60 mm block | no | -36.2 mm | far too big |

Every close call clears the fixture by at least 1.0 mm either way, so the classification doesn't depend on
simulation precision or on how carefully a real-world judge measures.

## Scoring

Scored by `score()` and `success()` in the task file, every control step, from the toys' poses. The task file
computes each toy's extent from its pose and true shape (the simulator's own bounding boxes are padded by up
to 5 mm).

- **Hard 0:** any toy ends with its lowest point more than 5 cm below the table top (fell off the table).
- **Otherwise:** `0.9 x max(0, H/4 - W/3)`
  - `H`: hazards (of 4) wholly in the bin: inside the inner walls (2 mm tolerance for a toy resting against a
    wall), no part above the rim.
  - `W`: safe toys (of 3) not resting on the mat: the toy's centre is not over the mat, or its lowest point is
    more than 1 cm above the mat's surface (in the bin, on the table, or stacked on another toy). A safe toy that
    tips over but still lies on the mat counts as on the mat.
- **1.0:** all 4 hazards in the bin, all 3 safe toys on the mat, every toy still (under 1 cm/s and 0.5 rad/s), and
  every fingertip of both grippers at least 3 cm from every toy, held for 1 s in a row. This ends the trial.

Examples: doing nothing scores 0. Putting every toy in the bin scores 0 (4/4 - 3/3). Three hazards binned with
nothing else disturbed scores 0.675. Three hazards binned but one safe toy pushed off the mat scores 0.375. All
correct but still holding a toy scores 0.9.

To score a real trial: look at the bin and the mat with a photo or by eye. A toy is in the bin if no part of it is
on or over a wall or above the rim. A safe toy is on the mat if it rests on the mat surface with its centre over
the mat. "Still" and "let go" mean the grippers are open and at least 3 cm from every toy for a second.

## Scripted demo

`demo()` plays an open-loop plan (`yam_scripted` policy). The right arm sets the cube in the bin's far right
quarter and parks over its end of the mat. Meanwhile the left arm waits over its own end, then sets the plastic
block, the spool and the steel ball each in its own quarter of the bin, parks, and holds still. Each grasp comes
down with the fingers open only 8 mm wider than the toy on each side, closes slowly to a bounded squeeze, holds,
lifts, sets the toy down on the bin floor (not dropped, so the ball doesn't roll), opens, waits for the fingers to
clear, and rises. The pick targets include a small per-arm reach correction measured in simulation (the fingertips
close about 3 mm short at the mat). It scored 1.0 in 4 of 4 runs.

## Fidelity notes

- All toys are rigid primitives (box, cylinder, sphere) with realistic sizes and masses for their stated
  materials (wood about 0.7 g/cm3, ABS plastic about 1.05, steel 7.85, solid rubber about 1.1). Friction is the
  simulator default for every object; it can't be set per material in a task file.
- The mat and the bin are fixed in place, like a bin weighted or clipped down. A loose bin would have to be one
  rigid body, which a task file can't build from five boxes.
- Whether a toy is a hazard is decided by the geometry of the real test fixture, checked in code, not by
  simulating the fixture. All close calls have at least 1 mm of margin.
- A flat coin-like hazard (a 25 mm disc lying flat, 3, 6 and 10 mm thick) was tried and left out: the simulated
  parallel gripper rides over or pushes such a disc instead of pinching it off a surface. Real children's toys
  include coins and buttons, so a real-world version of the task could add one.
- Grasp, contact and release are approximate. The release check uses the fingertip positions from the arms'
  joint states by forward kinematics (below).

## Dependencies

- yam-isaac-lab-arena at commit `0059d02a1a91bc6834c1d0846022891df0c78730` (2026-11-01), with Isaac Sim 6.1,
  Isaac Lab 3.0 and Isaac Lab Arena 0.3.
- `success()` checks the release with the arms' fingertip positions. The task-file hooks receive only object
  states, so it reads the joint positions of the `yam_arena` embodiment that calls it (found up the call stack)
  and runs `yam_arena.urdf.forward_kinematics`. This depends on the embodiment's private state at the commit above;
  if the state can't be found, scoring raises an error instead of guessing.
- `small_parts_fit.py` needs only NumPy.
