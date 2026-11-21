# Run a two-cell pool water test

A two-cell pool water tester (colour comparator) stands on its closed kit box with both cells filled to the line with
the water sample. Its loose cover lies on the table. Two squeeze dropper bottles of reagent lie on their sides with
their caps off: the chlorine reagent (yellow) to the robots' left and the pH reagent (red) to their right. Five drops
of each go into the matching cell from a bottle held nose-down; then the cover goes over both cells and the tester is
turned over and back three times with the cover held on, to mix, and stood back on the box.

## Files

- `task.py`: scene, liquid and mixing models, scoring and scripted demo (yam_arena task file). A step hook runs the
  models and prints lines starting `WT|` (state, every 5 control steps) and `WT!` (events: every drop and where it
  landed, every turn of the tester).
- `labkit.py`: parts built from several shapes, with hinges and slides. Each bottle's two wall pads and the cover's
  two lips are sprung slides. It hooks `yam_arena.scene` and the embodiment from the task's import, so no change to
  yam_arena is needed. This copy adds display-only shapes (`DISPLAY`), used to tint a dosed cell and to mark spilled
  drops.
- `arena_assets_fix.py`: imported by `labkit.py`; points Arena library assets at NVIDIA's production bucket. This task
  uses no library assets.
- `task.json`: the task's specification.

## Run a trial

Inside the Isaac Lab-Arena environment with `yam-isaac-lab-arena` installed (the Robocurve Brev launchable):

    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
      --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')" \
      --scorer task_score --max-steps 2400 --store-frames --log-dir logs

Scripted demo: the same command with `--policy yam_scripted` (it takes 102 s, 2,042 control steps).

The score is `task_score` in the log, computed by `score()` in `task.py`.

## How the liquid is modelled

There is no fluid. A bottle's squeeze is the mean inward travel of its two wall pads times their spring rate. While the
bottle's nose is within 20 degrees of straight down and the squeeze is above 3 N, liquid gathers at the tip at
12.5 uL/s per newton above that; when 50 uL has gathered a drop falls straight down; a partly formed drop is drawn back
when the squeeze is let off. A drop lands in a cell only if it is let go above the cell's 14 mm opening (less the
drop's radius), above the rim and no more than 5 cm above it, with the cover off; otherwise it lands on the tester's
top (or the cover), the box or the table and counts as spilled.

## How mixing is modelled

A cell is mixed once it holds its dose and the tester has been turned over and back three times under its cover. A turn
counts when the tester's tilt from upright passes 105 degrees (15 degrees past level, where the sample has run to the
cover) with the cover on, and returns to within 20 degrees of upright. Tipping the tester past 45 degrees without its
cover loses the sample. A dosed cell is drawn pale until it is mixed, then in its full test colour.

The cover is a plate with a sprung lip at each end. The lips stand 2 mm proud of the tester's ends, so fingers that
close on the tester's ends at its top press the lips in first and hold the cover and the tester together. The cover's
own fit on the cells is not modelled: let go of, it stays on only by its weight.

## How the demo works

A lying bottle is grasped from straight above across its two wall pads with about 1.5 N, lifted, and the hand is turned
level, which leaves the bottle hanging nose-down. One squeeze of about 7 N held for a second gives one drop. The cover
is taken by its ends with the hand pitched 20 degrees forward and set on the cells. The tester is then held level by
its ends at the top, lifted 75 mm, and rolled 114 degrees to one side and back three times: the wrist joint's travel is
120 degrees each way, so the tester goes well past level but not fully upside down.

## Runtime and seeds

- 2,400 control steps (2 minutes of simulated time) per trial.
- One fixed starting layout: run with `-E object_jitter=0`. No other randomness.

## Third-party assets

None. Every shape is built in `task.py` from boxes and cylinders.
