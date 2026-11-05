# Line in three ranging rods between two fixed poles

Two fixed yellow poles stand 447 mm apart on a diagonal across the table, one in front of the left arm and one
further out in front of the right arm. Nothing is drawn between them. The robot stands three loose red rods
upright between the poles so that all five are in one straight line, spread out along it, as a surveying
technician lines in ranging rods between two control points.

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM
embodiment for Isaac Lab-Arena, run through Inspect Robots), on the `station` scene. `near_misses.py` lists
near-successes and wrong outcomes with the score each must get; they are plain-Python checks of `score()` and
`success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Stand the three red rods upright on the table between the two yellow poles so that all five are in one straight line, with every gap between neighbouring rods or poles along the line at least 7 cm and at most 16 cm."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/rods-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/rods-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in
`task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started (its first start takes several minutes).
- **Seeds:** none needed. Submitted runs use `-E object_jitter=0`, so every object starts exactly where
  `task.py` puts it and the layout is the same every trial. With jitter on, seeded resets shift only the three
  loose rods.

## Scripted demo

`demo()` uses both arms. The right arm grips rod 3 near its top, carries it to the point three quarters of the
way from pole A to pole B, lowers it to about 2 mm above the table, lets go and parks out of the way. The left arm
does the same with rod 1 (to the quarter point) and then rod 2 (to the midpoint), so the rods end 111.8 mm apart
along the line. Rod 1's target is offset 2 mm toward the robot to cancel a landing error measured in traces on
the L40S (rod 1 landed about 4 mm beyond the line when aimed at it).

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases include: rods 5 mm off the line (1.0), 6 and 10 mm (0.6), 12 and 20 mm (0.3), 21 mm (0), an uneven
spacing with every gap between 70 and 160 mm (1.0), a first gap of exactly 70 mm (1.0) and of 65 mm (0), two rods
60 mm apart (0), the rods bunched in the middle (0), a rod on the line extended beyond either pole (0), a rod lying
down, still moving, held 10 mm up or stacked on another (0), and one rod left at its start (0).
