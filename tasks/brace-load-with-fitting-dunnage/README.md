# Brace a crate in its bay with the one dunnage block that fits the gap

A loose crate stands in a walled load bay, pushed against the far wall, with a 62 mm gap between it and the near
wall. Four wooden blocks of the same colour and height, 44, 54, 60 and 66 mm thick, stand beside the bay. The robot
has to judge the gap, pick the block that fills it (60 mm), and stand it in the gap so the crate can slide no more
than 3 mm.

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM
embodiment for Isaac Lab-Arena, run through Inspect Robots), on the `station` scene. `near_misses.py` lists
near-successes and wrong outcomes with the score each must get; they are plain-Python checks of `score()` and
`success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="The crate can slide in its bay. Fill the gap between the crate and the near wall of the bay with the one wooden block that fits it closely, so the crate can slide no more than 3 mm."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir <dir>

# Scripted demo (reaches 1.0 after about 12 s, then holds it for 1 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir <dir>
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started.
- **Seeds and jitter:** run with `-E object_jitter=0`, as above; every object then starts exactly where `task.py`
  puts it and no seed is needed. Jitter would shift the crate off the far wall and change the gap, so the task needs
  `object_jitter=0`.

## Scripted demo

`demo()` drives the left arm. It grips the 60 mm block from above across its 70 mm width, 18 mm below its top, so
the fingers stay above the crate and the walls; carries it over the gap; lowers it in; opens 3 mm above the floor
and lifts away. Gripping turns the block about 4.9 degrees and the arm sags about 4 mm short while carrying it, so
the demo turns the grip back by that angle and aims 4.2 mm long (both measured in trace runs).

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases include: the 60 mm block against the wall or the crate (1.0), the 54 mm block (0.5), the 44 mm block
(0), the right block still moving or held 3 mm above the floor (0), on the crate's top or bridging the gap on the wall
tops (0), the crate pushed to the near wall with the block in the far gap (1.0) or with no block (0), the crate lifted
onto the walls (0), and rotated blocks, whose simulator bounding boxes are widened: the 60 mm block turned 1.5 degrees
(1.0) or lying on its 75 mm face (1.0), the 54 mm block turned 4.9 degrees (0.5, not 1.0), the 54 mm block leaning
2.9 degrees and turned 1.4 degrees (0.5), the 44 mm block leaning 2.9 degrees and turned 3.9 degrees (0), and the
60 mm block leaning 4 degrees or turned 6 degrees (0). Play uses each block's own thickness, not its bounding box.
