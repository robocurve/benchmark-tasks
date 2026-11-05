# Corbel a bridge across a gap with bricks shorter than the gap

Two fixed stone piers stand 180 mm apart. Eight loose red-orange bricks, each 120 mm long, lie beside the arms. No brick can
span the gap, so the robot has to corbel: stack courses out from each pier, each one stepping further over the
gap, until the top bricks from the two sides meet over the middle. Every brick off the table must rest on something
and pass a centre-of-mass check, so a brick held in a gripper, or a stack that only stands while held, earns nothing.

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM
embodiment for Isaac Lab-Arena, run through Inspect Robots), on the `station` scene. `near_misses.py` lists
near-successes and wrong outcomes with the score each must get; they are plain-Python checks of `score()` and
`success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Build a bridge over the gap between the two grey blocks with the bricks: stack bricks out from both blocks so that the top bricks from the two sides meet over the middle of the gap. At the end, no brick may rest on the table in the gap."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir <dir>

# Scripted demo (reaches 1.0 after about 40 s and the run ends once it has held 1 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir <dir>
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 2 minutes of wall time on an
  L40S once Isaac Sim has started.
- **Seeds and jitter:** run with `-E object_jitter=0`, as above; every object then starts exactly where `task.py`
  puts it and no seed is needed. The demo's aim offsets were measured at this layout, so the scripted demo needs
  `object_jitter=0`; a policy run could use jitter, since the bricks start on open table.

## Scripted demo

`demo()` uses both arms: the left arm builds the left side and the right arm the right side, three courses each.
Each brick is gripped from above across its 40 mm width (fingers closing front to back), so no finger passes
beyond a brick's inner end into the gap. The courses' inner ends are 73, 50 and 1.5 mm from the midline: steps of
17, 23 and 48.5 mm, which keep the centre of mass of every partial stack at least 11 mm inside its support's edge.
The right arm waits 10 s before its top course so the two hands are never over the middle together. The fingers
open only partway, slowly, letting each brick go 6 mm above its course. Each course's target is offset by where that arm's brick came to rest
in a trace run (8 to 12 mm front to back for the left arm, up to 10 mm side to side for the right arm's top brick).
The spare brick on each side is not used.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases include: the demo bridge (1.0); tips 4.8 mm apart (1.0) and 6 mm apart (0.5); tips meeting with one side
only 78 mm out (0.5) or offset 25 mm front to back (0.5); a spare brick as a counterweight (1.0); one side built and
the other two courses (0.25); both sides 60 mm out (0.5) and 59 mm out (0); courses stepped so far that the stack's
centre of mass is past the pier's edge (0); a top brick overhanging past the course below (0); a brick held 5 mm
above its course or held in the air elsewhere (0); a brick dropped, or stood on end as a prop, on the table in the
gap (0); a spare dropped on the table outside the gap (1.0); a top brick still moving (0); every brick turned
1.7 degrees (1.0); a top brick turned 6 degrees or tilted 4 degrees (0); a top brick 1.5 mm above its course (0) or
0.8 mm above (1.0); a held counterweight (0); a centred capstone across two-course sides (1.0) and one 40 mm off
centre (0); a top brick on its 40 mm side (0); equal 30 mm steps and two courses per side (both 1.0); turned top
bricks at the 5 mm, 80 mm and 60 mm limits, judged from their true end faces, including two turned 4.9 degrees whose
ends are truly 7 mm apart (0.5); and bricks stacked with no overhang (0).
