# Draw the bottom bar out of a small pile while holding the bars it carries

A small pile of pine bars: three side by side on the table, and two short bars lying across the middle one, the
red bar. The red bar is 4 mm thicker than its neighbours, so it alone carries the two crossing bars. The robot has
to draw the red bar out endwise and lay it on a yellow pad without the crossing bars or the other bottom bars
moving more than 1 cm or turning more than 5 degrees. Pulled on its own, the red bar carries the crossing bars away with it, so one arm
has to hold them still while the other draws the bar out, and then set them down on the side bars.

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM
embodiment for Isaac Lab-Arena, run through Inspect Robots), on the `station` scene. `near_misses.py` lists
near-successes and wrong outcomes with the score each must get; they are plain-Python checks of `score()` and
`success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Draw the red bar out endwise from under the two crossing bars and lay it on the yellow pad. Do not let any of the other four bars move more than 1 cm or turn more than 5 degrees while you do it."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/draw-bar-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/draw-bar-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.
The scorer keeps one piece of trial history (whether a protected bar has moved more than 10 mm or turned more than
5 degrees). It is cleared only at a new trial: on the first step it sees, or when every bar is back within 0.5 mm of
its starting pose and at rest after a jump of more than 20 mm in one control step (the simulator's reset). Putting
the bars back by hand, step by step, does not clear it. Each `inspect-robots run` starts a fresh scorer.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started (its first start takes several minutes).
- **Seeds and jitter:** run with `-E object_jitter=0`, as above; every object then starts exactly where `task.py`
  puts it, and no seed is needed. Jitter shifts each loose bar independently, which would push the bars of the
  pile into each other, so the task needs `object_jitter=0`.

## Scripted demo

`demo()` uses both arms and reaches 1.0 after about 22 s. The right arm comes down over the pile, waits for itself
to settle, and squeezes the two crossing bars together, its fingers closing left to right with the finger tips 8 mm
above the red bar. The left arm grips the red bar's protruding end from above, its fingers closing front to back,
and draws the bar 290 mm toward its own side, along its length, while the right arm holds the crossing bars still.
Once the red bar is out, the crossing bars hang in the right gripper about 1 mm above the side bars; the right arm
raises them 4 mm more and opens slowly, and they drop onto the side bars (at about 14.8 s in the confirming trace).
The left arm lets go of the bar's end, grips it again at its middle, carries it to the pad and opens its fingers
fully with the bar's underside 7 mm above the pad (5 mm once the arm's sag is counted); the bar lands at about
21.0 s, after the fingers are open, and the 1.0 condition holds from then on.

The demo corrects for three effects measured in traces: both grippers close turned about 4 degrees from their
targets (uncorrected, this turns the crossing bars and the red bar, and the red bar shoves the far side bar), the
right arm needs a pause to settle before it closes, and fully closed fingers let the crossing bars slide between
them and kick them sideways on release, so the right fingers squeeze with their pads 4 mm inside the pair's faces.
In the confirming trace (same plan as the demo run), the protected bars moved at most 6.5 mm (a crossing bar, while
the right fingers closed on it) and 2.1 mm (the far side bar).

In the Dream-1 run (0.0), the robot pulled the red bar with one arm; the crossing bars came with it and the pile
was pulled apart.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases include: the red bar on the pad and the crossing bars down (1.0); each stated limit from both sides (the
red bar exactly 1 mm and 1.01 mm above the pad, the crossing bars exactly 1 mm and 1.01 mm above the side bars,
the red bar's side exactly 1 mm and 1.01 mm past the pad's edge, a side bar moved just under and just over 10 mm, a
crossing bar turned 4.99 and 5.01 degrees); the red bar held 2.5 mm above the pad or the crossing bars held 1.5 mm
above the side bars (0); the red bar overhanging the pad (0.5), drawn out and left on the table (0.5), drawn 200 mm,
out from under the crossing bars (0.25), drawn 150 mm with the crossing bars still on it (0); rotated states (the
red bar yawed 3 degrees or rolled onto its side on the pad, 1.0; standing on its end or propped up at one end, 0; a
side bar tipped over, 0); the crossing bars carried along with the red bar (0); and the crossing bars lifted 25 mm
and set back exactly where they started by hand, or a side bar pushed 12 mm and pushed back (0 for the rest of the
trial). The rotated cases are run by `python near_misses.py`; the factory checker runs the unrotated ones.
