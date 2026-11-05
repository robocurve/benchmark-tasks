# Set a pin at the one spot equally far from three survey marks

Three survey marks are fixed to the table; nothing else is drawn. The robot has to stand a loose pin at the one
point that is the same distance from all three: the centre of the circle through them. The marks lie on one side
of that centre, so it is outside the triangle they form and is not the centroid or the midpoint of any two marks.

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM
embodiment for Isaac Lab-Arena), run through Inspect Robots on the `station` scene. `near_misses.py` lists
near-successes, exact tier limits and wrong guesses with the score each must get; they are plain-Python checks of
`score()` and `success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Stand the blue pin upright on the table at the one spot that is the same distance from all three orange survey marks, then let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/pin-dream

# Scripted demo (reaches 1.0 in about 11 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/pin-demo
```

The run prints `task_score`: 1.0 if the success condition held for 1 s, otherwise `score()` at the last step.
Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started.
- **Seeds:** none needed. Submissions run with `-E object_jitter=0`, so every object starts exactly where `task.py`
  puts it. With jitter on, seeded resets shift only the pin; the marks are fixed.

## Scripted demo

`demo()` drives the left arm. The target is computed in `task.py` from the three mark positions (the
circumcentre). The arm grips the pin 20 mm below its top, lifts it 110 mm clear of the table (the marks are 80 mm
tall), carries it over the centre, lowers it to about 7 mm above the table and opens, so the pin drops and stands
on its own before any credit. The target is offset 2.4 mm to the robot's right to cancel the left arm's measured
tracking error while carrying a light object. In the recorded trace the pin came to rest about 1.3 mm from the
centre.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score and spread
```

The cases include: the pin at the centre on either end (1.0), tilted 4 and exactly 5 degrees (1.0) and 6 degrees
(0), lying down, moving or held 2 mm up (0), the bottom exactly 1 mm up (1.0), each tier limit exactly and just
past it in two directions, and the pin left at its start, at the centroid, at the incentre, at the midpoint of
marks A and C (spread 33 mm), between A and B, beside a mark, or balanced on a mark (all 0).
