# Set a lintel across two free-standing posts

Two loose square posts stand upright on the table, 100 mm apart. Nothing holds them, and a light sideways push near
the top tips one over. The robot picks up a wooden lintel from the right arm's side of the table, lowers it flat onto both
post tops at once, centred, and lets go, without knocking over or shifting either post.

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM
embodiment for Isaac Lab-Arena, run through Inspect Robots), on the `station` scene. `near_misses.py` lists
near-successes and wrong outcomes with the score each must get; they are plain-Python checks of `score()` and
`success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Lay the wooden lintel flat across the tops of the two standing posts, centred on them, and let go. Do not knock over or move either post."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/lintel-dream

# Scripted demo (reaches 1.0 in about 11 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/lintel-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in
`task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1 minute of wall time on an
  L40S once Isaac Sim has started (its first start takes several minutes).
- **Seeds:** none needed. Submissions run with `-E object_jitter=0`, so every object starts exactly where
  `task.py` puts it. The task needs `object_jitter=0`: the scorer measures post movement from the posts'
  positions in `task.py`, so a seeded reset that shifts a post would count as the robot moving it.
- **History:** the scorer keeps each post's largest displacement and rotation from its start over the trial, in
  module state. It is cleared only by a scene reset: every object within 0.5 mm of its start and at rest, reached
  by a jump of over 20 mm in one control step. A robot standing a post back on its mark moves it there step by
  step, so that does not clear it. Load the task afresh, or reset the scene, for each trial.

## Scripted demo

`demo()` drives the right arm with its grip turned 90 degrees, so the fingers close across the lintel's 40 mm width
at its middle. It lifts the lintel, carries it with its underside 20 mm above the post tops to over the posts'
midpoint, and lowers it toward 8 mm above the tops; carrying the lintel the arm settles lower, and in the run
the lintel hung about 1.5 mm above the tops until the fingers opened, then dropped onto them. It lets go and
rises. The fingers stay between the
posts, at least 45 mm from either one (checked on the arm's link geometry along the plan).

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases run in order and share the post record, like the steps of one trial; a reset case puts everything back
in one jump and starts a new trial. They include: centred (1.0); overhangs differing by 9.8 or 12 mm (1.0, 0.7);
9.9 or 12 mm off sideways (1.0, 0.7); 30 mm off along the lintel, still on both posts (0.7); a post top centre not
under the lintel (0); held still 1.5 mm above the tops (0) or 0.9 mm above (1.0, inside the 1 mm seat test); still
moving (0); the lintel turned 8, 10 or 20 degrees about the vertical (1.0, 0.7, 0), upside down (1.0), on its edge
(0) or tilted 6 degrees (0); a post leaning 2.5 or 4 degrees (1.0, 0); a post turned in place 12 or 45 degrees
(1.0, 0); a post shifted 15 mm, also after it is pushed back (0.5); posts pushed 30 or 70 mm together (0.5, 0);
posts swapped (0); a post lifted and put back exactly before the lintel is set (0); a post toppled and stood back
up step by step to within 0.4 mm of its mark before the lintel is touched, then the lintel set (0); posts laid down
with the lintel across them (0).
