# Take a heavy tin from the top drawer of a free-standing chest

A small chest of drawers stands loose on the table. Its top drawer holds a heavy red tin at the back. The top panel
sits just above the drawer, so the tin can only be lifted out once the drawer is pulled nearly all the way out (13 cm).
By then the loaded drawer has moved the chest's centre of mass past its front foot rail: if nothing holds the drawer
or the chest, the chest tips forward onto the drawer front. The robot has to keep the chest standing while the tin
comes out (for example by holding the drawer up by its handle, or pressing down on the back of the chest top), put
the tin on the green pad, and close the drawer.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena, run through Inspect Robots), on the `station` scene. The chest is built with `parts.py`, a small parts kit
included in this folder (multi-shape parts with sliders and hinges, built as Isaac Lab articulations; it needs only
NumPy and `yam_arena`). `near_misses.py` lists near-successes and wrong outcomes with the score each must get; they
are plain-Python checks of `score()` and `success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Take the red tin out of the chest's top drawer, put it on the green pad, and close the drawer."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/tin-dream

# Scripted demo (reaches 1.0 in about 20 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/tin-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1-2 minutes of wall time on an
  L40S once Isaac Sim has started.
- **Seeds:** none needed. The task needs `-E object_jitter=0`: jitter would shift the tin inside the drawer into its
  walls. With it, every object starts exactly where `task.py` puts it.

## How the simulator scores it

- The chest's lean is the angle between its own vertical axis and the world vertical. The worst lean over the trial is
  kept in a module-level record; above 10 degrees the trial scores 0 from then on. The record is cleared only by a
  scene reset: the chest (body and drawer) and the tin all within 0.5 mm of their start, at rest, after a jump of more
  than 20 mm in one control step (or at the first step).
- At the end the chest must lean at most 3 degrees, be at rest (under 1 cm/s and 0.5 rad/s), and its body origin (the
  centre of its footprint) must be within 3 cm of its start horizontally.
- Drawer closed: the slider's position is at most 0.01 m.
- Tin on the pad: its centre within the pad's 10 x 10 cm outline, the bottom of its bounding box within 1 mm of the
  pad's top, at rest, and lying flat (one of its body axes within 5 degrees of vertical). Tin on the table: the same
  with the table top in place of the pad. A tin tilted over the pad's edge or leaning on the chest scores 0. The demo releases the
  tin 7 mm above the pad, so a held tin does not count.
- All limits are compared with a 1e-6 margin, so a value exactly at a limit passes.

## Scripted demo

`demo()`: the right arm grips the handle fin from above, pulls the drawer out to its 13 cm stop (commanded 5 mm past
it) and keeps holding it there, which holds the chest up. The left arm lifts the tin out, carries it to the pad and
releases it 7 mm above. The right arm then pushes the drawer shut and lets go. In a GPU trace the chest leaned at most
0.6 degrees, the drawer reached 0.130 m, and the trial ended at 1.0 at step 385. A variant of the same demo whose
right arm let go of the handle right after pulling (the one-arm habit) tipped the chest 29 degrees onto its drawer
front.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The task is pass/fail. The cases include the tin on the pad with the drawer closed (1.0) or open (0), the tin on
the table off the pad (0), the drawer at exactly 1 cm and 1.01 cm open, the tin 1 mm and 1.5 mm above the
pad, the tin's centre exactly on and 1 mm past the pad's edge, the tin tilted 30 degrees over the pad's edge or
leaning 20 degrees on the chest (0), lying on its side (1.0), tilted 4.9 and 5.1 degrees, a moving tin, the chest slid exactly 3 cm and 3.1 cm,
leaning 3 and 4 degrees at the end, a 9.9 degree rock during the trial (allowed), a 10.1 degree lean backwards, a
forward tip followed by the chest being stood back up and the task finished (still 0), and a scene reset.
