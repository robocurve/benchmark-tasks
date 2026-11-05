# Load four blanks into a gravity magazine

A fixed steel magazine, a square channel 46 mm wide inside, stands on the table. Four 41 mm square aluminium blanks
lie on a tray pad, each turned a different amount. The robot picks each blank, turns it square to the channel,
centres it over the mouth and lets it drop in, until all four lie flat in a stack inside. With 2.5 mm of
clearance per side, a blank turned more than about 7.5 degrees or off-centre by more than 2.5 mm lands on the rim
instead of dropping in.

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM
embodiment for Isaac Lab-Arena), run through Inspect Robots on the `station` scene. `near_misses.py` lists
near-successes and wrong outcomes with the score each must get; they are plain-Python checks of `score()` and
`success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Load all four aluminium blanks from the blue tray into the steel magazine, each lying flat, one on top of another."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/magazine-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/magazine-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in
`task.json`.

- **Runtime per trial:** at most 120 s of simulated time (2,400 steps at 20 Hz); about two minutes of wall time on
  an L40S once Isaac Sim has started (its first start takes several minutes).
- **Seeds:** none needed. Submissions run with `-E object_jitter=0`, so every object starts exactly where
  `task.py` puts it and the layout is the same every trial. With jitter on, seeded resets shift only the four
  blanks.

## Scripted demo

`demo()` drives the left arm. For each blank in turn (nearest the magazine first) it turns the grip to the blank's
angle, grips it across two faces just above the pad, lifts it, turns it square while carrying it, stops with the
blank's bottom 10 mm above the rim over the channel centre, and opens the fingers; the blank drops in. The fingers
never enter the channel. They are commanded to close only 2 mm into each face of the blank: each finger joint's
force is capped at 50 N, and when both were commanded fully closed they sat at that cap and the clamped blank slid
up to 15 mm sideways in the grip while being carried. With the partial close, traces show each blank held within 2
mm of the channel centre before release, and the four end up stacked within 3 mm of it.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases include: four, three, two and one blanks loaded (1.0, 0.75, 0.5, 0.25); the top blank still falling
(0.75); blanks lying against the channel walls (still 1.0); a blank jammed across the mouth or resting on a wall
(0.75 with three below); a blank standing on edge in the channel (does not count); a blank turned 7 degrees, tilted
4.9 degrees about an edge or a diagonal, or upside down, on the floor (counts); tilted 6 degrees (does not count); a
blank turned 45 degrees across the rim (does not count); a top blank 5 mm above the one below it (not supported,
0.75) and 1.5 mm above it (contact, 1.0); three blanks stacked with nothing under them (0); blanks stacked on the
table beside the magazine (0). Rotated cases use real quaternions, with the rotated bounding box the simulator
reports.
