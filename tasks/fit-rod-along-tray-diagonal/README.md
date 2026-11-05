# Fit a rod that is longer than the tray by laying it corner to corner

An orange rod, 220 mm long, lies beside a tray whose inside is 200 x 150 mm. The rod is longer than both inside
sides, so it goes in flat only when turned to lie corner to corner (between about 30.5 and 37.5 degrees from the
tray's long side). The robot has to see that, pick the rod up, turn it to the diagonal over the tray, set it
down on the floor and let go.

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM
embodiment for Isaac Lab-Arena, run through Inspect Robots), on the `station` scene. `near_misses.py` lists
near-successes and wrong outcomes with the score each must get; they are plain-Python checks of `score()` and
`success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Put the long orange rod in the grey tray, lying flat on the tray floor, then let go."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/rod-dream

# Scripted demo (reaches 1.0 in about 12 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/rod-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in
`task.json`: 1.0 wholly inside the tray, flat on its floor and at rest; 0.3 propped in the tray with one end on
the floor and part of it over a wall; 0 otherwise.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started (its first start takes several minutes).
- **Seeds:** none needed. Runs use `-E object_jitter=0`, so the rod starts exactly where `task.py` puts it and
  the layout is the same every trial. With jitter on, seeded resets shift only the rod.

## Scripted demo

`demo()` drives the left arm: it grips the rod at its middle from above, lifts it 14 cm, turns it 34 degrees
while carrying it over the tray's centre, lowers it to 7 mm above the floor and lets go, so the rod drops onto the floor and the 1 mm floor-contact test cannot pass while it is held. The place target is
offset 2.4 mm in y and 4 mm in height to cancel the arm's measured tracking error while carrying.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases include: flat at 31, 34, 37 and -34 degrees (1.0); held 2 mm or 4 mm above the floor, still moving, or one end
raised 6 mm (0); propped on a wall top (0.3) and the same still sliding (0); resting across two wall tops along
the tray or turned 25 degrees (0); stood on end or leaning steeply (0); flat on the table beside the tray (0).
