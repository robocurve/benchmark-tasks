# Set an inspection cover onto its hidden locating dowel

A gearbox housing has one small dowel hole in its top. A grey inspection cover, a plain block too wide to grip, rests
on the housing over the hole, hiding it, tilted 4.6 degrees on a steel locating dowel that sticks out of its
underside, out of sight. The dowel is 18 mm short of the hole and 16 mm to its side, and the hole is off both of the
housing's centre lines, so neither a straight push along either edge of the cover nor centring it on the housing
seats it. Slid over the housing, the cover drags on its dowel
tip; when the dowel comes over the hole it drops in, the cover drops flat and stops sliding. A mechanic finds the
dowel this way, by feel. Lifting an edge to look under it with a wrist camera is also allowed.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), station scene. It uses `parts.py`, a small helper in this folder that builds multi-shape parts (the
housing is several boxes round the hole; the cover is a box with a cylinder under it). It imports only NumPy and
`yam_arena`. `near_misses.py` lists near-successes and wrong outcomes with the score each must get. It checks
`score()` and `success()` in plain Python and needs no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Fit the grey cover on the gearbox so it sits flat on its locating dowel."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/dowel-dream

# Scripted demo (reaches 1.0 after about 36 s of simulated time)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/dowel-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1 to 2 minutes of wall time on
  an L40S once Isaac Sim has started.
- **Seeds:** run with `-E object_jitter=0`, so every object starts exactly where `task.py` puts it and no seed is
  needed. Jitter shifts the cover's start by up to 3 cm, which changes how far the search has to go.

## How the simulator scores it

- Pass/fail, from the cover's pose and true size: its up axis within 1.5 degrees of vertical, all four underside
  corners within 1.5 mm of the housing top, the dowel's tip within 1.5 mm of the hole centre in x and in y and more
  than 5 mm below the housing top, moving under 1 cm/s and turning under 0.5 rad/s.
- Resting tilted on its dowel the cover is at 4.58 degrees (measured), with its near edge about 10 mm up, so a tilted
  cover never passes. The dowel is rigid and 8 mm long, so the cover can lie flat on the housing only with the dowel in the
  hole or off the housing's edge; the dowel position check fails the second.
- The scorer cannot see the grippers, so a cover pressed flat in the hole by a gripper would also count; a reviewer
  scores a cover still held as 0. Holding it cannot make it lie flat anywhere but with the dowel in the hole. The success condition must hold for 4 s (`SUCCESS_HOLD_S`), so the trial also shows it stays seated once
  the arm has moved away.
- The cover's turn about the dowel is not scored: one dowel does not set it.

## Scripted demo

`demo()` reads no object state. It is a fixed raster search: four rows in which the cover is slid sideways across
the housing (planned from 16 mm right of seated to 18 mm left and back), alternately by the right arm pushing the
cover's right face and the left arm pushing its left face, with fingers open so two finger tips meet the face about
9 cm apart. Between rows the left arm pushes the near face 9 mm further from the robot. Once the dowel drops into the
hole the cover cannot slide, and every later push only presses against it. The idle arm waits at its own side.
The plan runs 40.2 s; the arms track short of their targets, so the cover travels less than planned.

Measured with object traces on the L40S (Isaac Lab-Arena, station scene). Offsets are the dowel's from the hole
centre, x then y; tilts are the cover's.
- Search (`demo()`): the dowel starts at (-18, -16) mm. Row 1 takes it to (-18, +14), step 1 to (-13, +15), row 2 to
  (-13, -13), step 2 to (-5, -14). In row 3 its edge catches in the hole as it crosses (tilt 4.58 to 4.36 degrees)
  and the cover stops sideways at (-4, 0); step 3 pushes it in and it drops flat (0.0 degrees, dowel at (+1, +1))
  at about 31.5 s, and stays seated with the arms away. Score 1.0 at about 35.5 s.
- Shortcuts, each a copy of the task with only `demo()` replaced; each scored 0, the cover left tilted 4.58 to 4.59
  degrees:
  - one straight 45 mm push along x: the dowel passes the hole 16 mm to its side and ends at (+21, -16);
  - one straight 40 mm push along y: the dowel passes 17 mm short of the hole and ends at (-17, +19);
  - centre the cover side to side on the housing, then push 45 mm along x: the dowel ends at (+21, -31);
  - one diagonal push (closed fingers at the corner) toward the housing's centre: the cover moved about 7 mm toward
    the robot and 11 mm right and turned 4 degrees, leaving the dowel at (-25, -27).

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

Cases include: seated (1.0), tilted where each single straight push leaves it (0), seated but moving (0), seated turned 20 degrees about the dowel (1.0), dowel 1.5 mm off
the hole centre (1.0) and 2 mm off (0), tilted on its dowel at the start, centred over the hole and just short of it
(0), tilted 1.0 degree (1.0) and 1.4 degrees with a corner 1.6 mm up (0), 1.4 mm (1.0) and 2 mm (0) above
seated, flat with the dowel off the housing's near or side edge (0), upside down (0), and pushed onto the table (0).

## Licence

All geometry is built from primitives in `task.py`; no third-party assets.
