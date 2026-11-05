# Take only the top plate off a pile in a destacking cradle

Three aluminium plates (the top one blue) lie stacked between two guide rails that leave no room for fingers beside them. The robot
has to take only the top plate and lay it flat on a green pad, without dragging, lifting or tilting the two plates
under it. The plate cannot be gripped where it lies; it has to be slid out past the open end of the cradle first.

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM
embodiment for Isaac Lab-Arena), on the `station` scene. `near_misses.py` lists near-successes and wrong outcomes
with the score each must get; they are plain-Python checks of `score()` and `success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Put only the top plate of the pile on the green pad, lying flat. Leave the other two plates stacked where they are."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/plate-dream

# Scripted demo (reaches 1.0 in about 20 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/plate-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.
Whether a lower plate has moved is judged over the whole trial: `task.py` keeps that record in a module-level
variable and clears it only on a scene reset (every plate back within 0.5 mm of its start after a jump of more than
20 mm in one step); rebuilding the pile by hand does not clear it. The top plate is blue, so a swap is visible.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1 to 2 minutes of wall time
  on an L40S once Isaac Sim has started (its first start takes several minutes).
- **Seeds:** none needed. Run with `-E object_jitter=0`, as submissions are: every object then starts exactly
  where `task.py` puts it. The task needs `object_jitter=0`: jitter would push the plates into the rails, which
  leave 3 mm of clearance.

## Scripted demo

`demo()` drives the left arm. Its closed finger tips, at the top plate's mid-height (6 mm above the middle plate),
push the plate's near end so it slides about 58 mm along the cradle and overhangs the far end of the pile. The arm
is soft under load: pushing the plate it stops 50 to 65 mm short of its target (measured in traces), so the demo
aims 65 mm further. The open fingers then come down just past the rail ends, close across the overhanging part's
80 mm width, lift the plate, carry it over the pad and release it with its underside about 7 mm above the pad.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases include the plate on the pad (1.0), turned 10 degrees or tilted 2 degrees on it (1.0), every limit at
exactly its value (passes), held 1.4 mm above the pad (0), turned 25 degrees or overhanging the pad (0.5), on the
table out of the cradle's lane (0.5), pushed off either end into the lane (0), slid 60 mm or 40 mm along the pile
(0.25) but not 35 mm (0), the pile rebuilt by hand after a violation (still 0), tilted 4 or 15 degrees, standing on edge, held above the pad or still moving (0), and the unintended
strategies: both upper plates lifted together, the middle plate swapped onto the pad, the whole pile pushed, and a
lower plate pulled out and put back (all 0).
