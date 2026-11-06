# Set each stack-nest tote the right way round so it stacks on the full tote instead of nesting onto its parts

Two fixed blue totes each hold an orange parts box. Yellow ledges stick out from their side walls 15 mm below the
rim. Two empty grey totes stand in front of them; each has two full-height slots in each side wall, placed off
centre, with their edges marked dark. Turned one way, a grey tote's slots line up with the ledges and it nests down onto the parts box. Turned end
for end, solid wall lands on the ledges and it stacks clear of the parts. The left grey tote starts in the nesting
orientation and the right one in the stacking orientation, so the robot has to read each tote's slots against the
ledges and turn only the left one.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena), station scene. It uses `parts.py`, a small helper in this folder that builds multi-shape parts (the totes
are made of several boxes each); it imports only NumPy and `yam_arena`. `near_misses.py` lists near-successes and
wrong outcomes with the score each must get. It checks `score()` and `success()` in plain Python and needs no
simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Stack each empty grey tote on the full blue tote behind it."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/totes-dream

# Scripted demo (reaches 1.0 after about 12 s of simulated time)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/totes-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1 to 2 minutes of wall time on
  an L40S once Isaac Sim has started.
- **Seeds:** run with `-E object_jitter=0`, so every object starts exactly where `task.py` puts it and no seed is
  needed. With jitter on, seeded resets would shift the grey totes and parts boxes by up to the jitter distance, and a
  parts box could be pushed into a blue tote wall; use `object_jitter=0`.

## How the simulator scores it

- A grey tote counts as stacked on a blue tote when each of its four outer top corners, computed from its pose and
  true dimensions, is within 1.5 mm of 100 mm above the table (the ledge tops are at 45 mm and the tote is 55 mm
  tall), its centre is within 6 mm of the blue tote's centre in x and y, and it moves under 1 cm/s and turns under
  0.5 rad/s. The scorer cannot see the gripper. The 1.5 mm band is tighter than the demo's release height (5 to 7 mm
  above the ledges), so a tote still held at release height does not count.
- "Pressing on the parts" is kept as a record over the whole trial: once any grey tote's lowest point goes below
  43 mm above the table (2 mm below the ledge tops, so past the ledges and on its way to the parts box) while its
  centre is over a blue tote's inside, that blue tote can no longer qualify, so the trial scores 0 from then on. A tote turned to stack is
  held at 45 mm by the ledges and cannot reach 43 mm. The record is cleared only at a new trial: when every loose object is back
  within 0.5 mm of its start, at rest, having got there by a jump of more than 20 mm in one control step (a scene
  reset, which the robot cannot produce).
- A parts box counts as unmoved when its centre is within 5 mm of its start.
- Either grey tote may go on either blue tote.

## Scripted demo

Each arm pinches the near end wall of its grey tote from above, with the fingers closing front to back. The left arm
lifts its tote 25 mm clear of the blue rim, turns it end for end in two quarter turns while carrying it out, and
lowers it into the blue tote's mouth. It lets go with the underside about 6 mm above the ledges. The right arm
carries its tote straight across without turning it and lets go about 5 mm above the ledges. Both arms aim off by
the tracking error measured in GPU traces (right arm: 4.5 mm in x, 2 mm in y; left arm: 2.4 mm in y, 4 mm in z), so
each tote enters its 4 mm-per-side opening centred.

A GPU trace of the same demo without the turn showed the left tote nest flat onto its parts box (underside 35 mm
above the table), so that blue tote did not qualify and the trial scored 0.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable
```

The task is pass/fail: 1.0 only when both blue totes qualify, otherwise 0. The cases include: both stacked (1.0);
each one alone (0); never turning, so the left tote nests (0); nesting and then lifting out, turning and stacking
(still 0, from the trial record); always turning, so the right tote nests (0); the two grey totes swapped between
blue totes (1.0); lowered nest-wise to an underside height of 43.0 mm (no penalty) versus 42.9 mm (penalty); 1.5 mm
above the ledges (1.0) versus 2 mm or 10 mm (held, 0); tilted 2 degrees; still moving; 6 mm
versus 7 mm off centre; parts box moved 5 mm versus 6 mm; upside down; one grey tote on top of the other; parts box
taken out and the tote nested to the blue floor.
