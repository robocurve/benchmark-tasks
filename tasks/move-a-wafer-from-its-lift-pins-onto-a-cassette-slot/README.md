# Move a wafer from its lift pins onto a cassette slot

A wafer lies on three lift pins. The robot lifts it by its edge, carries it to an open-topped cassette slot, lowers
it between the side walls onto the two narrow ledges, pushes it back against the two stop posts and lets go, without
dropping it. The side walls leave only 6 mm beside the wafer, so the fingers have to hold it front and back; the
back finger passes through the gap between the stop posts.

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM
embodiment for Isaac Lab-Arena, run through Inspect Robots), on the `station` scene. `near_misses.py` lists
near-successes and wrong outcomes with the score each must get; they are plain-Python checks of `score()` and
`success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Move the wafer from the three lift pins onto the two ledges of the cassette slot, lying flat and pushed back against both stop posts, then let go. Do not drop it."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=<path to this folder>/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 900 --store-frames --log-dir <log dir>

# Scripted demo (reaches 1.0 in about 12 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=<path to this folder>/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 900 --store-frames --log-dir <log dir>
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 45 s of simulated time (900 steps at 20 Hz); about 1 to 2 minutes of wall time on
  an L40S once Isaac Sim has started (its first start takes several minutes).
- **Seeds:** none needed. Run with `-E object_jitter=0`, as submitted: every object starts exactly where `task.py`
  puts it. With jitter on, seeded resets shift the wafer by up to the jitter distance, which can move it off the
  pins, so the task needs `object_jitter=0`.
- **History:** `task.py` keeps two records between control steps. The drop rule is judged over the whole trial: a
  record of whether the wafer ever moved faster than 0.5 m/s, cleared only at a scene reset (the wafer back within
  0.5 mm of its start after a jump of more than 20 mm in one step). At rest is judged from the wafer's last 10 poses
  (at most 0.5 mm and 0.5 degrees of change), because the simulator reports a small velocity for it while it lies
  still on the ledges.

## Scripted demo

`demo()` drives the left arm with the fingers closing front to back: it opens around the wafer on the pins, grips
its edge with the finger tips 2 mm below its underside, lifts it 80 mm, carries it over the slot, lowers it between
the walls, slides it 5 mm back against the stop posts and lets go about 7 mm above the ledges. The TCP aims 3 mm high
to cancel the arm's measured sag while carrying.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases include: seated (1.0), seated upside down (1.0), 2 mm and 3 mm short of the stop (1.0 and 0.5), mid-slot
(0.5), off-centre against one post (0.5), too near the slot's open end (0), held 3 mm above the ledges (0), sliding
(0), cross-slotted on one ledge (0), on the pins, the table, the wall tops, the post tops or the table between the
ledges (0), and a drop followed by a seated wafer (0, until a scene reset; putting it back on its pins by hand does not clear it). Each end state is held for 10 steps, because at rest needs 10 still steps; the cases also cover every limit exactly and just past it.
