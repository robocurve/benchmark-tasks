# Walk a washing machine into its space, corner by corner

A small, heavy washing machine stands just in front of its taped space. Its rubber feet stop it sliding, it is too
wide to grip, and it must not be lifted or tipped over. The way in is to walk it: tip it a few degrees onto one far
corner, swing it about that corner, set it down, then do the same on the other corner.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for Isaac
Lab-Arena, run through Inspect Robots), on the `station` scene. It uses `parts.py` (in this folder) to build the
machine as one rigid body of two coloured boxes with its own friction. `near_misses.py` lists near-successes and
wrong outcomes with the score each must get; it runs without a simulator.

## Run a trial

From this folder, on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Move the washing machine back into its taped space."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames

# Scripted demo (reaches 1.0 after about 34 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames
```

The run prints `task_score`: 1.0 on success, otherwise `score()` at the end of the episode.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz), about 1-2 minutes of wall time on an
  L40S once Isaac Sim has started.
- **Seeds:** none needed. Submissions run with `-E object_jitter=0`, so the machine starts exactly where `task.py`
  puts it.

## Scoring details

`score()` and `success()` read the machine's pose. The tilt is the angle between its own vertical and straight up;
the corners are the cabinet's four bottom corners. Two records are kept over the whole trial: the largest tilt and
the largest height of the lowest corner. A tilt past 20 degrees or a lift of the lowest corner past 2 mm (the walk keeps it under 0.1 mm) scores 0
for the rest of the trial, even if the machine is later stood back in its space. The records clear only on a scene
reset: the machine back within 0.5 mm of its start, upright and still, after a jump of more than 20 mm in one control
step, which the robot cannot cause. "Standing" means every bottom corner within 1.5 mm of the table (and the tilt at
most 5 degrees) and under 1 cm/s and 0.5 rad/s; success must hold for 1 s.

The scoring text asks for the machine set down and left still for 1 s. The scorer cannot see the gripper; it checks
the 1 s stillness, so a machine held perfectly still, upright and on all four feet in its space would also be credited. The
demo lets go before the success hold starts.

## Scripted demo

`demo()` uses fixed waypoints only, worked out from the starting layout. The right arm grips the console from above
with the fingers closing across it. In each of four steps it moves the grip 25 mm toward one far corner and 8 mm up
(the machine tips about 4-7 degrees onto that corner), turns the grip 26 degrees about that corner, and sets the
machine back down on all four feet: far-left corner, far-right, far-left, far-right. The plan expects each step to
turn the machine 90% of the commanded angle, as measured on the GPU. The arm then lets go and moves away.

Failure replays (same command with the scripted policy):
- `WASHER_DEMO=push`: a straight push at mid-height, commanded 6 cm past the front face. Measured: the machine
  rocks 1.5 degrees and settles back where it was (score 0).
- `WASHER_DEMO=high`: a straight push on the console's front, commanded 14 cm past it. Measured: it tips over
  forward within a second (score 0).

- `WASHER_DEMO=drag`: the grip of the demo, then one straight, level move 8 cm toward the space, held 2 s, and let
  go. Measured: it tips forward 12 degrees onto its far edge instead of sliding and, once released, stands 2.4 mm
  from where it started (score 0).

- `WASHER_DEMO=low`: both arms push low on the front face, pad tips 5 mm above the table, 14 cm past the face.
  Measured: it tips forward to 69 degrees (no slide) and falls back where it started (score 0: tilted past 20).
- `WASHER_DEMO=truck`: grip the console, try to tip it back about 15 degrees onto its near edge, drag it 9 cm and stand
  it up. Measured: the arm cannot tip it back (2 degrees); the drag tips it forward 12.5 degrees, and once released it
  stands 2.1 mm from its start (score 0).

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```
