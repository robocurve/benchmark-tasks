# Push a crate too wide to grip round the corner of a walled maze without driving it into the dead end

A wooden crate, wider than the open gripper, stands in a walled corridor. The corridor turns off to the left
partway along and carries straight on into a dead end. The robot has to push the crate along the corridor, stop it
where it lines up with the side corridor, move round to its side and push it down the side corridor into the goal
marked by green tape on the wall tops. Pushed on into the dead end, the crate is hard to recover with one arm;
entering it (the crate's centre passing 440 mm out) scores the whole trial 0, as does lifting the crate more than
20 mm.

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM
embodiment for Isaac Lab-Arena, run through Inspect Robots), on the `station` scene. `near_misses.py` lists
near-successes and wrong outcomes with the score each must get; they are plain-Python checks of `score()` and
`success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Push the wooden crate through the maze into the goal between the green marks, without pushing it into the dead end. Keep it on the floor; do not lift it."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/maze-dream

# Scripted demo (reaches 1.0 in about 16 s)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 450 --store-frames --log-dir ~/yam-eval/maze-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in `task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); about 1 minute of wall time on an
  L40S once Isaac Sim has started.
- **Seeds:** none needed. Run with `-E object_jitter=0`, as above: the crate then starts exactly where `task.py`
  puts it. The scorer's whole-trial record (lifted, dead end) is cleared only on a real reset: the crate appearing
  within 0.5 mm of its start pose, at rest, by a jump of more than 20 mm between two scorer calls. Carrying it back
  to its start does not clear it.

## Scripted demo

`demo()` drives the right arm with its fingers closed. It lowers the finger tips to 8 mm above the table behind
the crate, pushes the crate along the straight corridor until its centre is 370 mm out (lined up with the side
corridor), backs off, lifts over the wall tops, lowers the fingers into the 60 mm gap between the crate and the
right wall, and pushes the crate down the side corridor to its end wall. The first push aims 40 mm further than
the crate's goal position: measured with the trace wrapper, the arm's compliance under the push load leaves the
finger that much short.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable; prints each case's score
```

The cases include: the crate against the end wall or the far wall, turned 3 and 10 degrees, and with its near
corners exactly on the goal line (1.0); 0.1 mm and 3 mm short of the line, a turned corner out, the centre on the
left wall line (0.5); lined up at the bend at exactly 350, 370 and 390 mm (0.25) and at 349.9 and 390.1 mm (0);
still sliding, tipped, propped at 8 degrees, pushed out of the open end (0); lifted 15 mm and set down in the goal
(1.0); centre exactly 440 mm out and then into the goal (1.0); 441 mm out, then into the goal (0); lifted 30 mm,
set back down at its start and pushed to the goal (0); lifted over the walls into the goal (0).

## Known limits

- With both arms, the crate could be clamped from its two sides (the corridor leaves 60 mm beside it) and drawn
  back out of the dead end along the floor, but entering the dead end has already scored the trial 0.
- Nothing stops a policy from sliding the crate while gripping it from two sides with both arms, as long as it
  stays on the floor.

## Simulator details behind the scoring

The scorer counts the crate as stopped when it moves under 1 cm/s and turns under 0.5 rad/s. The whole-trial record
(lifted, dead end) is cleared only when the crate is reset to its start pose between trials. The trial ends at 1.0
once the 1.0 condition has held for 1 s; otherwise it scores the condition at the last step. Contact friction is the
simulator's rubber-like value (1.2 static, 1.0 dynamic) for the crate and walls, higher than wood on a table.
