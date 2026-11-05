# Clear the swing path of a fire door

A closed fire door in the far wall of a scale-model room opens toward the robot on its hinge side. Nothing marks
the floor area it sweeps. The robot has to work out the quarter circle from the hinge and the door's width (plus
the strip the handle sweeps at the very end), move the three items inside it (a chair, a waste bin and a carton)
and leave them standing in the room.

The test is in what it must leave alone. A storage box stands in the square corner beyond the arc, its nearest
corner only 12.6 mm outside it. It is in the way of a door read as a rectangle, and of a door hinged at the other
end. A plant pot stands beside the hinge, behind the open door's line. Moving either fails the trial, judged over
the whole trial. So the robot cannot clear everything near the door, as it could for a marked keep-clear area or
a fixed-width band. It has to get the arc right.

The task is one file, `task.py`, for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM
embodiment for Isaac Lab-Arena, run through Inspect Robots), on the `station` scene. `near_misses.py` lists
near-successes and wrong outcomes with the score each must get. They are plain-Python checks of `score()` and
`success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="This fire door opens into the room, towards you, on the side with the black hinge. Move everything that would stop it swinging fully open, a quarter turn, and leave each thing you move standing upright inside the taped room. Do not move anything that is not in the door's way."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir logs/fire-door-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir logs/fire-door-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`, described in words in
`task.json`.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); a few minutes of wall time on an
  L40S once Isaac Sim has started.
- **Seeds:** none needed. Run with `-E object_jitter=0`, as submissions are: every object then starts exactly
  where `task.py` puts it. The storage box starts only 12.6 mm outside the swing area, so jitter can change which
  items are in the door's way. Keep jitter off.
- **Trial history:** the scorer remembers, across steps, whether the storage box or the plant pot has moved more
  than 10 mm or tipped. The record clears only when the simulator resets the scene (every item back within 0.5 mm
  of its start after a jump of more than 20 mm in one step).

## Scripted demo

`demo()` drives the left arm through three top-down pick-and-place moves. The fingers open only part way (66 mm
between the pads) so they fit between neighbouring items. Each grip has the finger tips 18 mm below the item's top,
and each release is 7 mm above the floor: the chair to (18.0, +18.0) cm, the waste bin to (18.0, +8.0)
and the carton to (16.0, -3.0). It carries each item 14 cm above the table, so it passes over the storage box
and never touches it or the plant pot.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable
```

The cases include: each tier (0, 1/3, 2/3, 1.0); items exactly 5 mm and 4.9 mm from the swing area, at the arc
and beyond the handle strip; a carton just past the open door's line but where the handle ends up (in the way); a chair whose square outline is clear but whose corner enters the swing when turned
45 degrees; tipped, lifted, stacked, sliding and out-of-room items; the storage box nudged exactly 10 mm (allowed),
9.9 mm toward the hinge so that it is in the door's way (fails), and 10.1 mm (fails); a protected item moved and put back (still 0); clearing the whole corner (0); and moving the
storage box as if the hinge were on the other side (0).
