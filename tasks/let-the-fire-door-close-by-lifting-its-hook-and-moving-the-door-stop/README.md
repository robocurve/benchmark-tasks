# Let a fire door close: lift its hold-open hook and take away the door stop

A fire door in a scale-model room corner stands fully open, and its closer is pulling it shut. Two devices of
different kinds hold it open. A hinged hook on the side wall rests on the door's top edge, its end dropped in front
of the door. A heavy door stop stands on the floor against the door. The instruction names neither. The robot has
to find both and remove both, and the door's own closer then shuts it. Each device holds the door alone. Lifting
the hook while the door stop is still there leaves the door where it is, and the hook falls back when let go.
Taking the door stop away leaves the door on the hook.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for
Isaac Lab-Arena, run through Inspect Robots), on the `station` scene. The door and hook are hinged parts built with
`parts.py`, which must sit next to `task.py`. It imports only NumPy and `yam_arena`, and adds hinged parts to the
scene when the embodiment builds it. `near_misses.py` lists near-successes and wrong outcomes with the score each
must get. They are plain-Python checks of `score()` and `success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="This fire door has a closer. Make it free to close by itself."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir logs/hold-open-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir logs/hold-open-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`; `task.json` describes it in words.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz). The demo ends at about 21 s.
  A few minutes of wall time on an L40S once Isaac Sim has started.
- **Seeds:** none needed. Run with `-E object_jitter=0`, as submissions are. The door stop must start touching
  the door: with jitter it can start apart from it, and the door would then swing until it meets it. Keep jitter
  off.

## How the simulator scores it

- **Door angle.** The door's hinge angle is read directly. 0 is fully open (its starting pose) and -90 degrees is
  shut against the stop. "Shut" means within 2 degrees of that stop.
- **Door stop.** It is checked from its true corners (centre, size and orientation), not its bounding box. It must
  be at rest (under 1 cm/s and 0.5 rad/s), with its lowest corner within 1 mm of the table top. An 11 x 11 grid of
  points over its top and bottom faces must all lie outside the swing area and at least 5 mm from its outline,
  which is sampled every 1 mm. Exact limits count, with 1e-6 margin.
- **The hook** is not scored. It cannot stay lifted on its own (it falls back by gravity), so it earns no credit
  by itself. Lifting it is still needed: the door cannot move past it otherwise. The hook's pivot sits
  about 6 mm below the centre of its contact with the door, so the closer's push turns the hook down, not up. The
  door cannot push it off. In the trace, with the door stop gone and the hook down, the door stayed held at 1.9
  degrees from fully open for 5 s.
- **The closer.** At each reset the door joint's spring target is set 0.3 rad past shut (`parts.on_reset`). The
  door starts fully open with the closer already pulling.
- **Success** (score 1.0) must hold for 3 s before the trial ends, so the video shows the arm letting go of the
  hook and the hook dropping.

## Scripted demo

`demo()` uses the left arm only; the right arm stays where it starts.

1. **The door stop.** The arm grips it with the fingers closing along x, opened to about 56 mm so they stay clear
   of the hook. The grip is 15 mm off the block's centre, away from the door, because the finger tips are wider
   than the block. The arm carries it toward the robot's right, away from the hook (which stands between the door
   stop and the robot), and releases it 7 mm above the table at (17.0, -6.0).
2. **The hook.** The arm grips the hook's knob from above, tips 4 mm above the arm, and lifts along the hook's arc,
   commanded to 0.9 rad. In the trace the hook reached about 26 degrees, enough for its end to clear the door's
   top. The door swings shut in about 3 s. The arm holds the hook, lets go, and lifts away, and the hook drops
   to hang beside the side wall.
