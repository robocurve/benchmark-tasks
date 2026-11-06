# Test both self-closing gates, tag the one that stays open, and leave both shut

A scale-model guard fence has two gates that look the same. The left one has a working closer and swings shut
when let go. The right one's closer has failed, so it stays wherever it is left. The robot cannot tell them apart
by looking. It has to open each gate, let go and watch. Then it puts the red tag on the pad beside the gate that
stayed open and shuts that gate by hand, because the barrier must not be left open.

The task is `task.py` for the `yam_arena` embodiment of yam-isaac-lab-arena (the program's YAM embodiment for
Isaac Lab-Arena, run through Inspect Robots), on the `station` scene. The gates are hinged parts built with
`parts.py`, which must sit next to `task.py`. It imports only NumPy and `yam_arena`, and adds hinged parts to the
scene when the embodiment builds it. `near_misses.py` lists near-successes and wrong outcomes with the score each
must get. They are plain-Python checks of `score()` and `success()` and need no simulator.

## Run a trial

From this folder on a machine set up by the yam-isaac-lab-arena launchable:

```bash
I="Check that each gate shuts by itself after you open it and let go. Put the red tag on the white pad beside any gate that does not, and leave both gates shut."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir logs/gates-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 1200 --store-frames --log-dir logs/gates-demo
```

The run prints `task_score`. Scoring is `score()` and `success()` in `task.py`; `task.json` describes it in words.

- **Runtime per trial:** at most 60 s of simulated time (1,200 steps at 20 Hz); the demo ends at about 28 s.
  A few minutes of wall time on an L40S once Isaac Sim has started.
- **Seeds:** none needed. Run with `-E object_jitter=0`, as submissions are, so the tag starts exactly where
  `task.py` puts it. The gates are fixed and are not jittered.

## How the simulator scores it

- **Joint angles.** Each gate's hinge angle is read directly: 0 is shut against the stop, positive is open.
  "Shut" means at most 5 degrees.
- **Left gate self-closed.** Its angle reached at least 15 degrees and later came back to 5 degrees or less in a
  control step in which the right gate moved under 0.1 degree. A robot holding the left gate and pushing it shut
  while it works the right gate does not qualify. Pushing it shut by hand while the right gate stands still would,
  as the simulator cannot see the gripper; on the real rig the video shows it.
- **Stood open.** The right gate has stood open when its angle stayed at least 15 degrees for 20 control steps in
  a row (1 s) while moving less than 0.1 degree per step. The simulator cannot see the gripper, so a gate held
  perfectly still also counts. On the real rig the video shows whether it was let go. This rule stops the robot
  from earning credit by pushing the right gate straight back shut, which never reveals that it fails.
- **What these two tests do not see.** Both are measured from the gate angles alone, so a hand could fake them:
  pushing the left gate from 15 degrees to shut while the right gate stands still, or holding the right gate
  open and still for 1 s. Neither the scorer nor the scoring text checks that the arm let go; a video judge applies
  the instruction's "let go".
- **Tag order.** If the tag touches either pad before both tests are done (its box overlapping the pad seen from
  above, its bottom within 5 mm of the pad's top), the trial scores 0 from then on. This stops a policy that always
  tags the right pad from scoring by testing afterwards.
- **The tag** is checked from its true corners (centre, size and orientation), not its bounding box. It must be at
  rest (under 1 cm/s and 0.5 rad/s), with a face within 10 degrees of level, its lowest corner within 1 mm of the
  pad's top, and every corner over the pad. Exact limits count, with 1e-6 margin.
- **Trial history.** `score()` and `success()` keep a record across steps: how far each gate has opened, and
  whether the left gate has self-closed, whether the right gate has stood open, and whether the tag touched a pad
  too early. The record clears when the scene resets. `parts.on_reset` clears it, and
  so does a scene reset seen in the states: both gates within 0.1 degree of shut, and the tag at rest within 0.5 mm
  of its start after jumping more than 20 mm in one step. Clearing only removes credit.
- **The closer.** The left gate's spring target is set to -0.5 rad at each reset, so the spring presses the gate
  against its stop instead of only reaching it slowly. Without the reset hook it still closes, more slowly.
- **Success** (score 1.0) must hold for 3 s before the trial ends, so the video shows the arm letting go of the
  right gate first.

## Scripted demo

`demo()` uses both arms. The left arm waits 3 s first, so it lets go of its gate after the right gate has come
to rest. Each arm comes down over its gate's knob, with the fingers closing along the leaf and the
finger tips 12 mm above the leaf's top. It pulls the gate open along its arc to 46 degrees, turning the grip with
the leaf, then opens the fingers and lifts away. The left gate swings shut (about 1 s), and the left arm parks. The
right gate stays at about 45 degrees. The right arm waits 2 s, picks the tag, and releases it 7 mm above the right
pad. Then it grips the knob where the gate stood (44.6 degrees in the trace) and drives it to -8 degrees. The arm
gives about 7 degrees, so the gate ends against its stop. The arm then lets go and lifts. The closing angle was
measured with the `trace` wrapper on an L40S.

## Checks without a simulator

```bash
python near_misses.py   # needs yam_arena importable
```
