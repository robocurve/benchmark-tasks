# Swing a mitre saw's turntable to its 45 degree detent with the squeeze release and leave it locked

A 10 inch compound mitre saw sits on the bench, front toward the arms, set square with its head raised. Its turntable
is held at detents by a spring plunger. A paddle in front of the turntable's front handle lifts the plunger when it is
squeezed against the handle. The robot is told to set the saw to a 45 degree mitre and leave it locked in the detent.

Setting the mitre angle is a routine step of every trim, framing or frame cut. The turntable only turns while the
release is squeezed, and it only locks if it is let go exactly at the detent. Let go between detents, it is left loose
and the cut comes out wrong. The task tests turning a heavy turntable through an arc by its handle while holding a
release, and letting go at the right point.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the saw, its turntable and release paddle, the detent plunger, `success()` and `score()` |
| `partkit.py` | Rigid parts built from boxes and cylinders, with hinges, for yam_arena task files |
| `demo.py` | The scripted full-score demo and its negative variants |

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set the mitre saw to a 45 degree mitre and leave it locked in the detent." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir dream_out
```

## Scripted demo

```bash
python demo.py task.py demo_out                    # full score: writes demo_*_cam.mp4, setup.png, result.json
DEMO_VARIANT=short python demo.py task.py short    # lets go at 25 degrees: not at 45 (0.25)
DEMO_VARIANT=held python demo.py task.py held      # swings to 45 degrees but never lets go: the paddle stays squeezed (0.25)
```

The right arm closes on the front handle from above. Its fingers go across the handle, so the near finger squeezes
the paddle against it. The arm swings the turntable along the handle's arc, turning the gripper with it, and trims
the swing until its own measured tool point reads 44 degrees. Then it opens, so the released plunger catches the
45 degree detent as the table settles, and lifts away. Full score is reached in 356 steps. The demo reads only the
layout and its own joints.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`).

- 0 if the saw is moved 10 mm on the bench at any time. This is permanent.
- 1.0 (success, which must hold for 1 s): locked in a 45 degree detent (either side) with the paddle let go, both
  grippers clear of the handle and paddle, the table still.
- 0.75: locked at 45 degrees but a gripper is still at the handle or the table still moves.
- 0.5: locked in another mitre detent (15, 22.5 or 31.6 degrees).
- 0.25: the table was swung 10 degrees at some point.
- Otherwise 0.

## Runtime and seeds

About 1 minute per trial for the full 1,200 steps, plus Isaac Sim start-up. The layout is fixed (`object_jitter=0`);
there are no seeds to choose.

## Fidelity

Rigid bodies at true size. The detent plunger is modelled in the task code: while the paddle is squeezed 2.7 mm the
turntable turns freely; let go within 1.5 degrees of a detent it is pulled into the detent and held stiffly
(1,000 N m/rad), elsewhere it is left free. The turntable turns against 3 N m s/rad of damping. The head is fixed in its raised position; the motor, blade guard and lock knob
are simplified to boxes and a disc.

## Real-world build

- A 10 inch compound mitre saw with a squeeze-type mitre detent release on its front handle, unplugged, on the bench
  against two screwed-on stops.
- Mitre detents as built into the saw (0, 15, 22.5, 31.6, 45 degrees).

Read the result from the saw's mitre scale. Check the lock by pushing the handle gently without squeezing the release.
