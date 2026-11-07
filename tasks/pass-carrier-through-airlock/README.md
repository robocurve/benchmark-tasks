# Pass a wafer carrier through an airlock whose two gates are never open at once

A low-friction lane slopes down to the left through a small chamber with a mesh roof. A red gate closes each end of the
chamber, and each falls shut when let go. A wafer carrier rests against gate A. The robot must pass it through to the stop
on the left and shut both gates, without the gates ever being open at the same time.

Semiconductor technicians pass carriers between cleanroom zones this way (O*NET 51-9141.00): only one door of a
pass-through airlock may be open, or dirty air reaches the clean side. The task needs both arms for a measured reason: with
the same grip, the right arm can't reach gate B and the left arm can't reach gate A (an offline check of the arms'
kinematics gives over 30 mm of error at the far gate, against 0.08 mm at the near one). So the arms must take turns, each
waiting for the other's gate to shut. It tests a rule that holds across the whole sequence, and across both arms.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()`, `score()` and the two-arm `demo()` |
| `rigidparts.py` | Rigid parts built from several boxes, here also tilted onto the slope (the lane and the carrier) |

yam_arena's objects are single primitives turned only about the vertical, so `rigidparts.py` builds the lane and carrier. It
wraps `yam_arena.scene.task_object` and `object_states` when they load, without modifying yam_arena, so the task runs
with the standard `yam_arena` embodiment, `yam_scripted` policy and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pass the carrier through the airlock to the stop on the left and shut both gates. The two gates must never be open at the same time, and the carrier must stay on the lane." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/airlock-dream
```

The scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pass the carrier through the airlock to the stop on the left and shut both gates. The two gates must never be open at the same time, and the carrier must stay on the lane." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/airlock-demo
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. Run one trial per
process, or let the scorer see the reset: it clears its record when the carrier jumps more than 3 cm back up the lane in one
control step.

## Scripted demo

1. The right arm grips gate A and raises it about 4.5 cm, and the carrier slides into the chamber, stopping at gate B.
2. The right arm lowers gate A. The left arm, gripping gate B all along, waits for that.
3. The left arm raises gate B, the carrier slides out to the stop, and the left arm lowers gate B.

The demo reaches 1.0 (success) in about 330 steps (16 s) with `-E object_jitter=0`, on NVIDIA L40S, RTX A6000 and RTX A5000
GPUs, and with 5 mm of object jitter in two of two runs. Dream-1 (`dreamlabs/dream-1-beta`, 2,400 steps) scored 0.0. The right
arm's half alone scored 0.25, the carrier shut in the chamber; raising gate B while gate A was open scored 0.5.

## Scoring

Evaluated every control step; the score at the end of the episode counts. A gate is open when its lower edge is more than
1 cm above where it closes, and shut when within 3 mm.

- 0 if the carrier ever rises more than 1 cm off the lane or moves more than 2 cm from its centre line, so it can't be
  carried past the gates.
- 1.0 (success, held for 1 s): the carrier's nose within 1 cm of the stop, both gates shut, never open at the same time,
  carrier and gates still.
- 0.5: the carrier's nose within 1 cm of the stop and both gates shut, but they were once open at the same time.
- 0.25: the carrier wholly between the gates, both shut, never open at the same time.
- Otherwise 0.

In a real trial: a side video against 1 cm marks on a card beside each gate shows when each is open; a tape measure from
the stop checks the carrier.

## Runtime and seeds

A Dream-1 trial of 2,400 steps took about 15 minutes on one RTX-class GPU, mostly waiting on the model, plus about 1 minute
for Isaac Sim to start (longer on a fresh instance). The scripted demo takes under a minute.
The layout is fixed (`-E object_jitter=0`); there are no seeds to choose. The demo is open-loop and tuned to this layout,
so it may not reach 1.0 with object jitter.

## Fidelity

All parts are rigid. There is no air: the gates are plain plates sliding in channels, without seals or an electrical
interlock, and the rule that they are never open together is what is scored. The lane is a low-friction surface, so the
carrier slides. Nothing is deformable.

## Real-world build

A 48 cm lane with a slick plastic strip on a 10-degree slope, 6.6 cm wide between 2 cm rails, with a stop at its low end; two
11 x 11 x 2 cm ABS plates in steel channels; a small chamber of sheet metal with a roof of bars; a hollow 6 x 10 x 4 cm
plastic box of 0.15 kg. Keep hands clear of the gates when they are raised.
