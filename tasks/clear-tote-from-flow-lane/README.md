# Pull a red-tagged tote from a flow lane without letting the queue move more than 2 cm

A short gravity flow lane slopes down to the left, ending at a fixed stop. Three totes stand nose to tail in it: a red
one at the stop and two grey ones behind it. The robot must take the red tote out and put it in a green bin, keeping the
grey totes from moving more than 2 cm until it is in the bin, then let them slide forward to the stop.

Warehouse and distribution workers clear totes from gravity flow lanes this way (O*NET 53-7062.00, 53-7065.00). The task
needs both arms for a physical reason: the moment the red tote leaves, nothing but a hand stops the queue sliding, and
the rule holds until the red tote is in the bin, so one arm can't snatch it out first. The task tests a sustained hold, a
transfer by the other arm, and a release timed on what the other arm has done.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()`, `score()` and the two-arm `demo()` |
| `rigidparts.py` | Rigid parts built from several boxes, here also tilted onto the slope (the lane and the totes) |

yam_arena's objects are single primitives turned only about the vertical, so `rigidparts.py` builds the lane and totes. It
wraps `yam_arena.scene.task_object` and `object_states` when they load, without modifying yam_arena, so the task runs
with the standard `yam_arena` embodiment, `yam_scripted` policy and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hold back the grey totes, put the red tote in the green bin, then let the grey totes slide forward to the stop. They must not move more than 2 cm until the red tote is in the bin." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/lane-dream
```

The scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hold back the grey totes, put the red tote in the green bin, then let the grey totes slide forward to the stop. They must not move more than 2 cm until the red tote is in the bin." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/lane-demo
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. Run one trial per
process, or let the scorer see the reset: it clears its record when the queue jumps more than 3 cm back up the lane in one control step.

## Scripted demo

1. The right arm grips the first grey tote near its back end and holds it, which holds the whole queue.
2. The left arm grips the red tote near its front end, lifts it straight out, carries it to the bin and lets go. The hands
   grip the far ends of the two totes because they need about 19 cm between them.
3. The right arm lets go and rises, and the grey totes slide forward to the stop.

The demo reaches 1.0 (success) in about 310 steps (16 s) with `-E object_jitter=0`, on NVIDIA L40S, RTX A6000 and RTX A5000
GPUs. Dream-1 (`dreamlabs/dream-1-beta`, 2,400 steps) scored 0.0. The left arm's half alone, with nothing holding the queue,
scored 0.5: the red tote reached the bin and the queue slid forward, but early.

## Scoring

Evaluated every control step; the score at the end of the episode counts. The scorer records how far along the lane the
grey tote next to the red one has moved until the red tote is first in the bin.

- In the bin: the red tote's centre is inside the walls and its lowest point within 1 cm of the table.
- Forward: that grey tote's centre is within 5 mm of where the red tote's started, the other grey tote is behind it, and
  both are within 1 cm of the lane's centre line.
- 1.0 (success, held for 1 s): in the bin, forward, the queue moved no more than 2 cm before the red tote was in the bin,
  and all three are still.
- 0.5: in the bin and forward, but the queue moved more than 2 cm first.
- 0.25: in the bin, but not forward.
- Otherwise 0.

In a real trial: a side video against 2 cm marks on the lane's rail gives the queue's movement; a tape measure from the
stop checks where the totes end.

## Runtime and seeds

A Dream-1 trial of 2,400 steps took about 11 minutes on one RTX-class GPU, mostly waiting on the model, plus about 1 minute
for Isaac Sim to start (longer on a fresh instance). The scripted demo takes under a minute.
The layout is fixed (`-E object_jitter=0`); there are no seeds to choose. The demo is open-loop and tuned to this layout,
so it may not reach 1.0 with object jitter.

## Fidelity

All parts are rigid. The lane is a low-friction surface (0.05) in place of real rollers, so the totes slide rather than
roll, and the totes are solid blocks in place of hollow ones. Nothing is deformable.

## Real-world build

A 44 cm lane of skate wheels or a slick plastic strip on a 10-degree slope, 6.6 cm wide between 2 cm rails, with a stop at
its low end; three 6 x 12 x 5 cm blocks of 0.25 kg, one painted red; a low 16 cm bin.
