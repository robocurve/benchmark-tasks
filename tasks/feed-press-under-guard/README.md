# Feed a press blank under a guard raised no more than 8 cm

A small press bed sits under a fixed mesh hood on the station table. The only way in is under a red polycarbonate guard that
slides in two channels and falls shut when let go. A flat aluminium blank lies in the infeed lane, and a wooden push
stick lies on the table. The robot must raise the guard no more than 8 cm, push the blank in to the back stop with the
stick, take the stick out, and lower the guard gently.

Press operators work this way (O*NET 51-4031.00): hands stay out of the point of operation, and parts are fed with a hand
tool. The task needs both arms for a physical reason: the guard falls when let go, and the arm that holds it can't also
feed the blank. Nothing can reach the bed from above, since the hood's mesh gaps are 15 mm. The task tests a sustained
hold within a numeric limit, tool use, a fixed order of steps, and a controlled release.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()`, `score()` and the two-arm `demo()` |

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Raise the guard no more than 8 cm, push the blank in to the back stop with the push stick, take the stick out and lower the guard." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/press-dream
```

The scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Raise the guard no more than 8 cm, push the blank in to the back stop with the push stick, take the stick out and lower the guard." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/press-demo
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. Run one trial per
process, or let the scorer see the reset: it clears its record when the blank jumps more than 3 cm back toward its start in one control step.

## Scripted demo

1. The right arm grips the guard's top edge and raises it about 4 cm (the plate slips a little in the fingers), then holds it.
2. The left arm takes the stick by its tail, sets it in the lane behind the blank, pushes the blank to the back stop and
   draws the stick back out.
3. The right arm lowers the guard over 3 s and lets go.

The demo reaches 1.0 (success) in about 380 steps (19 s) with `-E object_jitter=0`, on NVIDIA L40S, RTX A6000 and RTX A5000
GPUs. Dream-1 (`dreamlabs/dream-1-beta`, 2,400 steps) scored 0.0.

The push stops about 1.6 cm short of where the hand goes, because the stick gives a little in the grip, and closing the
fingers shifts the stick 4 mm across the lane; the demo allows for both (`PUSH_LAG` and `STICK_GRIP_SHIFT` in `task.py`).

## Scoring

Evaluated every control step; the score at the end of the episode counts. The scorer keeps two records through the trial:
the highest the guard's lower edge has been above the table, and the fastest the guard has moved downward.

- 0: the guard's lower edge ever rose more than 13 cm, which takes it out of its channels.

- 1.0 (success, held for 1 s): the blank lies flat with its front edge within 5 mm of the back stop; the guard's lower
  edge is within 3 mm of the table; the stick is wholly left of the guard; the guard was never raised over 8 cm and
  never moved down faster than 0.3 m/s; blank and guard are still.
- 0.75: the blank and guard end that way, with the guard raised at most 12 cm and never moving down faster than 0.3 m/s.
- 0.5: they end that way, but the guard was raised more than 12 cm, up to 13 cm, or moved down faster than 0.3 m/s.
- 0.25: the blank is wholly right of the guard, but those end conditions aren't met.
- Otherwise 0.

In a real trial: a side video at 30 frames a second against 8, 12 and 13 cm marks on a card beside the guard gives the
raise; 0.3 m/s is 1 cm between frames; a tape measure from the back stop checks the blank.

## Runtime and seeds

A Dream-1 trial of 2,400 steps took about 13 minutes on one RTX-class GPU, mostly waiting on the model, plus about 1 minute
for Isaac Sim to start (longer on a fresh instance). The scripted demo takes under a minute.
The layout is fixed (`-E object_jitter=0`); there are no seeds to choose. The demo is open-loop and tuned to this layout,
so it may not reach 1.0 with object jitter.

## Fidelity

All parts are rigid boxes with the station's default contact material. The guard is a plain plate sliding in channels,
without counterweights or an interlock. The press itself is not modelled: the blank only has to reach the back stop.
Nothing is deformable.

## Real-world build

A 14 x 16 cm polycarbonate plate, 2 cm thick, in two 13 cm steel channels; a hood of sheet metal and bars
with 15 mm gaps; two low guide strips; a 6 x 10 x 2 cm aluminium block; a 2 x 3 x 20 cm wooden stick. Keep the hood fixed
to the table, and keep hands clear of the guard when it is raised.
