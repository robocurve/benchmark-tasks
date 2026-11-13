# Give the last slice of cake to plate number 1, not the nearest customer

On the station table: three of the station's plates in a row across the far side, each with a printed ticket number beyond it showing the order its customer arrived: 2 on the left, 3 in the middle, 1 on the right. One slice of cake is left, standing just left of centre in front of the arms, nearest the middle plate (ticket 3). The instruction says to put it on plate 1, on the far right, so the robot must read the numbers rather than use the nearest plate. The plates and tickets are fixed; the cake is loose.

| File | What it is |
|---|---|
| `task.json` | The task's details: instruction, setup, scoring, run limit, fidelity, scores, media ID |
| `task.py` | The `yam_arena` task file: the objects, `success()`, `score()`, `outcome()` and the scripted `demo()` |

## Requirements

The Isaac Lab-Arena environment from the program's Brev launchable (simulation walkthrough), with the `yam_arena`
embodiment, `inspect-robots` and the Dream-1 client. The task uses only `yam_arena`'s task-file objects and the
standard `task_score` scorer; nothing else to install.

## Run a trial

From this folder, after `source <ARENA_DIR>/yam.env`:

```bash
# Dream-1, at the run limit
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "one slice of cake is left: put it on the plate numbered 1" \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/last-cake-to-first-customer/dream

# Scripted demo (reaches 1.0 in about 192 steps)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "one slice of cake is left: put it on the plate numbered 1" \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/last-cake-to-first-customer/demo

# Videos (one MP4 per camera, next to each run's frames)
for log in ~/yam-eval/last-cake-to-first-customer/*/*.json; do inspect-robots video "$log"; done
```

The demo is open loop, so it needs `-E object_jitter=0`. The right arm carries the cake to the plate of ticket 1.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s (the episode then ends), and otherwise `score()` at the end of the episode. An object counts as still when it moves under 1 cm/s and 0.5 rad/s, or has not moved at all (under 0.5 mm) since the previous control step: PhysX can put a settled body to sleep while it still reports a small velocity.

| Outcome (`outcome()`) | Condition | Score |
|---|---|---|
| `first_served` | The cake on plate 1 | 1.0 |
| `ticket_N` | The cake on plate N (2 or 3) | 0.0 |
| `unserved` | The cake on no plate | 0.0 |

The cake is on a plate when it rests still with its centre within 6.5 cm of the plate's centre and its bottom within 2 cm of the plate's top. In the real world, check these by eye or with a tape measure.

## Runtime, seeds and reset

- Run limit: 1,200 steps (1 minute at 20 Hz).
- Runtime per trial on an L40S, including Isaac Sim's start-up (about 25 s once its shaders are cached; the first
  start on a new instance takes several minutes more): about 50 s for the scripted demo and
  207 s for the Dream-1 run.
- Seeds: the layout is fixed (`-E object_jitter=0`), so every trial starts identically; no seed is needed.
- Reset: Put the cake back on its starting spot. The plates and tickets are fixed (putty or tape) and stay in place.

## Results in simulation

One trial each, with the commands above (Isaac Lab-Arena 0.3.1, `-E object_jitter=0 -E control_hz=20`):

| Run | `task_score` |
|---|---|
| Scripted demo | 1.0 |
| Dream-1 | 0.0 |
