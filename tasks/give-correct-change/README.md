# Give a customer exactly the right change

On the station table: a change dish in the middle of the far side, a printed card beyond it reading "PAID 1.00" and "PRICE 0.30", and five chunky play coins standing in a row in front of the arms, each with a printed card in front of it giving its value in pence: 50, 10, 5, 20 and 10. The customer paid one pound for a 30p item, so their change is 70p: 50 + 20, or 50 + 10 + 10. Short-changing the customer is the failure. The dish and cards are fixed; the coins are loose.

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
  --instruction "give the customer their change: they paid one pound for a 30p item; put exactly the right coins in the change dish" \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/give-correct-change/dream

# Scripted demo (reaches 1.0 in about 386 steps)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "give the customer their change: they paid one pound for a 30p item; put exactly the right coins in the change dish" \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/give-correct-change/demo

# Videos (one MP4 per camera, next to each run's frames)
for log in ~/yam-eval/give-correct-change/*/*.json; do inspect-robots video "$log"; done
```

The demo is open loop, so it needs `-E object_jitter=0`. The left arm puts the 50p in the dish; the right arm waits above its side, then puts the 20p beside it.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s (the episode then ends), and otherwise `score()` at the end of the episode. An object counts as still when it moves under 1 cm/s and 0.5 rad/s, or has not moved at all (under 0.5 mm) since the previous control step: PhysX can put a settled body to sleep while it still reports a small velocity.

| Outcome (`outcome()`) | Condition | Score |
|---|---|---|
| `exact` | Exactly 70p in the dish | 1.0 |
| `over_N` | More than 70p (the shop loses) | 0.25 |
| `short_N` | Less than 70p, including none (the customer is short-changed) | 0.0 |

A coin is in the dish when it rests still with its centre inside the walls and its underside within 7 cm of the table (so a coin on another counts). In the real world, check these by eye or with a tape measure.

## Runtime, seeds and reset

- Run limit: 1,200 steps (1 minute at 20 Hz).
- Runtime per trial on an L40S, including Isaac Sim's start-up (about 25 s once its shaders are cached; the first
  start on a new instance takes several minutes more): about 182 s for the scripted demo and
  585 s for the Dream-1 run.
- Seeds: the layout is fixed (`-E object_jitter=0`), so every trial starts identically; no seed is needed.
- Reset: Stand the five coins back on their marks behind their value cards. The dish and the cards are fixed (putty or tape) and stay in place.

## Results in simulation

One trial each, with the commands above (Isaac Lab-Arena 0.3.1, `-E object_jitter=0 -E control_hz=20`):

| Run | `task_score` |
|---|---|
| Scripted demo | 1.0 |
| Dream-1 | 0.0 |
