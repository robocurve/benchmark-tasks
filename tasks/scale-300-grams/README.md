# Make a scale read 300 grams by choosing the right weights

On the station table: a round scale pan in the middle of the far side, and four brass weights standing in a row in front of the arms, each with a printed card in front of it giving its mass: 500 g and 100 g on the left, 200 g and 200 g on the right. The robot must make the scale read 300 g: a 100 g and a 200 g weight on the pan and nothing else. Only weights resting on the pan count, so pressing on it or holding a weight over it does not. The pan and the cards are fixed; the weights are loose.

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
  --instruction "make the scale read 300 grams using the weights" \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/scale-300-grams/dream

# Scripted demo (reaches 1.0 in about 386 steps)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "make the scale read 300 grams using the weights" \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/scale-300-grams/demo

# Videos (one MP4 per camera, next to each run's frames)
for log in ~/yam-eval/scale-300-grams/*/*.json; do inspect-robots video "$log"; done
```

The demo is open loop, so it needs `-E object_jitter=0`. The left arm puts the 100 g weight on the pan; the right arm waits above its own side, then puts the nearer 200 g weight beside it.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s (the episode then ends), and otherwise `score()` at the end of the episode. An object counts as still when it moves under 1 cm/s and 0.5 rad/s, or has not moved at all (under 0.5 mm) since the previous control step: PhysX can put a settled body to sleep while it still reports a small velocity.

| Outcome (`outcome()`) | Condition | Score |
|---|---|---|
| `exact` | Exactly 300 g on the pan | 1.0 |
| `off_by_100` | 200 g or 400 g on the pan | 0.5 |
| `reads_N` | Any other reading, including the empty pan | 0.0 |

The scale reads the total mass of the weights resting still on the pan (centre over the pan, underside within 8 cm of its top, so a weight stacked on another counts). In the real world, check these by eye or with a tape measure.

## Runtime, seeds and reset

- Run limit: 1,200 steps (1 minute at 20 Hz).
- Runtime per trial on an L40S, including Isaac Sim's start-up (about 25 s once its shaders are cached; the first
  start on a new instance takes several minutes more): about 150 s for the scripted demo and
  472 s for the Dream-1 run.
- Seeds: the layout is fixed (`-E object_jitter=0`), so every trial starts identically; no seed is needed.
- Reset: Put the four weights back on their marks in front of their labels. The pan and labels are fixed (putty) and stay in place.

## Results in simulation

One trial each, with the commands above (Isaac Lab-Arena 0.3.1, `-E object_jitter=0 -E control_hz=20`):

| Run | `task_score` |
|---|---|
| Scripted demo | 1.0 |
| Dream-1 | 0.0 |
