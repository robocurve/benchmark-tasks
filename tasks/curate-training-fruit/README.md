# Curate a robot dataset: fruit of a known kind in the bowl, a new kind on the plate

The robot is curating a robot-learning dataset. A grey mat shows the kinds of fruit already in the training set (an apple and an orange). Each loose fruit of a known kind goes in the bowl and each new kind goes on the plate. The nearest target is the wrong one for the lemon and the orange. The bowl, plate, apple and orange are the station's own models, where Dream-1's training puts them.

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
  --instruction "put fruit of a kind shown on the training set mat in the bowl, and fruit of a new kind on the plate" \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/curate-training-fruit/dream

# Scripted demo (reaches 1.0 in about 394 steps)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "put fruit of a kind shown on the training set mat in the bowl, and fruit of a new kind on the plate" \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/curate-training-fruit/demo

# Videos (one MP4 per camera, next to each run's frames)
for log in ~/yam-eval/curate-training-fruit/*/*.json; do inspect-robots video "$log"; done
```

The demo is open loop, so it needs `-E object_jitter=0`. The left arm puts the apple in the bowl; the right arm puts the lemon on the plate and then the orange in the bowl, once the left arm is clear.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s (the episode then ends), and otherwise `score()` at the end of the episode. An object counts as still when it moves under 1 cm/s and 0.5 rad/s, or has not moved at all (under 0.5 mm) since the previous control step: PhysX can put a settled body to sleep while it still reports a small velocity.

| Outcome (`outcome()`) | Condition | Score |
|---|---|---|
| `curated` | Apple and orange in the bowl, lemon on the plate | 1.0 |
| `R_right_W_wrong` | R fruits on their right target, W on the wrong one | R/3 - W/6, at least 0 |

A fruit is in the bowl when it rests still within 6 cm of the bowl's centre with its bottom below the rim, and on the plate when it rests still within 6 cm of the plate's centre with its bottom within 3 cm of the plate's top. In the real world, check these by eye.

## Runtime, seeds and reset

- Run limit: 1,200 steps (1 minute at 20 Hz).
- Runtime per trial on an L40S, including Isaac Sim's start-up (about 25 s once its shaders are cached; the first
  start on a new instance takes several minutes more): about 133 s for the scripted demo and
  382 s for the Dream-1 run.
- Seeds: the layout is fixed (`-E object_jitter=0`), so every trial starts identically; no seed is needed.
- Reset: Put the apple, lemon and orange back on their starting spots (see `setup`). The bowl, plate, mat, its fruit and the card are fixed.

## Results in simulation

One trial each, with the commands above (Isaac Lab-Arena 0.3.1, `-E object_jitter=0 -E control_hz=20`):

| Run | `task_score` |
|---|---|
| Scripted demo | 1.0 |
| Dream-1 | 0.0 |
