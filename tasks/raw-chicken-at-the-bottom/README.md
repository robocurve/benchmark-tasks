# Pack the shopping into a cool box without putting the raw chicken on top

On the station table: an open cool box in the middle of the far side with room for two food packs side by side, and three packs of shopping in front of the arms: a pink pack of raw chicken (left), a white sandwich box (right) and a yellow block of cheese (far left). All three must go in the cool box, so two of them have to be stacked; the raw chicken must not go on top of the others, where its juices could drip onto food that is eaten without cooking. The cool box is fixed; the packs are loose.

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
  --instruction "pack the shopping into the cool box" \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/raw-chicken-at-the-bottom/dream

# Scripted demo (reaches 1.0 in about 579 steps)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "pack the shopping into the cool box" \
  --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/raw-chicken-at-the-bottom/demo

# Videos (one MP4 per camera, next to each run's frames)
for log in ~/yam-eval/raw-chicken-at-the-bottom/*/*.json; do inspect-robots video "$log"; done
```

The demo is open loop, so it needs `-E object_jitter=0`. The left arm lays the chicken in the left half of the box; the right arm lays the sandwich box beside it in the right half; then the left arm stacks the cheese on the sandwich box, so nothing sits under the chicken for it to drip onto.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s (the episode then ends), and otherwise `score()` at the end of the episode. An object counts as still when it moves under 1 cm/s and 0.5 rad/s, or has not moved at all (under 0.5 mm) since the previous control step: PhysX can put a settled body to sleep while it still reports a small velocity.

| Outcome (`outcome()`) | Condition | Score |
|---|---|---|
| `packed` | All three packs in the cool box, chicken not on top | 1.0 |
| `N_packed` | N packs in the cool box, chicken not on top | N/3 |
| `chicken_on_top` | Raw chicken above the sandwich box or the cheese, whatever else | 0.0 |

A pack is in the box when it rests still with its centre inside the walls and its underside within 10 cm of the table. In the real world, check these by eye or with a tape measure.

## Runtime, seeds and reset

- Run limit: 1,200 steps (1 minute at 20 Hz).
- Runtime per trial on an L40S, including Isaac Sim's start-up (about 25 s once its shaders are cached; the first
  start on a new instance takes several minutes more): about 91 s for the scripted demo and
  150 s for the Dream-1 run.
- Seeds: the layout is fixed (`-E object_jitter=0`), so every trial starts identically; no seed is needed.
- Reset: Put the three packs back on their starting spots. The cool box is fixed (putty) and stays in place.

## Results in simulation

One trial each, with the commands above (Isaac Lab-Arena 0.3.1, `-E object_jitter=0 -E control_hz=20`):

| Run | `task_score` |
|---|---|
| Scripted demo | 1.0 |
| Dream-1 | 0.0 |
