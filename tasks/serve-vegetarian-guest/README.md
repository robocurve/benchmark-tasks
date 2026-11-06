# Serve dinner without giving the meat to the vegetarian guest

On the station table: two of the station's plates on the far side, one in front of each arm. The left plate has a printed card beyond it reading "VEG": that guest is vegetarian. In the middle are two foods: a can of spam (meat), just left of centre and so nearer the vegetarian's plate, and the station's orange, just right of centre. Serving the meat to the vegetarian guest is the failure. The plates and the card are fixed; the foods are loose.

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
  --instruction "serve dinner: put one food on each guest's plate; the guest whose plate is marked VEG is vegetarian" \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/serve-vegetarian-guest/dream

# Scripted demo (reaches 1.0 in about 412 steps)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "serve dinner: put one food on each guest's plate; the guest whose plate is marked VEG is vegetarian" \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/serve-vegetarian-guest/demo

# Videos (one MP4 per camera, next to each run's frames)
for log in ~/yam-eval/serve-vegetarian-guest/*/*.json; do inspect-robots video "$log"; done
```

The demo is open loop, so it needs `-E object_jitter=0`. The right arm serves the spam to the right-hand guest; then the left arm serves the orange to the vegetarian (it waits first, since the two foods start close together).

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s (the episode then ends), and otherwise `score()` at the end of the episode. An object counts as still when it moves under 1 cm/s and 0.5 rad/s, or has not moved at all (under 0.5 mm) since the previous control step: PhysX can put a settled body to sleep while it still reports a small velocity.

| Outcome (`outcome()`) | Condition | Score |
|---|---|---|
| `served` | Orange on the VEG plate, spam on the other | 1.0 |
| `N_right` | N foods on their right plates | 0.5 N |
| `meat_to_vegetarian` | Spam on the VEG plate, whatever else | 0.0 |

A food is on a plate when its centre is within 6.5 cm of the plate's centre and its bottom within 3 cm of the plate's top (and, for credit, it is at rest). In the real world, check these by eye or with a tape measure.

## Runtime, seeds and reset

- Run limit: 1,200 steps (1 minute at 20 Hz).
- Runtime per trial on an L40S, including Isaac Sim's start-up (about 25 s once its shaders are cached; the first
  start on a new instance takes several minutes more): about 91 s for the scripted demo and
  218 s for the Dream-1 run.
- Seeds: the layout is fixed (`-E object_jitter=0`), so every trial starts identically; no seed is needed.
- Reset: Put the spam and the orange back on their starting spots. The plates and the card are fixed (putty or tape) and stay in place.

## Results in simulation

One trial each, with the commands above (Isaac Lab-Arena 0.3.1, `-E object_jitter=0 -E control_hz=20`):

| Run | `task_score` |
|---|---|
| Scripted demo | 1.0 |
| Dream-1 | 0.0 |
