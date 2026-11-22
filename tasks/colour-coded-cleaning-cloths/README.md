# Put each cleaning cloth in its area's bucket by the cleaning colour code

On the station table: two cleaning buckets on the far side, the left one with a card beyond it reading "TOILET" and the right one "KITCHEN", and two folded cleaning cloths in front of the arms: a red one just right of centre (nearer the kitchen bucket) and a green one just left of centre (nearer the toilet bucket). Under the cleaning industry's colour code, red cloths are for toilets and green ones for kitchens and food areas; a toilet cloth in the kitchen spreads germs where food is made. The buckets and cards are fixed; the cloths are loose.

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
  --instruction "put each cleaning cloth in the bucket for the area it is used in, following the standard colour code" \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/colour-coded-cleaning-cloths/dream

# Scripted demo (reaches 1.0 in about 386 steps)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "put each cleaning cloth in the bucket for the area it is used in, following the standard colour code" \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/colour-coded-cleaning-cloths/demo

# Videos (one MP4 per camera, next to each run's frames)
for log in ~/yam-eval/colour-coded-cleaning-cloths/*/*.json; do inspect-robots video "$log"; done
```

The demo is open loop, so it needs `-E object_jitter=0`. The left arm takes the red cloth (just right of centre) to the toilet bucket; then the right arm takes the green cloth to the kitchen bucket, once the left arm is clear.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s (the episode then ends), and otherwise `score()` at the end of the episode. An object counts as still when it moves under 1 cm/s and 0.5 rad/s, or has not moved at all (under 0.5 mm) since the previous control step: PhysX can put a settled body to sleep while it still reports a small velocity.

| Outcome (`outcome()`) | Condition | Score |
|---|---|---|
| `sorted` | Red cloth in the toilet bucket, green in the kitchen bucket | 1.0 |
| `R_right_W_wrong` | R cloths in their right buckets, W (the green) in the toilet bucket | R/2 - W/4, at least 0 |
| `toilet_cloth_in_kitchen` | Red cloth in the kitchen bucket, whatever else | 0.0 |

A cloth is in a bucket when it rests still with its centre inside the walls and its underside within 6 cm of the table. In the real world, check these by eye or with a tape measure.

## Runtime, seeds and reset

- Run limit: 1,200 steps (1 minute at 20 Hz).
- Runtime per trial on an L40S, including Isaac Sim's start-up (about 25 s once its shaders are cached; the first
  start on a new instance takes several minutes more): about 122 s for the scripted demo and
  382 s for the Dream-1 run.
- Seeds: the layout is fixed (`-E object_jitter=0`), so every trial starts identically; no seed is needed.
- Reset: Put the two cloths back on their starting spots, folded. The buckets and cards are fixed (putty or tape) and stay in place.

## Results in simulation

One trial each, with the commands above (Isaac Lab-Arena 0.3.1, `-E object_jitter=0 -E control_hz=20`):

| Run | `task_score` |
|---|---|
| Scripted demo | 1.0 |
| Dream-1 | 0.0 |
