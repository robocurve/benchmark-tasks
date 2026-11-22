# Fix a child's wrong answer on a block sum

A printed card reads "2 + 3 =" and a child has laid six blocks on the answer mat. The sum is wrong. Fixing it means taking exactly one block off the mat and putting it in the spare tray: checking and correcting the child's work, not redoing it.

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
  --instruction "the child's sum is wrong: fix the blocks on the answer mat, putting any spare block in the spare tray" \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/fix-childs-sum/dream

# Scripted demo (reaches 1.0 in about 191 steps)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "the child's sum is wrong: fix the blocks on the answer mat, putting any spare block in the spare tray" \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/fix-childs-sum/demo

# Videos (one MP4 per camera, next to each run's frames)
for log in ~/yam-eval/fix-childs-sum/*/*.json; do inspect-robots video "$log"; done
```

The demo is open loop, so it needs `-E object_jitter=0`. The right arm takes the nearest block off the mat and sets it in the spare tray.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s (the episode then ends), and otherwise `score()` at the end of the episode. An object counts as still when it moves under 1 cm/s and 0.5 rad/s, or has not moved at all (under 0.5 mm) since the previous control step: PhysX can put a settled body to sleep while it still reports a small velocity.

| Outcome (`outcome()`) | Condition | Score |
|---|---|---|
| `fixed` | Five blocks on the mat, the sixth in the spare tray | 1.0 |
| `fixed_untidy` | Five blocks on the mat, the sixth elsewhere | 0.5 |
| `N_on_mat` | Any other number on the mat, including the untouched six | 0.0 |

A block is on the mat when it rests still with its centre over the mat and its underside within 1 cm of the table, and in the spare tray when it rests still with its centre inside the tray's walls. In the real world, count them.

## Runtime, seeds and reset

- Run limit: 1,200 steps (1 minute at 20 Hz).
- Runtime per trial on an L40S, including Isaac Sim's start-up (about 25 s once its shaders are cached; the first
  start on a new instance takes several minutes more): about 48 s for the scripted demo and
  193 s for the Dream-1 run.
- Seeds: the layout is fixed (`-E object_jitter=0`), so every trial starts identically; no seed is needed.
- Reset: Put all six blocks back on the answer mat in their two rows of three (see `setup`). The card, mat and tray are fixed.

## Results in simulation

One trial each, with the commands above (Isaac Lab-Arena 0.3.1, `-E object_jitter=0 -E control_hz=20`):

| Run | `task_score` |
|---|---|
| Scripted demo | 1.0 |
| Dream-1 | 0.0 |
