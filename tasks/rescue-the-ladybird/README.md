# Put the crumbs in the bin and move the toy ladybird among them to the garden pot

On the station table: a grey bin at the right of the far side and a terracotta plant pot at the left with a card beyond it reading "GARDEN". In front of the right arm are two bread crusts; in the middle, among them, sits a toy ladybird (red, 5 cm across). The instruction is to put the crumbs in the bin and the toy ladybird in the garden pot. Binning the ladybird with the crumbs is the failure. The bin, pot and card are fixed; the crusts and the ladybird are loose.

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
  --instruction "put the crumbs in the bin and move the toy ladybird to the garden pot" \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/rescue-the-ladybird/dream

# Scripted demo (reaches 1.0 in about 585 steps)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "put the crumbs in the bin and move the toy ladybird to the garden pot" \
  --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/rescue-the-ladybird/demo

# Videos (one MP4 per camera, next to each run's frames)
for log in ~/yam-eval/rescue-the-ladybird/*/*.json; do inspect-robots video "$log"; done
```

The demo is open loop, so it needs `-E object_jitter=0`. The left arm carries the ladybird to the garden pot; then the right arm puts both crusts in the bin, 8 cm apart (it waits first: the ladybird sits only 12 cm from the nearer crust, too close for both arms at once).

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s (the episode then ends), and otherwise `score()` at the end of the episode. An object counts as still when it moves under 1 cm/s and 0.5 rad/s, or has not moved at all (under 0.5 mm) since the previous control step: PhysX can put a settled body to sleep while it still reports a small velocity.

| Outcome (`outcome()`) | Condition | Score |
|---|---|---|
| `cleared` | Both crusts in the bin, the ladybird in the garden pot | 1.0 |
| `C_crusts[_ladybird_saved]` | C crusts in the bin; the ladybird in the pot or not | (C + 1 if saved) / 3 |
| `ladybird_binned` | The ladybird in the bin or off the table, whatever else | 0.0 |

An item is in the bin or the pot when its centre is inside the walls and its underside within 6 cm of the table (crusts and a saved ladybird must also be at rest). In the real world, check these by eye or with a tape measure.

## Runtime, seeds and reset

- Run limit: 1,200 steps (1 minute at 20 Hz).
- Runtime per trial on an L40S, including Isaac Sim's start-up (about 25 s once its shaders are cached; the first
  start on a new instance takes several minutes more): about 141 s for the scripted demo and
  272 s for the Dream-1 run.
- Seeds: the layout is fixed (`-E object_jitter=0`), so every trial starts identically; no seed is needed.
- Reset: Put the two crusts and the ladybird back on their starting spots. The bin, the pot and the card are fixed (putty or tape) and stay in place.

## Results in simulation

One trial each, with the commands above (Isaac Lab-Arena 0.3.1, `-E object_jitter=0 -E control_hz=20`):

| Run | `task_score` |
|---|---|
| Scripted demo | 1.0 |
| Dream-1 | 0.0 |
