# Share oranges fairly when one child already has some

Four oranges sit on the table between two trays. Leo's blue tray already holds two oranges; Maya's red tray is empty. The robot is asked to share the oranges fairly. A fair share counts what Leo already has: three to Maya and one to Leo, so each ends with three. Splitting the four evenly (two each) leaves Maya with two and Leo with four. The oranges are the station's own orange model, the object Dream-1 handles best.

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
  --instruction "share the oranges fairly between Maya's red tray and Leo's blue tray" \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/share-fruit-fairly/dream

# Scripted demo (reaches 1.0 in about 776 steps)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "share the oranges fairly between Maya's red tray and Leo's blue tray" \
  --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval/share-fruit-fairly/demo

# Videos (one MP4 per camera, next to each run's frames)
for log in ~/yam-eval/share-fruit-fairly/*/*.json; do inspect-robots video "$log"; done
```

The demo is open loop, so it needs `-E object_jitter=0`. The right arm moves one orange into Leo's blue tray. The left arm waits above Maya's tray until the right arm is done (the two grippers would meet in the middle of the table), then moves the other three into Maya's red tray, one at a time, letting each go just above the walls.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s (the episode then ends), and otherwise `score()` at the end of the episode. An object counts as still when it moves under 1 cm/s and 0.5 rad/s, or has not moved at all (under 0.5 mm) since the previous control step: PhysX can put a settled body to sleep while it still reports a small velocity.

| Outcome (`outcome()`) | Condition | Score |
|---|---|---|
| `fair` | Three oranges in each tray | 1.0 |
| `one_off` | One tray has exactly one more orange than the other | 0.5 |
| `equal_short` | Both trays have the same number, fewer than three each | 0.25 |
| `unfair` | The trays differ by two or more (as at the start, or after a two-each split) | 0.0 |

An orange counts for a tray when it rests still with its centre inside the tray's walls and its underside within 3.5 cm of the table, so one resting on another still counts. In the real world, count them.

## Runtime, seeds and reset

- Run limit: 2,400 steps (2 minutes at 20 Hz).
- Runtime per trial on an L40S, including Isaac Sim's start-up (about 25 s once its shaders are cached; the first
  start on a new instance takes several minutes more): about 132 s for the scripted demo and
  346 s for the Dream-1 run.
- Seeds: the layout is fixed (`-E object_jitter=0`), so every trial starts identically; no seed is needed.
- Reset: Put four oranges back on the table between the trays (see `setup`) and two in Leo's tray. The trays are fixed.

## Results in simulation

One trial each, with the commands above (Isaac Lab-Arena 0.3.1, `-E object_jitter=0 -E control_hz=20`):

| Run | `task_score` |
|---|---|
| Scripted demo | 1.0 |
| Dream-1 | 0.0 |
