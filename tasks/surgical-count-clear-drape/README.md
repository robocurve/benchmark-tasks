# Return every instrument from the surgical drape to the tray so the count is complete

The procedure is over. Two instruments (scissors and a spring clamp) still lie on the green drape over the patient, and two (a clamp and a marker) are on the instrument tray. Every instrument must be back on the tray before the count is signed off: an instrument left on the drape is a retained surgical item, a never event, so it scores 0 however tidy the rest is.

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
  --instruction "the procedure is finished: return all four instruments to the instrument tray so the count is complete" \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/surgical-count-clear-drape/dream

# Scripted demo (reaches 1.0 in about 378 steps)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "the procedure is finished: return all four instruments to the instrument tray so the count is complete" \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/surgical-count-clear-drape/demo

# Videos (one MP4 per camera, next to each run's frames)
for log in ~/yam-eval/surgical-count-clear-drape/*/*.json; do inspect-robots video "$log"; done
```

The demo is open loop, so it needs `-E object_jitter=0`. The right arm lifts the scissors off the drape, turns them a quarter turn so they lie left to right, and sets them on the tray's free right half; then it moves the spring clamp the same way, without turning it. The left arm holds its ready pose.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s (the episode then ends), and otherwise `score()` at the end of the episode. An object counts as still when it moves under 1 cm/s and 0.5 rad/s, or has not moved at all (under 0.5 mm) since the previous control step: PhysX can put a settled body to sleep while it still reports a small velocity.

| Outcome (`outcome()`) | Condition | Score |
|---|---|---|
| `count_correct` | All four instruments on the tray, none on the drape | 1.0 |
| `one_missing` | Drape clear, three on the tray (one dropped elsewhere) | 0.5 |
| `drape_clear_short` | Drape clear, two or fewer on the tray | 0.25 |
| `retained` | Any instrument still on the drape | 0.0 |

An instrument is on the drape when its centre is over the drape and its underside within 5 cm of it, and on the tray when it rests still with its centre inside the tray's walls and its underside within 3 cm of the table. In the real world, check these by eye and with a tape measure.

## Runtime, seeds and reset

- Run limit: 1,200 steps (1 minute at 20 Hz).
- Runtime per trial on an L40S, including Isaac Sim's start-up (about 25 s once its shaders are cached; the first
  start on a new instance takes several minutes more): about 67 s for the scripted demo and
  198 s for the Dream-1 run.
- Seeds: the layout is fixed (`-E object_jitter=0`), so every trial starts identically; no seed is needed.
- Reset: Put the scissors and the spring clamp back on the drape and the clamp and the marker back on the tray, at their starting spots (see `setup`). The tray and the drape are fixed.

## Results in simulation

One trial each, with the commands above (Isaac Lab-Arena 0.3.1, `-E object_jitter=0 -E control_hz=20`):

| Run | `task_score` |
|---|---|
| Scripted demo | 1.0 |
| Dream-1 | 0.0 |
