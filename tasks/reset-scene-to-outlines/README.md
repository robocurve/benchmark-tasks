# Reset the table to its outlines for the next episode

Four objects are scattered in front of the arms, and four outlines are taped in a row across the far side, each the colour of its object. The robot must put every object back on its outline, the cracker box turned a quarter turn to match its outline, so the next data-collection episode can start. Robots that reset their own workspace let data collection run without a person.

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
  --instruction "put everything back on its outline so the next run can start" \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/reset-scene-to-outlines/dream

# Scripted demo (reaches 1.0 in about 409 steps)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "put everything back on its outline so the next run can start" \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/reset-scene-to-outlines/demo

# Videos (one MP4 per camera, next to each run's frames)
for log in ~/yam-eval/reset-scene-to-outlines/*/*.json; do inspect-robots video "$log"; done
```

The demo is open loop, so it needs `-E object_jitter=0`. The left arm puts the bowl home, gripping it by its rim, and then the cube; at the same time the right arm puts the soup can home and then the cracker box, turning it a quarter turn on the way.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s (the episode then ends), and otherwise `score()` at the end of the episode. An object counts as still when it moves under 1 cm/s and 0.5 rad/s, or has not moved at all (under 0.5 mm) since the previous control step: PhysX can put a settled body to sleep while it still reports a small velocity.

| Outcome (`outcome()`) | Condition | Score |
|---|---|---|
| `reset` | All four objects home | 1.0 |
| `3_of_4` | Three objects home | 0.75 |
| `2_of_4` | Two objects home | 0.5 |
| `1_of_4` | One object home | 0.25 |
| `0_of_4` | None home | 0.0 |

An object is home when it rests still, upright, with its underside on the table and its centre within 2 cm of its outline's centre (upright means at least 4 cm tall for the cube, 9 cm for the can and 12 cm for the cracker box). The cracker box must also be turned to match its outline, measuring no more than 8 cm left to right. In the real world, check these with a tape measure.

## Runtime, seeds and reset

- Run limit: 2,400 steps (2 minutes at 20 Hz).
- Runtime per trial on an L40S, including Isaac Sim's start-up (about 25 s once its shaders are cached; the first
  start on a new instance takes several minutes more): about 84 s for the scripted demo and
  351 s for the Dream-1 run.
- Seeds: the layout is fixed (`-E object_jitter=0`), so every trial starts identically; no seed is needed.
- Reset: Put the four objects back on their scattered starting spots (see `setup`), the cracker box turned left to right. The outlines are fixed.

## Results in simulation

One trial each, with the commands above (Isaac Lab-Arena 0.3.1, `-E object_jitter=0 -E control_hz=20`):

| Run | `task_score` |
|---|---|
| Scripted demo | 1.0 |
| Dream-1 | 0.0 |
