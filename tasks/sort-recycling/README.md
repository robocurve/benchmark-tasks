# Sort rubbish into compost, landfill and recycling by a stated rule

Four pieces of rubbish lie in front of three bins: green compost, grey landfill and blue recycling. The instruction gives the rule: cans in the blue bin, food scraps in the green bin, everything else in the grey bin. The nearest bin is the wrong one for three of the four items, so the robot has to follow the rule rather than proximity. Rubbish in the wrong bin contaminates a whole load.

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
  --instruction "sort the rubbish: cans in the blue bin, food scraps in the green bin, everything else in the grey bin" \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/sort-recycling/dream

# Scripted demo (reaches 1.0 in about 424 steps)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "sort the rubbish: cans in the blue bin, food scraps in the green bin, everything else in the grey bin" \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/sort-recycling/demo

# Videos (one MP4 per camera, next to each run's frames)
for log in ~/yam-eval/sort-recycling/*/*.json; do inspect-robots video "$log"; done
```

The demo is open loop, so it needs `-E object_jitter=0`. The left arm puts the banana in the green bin and then the mug in the grey one; at the same time the right arm puts the tuna can and then the soup can in the blue bin. Each item is let go 1 cm above where it rests, so it doesn't land tilted.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s (the episode then ends), and otherwise `score()` at the end of the episode. An object counts as still when it moves under 1 cm/s and 0.5 rad/s, or has not moved at all (under 0.5 mm) since the previous control step: PhysX can put a settled body to sleep while it still reports a small velocity.

| Outcome (`outcome()`) | Condition | Score |
|---|---|---|
| `sorted` | All four items in their right bins | 1.0 |
| `R_right_W_wrong` | R items in their right bins, W in a wrong bin | 0.25 R - 0.125 W, at least 0 |

The right bins: the mug in the grey bin, the banana in the green bin, both cans in the blue bin. An item is in a bin when it rests still with its centre inside that bin's walls and its underside within 4 cm of the table; an item outside every bin counts for nothing. In the real world, check these by eye.

## Runtime, seeds and reset

- Run limit: 1,200 steps (1 minute at 20 Hz).
- Runtime per trial on an L40S, including Isaac Sim's start-up (about 25 s once its shaders are cached; the first
  start on a new instance takes several minutes more): about 84 s for the scripted demo and
  208 s for the Dream-1 run.
- Seeds: the layout is fixed (`-E object_jitter=0`), so every trial starts identically; no seed is needed.
- Reset: Put the four items back on their starting spots (see `setup`). The bins are fixed.

## Results in simulation

One trial each, with the commands above (Isaac Lab-Arena 0.3.1, `-E object_jitter=0 -E control_hz=20`):

| Run | `task_score` |
|---|---|
| Scripted demo | 1.0 |
| Dream-1 | 0.0 |
