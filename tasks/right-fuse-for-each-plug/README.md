# Fit a 3 A fuse in the lamp's plug and a 13 A fuse in the kettle's

On the station table: two open plugs at the far side, the lamp's on the left and the kettle's on the right, each with a card beyond it naming the appliance and its power ("LAMP 100 W", "KETTLE 3000 W") and an empty fuse slot. In front of the arms lie three fuses in carriers, each with a card naming its rating: 13 A (far left, nearest the lamp's plug), 3 A (just right of centre) and 13 A (right). A UK plug takes a 3 A fuse for appliances under about 700 W and 13 A above: a 13 A fuse in the lamp's plug would let its thin cord overheat before the fuse blew, and a 3 A fuse in the kettle's plug blows at once. The plugs and cards are fixed; the fuses are loose.

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
  --instruction "fit the right fuse in the lamp's plug and in the kettle's plug" \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/right-fuse-for-each-plug/dream

# Scripted demo (reaches 1.0 in about 614 steps)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "fit the right fuse in the lamp's plug and in the kettle's plug" \
  --scorer task_score --max-steps 800 --store-frames --log-dir ~/yam-eval/right-fuse-for-each-plug/demo

# Videos (one MP4 per camera, next to each run's frames)
for log in ~/yam-eval/right-fuse-for-each-plug/*/*.json; do inspect-robots video "$log"; done
```

The demo is open loop, so it needs `-E object_jitter=0`. The left arm fits the 3 A fuse in the lamp's plug; then the right arm fits the right-hand 13 A fuse in the kettle's plug (it waits, parked, while the left arm crosses the middle).

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s (the episode then ends), and otherwise `score()` at the end of the episode. An object counts as still when it moves under 1 cm/s and 0.5 rad/s, or has not moved at all (under 0.5 mm) since the previous control step: PhysX can put a settled body to sleep while it still reports a small velocity.

| Outcome (`outcome()`) | Condition | Score |
|---|---|---|
| `fused` | 3 A fuse in the lamp's plug, 13 A in the kettle's | 1.0 |
| `N_right` | N plugs with the right fuse fitted | 0.5 N |
| `lamp_overfused` | A 13 A fuse in the lamp's plug, whatever else | 0.0 |

A fuse is fitted in a plug when it rests still, flat in the slot (not propped on a rail), with its centre within 1.5 cm front to back and 9 mm across of the slot's centre. In the real world, check these by eye or with a tape measure.

## Runtime, seeds and reset

- Run limit: 1,200 steps (1 minute at 20 Hz).
- Runtime per trial on an L40S, including Isaac Sim's start-up (about 25 s once its shaders are cached; the first
  start on a new instance takes several minutes more): about 316 s for the scripted demo and
  612 s for the Dream-1 run.
- Seeds: the layout is fixed (`-E object_jitter=0`), so every trial starts identically; no seed is needed.
- Reset: Lay the three fuses back on their starting spots. The plugs and cards are fixed (putty) and stay in place. Use blown or dummy fuses for real trials.

## Results in simulation

One trial each, with the commands above (Isaac Lab-Arena 0.3.1, `-E object_jitter=0 -E control_hz=20`):

| Run | `task_score` |
|---|---|
| Scripted demo | 1.0 |
| Dream-1 | 0.0 |
