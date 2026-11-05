# Top up line-side bins to their max level

Instruction given to the policy: `Each blue line-side bin is for one part type and its max level is 3 parts. Top every line-side bin up to exactly 3 with parts of its own type from the grey supermarket bins. Do not overfill a bin, do not put a part in a bin of another type, and leave a full bin alone.`

Files: `task.json`, `line_side_min_max_replenish.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/line-side-min-max-replenish/line_side_min_max_replenish.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 12 cases pass
left: 39.6 s plan, worst IK 0.093 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/line-side-min-max-replenish/line_side_min_max_replenish.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Each blue line-side bin is for one part type and its max level is 3 parts. Top every line-side bin up to exactly 3 with parts of its own type from the grey supermarket bins. Do not overfill a bin, do not put a part in a bin of another type, and leave a full bin alone." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/line-side-min-max-replenish-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/line-side-min-max-replenish/line_side_min_max_replenish.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Each blue line-side bin is for one part type and its max level is 3 parts. Top every line-side bin up to exactly 3 with parts of its own type from the grey supermarket bins. Do not overfill a bin, do not put a part in a bin of another type, and leave a full bin alone." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/line-side-min-max-replenish-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 40 s of plan (about 992 steps
including the success hold), inside the run limit of 1,800 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/line-side-min-max-replenish-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/line-side-min-max-replenish-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,800 steps (90 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Put 1 square nut, 2 bolts and 3 bushings back in the line-side bins, front places first, and 3 spares in each supermarket bin.
