# Start the nuts on a six-stud flange in star order

Instruction given to the policy: `Start a nut on every stud of the flange, in the star (cross) bolting order: the orange stud first, then the stud directly across from it, then the next stud clockwise from the orange one (seen from above), then the one across from that, and so on until all six studs have a nut. Take the nuts from the black mat and set each one flat on the end of its stud.`

Files: `task.json`, `flange_caps_star_order.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/flange-caps-star-order/flange_caps_star_order.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 12 cases pass
left: 99.0 s plan, worst IK 0.098 mm / 0.014 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/flange-caps-star-order/flange_caps_star_order.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Start a nut on every stud of the flange, in the star (cross) bolting order: the orange stud first, then the stud directly across from it, then the next stud clockwise from the orange one (seen from above), then the one across from that, and so on until all six studs have a nut. Take the nuts from the black mat and set each one flat on the end of its stud." \
  --scorer task_score --max-steps 3600 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/flange-caps-star-order-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/flange-caps-star-order/flange_caps_star_order.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Start a nut on every stud of the flange, in the star (cross) bolting order: the orange stud first, then the stud directly across from it, then the next stud clockwise from the orange one (seen from above), then the one across from that, and so on until all six studs have a nut. Take the nuts from the black mat and set each one flat on the end of its stud." \
  --scorer task_score --max-steps 3600 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/flange-caps-star-order-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 99 s of plan (about 2180 steps
including the success hold), inside the run limit of 3,600 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/flange-caps-star-order-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/flange-caps-star-order-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 3,600 steps (3 minutes at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Stand the six nuts back on their marks on the mat. On a real rig the order is judged from the trial video.
