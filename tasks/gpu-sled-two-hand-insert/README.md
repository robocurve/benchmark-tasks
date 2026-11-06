# Push a wide GPU sled home with both arms without racking it

Instruction given to the policy: `Push the grey GPU sled all the way into its bay until it meets the black backplane. Use both arms at once, one on each side of its front face, and keep it square: it must not turn more than 2 degrees while it is between the steel guides.`

Files: `task.json`, `gpu_sled_two_hand_insert.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/gpu-sled-two-hand-insert/gpu_sled_two_hand_insert.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 12 cases pass
left: 17.8 s plan, worst IK 0.098 mm / 0.013 deg
right: 17.8 s plan, worst IK 0.098 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/gpu-sled-two-hand-insert/gpu_sled_two_hand_insert.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Push the grey GPU sled all the way into its bay until it meets the black backplane. Use both arms at once, one on each side of its front face, and keep it square: it must not turn more than 2 degrees while it is between the steel guides." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/gpu-sled-two-hand-insert-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/gpu-sled-two-hand-insert/gpu_sled_two_hand_insert.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Push the grey GPU sled all the way into its bay until it meets the black backplane. Use both arms at once, one on each side of its front face, and keep it square: it must not turn more than 2 degrees while it is between the steel guides." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/gpu-sled-two-hand-insert-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 18 s of plan (about 556 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/gpu-sled-two-hand-insert-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/gpu-sled-two-hand-insert-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (1 minute at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Slide the sled back out to its start mark, square, with its far 3.5 cm between the guides; the guides and backplane do not move. On a real rig racking is judged from the trial video.
