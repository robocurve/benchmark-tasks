# Extend the rack's anti-tip foot before drawing a server out to service

Instruction given to the policy: `The grey server needs to come out of the rack to its service position. First pull the yellow stabiliser foot out of the bottom of the rack, about 8 cm, along the floor. Only then draw the server straight out on its steel rails, about 10 cm, toward you. Slide both along the table without lifting them, and do not move the server before the foot is out.`

Files: `task.json`, `rack_anti_tip_before_extend.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/rack-anti-tip-before-extend/rack_anti_tip_before_extend.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 13 cases pass
left: 32.0 s plan, worst IK 0.087 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/rack-anti-tip-before-extend/rack_anti_tip_before_extend.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The grey server needs to come out of the rack to its service position. First pull the yellow stabiliser foot out of the bottom of the rack, about 8 cm, along the floor. Only then draw the server straight out on its steel rails, about 10 cm, toward you. Slide both along the table without lifting them, and do not move the server before the foot is out." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/rack-anti-tip-before-extend-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/rack-anti-tip-before-extend/rack_anti_tip_before_extend.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The grey server needs to come out of the rack to its service position. First pull the yellow stabiliser foot out of the bottom of the rack, about 8 cm, along the floor. Only then draw the server straight out on its steel rails, about 10 cm, toward you. Slide both along the table without lifting them, and do not move the server before the foot is out." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/rack-anti-tip-before-extend-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 32 s of plan (about 840 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/rack-anti-tip-before-extend-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/rack-anti-tip-before-extend-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (1 minute at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Push the server back into the rack and the foot back into its guide, both to their marks; the uprights, rails and guide do not move. On a real rig the order is judged from the trial video.
