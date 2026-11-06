# Fit soft jaws to the vise before setting a finished part in it

Instruction given to the policy: `The blue part is finished and its faces must not be marked by the bare steel vise jaws. First fit the two brass soft jaws: stand one upright against the inner face of each grey steel jaw. Only then set the blue part upright on the vise bed, centred between the soft jaws.`

Files: `task.json`, `soft_jaws_before_clamping.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/soft-jaws-before-clamping/soft_jaws_before_clamping.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 13 cases pass
left: 52.8 s plan, worst IK 0.091 mm / 0.014 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/soft-jaws-before-clamping/soft_jaws_before_clamping.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The blue part is finished and its faces must not be marked by the bare steel vise jaws. First fit the two brass soft jaws: stand one upright against the inner face of each grey steel jaw. Only then set the blue part upright on the vise bed, centred between the soft jaws." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/soft-jaws-before-clamping-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/soft-jaws-before-clamping/soft_jaws_before_clamping.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The blue part is finished and its faces must not be marked by the bare steel vise jaws. First fit the two brass soft jaws: stand one upright against the inner face of each grey steel jaw. Only then set the blue part upright on the vise bed, centred between the soft jaws." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/soft-jaws-before-clamping-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 53 s of plan (about 1256 steps
including the success hold), inside the run limit of 1,800 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/soft-jaws-before-clamping-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/soft-jaws-before-clamping-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,800 steps (90 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Stand the two soft jaws back on their table marks and the part on the white tray. On a real rig the order is judged from the trial video.
