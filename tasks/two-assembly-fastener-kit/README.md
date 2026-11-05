# Kit fasteners for two bracket assemblies

Instruction given to the policy: `Kit two bracket assemblies, one in each half of the white kit tray. Each assembly needs two bolts, a nut on every bolt and a washer under every bolt head and every nut. Put each part type in its own pocket and leave every other part in the bins.`

Files: `task.json`, `two_assembly_kit.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/two-assembly-fastener-kit/two_assembly_kit.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 15 cases pass
left: 105.6 s plan, worst IK 0.095 mm / 0.013 deg
right: 105.6 s plan, worst IK 0.095 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/two-assembly-fastener-kit/two_assembly_kit.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Kit two bracket assemblies, one in each half of the white kit tray. Each assembly needs two bolts, a nut on every bolt and a washer under every bolt head and every nut. Put each part type in its own pocket and leave every other part in the bins." \
  --scorer task_score --max-steps 4800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/two-assembly-fastener-kit-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/two-assembly-fastener-kit/two_assembly_kit.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Kit two bracket assemblies, one in each half of the white kit tray. Each assembly needs two bolts, a nut on every bolt and a washer under every bolt head and every nut. Put each part type in its own pocket and leave every other part in the bins." \
  --scorer task_score --max-steps 4800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/two-assembly-fastener-kit-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 106 s of plan (about 2312 steps
including the success hold), inside the run limit of 4,800 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/two-assembly-fastener-kit-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/two-assembly-fastener-kit-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 4,800 steps (4 minutes at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Stand every part back on its mark in its bin; the bins and tray do not move.
