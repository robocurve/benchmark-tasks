# Rework a terminal strip to its terminal plan

Instruction given to the policy: `The terminal strip was started wrong. Take the blue block off the rail, then fit the strip to the terminal plan, starting against the fixed end clamp on the left: grey, grey, blue, green, grey. Close the strip with the loose end clamp. Leave no gap of more than 3 mm.`

Files: `task.json`, `terminal_strip_to_plan.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/terminal-strip-to-plan/terminal_strip_to_plan.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 15 cases pass
left: 123.9 s plan, worst IK 0.089 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/terminal-strip-to-plan/terminal_strip_to_plan.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The terminal strip was started wrong. Take the blue block off the rail, then fit the strip to the terminal plan, starting against the fixed end clamp on the left: grey, grey, blue, green, grey. Close the strip with the loose end clamp. Leave no gap of more than 3 mm." \
  --scorer task_score --max-steps 4800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/terminal-strip-to-plan-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/terminal-strip-to-plan/terminal_strip_to_plan.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The terminal strip was started wrong. Take the blue block off the rail, then fit the strip to the terminal plan, starting against the fixed end clamp on the left: grey, grey, blue, green, grey. Close the strip with the loose end clamp. Leave no gap of more than 3 mm." \
  --scorer task_score --max-steps 4800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/terminal-strip-to-plan-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 124 s of plan (about 2678 steps
including the success hold), inside the run limit of 4,800 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/terminal-strip-to-plan-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/terminal-strip-to-plan-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 4,800 steps (4 minutes at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Stand the blue block back against the fixed clamp and the spares and the loose end clamp on their marks.
