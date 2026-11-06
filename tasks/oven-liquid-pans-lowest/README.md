# Load three pans into an oven with the liquid pan on the lowest runners

Instruction given to the policy: `Load the three pans from the counter into the oven, one pan on each runner level. The brown pan is stock, a liquid: it goes on the lowest runners, so it cannot spill or boil over into food below, and it must be kept level all the way. The roast and the vegetables go on the two levels above it. Slide each pan in from the front, resting on its two runners.`

Files: `task.json`, `oven_liquid_pans_lowest.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/oven-liquid-pans-lowest/oven_liquid_pans_lowest.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 14 cases pass
left: 68.4 s plan, worst IK 0.094 mm / 0.012 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/oven-liquid-pans-lowest/oven_liquid_pans_lowest.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Load the three pans from the counter into the oven, one pan on each runner level. The brown pan is stock, a liquid: it goes on the lowest runners, so it cannot spill or boil over into food below, and it must be kept level all the way. The roast and the vegetables go on the two levels above it. Slide each pan in from the front, resting on its two runners." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/oven-liquid-pans-lowest-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/oven-liquid-pans-lowest/oven_liquid_pans_lowest.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Load the three pans from the counter into the oven, one pan on each runner level. The brown pan is stock, a liquid: it goes on the lowest runners, so it cannot spill or boil over into food below, and it must be kept level all the way. The roast and the vegetables go on the two levels above it. Slide each pan in from the front, resting on its two runners." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/oven-liquid-pans-lowest-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 68 s of plan (about 1568 steps
including the success hold), inside the run limit of 1,800 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/oven-liquid-pans-lowest-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/oven-liquid-pans-lowest-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,800 steps (90 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Stand the three pans back on the counter at their marks (roast left, stock in the middle, vegetables right); the oven, runners and counter are fixed. On a real rig, use a pan with water for the stock and judge tipping from the trial video.
