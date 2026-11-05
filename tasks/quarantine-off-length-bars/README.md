# Quarantine the bars cut short against the master sample

Instruction given to the policy: `Check the four steel bars in the black tray against the master bar on the white plate. Bars as long as the master go in the grey line tray; any bar shorter than the master goes in the red quarantine bin. Lay each bar flat, and leave the master where it is.`

Files: `task.json`, `quarantine_off_length_bars.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/quarantine-off-length-bars/quarantine_off_length_bars.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 11 cases pass
left: 62.4 s plan, worst IK 0.092 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/quarantine-off-length-bars/quarantine_off_length_bars.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Check the four steel bars in the black tray against the master bar on the white plate. Bars as long as the master go in the grey line tray; any bar shorter than the master goes in the red quarantine bin. Lay each bar flat, and leave the master where it is." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/quarantine-off-length-bars-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/quarantine-off-length-bars/quarantine_off_length_bars.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Check the four steel bars in the black tray against the master bar on the white plate. Bars as long as the master go in the grey line tray; any bar shorter than the master goes in the red quarantine bin. Lay each bar flat, and leave the master where it is." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/quarantine-off-length-bars-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 62 s of plan (about 1448 steps
including the success hold), inside the run limit of 2,400 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/quarantine-off-length-bars-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/quarantine-off-length-bars-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 2,400 steps (2 minutes at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Lay the four bars back in the incoming tray on their marks, near ends on the stop line, the short ones second and fourth.
