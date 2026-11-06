# Find the drive the controller released by its offset, and replace it with the spare

Instruction given to the policy: `One drive in this cage has failed and the storage controller has released it: its carrier stands out from the bay, further towards you than the others. Take that drive out and put it in the black return bin, then seat the spare drive from the white tray in the same bay, pushed fully home against the backplane like its neighbours. Do not move the drives that are still seated.`

Files: `task.json`, `failed_drive_by_ejection.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/failed-drive-by-ejection/failed_drive_by_ejection.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 16 cases pass
left: 37.8 s plan, worst IK 0.092 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/failed-drive-by-ejection/failed_drive_by_ejection.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "One drive in this cage has failed and the storage controller has released it: its carrier stands out from the bay, further towards you than the others. Take that drive out and put it in the black return bin, then seat the spare drive from the white tray in the same bay, pushed fully home against the backplane like its neighbours. Do not move the drives that are still seated." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/failed-drive-by-ejection-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/failed-drive-by-ejection/failed_drive_by_ejection.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "One drive in this cage has failed and the storage controller has released it: its carrier stands out from the bay, further towards you than the others. Take that drive out and put it in the black return bin, then seat the spare drive from the white tray in the same bay, pushed fully home against the backplane like its neighbours. Do not move the drives that are still seated." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/failed-drive-by-ejection-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 38 s of plan (about 956 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/failed-drive-by-ejection-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/failed-drive-by-ejection-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Stand drives back in bays 1, 2 and 4 against the backplane, the released drive in bay 3 at 3 cm proud, and the spare in the white tray; the cage, tray and bin are fixed. On a real rig, mark the 3 cm offset with a gauge block.
