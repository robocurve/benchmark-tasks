# Move a failed server's drives into the same bay numbers of a turned replacement chassis

Instruction given to the policy: `The nearer chassis has failed. Move all four drives into the replacement chassis further out, each into the bay with the same number it had in the old chassis, so the RAID array assembles. In each chassis the bays are numbered 1 to 4 starting from the end with the orange end wall. Stand each drive upright between its bay's guides.`

Files: `task.json`, `drive_migration_same_bays.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/drive-migration-same-bays/drive_migration_same_bays.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 17 cases pass
left: 82.8 s plan, worst IK 0.086 mm / 0.016 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/drive-migration-same-bays/drive_migration_same_bays.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The nearer chassis has failed. Move all four drives into the replacement chassis further out, each into the bay with the same number it had in the old chassis, so the RAID array assembles. In each chassis the bays are numbered 1 to 4 starting from the end with the orange end wall. Stand each drive upright between its bay's guides." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/drive-migration-same-bays-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/drive-migration-same-bays/drive_migration_same_bays.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The nearer chassis has failed. Move all four drives into the replacement chassis further out, each into the bay with the same number it had in the old chassis, so the RAID array assembles. In each chassis the bays are numbered 1 to 4 starting from the end with the orange end wall. Stand each drive upright between its bay's guides." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/drive-migration-same-bays-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 83 s of plan (about 1856 steps
including the success hold), inside the run limit of 2,400 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/drive-migration-same-bays-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/drive-migration-same-bays-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 2,400 steps (2 minutes at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Stand the four drives back in the old chassis, red in bay 1 to green in bay 4; both chassis are fixed.
