# Open a shelf-ready can tray by tearing its front panel off along the perforation

Instruction given to the policy: `Open the shelf-ready tray so it can go straight on the shelf.`

Files: `task.json`, `open_shelf_ready_tray_perforation.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/open-shelf-ready-tray-perforation/open_shelf_ready_tray_perforation.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 17 cases pass
demo left: 25.1 s plan, worst IK 0.078 mm / 0.013 deg
demo right: 25.1 s plan, worst IK 0.096 mm / 0.013 deg
naive left: 27.2 s plan, worst IK 0.078 mm / 0.013 deg
naive right: 27.2 s plan, worst IK 0.079 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/open-shelf-ready-tray-perforation/open_shelf_ready_tray_perforation.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the shelf-ready tray so it can go straight on the shelf." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/open-shelf-ready-tray-perforation-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/open-shelf-ready-tray-perforation/open_shelf_ready_tray_perforation.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the shelf-ready tray so it can go straight on the shelf." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/open-shelf-ready-tray-perforation-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 27 s of plan (about 744 steps
including the success hold), inside the run limit of 2,400 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/open-shelf-ready-tray-perforation-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/open-shelf-ready-tray-perforation-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 2,400 steps (120 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Replace the tray with a fresh one of the same make (the torn panel is consumed); stand the twelve cans in it 4 x 3, front row against the front wall, and set the tray on its marks with the front wall toward the robot. Put the torn panel aside.

## Assets

None from third parties: the tray, the panel rows and the cans are built from primitive boxes and cylinders in the
task file (`mechkit.py` adds the hinges and the perforation ties in memory).
