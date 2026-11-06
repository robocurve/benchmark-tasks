# Store aliquots in a freezer box by the sample map

Instruction given to the policy: `Store the four aliquot tubes from the bench rack in the 5 x 5 freezer box, each in the cell the sample map assigns it. The map card is a half-size copy of the box's grid, in the same orientation as the box; each blue peg on it stands in the cell for one tube and is a half-size model of that tube: thin or thick, tall or short. Stand each tube upright in its cell.`

Files: `task.json`, `freezer_box_by_map.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/freezer-box-by-map/freezer_box_by_map.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 12 cases pass
left: 57.6 s plan, worst IK 0.086 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/freezer-box-by-map/freezer_box_by_map.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Store the four aliquot tubes from the bench rack in the 5 x 5 freezer box, each in the cell the sample map assigns it. The map card is a half-size copy of the box's grid, in the same orientation as the box; each blue peg on it stands in the cell for one tube and is a half-size model of that tube: thin or thick, tall or short. Stand each tube upright in its cell." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/freezer-box-by-map-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/freezer-box-by-map/freezer_box_by_map.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Store the four aliquot tubes from the bench rack in the 5 x 5 freezer box, each in the cell the sample map assigns it. The map card is a half-size copy of the box's grid, in the same orientation as the box; each blue peg on it stands in the cell for one tube and is a half-size model of that tube: thin or thick, tall or short. Stand each tube upright in its cell." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/freezer-box-by-map-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 58 s of plan (about 1352 steps
including the success hold), inside the run limit of 1,800 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/freezer-box-by-map-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/freezer-box-by-map-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,800 steps (90 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Stand the four tubes back in the bench rack in the order thick-tall, thin-tall, thick-short, thin-short from the arms outwards; the box, rack, card and pegs do not move.
