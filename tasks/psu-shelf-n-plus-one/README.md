# Populate a power shelf to N+1 from the rack's load

Instruction given to the policy: `This power shelf feeds a rack that draws 8.4 kW. Each PSU supplies 3 kW, and the shelf must run N+1: enough PSUs to carry the whole load, plus one redundant PSU. Two bays already hold PSUs. Install spare PSUs from the mat into empty bays so the shelf has exactly the N+1 count, no more, and leave the other spares on the mat. Do not move the installed PSUs.`

Files: `task.json`, `psu_shelf_n_plus_one.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/psu-shelf-n-plus-one/psu_shelf_n_plus_one.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 14 cases pass
left: 36.4 s plan, worst IK 0.094 mm / 0.012 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/psu-shelf-n-plus-one/psu_shelf_n_plus_one.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "This power shelf feeds a rack that draws 8.4 kW. Each PSU supplies 3 kW, and the shelf must run N+1: enough PSUs to carry the whole load, plus one redundant PSU. Two bays already hold PSUs. Install spare PSUs from the mat into empty bays so the shelf has exactly the N+1 count, no more, and leave the other spares on the mat. Do not move the installed PSUs." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/psu-shelf-n-plus-one-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/psu-shelf-n-plus-one/psu_shelf_n_plus_one.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "This power shelf feeds a rack that draws 8.4 kW. Each PSU supplies 3 kW, and the shelf must run N+1: enough PSUs to carry the whole load, plus one redundant PSU. Two bays already hold PSUs. Install spare PSUs from the mat into empty bays so the shelf has exactly the N+1 count, no more, and leave the other spares on the mat. Do not move the installed PSUs." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/psu-shelf-n-plus-one-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 36 s of plan (about 928 steps
including the success hold), inside the run limit of 1,800 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/psu-shelf-n-plus-one-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/psu-shelf-n-plus-one-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,800 steps (90 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Stand the four spares back on the mat at their marks; the PSUs in bays 2 and 5 stay.
