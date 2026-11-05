# Swap two failed fans in a running fan wall, one bay at a time

Instruction given to the policy: `Two fans in this running server's fan wall have failed: the bays with a red status tile in front of them. Replace both with the spares from the white tray and put the failed fans in the black return bin. The servers stay on, so never have more than one fan bay empty at a time: take out one failed fan, seat a spare in its bay, then do the other. Do not move the working fans.`

Files: `task.json`, `fan_wall_swap_one_at_a_time.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/fan-wall-swap-one-at-a-time/fan_wall_swap_one_at_a_time.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 17 cases pass
left: 70.8 s plan, worst IK 0.094 mm / 0.015 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/fan-wall-swap-one-at-a-time/fan_wall_swap_one_at_a_time.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Two fans in this running server's fan wall have failed: the bays with a red status tile in front of them. Replace both with the spares from the white tray and put the failed fans in the black return bin. The servers stay on, so never have more than one fan bay empty at a time: take out one failed fan, seat a spare in its bay, then do the other. Do not move the working fans." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/fan-wall-swap-one-at-a-time-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/fan-wall-swap-one-at-a-time/fan_wall_swap_one_at_a_time.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Two fans in this running server's fan wall have failed: the bays with a red status tile in front of them. Replace both with the spares from the white tray and put the failed fans in the black return bin. The servers stay on, so never have more than one fan bay empty at a time: take out one failed fan, seat a spare in its bay, then do the other. Do not move the working fans." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/fan-wall-swap-one-at-a-time-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 71 s of plan (about 1616 steps
including the success hold), inside the run limit of 1,800 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/fan-wall-swap-one-at-a-time-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/fan-wall-swap-one-at-a-time-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,800 steps (90 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Stand the two failed fans back in bays 2 and 4 and the spares in the white tray at their marks; the working fans, sill, dividers, tiles, tray and bin do not move. On a real rig the one-bay-at-a-time rule is judged from the trial video.
