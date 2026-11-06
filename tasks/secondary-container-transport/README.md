# Carry two sample bottles across a corridor only inside the secondary carrier

Instruction given to the policy: `Deliver both amber sample bottles to the receiving lab: they must end up standing in the white carrier, with the carrier set down on the green receiving mat across the corridor. Samples may cross the corridor (the grey strip between the yellow lines) only inside the carrier, so never take a bottle over the corridor by itself.`

Files: `task.json`, `secondary_container_transport.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/secondary-container-transport/secondary_container_transport.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 14 cases pass
left: 46.2 s plan, worst IK 0.090 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/secondary-container-transport/secondary_container_transport.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Deliver both amber sample bottles to the receiving lab: they must end up standing in the white carrier, with the carrier set down on the green receiving mat across the corridor. Samples may cross the corridor (the grey strip between the yellow lines) only inside the carrier, so never take a bottle over the corridor by itself." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/secondary-container-transport-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/secondary-container-transport/secondary_container_transport.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Deliver both amber sample bottles to the receiving lab: they must end up standing in the white carrier, with the carrier set down on the green receiving mat across the corridor. Samples may cross the corridor (the grey strip between the yellow lines) only inside the carrier, so never take a bottle over the corridor by itself." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/secondary-container-transport-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 46 s of plan (about 1124 steps
including the success hold), inside the run limit of 1,200 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/secondary-container-transport-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/secondary-container-transport-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,200 steps (60 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Stand the bottles back at their marks on the sending bench and the empty carrier at its mark. The corridor and mat do not move. On a real rig the route rule is judged from the trial video (a bottle over the corridor outside the carrier).
