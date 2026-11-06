# Swap a failed optical transceiver, keeping the cage and the new optic capped

Instruction given to the policy: `The optic in the switch cage with the red link tile has failed. Put it in the black return tray. As soon as it is out, plug the empty cage with the blue dust plug from the blue tray. Then take the plug out again, put it back in the blue tray, and seat the new optic from the white tray in that cage, keeping its black dust cap on until it is seated. Finally lift the cap off and put it in the blue tray. Do not touch the working optics.`

Files: `task.json`, `transceiver_dust_caps.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/transceiver-dust-caps/transceiver_dust_caps.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 17 cases pass
left: 89.5 s plan, worst IK 0.097 mm / 0.012 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/transceiver-dust-caps/transceiver_dust_caps.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The optic in the switch cage with the red link tile has failed. Put it in the black return tray. As soon as it is out, plug the empty cage with the blue dust plug from the blue tray. Then take the plug out again, put it back in the blue tray, and seat the new optic from the white tray in that cage, keeping its black dust cap on until it is seated. Finally lift the cap off and put it in the blue tray. Do not touch the working optics." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/transceiver-dust-caps-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/transceiver-dust-caps/transceiver_dust_caps.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The optic in the switch cage with the red link tile has failed. Put it in the black return tray. As soon as it is out, plug the empty cage with the blue dust plug from the blue tray. Then take the plug out again, put it back in the blue tray, and seat the new optic from the white tray in that cage, keeping its black dust cap on until it is seated. Finally lift the cap off and put it in the blue tray. Do not touch the working optics." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/transceiver-dust-caps-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 90 s of plan (about 1990 steps
including the success hold), inside the run limit of 2,400 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/transceiver-dust-caps-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/transceiver-dust-caps-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 2,400 steps (2 minutes at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Stand the failed optic back in cage 2, the new optic in the white tray with its cap on top, and the plug in the blue tray at its mark; the working optics stay in their cages. On a real rig the order is judged from the trial video.
