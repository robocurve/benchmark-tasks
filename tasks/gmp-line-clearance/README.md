# Clear a packaging line of the last batch before setting out the next

Instruction given to the policy: `Changeover on this packaging line. First clear the steel line bed: every item left on it from the last batch, including anything hidden behind something else, goes into the blue reconciliation bin. Only when the line bed is completely clear, take the new batch's blue label reel from the staging mat and stand it on the white set-out tile. Never bring the new reel onto the line before it is clear.`

Files: `task.json`, `gmp_line_clearance.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/gmp-line-clearance/gmp_line_clearance.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 14 cases pass
left: 70.5 s plan, worst IK 0.088 mm / 0.013 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/gmp-line-clearance/gmp_line_clearance.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Changeover on this packaging line. First clear the steel line bed: every item left on it from the last batch, including anything hidden behind something else, goes into the blue reconciliation bin. Only when the line bed is completely clear, take the new batch's blue label reel from the staging mat and stand it on the white set-out tile. Never bring the new reel onto the line before it is clear." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/gmp-line-clearance-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/gmp-line-clearance/gmp_line_clearance.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Changeover on this packaging line. First clear the steel line bed: every item left on it from the last batch, including anything hidden behind something else, goes into the blue reconciliation bin. Only when the line bed is completely clear, take the new batch's blue label reel from the staging mat and stand it on the white set-out tile. Never bring the new reel onto the line before it is clear." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/gmp-line-clearance-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 70 s of plan (about 1610 steps
including the success hold), inside the run limit of 1,800 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/gmp-line-clearance-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/gmp-line-clearance-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,800 steps (90 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Put the carton, both vials (one tight behind the carton) and the leaflets back on the line bed on their marks and the reel on the staging mat; the bed, tile, mat and bin do not move. On a real rig the clear-before-set-out rule is judged from the trial video.
