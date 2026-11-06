# Draw a pan from the back of a deep oven with the peel, then lift it out

Instruction given to the policy: `The baking pan at the back of the oven is too far in to reach. Take the wooden peel, slide its blade between the two steel runners under the pan, raise it to lift the pan off the runners, and draw the pan forward to the oven mouth. Set the pan back down on the runners there, take the peel out and lay it on the table. Then lift the pan out of the oven and put it on the white pass pad.`

Files: `task.json`, `peel_from_deep_oven.py` (scene, `success()`, partial-credit `score()`, the scripted `demo()` and an offline
self-test), `README.md`.

## Environment

The program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim, L40S) with yam-arena and inspect-robots as it
installs them. Before every shell: `source ~/IsaacLab-Arena/yam.env`; the Dream key is in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with yam-arena importable:

```bash
python tasks/peel-from-deep-oven/peel_from_deep_oven.py
```

It asserts the scoring rules on hand-built states and replays every arm's demo plan through the kinematics at 20 Hz
(every joint inside the URDF limits, IK error under 0.5 mm). Expected output:

```
scoring: 12 cases pass
left: 71.8 s plan, worst IK 0.099 mm / 0.012 deg
```

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/peel-from-deep-oven/peel_from_deep_oven.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The baking pan at the back of the oven is too far in to reach. Take the wooden peel, slide its blade between the two steel runners under the pan, raise it to lift the pan off the runners, and draw the pan forward to the oven mouth. Set the pan back down on the runners there, take the peel out and lay it on the table. Then lift the pan out of the oven and put it on the white pass pad." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/peel-from-deep-oven-dream
```

`task_score` is 1.0 when `success()` has held for 1 s (the episode ends with `termination_reason="success"`),
otherwise `score()` at the last step. The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/peel-from-deep-oven/peel_from_deep_oven.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The baking pan at the back of the oven is too far in to reach. Take the wooden peel, slide its blade between the two steel runners under the pan, raise it to lift the pan off the runners, and draw the pan forward to the oven mouth. Set the pan back down on the runners there, take the peel out and lay it on the table. Then lift the pan out of the oven and put it on the white pass pad." \
  --scorer task_score --max-steps 1800 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/peel-from-deep-oven-demo
```

The demo is `demo()` in the task file: open loop and deterministic, 72 s of plan (about 1636 steps
including the success hold), inside the run limit of 1,800 steps.

## Videos and screenshot

```bash
for log in ~/yam-eval/peel-from-deep-oven-*/*.json; do inspect-robots video "$log"; done
ffmpeg -y -i ~/yam-eval/peel-from-deep-oven-demo/frames/<run>/scene-0-e0_top_cam.mp4 -frames:v 1 setup.png
```

## Run limit, runtime and seeds

- Run limit: 1,800 steps (90 seconds at 20 Hz).
- One fixed layout: `-E object_jitter=0`, so the seed does not change the start. Dream-1 is hosted and not seeded;
  the demo is deterministic.
- Runtime per trial on an L40S: roughly 1 minute of wall clock per 400 steps for Dream-1, plus Isaac Sim warm-up on
  the first run.

## Reset between trials

Stand the pan back on the runners at the back of the deck and lay the peel on its mark; the deck, walls, runners and pass pad do not move.
