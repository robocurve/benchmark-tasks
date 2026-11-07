# Swap an orbital replacement unit

Instruction given to the policy: `Pull the red unit out of the bay and stow it in the return slot, turned 90 degrees to fit, then seat the grey spare in the empty bay.`

Files in this folder: `task.json`, `oru_bay_swap.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: 5791017ece79 (`sha256sum oru_bay_swap.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/oru-bay-swap/oru_bay_swap.py
```

It replays both arms' plans through the kinematics at 20 Hz, asserts every joint stays inside the URDF limits with an IK error under 0.5 mm, and asserts the scoring ladder on hand-built states (start 0, red out 0.25, red lying 0, red stowed 0.5, spare seated 0.5 or 0.75, done 1.0, a unit in a pocket corner or upside down still seated, a unit held in the air never credited).

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/oru-bay-swap/oru_bay_swap.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pull the red unit out of the bay and stow it in the return slot, turned 90 degrees to fit, then seat the grey spare in the empty bay." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/oru-bay-swap-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, on the ladder in `task.json` (0, 0.25, 0.5, 0.75, 1.0); otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/oru-bay-swap/oru_bay_swap.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pull the red unit out of the bay and stow it in the return slot, turned 90 degrees to fit, then seat the grey spare in the empty bay." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/oru-bay-swap-demo
```

The demo is `demo()` in the task file, open loop and deterministic: the left arm lifts the red unit out, turns it and sets it in the slot, then parks; the right arm picks the spare up while the red unit comes out and seats it once the bay is empty. Every grip closes to 8 mm narrower than the part rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the part (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before each close and release, pauses above each pocket and makes the last descent slowly, because the simulated wrist trails a moving target by several millimetres. It reached `task_score: 1.0` in 391 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 8 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 1 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_7e6fff60.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_f4a3a653.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 391 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the pocket walls are legible; both scored runs and their videos are at 320 x 176.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Stand the red unit back in the bay with its 60 mm side along x and the grey unit on the pad with its 60 mm side along y. Pockets and pad do not move.
