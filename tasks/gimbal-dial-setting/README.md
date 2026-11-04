# Set a pointer to 120 degrees on a graduated dial

Instruction given to the policy: `Lay the blue pointer flat across the centre of the dial, lined up with the 120 degree graduation. The red tick is zero and the scale runs to 180 degrees.`

Files in this folder: `task.json`, `gimbal_dial_setting.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: 3cd26037e9ba (`sha256sum gimbal_dial_setting.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/gimbal-dial-setting/gimbal_dial_setting.py
```

It asserts the scoring rules on hand-built states (start 0; done on the 120 graduation 1.0, also upside down, reversed end for end, flat on the dial face beside the cross, on its 170 x 20 side face, rolled 4 degrees, 4.9 or exactly 5.0 degrees off, exactly 3 mm up, with the centre 11.9 or exactly 12 mm off along the bar, 4 or 9.2 mm off sideways, or 4.9 degrees off with the centre 9 mm sideways so its line still meets the 120 tick; 7 degrees off either way, or on 120 with the centre 11.9 mm sideways either way, 0.5; exactly 7.5 degrees off 0; the 90, 105, 135, 75, 60, 0 and 180 graduations 0; 4.9 degrees toward 105 with the centre 6 or 11.9 mm sideways toward 105, where its line reads 105, 0; the centre 13 mm off, one end on the 120 graduation but not across the centre, pitched 6 degrees along its length, 3.5 mm up, held still at the release height, still moving, on the table beside the dial or with one end on the table 0) and replays the left arm's plan through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/gimbal-dial-setting/gimbal_dial_setting.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lay the blue pointer flat across the centre of the dial, lined up with the 120 degree graduation. The red tick is zero and the scale runs to 180 degrees." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/dial-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is the 0 / 0.5 / 1.0 ladder defined in `task.json`; otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/gimbal-dial-setting/gimbal_dial_setting.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lay the blue pointer flat across the centre of the dial, lined up with the 120 degree graduation. The red tick is zero and the scale runs to 180 degrees." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/dial-demo
```

The demo is `demo()` in the task file, open loop and deterministic: the left arm picks the pointer at its centre with the fingers across its 24 mm width and the pad tips 5 mm above the table, lifts it with the TCP 145 mm above the table (the bar's bottom 127 mm up), carries it to the dial centre while turning the grip 30 degrees clockwise seen from above so the bar's long axis lies along the 120 degree graduation, pauses with the bar 21 mm above the dial, lowers it slowly and releases it 6 mm above the dial top, from where it drops onto the centre cross. The grip closes to 8 mm narrower than the part rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the bar and keeps it from turning (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before the close and the release, pauses above the dial and makes the last descent slowly, because the simulated wrist trails a moving target by several millimetres. The release is commanded 6 mm up, beyond the 3 mm limit for a placed bar, so the still, held bar does not read as placed before the fingers open. It reached `task_score: 1.0` in 299 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 12 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 2 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_db74b611.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_42190196.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 299 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the 8 mm graduations and the red zero tick are legible; both scored runs and their videos are at 320 x 176, where all thirteen graduations still show in the top camera's first frame. The scale rises toward the midline, so while the left arm places, the 90 to 180 graduations, the 120 target among them, stay in the top camera's view; the arm covers 0 to 60, and the carried bar briefly covers 75 as it arrives above the dial.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Lay the pointer back flat on its mark with its long axis forward; the dial and its graduations do not move.
