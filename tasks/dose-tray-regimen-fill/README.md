# Fill a three-day dose tray from a regimen

Instruction given to the policy: `Fill the dose tray with one cup per due dose and nothing else. Days 1 to 3 run left to right; the near row is 08:00, the far row 20:00. Blue is first due day 2 08:00, then every 24 h. Red is first due day 1 20:00, then every 48 h.`

Files in this folder: `task.json`, `dose_tray_regimen_fill.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: 4461d6195ba8 (`sha256sum dose_tray_regimen_fill.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/dose-tray-regimen-fill/dose_tray_regimen_fill.py
```

It asserts that the regimen gives exactly the four due cells, then the scoring rules on hand-built states (start 0, done 1.0 with any cup of a colour, spares left anywhere off the tray ignored, one, two and three correct doses 0.25, 0.5 and 0.75, every 48 h for both colours 0.75, every 24 h for both colours 0, a must-stay-empty cell filled 0, all six cells filled 0, one colour in every due cell 0, days read right to left 0, rows or colours swapped 0, two cups in one cell 0, a cup on another cup, lying on the tray, spilled in its own cell, between cells or 22 mm off a cell centre 0, a cup 20 mm off centre, tilted 8 degrees or upside down counted, tilted 12 degrees not, a cup still moving or carried over the tray ignored, every cup knocked over 0, and done with a spare still rolling across the tray or creeping through a must-stay-empty cell 1.0 but not yet success) and replays the left arm's plan through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/dose-tray-regimen-fill/dose_tray_regimen_fill.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fill the dose tray with one cup per due dose and nothing else. Days 1 to 3 run left to right; the near row is 08:00, the far row 20:00. Blue is first due day 2 08:00, then every 24 h. Red is first due day 1 20:00, then every 48 h." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/dose-tray-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is the gated count defined in `task.json` (0 for a wrong colour, a filled must-stay-empty cell, two cups in a cell or a loose cup on the tray, otherwise correctly filled cells over four); otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/dose-tray-regimen-fill/dose_tray_regimen_fill.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fill the dose tray with one cup per due dose and nothing else. Days 1 to 3 run left to right; the near row is 08:00, the far row 20:00. Blue is first due day 2 08:00, then every 24 h. Red is first due day 1 20:00, then every 48 h." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/dose-tray-demo
```

Left to right means as seen from behind the arms, looking forward, which is also left to right in the top camera image: day 1 is the column farthest to the left. The demo is `demo()` in the task file, open loop and deterministic: the left arm sets a blue cup on day 2 08:00 and a red cup on day 1 20:00 from the left supply column, then a blue cup on day 3 08:00 and a red cup on day 3 20:00 from the right one, four top-down pick and place moves, and leaves one cup of each colour in the supply. Each pick closes the fingers across its supply column and each place closes them along the axis that has no filled cell on it (sideways, except at day 3 08:00, whose day 2 neighbour is already filled, where they close front to back), so the open fingers stay at least 20 mm from every other cup. Every grip closes to 8 mm narrower than the cup rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the cup (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before each close and release, pauses with the cup 30 mm above its pad, makes the last descent slowly and releases the cup 5 mm above the pad, because the simulated wrist trails a moving target by several millimetres. It reached `task_score: 1.0` in 1272 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 13 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 5 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_28242ae5.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_e9bf3feb.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 1272 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the 2 mm cell pads on the dark tray and the 30 mm cups are legible; both scored runs and their videos are at 320 x 176.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Stand the six cups back on their marks in the two supply columns; the tray and its pads do not move.
