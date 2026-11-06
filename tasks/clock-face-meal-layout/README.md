# Lay out a meal by the clock face for a crewmate who cannot see

Instruction given to the policy: `Lay out the meal for the crewmate seated at the left end, who cannot see. By the clock face as they sit, put the blue cup at 2, the green pouch at 7 and the orange spoon case at 11 o'clock, 10 cm from the plate centre.`

Files in this folder: `task.json`, `clock_face_meal_layout.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: d66e36c7be7a (`sha256sum clock_face_meal_layout.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/clock-face-meal-layout/clock_face_meal_layout.py
```

It asserts the scoring rules on hand-built states (start 0; done 1.0; any one or two items at their hours 1/3 or 2/3; the robot's frame with 12 straight ahead, the frame of someone facing the arms, the frame of someone at the other end and the diner's clock run counterclockwise 0; the cup and the pouch swapped 1/3, for the case alone; a conventional setting for the arms' seat 0 and one for the diner 1/3; the case at 10 cm laid tangentially counted; 14 degrees off an hour, 85 mm (the pouch laid tangentially) and 125 mm out counted, 16 degrees off, 82 mm and 128 mm out not; the cup on its side, tilted 10 degrees, on the plate, still moving or held still at the 5 mm release height not counted, tilted 6 degrees, upside down or 3 mm up counted; the pouch on its edge or its end counted, leaning 15 degrees, one end up on the plate rim at 85 mm or stacked on the case not; the case upside down counted, 6 mm up on the plate rim, leaning on the rim with one end on the table or held 20 mm up not; everything knocked over or piled at one hour 0; the supply row slid 90 mm toward the plate 0 and a slide of the row aimed at the case's hour 1/3; every supply spot outside the band and at least 60 degrees off its own hour; every straight slide of the whole row, any direction up to 300 mm, at most 1/3, and 0 within 30 degrees of straight toward the diner up to 200 mm; each item pulled straight in to any radius in the band 0) and replays the left arm's plan through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/clock-face-meal-layout/clock_face_meal_layout.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lay out the meal for the crewmate seated at the left end, who cannot see. By the clock face as they sit, put the blue cup at 2, the green pouch at 7 and the orange spoon case at 11 o'clock, 10 cm from the plate centre." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/clock-meal-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is 1/3 per item resting level on the table 85 to 125 mm from the plate centre within 15 degrees of its hour on the diner's clock, as defined in `task.json`; otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/clock-face-meal-layout/clock_face_meal_layout.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lay out the meal for the crewmate seated at the left end, who cannot see. By the clock face as they sit, put the blue cup at 2, the green pouch at 7 and the orange spoon case at 11 o'clock, 10 cm from the plate centre." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/clock-meal-demo
```

The left end is the left as seen from behind the arms, looking forward, which is also the left of the top camera image; the diner stand-in sits there facing along the table, so the diner's 12 o'clock points to the right of the image and 3 o'clock toward the arms. The demo is `demo()` in the task file, open loop and deterministic: the left arm takes the cup from the far end of the supply row to 2 o'clock, the pouch to 7 o'clock turned 60 degrees so its 60 mm side lies along the radius, and the spoon case from the near end to 11 o'clock turned -60 degrees so its long axis lies along the radius, 110 mm out so its inner end stays 15 mm off the plate rim. These are three top-down pick and place moves, each pick with the fingers closing across the supply row; the pouch and the case are set down with the fingers closing along the clock circle, so the open fingers stay clear of the plate. Every grip closes to 8 mm narrower than the part rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the part (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before each close and release, pauses with the part 20 mm above the table, makes the last descent slowly and releases the part 5 mm above the table, because the simulated wrist trails a moving target by several millimetres. It reached `task_score: 1.0` in 948 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 10 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 4 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout, and the open-loop demo needs it. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_a34c8b1a.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_8cd7f4fb.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 948 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the diner stand-in at the left edge of the image and the 12 mm plate are legible; both scored runs and their videos are at 320 x 176.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Stand the cup, the pouch and the spoon case back on their marks in the supply row, long axes along x; the plate and the diner stand-in do not move.
