# Continue a harness standoff run at the same pitch

Instruction given to the policy: `Stand the two loose standoffs on the black line so the run continues with the same spacing as the three glued standoffs.`

Files in this folder: `task.json`, `harness_standoff_pitch.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: 265329cf3f45 (`sha256sum harness_standoff_pitch.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/harness-standoff-pitch/harness_standoff_pitch.py
```

It asserts the scoring rules on hand-built states (start 0; done 1.0, also with the standoffs swapped, upside down, tilted 8 degrees, exactly 3 mm up, exactly 6 mm off the line or exactly 8 mm off a station; S4 alone at full 0.5 and at half 0.25; S5 alone 0, the run having skipped a station, and so is one alone at a 130 mm pitch or midway along the empty line; one full and one half 0.75, as left by a 60 or a 70 mm pitch; a 50, 55, 75 or 80 mm pitch 0.25; a 40, 45, 85 or 130 mm pitch 0; 8.5 and exactly 16 mm off half, 16.5 mm off nothing; set touching the glued run, spread evenly to the board end in two or three gaps (also with the end one fallen off), started one pitch late at S5, or halfway between the stations 0; a standoff at S4 with the other crowded in before it 0, with the other just past it 0.5, the standoffs being taken in order along the run; a standoff between two glued ones, lying down, tilted 12 degrees, 7 mm off the line, held 10 mm up, still moving, stacked on the other, on the table beside the board or on the board off the line counts nowhere, and one lying down at S4 leaves the other alone at S5, 0) and replays the left arm's plan through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/harness-standoff-pitch/harness_standoff_pitch.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stand the two loose standoffs on the black line so the run continues with the same spacing as the three glued standoffs." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/standoff-pitch-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is the sum over the two stations of 0.5 at full or 0.25 at half as defined in `task.json`; otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/harness-standoff-pitch/harness_standoff_pitch.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stand the two loose standoffs on the black line so the run continues with the same spacing as the three glued standoffs." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/standoff-pitch-demo
```

The demo is `demo()` in the task file, open loop and deterministic: the left arm stands the outboard loose standoff on the line 65 mm past the third glued standoff and the inboard one 130 mm past it, two top-down pick and place moves. Each pick is at yaw 0, the fingers closing along y with the pad tips 12 mm above the table, so the other loose standoff, 80 mm away, keeps its near side 70 mm from the grasp centre; each place is at yaw -90 degrees, the fingers closing along x across the line, so the neighbour 65 mm along the line sits beside the pads rather than in their path (+90 degrees drives the wrist joint to its limit at these stations). Each standoff is carried at 14.5 cm, paused 25 mm above the board, lowered slowly and released 5 mm above the board. Every grip closes to 8 mm narrower than the standoff rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the part (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before each close and release and makes the last descent slowly, because the simulated wrist trails a moving target by several millimetres. It reached `task_score: 1.0` in 624 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 6 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 3 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_47befea4.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_6ce756f2.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 624 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the 10 mm line, the red start mark and the 65 mm pitch (about 26 px there) are legible; both scored runs and their videos are at 320 x 176, where the pitch is about 13 px along the image's horizontal, about 5 mm per pixel, and all five standoffs show in the top camera's first frame. A full score at S5 needs the pitch within 4 mm, so it takes a wrist-camera look at the glued run (about 1 mm per pixel from travel height) or sub-pixel estimation; the demand is deliberate.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Stand the two loose standoffs back on their marks on the table; the board, the start mark and the glued standoffs do not move.
