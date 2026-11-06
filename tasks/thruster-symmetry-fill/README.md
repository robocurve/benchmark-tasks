# Complete a thruster layout symmetric about both centre lines

Instruction given to the policy: `Fit thrusters into empty nests so the layout becomes symmetric about both black centre lines, using as few thrusters as possible.`

Files in this folder: `task.json`, `thruster_symmetry_fill.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: 72cf7c6f02a9 (`sha256sum thruster_symmetry_fill.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/thruster-symmetry-fill/thruster_symmetry_fill.py
```

It checks that the five forced nests are exactly the closure of the three fitted thrusters under both reflections, less the fitted three, and that no larger symmetric layout fits six loose thrusters (the next needs nine), then asserts the scoring rules on hand-built states (start 0, done 1.0, any five of the six thrusters accepted, the corner pair taken as balanced with only the inner set completed 0.6, mirroring about either line alone 0.6, each thruster mirrored once about each line with the inner one's double mirror missed 0.8, all six used 0.8, extras netted against forced nests down to 0, a thruster 4 mm off centre, tilted 8 degrees or upside down counted, one leaning 35 degrees on a wall, lying across a nest, standing on the walls between nests, on a fitted thruster, held 30 mm up, still moving, 3.5 mm high, on the table or off it not counted) and replays the left arm's plan through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/thruster-symmetry-fill/thruster_symmetry_fill.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit thrusters into empty nests so the layout becomes symmetric about both black centre lines, using as few thrusters as possible." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/thruster-sym-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is max(0, R - X) / 5 as defined in `task.json` (R forced nests filled, X thrusters seated in any other nest); otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/thruster-symmetry-fill/thruster_symmetry_fill.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit thrusters into empty nests so the layout becomes symmetric about both black centre lines, using as few thrusters as possible." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/thruster-sym-demo
```

The demo is `demo()` in the task file, open loop and deterministic: the left arm fits five thrusters into the five forced nests, the far corner first, and leaves the sixth on its mark. Each pick is top-down at yaw 0, the fingers closing along y so the thruster's row neighbours sit beside the pads; each place is at yaw 45 degrees, so the open fingers lie along the nest diagonal, clear of the fitted thrusters in the neighbouring nests. Each thruster is carried at 14.5 cm, paused with its bottom 12 mm above the nest walls, lowered slowly and released 5 mm above the plate inside the pocket. Every grip closes to 8 mm narrower than the thruster rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the part (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before each close and release and makes the last descent slowly, because the simulated wrist trails a moving target by several millimetres. It reached `task_score: 1.0` in 1595 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 16 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 10 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_4a700d7b.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_bf7e6305.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 1595 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the 5 mm nest walls are legible; both scored runs and their videos are at 320 x 176, where each 15 mm centre line is about 3 px wide and is the only near-black feature on the plate.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Stand the six loose thrusters back on their marks; the plate, the nests, the lines and the three fitted thrusters do not move.
