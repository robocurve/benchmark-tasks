# Tag the stuck valve from the flow indicators

Instruction given to the policy: `Two pneumatic panels each have one grey valve stuck shut and every other valve open. Dark blue lines are pipes from the blue supply. Outlet indicators show green for flow and black for no flow. Set one red tag on top of each panel's stuck valve.`

Files in this folder: `task.json`, `stuck_valve_diagnosis.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: da14004a54bd (`sha256sum stuck_valve_diagnosis.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/stuck-valve-diagnosis/stuck_valve_diagnosis.py
```

It first checks each panel's drawing and diagnosis: the strip annotations carry exactly the pipe graph; each strip as built touches, with zero gap, exactly the blocks of its graph edges and, at a junction, the strips that share it, and nothing overlaps; blocks stand at least 20 mm apart and 15 mm inside the board edge; every outlet flows with all valves open; V3 is the only valve whose closure gives the indicated outlets on each panel; and the main valve, the valve nearest a dead outlet, the valve nearest the tag and the valve farthest from the supply are wrong on both panels, as is copying one panel's answer to the same or the mirrored spot on the other. It then asserts the scoring rules on hand-built states (start 0; done 1.0, also with the tags swapped between the panels; one panel only 0.5; the left V1, V2, V4 or V5 and the right V1, V2 or V4 0 for that panel; both tags on valves of the left panel 0; a tag stacked on a tag not counted; 11.9 mm and exactly 12 mm off the valve centre counted, 13 mm not; tilted 8 degrees counted, 12 degrees not; lying on its side, upside down or yawed 45 degrees counted; held still 6 mm above the valve top not counted, exactly 4 mm counted; still moving, on the board, on a live or a dead outlet indicator, on a pipe strip, on the supply bottle or on the table not counted) and replays both arms' plans through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/stuck-valve-diagnosis/stuck_valve_diagnosis.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Two pneumatic panels each have one grey valve stuck shut and every other valve open. Dark blue lines are pipes from the blue supply. Outlet indicators show green for flow and black for no flow. Set one red tag on top of each panel's stuck valve." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/stuck-valve-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is 0.5 per panel whose stuck valve, and no other valve of that panel, carries a tag, as defined in `task.json`; otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/stuck-valve-diagnosis/stuck_valve_diagnosis.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Two pneumatic panels each have one grey valve stuck shut and every other valve open. Dark blue lines are pipes from the blue supply. Outlet indicators show green for flow and black for no flow. Set one red tag on top of each panel's stuck valve." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/stuck-valve-demo
```

The demo is `demo()` in the task file, open loop and deterministic: both arms work at once, each in front of its own base. Each arm picks its tag from the table with the fingers closing along x across the 30 mm tag and the pad tips 6 mm above the table, lifts it to a carry with the TCP 145 mm above the table, carries it over its panel to that panel's V3, pauses with the tag bottom 16 mm above the valve tops, lowers it slowly and releases it 6 mm above the valve top, from where it drops onto the valve. The grips are mirrored (yaw -90 degrees on the left, +90 on the right) to keep the wrist joint well inside its range. Every grip closes to 8 mm narrower than the part rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the tag (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before each close and release, pauses above the valve and makes the last descent slowly, because the simulated wrist trails a moving target by several millimetres. The release is commanded 6 mm up, beyond the 4 mm band for a seated tag, so a still, held tag does not read as placed before the fingers open. It reached `task_score: 1.0` in 301 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 15 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 2 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_7b61f336.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_c7e6fb0b.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 301 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the 10 mm dark blue line strips and the junctions between them are legible; both scored runs and their videos are at 320 x 176, where each 30 mm valve or indicator spans about 6 x 5 pixels and each line about 2 pixels. At the ready pose neither arm hides any panel block or either tag from the top camera; the tags start inboard of the panels for that reason.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Stand the two tags back on their marks at (0.150, 0.110) and (0.150, -0.110), square to the table edges; the panels do not move.
