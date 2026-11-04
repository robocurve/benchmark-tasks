# Stow hazardous canisters by containment level

Instruction given to the policy: `Stow each canister upright in its own spot, a ring pocket or the white shelf, with at least as many wall rings around it as its hazard level: red needs two, yellow one, green none.`

Files in this folder: `task.json`, `containment_level_stow.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: 6c283e4f4e8e (`sha256sum containment_level_stow.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/containment-level-stow/containment_level_stow.py
```

It asserts the scoring rules on hand-built states (start 0; the demo layout, and red in the other locker pocket with the yellows the other way round, 1.0; each canister to its nearest spot, the nearest free spot taken from the inboard end (red in the single bay) or from the outboard end (red on the shelf), and one spot per level with the second yellow on the shelf 0; an exact match of level to spot that leaves the second yellow on the plate 0.75, as do green in the locker with the second yellow left on the plate or on the table, red with both yellows right and green left on the plate, and the rest stowed with red left untouched on the plate or set upright on the table; one canister stowed 0.25 for red in a locker pocket, green on the shelf or in a pocket, or a yellow in the single bay, and 0 for red in the single bay or on the shelf or a yellow on the shelf; red lying on the shelf, leaning 40 degrees in the single bay or held still 3 mm above its floor 0 whatever else is stowed; red held still 12 mm over the shelf, carried across it, or held still over its own pocket at the demo's entry height no penalty; green and a yellow both on the shelf 0; with the rest stowed, red on the single bay's wall top, lying across its walls, standing on the outer ring's wall top or stacked on the yellow in the single bay, red knocked off the table or tipped over on it, and green tipped over on the plate or knocked off the table all 0, as is a yellow stacked on red in a locker pocket; green still moving, held at the demo's release height (6 mm up), 18 mm off the shelf centre or lying on the shelf, and red tilted 12 degrees in its pocket earning nothing; green 4 mm up, red on its other end, tilted 8 degrees, touching a ring wall or 1 mm into it, and green 17 mm off the shelf centre still counting). It then enumerates all 625 end layouts with each canister stowed in one spot or in none: four score 1.0, all with red in a locker pocket and green on the shelf, and 32 score 0.75, 38 score 0.5, 12 score 0.25 and 539 score 0. Last, it replays the left arm's plan through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

The four spots stand in a row across the table at x = 0.270:

| Spot | Centre (m) | Walls round it | Barriers |
|---|---|---|---|
| outboard locker pocket | (0.270, 0.360) | its own 15 mm grey ring inside the 25 mm dark outer ring | 2 |
| inboard locker pocket | (0.270, 0.275) | its own 15 mm grey ring inside the 25 mm dark outer ring | 2 |
| single bay | (0.270, 0.175) | one 15 mm grey ring | 1 |
| open shelf | (0.270, 0.090) | none | 0 |

The four canisters stand in a row on the plate at x = 0.130:

| Canister | Level | y (m) | Nearest spot | Spots that meet its level |
|---|---|---|---|---|
| red | 2 | 0.140 | single bay | either locker pocket |
| yellow | 1 | 0.210 | single bay | single bay, either locker pocket |
| yellow | 1 | 0.280 | inboard locker pocket | single bay, either locker pocket |
| green | 0 | 0.350 | outboard locker pocket | any spot |

The row runs the opposite way to the containment: red starts beside the weak spots and green beside the locker. Filling the nearest spot from either end therefore puts red in the single bay or on the shelf, and an exact match of level to spot leaves the second yellow nowhere at its own level: on the shelf it scores 0, and left on the plate it caps the trial at 0.75. Only the up-move into the locker keeps every canister at or above its level.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/containment-level-stow/containment_level_stow.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stow each canister upright in its own spot, a ring pocket or the white shelf, with at least as many wall rings around it as its hazard level: red needs two, yellow one, green none." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/containment-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is 0 if any canister is placed in a spot with fewer barriers than its level, two share a spot, or any canister ends off the table, tipped over outside a spot or perched on a wall top or another canister, and otherwise 0.25 per canister stowed upright in a spot, as defined in `task.json`; otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/containment-level-stow/containment_level_stow.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stow each canister upright in its own spot, a ring pocket or the white shelf, with at least as many wall rings around it as its hazard level: red needs two, yellow one, green none." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/containment-demo
```

The demo is `demo()` in the task file, open loop and deterministic: the left arm sets red in the inboard locker pocket and the inboard yellow in the single bay, then carries the outboard yellow up into the outboard locker pocket rather than to the single bay, and sets green on the shelf. These are four top-down pick and place moves with the fingers closing along x (wrist yaw -90 degrees), so each plate neighbour, 70 mm away along y, stays beside the pads. Green goes last, when no neighbour is left, and is picked at -60 degrees so that joint 6 keeps 41 degrees from its stop. Each canister is gripped with the TCP 38 mm above its bottom, carried at 14 cm, paused with its bottom 10 mm over the rings, lowered slowly and released 7 mm above the floor. At that height the pad tips stay 7 mm over the outer ring's 25 mm walls under the open fingers, and a canister still held reads about 6 mm up, never as stowed. Every grip closes to 8 mm narrower than the part rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the part (a fully commanded close saturates both fingers and lets the part slide onto one of them). The arm settles before each close and release, pauses above each spot and makes the last descent slowly, because the simulated wrist trails a moving target by several millimetres. It reached `task_score: 1.0` in 1272 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 13 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 6 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout, and the open-loop demo needs it. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_72ea4bdd.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_1035061a.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 1272 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360; both scored runs and their videos are at 320 x 176. At the ready pose neither arm hides any canister, pocket floor, ring wall top or the shelf top from the top camera. This was checked by projecting the arms' collision hulls through the calibrated camera; 60 mm further outboard, the left arm hid the outer ring's outboard wall. From that camera the walls read by their outlines rather than their height. A 50 mm pocket spans about 10 px at 320 x 176 and 19 px at 640 x 360, and a 6 mm wall about 1 and 2 px. The near face of a 25 mm outer-ring wall shows about 2 and 4 px tall, against 1 and 2 px for a 15 mm ring. A locker pocket shows as a grey ring inside the dark outer ring, and the single bay as a lone grey ring.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Stand the four canisters back on their marks on the plate at x = 0.130: red at y = 0.140, yellow at 0.210 and 0.280, green at 0.350. The four rings, the plate and the shelf do not move.
