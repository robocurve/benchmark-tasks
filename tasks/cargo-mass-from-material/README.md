# Load cargo heaviest first by mass worked out from size and material

Instruction given to the policy: `Load the four cargo blocks into the rack in order of mass, one per slot, heaviest in slot 1 by the red bar and lightest in slot 4. Dark grey is steel (7.9 g/cm3), silver aluminium (2.7), white nylon (1.15), blue foam (0.25). All are 30 mm tall.`

Files in this folder: `task.json`, `cargo_mass_from_material.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: 954d366ef988 (`sha256sum cargo_mass_from_material.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/cargo-mass-from-material/cargo_mass_from_material.py
```

It computes each mass as the block's volume times its stated density and checks it against the grams stated here, checks that all four blocks are 30 mm tall, that the masses fall 1.5 to 1.9 times apart at each step and that footprint times density (the top view) gives the mass order, then the scoring rules on hand-built states (start 0, done 1.0; density order, size order, size order with nylon and foam swapped, the reverse order, each block into the slot straight ahead of it and that row reversed all 0; each of the three neighbouring swaps 0.5; slots 1 and 3 swapped and two neighbouring swaps 0; three right with the fourth on the row or on the table beyond the rack 0; two blocks in one slot, a block stacked on another, a block on a divider top, a block held still 10 mm above its pocket floor and a block still moving not loaded; a block lying on its side, yawed 30 degrees, upside down or tilted 3 degrees still loaded, and one tilted 6 degrees or a nylon block yawed 45 degrees, too long for its pocket and propped on a wall top at 8.75 degrees not; the pocket edge, the 5 mm bottom limit and their margins exact), checks that loading the four at random scores 1.0 in one order of 24 and 0.5 in three, and replays the left arm's plan through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

The four blocks, as they stand at the start in one row at x = 0.140 m, all 30 mm tall:

| Block | Colour | Size x, y, z (mm) | Volume (cm3) | Density (g/cm3) | Mass (g) | Start y (m) | Belongs in |
|---|---|---|---|---|---|---|---|
| steel | dark grey | 18 x 18 x 30 | 9.72 | 7.9 | 76.8 | 0.195 | slot 2 |
| foam | blue | 64 x 54 x 30 | 103.68 | 0.25 | 25.9 | 0.265 | slot 4 |
| aluminium | silver | 24 x 25 x 30 | 18.0 | 2.7 | 48.6 | 0.338 | slot 3 |
| nylon | white | 64 x 52 x 30 | 99.84 | 1.15 | 114.8 | 0.410 | slot 1 |

The densities and the one 30 mm height are stated in the instruction and the footprints are read from the scene; the masses are neither shown nor measured. With one height for all, the footprint seen from above carries the whole volume, so footprint times density gives the mass order and nothing rests on judging a height. Neither cue alone gives the order. By density the order is steel, aluminium, nylon, foam, and by size it is foam, nylon, aluminium, steel; only the product puts the nylon first, because it is both large and fairly dense, and the foam last, because it is the largest but the least dense. The deciding pair, nylon against steel, is 115 g against 77 g: a footprint ratio of 10.3 against a density ratio of 6.9. The score counts the pairs of blocks, of six, whose slot order matches their mass order:

| Loaded from slot 1 to slot 4 | Pairs in mass order | Score |
|---|---|---|
| nylon, steel, aluminium, foam (mass order) | 6 | 1.0 |
| any one neighbouring pair swapped | 5 | 0.5 |
| steel, aluminium, nylon, foam (density order) | 4 | 0 |
| nylon, aluminium, foam, steel (the row reversed) | 4 | 0 |
| nylon, foam, aluminium, steel (size order, the two big blocks swapped) | 3 | 0 |
| foam, nylon, aluminium, steel (size order) | 2 | 0 |
| steel, foam, aluminium, nylon (each block straight ahead) | 2 | 0 |
| foam, aluminium, steel, nylon (the reverse of mass order) | 0 | 0 |

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/cargo-mass-from-material/cargo_mass_from_material.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Load the four cargo blocks into the rack in order of mass, one per slot, heaviest in slot 1 by the red bar and lightest in slot 4. Dark grey is steel (7.9 g/cm3), silver aluminium (2.7), white nylon (1.15), blue foam (0.25). All are 30 mm tall." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/cargo-mass-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is 1.0, 0.5 or 0 from the pairs in mass order once all four blocks are loaded one per slot, as defined in `task.json`; otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/cargo-mass-from-material/cargo_mass_from_material.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Load the four cargo blocks into the rack in order of mass, one per slot, heaviest in slot 1 by the red bar and lightest in slot 4. Dark grey is steel (7.9 g/cm3), silver aluminium (2.7), white nylon (1.15), blue foam (0.25). All are 30 mm tall." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/cargo-mass-demo
```

The demo is `demo()` in the task file, open loop and deterministic: the left arm takes the steel from the inboard end of the row into slot 2, then the nylon from the outboard end into slot 1, the aluminium into slot 3 and the foam into slot 4, four top-down pick and place moves. Every grip closes the fingers along x, across the block's 18, 64, 24 or 64 mm side, so the neighbours standing along y on the row and in the rack sit beside the pads rather than in front of them, and every block keeps its start orientation. The wrist yaw is -90 degrees for the inboard steel and +90 degrees for the rest, so that joint 6 stays inside its limits; between the two the open wrist pauses for 1 s at yaw 0 above slot 2, so the half turn goes as two quarter turns. The pad tips grip 10 mm above the table. Each block is carried at 14.5 cm, paused with its bottom 10 mm above the 8 mm walls, lowered slowly and released 5 mm above the pocket floor, with the pad tips 7 mm over the walls. Every grip closes to 8 mm narrower than the block rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the block (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before each close and release and makes the last descent slowly, because the simulated wrist trails a moving target by several millimetres. It reached `task_score: 1.0` in 1294 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 13 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 5 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout, and the open-loop demo needs it. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_faa0ac07.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_7020120b.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 1294 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the four block sizes and the red bar are legible; both scored runs and their videos are at 320 x 176. Projecting both arms' collision hulls at the ready pose through the calibrated top camera shows that no arm hides any block, any pocket or the red bar. At 320 x 176 the top faces span about 4 x 4 px for the steel, 6 x 5 px for the aluminium, 13 x 12 px for the nylon and 13 x 12 px for the foam (twice that at 640 x 360). Their projected areas times density (104, 66, 156 and 35 px2 x g/cm3 for steel, aluminium, nylon and foam) already give the true order, so the top view alone suffices; the two small metal blocks are still the hardest to size. At the ready pose the left wrist camera looks past the row and the rack, so a policy that wants a closer look has to bring it over the row.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Stand the four blocks back on their marks in the row, each on the face it started on (30 mm tall) with its edges along x and y: steel, foam, aluminium, nylon from inboard to outboard. The rack and the red bar do not move.
