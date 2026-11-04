# Quarantine a sample and its contact chain

Instruction given to the policy: `The red sample may be biologically contaminated. Put it in the yellow isolation box with every sample that touches it now or touches one that does, and put the other samples in the white rack.`

Files in this folder: `task.json`, `contact_chain_quarantine.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: f4d100601b1c (`sha256sum contact_chain_quarantine.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/contact-chain-quarantine/contact_chain_quarantine.py
```

It asserts the scoring rules on hand-built states (start 0, done 1.0 in two pocket orders; the red sample and the two nearest it isolated, only the red sample and its direct neighbour isolated, only the red sample isolated, the row split in half, the inboard three into the white rack, the tall samples isolated and the short ones curated or the reverse, all 0; the chain isolated with the clean samples left on the mat, or the clean samples curated with the chain left, 0.5; touching treated as contamination 4/6; the pair isolated with the red sample 2/6; a clean sample isolated so one of the chain is left out 4/6; a contaminated sample on top of a clean one in the white rack, lying across the white rack, standing against its outside, lying against its inboard end, lying with a flat end against its near wall or its inboard end, tipped from its spot until its top rim rests 3 mm onto the near wall, lying across the gap between the racks on both end walls, on top of two samples stacked in a white pocket, or held still high over the rack 0; one standing 3 mm clear of the white rack, lying end on 3 mm clear of it, carried high over it or standing between the two racks not gated; a sample still moving, held 5 mm above the floor, tilted 12 degrees, on a divider, lying on the isolation box, on the mat or knocked over not counted; 3.5 mm above the floor, tilted 8 degrees or 19 mm off the pocket centre counted; two samples in one pocket counted once), checks that the contact chain computed from the start layout is the red sample and the two grey samples beyond it, that the gaps are 29, 0.5, 0.5, 45 and 0.5 mm, that the clean grey sample beside the chain stands nearer the red one than the chain's far end does, and that both heights appear among the contaminated and the clean samples, and replays the left arm's plan through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/contact-chain-quarantine/contact_chain_quarantine.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The red sample may be biologically contaminated. Put it in the yellow isolation box with every sample that touches it now or touches one that does, and put the other samples in the white rack." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/contact-chain-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is 0 if a contaminated sample is in, on or against the white rack and otherwise 1/6 per sample standing in a pocket of the right kind, as defined in `task.json`; otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/contact-chain-quarantine/contact_chain_quarantine.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The red sample may be biologically contaminated. Put it in the yellow isolation box with every sample that touches it now or touches one that does, and put the other samples in the white rack." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/contact-chain-demo
```

The demo is `demo()` in the task file, open loop and deterministic. The left arm moves the clean samples first, as a curator would: the tall sample of the pair, its short partner, then the short sample beside the chain, into the white rack. Then it moves the red sample, the tall end of the chain and the short sample that linked them into the yellow box. These are six top-down pick and place moves. The finger bodies are wider than a sample, so a tall sample is always taken while its touching neighbour is short, gripped 15 mm under its top so the pads pass 12 mm over the neighbour, and a short sample is taken only once it stands alone. Wherever a neighbour stands along y, at the row or in a rack, the fingers close along x, so the neighbour sits beside the pads rather than in front of them; the wrist yaw is chosen per move so that joint 6 stays inside its limits, and the wrist turns in two 90 degree steps above the first isolation pocket. Each sample is carried at 15 cm, paused with its bottom 10 mm above the 12 mm walls, lowered slowly and released 5 mm above the pocket floor. Every grip closes to 8 mm narrower than the part rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the part (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before each close and release and makes the last descent slowly, because the simulated wrist trails a moving target by several millimetres. It reached `task_score: 1.0` in 1941 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 7 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 8 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout, and the open-loop demo needs it. Jitter would also change which samples touch, while the scorer's contaminated set is fixed by the nominal layout. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_fd828f82.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_206fd94a.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 1941 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the contacts and the gaps are legible; both scored runs and their videos are at 320 x 176. Ray-cast through the top camera's calibration at 320 x 176, each sample is about 6 to 10 px wide. In every image row where both neighbours show, the 29 mm gap leaves at least 4 px of mat between them and the 45 mm gap at least 6 px, while touching samples meet with no gap in their lower rows and show at most 3 px of mat past a short sample's top. At 640 x 360 the two gaps are at least 9 and 12 px. The left wrist camera shows the row larger.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Stand the six samples back on their marks on the mat, inboard to outboard short grey, tall red, short grey, tall grey, tall grey, short grey, and push the chain and the pair together so they touch. The mat and both racks do not move.
