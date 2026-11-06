# Balance a payload deck with two ballast blocks

Instruction given to the policy: `Place both grey ballast blocks flat on the white deck so that the combined centre of mass of the gold instrument and the two blocks lies over the deck's centre mark, within 5 mm horizontally. The instrument is 300 g, the large block 120 g and the small block 60 g.`

Files in this folder: `task.json`, `payload_deck_balance.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: d36d81276231 (`sha256sum payload_deck_balance.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/payload-deck-balance/payload_deck_balance.py
```

It asserts the scoring rules on hand-built states (start 0, the demo's placement 1.0, both blocks 100 mm opposite the instrument 1.0 and 120 mm opposite 0.5; both set opposite the instrument at its own distance, both beside the mark, the bar opposite with the cube on the mark, the bar alone at the demo's spot and the cube alone all 0; a deck with one block capped at 0.25, even when that block alone balances the instrument; the cube stacked on the bar, on the instrument, on the table, held 10 mm up, leaning 8 degrees or still moving counted as missing; the bar stood on its end, yawed 30 degrees, tilted 4 degrees or upside down still counted, and a bar overhanging the deck edge with its centre inside counted; errors of exactly 5, 10 and 12.5 mm passing their tier and 0.4 mm more dropping a tier) and replays the left arm's plan through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/payload-deck-balance/payload_deck_balance.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Place both grey ballast blocks flat on the white deck so that the combined centre of mass of the gold instrument and the two blocks lies over the deck's centre mark, within 5 mm horizontally. The instrument is 300 g, the large block 120 g and the small block 60 g." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/payload-deck-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is the 0 / 0.25 / 0.5 / 1.0 ladder on the combined centre of mass defined in `task.json`; otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/payload-deck-balance/payload_deck_balance.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Place both grey ballast blocks flat on the white deck so that the combined centre of mass of the gold instrument and the two blocks lies over the deck's centre mark, within 5 mm horizontally. The instrument is 300 g, the large block 120 g and the small block 60 g." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/payload-deck-demo
```

The demo is `demo()` in the task file, open loop and deterministic: the left arm takes the bar across its 30 mm width and sets it flat along x, 80 mm behind and 40 mm right of the mark; then it takes the cube square, turns it in the carry so the open fingers point nearly along x, clear of the bar, and sets it 60 mm right of the bar. The two moments, 120 g x (-80, -40) mm and 60 g x (-80, -100) mm, cancel the instrument's 300 g x (48, 36) mm exactly. Each block is carried at 140 mm and released 5 mm above the deck. Every grip closes to 8 mm narrower than the part rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the part (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before each close and release, pauses above each place and makes the last descent slowly, because the simulated wrist trails a moving target by several millimetres. The cube's place yaw is -75 degrees because -90 degrees drives the wrist joint to its limit. It reached `task_score: 1.0` in 624 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 8 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 2 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_71226fe1.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_5b8e82bd.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 624 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the centre mark, the instrument's offset and the two block sizes are legible; both scored runs and their videos are at 320 x 176.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Stand the bar back on its mark with its long axis along x and the cube on its mark with its faces along x and y; the deck, the mark and the instrument do not move.
