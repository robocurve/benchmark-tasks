# Bring the item a crewmate points at

Instruction given to the policy: `The crewmate cannot speak and is pointing at the item they need. Put that item upright on the white tray and leave every other item where it is.`

Files in this folder: `task.json`, `fetch_pointed_item.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: 8ce46c7027e0 (`sha256sum fetch_pointed_item.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/fetch-pointed-item/fetch_pointed_item.py
```

It asserts the scoring rules on hand-built states (start 0; the green item on the tray 1.0, also upside down, 20 mm off the tray centre in x and in y or tilted 8 degrees; tilted 12 degrees, lying on the tray, held still 10 mm above the floor, still moving, on the table beside the tray, on a rim top, knocked over or lifted over its mark 0; the red item (nearest the fingertip, straight in front of the left arm), the blue item (straight toward the arms from the fingertip, nearest the left arm's base, farthest from the crewmate) or the yellow item (nearest the tray) brought instead 0; all four on the tray, green and yellow together, an item stacked on the green one, held over the tray or leaning in over the rim 0 whatever else is right; the green item delivered with another item knocked over, slid 25 mm, tipped 12 degrees, lifted, standing against the rim outside, or all three knocked over 0.5; with another item slid 19 mm or upside down on its mark 1.0) and the layout claims (the red item is the one nearest the fingertip and nearer the left arm's centreline than the green one by at least 10 mm, the blue one straight toward the arms from the fingertip, nearest the left arm's base by at least 15 mm and farther than the green one from the crewmate's torso centre and its front face by at least 10 mm, the yellow one nearest the tray, only the green one on the pointing line and every other one at least 30 degrees and 54 mm off it, neighbouring items at least 100 mm apart), and replays the left arm's plan through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/fetch-pointed-item/fetch_pointed_item.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The crewmate cannot speak and is pointing at the item they need. Put that item upright on the white tray and leave every other item where it is." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/pointed-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is the 0 / 0.5 / 1.0 ladder defined in `task.json`; otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/fetch-pointed-item/fetch_pointed_item.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The crewmate cannot speak and is pointing at the item they need. Put that item upright on the white tray and leave every other item where it is." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/pointed-demo
```

The demo is `demo()` in the task file, open loop and deterministic: the left arm picks the green item, the one on the pointing line, with the grip turned 60 degrees so the pads close along that line across its 40 mm diameter, pad tips 27 mm up its side, and the fingers and the rest of the arm stay more than 45 mm from the red, blue and yellow items. It lifts the item with the TCP 145 mm above the table, carries it over the yellow item to the tray centre, pauses with the item's bottom 16 mm above the 15 mm rims, lowers it slowly and releases it 5 mm above the tray floor, from where it drops and stands. The red, blue and yellow items are never touched. The grip closes to 8 mm narrower than the part rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the part (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before the close and the release, pauses above the tray and makes the last descent slowly, because the simulated wrist trails a moving target by several millimetres. It reached `task_score: 1.0` in 301 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 7 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 1 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout, and the open-loop demo and the in-place rule need it. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_cf54ab0f.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_ad0d51e3.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 301 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the 18 mm finger is legible; both scored runs and their videos are at 320 x 176, where the forearm, palm and finger still read as one 280 mm line in the top camera's first frame. The whole stand-in, the four items and the tray lie clear of both arms at their ready pose in that view.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Stand the four items upright back on their marks; the stand-in and the tray do not move.
