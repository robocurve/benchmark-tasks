# Rebalance three emergency kits without spares

Instruction given to the policy: `Each grey mat is an emergency kit. Make each kit hold exactly one white mask, one yellow torch and one red dressing pack. There are no spares, so move items between the kits. Leave items that already belong where they are.`

Files in this folder: `task.json`, `emergency_kit_cycle_rebalance.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: c7128e21fbfa (`sha256sum emergency_kit_cycle_rebalance.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/emergency-kit-cycle-rebalance/emergency_kit_cycle_rebalance.py
```

It asserts the scoring rules on hand-built states (start 0, the three-way cycle 1.0 whichever of kit 1's masks stays, one cycle move 0, any two cycle moves 1/3, a swap of two kits' surpluses 1/3, filling a gap from the nearest kit with that kind 0, then 1/3 once another move completes kit 3, 1.0 when repaired with a mask back on kit 2's mask place and 2/3 with that mask on another place, a regrouping one of each per mat that moves items which belonged 1/3, a kept item moved to another place or off both places of its kind spoiling its kit, a kept item 15 mm from its start place counted and 16 mm not, a kept item turned and tipped where it stands counted, surpluses taken to the table 0, moved items anywhere along a mat counted, an item left in a gap between mats counted nowhere, a torch lying on its mat, a mask on its side and a pack on its narrow side counted, a torch lying 8 degrees off level counted, a torch leaning 30 degrees above level, a mask leaning 45 degrees on an edge or a mask stacked on a dressing pack spoiling that kit, a torch tilted 8 degrees from upright counted and 12 degrees not, upside-down and yawed items counted, a mask 5 mm above the mat spoiling its kit, a mask held 150 mm up or still moving, and a torch still rolling or knocked off the table, not counted, an extra item still sliding or being lowered onto a kit spoiling it and one carried 120 mm over a kit not, the mat edge exact to 1 mm, every item tipped onto its side where it stood 0 and the cycle done with every item on its side 1.0, every item swept onto one mat 0) and replays the left arm's plan through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/emergency-kit-cycle-rebalance/emergency_kit_cycle_rebalance.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Each grey mat is an emergency kit. Make each kit hold exactly one white mask, one yellow torch and one red dressing pack. There are no spares, so move items between the kits. Leave items that already belong where they are." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/kit-rebalance-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is the kit count defined in `task.json` (complete kits over three, a kit complete only when exactly one mask, one torch and one dressing pack rest on its mat, each on a face or an end, nothing else lies on it, and the items it kept are still within 15 mm of their start places); otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/emergency-kit-cycle-rebalance/emergency_kit_cycle_rebalance.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Each grey mat is an emergency kit. Make each kit hold exactly one white mask, one yellow torch and one red dressing pack. There are no spares, so move items between the kits. Leave items that already belong where they are." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/kit-rebalance-demo
```

Kit 1 is the mat farthest to the left as seen from behind the arms, which is also the left of the top camera image. The demo is `demo()` in the task file, open loop and deterministic: the left arm takes kit 1's extra mask to the free place on kit 3, kit 3's extra dressing pack to the free place on kit 2 and kit 2's extra torch to the free place on kit 1, three top-down pick and place moves that never touch the six items that already belong, with the fingers closing along the line across the mats, so the next mat's items sit 120 mm off the grasp centre and the same mat's neighbours sit beside the pads rather than in front of them. Each item is carried at 14.5 cm, paused with its bottom 20 mm above the mat, lowered slowly and released 5 mm above the mat; the pad tips stay at least 4 mm above the mats throughout. Every grip closes to 8 mm narrower than the item rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the item (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before each close and release and makes the last descent slowly, because the simulated wrist trails a moving target by several millimetres. It reached `task_score: 1.0` in 948 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 9 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 4 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_4287b21d.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_fa320b75.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 948 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the three grey mats and the nine 25 to 60 mm items, white, yellow and red, are legible; both scored runs and their videos are at 320 x 176.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Stand the nine items back on their start places: kit 1 mask, mask, dressing; kit 2 torch, torch, mask; kit 3 dressing, dressing, torch, at x = 0.125, 0.185 and 0.245, leaving x = 0.305 free on every mat. The mats do not move.
