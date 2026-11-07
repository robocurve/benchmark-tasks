# Close out a shadow board and flag the missing tool

Instruction given to the policy: `Put each tool on its matching outline, the stray part that matches no outline in the yellow FOD bin, and the red tag on the outline whose tool is missing. A brass chit means that tool is signed out.`

Files in this folder: `task.json`, `shadow_board_closeout.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: 03ca8c0ab57c (`sha256sum shadow_board_closeout.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/shadow-board-closeout/shadow_board_closeout.py
```

It asserts the scoring rules on hand-built states (start 0; done 1.0; the tag alone on the pliers outline 0.40; each tool alone home, or the bracket alone binned, 0.15; the tag on the cutters outline's chit, standing beside the chit, between the two empty outlines, on the board centre, on any other outline, on an adapter set on the pliers outline, tilted 8 degrees, held still above the outline, moving or 16 mm off earns no tag credit, and 14 mm off, turned along the outline or standing on edge still counts; the bracket on the pliers outline, on the cutters outline beside its chit, on the chit, lying on the board edge or on the socket scores 0 whatever else is right; the bracket lying or leaning on the bin floor counts, on a wall top, outside the bin, held 20 mm up or moving does not, and a socket or the tag in the bin with it voids the bin credit; every loose part dumped in the bin 0; the wrench not turned, 11 degrees off, on end or 9 mm off is not home, and 9 degrees off, end for end, rolled onto another face or 7 mm off is; the socket upside down is home, lying, tilted 8 degrees, 9 mm off, on the pliers outline, which it fits, or on the table beside the board is not; the adapter left at 30 degrees or on its side is not home, a quarter turn plus 6 degrees or upside down is; two tools on each other's outlines or one stacked on another earn nothing; a tool still moving is not home; every tool knocked over 0) and the layout claims (the board as tall as the tallest loose part, the bin walls taller, every place target within 0.30 m of the left base, the pliers and cutters outlines identical, the chit wholly on the cutters outline, and the cutters outline nearer the tag's start than the pliers outline), and replays the left arm's plan through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/shadow-board-closeout/shadow_board_closeout.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put each tool on its matching outline, the stray part that matches no outline in the yellow FOD bin, and the red tag on the outline whose tool is missing. A brass chit means that tool is signed out." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/shadow-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is 0.15 per tool home, 0.15 for the bracket binned and 0.40 for the tag on the pliers outline, with the bracket-on-the-board override, as defined in `task.json`; otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/shadow-board-closeout/shadow_board_closeout.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put each tool on its matching outline, the stray part that matches no outline in the yellow FOD bin, and the red tag on the outline whose tool is missing. A brass chit means that tool is signed out." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/shadow-demo
```

The demo is `demo()` in the task file, open loop and deterministic: the left arm takes the five parts in row order from outboard, the wrench onto its outline turned a quarter, the socket onto its disc, the bracket into the bin, the adapter onto its square turned 30 degrees and the tag across the pliers outline, five top-down pick and place moves at 145 mm travel height. Every pick closes along x, across the row, so each part's neighbours sit beside the pads; the wrist is set to +90 or -90 degrees for the same closing line so its last joint stays inside its limits, and the adapter is gripped across its faces at 30 degrees. The wrench is set down with the fingers closing along y before anything stands on the socket outline under them, and the bracket is released inside the bin with the open fingers about 10 mm clear of its walls. Each part is released 5 mm above its resting surface, the board's outlines or the bin floor. Every grip closes to 8 mm narrower than the part rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the part (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before each close and release, pauses above each place and makes the last descent slowly, because the simulated wrist trails a moving target by several millimetres. It reached `task_score: 1.0` in 1595 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 14 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 6 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout, and the open-loop demo needs it. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_cc673332.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_da49fe71.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 1595 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the five outlines and the chit are legible; both scored runs and their videos are at 320 x 176, where every tool's outline differs by 10 mm or more from any other outline of its shape, and the brass chit, about 5 x 5 px, stands out on the cutters outline.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Stand the five loose parts back on their marks in the row, the wrench's long axis along y and the adapter turned 30 degrees; the board, outlines, chit and bin do not move.
