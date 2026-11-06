# Sort parts by the rule shown on a limit-sample board

Instruction given to the policy: `The sample board shows accepted parts on its green half and rejected parts on its red half. Sort the four loose parts by the same rule: accepted into the green tray, rejected into the red tray.`

Files in this folder: `task.json`, `limit_sample_sort.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: 144505484fc4 (`sha256sum limit_sample_sort.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/limit-sample-sort/limit_sample_sort.py
```

It asserts the scoring rules on hand-built states (start 0, done 1.0; sorting by colour or by shape either way round, height the wrong way round, all four in one tray, the nearest tray, alternating trays from either end and the board half straight ahead of each part 0; only the two short parts in green or only the two tall parts in red 0.5; three right with the fourth on its mark 0.75 and with the fourth in the wrong tray 0.5; a part lying or leaning on a pocket floor counted; a part on a wall top, standing on another part, resting on a lying part, held moving over a pocket, held still 5 mm (the demo's release height) or 12 mm above the floor, put on the board, beside a tray, on the floor under a tray, knocked over, or still rolling not counted; the pocket edge exact to 1 mm), checks that both colours and both shapes sit on both board halves and that no loose part shares a colour with an example, and replays the left arm's plan through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

The board carries five fixed examples, each 30 mm across, and the self test checks the split below:

| Board half | Example | Colour | Shape | Height |
|---|---|---|---|---|
| green (accepted) | 1 | grey | cylinder | 35 mm |
| green (accepted) | 2 | blue | box | 35 mm |
| green (accepted) | 3 | grey | box | 35 mm |
| red (rejected) | 4 | blue | box | 70 mm |
| red (rejected) | 5 | grey | cylinder | 70 mm |

Grey and blue both appear on both halves, and so do box and cylinder, so neither colour nor shape separates the halves. Every accepted example is 35 mm tall and every rejected one 70 mm, so height does. Volume splits the same way (24.7 and 31.5 cm3 accepted, 49.5 and 63 cm3 rejected), so every single-attribute rule that fits all five examples gives the same answer. The four loose parts are white and yellow, colours that never appear on the board, so no loose part has an identical twin to copy:

| Loose part | Height | Belongs in | y (m) |
|---|---|---|---|
| white box | 35 mm | green tray | 0.395 |
| yellow box | 70 mm | red tray | 0.335 |
| white cylinder | 70 mm | red tray | 0.275 |
| yellow cylinder | 35 mm | green tray | 0.215 |

Each colour and each shape has one accepted and one rejected part, so sorting by colour or by shape, either way round, puts two parts right and two wrong and scores 0. The row runs accept, reject, reject, accept from outboard to inboard, so the nearest tray, alternating trays from either end, and the board half straight ahead of each part also score 0. With four parts in a row one of the three two-and-two splits must match the rule: here it is the two end parts against the middle two, which are also the two nearest the left arm's base.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/limit-sample-sort/limit_sample_sort.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The sample board shows accepted parts on its green half and rejected parts on its red half. Sort the four loose parts by the same rule: accepted into the green tray, rejected into the red tray." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/limit-sample-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is max(0, C - W) / 4 with C the parts in the tray their height calls for and W the parts in the other tray, as defined in `task.json`; otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/limit-sample-sort/limit_sample_sort.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The sample board shows accepted parts on its green half and rejected parts on its red half. Sort the four loose parts by the same rule: accepted into the green tray, rejected into the red tray." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/limit-sample-demo
```

The demo is `demo()` in the task file, open loop and deterministic: the left arm takes the two 70 mm parts standing into the red tray, then the two 35 mm parts into the green tray, four top-down pick and place moves. Wherever another part stands beside the one being moved, at the row or in the tray, the fingers close along x, so the neighbour sits beside the pads rather than in front of them; the wrist yaw is chosen per move so that joint 6 stays inside its limits. The tall parts go first, so no short part is gripped beside a taller neighbour. A short part is gripped with the pad tips 14 mm above the table, so the pads pass 9 mm over the 10 mm walls when they open; a tall part is gripped 15 mm under its top. Each part is carried at 14.5 cm, paused with its bottom 10 mm above the walls, lowered slowly and released 5 mm above the pocket floor. Every grip closes to 8 mm narrower than the part rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the part (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before each close and release and makes the last descent slowly, because the simulated wrist trails a moving target by several millimetres. It reached `task_score: 1.0` in 1272 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 14 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 5 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout, and the open-loop demo needs it. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_3bd32e2e.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_adade658.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 1272 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360; both scored runs and their videos are at 320 x 176. Frame 0 of the top camera was rendered at both sizes and checked against the calibrated camera. No arm hides any example, though the outboard grey box stands in the left gripper's shadow. The near side of a 35 mm example shows as about 6 px at 640 x 360 and 3 px at 320 x 176, and a 70 mm example's as about 13 and 6 px; the loose parts show about 5 against 9 px and 2 against 5 px. At the ready pose the left arm's white upper link hides the green tray's outboard wall, about half of its near wall, over a third of its far wall and about half of its pocket in the top camera; its inboard wall and the whole red tray stay visible. The cue reads from the top camera. At its ready pose the left wrist camera sees only the examples' top faces along its bottom edge; looking down at the row from travel height it shows the loose parts rising 20 px against 40 px.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Stand the four loose parts back on their marks in the row: white box, yellow box, white cylinder, yellow cylinder from outboard to inboard. The board, its glued examples and the trays do not move.
