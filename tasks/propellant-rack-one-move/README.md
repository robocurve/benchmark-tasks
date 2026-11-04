# Separate fuel from oxidizer with one move

Instruction given to the policy: `No blue fuel canister may stand in a bay that shares a wall with a bay holding an orange oxidizer canister. Fix the rack by moving as few canisters as possible, and leave every canister upright in a bay.`

Files in this folder: `task.json`, `propellant_rack_one_move.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: 424c68c4443a (`sha256sum propellant_rack_one_move.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/propellant-rack-one-move/propellant_rack_one_move.py
```

It asserts the scoring rules on hand-built states (start 0; the one-move fix 1.0; the nearer empty bay B3 0; either orange canister moved alone, the other blue canister moved alone or a white canister moved alone 0; both orange canisters moved, the fix plus a white canister moved, the A1 blue canister into B4 with the A2 one into the freed A1, or an orange canister out of A3 with the A2 blue one into it, 0.6; three moved, the fix with two orange or two white canisters traded, and a six-move rebuild 0.3; the two blue canisters traded 0; the fix followed by the other blue canister into the freed bay 0; a canister nudged inside its own bay not counted as moved; the moved canister touching two walls, leaning 8 degrees, on its other end or held still 3 mm up still counted; leaning 12 degrees, lying, still moving, held 4 mm up or at the demo's release height, a white canister knocked over, two canisters in one bay, a canister on a divider, on another canister, outside the rack or off the table 0), checks by enumeration that the fix is the only compliant single move and that of the 20,160 end layouts with one canister per bay one scores 1.0, nine 0.6 and 2,390 0.3, and replays the left arm's plan through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/propellant-rack-one-move/propellant_rack_one_move.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "No blue fuel canister may stand in a bay that shares a wall with a bay holding an orange oxidizer canister. Fix the rack by moving as few canisters as possible, and leave every canister upright in a bay." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/propellant-rack-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is 0 if a blue canister shares a wall with an orange one, a canister is out of a bay or two share a bay, and otherwise 1.0, 0.6 or 0.3 for one, two, or three or more canisters moved (those that end in a bay other than their own start bay), as defined in `task.json`; otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/propellant-rack-one-move/propellant_rack_one_move.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "No blue fuel canister may stand in a bay that shares a wall with a bay holding an orange oxidizer canister. Fix the rack by moving as few canisters as possible, and leave every canister upright in a bay." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/propellant-rack-demo
```

The demo is `demo()` in the task file, open loop and deterministic: the left arm makes one top-down pick and place, the blue canister from bay A2 to bay B4, carried by way of the nearer empty bay B3 so it never passes over an orange canister (a straight carry would cross the rim of A3; through B3 both orange centres stay 63.6 mm away in plan). The fingers close across the table, so at A2 the row neighbours stand 70 mm from the grasp centre to their surface and at B4 the white canister in A4 stands beside the pads. The pad tips stay 12 mm above the 15 mm dividers, the canister is carried at 14 cm, paused with its bottom 10 mm above the dividers, lowered slowly and let go 7 mm above the table, so it never reads as in a bay while still held. Every grip closes to 8 mm narrower than the part rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the part (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before each close and release, pauses above the bay and makes the last descent slowly, because the simulated wrist trails a moving target by several millimetres. It reached `task_score: 1.0` in 330 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 13 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 2 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout, and the open-loop demo needs it. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_98cdf129.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_27ff4879.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 330 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the dark 6 mm dividers and the three canister colours are legible; both scored runs and their videos are at 320 x 176.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Stand each canister back at the centre of its own start bay, by the label on its top: blue in A1 and A2, orange in A3 and B2, white in A4 and B1, with B3 and B4 empty. The rack does not move.
