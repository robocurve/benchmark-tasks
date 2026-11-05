# Site a relay beacon with sight of habitat and rover

Instruction given to the policy: `Stand the white relay beacon upright on the grey plateau where it has a clear straight line of sight from its centre to the centre of the blue habitat and to the centre of the orange rover. The dark ridges block sight.`

Files in this folder: `task.json`, `relay_line_of_sight.py` (scene, success, partial credit and the scripted demo), `README.md`.

## Environment

Built from the program's Brev launchable (Isaac Lab-Arena 0.3.1, Isaac Sim 6.1, L40S). Versions used for the submitted runs:

- inspect-robots 0.60.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta`
- task file SHA-256, first 12 hex: fc58b74a2893 (`sha256sum relay_line_of_sight.py | cut -c1-12`; equals `eval.environment_revision` in both logs)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. The Dream key sits in `~/.dreamlabs/credentials`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/relay-line-of-sight/relay_line_of_sight.py
```

It asserts the scoring rules on hand-built states (start 0; the demo's spot 1.0, also upside down, at 4 degrees of tilt and at a second point in the pocket; the plateau centre, the midpoint between the sites and a point beside the habitat 0.25, each seeing one site; the plateau spot nearest the start 0, seeing neither; tilted 8 degrees, lying on its side, held 10 mm up, still moving, on the table past the plateau edge, 1 mm outside the plateau outline, on a ridge, on the habitat or on the rover 0), the segment-to-rectangle distance on known cases, and the layout's claims (every ridge top at least 25 mm above the beacon, the habitat and the rover; every ridge on the plateau; the demo's spot at least 45 mm from every ridge with both lines clear by at least 15 mm; the start 30 mm off the plateau edge; on a 5 mm grid, 10 to 17 percent of the free plateau seeing both sites, all but a sliver at the back edge in one connected pocket, and 36 to 46 percent seeing one, so a blind upright placement averages 0.22 to 0.26), and replays the left arm's plan through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.

## Run and score one Dream-1 trial

From the root of this repository on the launchable:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/relay-line-of-sight/relay_line_of_sight.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stand the white relay beacon upright on the grey plateau where it has a clear straight line of sight from its centre to the centre of the blue habitat and to the centre of the orange rover. The dark ridges block sight." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/relay-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends with `termination_reason="success"`; otherwise the task file's `score()` at the end of the episode, which is the 0 / 0.25 / 1.0 ladder defined in `task.json`; otherwise 0.0. The CLI prints `task_score: <value>` and the log keeps it at `results.metrics.task_score` and `samples[0].epochs[0].task_score`.

Repeat trials: add `--epochs K`; trial k is `scene-0-e<k>`. With `object_jitter=0` the seed changes nothing in the scene.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/relay-line-of-sight/relay_line_of_sight.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stand the white relay beacon upright on the grey plateau where it has a clear straight line of sight from its centre to the centre of the blue habitat and to the centre of the orange rover. The dark ridges block sight." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/relay-demo
```

The demo is `demo()` in the task file, open loop and deterministic: the left arm grips the beacon across its 30 mm diameter with the fingers closing along x and the pad tips 22 mm up its side, lifts it with the TCP 145 mm above the table so its bottom passes 28 mm over the ridge tops, and turns the grip to 45 degrees on the way, so the fingers open parallel to ridge 3. It pauses with the beacon 15 mm above the spot at (0.205, 0.365), lowers it slowly and releases it 5 mm above the plateau, 48 mm from ridge 3; from there the lines to the habitat and the rover clear the ridges by 48 and 19 mm. The grip closes to 8 mm narrower than the part rather than fully shut, so each finger pushes about 20 N, under its 50 N cap, and the pair centres the part (a fully commanded close saturates both fingers and lets the part slide onto one of them); the arm settles before the close and the release, pauses above the spot and makes the last descent slowly, because the simulated wrist trails a moving target by several millimetres. It reached `task_score: 1.0` in 300 steps (`stats.total_steps`), ended by `success`, within the run limit of 2,400.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400` above, recorded as `eval.max_steps`.
- Runtime per trial on an L40S: Dream-1 about 12 min wall clock for the submitted 2,400-step trial (Dream round trips pause the simulator); the demo about 2 min; the first run after boot adds about 4 to 6 min of Isaac Sim warm-up.
- Seeds: `--seed 0` (the inspect-robots default) for every submitted run; `-E object_jitter=0` fixes the layout, and the open-loop demo needs it. Dream-1 is a hosted model without a seed, so its trials are not bitwise reproducible; the demo is.

## Submitted videos and scores

- Dream-1 run: log `adhoc_289db21b.json`, trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps.
- Demo: log `adhoc_b5cdec77.json`, trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 300 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

Videos were rendered with `inspect-robots video <log>` (one H.264 MP4 per camera in `stats.frames_dir`; the uploaded file is the `top_cam` stream) and the screenshot with `ffmpeg -y -i "<frames>/<trial>_top_cam.mp4" -frames:v 1 setup.png`. The screenshot is rendered at 640 x 360 so the 12 mm thick ridges and the 40 mm sites are legible; both scored runs and their videos are at 320 x 176. In the top camera's first frame no arm at its ready pose hides the plateau, a ridge, a site or the beacon.

What the top camera shows: it looks down at 60 degrees and resolves about 5 mm per pixel on the plateau at 320 x 176. The ridges are about 2 px thick and 15 px long, the sites 8 to 9 px across, the beacon 7 to 9 px and the pocket about 24 x 22 px. Because the view is oblique, a 70 mm ridge's top appears 40 to 50 mm beyond its footprint, and a line 30 to 45 mm above the plateau appears 17 to 27 mm beyond its plan position. A line drawn on the image from a beacon at most points of the pocket to the rover's top therefore appears to touch the silhouette of ridge 2 or ridge 3 by about 1 px (at most about 3 px), although in plan the footprints clear it (by 19 mm at the demo's spot). The scoring uses the plan view, never lines drawn on the photo. Ridge 1 hides under a tenth of the rover's top. Ridge 3 hides part of the pocket's floor behind it, and in the strip of the pocket just behind ridge 3 it hides the lower side of a standing beacon; the centre of the beacon's top stays in view everywhere in the pocket.

## Verify

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, hz: .eval.embodiment_info.control_hz, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Reset between trials

Nothing is consumed. Stand the beacon upright back on its mark on the table; the plateau, the ridges, the habitat and the rover do not move.
