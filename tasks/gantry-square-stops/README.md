# Square a racked two-rail gantry beam by moving both carriages onto their front end stops together

The robot runs the two ends of a printer or CNC gantry beam forward onto their end stops together. The beam is a
20 x 20 x 400 mm aluminium extrusion carried by a linear-rail carriage block at each end (MGN12 pattern, 340 mm
apart), with its belts loose. It starts racked, with one end 2.7 mm ahead of the other. At the end both carriage
blocks rest against their front stops (a 0.05 mm feeler does not enter at either), the beam is square, and both
grippers have let go. Technicians who build and service 3D printers, CNC routers and Cartesian robots do this every
time they square an axis.

The ability is driving the two ends of a racked rigid beam forward in step so it squares. Nothing is placed, locked
or fastened.

Physics, not the instruction, forces both arms. Each carriage block has 0.25 mm clearance a side on its rail, so the
beam can only yaw 0.63 deg (3.7 mm between its ends) before both blocks cock on their rails. The beam starts at
-0.45 deg with the left end ahead. Pushed at one end only, the beam yaws to that limit and the cocked blocks wedge
by contact friction: the wedge holds at any push force, so the other carriage never reaches its stop. Only two pushes
that advance in step, each end by its own remaining distance, land both blocks on the stops.

Instruction given to the policy (23 words; it states the goal only):

`Move the gantry beam's two carriages forward along the rails until both rest against their front stops, then let go with both grippers.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta` (for the Dream-1 trial)
- task file SHA-256, first 12 hex: 1791c2062a6f (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision` in the
  Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. `linkkit.py` and `linkdemo.py` must sit next to `task.py`;
importing `linkkit` lets `yam_arena.scene` spawn the rails and the gantry as compound rigid bodies built from box
colliders in `task.py`. No fork of yam_arena and no third-party assets.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

16 tests feed synthetic records to the scorer: a demo-shaped trace scores 1.0; nothing and one end only score 0; the
gap is the worse of the two ends; the feeler limit (0.049 vs 0.051 mm), the 5 mm near rung and the 25 mm rung (both
ends needed); a fast or lifted moment mid-run does not zero a squared end state, and an end with a block lifted 1.0 mm (0.9 passes) scores 0.75; stillness
(1 s) and the release gate (29 vs 30 mm, 6 mm/s, 0.95 s); the record is clean after a reset; and the measurement from a
beam pose (start gaps 40.0 and 42.7 mm, 0 at the stops, a racked beam at a stop reads the feeler gap, speed from the
pose change). They need `yam_arena` importable (run them on the VM after sourcing `yam.env`).

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Move the gantry beam's two carriages forward along the rails until both rest against their front stops, then let go with both grippers." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/gantry-square-stops-dream
```

Scoring is automatic: `task_score` is 1.0 when `success()` holds for 1.0 s and the episode ends with
`termination_reason="success"`; otherwise it is `score()` at the end (`SCORE_OVER = "end"`). With `object_jitter=0`
the seed changes nothing in the scene.

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                          # scripted demo
python linkdemo.py task.py cheat_out --variant left_only     # a cheat variant (or DEMO_VARIANT=left_only)
```

`linkdemo.py` runs `demo_script()` from `task.py` at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`, `right_cam.mp4`,
`setup.png` and `result.json`. The script reads only the beam's pose (the block front faces) and the robot's grippers.

1. Both arms together: each pinches the beam from above over its own carriage (pads 17 mm apart on the 20 mm
   extrusion, closing along x), then pushes forward at 8 mm/s by its own remaining gap plus 4 mm, so the left end
   travels 44 mm and the right 47 mm and the 2.7 mm lead is kept.
2. Each arm reads the front-face gap from the seen pose and presses on by what is left (the arm's compliance leaves
   3 to 4 mm), up to three times.
3. Each arm waits 1 s, lets go, and lifts away.

The demo scores 1.0 (`success`, episode ended at step 392 of 1,200): both gaps 0.00 mm, yaw 0.00 deg, peak beam speed
16 mm/s, peak block lift 1.3 mm, both grippers 63 mm clear and still. Worst IK error 0.29 mm and 1.8 deg.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the submitted task file (1791c2062a6f); every one scores exactly 0 (the scorer reads only the end state, so a fast push that still squares the beam is not a shortcut and scores like the demo):

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves. | 0.0 |
| `left_only` | Left arm pushes its end; the right arm stays at home. | The leading end is already at the clearance limit, the blocks wedge: left advances 1.8 mm, right 0.9 mm. | 0.0 |
| `right_only` | Right arm pushes its end; the left arm stays at home. | The right end closes the 2.7 mm lead, then wedges at the other limit: right advances 7.5 mm, left 1.3 mm. | 0.0 |
| `one_then_other` | Left arm pushes its full distance, then the right arm. | Wedges as in `left_only`; the right push then wedges too: left 3.2 mm, right 9.5 mm. | 0.0 |

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step the record (`Record` in `task.py`, a `linkkit.History`) measures the largest gap (mm) from each
block's front face to its stop face, each block's height above its rail, and the beam's speed from its pose change.

- No gate: only the end state counts. A fast moment, a lifted block or a wrong first push that is undone costs nothing if the final state is complete.
- 0.25: both carriages came at least 25 mm forward at some time.
- 0.5: at the end both gaps are at most 5 mm.
- 0.75: at the end both gaps are at most 0.05 mm (squared).
- 1.0: squared, both blocks seated (lift under 1 mm), the beam under 3 mm/s for 1 s, and the release gate: every pad point of both grippers at least 30 mm
  from the beam and both grippers moving under 5 mm/s for 1 s.

Real trial: a 0.05 mm feeler gauge at each stop face (it must not enter), a tape for 25 mm and 5 mm travel, 60 fps
video for 1 s stillness, a ruler or feeler at the block ends for a lifted block. The same limits are in `task.py` and `task.json`.

## Run limit, runtime and seeds

- Run limit 1,200 control steps at 20 Hz (`--max-steps 1200`); the demo succeeds at step 392.
- A demo run took about 4 minutes wall clock on the shared L40S, including about 2 minutes of Isaac Sim start-up.
- Seed 0 for every run; `object_jitter=0` fixes the layout; the scripted demo is deterministic.

## Submitted videos and scores

- Demo: `python linkdemo.py task.py demo_out`, `demo_out/result.json`: `task_score = 1.0`, success at step 392;
  video `demo_out/top_cam.mp4`, screenshot `demo_out/setup.png`.
- Dream-1 run: 0.0 (max_steps), 1200 steps, 3 minutes; log adhoc_c7bfdd0d.json, made at filing with the command above on
  this exact `task.py` (1791c2062a6f). It was not refused and ran to its stop condition; the measured result is in the Scored runs table.(`--max-steps 40`, no frames).

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Materials

- Linear rails: MGN12 pattern, hardened steel, 12 mm wide x 8 mm high x 250 mm long, two, fixed to the table along x
  from x 0.030 to 0.280 at y = +-0.170 (340 mm apart). Friction 0.2.
- End-stop blocks: steel, 10 x 30 x 13 mm, fixed at the front end of each rail, stop face at x 0.280, red.
- Carriage blocks: steel, 45.4 x 27 x 13 mm, U-channel 12.5 mm wide over the 12 mm rail (0.25 mm clearance a side),
  grey; the walls end 1 mm above the table.
- Beam: 2020 aluminium extrusion 20 x 20 x 400 mm, silver, on top of the blocks, with an orange toolhead 40 x 50 x
  40 mm on its front at the centre. Beam, blocks and toolhead are 0.80 kg together.
- Layout chosen for reach: the rails sit 0.17 m either side of the midline, 0.135 m inside each arm base, and the
  stops 0.28 m out, so each grasp is within 0.33 m of its arm base. The 0.25 mm clearance is a worn guide: a new
  linear guide is tighter and also binds, over a smaller racking range.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Slide both carriages back along the rails until the left front face is 40 mm and the right 42.7 mm
from its stop (tape), the beam racked with the left end ahead. Wipe the rails dry; check a 0.2 mm feeler passes the
clearance at each block's side.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.
