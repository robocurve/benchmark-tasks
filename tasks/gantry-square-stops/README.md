# Square a racked two-rail gantry beam by moving both carriages onto their front end stops together

The robot runs the two ends of a printer or CNC gantry beam forward onto their end stops together. The beam is a
20 x 20 x 400 mm aluminium extrusion carried by a captured linear-rail carriage block at each end (MGN12H size, 340 mm
apart), with its belts loose. It starts racked, with one end 2.7 mm ahead of the other. At the end both carriage
blocks rest face to face against their front stops (a 0.05 mm feeler does not enter at either corner), both blocks sit
seated on their rails, the beam is square, and both grippers have let go. Technicians who build and service 3D
printers, CNC routers and Cartesian robots do this every time they square an axis.

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

## Revision 3

The reviewer's third review asked for four things. Each, and what changed:

1. The open-bottom carriage channels let the beam be lifted off its rails and repositioned instead of slid. The
   carriages are now captured. Each rail has a ball groove down both sides (foot 12 x 3 mm, neck 8 x 2 mm, head
   12 x 3 mm; groove 2 mm high and 2 mm deep). Each carriage is an MGN12H-size block, 45.4 x 27 x 13 mm: top plate on
   the rail head, side walls 7.25 mm thick with their bottoms 3 mm above the table (the real H1), and on each wall a
   1.5 mm high lip that sits in the groove, 0.25 mm above the foot, 0.25 mm under the head and 0.25 mm off the neck.
   The lips lie under the rail head, so the beam cannot be lifted more than 0.25 mm off its rails and can only be
   slid. A real MGN12H carriage is held on its rail the same way by its recirculating balls in the same groove; the
   simulation uses fixed lips instead of balls, and preload is not modelled. The beam is bolted to the carriages, and
   the carriages are fitted from the rail's rear end.
2. Positive credit now requires valid rail engagement during the movement. A carriage is on its rail when its block
   is at seated height (every top corner within 1 mm), within 0.6 mm of the rail centreline, yawed under 0.7 deg, and
   between the rail's rear end and the stop face. Each front face's forward travel is summed only over steps where
   that carriage was on its rail ("slid"). The 0.25 rung needs both carriages slid at least 25 mm; every higher rung
   needs both on their rails at the end and the whole displacement made by sliding (within 2 mm). A beam lifted off
   the rails and carried scores 0. A harmless moment with no travel (a brief lift, a fast push) still costs nothing.
3. Full credit now needs real face-to-face contact, correct height and position on both rails. The old gap was the
   largest corner gap and could not see a block past the stop; the gap is now signed per front corner. At 0.75 and
   1.0 every front corner of both blocks is within 0.05 mm of the stop face and not more than 0.10 mm past it, and
   every top corner of each block is within 0.5 mm of its seated height in either direction (the old lift check read
   only one side and only the highest point, so a sunk or tilted block passed), the block centre is on the rail
   centreline, and the block is not yawed off.
4. Tests added in `tests/test_scoring.py` for off-rail and beyond-stop states (see below), and a new demo variant,
   `lift_carry`, that lifts the beam and tries to carry it onto the stops.

The demo script was updated for the captured carriages: the demo still pinches the beam above each carriage and
slides it, now with a settled entry pause and a slow final descent before the pinch, and it reads the worst corner gap
from the new measurement. No simulator was available for this revision, so the revised scene, the demo and the
variants have not been run; the results quoted below are from the previous revision and are marked as such.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- inspect-robots-dream 0.1.0, Dream checkpoint `dreamlabs/dream-1-beta` (for the Dream-1 trial)
- task file SHA-256, first 12 hex: c3df0c40e340 (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision` in the
  Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env`. `linkkit.py` and `linkdemo.py` must sit next to `task.py`;
importing `linkkit` lets `yam_arena.scene` spawn the rails and the gantry as compound rigid bodies built from box
colliders in `task.py`. No fork of yam_arena and no third-party assets.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

30 tests feed synthetic records and beam poses to the scorer. A demo-shaped trace scores 1.0; nothing and one end only
score 0; the gap is the worse of the two ends; the feeler limit (0.049 vs 0.051 mm), the 5 mm near rung and the 25 mm
rung (both ends needed, travel counted only on the rails); a fast moment or a brief lift with no travel does not zero
a squared end state; stillness (1 s) and the release gate (29 vs 30 mm, 6 mm/s, 0.95 s); the record is clean after a
reset. New for off-rail and beyond-stop states: a beam lifted off both rails, carried 40 mm and set down on the stops
scores 0; lifting only one end scores 0; slid part of the way and carried the rest is capped at 0.25; an end state
lifted 5 mm, shifted 1 mm sideways or yawed 0.9 deg is off the rail and capped at 0.25; a block 0.09 mm past its stop
is full, 0.11 mm past is 0.5, 3 mm past (through the stop) is 0.25; a block lifted on one side only (0.7 mm) is not
full; one corner on the stop and the other 0.2 mm off is not face to face; the measurement from a beam pose (start gaps
40.1 and 42.8 mm worst corner, 0 at the stops, a pose past the stop reads negative, height is read in both
directions, a 1 mm sideways shift is read, speed from the pose change); and the lip and groove dimensions of the
captured carriage. They need `yam_arena` importable (run them on the VM after sourcing `yam.env`).

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

1. Both arms together: each hovers over the beam above its own carriage, pauses 20 mm above the grasp and descends
   over 2 s, then pinches the beam from above (pads 17 mm apart on the 20 mm extrusion, closing along x), and pushes
   forward at 8 mm/s by its own remaining gap plus 4 mm, so the left end travels 44 mm and the right 47 mm and the
   2.7 mm lead is kept. The beam only slides: the lips keep each block on its rail.
2. Each arm reads the worst front-corner gap from the seen pose and presses on by what is left (the arm's compliance
   leaves 3 to 4 mm), up to three times.
3. Each arm waits 1 s, lets go, and lifts away.

By the waypoint timing the demo script ends near step 450 of 1,200 (about 1 s of start wait, 7 s to grasp, about 6 s of
push, about 3 s of pressing, 3 s of release and 2 s of final wait at 20 Hz); this is an estimate, not a run.

Results of the previous revision (open-bottom channels, demo of that scene, not re-run on the captured carriages): the
demo scored 1.0 (`success`, episode ended at step 392 of 1,200), both gaps 0.00 mm, yaw 0.00 deg, peak beam speed
16 mm/s, both grippers 63 mm clear and still; each variant below scored 0 in Isaac Sim on that earlier scene. For this
revision the same variants are expected to score 0 or below the demo, and are to be re-run with the filing run.

| Variant | What it does | Previous revision (open channels) | Expected on the captured scene |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves, 0.0. | 0.0 |
| `left_only` | Left arm pushes its end; the right arm stays at home. | Blocks wedge: left advances 1.8 mm, right 0.9 mm, 0.0. | 0.0 (wedge by friction, unchanged) |
| `right_only` | Right arm pushes its end; the left arm stays at home. | Right advances 7.5 mm, left 1.3 mm, 0.0. | 0.0 |
| `one_then_other` | Left arm pushes its full distance, then the right arm. | Left 3.2 mm, right 9.5 mm, 0.0. | 0.0 |
| `lift_carry` | Both arms pinch the beam, lift it 40 mm, carry it forward and set it down. | Not available (the open channels allowed it). | The lips stop the lift at 0.25 mm; any travel is a slide on the rails; a beam that ends off its rails or is carried scores at most 0.25. Not yet run. |

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step the record (`Record` in `task.py`, a `linkkit.History`) measures, for each carriage block: the signed
gap (mm, negative is past the stop) of its two front corners to the stop face, the height of its four top corners from
the seated height, its lateral offset from the rail centreline and its front-face x; and the beam yaw and speed.

- A carriage is on its rail when the worst top corner is within 1 mm of seated height, the block centre is within
  0.6 mm of the rail centreline, the beam is yawed under 0.7 deg, and no front corner is more than 0.5 mm past the stop
  face or beyond the rail's rear end. The lips allow 0.25 mm of lift and of lateral play, so these limits sit just
  outside what the real parts allow.
- Slid travel: each front face's forward travel summed over steps where that carriage was on its rail at both ends of
  the step. A beam lifted off and set down further on adds nothing.
- No gate: only the end state and the slid travel count. A fast moment, a brief lift with no travel or a wrong first
  push that is undone costs nothing if the final state is complete.
- 0.25: both carriages were slid at least 25 mm forward on their rails.
- 0.5: both on their rails at the end, the whole displacement made by sliding (within 2 mm), every front corner within
  5 mm of its stop (not more than 5 mm past it).
- 0.75: as 0.5, and face to face and seated: every front corner of both blocks at most 0.05 mm from its stop face and
  not more than 0.10 mm past it, every top corner within 0.5 mm of seated height.
- 1.0: also the beam under 3 mm/s for 1 s, and the release gate: every pad point of both grippers at least 30 mm from
  the beam and both grippers moving under 5 mm/s for 1 s.

Real trial: a 0.05 mm feeler gauge at both corners of each stop face (it must not enter), a dial gauge or feeler at the
block ends for height, a tape for 25 mm and 5 mm travel, 60 fps video for 1 s stillness and to show the carriages
slide on the rails. The same limits are in `task.py` and `task.json`.

## Run limit, runtime and seeds

- Run limit 1,200 control steps at 20 Hz (`--max-steps 1200`); the revised demo is estimated to end near step 450.
- A demo run on the previous revision took about 4 minutes wall clock on the shared L40S, including about 2 minutes of
  Isaac Sim start-up.
- Seed 0 for every run; `object_jitter=0` fixes the layout; the scripted demo is deterministic.

## Submitted videos and scores

- Demo: `python linkdemo.py task.py demo_out`, `demo_out/result.json`, video `demo_out/top_cam.mp4`, screenshot
  `demo_out/setup.png`; the figures in this README for the demo are from the previous revision until the filing run
  replaces them.
Dream-1 run: not run on this revision. The run box ran out of compute credits before the revised task.py could be trialled, so no Dream-1 score is claimed for it.
  this exact `task.py` (c3df0c40e340). It was not refused and ran to its stop condition; the measured result is in the Scored runs table.(`--max-steps 40`, no frames).

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Materials

- Linear rails: MGN12 pattern, hardened steel, 12 mm wide x 8 mm high x 250 mm long, two, fixed to the table along x
  from x 0.030 to 0.280 at y = +-0.170 (340 mm apart). Profile: foot 12 x 3 mm, neck 8 x 2 mm, head 12 x 3 mm (a
  2 mm high, 2 mm deep groove down both sides). Friction 0.2.
- End-stop blocks: steel, 10 x 30 x 13 mm, fixed at the front end of each rail, stop face at x 0.280, red.
- Carriage blocks: MGN12H size, steel, 45.4 x 27 x 13 mm, grey: top plate on the rail head, side walls 7.25 mm thick
  around the 12.5 mm channel (0.25 mm clearance a side) with bottoms 3 mm above the table, and on each wall a 1.5 mm high,
  2.5 mm long retaining lip in the groove (0.25 mm above the foot, 0.25 mm under the head, 0.25 mm off the neck), so the
  carriage cannot be lifted off the rail.
- Beam: 2020 aluminium extrusion 20 x 20 x 400 mm, silver, bolted on top of the blocks, with an orange toolhead 40 x 50 x
  40 mm on its front at the centre. Beam, blocks and toolhead are 0.80 kg together.
- Layout chosen for reach: the rails sit 0.17 m either side of the midline, 0.135 m inside each arm base, and the
  stops 0.28 m out, so each grasp is within 0.33 m of its arm base. The 0.25 mm clearance is a worn guide: a new
  linear guide is tighter and also binds, over a smaller racking range.
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Slide both carriages back along the rails until the left front face is 40 mm and the right 42.7 mm
from its stop (tape), the beam racked with the left end ahead. The carriages stay on their rails (the groove lips hold
them; refit from the rail's rear end if removed). Wipe the rails dry; check a 0.2 mm feeler passes the clearance at each
block's side.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.

## Run logs

This revision (task.py SHA-256 prefix c3df0c40e340) was not run in the simulator: the run box ran out of compute credits before the revised scene could be run. Any demo or cheat result described above comes from the previous revision unless its line says otherwise. The scripted demo is deterministic, and `python linkdemo.py task.py demo_out` runs it on the revised scene. The videos linked from task.json show the previous revision.
