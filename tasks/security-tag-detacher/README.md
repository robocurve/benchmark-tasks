# Unpin an anti-theft hard tag from a jacket sleeve cuff with a magnetic detacher

The robot seats a hand-held magnetic detacher on the lock dome of a retail hard tag so the tag's clutch lets go, holds
it there, and pulls the tag's pin out of the housing with the other hand. The tag is pinned through the hem tab of a
quilted navy jacket-sleeve cuff. At the end the pin is out of the tag by at least 15 mm and both grippers have let go
and backed off. Retail cashiers and retail salespersons (O*NET 41-2011.00 and 41-2031.00, each over 3 million US
workers) take a hard tag off every jacket they sell.

The ability is releasing a lock by seating a tool on it, then extracting the part the lock held. Nothing is placed in a
target pose for its own sake: the tag's pin only slides out while the detacher's pocket sits on the dome, and nothing
else releases it.

Physics, not the instruction, defeats the natural plan. The pin knob stands on top of the cuff's hem tab, so the
obvious move is to grip it and pull. The clutch is locked (60 N), so the pull lifts the whole tag with the pin and the
pin does not move relative to the housing. Dragging the cuff away does not work either: the pin shaft stands in the
tab's hole and drags the tag along. The detacher's nose pocket (18 x 16 mm, 4 mm deep) must go over the 14 mm dome
(2 mm clearance each side, 1 mm above and below) with the nose squared to the tag's face, so the hand has to slide
the detacher along the table through the cuff notch into position, correct it, and keep it there while the other
hand pulls.

Instruction given to the policy (16 words; it states the goal only, never the technique):

`Unpin the anti-theft tag on the navy jacket cuff with the orange detacher, then let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the
Dream-1 trial below:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- task file SHA-256, first 12 hex: d593e9926c64 (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision` in the
  Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the tag as a PhysX articulation (a housing with a sprung-off prismatic pin, whose clutch is a
stiff position drive that `task.py` switches off), and the cuff and the detacher as compound rigid bodies, and calls
the task's per-step record after every control step. No fork of yam_arena and no third-party assets: every body is
built from box and cylinder colliders in `task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

24 tests feed synthetic records to the scorer: a demo-shaped trace scores 1.0 and releases the clutch exactly once;
seating limits at their edges (3.0 vs 3.1 mm sideways, 2.0 vs 2.1 mm in height, rim gap -0.5 and +1.5 mm vs -0.6 and
+1.6, tilt 6.0 vs 6.1 deg); the 0.3 s seat hold (0.25 s releases nothing, a break restarts it); the gate (2.9 mm of pin
travel under a locked clutch is none, 3.0 mm is, and a perfect seat and pull afterwards cannot undo it; a pull during
the hold is gated); pin travel after the release is free; the rungs at 4.9 vs 5.0 mm and 14.9 vs 15.0 mm; the release
gate (29.9 vs 30 mm, 5.1 vs 5.0 mm/s, 0.95 s vs 1.1 s, a hand returning within 30 mm restarts the timer); a pin pushed
back in loses the end rungs; the record is cleared by a reset; and `seat_error()` on hand-built poses (aligned, 3.1 mm
sideways, 2.1 mm high, 6 and 7 deg of yaw, the detacher on the cuff side or handle-first, 2 mm into the housing).

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Unpin the anti-theft tag on the navy jacket cuff with the orange detacher, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/security-tag-detacher-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). Repeat trials: add `--epochs K`; with `object_jitter=0` the seed changes nothing in the scene.

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                          # scripted demo
python linkdemo.py task.py cheat_out --variant no_detacher   # a cheat variant (or DEMO_VARIANT=no_detacher)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json` (score, steps, IK error and the scorer's record). The script reads only
what a camera could measure: the tag housing, pin, detacher and cuff poses and the robot's own grippers. It never reads
joint values, the clutch state or the record.

1. Left: grips the detacher across its 55 mm width (pads closing along x) 72 mm behind its nose, carries it along the
   table to 30 mm short of the tag's face, then servoes from the seen poses (pocket centre to the dome's x, rim to the
   face plus a gap shrinking from 30 mm to 0.3 mm, yaw to zero by swinging the hand about the nose), so the pocket goes
   over the dome. It holds there. It ends 0.0 mm sideways, 0.0 mm high, 0.0 mm rim gap and 0.0 deg from square (seen).
2. Right, waiting 100 mm back and 130 mm above the table until the detacher is seen seated: moves over the knob with
   the hand leaning 35 deg away from the left hand (pads still closing along x, so the two hands stay clear), grips the
   14 mm knob and lifts 24 mm. The pin rises 17.6 mm and the housing stays down (0.5 mm).
3. Right lets go and backs off; left lets go, lifts and backs off; both wait.

The demo scores 1.0 (`success`, episode ended at step 621 of 1,200): the clutch released at 15.8 s, the pin ends 17.0 mm
out, and both grippers have been clear and still for 1.95 s at the end. Worst IK error 0.09 mm and 0.02 deg.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the demo's task file; every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves; the clutch never releases. | 0.0 |
| `no_detacher` | Right grips the knob and lifts 24 mm; the detacher is never used. | The locked clutch holds: the whole tag rises 16.7 mm with the pin (pin travel 0.07 mm), the clutch never releases. | 0.0 |
| `offset_detacher` | The detacher is slid in 8 mm off the dome's centre. | The nose stops on the dome 4.8 mm short and 9.7 mm off centre; it never seats and the pull moves nothing (0.0 mm). | 0.0 |
| `drag_cuff` | Right grips the cuff across its 90 mm width and drags it 50 mm away; no detacher. | The pin shaft in the tab hole takes the tag along; pin travel 0.02 mm, the clutch never releases. | 0.0 |

## Build log (10 demo runs, about 3 to 4 min each on the shared L40S)

| Run | Change | Result |
|---|---|---|
| 1 | first build (left hand at 90 deg yaw) | 0.0: the left wrist could not reach that yaw (23 deg orientation error) and the detacher went in 16 deg skewed |
| 2 | left hand at -90 deg yaw | 0.0: the detacher drifted 7 mm sideways while sliding and stopped on the dome |
| 3 | swing the hand about the nose to null the seen yaw | 0.0: same drift |
| 4 | continuous visual servo on the seen poses, gap shrinking to 0.3 mm | 0.25: seated to 0.3 mm, clutch released at 15.8 s |
| 5 to 8 | right hand collided with the left hand and wrist over the knob (its tool point was pushed 20 to 50 mm off its path), slimmer hand, waiting farther back | 0.25: the grip never closed on the knob |
| 9 | right hand leans 35 deg away from the left hand | 1.0 at step 621 |
| 10 | final task file (diagnostics removed) | 1.0 at step 621, the same end state; four cheat variants each 0.0 |

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step from reset, the on-step record (a `linkkit.History`) measures, in the tag housing's frame: the
detacher's pocket centre against the dome (sideways dx and height dz), its rim's distance past the housing's +y face
(gap; 0 = touching, the dome 3 mm into the 4 mm pocket) and its tilt (angle between its nose normal and the housing's
-y), the pin's travel out of the housing (m), and for each gripper the distance from its pad points to the detacher and
the tag (housing and pin) and its speed. The record keeps flags, peaks and held times for the whole episode and is
cleared only by the simulator reset, so pulling things apart and starting again never clears a violation.

- Seated: dx within 3 mm, dz within 2 mm, gap from -0.5 to +1.5 mm, tilt at most 6 deg. Seated for 0.3 s releases the
  clutch (the pin's lock is switched off) for the rest of the run.
- Gate, any time in the run, gives 0: the pin up 3.0 mm or more while the clutch is still locked (its stretch under a
  pull is 3 mm at 60 N, so this is a pull that forces it, never a seated release).
- 0.25: the clutch has been released (seated 0.3 s).
- 0.5: the pin was pulled 5 mm at some time after that.
- 0.75: the pin is out at least 15 mm at the end (the stroke is 30 mm; a freed pin holds where it is left, 0.4 N joint
  friction against its 0.12 N weight).
- 1.0: 0.75 plus the release gate: every pad point of both grippers at least 30 mm from the detacher, tag housing and pin,
  and both grippers moving under 5 mm/s, for 1 s.

`score()` returns the rung, `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s), and `distance()` is 1 - score. Real trial: a top photo with a 1 mm scale and a 1.5 mm
feeler gauge for the detacher's rim against the tag's face and its pocket centre on the dome, timed on video for the
0.3 s hold; a ruler and a 3 mm shim beside the knob for the 3, 5 and 15 mm pin heights (a pin up 3 mm before the seat
is the gate); a 30 mm spacer gauge and the video for the release gate.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 621.
- Runtime on one L40S: a demo run took about 3 to 4 min wall clock including Isaac Sim start-up on the shared VM
  (about 4 control steps/s). A Dream-1 trial is slower, since Dream round trips pause the simulator.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout; nothing is jittered. The scripted
  demo is deterministic (two runs ended in the same state to 0.01 mm); Dream-1 is a hosted model without a seed.

## Submitted videos and scores

- Demo: `python linkdemo.py task.py demo_out`, `demo_out/result.json`: `task_score = 1.0`, success at step 621. The
  video is `demo_out/top_cam.mp4` (with `left_cam.mp4` and `right_cam.mp4`) and the screenshot `demo_out/setup.png`
  (frame 0 of the top camera).
- Dream-1 run: made at filing with the command above on this exact `task.py` (d593e9926c64): inspect-robots-dream 1.0.0,
  1200 steps, 4 minutes, log adhoc_ce630ca5.json, task_score 0.0, termination reason max_steps.

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Materials

- Hard tag housing: ivory ABS, 56 x 46 x 22 mm, 30 g, a round silver lock dome 14 mm across and 3 mm proud on one
  long face at 10 mm height (friction 0.5 / 0.4). Typical retail size; the dome is the part the detacher's magnet
  acts through.
- Tag pin: grey steel, 3 mm shaft, 30 mm stroke through the cuff's hem tab into the housing, with a 14 mm diameter by
  14 mm knob on top of the tab (12 g). The clutch holds the pin against 60 N until a detacher is seated; after that
  the pin slides with a 0.4 N friction. To verify on a real tag: a spring scale on the knob must read at least 60 N
  before the pin moves with no detacher, and at most 3 N with the detacher seated. The knob is fully pinched by two
  pads 14 mm apart, with a squeeze of 1.5 mm.
- Sleeve cuff: quilted navy nylon, folded flat, 90 mm wide, 23 mm thick, 0.18 kg, friction 0.6. The body runs from
  y -0.230 to -0.055; two 15 mm arms run along the tag's sides to y 0.0 and leave a 60 x 55 mm notch (2 mm gaps either
  side of the 56 mm housing, 2 mm behind it) open toward +y; a 3 mm hem tab, 90 x 115 mm, laps over the notch 23 to
  26 mm above the table with a 4 mm square hole for the 3 mm pin shaft. Verify with a 56 x 46 mm gauge block dropped
  into the notch and a 3 mm rod through the hole.
- Magnetic hand detacher: orange ABS, 55 x 85 x 20 mm, 0.12 kg (friction 0.5 / 0.4), a pocket in its nose 18 mm wide,
  16 mm high and 4 mm deep (floor and roof 2 mm, side walls 18.5 mm). Verify the fit with a 14 mm round plug that
  goes into the pocket with a 2 mm feeler gauge on each side and a 1 mm one above and below, 3 mm deep.
- Layout and physics chosen for reach and feasibility: the tag stands 0.19 m out and the detacher starts 0.16 m to
  its left, inside both arms' reach; a hand-size detacher stands in for the larger counter unit (see `task.json`
  fidelity).
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Push the pin back into the tag until the clutch clicks and the knob rests on the hem tab. Slide the
tag into the cuff notch under the tab, lock dome toward +y, pin through the tab's hole. Put the detacher on the table
with its nose rim centred at (0.190, 0.130) and its handle end toward +y, using taped corner marks. Check the tag with
the detacher on the dome (the pin must be free) and without it (the pin must hold 60 N).

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.
