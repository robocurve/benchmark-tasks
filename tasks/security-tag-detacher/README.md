# Unpin an anti-theft hard tag from a jacket sleeve cuff with a magnetic detacher

The robot seats a hand-held detacher on the lock dome of a retail hard tag so the tag's spring latch lets go, holds
it there, and pulls the tag's pin out of the housing with the other hand. The tag is pinned through the hem tab of a
quilted navy jacket-sleeve cuff. At the end the pin is out of the tag by at least 15 mm, it was pulled while the
detacher sat on the dome, and both grippers have let go and backed off. Retail cashiers and retail salespersons (O*NET
41-2011.00 and 41-2031.00, each over 3 million US workers) take a hard tag off every jacket they sell.

The ability is releasing a latch by seating a tool on it, then extracting the part the latch held while the tool stays
there. Nothing is placed in a target pose for its own sake: the tag's pin only slides out while the detacher's pocket
sits on the dome, and the moment the detacher leaves, the spring closes the latch again.

Physics, not the instruction, defeats the natural plan. The pin knob stands on top of the cuff's hem tab, so the
obvious move is to grip it and pull. The latch bolt sits in the pin's groove, so the pull lifts the whole tag with the
pin and the pin does not move relative to the housing. Dragging the cuff away does not work either: the pin shaft stands in the
tab's hole and drags the tag along. The detacher's nose pocket (18 x 16 mm, 4 mm deep) must go over the 14 mm dome
(2 mm clearance each side, 1 mm above and below) with the nose squared to the tag's face, so that the pocket floor
presses the dome in 1.5 mm, and the hand has to slide
the detacher along the table through the cuff notch into position, correct it, and keep it there while the other
hand pulls. Taking the detacher away first does not help either: the spring pushes the bolt back into the groove.

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
- task file SHA-256, first 12 hex: 1ad11f89d5e7 (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision` in the
  Dream-1 log)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the tag as a PhysX articulation (a housing, a prismatic pin with 0.4 N of joint friction, and
a prismatic latch plunger with a return spring whose bolt stands in a groove of the pin), and the cuff and the detacher
as compound rigid bodies, and calls the task's per-step record after every control step. The latch is the plunger, its
spring and its contact with the pin: `task.py` never switches, freezes or frees any body. No fork of yam_arena and no third-party assets: every body is
built from box and cylinder colliders in `task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

31 tests feed synthetic records to the scorer: a demo-shaped trace scores 1.0; seating alone scores 0; seating limits
at their edges (3.0 vs 3.1 mm sideways, 2.0 vs 2.1 mm in height, rim gap -0.5 and +0.5 mm vs -0.6 and +0.6, tilt 6.0
vs 6.1 deg); the late pull (the detacher seated and then taken away, the pin pulled to any depth afterwards, hands
clear and still) scores 0; a pull started in place and finished after the detacher leaves gets only the seated part and
never succeeds; a pull made in place keeps its credit when the detacher leaves at the end; no record of a release
survives; an earlier tug that moved nothing is no violation; the rungs at 4.9 vs 5.0 mm and 14.9 vs 15.0 mm; the release
gate (29.9 vs 30 mm, 5.1 vs 5.0 mm/s, 0.95 s vs 1.1 s, a hand returning within 30 mm restarts the timer); a pin pushed
back in loses the end rungs; the record is cleared by a reset; `seat_error()` on hand-built poses (aligned, 3.1 mm
sideways, 2.1 mm high, 6 and 7 deg of yaw, the detacher on the cuff side or handle-first, 2 mm into the housing); and
the latch geometry (the bolt tip 0.5 mm inside the shaft outline, a 0.6 mm dome press clears it, the loosest seat still
presses more than that). They need no simulator; the 31 passed offline against a stub of the simulator's task-file
module.

## Revision 3

The reviewer's open points and what changed. The simulator was not available for this revision: the new scene's demo,
cheats and Dream-1 trial have not been run, and every simulator result quoted above is from the previous revision.

| Reviewer point | What changed |
|---|---|
| The simulated clutch stays released after the detacher moves away, so the pin can be pulled later without keeping the magnet in place. | The clutch is gone. The latch is now a real spring-loaded plunger: the dome is its head, a bolt on its stem sits in a groove in the pin shaft, and pressing the dome in withdraws the bolt. When the detacher leaves, the 500 N/m return spring pushes the bolt back to the shaft and it holds the pin by contact. `task.py` no longer switches a brake, a drive or a friction on any body. |
| The scorer permanently records the clutch as released. | The record keeps no clutch state at all (`evers` stays empty). The only memory is the furthest pin travel measured on steps where the detacher is seated, and the former forced-pin gate (a permanent zero) is removed. |
| A pull after the detacher has left must score 0; extraction credit needs the pin pulled while the detacher is in place. | Every rung needs seated pin travel of at least 5 mm; pin travel after the detacher has left adds nothing. A pull begun in place and finished afterwards gets at most the seated part (0.25 or 0.5) and never success. |
| Add a cheat for the late pull and tests. | New variant `late_pull` (the left hand carries the detacher 70 mm away and lets go, then the right hand pulls the pin). New tests: late pull of several depths scores 0, a partly seated then late pull scores 0.25, a pull made in place keeps its credit, no record of a release survives, plus latch geometry checks. |
| Provide a full-score demonstration of the corrected task. | `demo_script` still holds the detacher on the dome through the whole pull, now with a final approach gap of 0.1 mm so that the dome is pressed in about 1.4 mm. It is reasoned through above but not run here; the filing step runs it. |

Dimension changes behind the latch: the pocket is 1.5 mm deep (was 4 mm) so the pocket floor presses the dome, the
pin shaft is 20.5 mm long with a groove, and the seat window's gap limit is -0.5 to +0.5 mm (was -0.5 to +1.5 mm).

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
joint values, the plunger or the record.

1. Left: grips the detacher across its 55 mm width (pads closing along x) 72 mm behind its nose, carries it along the
   table to 30 mm short of the tag's face, then servoes from the seen poses (pocket centre to the dome's x, rim to the
   face plus a gap shrinking from 30 mm to 0.1 mm, yaw to zero by swinging the hand about the nose), so the pocket goes
   over the dome and its floor presses the dome in. It holds there until the pull is done.
2. Right, waiting 100 mm back and 130 mm above the table until the detacher is seen seated: moves over the knob with
   the hand leaning 35 deg away from the left hand (pads still closing along x, so the two hands stay clear), grips the
   14 mm knob and lifts 24 mm.
3. Right lets go and backs off; left lets go, lifts and backs off; both wait.

Results of the previous revision (the clutch released by a task rule): the demo scored 1.0 at step 621, with the pin
17.0 mm out. This revision's scene (the physical latch) has not been run in the simulator, so no demo or cheat result is
claimed for it; the filing step reruns them. Timing on the new scene, reasoned from the geometry: the pocket floor first
touches the dome 1.5 mm short of the face, the plunger is fully pressed at the face, and at the demo's final 0.1 mm gap
the bolt tip is already 1.4 mm clear of its 0.5 mm engagement, so the seat and the latch release coincide; the right
hand starts its pull 1.5 s after the seat (1.0 s wait, then 0.5 s) and lifts for 3 s while the left holds, so the pin
passes the bolt's height with the bolt drawn back; the left lets go only after the pull.

Variants (`DEMO_VARIANT`). The results in the table are from the previous revision (the clutch released by a task
rule); none has been run on the physical-latch scene, and the `late_pull` row is the expected result, not a measurement.

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | Nothing moves. Previous revision, measured. | 0.0 |
| `no_detacher` | Right grips the knob and lifts 24 mm; the detacher is never used. | The latch holds: the whole tag rises with the pin (pin travel about 0). Previous revision, measured. | 0.0 |
| `offset_detacher` | The detacher is slid in 8 mm off the dome's centre. | The nose stops on the dome short and off centre; it never seats. Previous revision, measured. | 0.0 |
| `drag_cuff` | Right grips the cuff across its 90 mm width and drags it 50 mm away; no detacher. | The pin shaft in the tab hole takes the tag along; pin travel about 0. Previous revision, measured. | 0.0 |
| `late_pull` | Left seats the detacher, then carries it 70 mm away and lets go; only then does right grip the knob and lift 24 mm. | Expected: the spring closes the latch, the lift raises the whole tag, pin travel about 0, the scorer's seated pin travel stays under 5 mm. Not yet run. | 0.0 (expected) |

## Build log (previous revision: 10 demo runs, about 3 to 4 min each on the shared L40S)

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
(gap; 0 = touching, the dome pressed in 1.5 mm) and its tilt (angle between its nose normal and the housing's -y), the
pin's travel out of the housing (m), the plunger's travel (m, for the report only), and for each gripper the distance
from its pad points to the detacher and the tag (housing and pin) and its speed. Nothing in the record stands for the
latch: it is the plunger, its spring and the bolt in the groove, simulated by PhysX. The scorer never records the clutch
as released.

- Seated, on one step: dx within 3 mm, dz within 2 mm, gap from -0.5 to +0.5 mm, tilt at most 6 deg. (The bolt clears
  the shaft once the dome is pressed 0.6 mm, so any gap up to 0.9 mm already frees the pin; the scorer asks for
  0.5 mm.)
- The record keeps one number for the pull: the furthest pin travel on a step where the detacher is seated. A pin that
  moves after the detacher has left adds nothing to it.
- 0: that travel is under 5 mm. This covers a pull after the detacher has left (the latch has closed again, the pin
  cannot move; if it is dragged out anyway it earns nothing), and seating without pulling.
- 0.25: seated pin travel 5 mm or more.
- 0.5: seated pin travel 15 mm or more.
- 0.75: 0.5 and the pin is out at least 15 mm at the end (the stroke is 30 mm; a freed pin holds where it is left, 0.4 N
  joint friction against its 0.12 N weight).
- 1.0: 0.75 plus the release gate: every pad point of both grippers at least 30 mm from the detacher, tag housing and pin,
  and both grippers moving under 5 mm/s, for 1 s.

There is no gate and no whole-run zero: a tug that moves nothing is harmless, and the pin cannot be forced against the
bolt because it is geometry.

`score()` returns the rung, `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s), and `distance()` is 1 - score. Real trial: a top photo with a 1 mm scale and a 1.5 mm
feeler gauge for the detacher's rim against the tag's face and its pocket centre on the dome; a ruler and a 3 mm shim
beside the knob for the 5 and 15 mm pin heights, read on video frames while the detacher is still on the dome (a
height reached after it has left does not count); a 30 mm spacer gauge and the video for the release gate.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The previous revision's demo succeeded
  at step 621; the new scene's demo has the same script apart from the final approach gap.
- Runtime on one L40S: a demo run took about 3 to 4 min wall clock including Isaac Sim start-up on the shared VM
  (about 4 control steps/s). A Dream-1 trial is slower, since Dream round trips pause the simulator.
- Seeds: `--seed 0` for every submitted run. `-E object_jitter=0` fixes the layout; nothing is jittered. The scripted
  demo is deterministic (two runs ended in the same state to 0.01 mm); Dream-1 is a hosted model without a seed.

## Submitted videos and scores

- Demo: `python linkdemo.py task.py demo_out`, `demo_out/result.json` (the filing step reruns it on this exact
  `task.py`; the 1.0 at step 621 is the previous revision's). The video is `demo_out/top_cam.mp4` (with `left_cam.mp4` and `right_cam.mp4`) and the screenshot `demo_out/setup.png`
  (frame 0 of the top camera).
- Dream-1 run: made at filing with the command above on this exact `task.py` (1ad11f89d5e7): inspect-robots-dream 1.0.0,
Dream-1 run: not run on this revision. The run box ran out of compute credits before the revised task.py could be trialled, so no Dream-1 score is claimed for it.

```bash
inspect-robots inspect <log>.json
jq '{status, scores: .samples[0].epochs, ends: .samples[0].termination_reasons, limit: .eval.max_steps, seed: .eval.seed, task_sha12: .eval.environment_revision}' <log>.json
```

## Materials

- Hard tag housing: ivory ABS, 56 x 46 x 22 mm, 30 g (friction 0.5 / 0.4). Typical retail size.
- Latch plunger: steel, 4 g, sliding along y in the housing, travel 1.5 mm, return spring 500 N/m (0.75 N at full
  travel) and 0.2 N of friction. Its head is the round silver lock dome, 14 mm across and 3 mm proud of one long face at
  10 mm height. A 3 mm wide, 4 mm high stem runs under the pin to a 3 mm wide riser on the pin's -y side, ending in a
  2 x 2 x 3 mm bolt facing the pin. Pressing the dome in more than 0.6 mm draws the bolt tip out past the shaft's
  outline. Verify on a real tag or a build: the dome's travel and spring force with a depth gauge and a spring scale.
- Tag pin: grey steel, 3 mm shaft 20.5 mm long (bottom 7.5 mm above the table), 30 mm stroke through the cuff's hem tab
  into the housing, with a groove 3.4 mm high (17.0 to 20.4 mm above the table) cut to a 1.2 mm core, and a 14 mm
  diameter by 14 mm knob on top of the tab (12 g). The bolt tip stands 0.5 mm inside the shaft's outline (0.4 mm short
  of the core) and holds the pin by contact: with the dome free, a spring scale on the knob must read at least 60 N
  before the pin moves; with the dome pressed in 1.5 mm it must slide at 3 N or less, and once the dome is let out it
  must hold again at 60 N (a pin left partway out does not re-lock until pushed back to the groove). After it is
  freed the pin stays where it is left on 0.4 N of friction. The knob is fully pinched by two pads 14 mm apart, with a
  squeeze of 1.5 mm.
- Sleeve cuff: quilted navy nylon, folded flat, 90 mm wide, 23 mm thick, 0.18 kg, friction 0.6. The body runs from
  y -0.230 to -0.055; two 15 mm arms run along the tag's sides to y 0.0 and leave a 60 x 55 mm notch (2 mm gaps either
  side of the 56 mm housing, 2 mm behind it) open toward +y; a 3 mm hem tab, 90 x 115 mm, laps over the notch 23 to
  26 mm above the table with a 4 mm square hole for the 3 mm pin shaft. Verify with a 56 x 46 mm gauge block dropped
  into the notch and a 3 mm rod through the hole.
- Hand detacher: orange ABS, 55 x 85 x 20 mm, 0.12 kg (friction 0.5 / 0.4), a pocket in its nose 18 mm wide, 16 mm
  high and 1.5 mm deep (floor and roof 2 mm, side walls 18.5 mm). Verify the fit with a 14 mm round plug that goes
  into the pocket with a 2 mm feeler gauge on each side and a 1 mm one above and below, 1.5 mm deep, so that a plug
  3 mm proud is pressed in 1.5 mm with the rim on the face.
- Layout and physics chosen for reach and feasibility: the tag stands 0.19 m out and the detacher starts 0.16 m to
  its left, inside both arms' reach; a hand-size detacher stands in for the larger counter unit (see `task.json`
  fidelity).
- Positions, colours and friction for every body are in `task.json` (setup) and at the top of `task.py`.

## Reset between trials

Nothing is consumed. Push the pin back into the tag until the bolt clicks into the groove and the knob rests on the hem tab. Slide the
tag into the cuff notch under the tab, lock dome toward +y, pin through the tab's hole. Put the detacher on the table
with its nose rim centred at (0.190, 0.130) and its handle end toward +y, using taped corner marks. Check the latch with
the detacher on the dome (the pin must be free) and without it (the pin must hold 60 N).

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file; `linkkit.py` and `linkdemo.py` carry the full MIT text). They use only public
Isaac Lab and USD Physics techniques, copy no code from other tasks in this repository, and the scene uses no
third-party assets or brands.

## Run logs

This revision (task.py SHA-256 prefix 1ad11f89d5e7) was not run in the simulator: the run box ran out of compute credits before the revised scene could be run. Any demo or cheat result described above comes from the previous revision unless its line says otherwise. The scripted demo is deterministic, and `python linkdemo.py task.py demo_out` runs it on the revised scene. The videos linked from task.json show the previous revision.
