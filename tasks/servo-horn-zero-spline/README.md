# Fit a servo horn onto its splined output shaft with the horn arm on the zero mark

The robot picks up a white single-arm servo horn, turns it so its arm lies on the engraved zero mark of a bolted-down hobby servo,
sets its bore over the 25-tooth output shaft, presses it flush on the boss and lets go. The shaft has 25 teeth, so the horn
seats at only 25 angles, 14.4 deg apart. Pressed at the angle it was picked up at, or wiggled until it drops in, it seats a
whole tooth away from the zero mark and stays there unless it is pulled off and refitted, so the joint is off. Robot builders do this for every joint
of every arm they assemble; a robot that can do it can build and repair robots.

The ability is choosing the wrist angle that matches one of 25 indexed positions before the axial press. It is not
pick and place (the pick is trivial; the part that decides the score is the angle, held to 2 deg through the grasp,
the lift and the release) and not a puzzle (nothing to deduce: the mark is engraved, the index spacing is fixed by the
part, and the horn has to be turned by the wrist while held, carried 25 cm and centred over a 5.9 mm shaft to 0.8 mm).

Physics, not the instruction, defeats the natural plan. The horn lies 68 deg from the mark, 4 deg from a spline position.
Put on as picked it rests 1.5 mm proud on the tooth crests and cannot be pressed in. Turned while pressed, it drops in at the
first index it passes, which is not the mark. A horn left seated on a wrong tooth scores 0; one pulled off again and
refitted on the mark is scored by where it sits at the end.

Instruction given to the policy (24 words, goal only):

`Fit the white servo horn onto the servo's splined output shaft with its arm lined up on the engraved zero mark, then let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the demo
and cheat runs below, and to be filled for the Dream-1 run:

- inspect-robots 1.0.0
- yam-arena at commit 0059d02a1a91
- task file SHA-256, first 12 hex: b9cbff6a8bba (`sha256sum task.py | cut -c1-12`)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the servo as a PhysX articulation (a fixed case with a spring-loaded seat ring on a prismatic
joint) and the horn as a compound rigid body, and calls the task's per-step record after every control step. No fork of
yam_arena and no third-party assets: every body is built from box and cylinder colliders in `task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

21 tests feed synthetic records and poses to the scorer: a demo-shaped trace scores 1.0; nothing is 0; a horn left
seated on index 1 is 0; a horn seated on index 1, pulled off and refitted on the mark scores 1.0; one pulled off and
dropped, or refitted on another tooth, is not 1.0; the seat window edge (1.9 vs 2.1 deg); the engagement window (2.0 deg
around each of the 25 indices, bore offset 0.8 mm, tilt 4 deg); the horn left on the crests at 68 deg scores 0; the 0.25
rung needs a hold and the mark within 3 deg; the end limits (gap 0.5 mm, tilt 2 deg, offset 0.8 mm); the release gate
(30 vs 29 mm, 5 mm/s, 0.95 s vs 1.05 s); the horn still for the last second; a horn unseated again after seating loses
the seated rungs; the record is clean after a reset; measure() on real 4x4 poses.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the white servo horn onto the servo's splined output shaft with its arm lined up on the engraved zero mark, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/servo-horn-zero-spline-dream
```

Scoring is automatic: `task_score` is 1.0 when `success()` holds for 1.0 s and the episode ends with
`termination_reason="success"`; otherwise it is the task file's `score()` at the end (`SCORE_OVER = "end"`). With
`object_jitter=0` the seed changes nothing in the scene.

Dream-1 run: score 0.0 in 1200 steps (5 min wall clock), log adhoc_4e08b11d.json. If it is not 1.0: max_steps.

```bash
inspect-robots inspect adhoc_4e08b11d.json
```

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                          # scripted demo
python linkdemo.py task.py cheat_out --variant as_picked     # a cheat variant (or DEMO_VARIANT=as_picked)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json`. The script reads only what a camera could measure (the horn's pose and
the robot's own tool points); it never reads the seat ring, joint values or the record.

1. Right arm: top-down on the hub (pads across its flats, 14 mm), lifts, and carries the horn over the shaft while
   turning the wrist until the seen arm lies on the mark, correcting position and azimuth from the seen horn pose.
2. Lowers it onto the crests (0.4 mm), lets go, backs up, closes the fingers, centres the pads over the hub from the
   seen tool point and presses 22 mm past contact; the seat ring travels 1.5 mm and the horn is flush. Backs up 80 mm.

The scripted demo does not pull a wrong-tooth horn off again. In the simulator the horn is lifted off the ring by the
hub alone (nothing else holds it); the ring's brake is released when the hub rises 0.8 mm. That path is covered by the
offline tests but was not run in the simulator (a two-pass demo does not fit 1,200 steps at demo speed).

The demo scores 1.0 (success at step 649 of 1,200). It ends with the horn flush (gap 0.11 mm), the arm 0.7 deg from the
mark, bore 0.1 mm off the shaft, and both grippers clear and still for 1.9 s. Worst IK error 0.1 mm and 0.02 deg.

Variants (`DEMO_VARIANT`), each run in Isaac Sim with the task file above; every one scores exactly 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `nothing` | Both arms stay at the ready pose. | The horn stays on the table. | 0.0 |
| `as_picked` | Carries the horn over the shaft at the azimuth it was picked up (68 deg), lets go and presses. | Rests on the crests 1.47 mm proud, 4.3 deg from index 5; the press only moves the ring 0.03 mm; never seated. | 0.0 |
| `wiggle` | As picked, then pushes down while turning the wrist back and forth. | Drops in at index 4 (57.8 deg) and stays: seated on a wrong tooth at the end. | 0.0 |
| `off_by_one` | Aligned to the next index (14.4 deg) and pressed; never pulled off. | Seats flush at index 1 (yaw 14.0 deg) and stays: wrong tooth at the end. | 0.0 |
| `drop` | Aligned on the mark, released 30 mm above the crests. | Lands tilted 98 deg and off the shaft, never held on the crests. | 0.0 |

`wiggle` and `off_by_one` are the wrong-tooth end states that must stay at 0 now that only the final state counts (the
earlier scorer zeroed any wrong-tooth seating at any time of the run). The task is one-armed (the right arm does
everything), so there is no `left_only` or `right_only` variant: `right_only` is the demo itself, and a left arm left
at home is `nothing`.

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step the record (`REC`, a `linkkit.History`) measures the horn in the servo frame: bore offset from the
shaft (mm), hub-bottom height over the boss, tilt, arm azimuth a from the zero mark (deg), the signed distance to the
nearest of the 25 indices (every 14.4 deg), whether a gripper holds it (pad within 4 mm, opening 30 mm or less), the
horn's speed and spin from its pose change between control steps, and each gripper's clearance and speed. The record is
cleared only by the simulator reset. The score reads the final state:

- 0: at the end the horn is seated (ring travel 1.3 mm of 1.5) with the arm more than 2.0 deg from the mark, which
  is a wrong tooth. A wrong-tooth seating earlier in the run that was pulled off and refitted costs nothing.
- 0.25: held on the crests (bore within 1.0 mm, tilt 4 deg or less, height 2.5 mm or less) with the arm within 3.0 deg
  of the mark, at some time.
- 0.5: at the end seated at index 0 (arm within 2.0 deg, bore within 0.8 mm, tilt 4 deg or less).
- 0.75: at the end flush: gap 0.5 mm or less, tilt 2 deg or less.
- 1.0: also the horn under 2 mm/s and 2 deg/s, and every pad point of both grippers at least 30 mm from the horn and
  servo with both grippers under 5 mm/s, for the last 1 s.

Real trial: a top photo with a ruler for the arm against the engraved mark (2.0 deg is 1.0 mm sideways at the arm tip,
14.4 deg is 7 mm); a 0.5 mm feeler gauge that does not pass under the hub for flush seating (a 1.5 mm feeler passes when
it is on the crests); video frames for the hold and the 1 s release.

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 649.
- Runtime on one L40S: about 3 steps/s, about 4 min per demo run plus start-up.
- Seeds: `--seed 0`; `-E object_jitter=0` fixes the layout. The demo is deterministic.

## Materials

- Servo: TowerPro MG996R (40.7 x 19.7 x 36 mm case, 55 g, black), bolted to the bench (fixed), 25-tooth output shaft
  5.9 mm across, 5 mm proud of an 11.5 mm boss 3 mm tall, 10 mm from the -x end, at (0.250, -0.050); a 1.2 x 24 mm white
  engraved zero mark along +x on the case top.
- Horn: the white nylon single-arm horn from the MG996R's own horn set (25T bore), 4 g, a 14 mm x 7 mm hub (bore
  6.8 mm), a 28 mm arm 2 mm thick; friction 0.4 / 0.3; lies at (0.220, -0.220), arm 68 deg from +x.
- Check of this pair against the stated numbers: 360 / 25 = 14.4 deg is exact for a 25T spline, so the seat positions
  hold. The 1.5 mm crest-on-crest stand-off and the 3.6 N seating force are not in the servo's datasheet and were not
  measured on a real part; they are set values. Before the first real trial, on the chosen servo and horn: crest on
  crest (arm turned half a tooth, 7.2 deg, from a seat) measure the stand-off with feeler gauges, then press the
  seated horn with a kitchen scale and note the force at which it goes flush. If the stand-off differs by more than
  0.5 mm or the force by more than 2 N, the scene parameters (`TIP`, `SEAT_SPRING`) are changed to the measured values.
  Known difference: a real spline is an interference fit and probably needs more than 3.6 N, and pulling a seated
  horn off needs a puller; the simulator models neither, and the pull-off is just a lift.

## Reset between trials

Nothing is consumed. Pull the horn off (by hand, a servo-horn puller or a 4 mm bolt through the hub), lay it flat at the
taped outline with its arm 68 deg from +x, and check with the 1.5 mm feeler gauge that a crest-on-crest horn stands
proud again. The servo stays bolted.

## Licence

MIT. Copyright (c) 2026 the task authors. `task.py`, `linkkit.py`, `linkdemo.py` and `tests/test_scoring.py` are our
own code (SPDX header in each file). They use only public Isaac Lab and USD Physics techniques, copy no code from other
tasks in this repository, and the scene uses no third-party assets.

## Revision

Review TK-3RLQU3 (not accepted), two points:

1. Scoring: a horn seated on the wrong tooth zeroed the whole run even if removed and correctly refitted without damage.
   Changed: the wrong-tooth gate that read the whole run is gone. The score reads the final state, so a horn pulled off
   and refitted on the mark scores by where it sits at the end (1.0 when still and released), and a horn left on a wrong
   tooth at the end scores 0. The simulator lets the horn come off (the ring brake releases when the hub rises 0.8 mm).
   Cheats `wiggle` and `off_by_one` end on a wrong tooth and score 0; offline tests cover the refit.
2. Specific servo and horn checked against the stand-off and seating force: the pair is now the MG996R with its own
   single-arm horn. The Materials section records the check (spline geometry holds; stand-off and force are set values
   to be measured with a feeler gauge and kitchen scale before the real trial) and the fidelity note lists the
   differences (real press fit needs more force, no pull-off force modelled).
