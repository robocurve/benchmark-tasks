# Assemble a rack-and-pinion parallel gripper so both fingers open evenly

The robot assembles a parallel-jaw gripper: two racks run on opposite sides of one pinion, and the loose far rack must
be seated in its channel at the same tooth phase as the near rack, so that the two finger plates open and close
evenly about the housing centre. Seated one tooth off, the jaws sit 4.7 mm off-centre for good. Mechatronics
technicians and robot assemblers (O*NET 17-3024.00) rebuild and repair these grippers for robot arms, and a gripper
whose jaws are off-centre biases every grasp a robot then learns on it.

The ability is lowering a toothed part onto a free pinion at the right phase, with the other rack closed first. The
near rack starts one tooth open and the pinion has turned with it. Lowered at the closed position over the open near
rack, the far rack would mesh one tooth off; lowered anywhere else its teeth rest on the pin tips. The robot has to
close the near rack to its stop first, then lower the far rack between the pins, then pull the jaws open to show that
the two racks move together.

Instruction given to the policy (22 words; it states the goal only, never the order or the tooth rule):

`Assemble the gripper: seat the loose white rack on the pinion so both fingers open evenly about the centre, then let go.`

Files in this folder: `task.json`, `task.py` (scene, whole-run record, scoring and the scripted demo), `linkkit.py`
(articulated rigid-link bodies for yam_arena), `linkdemo.py` (closed-loop scripted-demo runner),
`tests/test_scoring.py` (scorer edge cases), `README.md`.

## Requirements

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, Isaac Lab, one L40S). Versions used for the
demo and cheat runs below:

- inspect-robots 1.0.0 (`inspect-robots --version`)
- yam-arena at commit 0059d02a1a91 (`git -C ~/yam-isaac-lab-arena rev-parse --short=12 HEAD`)
- task file SHA-256, first 12 hex: 4a1d59fd43df (`sha256sum task.py | cut -c1-12`)

Before every shell: `source ~/IsaacLab-Arena/yam.env` (on our VM, `/data/IsaacLab-Arena/yam.env`).

`linkkit.py` and `linkdemo.py` must sit next to `task.py`. Importing `linkkit` installs an import hook that teaches
`yam_arena.scene` to spawn the housing and pinion as a PhysX articulation (a fixed housing with one free revolute
pinion, USD joints, spawned through Isaac Lab's `ArticulationCfg`), the two racks and the tray as compound rigid bodies,
and calls the task's per-step record after every control step. No fork of yam_arena and no third-party assets: every
body is built from box, cylinder and capsule colliders in `task.py`.

## Offline check, no simulator

```bash
python -m pytest -q tests
```

31 tests feed synthetic records to the scorer: a demo-shaped trace scores 1.0; a far rack lying on the tray scores 0;
the far rack's seated limits (centre height 1.5 vs 1.6 mm, sideways 1.9 vs 2.1 mm, tilt 3.0 vs 3.1 deg, sunk 1.6 mm
below the floor); the same three limits for the near rack (a near rack riding 1.6 mm up out of its channel scores 0);
a near rack lifted out and set 48 mm beside the housing with the far rack seated and both jaws reading 20 mm scores 0;
independently positioned racks (each dragged by its own hand, a pad within 5 mm of the far rack) score 0; racks moved
one after the other score 0; the coupling threshold (18 mm of coupled opening passes, 10 mm scores 0); coupling counted
only while both racks are seated; the symmetry rung (3.0 mm passes the open rungs, 3.2 mm scores 0.25); a tooth-off mesh
pulled open scores 0.25 and a tooth-off try that is undone costs nothing; opening 15.0 vs 14.9 mm each; closed jaws
score 0.5; one pad at 29 mm restarts the release timer and 30 mm does not; 0.9 s of release is not enough; a hand at
6 mm/s or racks at 10 mm/s near the end score 0.75; a rack off the bench at the end is not seated; and the record is
clean after a reset.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Assemble the gripper: seat the loose white rack on the pinion so both fingers open evenly about the centre, then let go." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/gripper-rack-pinion-mesh-dream
```

Scoring is automatic. `task_score` reports 1.0 when the task file's `success()` holds for 1.0 s and the episode ends
with `termination_reason="success"`; otherwise it is the task file's `score()` at the end of the episode
(`SCORE_OVER = "end"`). With `object_jitter=0` the seed changes nothing in the scene.

Dream-1 trial: on yam-arena at 0059d02a1a91 with task file 4a1d59fd43df, the run took 1200 control steps
and 4 minutes of wall clock, log `adhoc_c23125b9.json`; `task_score` 0.0, ended because max_steps.

## Scripted demo and cheat variants

```bash
python linkdemo.py task.py demo_out                          # scripted demo
python linkdemo.py task.py cheat_out --variant left_only    # a cheat variant (or DEMO_VARIANT=left_only)
```

`linkdemo.py` runs `demo_script()` from `task.py` closed loop at 20 Hz and writes `top_cam.mp4`, `left_cam.mp4`,
`right_cam.mp4`, `setup.png` and `result.json`. The script reads only what a camera could measure (the racks' poses)
and the robot's own grippers; it never reads joint values, the pinion angle or the record. Because the arms sag a few
millimetres, it corrects each grasp and each placement from the seen pose.

1. Right arm: pinches the near rack's handle post from above (pads closing along y, opened only 30 mm on the
   approach), pushes the rack 9.4 mm to its stop block (the pinion turns with it), lets go and lifts clear.
   Left arm, at the same time: pinches the far rack's handle post on the tray, lifts it 50 mm and waits.
2. Left arm, once the near rack is closed: carries the far rack over its channel 20 mm above its seat, servoes the
   seen rack position onto the goal to 0.5 mm, drops to 14 mm above the seat (the teeth 2 mm over the pin tips), and
   lowers in three steps (9.5, 5 and 0.5 mm above the seat), correcting x and y at each, then opens its pads, waits and
   lifts away. The rack finds the gap between the pins and settles on the floor.
3. Right arm alone: pinches the near rack's post again, pulls it 27 mm in the opening direction (the pinion carries the
   far rack the other way while the left gripper is 100 mm or more from it), lets go and lifts clear. Both wait.

The demo scores 1.0 (`success`, episode ended at step 986 of 1,200): both racks seated (far rack centre 0.0 mm above
its seated height, tilt 0.0 deg; near rack 0.0 mm off its channel), 23.7 mm of coupled opening travel (the near rack
pulled by the right hand alone, no pad within 5 mm of the far rack), the two fingers 25.5 and 21.3 mm open (plate-gap
centre 2.1 mm off the centre line, inside the 3.0 mm limit; the rest is the mesh's backlash), the racks still, both
grippers over 30 mm clear. Worst IK error 0.1 mm and 0.01 deg.

Variants (`DEMO_VARIANT`), run in Isaac Sim; every one scores 0:

| Variant | What it does | Result | task_score |
|---|---|---|---|
| `near_out` | The reviewer's loophole. Right hand lifts the near rack out of its channel and sets it on the table 49 mm beside the housing, 20 mm along from its stop. Left hand seats the far rack, lets go, grips it again and drags it 20 mm open by hand. | Far rack seated and 18.2 mm open, near rack reads 20.0 mm open but lies outside the channel (x 59 mm against 108 mm). Travel and symmetry look right; the near rack is not seated, so 0. | 0.0 |
| `uncoupled` | Independently positioned racks. After the far rack is seated, the left hand takes hold of it again and each hand drags its own rack open at the same time. | Not a clean cheat in the simulator: the left hand's second grip knocked the settled far rack 8 mm out of its channel (x 124 mm against 132 mm) and the near rack never moved, so the run scores 0 on the far rack's seating, not on coupling. The coupling rule is shown by the offline tests (independent racks with a pad on the far rack, and racks moved one after the other, both score 0). | 0.0 |
| `left_only` | Right arm idle. Left arm lowers the far rack over the pinion with the near rack still one tooth (9.4 mm) open, and lets go. | The far rack does not seat (it rests on the pin tips). | 0.0 |
| `right_only` | Left arm idle. Right arm closes the near rack, then pulls it open. | The far rack stays on its tray, never seated. | 0.0 |
| `one_tooth` | Near rack closed first, but the far rack is lowered one tooth (9.4 mm) toward its opening position, then the jaws are pulled. | The far rack never seats (it ends tilted on the pin tips), so 0. | 0.0 |

The demo, `near_out`, `left_only`, `right_only` and `one_tooth` ran on the file before its last edit, which only changed
the `uncoupled` branch of the script (the demo path is unchanged; the filing step reruns the demo on the submitted file).

`near_out` and `uncoupled` end in states the earlier scorer rewarded (far rack seated, both fingers open about 20 mm,
nothing else checked); they are what the revision closes. The `nothing` variant (both arms idle) is kept in the code
and is covered by the offline test of an unseated far rack.

## Build log (about two dozen development runs, 100 to 200 s each on the shared L40S for the original task, plus the revision runs below)

| Change | Result |
|---|---|
| first build: handle at each rack's tail, pads closing along x | 0: the left wrist could not reach that orientation and the rack tilted in the grip |
| handle post on the back bar, pads closing along y | 0: at the housing's first position (x 0.17) the arms sagged 45 to 50 mm short of their reach limit (0.32 m); the housing and tray moved 50 mm toward the arms |
| pad opening 30 mm instead of 94 | the far pad had been landing on the finger plates |
| tooth 3 mm, pin 3.5 mm | the two racks jammed at 9.8 mm opening: a kinematic check showed under 0.6 mm of clearance with both racks meshed; teeth went to 2 mm and pins to 3 mm (1.6 mm clearance, 0.7 mm at 1 mm of phase error) |
| rounded pin tops; servo from the seen pose; lower in three steps | the far rack seats and the jaws open evenly |
| left arm retreats clear of the right; pull 27 mm; shorter moves | 1.0 at step 986 |

Revision runs, after the review: the near rack's own seating test and the coupling test added to the scorer, the whole-run
gates removed (final state only), and the `near_out` and `uncoupled` cheats added. Three `uncoupled` scripts were tried (re-grip with
a lift, keep hold of the rack, re-grip after it settles); in each the second grip on the seated far rack disturbed it,
so none ended in the old full-score state.

## Scoring (`score()`, at the end of the episode; `success()` must hold 1 s)

Every control step the on-step record (`Record` in `task.py`, a `linkkit.History`) measures the near and far racks'
travel from their stops (s_N, s_F, mm), the far rack's pitch-line position, height, tilt and yaw, the racks' speeds
(from the pose change between control steps) and each gripper's pad clearance to the housing and racks. It is cleared
only by the simulator reset. d = (s_N - s_F) / 2 is where the middle of the two finger plates sits relative to the
housing centre line.

- Seated, far rack: pitch line within 2 mm of x 0.132, centre within 1.5 mm of its floor-seated height (and not more
  than 1.5 mm below it), within 3 deg of level and 4 deg of its channel direction.
- Seated, near rack: the same test against its own channel (pitch line x 0.108): within 2 mm, height within 1.5 mm,
  within 3 deg of level and 4 deg of its channel direction. A near rack lifted out and set beside the housing fails it.
- Coupled: while both racks are seated and no pad point of either gripper is within 5 mm of the far rack, the opening
  travel in which both racks moved open together (the smaller of the two openings over a 4-step window, summed) must
  reach 15 mm over the run. This is what shows that pulling one rack drives the other in the opposite direction.
- 0: either rack not seated, or the coupled travel under 15 mm. The whole run is judged by its final state: a wrong
  first try that is undone (a tooth-off mesh, a rack knocked out and put back) costs nothing.
- 0.25: both seated and coupled, |d| above 3.0 mm.
- 0.5: |d| at most 3.0 mm, the jaws not yet 15 mm open each.
- 0.75: |d| at most 3.0 mm, and both s_N and s_F at least 15 mm.
- 1.0: as 0.75, the racks still (under 3 mm/s) for the last 1 s, and the release gate: every pad point of both
  grippers at least 30 mm from the housing and racks and both grippers moving under 5 mm/s for the last 1 s.

`score()` returns the rung, `success()` is true on the 1.0 rung (yam_arena ends the episode once it has held for
`SUCCESS_HOLD_S = 1.0` s), and `distance()` is 1 - score.

Real trial, what is measured and with what. A 1 mm tape scale is stuck along each finger plate's path on the floor
plate, with the housing centre line marked on the plate: the top camera reads s_N and s_F from the plates' inner
edges, and d follows. Seating is checked with a straightedge laid across the pinion's top: the far rack's top must be
flush with it within 1.5 mm, and the pitch-line position with the wall marks. The near rack is checked the same way
(straightedge flush within 1.5 mm, wall feeler gaps). Coupling is read from the video: in the pull, only the near
rack's handle post is touched, the far rack's top stays in view free of any hand, and both plates advance together on
the two tape scales (15 mm or more). The release gate is a 30 mm marker block held beside each pad in the last second of video.

## Parts and how to verify each fit

- Housing: 52 x 160 x 4 mm ABS floor plate, two 3.5 x 150 x 12 mm walls glued on with their inner faces 45.0 mm apart,
  two 4 x 4 x 12 mm steel stop blocks glued so that a closed rack's finger plate reads about 1 mm on the scale. Check: a 45.0 mm gauge block
  slides between the walls.
- Racks: 11.75 mm wide, so each runs with 0.75 mm clearance to its wall; a 0.5 mm feeler passes and a 1.0 mm feeler
  does not. Check also that each rack slides under 20 gf on a spring scale (sim: 0.25 friction, 60 g).
- Pinion: 14 mm hub with eight 3 mm dowel pins on a 24 mm pitch circle, free on a vertical shaft; check that it turns
  under 3 gf pulled at a 100 mm arm (sim: 0.0015 N m).
- Mesh: with the near rack on its stop, a pin sits centred in a tooth gap of the near rack (7.4 mm gap, 3 mm pin); the
  far rack's gaps line up with the opposite pin when lowered at the closed position. Check with a 3 mm gauge pin.

## Materials, masses and licences

Housing: grey ABS, 1.0 kg. Pinion: stainless steel, 40 g. Racks: white powder-coated steel, 60 g each. Feed tray:
dark grey ABS, 0.3 kg. Masses are the simulator's values. No third-party assets; `task.py`, `linkkit.py` and
`linkdemo.py` are MIT-licensed (Copyright (c) 2026 the task authors).

## Run limit, runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (60 s simulated), `--max-steps 1200`. The demo succeeds at step 986.
- Runtime on one L40S: a demo run took about 3 to 4 minutes wall clock (3 to 10 control steps/s while another
  simulation shared the GPU, plus about 1 to 2 minutes of Isaac Sim start-up).
- Seeds: `--seed 0`. `-E object_jitter=0` fixes the layout. The scripted demo is deterministic (two runs of the final
  logic ended in the same state to 0.01 mm).

## Submitted videos and scores

- Demo: `python linkdemo.py task.py demo_out`, `demo_out/result.json`: `task_score = 1.0`, success at step 986. The
  video is `demo_out/top_cam.mp4` (with `left_cam.mp4` and `right_cam.mp4`) and the screenshot `demo_out/setup.png`.
- Dream-1 run: `adhoc_c23125b9.json`, `task_score = 0.0`, ended by `max_steps` after 1200 steps (4 min).

## Revision

Review of this task (not accepted), each point and what changed:

1. Construct validity, "a robot can remove the near rack and position it alongside the housing while placing the far
   rack at matching travel": the scorer now requires the near rack to be seated in its own channel as well (pitch line
   within 2 mm, height within 1.5 mm of the floor, within 3 deg of level, 4 deg of straight). The `near_out` cheat does
   exactly this (near rack set 49 mm beside the housing, far rack seated and 18.2 mm open, near rack reading 20.0 mm)
   and scores 0 in the simulator.
2. Scoring, "checks the near rack's travel but not its seating or coupled motion": besides the seating of both racks,
   the run must contain 15 mm of opening travel in which both racks moved open together, both seated, with no gripper
   within 5 mm of the far rack, so that pulling the near rack alone is shown to drive the far rack the other way. The
   `uncoupled` cheat (each hand drags its own rack) could not be scripted into a seated, open end state in yam-arena (the
   second grip on the far rack disturbs it), so that rule is demonstrated by the offline tests; the simulator run scores 0.
3. Tests: a near rack removed from its channel (set beside the housing, riding 1.6 mm up, 2.1 mm sideways, 3.1 deg
   tilted) and independently positioned, uncoupled racks (hands on the far rack; racks moved one after the other) were
   added to `tests/test_scoring.py` (now 31 tests).
4. Also changed, from the program's later guidance: the whole-run gates (a tooth-off mesh, a rack falling off the
   bench) no longer zero the run for good; only the final state is scored, so a wrong first try that is undone costs
   nothing. `skip_close` is renamed `left_only`, `n_only` is `right_only`. The cheat table now also has `left_only` and
   `right_only`, which show what each arm alone achieves.
