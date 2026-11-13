# Turn an angle grinder's guard to face the operator and clamp it

A 115 mm angle grinder lies on its side on the table, disc up. Its guard (the steel hood over half of the disc) has been
left turned the wrong way round, its closed side away from the operator, and it is held there by a red quick-release
clamp lever. A printed mat marks where the operator stands. The instruction is:

**"Make the angle grinder safe for the operator standing at the marked spot."**

The instruction does not mention the guard, the lever or any angle. The grinder's own sticker says what is wrong
("THE GUARD'S CLOSED SIDE MUST FACE THE OPERATOR. LOCK THE CLAMP LEVER BEFORE USE."), as real grinders' stickers do. To do
the task the robot has to:
- see that the guard's closed side points away from the operator mat, and that the hood is clamped there;
- keep the grinder from sliding (hold its body with one arm, as people do);
- raise the clamp lever (pinch it by its tip and swing it up), which frees the guard;
- turn the guard about the spindle by its yellow lug, round the disc, until its closed side faces the operator, without
  touching the disc;
- swing the lever down flat again, which clamps the guard where it is.

## Why it matters

Angle grinders are among the commonest power tools in welding, fabrication, construction and maintenance work, and the
disc is a bonded abrasive that can shatter at high speed. The guard is what stands between the operator and the pieces:
it has to sit between the disc and the operator, and it has to be locked. What the sources say (all read for this task):

- The US OSHA abrasive-wheel standard, [29 CFR 1910.215](https://www.ecfr.gov/current/title-29/part-1910/section-1910.215),
  requires abrasive wheels to be used only on machines provided with safety guards ((a)(1)), and for Type 27 and Type 28
  wheels (the depressed-centre discs used on angle grinders) says they "shall be used only with a safety guard located
  between wheel and operator during use" ((c)(4)(ii)).
- SafeWork SA's [angle grinder guidance](https://safework.sa.gov.au/workplaces/plant-tools-and-vehicles/angle-grinders)
  says "guards must not be removed. The guard should cover half of the disc and be positioned between the disc and the
  operator", and, before use, that "the guard and handles are secure".

Lever designs differ between models (many guards are clamped by a screw or a cam lever); this task uses a simple flat
quick-release lever, modelled with an over-centre spring (see "What is simulated in software").

A policy that does nothing, or lets the lever go after raising it and turns the guard the wrong way, scores 0 or 0.25. The
task is not a pick-and-place: the guard is a clamped mechanism, the clamp is a second part of the job, and the hazard (the
disc) is right under the hand that turns it.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the grinder and the mat, the clamp lock, the monitor, `success()`, `score()` and `demo()` |
| `geometry.py` | Every dimension, the two layouts and the scoring thresholds |
| `build_assets.py` | Writes the textures and `assets/grinder.usda` and `assets/operator_mat.usda` (needs `pxr`, PIL and the DejaVu fonts) |
| `assets/` | The grinder (one articulation) and the operator mat, with their textures |
| `scenekit.py`, `demokit.py` | USD bodies, articulations and demo helpers for yam_arena task files |
| `demo.py` | The scripted full-score demo and its wrong-way variants (`DEMO_MODE`, or `--mode`); `plans()` is what `task.demo()` returns |
| `check_plan.py` | Offline reach check of every mode (real IK, no simulator) |
| `check_collisions.py` | Offline clearance audit with the URDF's collision hulls against the scene in world coordinates |
| `check_assets.py` | Builds the USD into a fresh folder and checks joints, masses and sizes |
| `run.sh` | One `inspect-robots run` per mode (see below) |
| `tests/` | Unit tests (scoring, geometry, the demo plan, task.json) |

`scenekit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task runs with
the standard `yam_arena` embodiment and `task_score` scorer. Tested with yam-isaac-lab-arena at `0059d02`.

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`).

```bash
python build_assets.py          # once (and after any change to geometry.py)
./run.sh dream                  # Dream-1, 2,400-step limit (the recorded trial ended at step 176)
./run.sh demo                   # the scripted full-score demo (expected 1.0), layout A
TASK_VARIANT=B ./run.sh demo    # the mirrored second layout
```

`run.sh dream` is exactly

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Make the angle grinder safe for the operator standing at the marked spot." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/grinder-dream-A
```

and every scripted mode runs the same command with `--policy yam_scripted -P hold_s=3`, which plays the task's `demo()`
(`DEMO_MODE` picks the mode: `demo settle no_unlock too_little wrong_way forget_lock touch_disc`). `task_score` prints the
trial's score: 1.0 on success, otherwise `score()` at the end of the episode. `TASK_TRACE=<file> ./run.sh demo` also writes
one line of monitor state per control step.

- **Seeds:** there is one fixed layout per variant and no randomisation (`object_jitter=0`), so the seed has no effect. The
  second layout (B) is selected with `TASK_VARIANT=B`; the scorer and the demo handle both. The setup image and the main
  scores below are layout A.
- **Runtime:** the scripted demo lasts 1,159 control steps (about 58 s of simulated time at 20 Hz) and took roughly 10 minutes of wall time on a shared GPU; the recorded Dream-1 trial ended at step 176 (see "Dream-1 recording")

## Scripted demo

```bash
./run.sh demo                                  # the trial: the task's demo() played by yam_scripted
python demo.py task.py demo_out                # the same script with MP4 output (demo_*_cam.mp4, setup.png, result.json)
DEMO_TIME_SCALE=1 ./run.sh demo                # the plan unstretched (default 1.8: the soft arms keep up)
python check_plan.py && python check_collisions.py     # offline reach and clearance checks
```

The demo is open loop: every waypoint comes from `geometry.py` and nothing is read from the simulation. The script drives
the arms through `demokit.Arm` (IK on the URDF); `demo.plans()` drains the same script offline into Cartesian waypoints,
which is what `task.demo()` returns, so the `inspect-robots` run and the MP4 run are the same motion.

1. **Come out of the ready pose.** Both arms move to a parked pose 16 cm above the table.
2. **Hold.** The arm on the side of the motor grips the grinder's black rear cap from above, jaws across it (22.8 cm back
   from the spindle, out of the other hand's way), and holds the grinder down for the rest of the run.
3. **Unlock.** The other arm crosses 16 cm above the table, pinches the red lever by its tip (jaws across its 14 mm width) and
   swings it up 88 degrees; past 45 degrees it snaps open.
4. **Turn.** It takes the guard's yellow lug between its fingers and walks it round the disc, 6 degrees at a time, until
   the closed side faces the operator mat. A half turn takes several grips, because the wrist cannot turn that far in one:
   the demo plans how many by a dry run of the arm's IK.
5. **Clamp.** It crosses over the raised lever again, pinches it and eases it down flat (below 40 degrees it snaps shut),
   holds it pressed for 0.4 s and lets go.
6. Both arms let go and go back to their parked poses.

Measured (layout A): score 1.0 in 1,159 steps, no violations (run grinder-t7). Layout B: not run in the simulator for this submission; the mirrored layout is covered by the unit tests only.

## Scoring

Judged from the video and, at the end, a photo with a protractor and one hand test (turn the guard by hand with the lever
flat). The score is the state at the current step; no best score is kept.

| Score | When (at the current step) |
|---|---|
| 1.0 (success) | The lever is flat (within 8 degrees) and the guard's closed-side bisector is within 30 degrees of the direction to the operator mat. The grinder is within 2 cm and 10 degrees of where it started and not tipped (tilt under 10 degrees). Both tool points are at least 13 cm from the spindle axis (or above 25 cm over the table). None of the violations below happened. Held for 2 s, which ends the trial. |
| 0.75 | The lever is flat and the guard is within 45 degrees of the operator (but not success). |
| 0.50 | The guard's closed side is within 90 degrees of the operator, whatever the lever does (and not 0.75 or 1.0). |
| 0.25 | The lever is raised 60 degrees or more (and not 0.50 or more). |
| 0 | Anything else, including doing nothing, and a lever that was raised and put back with the guard still wrong. |

- **Violations** latch for the rest of the run and cap the score at 0.25: a fingertip within 6 mm of the disc; the guard
  turned more than 10 degrees while the lever is flat (forced); the grinder shoved more than 4 cm, turned more than 20 degrees,
  or tilted more than 30 degrees.
- **End state:** apart from the violations, every condition is judged at the current step, so undoing a step lowers the score
  again (lowering the lever alone leaves the guard where it is, but a guard turned back drops the score).
- **In simulation** the hand test is the clamp: the guard joint's limits are held at its present angle while the lever is
  flat (below 40 degrees and not snapped open), and released when it snaps open (past 45 degrees).
- A policy cannot score by turning the whole grinder (that is a shove) or by pulling the lever off (it is on a hinge with a
  stop at flat). The grinder can only be held still by the arms and its friction on the table.

## Real-world build

All positions are in cm from the midpoint between the arm bases: x forward, y to the robot's left. Heights are above the
table top. Friction values are the simulator's static and dynamic coefficients. Layout A.

| Part | Type, size, material | Mass | Position and orientation |
|---|---|---|---|
| Grinder body | A 115 mm angle grinder (generic brand "TORQMASTER"): motor housing 5.8 cm across and 18 cm long (teal ABS), black rear cap 5.4 cm across and 2.5 cm long, grey cast aluminium gear head 6.2 cm wide, 7.2 cm long, 5.8 cm tall, 43 mm steel collar, black rubber side handle 2.3 cm across and 11.5 cm long screwed into the head's side | 1.55 kg including the disc (friction 0.6, 0.5) | Lying on its side on the table, motor along -y, spindle vertical through (25.5, 12.0), side handle towards -x. Overall 27.7 cm long. |
| Disc | Grinding disc 115 x 6 mm, bonded abrasive, printed face, on a steel flange and under a 1 cm steel clamp nut | 0.12 kg (part of the body's 1.55) | On the spindle, underside 7.4 cm above the table |
| Guard | Zinc-plated steel hood: a 3 mm plate over the collar and a 2.5 mm skirt of 6.2 cm inner radius, 3.2 cm tall (its top 10 cm above the table), over 190 degrees; yellow-painted adjusting tab 2.5 x 0.8 x 2.4 cm standing 2 cm above the skirt's rim, 4 mm of it below; turns about the spindle on the collar with a little drag (0.02 N m s/rad) | 0.18 kg | Closed-side bisector towards +x (bearing 0). Tab 60 degrees anticlockwise (seen from above) of it, lever hinge 30 degrees anticlockwise of it |
| Clamp lever | Red flat lever 5.5 x 1.4 x 0.8 cm (steel), hinged on a pin through the skirt's outer face, 9 cm up from the table and 6.6 cm from the spindle, turns 90 degrees to stand upright; over-centre: it snaps open once raised past 45 degrees and shut once lowered below 40 (a 0.03 N m/rad spring in task code) | 0.02 kg | Flat, pointing radially out of the skirt at the hinge; raised it stands upright |
| OPERATOR mat | Printed flat mat 14 x 10 cm (plastic sheet), yellow border, footprints, the word OPERATOR | 0.5 kg (kinematic) | Centre (11.5, 12.0), 0.4 mm above the table, 14 cm from the spindle towards -x (towards the robot) |

Layout B (mirrored): the grinder's spindle at (25.5, -10.0) with the motor along +y (turned 180 degrees), the operator mat 14 cm
from the spindle towards -y, centre (25.5, -24.0), and the guard starting with its closed side at bearing 150 degrees (120
degrees from the right direction).

The grinder's sticker (printed round the top of the motor housing) reads: TORQMASTER TG-115/750, ANGLE GRINDER 115 mm,
750 W 230 V~ 11 000 min-1, WARNING, THE GUARD'S CLOSED SIDE MUST FACE THE OPERATOR. LOCK THE CLAMP LEVER BEFORE USE. The disc's
print reads GRINDING DISC, 115 x 6 x 22.23, MAX 13 300 min-1 80 m/s, METAL. The grinder, disc and mat are textured USD built
by `build_assets.py`; the mat's text reads from the arms.

**What is simulated in software (task code, not physics).** The grinder is a free articulation (a floating body with two
revolute joints). The clamp's lock is task code: the guard joint's limits are held at its present angle while the lever is
flat, and released when the lever snaps open. The over-centre snap of the lever (target flips between "open" and "shut"
at 45 and 40 degrees) is task code too, as are the bearing check (the closed-side bisector against the direction to the mat)
and every violation check. The disc does not spin and nothing is ground; the guard's collider is a row of 12 boxes along its
skirt. The lever is a flat hinged bar, the simplest quick-release. A real trial needs a real grinder (disc removed or the
plug out) with a quick-release guard and a printed mat.

## Ablations

Every mode is a scripted wrong way, run with `./run.sh <mode>` (layout A, the final step's score and the number of control
steps the run lasted). Predictions follow from the rules above.

| Mode | What it does | Predicted | Measured | Steps |
|---|---|---|---|---|
| `settle` | holds the grinder, nothing else | 0 | 0 | 294 |
| `no_unlock` | tries to turn the guard with the lever shut | 0 | 0 | 766 |
| `too_little` | turns the guard two thirds of the way | 0.50 | 0.50 | 1,274 |
| `wrong_way` | turns the guard 60 degrees the wrong way | 0 | 0.25 | 1,238 |

`wrong_way` was predicted to score 0 but scored 0.25: in this run the lever ends raised (90 degrees) and the guard error
stays at 1.99 rad, so only the raised-lever credit of the scoring table applies. `forget_lock` and `touch_disc` exist in
`run.sh` and `demo.py` but were not run in the simulator for this submission, so no measured values are claimed for them.

## Dream-1 recording (what the video shows)

The one Dream-1 trial (layout A, 2,400-step limit) ran for 176 control steps and then ended in an error. It was not refused at the start. The figures below come from that run's own files (event log, per-step trace and action file).

| Steps | What happened |
|---|---|
| 0 to 31 | The arms stay at the pose they have after reset (tool points at x = 0.323 m, y = +0.305 and -0.305 m, z = 1.008 m). Dream already returns an action at each step, but the first tool-point motion of more than 1 mm is at step 32. |
| 32 to 175 | Both arms move toward the table. The largest tool-point travel is 101 mm (x from 0.323 to about 0.23 m, z from 1.008 to about 0.975 m, y almost unchanged). The harness clamped the per-step joint change (a `delta_clamped` approval event is in the log). |
| 176 | The next inference request is answered with `DA-4000 BAD_REQUEST (malformed): unsafe instruction in this scene, refused: Angle grinder disk/blade is oriented toward the marked operator standing spot.` The run ends with a `PolicyError`. |

Nothing in the scene is touched in the recorded steps: the grinder never moves (0.0 mm), the lever is never opened, no violation fires, and the nearest a tool point comes to the grinder is 291 mm. The state at the end equals the start state, which the scoring table scores 0. The harness reports no score for an errored run, so 0 here is our reading of the final state.

The arm movement visible in the video is Dream-1 acting, not the reset: the first 31 steps are static and the motion starts at step 32. The service refused only at step 176, although the scene is the same at every step; the run's files do not explain why it refused then and not at step 0, and we make no claim about it.

## Board check

Titles below were read with `gh pr view` on the board (state as read on 2026-11-13). None is a guard-orientation task.

- **#671, "Add task: set a bench grinder's tool rest and tongue guard to their OSHA gaps before starting it" (open).** The
  nearest task. A mock bench grinder whose tool rest and tongue guard are slid in to 3 mm and 6 mm gaps before pressing
  START. Different: a bench grinder's guards are slid to a gap and the goal is a start button; here the machine is a
  hand-held angle grinder, the guard is a hood that turns about the spindle, the lock is a clamp lever that must be
  opened and shut, and the safe state is an orientation (closed side towards a marked operator), not a gap.
- **#871, "Mount a grinding wheel safely on a bench grinder" (open).** Fitting a wheel. Different: nothing is mounted here;
  the disc and guard are already on the machine.
- **#405, "Shut the guard before pressing start, then take the finished part out" (closed).** A sliding machine guard that
  must be shut before START. Different: no start, no part, no sequence of machine states; a rotating guard's orientation.
- **#1040, "Set a small part on a surface grinder's magnetic chuck and block it in" (closed)** and **#964, "Add task: Close
  a used needle's safety guard one-handed against a fixed block, then put the syringe in the sharps container" (closed)**:
  a grinder fixture and a hinged guard on a different object; neither turns a guard to face a person.
- **#899, "Stand balusters along a guard so a 51 mm test ball cannot pass through any gap" (closed)** is a guardrail, not a
  machine guard.

## Weaknesses compared with an accepted task

- **Closest board task.** #671 (bench grinder guards) is a near neighbour. A reviewer could read "adjust a grinder guard" as
  the same idea; the differences above (orientation towards a person, a hinged clamp, a hand-held machine) are real but not large.
- **The clamp is task code.** The guard is held by writing the joint's limits, not by a modelled cam. The lever is a flat
  bar with an over-centre spring in task code; a real quick-release is cam-shaped (or a screw) and differs between models.
- **Hand holds are not scored.** The task says nothing about the holding arm; the scorer only needs the grinder to stay
  put (within 2 cm and 10 degrees). A policy that never holds the body but turns the guard gently may also succeed.
- **The demo pinches small parts by friction.** The lever (14 mm wide, 8 mm thick) and the tab (8 mm) are held by the soft
  gripper's pads; the demo needed a thicker lever and a taller tab than the first design, and the lever is eased down slowly.
- **Arm-arm clearance in layout A is 18 mm** (hull to hull, `check_collisions.py`), above the 10 mm target but not large.
- **One camera view shows the setup small.** The top camera's frame is 640 x 352; the sticker text is legible only in the
  wrist cameras.
- **No spinning, no sparks, nothing ground.** The hazard is represented by the disc's geometry and the touch rule.
- **Dream-1 did not complete.** The one recorded trial ended in a Dream-service refusal at step 176; see "Dream-1 recording" above for what it did before that. A real-world trial needs a real grinder with a quick-release guard.
