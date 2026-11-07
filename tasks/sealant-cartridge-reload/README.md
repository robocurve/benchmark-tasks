# Replace a cartridge in a ratchet-rod dispenser

Exchange a spent cartridge for a prepared replacement in an unpowered bench tool. The rod has physical sawteeth and a passive spring pawl. Rotation clears those teeth before retraction; the piston must clear the cartridge before it can be lifted out. The free tool rests in a locating cradle; the reference does not hold its base with either hand. Only robot joints and grippers are actuated.

**Native demonstration: score 1.0 in 1,090 actions (54.50 simulated seconds).** Right unlocks and retracts the rod using the cradle for tool support; left only exchanges the refills and parks; right restores the ratchet, pauses, then releases and withdraws. Neither initial nor final left-hand base hold remains. The registered Inspect Robots runner also reached `task_score: 1.0` in 1,089 actions. The rod stays extended behind the fresh refill while its ratchet is engaged. The single hosted Dream-1 trial completed 2,400 actions with `task_score: 0.0`, normal action-limit termination and zero trial errors. All three current evaluations use unchanged executable source. The submission pairs the model attempt with the successful scripted demonstration.

## Instruction and relevance

“Replace the used orange cartridge with the prepared blue cartridge. Leave the used cartridge in the collection tray and the dispenser resting in its cradle, with the replacement seated and the piston rod re-engaged at its rear. The rod handle may remain extended behind the dispenser. Release both hands.”

The proposed application is consumable replenishment at an assembly or repair bench. [Newborn's manufacturer instructions](https://newborncaulkguns.com/pages/resources) describe rotating a ratchet rod to disengage it before retracting it and removing a cartridge. They distinguish this mechanism from thumb-release and drip-free tools. This task uses the rotating ratchet mechanism only. The apparatus is an original inert dispenser fixture built from primitive shapes, with a reusable outlet on the tool and plain cylindrical refills. Robot deployment for this particular replenishment operation has not been demonstrated.

The task differs from pulling a loose part or releasing an isolated latch: the rod must disengage, retract clear of the old cartridge, accommodate a replacement and return to its working orientation. The score judges observable outcomes and permits any physically valid arm assignment, regrasp or use of the cradle. A particular failed pull or prescribed action sequence is not required.

## Rod extension and ratchet engagement

The rod's translation positions the piston; its rotation disengages or re-engages the spring pawl and teeth. With a fresh refill, the piston sits at its rear and the T-handle remains farther behind the tool than it did with the empty refill. That extension is expected. The final state must still have the rod returned to its working orientation, the pawl returned and the piston within the scored distance of the refill's rear face.

The reference rotates the rod back, pauses for one second in that working orientation, then releases and lifts clear. The left hand does not hold the base during unlocking or relocking. Any other physically valid strategy still counts; the scorer does not prescribe a hand assignment.

## Environment and commands

Use the authenticated [Robocurve simulation walkthrough](https://tasks.robocurve.org/walkthrough). The public adapter revision in `pyproject.toml` is `yam-isaac-lab-arena@bd62771e60320433893bd1d2df0fc79b5ddcd6d1`, with Arena `release/0.3.1`, Isaac Sim 6.1 and Python 3.12. With those dependencies installed and the simulator environment activated:

```bash
uv pip install --no-deps --no-build-isolation -e tasks/sealant-cartridge-reload
inspect-robots list embodiments
python -m pytest tasks/sealant-cartridge-reload/tests -q
python tasks/sealant-cartridge-reload/check_assets.py --output /path/to/fresh/usd-check
python tasks/sealant-cartridge-reload/check_plan.py --mode demo
python tasks/sealant-cartridge-reload/run.py --policy hold --max-steps 100 --output /path/to/fresh/hold
python tasks/sealant-cartridge-reload/run.py --policy engaged_pull --output /path/to/fresh/engaged-pull
python tasks/sealant-cartridge-reload/run.py --policy release --output /path/to/fresh/release
python tasks/sealant-cartridge-reload/run.py --policy demo --output /path/to/fresh/demo
```

The operator must accept NVIDIA's license and configure its acceptance flag before launch. This package does not accept a license automatically. Each output directory must be new. `run.py` cannot call Dream-1; it executes only local scripted robot targets. `check_plan.py` checks offline URDF kinematics and is not physical feasibility evidence.

The registered embodiment `sealant_reload` uses the normal fourteen YAM joint/gripper targets and top plus two wrist RGB images. Scoring diagnostics are privileged; no object state is added to policy observations. There is one fixed layout, no object jitter and no change with seed. The reference controller uses fixed Cartesian waypoints converted to ordinary joint targets; it cannot write apparatus poses, joints or collision settings during a trial.

For a hosted model reproduction, which spends Dream credit, use the standard CLI command below with `--policy dream -P model=dreamlabs/dream-1-beta -P rows_per_request=48 -P record=false` in place of `--policy sealant_reload_scripted`, add `--epochs 1`, and choose a fresh log directory. Keep the exact instruction, seed, camera sizes, scorer and 2,400-action limit. The submitted model recording is one actual trial; no second trial was started. One expired request was retried within that episode. All returned predicted rows were within the declared action limits.

## Setup and reset

Coordinates are metres, x forward, y left and z up. Native quaternions are XYZW. The standard YAM table is 1,100 × 1,300 × 60 mm, centred at `(0.36, 0, 0.7175)`, with top z = 0.7475 m. Arm bases are `(0, ±0.305, 0.75)` m. Both arms start at `[0, 1, 1, -0.6, 0, 0]` rad with open grippers. The calibrated observation rig and lighting are unmodified.

[scene-inventory.json](scene-inventory.json) enumerates every task solid, body mass and joint. Box dimensions there are **half extents**; cylinder lengths are full lengths. Prismatic limits use metres and revolute limits use degrees as authored in USD. [setup-layout.json](setup-layout.json) gives spawn poses and the complete standard station inventory, including fixed camera support, floor and nominal materials. Fixture-solid coordinates are world coordinates; moving-solid coordinates are local to their named rigid body.

| Object | Dimensions, nominal material and placement |
| --- | --- |
| Free dispenser | Approximately 62 × 333 × 141 mm at reset; 494 g including its internal carrier. Steel rail frame, shaft, sawteeth and spring pawl; polymer stationary and rod grips; aluminium piston. Frame origin `(0.32, -0.03, 0.88)` m; its axis points +y. All joints start at zero. |
| Used cartridge | Orange rigid polymer cylindrical refill, 49 mm maximum diameter, 200 mm overall length with flush front face and no projecting tip; 45 g. Spawn centre `(0.32, 0.082, 0.88)` m, axis +y, open rear. Two 50 × 18 × 5.5 mm steel support bridges on the dispenser, at local y = 55/180 mm, support the refill coaxially with the piston. Their tops are 24.5 mm below the rod axis. |
| Prepared replacement | Blue cylindrical refill of the same outer geometry, with a fixed rear practice piston; 160 g. Spawn centre `(0.18, 0.40, 0.7725)` m, axis +y. It rests between shallow anti-roll guides on the table. |
| Tool cradle | Fixed rigid polymer foot, 112 × 210 × 6 mm, centre `(0.32, 0.0875, 0.7505)` m; four 12 × 18 × 90.5 mm posts support two 80 × 18 × 12 mm crossbars at y = 0.025 and 0.150 m, top z = 0.850 m. Four visible 8 × 24 × 18 mm locating edges at x = 284/356 mm, y = 25/150 mm and z = 853 mm leave 1 mm lateral clearance from the rails. The tool can lift out freely; these edges do not actuate the ratchet. |
| Spare guides | Two fixed rigid polymer rails, each 8 × 220 × 12 mm; centres `(0.15, 0.40, 0.7535)` and `(0.21, 0.40, 0.7535)` m. |
| Collection tray | Fixed rigid polymer tray, 118 × 288 × 20 mm overall, centred at x = 0.43, y = 0.40 m. Floor top z = 0.7515 m; inner opening 102 × 272 mm. |

The rod translates through the frame and rotates about its axis. A 1 g noncolliding carrier represents the two degrees of freedom using two native single-axis joints. It is not an external support or actuator. The piston plate cannot pass through the frame's small rod opening, so its rearward endpoint is a physical bulkhead contact. A separate passive hinged pawl has a 0.025 N m/rad spring, 0.002 N m s/rad damping and limits −35 to 0 degrees. Its sawtooth contacts permit forward ratcheting and resist rearward motion when engaged.

Reset restores all spawn poses and joint zeros, then settles the native scene for 0.5 seconds before the initial observation. Nothing is consumed: use inert reusable cartridges, with no wet adhesive, cutting or seal opening. Actual starting measurements and seed belong in each run receipt. A real reproduction should secure only the fixed cradle/guides/tray, leave the dispenser free, and use the same inert parts and aggregate masses.

## Scoring

The score is the highest applicable tier in the **current final state**, with no retained peak credit. Conditions are read at 20 Hz; workspace escape is also checked after each physics step at 240 Hz. All lower tiers require the used cartridge to be collected:

| Score | Outcome |
| --- | --- |
| 0 | No listed tier applies, or a permanent invalidation occurred. |
| 0.25 | The complete used-cartridge envelope is within the tray's inner scoring rectangle (±50 mm in x and ±135 mm in y). Its centre is within 25 mm in x/y of the tray centre and 8 mm of z = 0.7765 m, its axis is within the stated horizontal tolerance and its linear speed is below 20 mm/s. |
| 0.5 | Also, the replacement centre is within 6 mm on every axis of `(0, 0.112, 0)` in the measured frame coordinates, its axis is within 0.10 rad of the tool axis pointing forward, and its linear speed is below 20 mm/s. Roll about the cartridge axis is irrelevant. |
| 0.75 | Also, the piston face is from 1 mm beyond to 5 mm behind the replacement rear piston (1 mm numerical contact allowance), the rod's teeth-down rotation is within 0.15 rad of zero, and pawl angle is within 0.12 rad of zero. |
| 1.0 | Also, the frame is within 20 mm of its reset origin, within 10 degrees of its reset orientation and moving below 20 mm/s. Both measured gripper openings are at least 70 mm and all robot collision envelopes are at least 30 mm from all moving task-object envelopes. These complete conditions hold continuously for one second. |

The used-cartridge horizontal check is `abs(axis_z) < 0.1`. Simple rulers, a protractor and video can check the dimensions, placement, alignment, release and hold interval. Clearances use conservative axis-aligned bounds of the native collision geometry; they do not use TCP distance. The simulator computes the cylinder envelope with 64 angular samples (maximum radial discrepancy below 0.03 mm).

Any moving task solid extending below z = 0.7415 m or outside table x = −0.19 to 0.91 m or y = −0.65 to 0.65 m permanently sets the trial to zero. Nonfinite geometry also invalidates a trial. The complete geometric bounds include the tool handle, piston, rod, teeth and both cartridges. This task does not claim to model deformation, fracture or injury; no arbitrary force threshold awards or removes points. All remaining outcomes score zero.

## Recording and limits

Budget: 2,400 actions at 20 Hz, or 120 simulated seconds. The reference plan is 56.2 seconds. The native demonstration reached 1.0 at 54.50 simulated seconds, in 334.75 wall seconds including startup and three 320 × 176 streams. The registered runner reached 1.0 at 54.45 simulated seconds in 346.83 wall seconds including stored frames. The one Dream-1 trial scored 0.0 after all 2,400 actions / 120 simulated seconds and took 502.26 wall seconds, including startup, inference and frame recording. Maximum measured dispenser translation in the native demo was 2.63 mm without a left-hand base hold. All four offline plans, twelve scorer/envelope tests and the unchanged four USD assets pass their checks. Runtime depends on GPU, rendering size and cache. Validated seed is zero; layout and object jitter are fixed.

The recorder preserves the initial native PNGs, three H.264 MP4 streams, every action, measured mechanism/object/robot state, scorer output, executable source fingerprint, installed versions, GPU identity and wall-clock duration. It reports failure if a `demo` trial misses full score; read the receipt's `final_metrics.success` and `score` because native Kit shutdown may leave a zero shell exit status. Smaller diagnostic images can be requested with `--width 320 --height 176`; no paid inference is involved.

The standard CLI route is:

```bash
inspect-robots run --embodiment sealant_reload --policy sealant_reload_scripted \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Replace the used orange cartridge with the prepared blue cartridge. Leave the used cartridge in the collection tray and the dispenser resting in its cradle, with the replacement seated and the piston rod re-engaged at its rear. The rod handle may remain extended behind the dispenser. Release both hands." \
  --scorer task_score --max-steps 2400 --seed 0 --store-frames --no-prompt --grader none --log-dir /path/to/fresh/scripted
```

The submitted media contains the actual single Dream-1 attempt, the full-score scripted demo and their identical initial top image. Both videos preserve every recorded observation in order, with the top view above both wrists. `task.json` records the real scores and the media ID returned by the upstream uploader. The corresponding source, run and media hashes are retained in `validation/`.

## Fidelity and provenance

This is native PhysX rigid-body/contact execution at 240 Hz and robot control/scoring at 20 Hz. The spring pawl, rod translation/rotation, cartridge extraction and seating use physical joints and contacts. There are no software locks, success-triggered unlocks, magnetic grasps, runtime task-body pose writes, task actuators or attachment changes.

The geometry, aggregate masses and contact parameters are nominal and uncalibrated. Task solids use static friction 0.65, dynamic friction 0.5, zero restitution and 0.2 mm contact offsets; the robot and station retain the pinned adapter's defaults. The empty refill has an open rear; the prepared replacement has an immobile rear practice piston. Both have flush front faces; the outlet and its supporting bridge/seat remain on the dispenser frame. The open reload carrier and separate outlet are our own simplified tool geometry. Sealant, dispensing pressure, trigger advancement, tube compliance, wear and damage are outside the model. Completion demonstrates mechanical cartridge replacement with inert parts, not dispensing quality or real-hardware validation.

All task geometry is authored from primitives and contains no commercial CAD or external appearance image. [SOURCE.json](SOURCE.json) records references and integration provenance; [LICENSE](LICENSE) covers this self-contained implementation.
