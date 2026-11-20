# Free a sleeve held by a stopped wringer

Use the calibrated Isaac Lab Arena YAM environment and Inspect Robots with the yam_arena embodiment, yam_scripted policy and configured Dream endpoint. Run from this folder after loading the station environment. The task uses the neighbouring files under native/. The default script uses one gripper to lift the release crosshead and the other to withdraw the wrist.

```bash
task="$PWD/native/entry.py"
review_logs="${DATA:-$PWD/results}/logs/reviewer"
instruction="Free the trapped sleeve and leave the sleeve and arm form on the near side of the wringer."
common=(--embodiment yam_arena -E "task=$task" -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "$instruction" --scorer task_score --max-steps 360 --seed 0 --no-prompt --store-frames)
WRINGER_CONTROL=reference inspect-robots run --policy yam_scripted "${common[@]}" --log-dir "$review_logs/reference"
env -u WRINGER_CONTROL -u WRINGER_TRACE -u WRINGER_LIFT_LOG inspect-robots run --policy dream "${common[@]}" --log-dir "$review_logs/dream"
```

Use a fresh log directory for every attempt. The host environment used Inspect Robots 1.0.0, Isaac Sim 6.1.0.0, Isaac Lab 17.0.2 and the calibrated YAM station plugin. One trial allows 18 simulated seconds. Wall time depends on RTX rendering and inference latency. The fixture layout is fixed. The seed used for the accompanying results is 0. The native entry adds an in-process fixture loader and samples PhysX body transforms and gripper contacts after every physics step. The complete scoring conditions are in task.json and native/measurements.py. Task-specific contact telemetry can be enabled by setting WRINGER_TRACE to a new JSONL file before a trial.

Real-world comparisons should use a de-powered low-energy training fixture and an instrumented dummy wrist. The blue guided coupon represents a sleeve that is still captured after a machine stops. The spring reads tensile load in the direction of withdrawal. No person serves as the test subject.

The scripted demonstrators use fingertip contact and measured gap to adjust the lifting hand's joint targets under load. Both full rescues also stop if the measured opening is unproven at withdrawal or the dummy wrist reaches the 4 N control margin. The independent Dream evaluation receives the stock camera and proprioceptive observations.

The horizontal sleeve guide has passive damping of 1.0 N s/m, capped at 0.25 N. It exerts zero damping force at rest and has no position spring. The coefficient describes the simulated trainer.


## Assembly and reset

Coordinates below are in millimetres relative to the fixture base origin. Positive x points away from the operator. The station instantiates the supplied assembly at x = 350, y = 0, at its table surface. Both stock YAM arms start at the calibrated ready pose. Dimensions are x by y by z unless a diameter is given. All fixture shapes are rigid with the same nominal contact material, static friction 0.90, dynamic friction 0.75 and restitution zero.

| Component | Size, mass and fixed starting layout |
|---|---|
| Foot and two cheeks | Foot 120 x 190 x 16, centre (0,0,8). Each cheek 36 x 8 x 84, centred at (0,+/-69,49). Frame is kinematic. |
| Two stopped rollers | Each diameter 65 and width 120 along y. Lower centre at z = 43. Nominal upper centre at z = 111 before reset settling. Yoke mass 210 g including the upper roller and handles. |
| Yellow lifting handles | Each side carries a 15 x 52 x 14 stem, a 31 x 18 x 33 tab and a 37 x 28 x 4 lip. Stem centre y = +/-84. Tab and lip centres y = +/-104.16, at z = 10 and 28 relative to the yoke. |
| Blue sleeve coupon | Rigid 136 x 35 x 6 rectangle, 26 g, centred at (-41,0,78.5). The horizontal slide and coupon have no driven withdrawal target. |
| Orange wrist surrogate | 50 x 44 x 27 body with two 28 x 26 x 28 handles at local (0,+/-35,2). Total 145 g. Centre (-131,0,78.5). |
| Passive prismatic joints | Yoke z travel 0 to +36 from its nominal position, stiffness 1250 N/m, damping 4 N s/m, target -12 mm, maximum drive force 100 N. The empty stop leaves a nominal 3 mm gap. Coupon x travel -135 to +6. Wrist spring x travel -80 to +6 about its nominal 90 mm offset, stiffness 350 N/m, damping 0.04 N s/m. |

## Measuring a trial

Score the whole trajectory. A hard tug can invalidate an apparently successful final arrangement. Wrist traction is 350 times the positive extension in metres beyond the relaxed 90 mm axial separation. It is a proxy for tensile load on the surrogate. A 5 N trip corresponds to 14.29 mm extra separation and 1 N to 2.86 mm. Contact means a monitored robot-link/fixture pair exceeding 0.04 N. The two grippers can exchange roles. Guided distance counts each newly traversed millimetre once, over the first 61 mm only. Start from the fixed datum and use the physical poses measured at 240 Hz. A 2.55 s success hold at 20 Hz delays successful termination so the recording includes the fixture settling. The score itself uses a 0.5 s dwell. A time-limit termination uses the score at the end.

A fixture violation latches if translation exceeds 2 mm or the absolute scalar part of its unit quaternion falls below 0.9995 (about 3.62 degrees). Yoke, coupon or arm lateral displacement over 4 mm, coupon or arm height error over 4 mm, gap outside the open interval (-11,42) mm, stroke outside (-9,140) mm, or coupon absolute quaternion scalar below 0.998 also latch failure. The fixture's empty travel stop supplies a separate geometric check in the verification run. For simple bench cross-checks, mark the datum, corners, travel and wrist-extension limits on the dummy rig and film in profile with a scale. A peak witness or adequately sampled displacement recording is needed for brief excursions. The native telemetry is required to reproduce the exact substep scores near boundaries.

Physical replication uses only a low-energy, de-powered trainer with a surrogate wrist and independent risk assessment. The rigid guided coupon omits folding, tearing and soft-tissue compliance. No injury threshold has been inferred.

The accompanying runs took 55.3 s scripted and 71.6 s with Dream-1, measured by Inspect.
