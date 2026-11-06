# Release a camera from a cam-clamped mount

Remove an unpowered camera and its attached plate from a bench fixture and place the assembly on a green pad. A passive rotating cam retains the front of the plate; a fixed rear lip retains its other edge. Robot contacts must operate the mechanism and transfer the free payload. No task-body actuators or execution-time attachments, pose writes, collision changes or joint-limit changes supply the solution.

**Validation status:** three fresh native PhysX demonstrations reached 1.0 in 643 actions (32.15 simulated seconds), including the registered Inspect Robots `task_score` path with 320 × 176 observations. One hosted Dream-1 trial scored 0.0 after the full 2,400-step budget; no repeat trial was made. A direct-lift control scored zero, and a deliberate 49 mm unsupported descent permanently invalidated another trial despite its final upright pad placement. Nine scorer tests, all five offline URDF plans and three exported USD checks pass. See [validation/README.md](validation/README.md) for measured evidence and run conditions. `task.json` contains the actual scores and uploaded media ID `M-PRZQHX`.

## Instruction and relevance

“Remove the camera and its attached plate from the bench mount and set the assembly upright on the green pad. Keep the camera attached to the plate, do not drop it, and leave the assembly released.”

Camera and robotics technicians remove mounted sensors for servicing and reconfiguration. The mechanical prerequisite, rear-edge clearance and supported transfer distinguish this task from placing a loose camera or operating an isolated switch. Any physically valid method may count; scoring does not demand a particular hand or a prescribed failed pull. Commercial [quick-release camera adapters](https://www.manfrotto.com/global-en/camera-video-supports/quick-change-rectangular-plate-adapter-universal-threads-323/) establish the application; this authored cam-and-lip geometry is not a replica of that product.

The nearest known task families are camera/tripod assembly, DIN-rail snap mounting, two-latch lid removal and transport-key release. This task instead uses a captive rotary cam plus a separate fixed retaining edge, followed by supported dismounting of the camera/plate as one payload. Novelty depends on that mechanism and objective, not the camera appearance, color or a reversed mounting instruction.

The live board also contains pending task `TK-NCPNT9`, "Clamp a blank with an eccentric cam": it measures registration of a loose workpiece against stops and over-centre clamp closure against a compliant backstop. This camera task starts with a captured plate and measures rotary-toe release, clearance of an independent rear retaining lip and supported removal to a separate pad. Both involve passive rotary retention; this is a mechanism/objective distinction for reviewers to assess, not a claim of acceptance.

## Environment

Use the [Robocurve simulation walkthrough](https://tasks.robocurve.org/walkthrough) and the YAM adapter pinned in `pyproject.toml`: `yam-isaac-lab-arena@bd62771e60320433893bd1d2df0fc79b5ddcd6d1`. Its installer uses Isaac Lab-Arena `release/0.3.1`, Isaac Sim 6.1, and a Python 3.12 environment. Source/package versions and GPU information are retained with each recorded run.

On a Linux NVIDIA RTX host with that environment activated, install this folder:

```bash
uv pip install --no-deps --no-build-isolation -e tasks/camera-plate-release
inspect-robots list embodiments
```

The upstream simulator/YAM dependencies must already be installed. The new embodiment is `camera_release`, with the same fourteen joint/gripper actions, proprioception and three camera observations as `yam_arena`. Scorer state is privileged and is not exposed as a policy observation. NVIDIA license acceptance must be supplied by the operator before simulator launch; the task does not set acceptance flags.

## Fixed setup

Coordinates are metres: x forward from the arms, y left, z up. The calibrated station's tabletop is z = 0.7475 m; the arm bases are x = 0, y = ±0.305 m, z = 0.75 m. Both arms start in the station ready pose. There is no layout randomization; `object_jitter=0` is mandatory. Seeds are recorded but do not change this setup.

The standard station includes a 1,100 × 1,300 × 60 mm nominal wooden tabletop centred at `(0.36, 0, 0.7175)` m, two standard YAM arms with metal mechanisms and polymer covers, and their calibrated three-camera observation rig. Its fixed overhead-camera support has a 110 × 110 × 16 mm foot, a 24 mm diameter mast with 928.321 mm capsule spine, an 18 mm diameter arm with 53.852 mm capsule spine (nominal painted steel), and a 50 × 42 × 22 mm nominal ABS camera housing. These are background station parts, distinct from the removable task camera. [setup-layout.json](setup-layout.json) gives their exact positions/orientations, capsule radius/spine conventions, background floor, and every custom task part. Robot CAD and camera calibration are supplied by the pinned public YAM revision.

The fixture's root is `(0.28, 0, 0.7675)` m. The payload is spawned with its plate underside at `(0.28, 0, 0.7775)` m, then settles by 1 mm to z = 0.7765 m on the adapter before the initial image. The fixed receiving pad centre is `(0.20, 0.14, 0.7495)` m; its top is at z = 0.7515 m. Orientations start aligned to the world axes. The cam starts at zero and can rotate positively by 0.95 rad. Its lever is orange for visibility; the other mount parts are grey or dark metal.

| Part | Dimensions and nominal material |
| --- | --- |
| Fixture foot | 110 × 110 × 8 mm aluminium, secured to the bench |
| Short stand post | 26 × 26 × 8 mm aluminium |
| Adapter body | 70 × 52 × 9 mm aluminium |
| Rear rail | 54 × 6 × 6 mm aluminium |
| Rear lip | 54 × 10 × 3 mm aluminium |
| Two side guides | Each 4 × 38 × 4 mm aluminium |
| Two visible mounting screws | 6 mm diameter, 2 mm visible height, steel |
| Cam lever | 24 × 14 × 7 mm aluminium, orange finish |
| Cam bridge | 22 × 8 × 5 mm aluminium |
| Cam retaining toe | 28 × 8 × 4 mm steel |
| Cam pivot | 8 mm diameter × 8 mm long, steel |
| Camera plate | 50 × 38 × 5 mm aluminium |
| Camera | Unpowered D405, aluminium housing; licensed appearance mesh and simplified 23 × 42 × 42 mm collision box in its mounted orientation |
| Rigid camera-to-plate spacer | 12 mm diameter × 7 mm long, aluminium |
| Receiving pad | 130 × 110 × 4 mm green nominal polyurethane |

The camera/plate/spacer are one free 83 g rigid body. The cam is 18 g. The secured fixture has a nominal 0.4 kg assigned mass and fixed base. `build_scene.py` lists every subpart's local position and collider. No camera cable is present. Reset restores the fixed layout, initializes the passive cam angle and lets the native scene settle for 0.5 simulated seconds before the first screenshot.

Material names specify nominal physical reconstruction, not calibrated elasticity, density or friction. The simulation assigns the stated aggregate masses and contact coefficients independently of the display colors. The camera's nominal size and aluminium peripheral housing are supported by the [manufacturer's D400-series datasheet](https://dev.realsenseai.com/download/42003/); aggregate payload mass and fixture geometry are authored task parameters. For hardware reproduction secure the fixture to the bench, keep the camera unpowered and keep hands outside the moving arms' workspace. A full hardware trial has not been performed.

## Scoring

The endpoint score is the highest applicable tier in the current state, provided no episode-wide invalidation has occurred. The runtime uses explicit XYZW quaternions. Distances to robot surfaces are conservative lower bounds from enclosing collision boxes, rather than TCP distance. The scored mount clearance uses the foot, post, adapter, rear rail/lip, both guides and cam lever/bridge/toe/pivot envelopes in `camera_release/config.py`; the two fixed mounting-screw heads do not define that clearance. They remain physical colliders. These are bounding-envelope distances, not exact mesh-surface distances.

| Score | Condition |
| --- | --- |
| 0 | No tier below applies, or an irreversible invalidation occurred. |
| 0.25 | Cam at least 0.8 rad open; payload-to-scored-mount clearance is less than 10 mm and native contact is present. |
| 0.5 | Payload-to-scored-mount clearance is at least 10 mm and native contact is present, but the pad-placement tier does not apply. |
| 0.75 | Plate centre within 20 mm horizontally of pad centre; all payload collision geometry over the pad; plate underside within 5 mm of pad top; tilt at most 10 degrees; native supporting contact. Final release/stability is incomplete. |
| 1.0 | The 0.75 condition holds, every robot collision envelope is at least 30 mm from the payload envelope, and every sampled pose during a full one-second window is within 2 mm and 2 degrees of that window's initial pose. |

An episode is permanently invalidated by fixture translation over 5 mm or rotation over 5 degrees, cam anchor error over 2 mm, cam motion outside −0.01 to 0.96 rad, non-finite/non-unit state, workspace escape, or a drop. Workspace limits apply to the full payload collision envelope: x −0.19 to 0.91 m, y −0.65 to 0.65 m, z 0.7425 to 1.2475 m.

The operational drop criterion is more than 10 mm downward travel during an uninterrupted absence of measured native contact. Contact is detected above 0.02 N, using the payload's normal contacts and separate robot-finger partners so opposing grasp forces do not cancel. The post-physics audit samples at 240 Hz. This is a no-drop contact/motion criterion, not a material damage model. Fixture and joint diagnostics are checked at 20 Hz. All violation flags reset only with a new trial.

For real-world scoring use a ruler, protractor and video to measure the declared clearances/angles and visible support/release. Cam angle is measured relative to its initial zero-angle pose; the simulation does not contain a separate release-angle marking. A real-world drop is more than 10 mm downward travel during continuous absence of visible support, rather than movement while held or sliding under continuous contact. The fixture must be secured, all parts intact, and the camera kept attached. Native geometric/contact diagnostics are simulation approximations of these observable criteria, not independent wrist-force sensing. The 0.02 N native contact threshold is a numerical contact-presence threshold, not a required real-world force measurement.

## Runs, recording and reproduction

First run mathematical scorer tests and offline reachability:

```bash
python -m pytest tasks/camera-plate-release/tests -q
python tasks/camera-plate-release/check_plan.py
python tasks/camera-plate-release/check_assets.py --output /path/to/fresh/usd-check
```

These establish scorer behavior and URDF reachability only, not grasp/contact feasibility.

Then execute native controls and the demonstration, each with a fresh output directory:

```bash
python tasks/camera-plate-release/run.py --policy hold --output /path/to/fresh/hold
python tasks/camera-plate-release/run.py --policy open_only --output /path/to/fresh/open-only
python tasks/camera-plate-release/run.py --policy direct --output /path/to/fresh/direct
python tasks/camera-plate-release/run.py --policy demo --output /path/to/fresh/demo
python tasks/camera-plate-release/run.py --policy drop --output /path/to/fresh/drop
```

`hold` leaves the robot in its reset pose. `open_only` operates the cam. `direct` gently attempts a 5 mm lift without separately releasing it. `demo` operates the cam and completes the dismount/placement. `drop` repeats the transfer but releases 50 mm above the pad to exercise the permanent no-drop invalidation. All commands use native dynamics and robot targets only. The recorder exits with an error if `demo` misses full score or `drop` fails to record a permanent drop violation.

The allowed budget is 2,400 actions at 20 Hz, or 120 simulated seconds. The reference plan lasts 34.8 seconds; both measured native demos terminated successfully at 32.15 seconds. Their wall-clock times, including warm simulator startup and recording three 640 × 360 views, were 113.14 and 111.95 seconds on an RTX PRO 6000 Blackwell Max-Q. First-launch shader compilation took about four minutes in an earlier failed startup/reset trial. Runtime varies with GPU, cache and recording settings. Tested seeds are 0 and 1; the fixed layout does not vary with seed.

The recorder writes three H.264 MP4s, initial PNGs, the action/metric trace, source fingerprint, installed versions, host/GPU identity, elapsed runtime and measured final score. Images contain no terminal overlays or team identifiers. Retain failed trials as evidence; do not replace them with invented success receipts. Use `--no-record` for tuning and `--max-steps 200` for short development runs.

The registered reference policy can also run through Inspect Robots:

```bash
inspect-robots run --embodiment camera_release --policy camera_release_scripted \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Remove the camera and its attached plate from the bench mount and set the assembly upright on the green pad. Keep the camera attached to the plate, do not drop it, and leave the assembly released." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir /path/to/fresh/scripted
```

This registered route reached `task_score: 1.0` with seed 3 in 239.52 wall-clock seconds, including frame/action logging. Use `--seed 3` to repeat it. `inspect-robots video /path/to/log.json --out /path/to/videos` renders its three stored streams; this path was also verified. Scoring comes from the registered task scorer, rather than visual annotation.

## Dream-1 evaluation and submission

The required Dream-1 attempt was performed on November 4, 2026, after native validation and successful demonstration. One epoch with model `dreamlabs/dream-1-beta`, seed 0, 20 Hz, 320 × 176 inputs and the exact declared instruction reached the 2,400-step limit with `task_score: 0.0`. The camera remained in the mount. The evaluator completed without an evaluation error, and the executable source remained unchanged. The trial lasted 120 simulated seconds and 2,671.76 wall-clock seconds (44 min 31.76 s) on the local GPU. No second trial was run.

The SDK returned 50 action chunks through 99 request attempts, including 49 expired-session retries; it reported 2,115 predicted rows outside the declared joint limits. The ordinary YAM adapter clips targets to its limits. The observed score is reported with these conditions, rather than treated as an isolated measurement of task difficulty. The public [run summary](validation/dream-native.json) records counts and the source/log/video hashes without credentials or gateway session IDs.

To reproduce the model evaluation, configure `DREAM_API_KEY` through the operator's environment or Dream client credentials file, then run:

```bash
inspect-robots run --embodiment camera_release --policy dream \
  -E device=cuda:0 -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  -P model=dreamlabs/dream-1-beta -P rows_per_request=48 -P record=false \
  --instruction "Remove the camera and its attached plate from the bench mount and set the assembly upright on the green pad. Keep the camera attached to the plate, do not drop it, and leave the assembly released." \
  --scorer task_score --max-steps 2400 --epochs 1 --seed 0 \
  --store-frames --log-dir /path/to/fresh/dream
```

That command uses hosted inference credits. Encoding stored frames is local and needs no further inference. The task's `run.py` recorder supports only local scripted controls and never calls Dream-1. All model frames were retained, and one untrimmed composite video was encoded: top view above both wrists, 640 × 528, 20 fps, 2,401 frames including the initial image, H.264/yuv420p, 120.05 s, 3,223,363 bytes. An actual initial top frame is retained as the setup screenshot. There are no overlays or generated frames.

`task.json` records `Dream-1 run: 0.0; demo: 1.0` and media ID `M-PRZQHX`. The actual model video, existing full-score demo and model's initial top-camera screenshot were uploaded together using upstream `tools/upload_media.py`. The model and demo video hashes are retained in the validation records.

Both uploaded videos are H.264 MP4 below 50 MB; the setup image is below 10 MB. Task text and media contain no team identifiers.

## Fidelity and provenance

Rigid, simplified colliders represent an unpowered nominal camera mount. The plate and camera cannot separate in the model. Materials do not deform, fracture or wear. Task-owned mount, payload and pad contact materials use 0.6 static friction, 0.5 dynamic friction and zero restitution; robot/station material defaults and PhysX combination behavior come from the pinned integration/runtime. The passive cam has authored friction/damping. These nominal settings produced the recorded native demonstrations; they are not manufacturer measurements or real-hardware validation. Gripper control/gains and calibrated cameras come from the pinned YAM adapter.

The camera appearance retains its MIT source license under `assets/LICENSE.i2rt`; provenance/hashes are in `SOURCE.json`. The original authored mechanism and this adaptation are covered by `LICENSE`. The fixture is not a claimed replica of a named manufacturer's product. Camera electrical function and calibration are outside the task.
