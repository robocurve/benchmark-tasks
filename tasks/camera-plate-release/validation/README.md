# Native validation

These are measured local executions of the submitted task with native PhysX and robot joint/gripper targets, including scripted demonstrations/controls and one hosted Dream-1 trial. No hardware trial was performed. The local installation uses Python 3.12.12, Isaac Sim 6.1.0.0, Isaac Lab package metadata 17.0.2, Arena package metadata 0.3.0, YAM 0.1.0 and Inspect Robots 0.60.0. Source revisions are pinned in the task README. Execution used an NVIDIA RTX PRO 6000 Blackwell Max-Q, driver 580.178.04.

## Measurements

| Run | Seed | Actions / simulated time | Final score | Result |
| --- | --- | --- | --- | --- |
| First full demonstration | 0 | 643 / 32.15 s | 1.0 | No validity violations; 7,716 physics audit samples. |
| [Repeated demonstration](demo-native.json) | 1 | 643 / 32.15 s | 1.0 | No validity violations; 7,716 physics audit samples. |
| [Inspect Robots evaluation](inspect-robots-native.json) | 3 | 643 / 32.15 s | 1.0 | Registered `task_score`, successful termination, 320 × 176 observations. |
| [Single hosted Dream-1 evaluation](dream-native.json) | 0 | 2,400 / 120 s | 0.0 | Ended at the step limit; one epoch, no repeat trial; gateway retries and target clipping recorded below. |
| [Gentle direct lift](direct-native.json) | 0 | 201 / 10.05 s | 0.0 | Cam remained engaged; payload remained retained; no violation. |
| [Deliberate drop](drop-native.json) | 2 | 697 / 34.85 s | 0.0 | 49.16 mm unsupported descent; permanent `drop` flag despite final upright pad placement. |

The repeated demonstration, Inspect Robots evaluation and drop trial use source fingerprint `ea9eeed9e722a597e2e074b3fa361911e541e9a236bc61a96ed651cd7a48f403`. The first demonstration and direct-lift trial used the earlier fingerprint `8cee05ddf37c05c3a4a450f623880666d4a40e3669da8af4e6690aedb61f13ed`, before adding the audit-count guard and the deliberate-drop controller mode. No scene or scoring thresholds changed between those two revisions. Each recorder receipt confirms the executable source stayed unchanged during its run; the CLI log identifies the same environment revision. Documentation and receipt files are excluded from that executable fingerprint.

In the repeated demonstration the endpoint scorer first reached 0.25 at 7.85 s, 0.5 at 22.80 s, 0.75 at 29.05 s and 1.0 at 32.15 s. It finished with a 0.372 mm horizontal pad-centre error, at least 75.8 mm clearance from all robot collision envelopes, and no unsupported descent. The one-second released stability window completed before termination. Wall-clock runtime including simulator startup and all three recorded views was 111.95 s; the first successful demonstration took 113.14 s.

The direct-lift control is evidence for retention under that particular gentle motion, not proof that every alternative removal strategy fails. Scoring accepts safe physically valid alternatives and does not require a designated hand, cam-first action order or intentionally failed pull.

## Other checks and evidence

Nine mathematical scorer tests pass, including known XYZW rotations, partial credit, stable placement, robot clearance, containment, persistent invalidation and reset. [All five offline plans](offline-ik.json) pass URDF reachability checks with worst position error below 0.10 mm and rotation error below 0.00025 rad. [Three generated USD assets](offline-usd.json) pass schema/compliance and joint-reference checks. These offline checks are separate from the native feasibility evidence above.

Both successful native demonstrations have synchronized top/left-wrist/right-wrist H.264 recordings, initial images and per-action measured traces retained locally. A 32.2 s three-view demo is H.264/yuv420p, 1,280 × 1,080, 20 fps and 3,571,123 bytes; its setup image is 170,971 bytes. They fit the portal's respective 50 MB and 10 MB limits and contain only the simulated station. Media IDs are supplied only after upload.

The Inspect Robots evaluation separately stored and rendered all three 320 × 176 streams, 644 frames each. Its measured wall-clock runtime was 239.52 s with logging, and its official log reports zero errored trials and `task_score: 1.0`. Its summary retains the original log's SHA-256. The lower-resolution setup and cam-operation views were visually inspected, alongside the high-resolution recorded placement and release.

Earlier development failures are retained locally: the first simulator trial failed before any control action due to a pose dtype mismatch; two early cam-only controllers missed the target release state. Their outputs were used to correct pose conversion and robot finger/withdrawal trajectories. They are not represented as successful trials or hosted-model attempts.

## Scope

The full-score runs demonstrate feasibility inside the 2,400-action budget for this fixed native simulator setup. A real mount's friction, manufacturing tolerances, compliance and damage behavior have not been calibrated. The scene uses a nominal rigid mechanism, a licensed camera appearance and simplified collision geometry, as disclosed in the task specification.

The final hosted Dream-1 evaluation used the same executable source and instruction as the demonstrations. It completed one epoch without an evaluation error and scored 0.0 at the 2,400-step limit. The camera remained in its mount in the inspected middle/final views. Runtime was 2,671.76 wall-clock seconds, with 120 simulated seconds of robot control.

Its transcript contains 50 returned action chunks through 99 request attempts, 49 expired-session retries, 50 distinct gateway sessions and 2,115 predicted rows beyond the declared limits, which the ordinary adapter clips. These conditions limit interpretation as a clean measurement of task difficulty; the measured result is retained without a rerun. No gateway session identifiers or credentials are included in the public summary.

Exactly one model video was encoded locally from all 2,401 synchronized observations per camera, including the initial frame: H.264/yuv420p, 640 × 528, 20 fps, 120.05 s, 3,223,363 bytes. The first top frame is 55,027 bytes. Only native frames were resized and stacked. The video and setup image hashes are in [the run summary](dream-native.json). The model video, full-score demo and actual initial top frame were uploaded together as `M-PRZQHX`, the media ID recorded in task.json.
