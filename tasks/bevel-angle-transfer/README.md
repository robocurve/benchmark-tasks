# Transfer a reference angle with a bevel gauge

## Task

Fit the blue-stock bevel gauge to the two grey reference faces by adjusting its hinged blade. Lift and carry the gauge to the separate wooden board without changing the captured angle. Seat the stock against the board's near edge and mark one straight line through the red tick along the retained blade. Avoid extra marks and finish by lifting the marker.

The gauge begins 30° from the reference angle. The robot must adjust the passive hinge, release the blade at the reference fixture, and preserve the resulting setting through the carry and marking.

## Fixed scene

Positions are in metres, with x pointing away from the robot bases, y left, and z up. The tabletop is at z = 0.7475. The bimanual YAM bases are at (0, +0.305, 0.75) and (0, -0.305, 0.75). The board, reference fences, and marker holder are fixed. The gauge and marker move under ordinary contact.

| Item | Geometry and layout |
| --- | --- |
| Board | Wood, 140 × 180 × 16 mm, centred at (0.29, 0, 0.7555). Top at z = 0.7635. External fixings are simplified away. |
| Tick | Red printed visual mark, 4 × 2 mm, centred at (0.222, −0.03). |
| Reference stock fence | Grey rigid plastic, 20 × 50 × 25 mm, centred at (0.23, 0.365, 0.7600). Its mating face is x = 0.22 and spans y = 0.34–0.39. |
| Reference blade fence | Grey rigid plastic, 140 × 20 × 25 mm, centred at (0.295128859, 0.127004133, 0.7600), yaw −20°. It meets the gauge blade's low-y edge. |
| Gauge stock | Blue rigid plastic, 24 × 250 × 36 mm, 180 g. Initial root at (0.18, 0.28, 0.7655), yaw 0°. |
| Gauge blade and tab | Lightweight hollow plastic blade, 170 × 20 × 6 mm, with a raised 18 × 18 × 24 mm grasp tab, 18 g combined. Pivot at (0, −0.1075, 0.001) in the stock frame. Tab centre at (0.115, 0.015, 0.015) in the blade frame. Initial relative angle +10°. |
| Pivot | Passive vertical revolute hinge with limits of -45 to +45 degrees, 0.010 N·m static friction, 0.008 N·m dynamic friction and a 0.001 N·m·s/rad viscous coefficient. A constant zero-speed damper adds 0.010 N·m·s/degree (0.572958 N·m·s/rad), capped at 0.10 N·m. The drive has zero stiffness and no authored angle target. |
| Marker | Orange rigid plastic marker, 40 g, with a 22 × 22 × 80 mm barrel, a 4 × 4 × 23 mm nose, and a felt nib represented by a rigid sphere of radius 2 mm. Initial root at (0.18, −0.19, 0.8175). Barrel centre 15 mm above the root and nib centre 50 mm below it. |
| Marker holder | Fixed rigid plastic, with a 40 × 40 × 18 mm base centred at (0.18, −0.19, 0.7565). Four 15 mm-high walls form a 6 × 6 mm opening for the marker nose. |

Friction and damping are constant throughout the episode. Sufficient applied torque can change the hinge angle. The stock and blade overlap at the hinge, where self-collision is disabled. Both links collide with other bodies. The target runs from (0.22, −0.03) to (0.36, −0.080955833) and is about 149.0 mm long. Only the red tick is visible at reset.

## Score

Score the transfer from one continuous video and the finished line from one overhead photo or the board itself. The [physical judging guide](PHYSICAL_SCORING.md) includes an actual-size overlay. The simulator applies the same rubric using measured poses, gripper state and the visible ink footprint.

| Score | Conditions |
| --- | --- |
| 0 | The transfer is invalid or incomplete. |
| 0.25 | Fit the stock and blade to both reference faces with at most 3 mm of gap at each end of each mating span. The blade gripper visibly opens and clears the blade before the gauge leaves the fixture. Carry the gauge by the stock without dragging, setting it down, regrasping or readjusting. Seat the stock within 3 mm of the board edge, with the blade flat on the board (at most 3 mm of visible gap). The stock-to-blade angles at capture and board placement differ by at most 2°. |
| 0.5 | Complete the transfer and use the seated blade to guide the marker. The line comes within 5 mm of the red tick centre, covers at least 70 mm projected across the 140 mm board width, stays within 5 mm perpendicular to the target centreline and has internal gaps of at most 5 mm. |
| 1.0 | Complete the transfer and use the seated blade to guide the marker. The line comes within 3 mm of the tick centre, covers at least 126 mm across the board width, stays within 3 mm of the target centreline and has internal gaps of at most 3 mm. Finish with at least 5 mm of clearance from the nib tip to the board surface. |

Both line grades allow one line at most 4 mm wide with no separate specks or second line. Coverage sums distinct marked intervals across the board width. Internal gaps are measured between those intervals. All visible marker ink counts. Award the highest score whose conditions pass.

For simulator replay, each continuous pen-down stroke needs evidence that the marker followed the seated blade. A freehand stroke or stamp, ink made before transfer, or a consecutive unguided portion spanning at least one ink width (2 mm) along or across the target loses line credit. The spatial span uses the outermost positions, so dwelling or retracing within a smaller area adds no span.

## Install and run

Use the Robocurve YAM/Isaac environment with Python 3.12.16, Isaac Sim 6.1.0.0, Inspect Robots 0.60.0, inspect-robots-yam 0.37.0, inspect-robots-dream 0.1.0, dreamlabs-client 1.0.14 and NumPy 2.3.1. The simulation walkthrough is on the [Robocurve task board](https://tasks.robocurve.org/). The task verifies PhysX PGS with minimum position and velocity iteration counts of 64 and 16 at startup. Both commands below use seed 0, zero layout jitter and a 2,400-step limit at 20 Hz (120 s of simulated control time).

| Dependency checkout | Validated commit |
| --- | --- |
| yam-isaac-lab-arena | `0059d02a1a91bc6834c1d0846022891df0c78730` |
| IsaacLab-Arena | `c7edf3a9d5d8276565208f72422f8f8a0806d6cf` |
| IsaacLab submodule | `91a0906f93802eb859192dcb0bca879f891e92d6` |

```bash
source /data/IsaacLab-Arena/yam.env
"$HOME/.local/bin/uv" pip install --no-deps -e /path/to/bevel-angle-transfer
TASK_TEXT=$(python -c 'from bevel_angle_transfer.task_spec import INSTRUCTION; print(INSTRUCTION)')
inspect-robots run --embodiment bevel_angle_transfer --policy bevel_angle_transfer_demo \
  -E demo_observations=false -E terminate_on_success=false \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  -E output_dir=outputs/demo/traces --instruction "$TASK_TEXT" \
  --scorer task_score --max-steps 2400 --epochs 1 --seed 0 --store-frames \
  --grader none --no-prompt --no-rerun --no-rerun-save --log-dir outputs/demo/logs
```

The demonstration replays a fixed 1,897-step, 20 Hz joint/gripper program without reading observations, tool poses, contact forces or scorer flags. Its actions were calibrated in the 2026-11-12 development run; their provenance and checksum are packaged with the program. A fresh run with `demo_observations=false` reached 1.0 under the revised scorer. The historical feedback controller is separately registered as `bevel_angle_transfer_feedback` for development only, with `-P mode=demo|mechanism|hover -E demo_observations=true`. A valid transfer with hovering may earn 0.25 but must produce no ink.

For Dream-1, configure the Dream Labs API credential privately, then run the following in the same environment. The recorded evaluation wrapped this command with a persistent API request budget.

```bash
inspect-robots run --embodiment bevel_angle_transfer --policy dream \
  -P model=dreamlabs/dream-1-beta -P timeout_s=60 \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  -E output_dir=outputs/dream/traces --instruction "$TASK_TEXT" \
  --scorer task_score --max-steps 2400 --epochs 1 --seed 0 --store-frames \
  --grader none --no-prompt --no-rerun --no-rerun-save --log-dir outputs/dream/logs
```

Replay and export a trial with the following commands.

```bash
python -m bevel_angle_transfer.regrade outputs/demo/traces/trial-0000
inspect-robots video outputs/demo/logs/RUN.json
```

The regrader reconstructs the transfer and ink from raw physics traces, checks the source and configuration, and compares the exported metrics and ink. The v2 trace records measured jaw opening and finger link poses. It cannot certify old traces that lack those measurements: use the preserved v1 source to reproduce historical grades. Keep a frozen source snapshot with each recorded run. Task Python, USD assets, finger envelopes and the fixed action program are included in the implementation hash.

The native top camera renders at twice the adapter's default width and height, with its calibrated intrinsics rebuilt at that size. This supplies a clearer final board photo. Policy images remain at the requested 320 x 176 resolution; wrist camera settings and all camera poses are unchanged. Actual image sizes are recorded in `run_config.json`.

## Validation status

Verified on 2026-11-12, fixed layout, seed 0:

| Evaluation | Score | Control steps | Simulated control time | Wrapper wall time |
| --- | ---: | ---: | ---: | ---: |
| Fixed joint-action demonstration, no simulator feedback | 1.0 | 1,897 | 94.85 s | 930 s |
| Dream-1 beta | 0.0 | 2,400 | 120 s | 1,158 s |

Both runs used `demo_observations=false`. They ran concurrently on one L40S; wall times include startup and shared-resource execution. The demo marked 136.97 mm of the 140 mm width, reached the tick, had no internal gap, a maximum template offset of 2.52 mm and maximum line width of 2.34 mm. It finished with the marker clear and zero transfer invalidations. Dream made no ink and completed no valid transfer. Its 95 SDK calls included 45 handled expiry notices; the evaluation completed normally.

Both complete measurement traces independently regraded to exactly the exported scores and ink histories on the VM and on a separate Windows CPU. All 63 automated tests passed. They cover the physical scoring boundaries, invalid captures/carries, freehand marks, small contact-boundary noise, replay integrity, robot geometry, and a fixed demo that cannot read observations.

```bash
python -m pytest -q
```

Implementation revision: `b257d06a6571d68b156ea146279366b39229cf9b0f5197700e4538cb1028decc`.

This establishes fixed-layout simulated feasibility. The physical judging procedure has not been tested on hardware. This revision's media has not been uploaded; `task.json.media` remains empty until the new recordings are uploaded.

### Historical results

These seed-0 trials used the previous scorer and frozen task source. Media ID `M-6KCFQS` belongs to those trials.

| Trial at seed 0 | Score | Control steps | Wall time on the validation VM |
| --- | ---: | ---: | ---: |
| Scripted demonstration | 1.0 | 1,834 | 610.4 s |
| Dream-1 beta | 0.0 | 2,400 | 578.8 s |

Both raw contact traces reproduced their exported scores and ink histories on the validation VM and a separate CPU environment. All 44 automated tests passed. These results cover one trial per policy. The optional hover and Dream smoke trials were skipped.

Historical implementation revision `a526f28723c87d20f092a066966df3ec6902bec5b4022caa0449a6a3f49cdc23`.

## Fidelity and reset

The fixed demonstration executes calibrated joint and gripper targets without reading object poses, forces or scorer flags. The separately registered development controller uses simulator feedback to prepare those targets. Dream uses camera and joint observations. All controllers act on free tools. Initial tool pose and hinge-angle writes occur at reset.

The simulation uses rigid contact, a passive hinge and geometric ink generated by nib contact. It simplifies the marker's fluid flow, nib wear and compliance, wood deformation and surface roughness. The fixed layout tests simulated feasibility at one angle.

Reset clears the ink and temporal scoring state and restores the fixed layout, loose tool poses and initial +10° blade angle. For physical reproduction, clean or replace the board and restore the fixtures and tools. Use blunt, lightweight, unpowered tools.

The task code and authored gauge, board and marker assets are MIT licensed. Finger-envelope dimensions are derived from the pinned YAM collision geometry; their JSON records the source URDF and mesh checksums. External robot and simulator dependencies retain their own licences.
