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

The scorer reconstructs each stage from measured poses and filtered contacts.

| Score | Conditions |
| --- | --- |
| 0 | No valid retained transfer at the end of the episode. |
| 0.25 | Complete reference capture, blade release, airborne carry and board seating in order. Preserve the captured relative blade angle within 2° through the end. |
| 0.5 | Also cover at least 50% of the target within 5 mm, with internal gaps at most 5 mm, angle error at most 5°, stray ink at most 10 mm-equivalent, transverse RMS error at most 3 mm and at least 50% guided ink. |
| 1.0 | Cover at least 90% within 3 mm, with gaps at most 3 mm, angle error at most 2°, stray ink at most 5 mm-equivalent, transverse RMS error at most 1.5 mm and at least 90% guided ink. Then hold the nib at least 5 mm above the board for 0.5 s without new ink. |

**Capture and release.** Both reference side contacts must exceed 5 mN and fit within 2° and 3 mm for 0.3 s. The blade must have moved at least 10° from its measured starting relative angle, which must be at least 15° away from the reference. During the capture window the relative-angle span must be at most 0.25° and the diagonal of the stock-position envelope at most 0.5 mm. To release, both right-finger forces on the blade must stay at or below 5 mN and the right TCP must stay at least 18 mm from both the blade body and tab for 0.1 s. During this release hold, both reference contacts must still qualify or the stock must be within 5 mm of its capture position. Subsequent finger contact above the threshold or loss of clearance invalidates the capture.

**Carry and seating.** Lift the stock at least 20 mm from capture height, then move it at least 80 mm horizontally while remaining at least 10 mm above capture height. Complete the transfer by seating the stock against the board after leaving the reference fit. The seating and blade placement tests below also apply at this stage.

**Ink and guidance.** Ink requires marker-board contact above 5 mN within 4 mm of the nib centre and 1 mm of the board top, with the nib bottom within 2 mm of the surface, tilt at most 25°, and the contact point inside the board. Contact loss or jumps over 10 mm break strokes. Guidance requires stock-board normal force above 5 mN, stock seating within 3 mm, stock yaw and blade yaw/tilt within 2°, blade edge height within 2 mm, blade edge offset 0.5–4.5 mm from the target (2.5 mm nominal) and the nib within 3.5 mm transversely of the blade edge and at most 2 mm beyond either endpoint. Marker-blade force is recorded as a diagnostic.

The visible ink and the guided subset must each meet the geometry thresholds for the grade. Both line grades limit total ink-equivalent length to 150% of the target. Stray ink is measured outside the finite 5 mm target corridor. The scorer grades the union of the visible 2 mm-wide strokes on a 0.25 mm raster. Earlier marks, including marks before capture or from an invalidated transfer, remain visible and count toward stray ink. Only ink guided by the current retained transfer can supply guided coverage.

## Install and run

Use the Robocurve YAM/Isaac environment with Isaac Sim 6.1.0.0, Inspect Robots 0.60.0, inspect-robots-dream 0.1.0 and dreamlabs-client 1.0.14. The simulation walkthrough is on the [Robocurve task board](https://tasks.robocurve.org/). The task verifies PhysX PGS with minimum position and velocity iteration counts of 64 and 16 at startup. Both commands below use seed 0, zero layout jitter and a 2,400-step limit at 20 Hz (120 s of simulated control time).

| Dependency checkout | Validated commit |
| --- | --- |
| YAM-inspect-Isaac | `d825f0c51724f750f26e8c3f5620a14b5b0261b0` |
| yam-isaac-lab-arena | `0059d02a1a91bc6834c1d0846022891df0c78730` |
| IsaacLab-Arena | `c7edf3a9d5d8276565208f72422f8f8a0806d6cf` |
| IsaacLab submodule | `91a0906f93802eb859192dcb0bca879f891e92d6` |

```bash
source /data/IsaacLab-Arena/yam.env
"$HOME/.local/bin/uv" pip install --no-deps -e /path/to/bevel-angle-transfer
TASK_TEXT=$(python -c 'from bevel_angle_transfer.task_spec import INSTRUCTION; print(INSTRUCTION)')
inspect-robots run --embodiment bevel_angle_transfer --policy bevel_angle_transfer_demo \
  -P mode=demo -E demo_observations=true -E terminate_on_success=true \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  -E output_dir=outputs/demo/traces --instruction "$TASK_TEXT" \
  --scorer task_score --max-steps 2400 --epochs 1 --seed 0 --store-frames \
  --grader none --no-prompt --no-rerun --no-rerun-save --log-dir outputs/demo/logs
```

Use `-P mode=mechanism` for transfer only or `-P mode=hover` for marking motions with the nib 15 mm above the surface. Hover can earn 0.25 for a retained transfer and must produce zero ink.

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

The regrader reconstructs capture, retention and ink from raw physics traces, checks the source and configuration, and compares the exported metrics and ink. Keep a frozen source snapshot with each recorded run. Changes to task Python or USD assets change the implementation revision.

## Recorded results

| Trial at seed 0 | Score | Control steps | Wall time on the validation VM |
| --- | ---: | ---: | ---: |
| Scripted demonstration | 1.0 | 1,834 | 610.4 s |
| Dream-1 beta | 0.0 | 2,400 | 578.8 s |

Both raw contact traces reproduced their exported scores and ink histories on the validation VM and a separate CPU environment. All 44 automated tests passed. These results cover one trial per policy. The optional hover and Dream smoke trials were skipped.

Implementation revision `a526f28723c87d20f092a066966df3ec6902bec5b4022caa0449a6a3f49cdc23`.

## Fidelity and reset

The scripted controller uses measured simulator tool poses and ordinary joint and gripper targets. It draws along the measured retained blade edge. Dream uses camera and joint observations. Both policies act on free tools. Initial tool pose and hinge-angle writes occur at reset.

The simulation uses rigid contact, a passive hinge and geometric ink generated by nib contact. It simplifies the marker's fluid flow, nib wear and compliance, wood deformation and surface roughness. The fixed layout tests simulated feasibility at one angle.

Reset clears the ink and temporal scoring state and restores the fixed layout, loose tool poses and initial +10° blade angle. For physical reproduction, clean or replace the board and restore the fixtures and tools. Use blunt, lightweight, unpowered tools.

The included task code and assets are original MIT-licensed work. External dependencies retain their own licences.
