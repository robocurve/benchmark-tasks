# Join a drive-chain service joint

The right YAM arm picks up a loose yellow two-pin connecting link and slides it into an empty service cradle. Both arms pinch the bare dark grey end links, lift and align their holes, then lower them over the pins. The left arm picks up the blue forked keeper from the left side of the table and slides it under both heads, releases it and withdraws. A fixed toothed service former guides the loop throughout assembly. This is an unpowered maintenance assembly exercise.

The fresh native three-camera reference and independent registered Inspect camera evaluation both reach **1.0 at 1,180 actions / 59 seconds**. Omitting the retaining slide scores **0.4**. All **37 tests** and **four USD structure checks** pass.

The reference uses a fixed 20 Hz program of fourteen robot joint/gripper commands for this calibrated fixture. A fresh native execution, rather than prerecorded object motion, is required to demonstrate success. Validation for the revised controller is recorded in `validation/`. The matching Dream-1 episode completed all 2,400 actions / 120 simulated seconds and scored **0.0**, with no execution error. Its complete three-camera recording accompanies the full-score scripted demo.

## Real-world operation and approximation

Industrial maintenance technicians rejoin open conveyor or drive chains by aligning both free ends on a connecting link and fitting its retaining hardware. [Tsubaki's chain-splicing instructions](https://en.tt-net.tsubakimoto.co.jp/tecs/engd/cdc/engd_cdc_tri_kir.asp) describe using sprocket teeth to locate the chain during this operation. This reassembly work belongs to industrial machinery mechanics, machinery maintenance workers and millwrights: [BLS describes their work on conveying systems](https://www.bls.gov/ooh/installation-maintenance-and-repair/industrial-machinery-mechanics-and-maintenance-workers-and-millwrights.htm), and [O*NET lists repair and reassembly duties](https://www.onetonline.org/link/summary/49-9041.00). This motivates the **Impact on jobs** category.

This task isolates that substep on a bench fixture. Both free links articulate through their existing hinges, so aligning one end changes the shared assembly; the robot must engage two holes and retain them without disturbing the guided loop. The right arm places the connector, both arms join the ends, and the left arm fits the keeper. The score accepts any physically valid approach and does not require these particular arm assignments or a prescribed sequence.

The 100 mm-pitch printed parts and one-piece keeper are custom reproducible surrogates. Commercial roller-chain connections normally use an outer plate plus a spring clip or cotters; the forked keeper here combines the plate and axial-retention functions under headed pins and is held laterally by friction. The fixed toothed wheel is an assembly former. All parts are original primitive geometry under the included MIT license; manufacturer references supply context only.

## Install and run

Use the [Robocurve walkthrough](https://tasks.robocurve.org/walkthrough) environment: Python 3.12, Isaac Sim 6.1 and Isaac Lab-Arena. `SOURCE.json` records validated versions and the official asset-root fix. NVIDIA license acceptance is required. From this task folder:

```bash
uv pip install --python "$(command -v python)" -e .
python -m unittest discover -s tests -v
python check_assets.py --output /tmp/chain-assets
python check_plan.py
```

Native scripted reference with actual cameras:

```bash
python run.py --policy demo --device cuda:0 --physics-device cpu \
  --width 320 --height 176 --max-steps 2400 --seed 1415158438 \
  --output /tmp/chain-native
```

Registered rendered evaluation:

```bash
instruction=$(python -c 'from chain_link.config import INSTRUCTION; print(INSTRUCTION)')
inspect-robots run --embodiment chain_link --policy chain_link_scripted \
  -E device=cuda:0 -E physics_device=cpu -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
  --scorer task_score --max-steps 2400 --epochs 1 --seed 0 --store-frames \
  --no-prompt --grader none --no-rerun --no-rerun-save \
  --log-dir /tmp/chain-inspect
```

The recorded hosted run uses the same registered command with `--policy dream -P model=dreamlabs/dream-1-beta -P rows_per_request=48 -P record=false`, with a privately configured `DREAM_API_KEY`. Hosted evaluation consumes credits; the supplied result is the one completed episode. `validation/model-native.json` records its measured score, termination and source identity.

Each command needs a fresh output directory. The fixed layout has object jitter 0. Inspect evaluation seed 0 derives trial seed 1415158438; the direct native recipe uses that same trial seed. The calibrated program contains 1,180 commands, followed by a three-second hold (62 simulated seconds total), and normally stops earlier once success holds. CPU development takes about 1–5 minutes wall time. Three-camera evaluation takes several minutes on an A6000, including renderer warm-up; hosted inference adds time. Limit: 2,400 actions / 120 simulated seconds at 20 Hz, with 240 Hz physics.

The fixed reference is calibrated with cameras enabled at 320x176, including the standard Inspect action guards. CPU-only development adds `--device cpu --no-cameras --no-record` to the native command, or `-E device=cpu -E render_images=false` to Inspect. It produces no submission video. `--policy connector` stops after placing the loose connector; `--policy unlocked` omits the keeper slide; `--policy ends` stops after joining the two ends.

Model observations are the standard fourteen joint/gripper values and calibrated `top_cam`, `left_cam` and `right_cam` RGB views. The reference policy uses only elapsed time, just as a taught fixed-fixture robot program does. `chain_link/motions/*.json` contain robot commands and stage names only, and are included in the source fingerprint and installed package. They were extracted once from earlier simulator-assisted development trials; that calibration origin is explicit in each file. The submitted reference re-executes those commands with ordinary contact physics and no live object-pose, contact or score input. This fixed-layout reference does not establish robustness to unseen placements. The runner's native diagnostics are used only after actions for scoring and logs.

All post-reset actuation is through the ordinary fourteen robot controls. No task-body pose writes, grasp attachments, software latches or runtime collision changes are used. A test executes the entire registered policy with an observation that denies access to anything except elapsed time. The environment has no controller-access registry.

Passive hinges use the standard PhysX actuator path, `sim.use_newton_actuators=False`; task joints have no actuator groups. This prevents the optional native actuator wrapper from acting on uncovered passive joints. Standard robot PD gains and physical contact remain in use.

## Primary score

Scoring uses the current state. Exact predicates are in `chain_link/scoring.py`; numerical diagnostics are recorded in each native trace. Coordinates below are connector- or keeper-local unless stated otherwise.

- **0:** either end hole is unseated, or a permanent workspace/invalid-state violation occurred.
- **0.4:** both hole centres are within 12 mm radially of their respective stems, at z strictly between 3 and 14 mm, with link-normal dot product greater than 0.98 against the connector normal.
- **0.7:** both ends are seated; keeper centre z is strictly between 16 and 25 mm, relative orientation is at most 0.10 rad, and both stem projections lie in its slots: x strictly between −22.5 and 36 mm, y within strictly 5 mm of its own ±50 mm slot centre.
- **1:** both projections additionally have x strictly between −22.5 and −8 mm; connector stays within strictly 15 mm and at most 10 degrees of its cradle pose; all moving bodies stay below 20 mm/s and 0.3 rad/s; both grippers open at least 70 mm; all robot collision bodies clear all moving task bodies by at least 25 mm; and all nine resting links remain seated on the guide. These conditions hold continuously for one second.

Guide seating uses the actual link windows. For each link from `Link01` through `Link09`, its fixed tooth axis, evaluated at the nominal chain plane and projected into the link frame must have absolute x below 21 mm and absolute y below 11.5 mm. The 6 mm-radius tooth fits inside the nominal ±27 mm central opening and ±17 mm space between side rails; the lateral bound includes a 0.5 mm contact margin. Link centres must remain within strictly 6 mm of world z=0.847 m, with world-z normal dot product greater than 0.995. A folded or lifted loop cannot earn full credit.

Keeper slots end at x=−26 mm. An 8 mm stem bottomed there has centre x=−22 mm, accepted with a 0.5 mm contact margin. The 24 mm heads cannot pass vertically through the 18 mm slots. Raised 5 mm shoulders on the free end holes support the keeper at the proper height when the chain lies flat.

Every native physics step audits every moving solid against x=[−0.19,0.91] m, y=[−0.65,0.65] m and z≥0.7415 m. Any escape or nonfinite state permanently forces zero. There must be exactly twelve audited physics samples per action. Whole-robot clearance uses conservative collision-body bounding boxes; settling includes every link, connector and keeper.

Ruler measurements and overhead/side photos establish insertion, slot displacement, cradle location, guide engagement and clearance. A one-second release video establishes settling. Rated load capacity is not scored.

## Setup, fidelity and reset

`setup-layout.json` lists all fixed poses, dimensions, masses, nominal materials and calibrated cameras. `scene-inventory.json` contains every original body, primitive solid and hinge; `build_scene.py` generates them. The adapter supplies the standard YAM station, table, cameras, stand, floor and lights.

The dark grey former has one machined-looking hub and nine physical guide pins entering the resting link windows, with an open front service sector. Each pin has a 6mm radius, 50mm straight height and 3mm tapered lead, preserving engagement during the 40mm lifting phase. The guide is an assembly fixture rather than a powered sprocket. The eleven 100 mm-pitch links share the same working plane; only the two free end links start splayed. The yellow connector starts on the right side of the table at (0.35, -0.35, 0.7495) m. Its 4mm backing slides beneath the flat chain end plates into the open front of a three-sided cradle at (0.195, 0, 0.839) m. The 160mm initial end-hole gap leaves space for the stems. The two end links have no grasp posts: the central 30mm of their existing dark grey side rails is thickened to 20mm total height for finger contact. The open holes and hinges retain their original plate geometry. The free blue keeper starts on the left at (0.18, 0.35, 0.7515) m.

The reusable printed-polymer links, metal pins and bench fixture use native rigid contact, ten passive hinges and Coulomb friction. The keeper blocks upward removal beneath the pin heads; reversing its lateral slide releases it. The scored outcome is a stationary retained joint.

Reset restores all free bodies and ten passive hinges, opens the grippers at ready and settles for 0.5 seconds with auditing disabled. Scoring begins at t=0 after settling. Nothing is consumed. Final media must show the current loose-part starting layout and exact instruction. The matching hosted run is complete. Canonical metadata is in `task.json`; the matching model/demo videos and initial screenshot are uploaded as media `M-4UKEJ5`.
