# Realign and secure a robot conveyor's photoelectric detector

The revised fixed-layout RGB-only demonstration reaches 1.0 in 1,034 control steps (51.7 simulated seconds), including the ten-step released dwell. The complete post-motion reset and recorded-score/camera audits pass. A genuine bounded Dream-1 beta trial scored 0.0 in 1,200 steps without a task-validity fault. Both recordings use this revised clamp and camera implementation.

This task represents mechanical commissioning of a retroreflective object-presence detector in a robot conveyor assembly. The useful result is an optical setting that survives fastening and release, verified by an actual blocked/clear output cycle. The conveyor remains stationary throughout. Electrical connection, sensitivity setup and machine isolation are preconditions.

## Running and scoring

`DEPENDENCIES.json` pins the tested simulator, robot, Arena and Inspect versions. Install Isaac Sim 6.2.0.0 and IsaacLab 47.1.0 with Python 3.12.13. Set `ISAAC_PYTHON` to that environment's Python, `INSPECT_OVERLAY` to an isolated package directory, and `ARENA_ROOT` to an isolated Arena checkout. The tested backend is native PhysX on CPU with RTX RGB rendering. Robot assets remain external under their upstream licenses.

Run installation and trial commands from this task folder:

```bash
git clone https://github.com/isaac-sim/IsaacLab-Arena "$ARENA_ROOT"
git -C "$ARENA_ROOT" checkout c7edf3a9d5d8276565208f72422f8f8a0806d6cf
uv pip install --python "$ISAAC_PYTHON" --target "$INSPECT_OVERLAY" --no-deps \
  inspect-robots==0.60.0 inspect-robots-yam==0.37.0 dreamlabs-client==1.0.14 \
  isaaclab-teleop==0.11.3 \
  'yam-arena @ git+https://github.com/kairosiann/yam-isaac-lab-arena@0059d02a1a91bc6834c1d0846022891df0c78730' \
  'inspect-robots-dream @ git+https://github.com/kairosiann/inspect-robots-dream@b5e936a201cec66c54c297e522c920d98445e339'
python prepare_arena_compat.py "$ARENA_ROOT"
PYTHONPATH="$INSPECT_OVERLAY:$ARENA_ROOT" "$ISAAC_PYTHON" "$INSPECT_OVERLAY/bin/inspect-robots" list embodiments
PYTHONPATH="$INSPECT_OVERLAY:$ARENA_ROOT" "$ISAAC_PYTHON" "$INSPECT_OVERLAY/bin/inspect-robots" list policies
"$ISAAC_PYTHON" -m unittest discover -s . -p 'test_*.py' -v
bash run_inspect.sh --policy hold --steps 200 --output runs/hold
bash run_inspect.sh --policy demo --steps 200 --output runs/demo-smoke
bash run_inspect.sh --policy demo --steps 1140 --check-reset --output runs/demo
python scorer.py runs/demo/evidence/episode-00/trace.jsonl
"$ISAAC_PYTHON" audit_run.py runs/demo --require-full
```

These overlay commands assume the installed simulator supplies its scientific and rendering dependencies; `--no-deps` preserves them. Supporting versions include NumPy 2.5.1, Pillow 12.3.0, websockets 14.2 and Torch 2.11.0+cu130. FFmpeg is required for video export. The exact Arena Git revision is authoritative; its source metadata declares version 0.3.0. The compatibility script changes only the isolated Arena checkout.

After the local feasibility and observation checks pass, supply `DREAM_API_KEY` privately in the environment and retain one budget ledger across all paid attempts:

```bash
bash run_inspect.sh --policy dream --steps 200 --output runs/dream-smoke \
  --budget-ledger runs/dream-budget.jsonl
bash run_inspect.sh --policy dream --steps 1200 --output runs/dream \
  --budget-ledger runs/dream-budget.jsonl
PYTHONPATH="$INSPECT_OVERLAY" "$ISAAC_PYTHON" "$INSPECT_OVERLAY/bin/inspect-robots" video \
  runs/dream/inspect/LOG.json --out runs/dream-video --fps 20
python export_media.py runs/dream/evidence/episode-00 runs/model-media --name model_run
python export_media.py runs/demo/evidence/episode-00 runs/demo-media --name demo
```

Replace `LOG.json` with the generated Inspect log. The launcher uses `store_frames=True`, takes an exclusive local GPU lock, refuses existing output directories and imposes a 3,600-second wall timeout. The task limit is 2,000 control steps at 20 Hz, including the Dream adapter's initial image-history warm-up. Physics runs at 2,400 Hz, with material updates at 480 Hz. Evaluation seed is 0 and object jitter is 0. The full-score demonstration took 1,031.3 seconds for the episode and 1,075.9 seconds including initialization and its reset regression. The recorded Dream-1 trial used a 1,200-step evaluation cap within the 2,000-step task limit: 60 simulated seconds, 1,076.5 seconds for the episode and 1,079.4 seconds including initialization. It completed 25 action chunks, with 50 adapter attempts and 24 recovered session expirations. The adapter reported a mean per-step maximum joint error of 0.152 rad and an overall maximum of 0.625 rad; 17 chunks contained out-of-limit model requests, which the embodiment clips to its declared joint limits. Applied arm targets tracked with 0.0045 rad mean absolute error and 0.0562 rad maximum. These are measured run characteristics; the model did not complete alignment. Service latency adds time to model trials.

The persistent spending guard checks the current published model price and reserves USD 0.02 per adapter attempt, including a possible transport retry, with a USD 2.00 cumulative cap across invocations using the ledger. Reservations are upper bounds, not invoices. Account-wide spending outside this ledger must also be counted. Never replace the ledger to restart its allowance.

Each run preserves the executable source snapshot and hashes, Inspect actions and policy frames, native camera frames, evaluator evidence, reset state and process result. `scorer.py` replays every recorded grade and dwell count; that checks telemetry consistency and is not independent physical judging. `audit_run.py` additionally checks frame completeness, native-to-policy image resizing, source hashes and reset velocities. The exported review video stacks the three native views without identifying overlays. Official Inspect videos use the delivered policy frames. All MP4s use H.264, yuv420p and faststart, below 50 MB; `setup.png` is the first overhead frame, below 10 MB.

## Reconstructing the setup

Coordinates are metres, x away from the robot bases, y left and z up. The table top is 0.7475 m; arm bases are at (0, ±0.305, 0.75). All initial velocities are zero. `task.json`, `spec.py`, `scene.py` and the first IK waypoint give the complete fixed layout. All dimensions below are actual size, not scaled for the robot.

The 360 × 360 × 45 mm stationary conveyor frame carries a 330 × 340 × 3 mm belt. A 65 × 45 × 29 mm pedestal and 70 × 45 × 6 mm steel bridge join the sensor post to the frame. The 12 mm diameter, 60 mm long post is vertical. Its centre is (0.330, 0.0273, 0.7890); the clamp's adjustment datum is (0.330, 0.0273, 0.8075).

The selected fastening hardware is the **Omron E39-L261**, with its supplied M5×15 screw and M5 nut. Its specified tightening range is **3.0–3.5 N m**. The body is 23 × 42.3 × 23 mm; its circular plate is 24 mm diameter and 5 mm thick. The post allows yaw and height adjustment; the sensor plate pitches about a horizontal axis. The M5 screw axis is horizontal and perpendicular to both. Four rigid boxes approximate the clamp body's post bore. Native joints reduce the groove and post contact constraints; they do not snap to any alignment.

A custom 30 × 4 × 24 mm steel adapter connects the supplied plate to a SICK-style bracket: 30 × 23.5 × 2 mm foot and 2 × 19 × 46.5 mm upright. This is explicitly a custom adapter, not a claimed manufacturer-listed SICK–Omron combination. The sensor envelope is 42 × 15.6 × 48.5 mm. Plate, adapter, bracket and sensor total 155 g; clamp body is 70 g. The sensor has a physical 5 mm received-light indicator. The reflector is 4 × 18 × 60 mm, centred at (0.730, 0, 0.83275). The electrical connection, sensitivity setting and isolated, stationary conveyor are preconditions.

The torque wrench has an **8.1 mm AF, 12 mm deep socket with 0.3 mm entry bevel**, a 12 × 12 × 22 mm square shaft and a 36 mm diameter × 10 mm spring head with a 60 mm diameter × 1 mm indicating dial. The independent lever is 120 × 12 × 8 mm; its rectangular 35 × 20 × 20 mm grip is 120 mm from the axis and 45 mm behind the socket mouth. Socket/head mass is 100 g; lever/grip mass is 120 g. The output and handle indexes are black and yellow. An elastic spring couples them at **10 N m/rad** with 0.10 N m s/rad damping. A black dial pointer magnifies this relative rotation at 90° per N m; its green band is **3.1–3.4 N m**. The dial represents ordinary internal pointer gearing, not a success light. The required physical instrument accuracy is ±0.07 N m, leaving the entire green band within the clamp's 3.0–3.5 N m requirement. These are dimensioned tool specifications, not a claim to reproduce a particular commercial CAD model. The 120 mm lever requires approximately 27 N tangential hand force at its calibrated torque.

The right gripper starts holding the rectangular wrench grip, with the socket already seated on the loose M5 nut at a 1.5 mm inward mouth depth. Its axis follows the initial -14 degree clamp yaw and the lever points upward. The hand holds the wrench through native contact. A native sliding keyed constraint approximates positive engagement between the seated socket and nut. This reset isolates the alignment-and-fastening operation; tool selection and socket pickup are preconditions. The right hand may retain the wrench away from the detector during the released optical check. The orange 40 x 40 x 60 mm, 45 g block starts at (0.250, 0.180, 0.7785), on its 75 x 75 x 1 mm green pad.

### Repeatable loose reset and mechanical approximation

For a physical trial, support the sensor with a setup jig; loosen the M5 nut, set the published height, yaw **−14°** and plate pitch **0°**, then tighten to **0.10 ±0.01 N m** with a calibrated low-range instrument. Mark the nut's zero index and remove the jig. This is a reproducible light seating preload, not a quarter-turn gap or an assumed secure state. Verify the indicating wrench's zero and calibration before the trial. Hold the output in a calibration fixture and apply tangential force at the 120 mm grip datum using a force gauge: 25.000, 26.667 and 29.167 N represent 3.0, 3.2 and 3.5 N m. Approach loads slowly, verify readings within ±0.07 N m and check return to zero. A spring cartridge and calibrated geared dial implement the specified tool; internal spring and gear geometry are reduced in the simulation. No physical instrument has been built or measured for this submission.

The reduced thread law uses 0.8 mm advance per revolution, 40 MN/m effective compression stiffness, a 0.20 nut factor and 5 mm bolt diameter. Initial force is 98 N, making initial thread resistance 0.10 N m including a 0.002 N m residual term. Clockwise rotation raises compression; preload is capped at 16 kN. The nominal 3.20 N m point is about 34.875° from this reset, but **that angle does not award fastening credit**. A five-degree turn produces only about 0.54 N m; the old 95° threshold would exceed the new hardware's limit.

Native force-capped elastic friction resists plate rotation, post yaw and post sliding. Friction is 0.25; effective plate friction radius is 10 mm and post radius is 6 mm. Estimated elastic microslip stiffness is 1,000 N m/rad rotationally and 1 MN/m axially, with damping 0.2 N m s/rad and 10 N s/m. When actual displacement exceeds resistance/stiffness, the local rest coordinate follows that slip; it never receives a desired alignment or scoring state. This passive return-mapping law avoids the idle drift observed with damping-only constraints. Rotational values are converted to USD's per-degree convention. The nut has the same native, torque-capped friction model, and the independent wrench bodies have a native elastic torque spring. Its maximum-force setting is 100 N m, well above the scored working range; it is an indicating instrument, not an overload-limiting clutch. The seated hex interface is reduced to a native sliding key: transverse clearance is ±0.05 mm, angular clearance is ±0.5°, and axial motion is free. It releases when the socket mouth passes 0.75 mm beyond the nut centre, where its entry bevel clears the loaded hex faces. Re-engagement requires 1–2 mm inward mouth depth, radial error at most 0.05 mm, aligned axes and hex phase within 0.5°; no body pose or velocity is assigned. The keyed interface replaces fine hex contact while engaged, and ordinary collisions remain after withdrawal. No tool is attached to a hand. The torque witness rejects direct hand/prop contact with the nut, as well as any external bracing of the wrench. The overload history records torque only while the socket is engaged, so free-tool acceleration does not count as clamp overtightening.

Compression stiffness, friction, nut factor, masses and contact details are engineering approximations, **not physical calibration measurements**. The real tightening requirement comes from the selected clamp's own manual. Thread helix, axial nut motion, pressure distribution, clamp groove compliance and wiring geometry are omitted; fine hex-face contact is replaced by the sliding keyed interface. A passive cable load uses 0.05 N m/rad stiffness and 0.001 N m s/rad damping, capped at 0.03 N m. The cable is not manipulated.

Workpiece friction is 0.25 with zero restitution. Finger pads use upstream rubber-like estimates of 1.2 static and 1.0 dynamic friction and a 2 mm torsional patch radius. These are native contacts with no grasp latch. The YAM keeps its native joint/effort limits, 75 mm aperture and 40 N finger limits. Physics runs at 2,400 Hz with TGS, 255 prop position iterations and zero velocity iterations. Friction and the dial update at 480 Hz through a wrapper around the actual native physics step, with a recorded substep-count assertion. The native-step wrapper makes the material-update count explicit and auditable. The 480 Hz physics configuration failed the keyed-tool calibration and was rejected; 2,400 Hz passes.

Nine rays approximate optical return using the sensor's documented 25 mm spot at 1.5 m. Seven returned rays turn on the physical receive LED. Native scene queries check occlusion by the robot, block and fixtures. Reflector incidence is limited to 30°. Transparent-object attenuation, reflectivity variation and full sensor electronics are omitted.

### Numerical load-path calibration

`calibrate_tool.py --output NEW_REPORT.json` runs an isolated native-physics test using the same scene geometry. A temporary revolute support steadies the handle; the socket engages the free M5 nut through the same sliding key as the task. With nut resistance held at 3.4 N m, external torque is ramped over 0.5 s and held. The 2,400 Hz / 480 Hz configuration reads 3.011, 3.213 and 3.518 N m for applied 3.0, 3.2 and 3.5 N m respectively. The nut remains within 0.19° below resistance and turns at 3.5 N m. These are numerical calibration results, separate from a robot demonstration and from physical hardware measurements. Step-load transients are not used as static calibration.

## Observations and demonstration

Policies receive only `overhead_cam`, `left_wrist_cam` and `right_wrist_cam`, through Inspect names `top_cam`, `left_cam` and `right_cam`. The fixed camera is at (-0.12,0,0.87) m, aimed at (0.45,0,0.83) m; both wrist-camera mounts remain standard. These cameras render 640 × 360 RGB; actual policy frames are bilinearly resized to 320 × 176 at 20 Hz. Dream also receives standard robot joint/gripper readings. Object poses, contacts, optical-ray diagnostics and score remain evaluator-only data.

The scripted demonstration accepts RGB only and uses a fixed offline IK rehearsal for this zero-jitter layout. It includes offline compensation for the specified wrench payload and a prescribed tangential tightening force up to 27.5 N using nominal mass, geometry and robot joint stiffness; it does not query object poses during execution and is not a general visual servo policy. Yellow-pixel observations are recorded for audit, not used as hidden phase feedback. Every IK waypoint must meet a 3 mm position and 0.02 rad orientation residual gate before execution.

The successful episode was checked in native and actual 320 × 176 policy frames. The fixed view exposes the torque pointer and green band during the fastening witness; the gripping arms occlude some other views. After tool withdrawal, the right wrist aims at the received-light indicator while the left arm performs the block test. The physical LED is visibly on, off with the block alone, then on after block return and release. At policy resolution it occupies about 5–6 bright yellow pixels in the sampled released views, so it is a small but visible cue. No persistent z-fighting was observed in these sampled views. Camera buffers are explicitly invalidated after physical reset and after LED/dial updates, preventing a stale first frame or one-frame indicator lag.

## Physical judging and score

Use the calibrated indicating wrench, the three videos, a ruler, 5 mm clearance gauge and a timed close view of the dial and output index. The primary fastening witness is **a calibrated torque reading while the output remains stationary**, followed by retained optical operation. Nut rotation alone and a hidden preload estimate alone cannot earn 0.5 or 1.0.

- **0.25:** the detector's received-light LED is on.
- **Fastening witness:** the socket is seated, LED is on, and the calibrated dial stays in the 3.1–3.4 N m green band for at least 0.5 s while nut angle varies by at most 1°. The socket must have no contact except the nut, and the handle no contact except the gripper; a braced or jammed wrench cannot prove fastening. The indicator is derived from its actual elastic deflection. A thread-angle preload estimate does not award or gate this witness.
- **0.5:** fastening witness completed, LED on, and all robot parts and the tool at least 5 mm clear of the detector, bracket, plate and clamp.
- **1.0:** at that released setting, receive light for 0.5 s; then block the beam with the orange block alone for 0.5 s, LED off. Return the block wholly inside its pad, bottom within 3 mm of pad top, and release it with at least 5 mm robot clearance. All terminal conditions and LED on must then hold for ten consecutive 20 Hz steps (0.5 s).
- **0.0:** no current qualifying stage, or permanently after a fixture/shortcut fault: chassis tilt over 15°, dynamic objects outside the stated workspace, bracket separation at least 2 mm, fixed frame or reflector movement over 2 mm, or indicated tool torque above 3.5 N m while the socket is engaged. Calibration and visible tool bypass/damage checks accompany physical judging.

Workspace bounds are x=[−0.19,0.91], y=[−0.65,0.65], z=[0.7375,1.25] m, checked against whole-object surfaces. Cube/mesh vertices and analytic transformed-cylinder extents determine these bounds; a tilted circular dial is not treated as its enclosing square. Per-object bounds are retained in the trace. Regripping, tool contact or a setting change over 1° clears the optical proof. Loosening the nut over 1° after the fastening witness clears that witness. A hand or tool blocking the beam is not a detection test. The current score is zero during legitimate block testing; its required history persists. Loss of a terminal condition removes full credit.

Simulator measurements implement these physical conditions but are not policy observations. The real judging procedure and preload estimates have not been calibrated on physical hardware.

Reset rebuilds bodies, native constraints, physics views and cameras; restores all poses, joints and zero velocities; reinstates the 0.10 N m loose preload and seated sliding-key state; and clears friction rest coordinates, nut/tool history, dial pointer, peak torque, material-update phase, witness windows, optical proof and dwell counters. The regression compares complete prop and robot states and initial camera hashes, checks cleared histories/timestamps, then runs a 20-step zero-score hold.

## Sources and distinctness

The [Omron E39-L261 instruction sheet](https://www.omron-ap.com/data_pdf/mnu/e39-l261_inst-5630532-1a.pdf?id=3779) specifies the supplied M5×15 screw/nut and **3.0–3.5 N m tightening range**. The [E3AS series dimensional catalog, printed page 50](https://files.omron.eu/downloads/latest/datasheet/en/e3as_series_datasheet_en.pdf?v=1) dimensions the E39-L261 and 12 mm support posts. The clamp's published requirement replaces the former unsupported angle-only fastening threshold.


The [SICK operating instructions](https://www.sick.com/media/docs/4/34/634/operating_instructions_wl12g_3_en_de_fr_pt_it_es_ja_zh_im0035634.pdf) describe alignment using received-light indication and the blocked/clear test. The [WL12G-3B2531T01 datasheet](https://www.sick.com/media/pdf/9/89/389/dataSheet_WL12G-3B2531T01_1041458_en.pdf) supplies the envelope, mass, nominal beam spot and compatible bracket listing. The [BEF-WG-W12 drawing](https://cdn.sick.com/media/pdf/2/32/432/dataSheet_BEF-WG-W12_2013942_en.pdf) supplies the stainless bracket envelope and slotted-base reference.

A [documented robot-unloading application](https://www.bannerengineering.com/br/pt/applications/other-applications/positioning-pallets-for-unloading-by-robot-arms.html) uses retroreflective detectors along conveyor rails to trigger robot unloading. It supports the robot-cell integration context, not a claim that this application uses the modeled sensor.

The distinctive outcome is restoring optical detection and retaining it through fastening and an output test. Clearing cartons from a fixed photo-eye beam, connecting an M12 sensor cable, and setting laser focus height are nearby operations. Holding and turning a fastener alone would not establish distinctness. The live board was screened before initial submission and refreshed against 4,034 unique task IDs on 20 November 2026, including accepted and pending entries. LIBERO, SimplerEnv, RoboCasa, BEHAVIOR-1K, RoboDojo and DataCenterBench were also screened. RoboDojo’s screw-fastening task is a nearby constituent skill; optical alignment retained through fastening and an output test is the additional operational requirement. This is a screening judgment, not a guarantee of reviewer acceptance.

Original task code uses the accompanying MIT license. The external **YAM robot USD and mesh assets** are supplied by [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam), revision `0059d02a1a91bc6834c1d0846022891df0c78730`. These robot assets are **MIT licensed**; retain their upstream [LICENSE.i2rt](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt). The assets are loaded externally and are not redistributed in this task. Simulator dependencies retain their respective upstream terms.
