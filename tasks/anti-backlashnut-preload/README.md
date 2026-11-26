# Reset a mounted anti-backlash lead nut's cam preload

The real operation is the Novanta manufacturer's recommended mounted-nut preparation: turn the stainless cam down the ramps until play is felt, then slowly allow its torsion spring to unwind and restore preload. This task represents that operation on a real-size MDrive23-family flanged nut used to build a robot's screw-driven stage. The mounted screw stays stationary. Nut installation, screw rotation and ballnut transfer are outside this task.

The robot manipulates the original20.8mm cam body. No enlarged handle, substituted slider or powered adjustment is provided. The front nut is already flange-mounted on an ordinary upright90mm actuator-carriage support at(0.30,0,0.8375)m. The203.2mm screw remains full diameter9.525mm; no object is uniformly scaled to fit the gripper. `task.json` and the relative local asset's `parameters.json` give all parts, poses, material estimates and numeric scoring rules.

`make_environment` preserves the stock YAM rig/gains and creates native independent bodies rather than an articulation, because PhysxLimitAPI soft limits do not operate on articulation joints. The cam has a native passive revolute spring. Three literal wedge ramps contact followers on the rear nut. Its native axial soft-limit joint represents already-engaged opposing thread flanks, including60µm total lost motion and finite compliant loading. `dynamics.py` is a read-only tensor/contact observer. There are no pose/velocity writes, commanded task joints, fake attachments, event releases or task-body forces. Only the robot's ordinary stock contacts can turn the cam.

The estimated dry friction permits a manually forced excessive preload to remain stuck at reset. Moving down a ramp unloads the thread flanks; releasing from below lets the spring load them again. This hysteresis and absence of spontaneous success require native validation; pure algebra or a setup image cannot prove it. The submitted YAM trajectory reaches the full score.

## Primary data and approximation boundary

- [MDrive23 manufacturer datasheet](https://www.novantaims.com/downloads/datasheets/MLM23.pdf), pages4–5: exterior nut/screw dimensions,1–3oz-in drag and25lb load limit. The12.7µm repeatability number is not asserted as a backlash tolerance.
- [Manufacturer mounted-nut correction and assembly procedure](https://novanta.com/robotics-automation/installing-anti-backlash-nut/): cam down-ramp/unwind, spring tangs, front/back nuts, washer and indexed ramps. No threading/assembly step from that separate procedure is claimed as completed here.
- [Thomson ActiveCAM description](https://www.thomsonlinear.com/en/training/lead-screws/xc) confirms the cam/torsion-spring bias mechanism; its dimensions are not substituted for the Novanta nut.

The external envelope is manufacturer-based. Internal ramp rise, spring constant, coil equivalent, masses, axial clearance/stiffness and contact friction are declared estimates. They have not been measured on OEM hardware. The fixed flange/carriage is appropriate only while adjusting the cam with the screw stationary. The axial joint is a local thread-flank approximation; it cannot reproduce thread pitch advancement or nut removal. Estimated drag is `thread_friction * elastic_axial_load * screw_radius`, with the elastic load inferred from actual native soft-limit deflection. It is not an executed native rotational torque test. The model's predicted drag window therefore needs physical checking before claiming OEM actuator performance.

## Score and hardware counterpart

The scorer has0/.25/.5/1 tiers, the published25lb axial limit, the original-ramp interval(-42,+78)deg and a2s hands-clear endpoint. Detailed thresholds cover initial, moving, partially unloaded, improperly loaded, released, missing/nonfinite and over-limit states in `task.json`/`scoring.py`. Contact is native; missing robot state never defaults to released. The partial unloaded state rejects direct rear-nut contact. Full credit depends on the released proper preload, not a counted hand sequence.

For hardware, photograph the original ramp index and measure the gap ratio with a magnifier/scale. With the robot clear, use an appropriate mN·m torque gauge on the unpowered screw while its carriage can follow the5.08mm lead, recording steady low-speed drag over the same short travel. An alternative is a50mm lever and0.141231–0.423693N tangential pull, measured with a sufficiently sensitive force gauge; bearing/tool tare must be subtracted. Video and a clock measure2s release/settling. Native inferred drag and actual hardware torque are different evidence types. This physical assay/preparation is documented but has not been executed; no hardware calibration or feasibility result is claimed.

## Reproduce/reset

Use the workspace's pinned `yam-inspect-isaac:6.1.0` runtime and standard task loader. `task.py` supplies `make_environment`, `make_dynamics`, `read_object_states`, `make_scorer` and an empty, unexecuted `demo`. CPU PhysX uses1ms physics steps and50 substeps per20Hz control. Recreate the scene, restore the described excessive initial preload and clear the scorer; the model uses no consumables. In hardware record the actual excessive initial drag rather than treating the estimated native24N as a manufacturer's reset specification.

Run pure source/scorer checks with `python3 -m unittest discover -s tests -v`. Passive native reset and images are separate evidence under `runs/anti-backlashnut-preload/`; static tests are not a physical robot demonstration. 

## Asset license

`assets/anti-backlashnut-preload/` is a portable copy of the original reusable package, with only relative references. Geometry and code are original CC0-1.0; see its `LICENSE` and `PROVENANCE.md`. No third-party CAD, textures or photographs are included. Manufacturer links establish physical facts and remain owned by their authors.

Native reporting uses a task-owned ContactSensorCfg created after the PhysX manager configuration and before scene warmup. The pinned SDK otherwise sets /physics/disableContactProcessing to true during initialization, overriding a pre-build setting. The task refuses missing or disabled reporting rather than treating it as zero contact. This changes reporting initialization only; all physical geometry, material properties, fracture thresholds and scoring are unchanged. The current-source passive initial check below confirms enabled reporting; the subsequent loaded trial is reported below.

The earlier spring revision was captured after contact-report initialization was corrected. The 31-control check completed with finite measurements and contact processing enabled, without failure and with primary score zero. Initial rear-nut deflection was 0.14717 mm; the material model estimates 23.4339 N axial preload and 0.0390614 Nm drag, not measured hardware forces or torque. The later submission trial below demonstrated unloading and controlled return.

For hardware reset, use the same tared low-speed shaft-drag assay to prepare 35–45 mN·m initial steady drag (0.035–0.045 Nm), above the proper 1–3 oz-in window. This is an original benchmark preparation band, not a published manufacturer reset specification or an executed calibration. Restore the stated original ramp index and stationary-screw robot setup after preparation; do not infer the hardware value from the simulated 23.4339 N axial-load estimate.

## Passive spring estimate

Three hands-clear trials of the earlier 0.006 N·m/rad estimate returned shaft-drag estimates with a median of 0.02954 N·m. The manufacturer range is 0.00706–0.02118 N·m. Scaling the estimated torsion rate by nominal midrange drag divided by measured median gives 0.0029 N·m/rad. This is a first-order calibration of the simulated mechanism. Geometry, ramp friction, thread compliance and the published drag criterion are held fixed for the comparison. A hardware shaft-drag test would be needed to validate the spring response.

## Recorded submission trials

Dream-1 run: 0 after 129 steps (6.45 simulated seconds); termination: DreamSessionExpired:expired. Scripted/teleoperated demo: 1 after 979 steps. The measured waypoint trajectory reached the full score in 48.95 s. Replaying all 979 controls gave identical scorer output.

## Score the recorded trial

The demo lasts 48.95 s at 20 Hz. From this folder in the pinned YAM simulator Python environment, replay its recorded measurements through the task scorer with the following commands. The first command decompresses the trace. The second checks the measured states and reports the final score.

```sh
gzip -dc runtime/recorded-demo.jsonl.gz > /tmp/nut-demo.jsonl
python runtime/rescore.py --task task.py /tmp/nut-demo.jsonl
```

Use `python run.py --policy dream --episodes 1 --seed 0 --output-dir /outputs --viz none` inside the configured Isaac Sim image for a new model trial with the account ledger mounted.

## In-trial displacement and load scoring

During the physical trial log rear-ring axial displacement relative to the fixed flange with a 0.5 µm or finer displacement gauge, synchronized to each 20 Hz control sample. Define the zero at the midpoint of the unloaded 60 µm axial dead band, positive toward the loaded rear flank. Grade the same declared elastic load estimate as in simulation: 200000 N/m times the amount by which displacement exceeds +30 µm, or zero below that point. At a sampled displacement up to +35 µm the estimate is at most 1 N; above +586.0275 µm it exceeds 111.2055 N and permanently zeros the score. Use the simultaneous side-camera cam-angle and finger-clearance observations. The published limit is used with this explicit estimate; it is not a direct force measurement.

Before setting the initial drag, release ramp pressure and take the two rear-ring first-contact positions with a low-force axial indicator sweep at the fixed screw. Set their midpoint as zero; verify 60 ± 2 µm free travel. Keep the gauge referenced to the carriage as the over-tightened start is set. Log at 100 Hz or faster and use the sample at each 50 ms control boundary. Tare the force gauge and retain the before/after shaft-drag readings.

## Detailed fidelity and fixture reset

Original full-size compound convex geometry and native CPU PhysX1kHz with20Hz controls; stock YAM forces/gains. Actual cam-ramp/follower contacts transfer load, and a native revolute spring returns the cam. Spring values are converted from SI/radian to USD per-degree units. A native NON-articulation prismatic soft limit represents the already-engaged screw's local axial clearance/compliance and nested nut rotation constraint. This stationary model deliberately does not represent threading, lead advance, screw rotation, rolling balls or a complete robot actuator. Internal dimensions, masses, stiffness and friction are engineering estimates, not OEM hardware calibration. Drag is inferred from the modeled static flank load, not read from an executed shaft torque sensor. No dynamics code applies forces, changes joints, writes poses/velocities, attaches objects, or releases anything on a timer/command; dynamics reads native state/contact only. Circular instead of three-lobed flange, faceted ramps and visual screw-crest rings are declared geometry approximations. All asset geometry/code are original CC0; linked manufacturer documentation is not redistributed. A task-owned native contact sensor enables reporting after the pinned SDK initialization; a strict managed-setting check refuses disabled or absent contact processing. No material, geometry or score threshold changes accompany this reporting correction. The estimated torsion rate is fitted to nominal released drag using prior simulation measurements. The OEM spring rate and hardware response have not been measured.

Recreate the scene and scorer for every trial. Restore the same mounted front nut, stationary screw, cam mark0deg and rear root to its listed+0.150mm preloaded displacement; keep the same original ramp index. Clear all score history. Nothing is consumed. In hardware, an assembler may produce excessive drag by inadvertently twisting the cam during mounting, as the manufacturer states; use a torque gauge to record the actual initial resistance, and do not pretend the simulation's estimated24N preload is an OEM setting. End the trial before any post-trial hardware torque assay, which must not be performed with the robot holding the cam. For hardware reset, use the same tared low-speed shaft-drag assay to prepare 35–45 mN·m initial steady drag (0.035–0.045 Nm), above the proper 1–3 oz-in window. This is an original benchmark preparation band, not a published manufacturer reset specification or an executed calibration. Restore the stated original ramp index and stationary-screw robot setup after preparation; do not infer the hardware value from the simulated 23.4339 N axial-load estimate. Before setting the initial drag, release ramp pressure and take the two rear-ring first-contact positions with a low-force axial indicator sweep at the fixed screw. Set their midpoint as zero; verify 60 ± 2 µm free travel. Keep the gauge referenced to the carriage as the over-tightened start is set. Log at 100 Hz or faster and use the sample at each 50 ms control boundary. Tare the force gauge and retain the before/after shaft-drag readings.
