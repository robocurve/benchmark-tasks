# Captive crossbar docking

Centre the protruding orange captive crossbar, dock its intact blue carrier in the low grey opening, release it and move both hands clear. The rear handle stays outside the mouth.

**Review only.** The frozen final geometry scored **1.0 in both consecutive validation02 epochs and in the standalone final demo**. The standalone demo took **799 steps / 39.95 s simulated time / 186.713 s wall time (about 3 min 7 s on a warmed cache)**. The genuine Dream-1 trial scored **0.0** after all **1600 steps / 80 s simulated time / 256.763 s wall time**, with no execution errors. Its guarded ledger reserved **$0.33 for 33 SDK calls**; this is a conservative reservation, not an invoice. No publication, physical-robot validation, randomized-layout reliability or camera-only control capability is claimed.

## Reproducible scene

All lengths are metres, masses kg; world +x points forward across the table, +y left and +z up. Initial rotations are identity. `captive_crossbar_docking/contract.py` is the source for the committed generated USD assets.

- Two YAM arms have bases `(0,+.305,.750)` and `(0,-.305,.750)`, face +x and start open in the pinned adapter's `READY` configuration. Each has six joint commands and one normalized gripper command. Control is **20 Hz**.
- The oak-appearance table has size `(1.10,1.30,.060)`, centre `(.360,0,.7175)`, top **z=.7475**. The pinned station supplies robot geometry, lights, cameras and default contact materials.
- The carrier is a **free floating rigid link, mass .250**, initially at `(.280,0,.7555)`. Nominal fully docked root is `(.475,0,.7555)`; measured containment determines the score.
- The crossbar is a second rigid link, **mass .040**, connected by an unpowered y-prismatic joint. Its link origin relative to the carrier is `(0,q,.022)`, same rotation. Initially **q=+.050**, joint limits **[-.055,+.055]**.
- The dock is fixed/kinematic at `(.4825,0,.7900)`, nominal bounding size `(.225,.170,.085)`, authored mass 1.0. Free inner volume: **x=.380..585, y=+/-.075, z=.7475..8225**. The table is its floor.

Each row below is one box collider; centres are in the corresponding link frame.

| Link / part | Size (x,y,z) | Local centre (x,y,z) |
|---|---|---|
| Carrier / Body | .180,.140,.016 | 0,0,0 |
| Carrier / RearStem | .040,.024,.016 | -.110,0,0 |
| Carrier / HandlePost | .024,.024,.048 | -.120,0,.024 |
| Carrier / Handle | .024,.024,.032 | -.120,0,.056 |
| Carrier / GuideFront | .008,.140,.024 | .018,0,.020 |
| Carrier / GuideBack | .008,.140,.024 | -.018,0,.020 |
| Crossbar / Bar | .020,.140,.016 | 0,0,0 |
| Crossbar / Grip | .020,.024,.028 | 0,0,.024 |
| Dock / LeftWall | .205,.010,.085 | 0,.080,0 |
| Dock / RightWall | .205,.010,.085 | 0,-.080,0 |
| Dock / BackWall | .010,.170,.085 | .1075,0,0 |
| Dock / RoofFront | .012,.170,.010 | -.0965,0,.0375 |
| Dock / RoofSecond | .012,.170,.010 | -.0625,0,.0375 |
| Dock / RoofThird | .012,.170,.010 | .0125,0,.0375 |
| Dock / RoofRear | .012,.170,.010 | .0965,0,.0375 |

The final 12 mm roof strips have world-x centres **.386,.420,.495,.579**. Their windows expose the orange link. Blue/orange/grey are visual cues. Printed-polymer-like rigid surrogates are intended, but elasticity, wear, fabrication tolerances and material contact coefficients are uncalibrated. Box contact offset=.0005 and rest offset=0. Self-collision within the carrier/crossbar articulation is disabled. The ideal joint supplies captivity; the two guide cheeks illustrate the slide, not a complete manufacturable captive-guide mechanism. A physical rebuild requires an actual captive guide and independent safety/feasibility verification.

White noncolliding cues: carrier depth witness size `(.006,.024,.0004)` at `(-.095,0,.0083)`; centre bands `(.045,.008,.0004)` at `(+/-.055,0,.0083)`; edge bands `(.165,.004,.0004)` at `(0,+/-.067,.0084)`; crossbar grip band `(.0205,.007,.0004)` at `(0,0,.0383)`.

Physics: **PGS, 64 position / 16 velocity iterations**. Joint drive stiffness=0, target velocity=0, damping=.50 N*s/m, maximum force=.40 N. Authored joint static/dynamic friction efforts=.15/.10 N, viscous coefficient=.05. Runtime verifies floating base, one joint, solver, joint limits, zero stiffness, .50 damping, .40 effort limit and .15 reported friction. Dynamic/viscous friction are authored properties, not a physical friction fit. No powered position target, snap, latch, attachment or episode-time object-pose correction is used.

## Numeric grading

Grades describe the **current terminal outcome**, not the historical maximum. Measured carrier/crossbar/dock poses, velocities, slide state, TCPs and gripper openings determine the result; policy phases and saved success flags are ignored. Contact-assisted centring is permitted.

Every nonzero grade requires an intact assembly and:

- **|q|<=.004**, carrier tilt from world-up **<=5 degrees**, and lowest body corner within **.004** of tabletop z=.7475.
- Both link linear speeds **<=.010 m/s**, angular speeds **<=.100 rad/s**, and slide speed **<=.010 m/s**. The larger of reported speed and consecutive measured-pose/joint finite difference is used.

| Score | Additional conditions |
|---|---|
| **0** | Common conditions fail, or any permanent invalid condition below occurs. |
| **.25** | Common conditions hold; no insertion or hand release is required. |
| **.50** | Body world-x extent midpoint **>=.380** (half its x extent inserted). Every payload corner: **|y|<=.0755, z=.7470..8230, x<=.5855**. The entire rear Handle has **x<=.3805**. Every assembly corner with **x>=.3795** obeys the same y/z/back limits. |
| **1.00** | All .50 conditions, plus every payload corner **x>=.3795**; both grippers **>=.80** open; each actual TCP **>=.080 m** from the nearest point on any assembly solid, including rear handle. All full-goal conditions hold continuously **1.00 s**, with sample gaps **<=.075 s**. |

Payload means carrier Body, GuideFront, GuideBack and complete crossbar Bar/Grip. Rear stem/post/handle need not be inside; the handle must stay outside the mouth. Nominal correctly docked Handle xmax=.367. A backwards assembly cannot bypass this check. Containment uses .5 mm geometric tolerance. Interrupted holds restart; an incomplete full-goal hold earns .50 if half-insertion conditions remain satisfied. Default `terminate_on_success=true` ends at the first completed goal, so optional post-goal controller drift is outside the normal endpoint.

**Permanent zero:** malformed/missing/nonfinite measurements; quaternion norm differing from 1 by >.001; gripper outside [0,1]; nonincreasing time/step; incorrect initial layout; moved fixed dock; inconsistent linkage; or any assembly corner below **z=.7275**. First record must be step=0,t=0, root within **.003 m** of start, rotation within **2 degrees** of identity, and q within **.003 m** of +.050. Dock must stay within **.002 m / 2 degrees** of its fixed pose. Actual crossbar transform must match carrier transform of `(0,q,.022)` within **.002 m / 2 degrees**. q outside **[-.0555,+.0555]** is invalid. Comparisons include only the implemented 1e-9 numerical epsilon. Invalid episodes cannot regain credit.

## Install and run

Use an already authorized Linux NVIDIA GPU host with working Isaac Sim/Vulkan support. Inspect `bootstrap_arena.sh` for pinned setup and actual system-library prerequisites on fresh Ubuntu; check the NVIDIA driver first. It installs into `/data` and does not allocate cloud resources, supply private credentials or publish. Existing compatible environments may skip it.

Tested components: Ubuntu **24.04**, NVIDIA **L40S**, driver **580.173**, Python **3.12**, Isaac Sim **6.1.0.0**, Isaac Lab **17.0.2**, Arena **0.3.0**, PyTorch **2.11.0+cu128**, `inspect-robots` **0.60.0**, `inspect-robots-yam` **0.37.0**, `inspect-robots-dream` **0.1.0**, `dreamlabs-client` **1.0.14**. Use these repository revisions as well as package labels:

| Dependency | Revision |
|---|---|
| yam-isaac-lab-arena | `0059d02a1a91bc6834c1d0846022891df0c78730` |
| IsaacLab-Arena | `c7edf3a9d5d8276565208f72422f8f8a0806d6cf` |
| its IsaacLab submodule | `91a0906f93802eb859192dcb0bca879f891e92d6` |
| inspect-robots-dream | `b5e936a201cec66c54c297e522c920d98445e339` |

From this task directory in the pinned environment:

```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --no-deps -e .
TEXT=$(python -c 'from captive_crossbar_docking.contract import INSTRUCTION; print(INSTRUCTION)')
RUN="$PWD/outputs/demo-unique"
inspect-robots run --embodiment captive_crossbar_docking \
  --policy captive_crossbar_docking_demo -P debug_path="$RUN/policy.jsonl" \
  -E demo_observations=true -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 -E output_dir="$RUN/traces" \
  --instruction "$TEXT" --scorer task_score --max-steps 1600 --epochs 1 --seed 0 \
  --store-frames --grader none --no-prompt --no-rerun --no-rerun-save --log-dir "$RUN/logs"
```

For consecutive reset validation use `--epochs 2` and a fresh output path. Dream is metered: privately configure the adapter's API credential and arrange a budget, timeout and resource shutdown controls before launch. Dream uses **no `demo_observations` flag**:

```bash
RUN="$PWD/outputs/dream-unique"
inspect-robots run --embodiment captive_crossbar_docking \
  --policy dream -P model=dreamlabs/dream-1-beta -P timeout_s=300 \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  -E output_dir="$RUN/traces" --instruction "$TEXT" --scorer task_score \
  --max-steps 1600 --epochs 1 --seed 0 --store-frames --grader none \
  --no-prompt --no-rerun --no-rerun-save --log-dir "$RUN/logs"
```

Workspace `ops/t20/run-task.sh` is an optional example of bounded launch, locking, ledger and watchdog controls; this task does not require that private workspace path.

Run cap: **1600 control actions / 80 s simulated time**, seed **0**, zero object jitter. The standalone demo used 799 actions / 39.95 simulated seconds, leaving headroom in the chosen action limit; its measured wall time was 186.713 s on the tested host with a warmed cache. The cap is not a wall-time estimate. Preserve declared control/physics/video timing.

## Observations, reset and evidence

Calibrated top/left/right RGB cameras render natively **640x360**, resized by the adapter to delivered **320x176** policy images; control remains **20 Hz**. Dream receives ordinary images and robot joint/gripper state, without task-object poses, q, metrics or controller labels. The demo explicitly enables privileged measured object poses/slide state via `demo_tools`, then emits ordinary joint/gripper actions. Its success does not establish visual-policy solvability.

Initial-image recognition checks passed. Independent final critical-cue review passed: orange link visible through top windows, blue body apparently in the dock, rear handle outside, hands visibly clear. Exact numeric tolerances, continuous stability from a single frame, and autonomous camera-only control remain unproven.

Reset restores documented root pose, q=+.050, zero object velocities, open robot arms/action buffers, scorer/hold state, trace counters and demo-controller state, then settles. Repeated reset purges old contacts with **.20 m** temporary staging clearance and **.10 s** free dynamics before fresh placement. This is confined to reset before the first scored observation; no episode-time object state writes occur. `reset_audit.json` records stages; runtime verifies settled measured pose/orientation/velocity/slide state. No material is consumed.

Reconstruct grades from raw evidence and run CPU checks:

```bash
python -m captive_crossbar_docking.regrade \
  "$RUN/traces/trial-0000/measurements.jsonl" --output "$RUN/regrade.json"
python -B -m pytest tests -q -s -p no:cacheprovider
```

The **49 CPU tests** cover partial/full thresholds, actual linkage consistency, invalid/drop persistence, backwards orientation, hover, full-assembly release distance, missing-sample holds, forged flags and finite-difference speed. A 1,441-angle yaw sweep rejects the initial offset's envelope and permits the centred one. Synthetic tests verify grader behavior, not physical feasibility.

Per-trial exports: raw `measurements.jsonl`, terminal `metrics.json`, `run_config.json` with privilege flag/physics readbacks/thresholds, `reset_audit.json`, executable/asset `source_manifest.json`, native setup/result images and exact delivered policy images. Inspect action/frame logs are under the run directory. Regrade replays raw measurements through the frozen scorer; it is not an independent physical sensor measurement.

The two consecutive validation epochs and standalone demo used frozen executable/asset manifest revision `019e0245c850b9f646be298526d9edeebe34ef1c8f456b2197efbb396042a53d`. Their historical whole-task-tree receipt hash was `b1c53eeec46c32a24f68feaf3e20d73a545c6927a792fb78b6f34e880da1bc60`; subsequent documentation and bootstrap-script additions do not assert whole-tree equality with that historical receipt. Final evidence must reconcile source hashes, terminal/replay grades, recordings and cost/resource receipts. `task.json.media` deliberately remains empty because this is review only. These three successful fixed-layout trials establish those trials, not statistical reliability or a physical rebuild. The genuine Dream-1 trial scored 0.0 on the same executable/asset revision and completed all 1600 control steps without execution errors. This is one observed trial, not a general model-performance claim.

## Attribution

Original scene and task code, MIT licensed. Runtime and controller scaffolding adapts the MIT-licensed square-cut-line, bevel-angle-transfer, and missing-washer-rework task implementations by their contributors. Isaac Lab Arena and the YAM/Dream adapters are external pinned dependencies; their licenses and notices remain applicable.
