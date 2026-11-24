# Settle a resident’s leg after a bedside cushion adjustment

> My heel is sore. I can lift my leg for a moment. Please settle my leg comfortably with the cushion, then give me room.

## Nursing-assistant work

**Impact on jobs.** Repositioning a resident in bed is a core nursing-assistant duty. Nursing assistants do this work under the supervision of licensed nurses. Here a resident can lift their leg briefly, and an assistant carrying out an assigned heel-offloading plan has time to move a loose cushion. The check comes after the resident relaxes. Has the calf taken the weight, and is the heel still clear? [O*NET, Nursing Assistants](https://www.onetonline.org/link/details/31-1131.00) · [BLS, Nursing Assistants and Orderlies](https://www.bls.gov/ooh/healthcare/nursing-assistants.htm#tab-2)

Indiana’s nurse-aide training curriculum teaches this specific skill in procedure 61, “Float Heels.” The aide supports the calves with a pillow so the heels are free. In this benchmark the resident supplies the brief lift independently. The robot performs the cushion adjustment, and the scorer checks support after the resident lowers the leg. Skin inspection and the other steps of the care procedure are outside the trial. [Indiana nurse-aide curriculum, procedure 61, step 4](https://www.in.gov/health/files/Indiana_Nurse_Aide_Curriculum_Appendix_A_Procedures_51-72.pdf#page=18).

The international heel pressure injury guideline describes supporting the calf with enough cushion height to keep the resting heel clear. In this mannequin trial, the heel can look elevated while still carrying load on the foam. The benchmark measures the immediate placement and timing of this bedside step. Patient outcomes and repeatability on a physical fixture remain unmeasured. [International Guideline, heel chapter](https://www.internationalguideline.com/heels)

## Scoring

Grade the final 41 observations at 20 Hz, spanning 2.00 s. A relaxed, supported leg is essential. Both the mattress and the cushion count as possible heel supports.

| Score | Observable result |
|---|---|
| 0.5 | After assistance ends, the heel carries at most half its own initial load. At least one calf region visibly bears on compressed foam, the cushion rests on the bed, and the arm is free of the person. The Achilles and back of the knee stay clear. The leg and cushion stay settled. |
| 1.0 | In addition, the heel clears every support by ≥10 mm and its single contact channel reads <2 N. Both marked calf regions bear across a span ≥86 mm and both distal arms are ≥25 mm clear. Open or closed parked jaws qualify. |
| 0 | No sustained relief or a latched event. A visible two-jaw grip on opposing surfaces of the same segment, or robot contact with heel loading above 150% of the original value, must persist for 0.50 s to latch. An ordinary bed touch has no independent penalty. |

Calibrate the heel channel and mark six 43 mm foam stations. Each qualifying stripe has ≥3 mm depression from its own tare and lies within 2 mm of the marked calf. Additional numerical limits, model-to-bench correspondence, motion tolerances, fault definitions and the unscorable rule are in the [physical calibration guide](docs/PHYSICAL_CALIBRATION.md). The simulator also records the older detailed force distribution for research comparison. The detailed worksheet runs through the station scorer. The [21-entry worksheet](docs/bench_checks.md) grades the same conditions from observed intervals, extrema and their uncertainty. Both formats require observations across the whole trial.

The contact checks operate on the modeled body segments at 20 Hz. They detect sustained opposing grasps and extra heel load during arm proximity. The model does not measure brief impacts, unilateral force elsewhere on the body, or pressure in clothing or tissue. The hand and trouser details used for appearance are outside the contact model. Half credit allows the robot to touch the cushion. The base-to-bed gap and bearing marks indicate where support occurs. The readout cannot apportion the cushion load between bed and arm. Full credit checks the distal links of both arms. The retained board overview illustrates the placement; the small 1–3 mm bench decisions require calibrated close views.

## Run and score a station trial

Start in this task folder on Linux with IsaacLab-Arena-YAM, its Inspect Robots YAM integration, FFmpeg and Python available. Set `YAM_ARENA_DIR` to the installed arena root if it differs from `$HOME/research/robocurve/IsaacLab-Arena-YAM`. Configure the normal Dream gateway for hosted trials. The task uses two parallel-gripper YAM arms. The scripted controller moves the right arm and can read fixture posture. Dream receives the station overview, wrist views and robot state.

```bash
bash reproduce.sh setup
bash reproduce.sh station-framing bedside-reproduction
bash reproduce.sh station-native bedside-reproduction
bash reproduce.sh station-dream bedside-reproduction
bash reproduce.sh station-qualify bedside-reproduction
```

The framing command runs the nominal and heel-on-foam controls. The native command completes the scripted matrix and scores each trace. The Dream command runs the hosted trial. The last command rechecks saved native records and produces the measured task manifest. A fresh run can perform the same sequence with `bash reproduce.sh station-all bedside-reproduction` after setup. Outputs are under `results/station/bedside-reproduction/`. Keep about 15 GB or more available for the raw camera arrays. Each completed episode covers 14 s of simulated time, with 20 control samples per second and physics at 240 Hz. Initialization, rendering and inference add wall time. The extra video frame is the initial camera sample.

Each trial starts from its fixture’s specified resident offset, cushion position, material and limb mass, with the heel initially loaded and the cushion clear of the calf. A timed mannequin lift and relaxation proceed independently of the robot. Reuse a tag to resume the same settings. Use a fresh tag for a fresh recording.

## Station measurements

| Native station control | Score | Final heel load (N) |
|---|---:|---:|
| Responsive demonstration | 1.0 | 0.00 |
| Slower demonstration | 1.0 | 0.00 |
| Working jaws closed after parking | 1.0 | 0.00 |
| Alternate cushion push point | 1.0 | 0.00 |
| Shorter successful push | 1.0 | 0.00 |
| Insufficient push | 0.0 | 16.91 |
| Idle | 0.0 | 16.91 |
| Arm remains at cushion | 0.5 | 0.00 |
| Incidental bed contact | 1.0 | 0.00 |
| Fixed commands, nominal pose | 1.0 | 0.00 |
| Same commands, +80 mm | 0.0 | 9.24 |
| Same commands, +200 mm | 0.0 | 18.11 |
| Same commands, lift delayed 3.5 s | 1.0 | 0.00 |
| Same commands, lift delayed 4.5 s | 0.0 | 16.85 |
| Responsive policy, lift delayed 3.5 s | 1.0 | 0.00 |
| Responsive policy, lift delayed 4.5 s | 1.0 | 0.00 |
| Softer foam | 1.0 | 0.00 |
| Heavier shin | 1.0 | 0.00 |
| Lighter shin | 1.0 | 0.00 |
| Same commands, +50 mm (follow-up) | 0.5 | 5.33 |

The final load column is a single endpoint sample. Grades use the full terminal dwell and the event history. Five interventions replayed the nominal fixed command sequence exactly. The 80 mm case had an initial half-load target of 8.53 N and remained above it for all 41 terminal observations. The 50 mm follow-up had an 8.56 N target and stayed below it for all 41. It was selected after the original 80 mm hypothesis missed. In an independent standalone CPU PhysX run with approximate gravity compensation, the same +80 mm setting earned 0.5 (9.74 N final heel load against a 9.98 N half-load target). This boundary case varied with the simulation setup. The table reports the recorded HPC01 station results. Small negative simulated gap values in the data indicate contact overlap in the geometry readout. [Station results and action digests](station-results.json)

## Hosted evaluation

Both hosted Dream-1 attempts ended at the first inference after 32 warm-up steps. The provider returned zero model actions, and Inspect assigned no numeric score. The model video contains 33 native camera samples from initialization and warm-up. The separate demonstration shows the scripted full-score trial. The published `no_video_reason` contains the latest error verbatim. [Station receipts](station-results.json)

**Latest provider error**

> DreamAPI refused the inference request: DA-4000 BAD_REQUEST (malformed): unsafe instruction in this scene, refused: Direct physical contact and manipulation of a human leg is unsafe.

**Earlier provider error**

> DreamAPI refused the inference request: DA-4000 BAD_REQUEST (malformed): unsafe instruction, refused: Direct physical contact with a person's leg carries risk of injury.

## Reproducing the physical measurement

The simulated YAM pushes a 380 g cushion with six compliant strips on a 24 mm rigid base. In a separate six-run standalone PhysX test, we disabled arm contact with the base. The reference still moved the cushion 280.46 mm through contact with the foam and scored 1.0. The heel-on-foam and idle controls both scored 0.0. The base still governs sliding and supports the foam. [Test method, results and reproduction](validation/cushion/README.md)

The mannequin supplies its own brief lift. A physical reproduction would use an external sling to assist an inert leg, one heel load channel, marked foam and calibrated views. The guide gives the recorded robot-to-bed registration and commissioning checks. The 820 × 1820 mm surface, arm mounting heights, sight lines and overhead support require facility-specific confirmation. The protocol includes the reset, slack-sling check, timing, camera uncertainty and instructions for marking an unreadable trial. A facility must commission these measurements on the built setup. The two sustained contact faults have boundary tests. Neither occurred in the 20 recorded station controls, which exercise placement and timing failures. Transfer to an ordinary pillow and agreement between independent physical graders remain untested. [Physical calibration and worksheet](docs/PHYSICAL_CALIBRATION.md)

## Third-party asset sources and licenses

| Files used by the task | Source and license |
|---|---|
| Robot description, `task/assets/yam/yam.urdf` | I2RT Robotics YAM v1, commit `120c3c81400171174604e503943f8d1ebc891058`, MIT license. [Pinned robot model](https://github.com/i2rt-robotics/i2rt/tree/120c3c81400171174604e503943f8d1ebc891058/i2rt/robot_models/arm/yam/v1) |
| Nine robot meshes in `task/assets/yam/assets/` — `base.stl`, `link1.stl`, `link2.stl`, `link3.stl`, `link4.stl`, `link5.stl`, `gripper.stl`, `tip_left.stl`, `tip_right.stl` | I2RT Robotics, same revision and MIT license. [Pinned mesh directory](https://github.com/i2rt-robotics/i2rt/tree/120c3c81400171174604e503943f8d1ebc891058/i2rt/robot_models/arm/yam/v1/assets) |

The robot description, meshes and MIT notice match the upstream files byte for byte. The task includes the original I2RT Robotics copyright and permission notice. [Bundled license](task/assets/yam/LICENSE.i2rt) · [Source revision](task/assets/yam/SOURCE_COMMIT) · [Upstream license](https://github.com/i2rt-robotics/i2rt/blob/120c3c81400171174604e503943f8d1ebc891058/LICENSE)

The bed, mannequin, clothing shapes and segmented cushion are generated from the task geometry. The USD fixtures are exports of that geometry, and the camera recordings are simulation outputs. [Scene definitions](task/station.py) · [USD exporter](isaac/build.py)

Native trials also use the separately installed YAM Arena robot USD, wrist cameras and station rig. In the pinned reference implementation its Dream appearance uses four external meshes: `link2_black.stl`, `link2_white.stl`, `link3_black.stl` and `link3_white.stl`. The runtime provenance record traces these splits of the I2RT link meshes to `dreamlabs` 1.0.23 and identifies their MIT license. That record lists all four SHA-256 hashes. The reference runtime builds the floor, lights and camera stand from station definitions. These assets are supplied by the runtime. [Appearance mesh provenance](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/dream/SOURCE) · [Station environment](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/scene.py)

The Arena README attributes the robot model to I2RT under MIT and lists no repository-wide license for the integration. The station recordings identify the runtime by its reported version, without a Git commit for the installed Arena checkout. [YAM Arena licensing note](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/README.md#license)
