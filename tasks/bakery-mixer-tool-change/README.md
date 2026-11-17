# Refit a bakery mixer beater and verify a guarded start and stop

Category: **Societal relatability**. Fitting the clean flat beater after washing is a familiar part of preparing cake or cookie batter. Under the fixed mixer head, with the bowl already installed, the robot must align the hub with a partly obscured pin, maintain spring compression while turning into the retaining pocket, and release the tool without support. It then seats the guard, raises the bowl and verifies an empty-bowl start and stop.

## Setup, hardware and scoring

`task.json` lists the instruction and fixed setup. `setup.png` shows the native settled reset; `setup-state.json` gives measured poses. `geometry.json` and `build_assets.py` define the original generic CAD. Units are metres; x forward, y left, z up. The bowl starts empty, mounted, clamped and lowered; the beater and guard start on separate supports.

The physical reference is specifically the guarded **Hobart N50-619 / N50CE**. Its [manufacturer listing](https://www.hobart.com.sg/products/food-preparation/mixers/product/n50) distinguishes it from the unguarded N50-604. The [Hobart-authored manual](https://resources.itwfeg.com/sites/resourcecenter/ProductDocumentation/F23138.PDF), printed pages 11 and 14, documents the pin/L-slot mounting, removable guard, bowl lift and agitator spring; page 18 identifies the spring as V-111227. Thus the spring serves a real retaining function. The simulation's passive collar, preload and stiffness approximate that function; they are not measurements of an OEM coupling.

[PHYSICAL_BUILD.md](PHYSICAL_BUILD.md) specifies a full-size, low-voltage empty-bowl build with a **$353.01 parts estimate / $400 planning allowance**, before tax/shipping, labour and already-owned workshop tools. [The fabrication package](fabrication/README.md) contains the actual printable meshes, dimensioned carrier/coupling drawings, stock cut lists and [costed BOM](fabrication/costed-bom.json). Static mesh checks cover the pin/slot fit and the 75 mm rack lift. The spring target remains 5 N preload and 0.4 N/mm; the estimated maximum fitting load is 10.7 N and lift-knob load 16.6 N. These estimates require physical measurement.

The build is unbuilt and uncalibrated. Its printed materials, captive ballast, hidden drive supports and lower nose cover differ from the native model; the build document and fidelity note state those differences. Commissioning requires measured spring force, clearances, grip/knob force, interlocks and ten fit/run/stop cycles before a real robot trial.

[PHYSICAL_SCORING.md](PHYSICAL_SCORING.md) gives the ruler, external-mark and timed-video procedure:

| Score | Observable outcome |
| --- | --- |
| 0.25 | Beater hangs released for one second, bowl lowered and guard away. |
| 0.50 | Guard is seated and released for one second, bowl raised. |
| 0.75 | Assembly remains fitted during five uninterrupted seconds of visible powered beater rotation. |
| 1.00 | STOP, followed by a two-second stationary, hands-clear endpoint. |

Every stage requires the earlier tests. Assembly loss cancels the corresponding proof; unsafe events latch zero. There is no submillimetre scoring band, concealed contact test, exact hub-yaw test or hidden shaft-speed requirement. The original precision guard/lift predicates still model the internal motor interlock; they are separate from scoring.

## Run and score

Linux, Python 3.12, NVIDIA GPU and FFmpeg are required. Recorded versions: Isaac Sim 6.1.0.0, Isaac Lab 17.0.2, Isaac Lab-Arena 0.3.0, inspect-robots 0.60.0, inspect-robots-dream 0.1.0 and dreamlabs-client 1.0.14. Use Arena revision `c7edf3a9d5d8276565208f72422f8f8a0806d6cf`; the bundled runtime pins YAM revision `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`. Install that compatible Arena/YAM environment first, then this runtime in a separate environment. Accept NVIDIA's applicable terms before setting `OMNI_KIT_ACCEPT_EULA=YES`.

From the repository root:

```bash
python -m pip install --no-deps -e tasks/bakery-mixer-tool-change/runtime
python tasks/bakery-mixer-tool-change/run_demo.py tasks/bakery-mixer-tool-change /tmp/mixer-demo --policy full --device cpu --seed 0
python tasks/bakery-mixer-tool-change/verify_trial.py /tmp/mixer-demo --mode full --final-media
PYTHONPATH=tasks/bakery-mixer-tool-change/runtime python -m unittest discover -s tasks/bakery-mixer-tool-change -p test_scoring.py
```

Use a fresh output directory outside the task folder. The runner saves a receipt, 20 Hz state trace, setup images and three H.264 videos. It must report `completed`, unchanged source and no reset/video errors. `--policy hold` checks reset; `raised_without_guard` and `guarded_without_lift` test the two START interlocks. The scripted policy uses privileged geometry and only sends robot joint/gripper targets.

With Dream credentials configured outside the repository:

```bash
inspect-robots run --embodiment bakery_mixer --policy dream \
  -P model=dreamlabs/dream-1-beta -P record=false -P dry_run=false \
  -E task=tasks/bakery-mixer-tool-change -E condition=recoverable -E object_jitter=0 \
  -E control_hz=20 -E cam_width=1280 -E cam_height=720 -E headless=true -E device=cpu \
  --instruction "Fit the clean flat beater to the stopped bakery mixer: align its slotted hub, push it onto the shaft against the spring-loaded collar, turn it into the retaining pocket and release it. Seat the guard on the lowered bowl, raise the bowl with the lift handle, then run the mixer at its preset low speed for at least five seconds and switch it off. Leave both hands clear, the beater secured and the guard fitted." \
  --scorer task_score --seed 0 --max-steps 3600 --store-frames \
  --no-prompt --grader none --no-rerun --no-rerun-save --fail-on-error 1 \
  --log-dir /tmp/mixer-dream
```

Export a completed evaluation with `inspect-robots video /path/to/final-eval.json --out /tmp/mixer-videos --fps 20`.

## Evidence and limits

A fresh native run of the **revised scorer** on 2026-11-17 also reached **1.0
in 3,540 controls / 177 seconds**, with no unsafe event. The independent verifier
checked every recorded metric, 275 qualifying powered controls, the final
40-control hold and all three 1280 x 720 H.264 videos. Final two-second angular
travel was below 0.1 degree for both beater and carrier. The run took 48.4 minutes
wall time on an RTX 3090. [fresh-native-validation.json](fresh-native-validation.json)
records the actual receipt/trace/video hashes and source mapping;
[fresh-demo-final-frame.png](fresh-demo-final-frame.png) is its exact endpoint.
The frozen run snapshot came from `c04199a2`; all 23 evaluated Python/runtime
and native configuration files still match this package. Only documentation
and manufacturing files were added or updated afterward. The uploaded media
remains the original recordings described below, independently rescored under
the same revised rubric.

The original native scripted demonstration reached **1.0** in 3,540 controls / 177 simulated seconds, within the 3,600-control limit, in 46.8 minutes wall time on an RTX 3090. The original Dream-1 beta trial reached **0.0** in 3,600 controls / 180 seconds, in 31.6 minutes wall time, with no evaluation error. Seed 0, the original physical sources and standard calibrated YAM cameras were used for both. `evaluation.json`, `evaluated-source-hashes.json` and `package-verification.json` describe those historical evaluations and their original rubric; they do not assert that the revised scorer ran then.

The revised scorer can be applied to the saved native demo without changing its receipt or trace:

```bash
python tasks/bakery-mixer-tool-change/verify_trial.py /path/to/mixer-job-full-068 --mode full --final-media --rescore
```

The historical demo rescored **1.0** with the revised rubric. The two-second endpoint's total visible angular travel is below 0.1 degree for both beater and carrier. `revision-validation.json` records the replay, unchanged physics/controller checks and current regression results. This is an audit of actual saved native output, not a fresh simulation.

The original Dream-1 trial also rescored **0.0** by direct application of the observable rubric to its exact final video frame: the beater is visibly on its separate draining rack, outside the mounted working band. The current-stage rule clears retention immediately in that state, so every positive stage fails regardless of earlier events. [dream-final-frame.png](dream-final-frame.png) and [dream-video-rescore.json](dream-video-rescore.json) provide the frame, provenance and reproduction command. This is a manual video rescore of the original model trial; it makes no claim about earlier partial stages or a new model inference. Complete Dream object-state traces are not available locally, so no full-trajectory numerical rescore is claimed.

The spring, wire guard and linked lift use native contacts and constraints. Masses and inertias are lumped approximations. Independently regulated torque-limited orbit/spin drives use an assumed ratio and two-second soft start/stop. Internal gearing, food, deformation and electrical faults are omitted. Interlock loss brakes immediately. Clearance checks are operational proxies, not certified injury thresholds.

The existing videos and setup image retain the standard calibrated YAM top camera at 1280 x 720 and 20 Hz; the tall housing extends beyond the upper image edge. External marks and side views specified for physical measurement have not been added to those historical recordings. `sources.json` lists mechanism references. The bundled runtime license is in `runtime/LICENSE`.

## Third-party YAM assets and license

Both robot arms and parallel grippers use I2RT Robotics' YAM model from the [pinned YAM asset directory](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam): `yam_arm.urdf`, its converted `usd/yam_arm/` model, the base/link/gripper/fingertip visual meshes in `assets/`, and the convex meshes in `collision/`. The station appearance also uses `dream/link2_black.stl`, `link2_white.stl`, `link3_black.stl` and `link3_white.stl`; their [source notice](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/dream/SOURCE) identifies them as splits of the same I2RT meshes.

These I2RT assets are covered by the **MIT License**, copyright I2RT Robotics: [full applicable license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/LICENSE.i2rt). It permits use, copying, modification, redistribution and commercial use, provided the copyright and permission notices are retained in copies or substantial portions. The assets are provided as is, without warranty; the license disclaims author liability.
