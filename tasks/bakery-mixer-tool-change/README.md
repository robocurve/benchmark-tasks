# Refit a bakery mixer beater and verify a guarded start and stop

Refitting mixer tools after cleaning and between batches is routine bakery equipment preparation. This task requires precise tool assembly followed by a guarded start-and-stop check.

## Setup and scoring

`task.json` contains the instruction, complete setup and numeric scoring rubric. `setup.png` shows the native settled reset; `setup-state.json` gives its measured poses. `geometry.json` and `build_assets.py` define the original generic CAD. Units are metres; x forward, y left, z up. The bowl is empty, mounted and clamped throughout.

Scores are 0.25 for a released, retained beater; 0.50 with the released guard seated; 0.75 with the bowl raised; and 1.00 after five seconds of measured powered rotation followed by STOP and a two-second stationary, hands-clear endpoint. Workspace violations or hand intrusion into the rotating bowl latch zero. `task.py` implements the assembly and operational checks; the bundled runtime enforces the endpoint hold.

For physical scoring, use calibrated top/side photographs, a ruler, reference marks and timed video. Measure the hub fit with a caliper/depth gauge while power is isolated, or a calibrated close-up with a scale in the hub plane; the overview cannot resolve submillimetre tolerances. Count tool/carrier turns over five seconds and observe the later stop and two-second hold. Use a tachometer on an exposed shaft reference if coupling cannot be confirmed visually. Physical trials require an empty bowl, functioning interlocks and power isolation during measurement or reset.

## Run and score

Linux, Python 3.12, NVIDIA GPU and FFmpeg are required. Evaluated versions: Isaac Sim 6.1.0.0, Isaac Lab 17.0.2, Isaac Lab-Arena 0.3.0, inspect-robots 0.60.0, inspect-robots-dream 0.1.0 and dreamlabs-client 1.0.14. Use Arena revision `c7edf3a9d5d8276565208f72422f8f8a0806d6cf`; the bundled runtime pins YAM revision `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`. Install that compatible Arena/YAM environment first, then install this runtime in a separate environment. Accept NVIDIA's applicable terms before setting `OMNI_KIT_ACCEPT_EULA=YES`.

From the repository root:

```bash
python -m pip install --no-deps -e tasks/bakery-mixer-tool-change/runtime
python tasks/bakery-mixer-tool-change/run_demo.py tasks/bakery-mixer-tool-change /tmp/mixer-demo --policy full --device cpu --seed 0
python tasks/bakery-mixer-tool-change/verify_trial.py /tmp/mixer-demo --mode full --final-media
```

Use a fresh output directory outside the task folder. The runner saves the receipt, 20 Hz state trace, setup images and three H.264 videos. It must report `completed`, unchanged source and no reset/video errors. `--policy hold` checks the reset; `raised_without_guard` and `guarded_without_lift` test the two start interlocks. The scripted policy uses privileged geometry and only sends robot joint/gripper targets.

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

Export the final evaluation with `inspect-robots video /path/to/final-eval.json --out /tmp/mixer-videos --fps 20`. Scorer regressions: `PYTHONPATH=tasks/bakery-mixer-tool-change/runtime python -m unittest discover -s tasks/bakery-mixer-tool-change -p test_scoring.py`.

## Results and fidelity

Scripted demonstration: **1.0**, 3,540 controls / 177 simulated seconds, 46.8 minutes wall time on an RTX 3090. Seed 0; limit 3,600 controls / 180 seconds. Both negative interlock trials kept the motor off after START presses. Dream-1 beta: **0.0**, 3,600 controls / 180 simulated seconds, 31.6 minutes wall time; one completed trial with no evaluation error. Both runs use identical physical sources and seed 0.

Generic rigid Isaac Sim/PhysX model at 960 Hz. Physical pin/L-slot contacts, a passive spring collar, wire guard and linked lift; masses and inertias are lumped approximations. Independently regulated torque-limited orbit/spin drives use an assumed ratio and 2 s soft start/stop. Internal gearing, food, deformation and electrical faults are omitted. Interlock loss brakes immediately. Hand-clearance checks are operational proxies, not certified injury thresholds.

The videos use the standard calibrated YAM top camera at 1280 x 720 and 20 Hz; the tall housing extends beyond the upper image edge. `sources.json` lists the equipment references. `evaluated-source-hashes.json` identifies byte-identical task, controller and runtime files from the evaluated revision. Packaging adds local import bootstrapping and a unique entry point and removes host-name restrictions from the runner and evidence verifier; it does not alter physics or scoring. The packaged copy passed a native reset plus 100 hold controls and all 34 scorer regressions (`package-verification.json`). The bundled runtime license is in `runtime/LICENSE`.

## Third-party YAM assets and license

Both robot arms and parallel grippers use I2RT Robotics' YAM model from the [pinned YAM asset directory](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam): `yam_arm.urdf`, its converted `usd/yam_arm/` model, the base/link/gripper/fingertip visual meshes in `assets/`, and the convex meshes in `collision/`. The station appearance also uses `dream/link2_black.stl`, `link2_white.stl`, `link3_black.stl` and `link3_white.stl`; their [source notice](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/dream/SOURCE) identifies them as splits of the same I2RT meshes.

These I2RT assets are covered by the **MIT License**, copyright I2RT Robotics: [full applicable license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/LICENSE.i2rt). It permits use, copying, modification, redistribution and commercial use, provided the copyright and permission notices are retained in copies or substantial portions. The assets are provided as is, without warranty; the license disclaims author liability.
