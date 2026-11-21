# Switch a push-button bottle to its wide spout

The task is to open an empty dual-mode bottle, draw out its fitted straw, and set the straw beside the upright bottle so the wide spout is clear. On this stock design, the spring presents the mouthpiece when the lid opens. The rest of the straw remains seated until the gripper grasps it and draws it through the guide. The manufacturer describes this operation for drinking or pouring from the larger spout. [Sinoglass Ringleader catalog, PDF page 15](https://www.sinoglass.com/wp-content/uploads/2025/12/2025_New-Catalog_Water-BottleTumbler_compressed.pdf#page=15)

The accepted latch task TK-4D4BN7 tests releasing a safety loop and rocker latch. TK-KAXT6V tests inserting a loose straw through a slitted cup lid. This task starts with the safety ring folded away from the button and measures steady, axial extraction of a retained assembly and the hands-clear outcome. [Accepted latch task](https://tasks.robocurve.org/board?task=TK-4D4BN7) · [Accepted straw insertion](https://tasks.robocurve.org/board?task=TK-KAXT6V)

## Setup and reset

Use the mint 450 mL Sinoglass Ringleader 68624003, a 96 × 82 × 170 mm, 244 g bottle with a clear molded-plastic body and lid, plastic cap, security ring, polymer straw, and silicone carrying strap. The bottle is empty. The optional filter is out. In the real setup, the strap hangs clear of the button. The catalog's 450 mL exploded view shows a removable tube nearly as long as the assembled bottle. The model gives it a 148 mm inlet-to-mouthpiece span and places the intake 9 mm above the vessel base with the lid closed. Both values are estimates from the catalog image and envelope. [Manufacturer catalog, PDF page 15](https://www.sinoglass.com/wp-content/uploads/2025/12/2025_New-Catalog_Water-BottleTumbler_compressed.pdf#page=15)

Use the warm laminate reference table, 110 × 130 × 6 cm, with a tabletop about 76 cm above the floor. The two fixed YAM arms are 61 cm apart. Place the bottle center 35.3 cm forward of their base line and 1.2 cm toward the left base from its midpoint. Face the push button toward the arms. The stock straw is seated, the lid is closed and the ring is folded down. Both grippers start open in the standard ready pose. The only other equipment is the fixed overhead stand and the wrist and side cameras. The setup PNG comes from the exact reset in the recorded run.

To reset, reinsert the clean original straw into the guide and push the lid shut until the catch engages. Fold the ring down. Return the bottle to the position and orientation above and open both grippers in the ready pose. Verify against the setup image. The fixed setup has no random seed. Repeated dry trials need only the original bottle and straw.

## Scoring

| Score | Criterion |
| --- | --- |
| 0.00 | No milestone, or any failure during the trial. Failures are opening without the button, bottle tilt beyond 18°, bottle lift over 21 mm, an item going off the tabletop, an arm strike, or damage in the real trial. A recovered failure stays counted. |
| 0.30 | The button releases the lid, which remains beyond 70° open at the end. |
| 0.75 | Grip the straw while it is still seated after the spring rise and maintain the grasp through at least 120 mm of upward withdrawal. The intake clears the spout by at least 9 mm within 20 mm of the guide axis. At the end, the lid remains beyond 70° open and the complete straw is separate from the spout. |
| 1.00 | Also hold the finished state for the final 1.2 s. The bottle rests on the worktop within 6° of vertical and its lid is beyond 90° open. Both ends of the straw are within 14 mm of the worktop, the straw rests on it with its centerline more than 55 mm from the bottle axis, and the hands are clear of both items. |

Read the button release and continuous grasp from the side and wrist recording. A ruler in view measures the withdrawal and guide clearance. Check the final end heights and lid angle from the side and the axial spacing in the overhead photo. In the simulator, opposing jaw contacts each exceed 0.07 N continuously during the qualifying stroke. That stroke starts within 15 mm of the seated spring extension, with the modeled intake no more than 40 mm above the vessel base. The straw is within 20° of the guide direction during extraction. The recorded data tracks native table support, the held lift and the crossing, and leaves the full score unearned while an arm holds either object. Any inter-arm normal force above 1 N counts as a strike. These contacts serve as machine measurements of the visible actions and outcome.

## Run the task

Use Python 3.12, Mesa EGL for offscreen rendering, and FFmpeg with H.264 support. The following installs the pinned physics and image dependencies into the active Python environment. The reference Dream wheel supplies station code, calibration and licensed YAM geometry. Only its station module is used, so install it separately without its optional simulation and machine-learning dependencies.

```sh
python -m pip install 'numpy==2.3.1' 'scipy==1.17.0' 'mujoco==3.11.0' 'opencv-python-headless-noffmpeg==4.14.0.94rc1'
python -m pip install --no-deps 'dreamlabs==1.0.23'
python run.py --replay commands.npy --out /tmp/bottle-scripted
python render.py /tmp/bottle-scripted
python run.py --mode open-only --out /tmp/bottle-open-only
python run.py --mode extract-only --out /tmp/bottle-extract-only
python run.py --mode hands-off --out /tmp/bottle-idle
python run.py --mode locked-press --out /tmp/bottle-locked-control
```

Use a new output path for each trial. Set `ROBOCURVE_DREAM_YAM` to the directory containing `yam_env.py` and its `assets/` only if using an already extracted copy of the same wheel. The demo replays joint targets from reset. It does not query any object's internal state to select actions. The model advances through contact physics at 1 kHz, with 20 Hz arm actions. The scorer records the contact and state measurements in `trace.jsonl` and writes its result to `summary.json`. The rendering command draws each recorded control state without resampling or changing the physics.

The 750-action limit is 37.5 seconds. The corrected scripted demo scored **1.00** in 564 actions (28.20 seconds), with 139.82 mm of held axial lift, 9.03 mm of intake clearance above the guide, and zero inter-arm contact. The finished state held for 3.02 seconds. The review video shows a fixed side view, overhead and both wrists at the 20 Hz control cadence. The Dream adapter passed its offline check for the corrected geometry. Its live result is pending. [Validation and file hashes](VALIDATION.json)

For the Dream policy, install the pinned Inspect Robots integration from [ENVIRONMENT.json](ENVIRONMENT.json). The offline command checks camera reads, normalized jaw commands, the native action clock and grading without contacting a service. The live command uses the same reset, clock, camera positions and grader, with only the three policy cameras and measured joints sent to the policy. Provide the Dream credential through the environment for the live command.

```sh
python -m pip install 'inspect-robots==0.60.0' 'inspect-robots-yam==0.37.0' 'dreamlabs-client==1.0.14'
python -m pip install 'inspect-robots-dream @ git+https://github.com/kairosiann/inspect-robots-dream.git@b5e936a201cec66c54c297e522c920d98445e339'
python dream_bottle.py --dry-run --out /tmp/bottle-policy-check
python dream_bottle.py --out /tmp/bottle-policy-run
```

Dream-1 scored 0.00 through all 750 steps. Its 82 inference requests used the same 148 mm straw and reset as the scripted demo. The lid remained closed.

## Fidelity and assets

MuJoCo contact advances both freely supported bodies. A passive button slider retracts the lid catch, the lid opens under spring torque, and a localized reciprocal spring presents the straw. Three small elastic retainers then resist the withdrawal. Both arms move with 27 N·m limits on joints 1–3 and 7 N·m on joints 4–6. The manufacturer establishes the operating sequence and nominal size and mass. Internal dimensions, spring curves, plastic friction and exact inertias remain estimates. The model omits the strap and optional filter, and represents an empty undamaged bottle. Fluid flow, leaks and material fracture are outside this simulation. The physical force curves have not been measured.

- The bottle, lid, ring, button, springs, retainers and straw use authored geometry. Catalog photos were consulted as references and are not bundled or shown in the recording.
- The pinned [Dream Labs wheel](https://pypi.org/project/dreamlabs/1.0.23/) supplies `yam_env.py`, oak texture `oak.png`, `sim-eval-yam.json` calibration and `model.json` hierarchy under its [Apache-2.0 license](licenses/Dream-Labs-Apache-2.0.txt) and [notice](licenses/Dream-Labs-NOTICE.txt). Its unlowered optical poses are based on the [pinned YAM calibration](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/yamlab/configs/robot/yam.yaml).
- The same wheel provides the I2RT YAM URDF, collision parts `collisions.npz` and eleven visual meshes `base_link_visual.stl`, `gripper.stl`, `link_1_visual.stl`, `link2_black.stl`, `link2_white.stl`, `link3_black.stl`, `link3_white.stl`, `link_4_visual.stl`, `link_5_visual.stl`, `tip_left.stl` and `tip_right.stl`. [I2RT MIT license](licenses/I2RT-MIT.txt) · [Upstream third-party notices](licenses/Dream-Labs-third-party.md)
