# Clean your wrist camera

Dry workshop dust on a robot's wrist-camera protective window obscures its close-range workspace view. Restoring that view is a sensing-maintenance step during assembly and servicing work; this task tests whether the robot can retrieve a brush and clean its own mounted camera using its other arm. The left camera remains mounted throughout the task.

## Setup

The fixed scene contains a bimanual YAM, its top and two wrist cameras, a mounted camera protective window, an upright brush and holder, and a printed test card on the table. `task.json` specifies dimensions and materials. `layout.json`, `initial_state.json` and `camera_mount.json` specify poses and mounting geometry. The procedural fixtures are built by `window.py`, `brush_geometry.py` and `test_card.py`; `test_chart.png` is the printable 260 × 180 mm card.

The protective glass front is 0.6 mm above its retaining bezel. The brush is 117 mm long and weighs 18 g. Its 42 × 44 mm holder stands 32 mm above the tabletop. The supplied starting arm poses place the camera within reach of the empty right hand. The policy receives the three native RGB views and joint/gripper state.

## Runtime and commands

Use the pinned YAM station environment identified in `dependencies.py`, with its Isaac Lab/Arena and Isaac Sim runtime. The task uses NumPy, SciPy, Pillow, PyTorch and Warp. Model evaluation uses the Dream and Inspect Robots adapters; video export uses `ffmpeg`. Run commands from this folder after activating that environment. Set `DREAM_API_KEY` in the private shell environment for model evaluation.

```bash
python test_contact_model.py
python test_scorer.py

python run_demo.py --out runs/setup --setup-only
python run_demo.py --out runs/demo
python render_native.py runs/demo --out runs/demo/demo.mp4
python render_assay.py --source runs/demo --out runs/demo_images
python score_images.py runs/demo_images
python scorer.py --trial runs/demo/result.json --images runs/demo_images/image_quality_score.json --out runs/demo/score.json

python run_dream.py --out runs/model
python render_native.py runs/model --out runs/model/model_run.mp4
python render_assay.py --source runs/model --out runs/model_images
python score_images.py runs/model_images
python scorer.py --trial runs/model/result.json --images runs/model_images/image_quality_score.json --out runs/model/score.json
```

Start each trial in a fresh process and output directory. Physics runs at 240 Hz and policy control at 20 Hz, with seed 42. The common limit is 1,800 steps, or 90 seconds. The recorded demonstration uses 1,203 steps (60.15 seconds); the model run uses all 1,800 steps. Reference trial wall times were approximately 11–12 minutes each, followed by the separate image assessment. Dream executes eight action rows per request and records session continuity.

## Score and image assessment

| Highest condition achieved | Score |
|---|---:|
| Before confirmed brush pickup | 0.00 |
| Confirmed brush pickup | 0.25 |
| Pickup, verified brush-head wiping and final image-error recovery ≥25% | 0.50 |
| Pickup, verified brush-head wiping and final image-error recovery ≥50% | 0.75 |
| Pickup, verified brush-head wiping and final image-error recovery ≥90% | 1.00 |

Pickup is latched after the brush rises at least 40 mm above its rest and remains stably carried for 0.5 seconds. The simulator checks a closed gripper command, at most 15 mm gripper-relative position error and 10 mm relative spread over that interval.

Recovery credit requires visible brush-head wiping after pickup. For a physical trial, record the window and the tool from the side so that the assessor can see the bristles make contact and translate across the exposed glass. If the gripper or its protective cover directly wipes the glass at any time, cap the score at pickup credit even if the brush was also used. Incidental contact with the bezel is allowed. In simulation, only tangential brush-face contacts with the glass body can change the contamination layer, and the scoring script requires evidence of that contact and removal.

For a physical trial, record the assessor's findings as JSON with boolean fields `grasp_achieved`, `verified_brush_wipe` and `direct_gripper_wipe`. Set `verified_brush_wipe` to false when a brush stroke cannot be verified. Score those annotations and the same image assay with `python scorer.py --physical --trial physical_trial.json --images image_quality_score.json --out score.json`.

After the trial, the evaluator standardizes camera pose, target position, lighting, exposure and focus. Five frames are averaged for each of the clean reference, dirty initial state and final state, using the same full test-card interior ROI. The robot's action budget covers the manipulation; the evaluator performs the measurement setup. The final photographs determine the recovery tier.

With RGB values scaled to [0,1], let `E` be mean squared error to the clean reference over the ROI. Recovery is `1 - E_after / E_dirty`. The initial measurement is accepted when `E_dirty > max(10 × E_clean_repeat, 1e-8)`. Repeated-clean and repeated-dirty photographs accompany each assessment. The fixed ROI and final-state rule are shared by model and demonstration scoring.

The shipped photographs can be rescored directly:

```bash
python score_images.py measurements/demo
python scorer.py --trial measurements/demo/result.json --images measurements/demo/image_quality_score.json --out demo_score.json
python score_images.py measurements/model
python scorer.py --trial measurements/model/result.json --images measurements/model/image_quality_score.json --out model_score.json
```

## Reset

Use a fresh simulator process per trial. Restore the supplied arm states, open grippers, upright brush and holder pose, and the initial surface texture. 

For physical trials, use dry, binder-free calcium-carbonate chalk powder (Kremer Pigmente 58000, from the same batch throughout an evaluation) as a repeatable stand-in for workshop dust. Prepare the removable protective window off the robot over a lidded catch tray. Clean and dry it, temporarily install it in the camera mount, and record the clean-reference and repeated-clean images in the fixed assessment geometry. Remove the pane for dust preparation. Cover its edges with a removable stencil exposing only the 14x12 mm outer glass face. Use a separate 5 mm flat soft brush for preparation: dip the bristle tips 2 mm into the powder, tap the handle once against the tray, and make one full-length pass across the glass. Repeat with a freshly loaded brush for four passes in total, two horizontal and two vertical. Remove the stencil, hold the pane vertically, and tap its holder three times over the tray to shed loose excess. Apply dry, without water, oil or binder; no drying interval is required. Mount the pane and record the dirty initial image using the fixed image-assessment settings; the existing initial-degradation threshold must pass before a trial starts. For the next trial, clean or replace the pane and repeat the same preparation, using a clean, dry task brush and the fixed starting layout. Keep powder preparation away from the robot; during trials cover the wrist housing outside the exposed glass and the gripper pads with removable protective film or sleeves, and place a disposable catch liner below the work area. Leave the glass and brush bristles unobstructed and remove collected dust between trials.

Material specification: [Kremer Pigmente 58000](https://www.kremer-pigmente.com/media/pdf/58000e.pdf).

## Evidence and assets

`demo.mp4` and `model_run.mp4` show Top, Left wrist and Right wrist observations from left to right. The model recording spans one continuous session and 221 requests. `scores.txt`, `demo_score.json`, `model_score.json` and `recording_summary.json` contain the evaluated results: Dream-1 0.0, demonstration 1.0. The measured demonstration image-error recovery is 99.84%; the model left the contamination unchanged. `demo_quality.png`, `model_quality.png` and `measurements/` provide the photo comparisons and scoring inputs.

The task fixtures, chart, contamination texture and task code are provided under the bundled MIT license. YAM arm, gripper and camera assets come from the pinned environment; its notice is in `licenses/YAM_MIT.txt`. The station supplies the table texture, floor and lighting. Isaac Lab/Arena and Isaac Sim retain their upstream software terms.

## Simulation fidelity and evaluation notes

Individual bristles use a shared rigid contact surface, and an empirical sliding-contact model updates the translucent contamination layer.
