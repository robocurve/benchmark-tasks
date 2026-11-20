# Secure your loose wrist-camera cable

The robot routes its own loose wrist-camera cable over a retaining peg and closes a hook-and-loop strap. The final state requires the working hand to release and move away while the cable remains secured.

This tests robot self-maintenance: restoring a cable-management fixture on the robot itself. The working hand must leave the cable and strap secured on the arm.

## Setup and mechanism

`task.json` describes the fixed scene; `layout.json` summarizes dimensions and initial robot joints. `hardware.py` constructs the camera housing, cable, side-mounted peg and articulated strap. The left wrist camera retains its calibrated optical pose. The policy receives the standard top, left-wrist and right-wrist RGB images and14D joint/gripper state.

Use a standard camera USB data cable with matching connectors. The simulated cable is 740 mm long and 6 mm in diameter. For the fixture, use a removable rubber-lined split collar around the arm link. Fit the collar bore to the link and fasten its two halves together using screws in the collar, leaving the arm shell intact. Mark the collar location and its rotation with removable tape so the fixture can be remounted consistently. Commercial robot cable fixtures provide a reference for the removable arm mount ([igus Cable Clip](https://www.igus.com/product/triflex-r-cable-clip?artnr=TR.COB.05.0500)); the peg itself uses the dimensions below.

Print the peg in PETG with a 20 mm diameter, 40 mm long shaft and a 36 mm diameter, 5 mm thick cap. Fasten through the collar into the peg base. Centre a 34 mm diameter adhesive loop patch on the outward face of the cap. Use a black 168 × 20 mm hook strip with a 24 mm wide handling end and back it with a thin springy PET strip, targeting 1.5 mm overall thickness. Form the backing over a curved jig and clamp the root to the collar. The open strap should leave the peg clear for cable insertion and be foldable by the gripper. Use the left link-3 frame as the installation datum. In that frame, the peg reference is (−50, 7, −160) mm and the shaft points along −X. The strap root is (−91.46, −23.79, −160) mm. To form the open bend, follow the 14 mm station spacing and 5° to 35° tangent schedule defined in `hardware.py`. Use the marked collar position and the setup views to check that the opening clears the cable path. To reset, peel the strip off the pad, restore its open curve and let the cable hang between the connectors as shown in the setup image.

The cable remains dynamically articulated and attached at its two connectors. The open strap bends through compliant revolute joints. After the inner strap face contacts the receiving pad and sustains compression for 0.10 seconds, `velcro_contact.py` applies an ideal local attachment to every segment that independently sustains compression; the remaining strip segments retain their elastic joints and dynamics.

## Run and score

Use the official YAM simulation environment from the program walkthrough. `dependencies.py` identifies the pinned YAM source. Dependencies are its Isaac Lab/Isaac Sim runtime, NumPy and SciPy; Dream evaluation also uses the official Dream and Inspect Robots adapters. Video export uses Pillow and `ffmpeg`.

Run these commands from this folder after activating the YAM environment:

```bash
python test_scorer.py
python run_setup.py --out runs/setup --steps 20
python run_complete.py --out runs/demo --steps 1800
python score_trace.py runs/demo
python render_native.py runs/demo

# Set DREAM_API_KEY privately in the shell.
python run_dream.py --out runs/model --steps 1800
python score_trace.py runs/model
python render_native.py runs/model
```

Use a new output directory and a fresh process for every trial. The supplied demo follows a fixed joint plan. Scene initialization is deterministic; the model scene seed is42. Physics runs at240 Hz and policy control at20 Hz. The recorded script stays within the shared90-second limit; exact action counts are in `recording_summary.json`. Wall time depends on GPU load, rendering and API latency; the model run records its wall time in `result.json`.

Three native camera views appear left to right as Top, Left wrist and Right wrist, with no overview or information bar. The current scripted video is continuous.

## Scoring

The rubric in `task.json` and `scorer.py` awards0,0.5,0.75 or1.0 for the final held state. `score_trace.py` replays measured scene geometry and robot state using the same evaluator for scripted and model trials.  For a physical trial, inspect from above to confirm the strap crosses the cable on the shaft and from the side to confirm the closed strap encloses it. The hand must then move clear and remain clear for the timed hold.

## Files

The setup image, scripted demo and Dream-1 recording are available through the task board media ID in `task.json`.

- `task.json`, `setup.png`: task and fixed initial observations.
- `demo.mp4`, `model_run.mp4`, `scores.txt`: recorded evidence and evaluated score.
- `run_complete.py`, `replay_actions.json`: continuous scripted feasibility run.
- `run_dream.py`, `dream_driver.py`: model-only control with native observations.
- `hardware.py`, `cable_usd.py`, `prepare_geometry.py`, `camera_mount.py`: scene construction and geometry.
- `scorer.py`, `score_trace.py`, `render_native.py`: scoring and native-view export.
- `EVIDENCE.md`, `recording_summary.json`: recording details and results.

## Assets and licenses

Task fixtures, cable and strap are procedural primitive geometry, under the bundled MIT license. YAM arm/gripper/camera assets come from the pinned program environment; its MIT notice is in `licenses/YAM_MIT.txt`. Table, floor and lights are station primitives.  Isaac Lab-Arena and Isaac Sim retain their upstream software terms.

## Implementation and recording notes

**Fastening model.** Hook-and-loop engagement is represented by a local contact bond after 0.10 s of compression. Every segment can attach after it sustains compression on the pad. The rest of the strap remains flexible. Scoring checks the resulting enclosure and hand release from measured geometry. The recorded demonstration retained the cable for 4.0 seconds with the right gripper open and clear.


**Recording details.** The demo uses a fixed joint plan without intermediate state restoration. Dream-1 run: 0.00 (1,800 steps, 90 seconds, one session).
