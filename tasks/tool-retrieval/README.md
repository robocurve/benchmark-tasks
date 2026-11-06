# Recover a part from under a machine guard

Use a spatula to draw a loose blue part out through a low front opening, put the part in the green tray, and return the spatula to its supports. This represents recovering a dropped component from beneath unpowered bench equipment. The part is inaccessible to a direct top-down grasp; it is not clamped or modelled as an interference-fit jam.

The recess has fixed sides and a closed back. A 20 mm roof slit permits observation of the 25 mm wide part but does not admit a grasp around it. The opening is 28 mm tall. The tool can pass beside the part, shift behind it, and draw it out through the same front opening.

## Installation and reproduction

Use the supplied Linux YAM environment with an RTX GPU. Install this folder as an editable package so its relative USD asset remains available:

```sh
source ~/IsaacLab-Arena/yam.env
~/.local/bin/uv pip install -e . --no-deps
python -m unittest discover -s . -p test_score.py
bash run_demo.sh demo
```

The extension registers embodiment `yam_retrieval` and scripted policy `yam_retrieval_scripted`. Use this embodiment for both policies: generic `yam_arena` does not spawn the spatula or track the required contact history.

After verifying a successful demo, run Dream with credentials installed outside the repository:

```sh
bash run_dream.sh dream
```

Both scripts use seed 0, zero object jitter, CPU PhysX at 240 Hz, 20 Hz control, the calibrated station cameras, 640 x 360 saved observations and a 2,400-step limit (120 simulated seconds). The scripted policy requests a stop three seconds after its plan if success has not already ended the episode. Dream's client resizes images to its model input contract. Logs and raw frames are written outside this folder.

The script sends robot joint targets only. It does not attach, teleport or otherwise move objects by writing their poses. It reads simulator tool poses to maintain the blade's orientation and placement, and the part's position for the second-arm pickup. Dream receives the ordinary camera images and joint state and does not receive that oracle feedback. All object motion during a trial comes from rigid-body physics and contact. The extension explicitly re-enables contact processing, which the Isaac Lab backend otherwise disables by default.

## Setup

Use the official calibrated YAM station. World x points forward, y left, z up. The table is at z = 0.7475 m, and arm bases are at (0, +0.305, 0.75) and (0, -0.305, 0.75). All task objects, exact dimensions, mass and spawn poses are in `task.py`; `scene_inventory.json` includes the same information in machine-readable form. Values below are metres; heights are above the table.

- Blue part: rigid 30 x 25 x 10 mm block, 25 g, centred at (0.40, 0.075, 0.005). It starts beneath the roof, resting on the table.
- Spatula: Arena's `spatula_15_vomp_robolab`, approximately 273 x 59 mm, with a 35 x 24 x 18 mm grip sleeve. Total assigned mass is 80 g. Its root starts at (0.24, 0.27, 0.039); the blade points along positive x. The grasp sleeve is centred 100 mm toward negative x and 6 mm above the source root. This visible sleeve is an adaptation for a parallel gripper.
- Two fixed grey tool supports, 35 mm tall, centred at x = 0.18 and 0.34, y = 0.27. They support the handle and blade while leaving the sleeve accessible.
- Fixed grey guard: side walls at y = -0.04 and +0.15, a back wall at x = 0.56, and two roof sections spanning x = 0.345 to 0.555. The roof underside is 28 mm above the table; its observation slit spans y = 0.065 to 0.085. The authored hood has chamfered powder-coated panel meshes, four steel mounting lugs, fasteners, slot trim and front warning markings. Five box collision envelopes preserve these panel clearances; external mounting lugs also collide. Visual chamfers are 0.7 mm. `build_fixture.py` regenerates the self-contained USD. Supports retain box geometry.
- Fixed green tray: 110 mm square floor, 4 mm thick, centred at (0.235, -0.175), with four 14 mm high rim pieces.

Reset places all objects back in their fixed initial poses, returns both arms to ready, clears contact and failure history, and settles physics. Nothing is consumed.

## Numerical scoring

The final score is used; intermediate maxima are not retained.

1. PhysX must report part–spatula contact. Only then can extraction be registered, when the entire part lies forward of x = 0.305 m and its bottom remains no more than 3 mm below the table surface. A contact alone earns nothing.
2. Score **0.25** when extraction was registered and the entire part still lies forward of x = 0.305 m, but it is not correctly resting in the tray.
3. Score **0.75** when extraction was registered and the part is at rest in the tray: its full world-aligned bounds lie inside x = (0.185, 0.285), y = (-0.225, -0.125), and its bottom is within 4 mm of the tray floor's upper surface.
4. Score **1.0** when the 0.75 conditions hold, the spatula is parked, and both grippers are open and clear. Parking requires the spatula bounding-box centre to lie within 25 mm of (0.24, 0.27), its bottom within 6 mm of the 35 mm support height, vertical extent under 25 mm, and rest. Both grippers must be at least 75% open and both TCPs at least 90 mm from the centres of both loose objects. Full success must hold for one second.

`resting()` means linear speed under 10 mm/s and angular speed under 0.5 rad/s. Direct robot–part contact before registered extraction permanently invalidates the trial. Dropping either loose object more than 20 mm below the tabletop, moving its centre outside the specified workspace x = (-0.15, 0.96), |y| <= 0.62, or a nonfinite pose also permanently invalidates the trial. An invalid trial scores zero regardless of final arrangement. All other outcomes score zero. Retrieval by the tool is required, but no arm assignment is enforced.

Contact reports are a simulation measurement. A real-world version can be checked from video for tool-only extraction and with a ruler for clearances and placement; simulation contact-report thresholds are not calibrated tactile sensors.

## Asset and fidelity

`assets/spatula.usdc` uses the existing Arena spatula mesh, flattened into a self-contained USD with solid display colours. `build_tool.py` records its original NVIDIA asset URL and regenerates it with `usd-core`. Original mesh geometry is retained; the handle and blade each use one convex hull, so small visual slots are filled for collision. The added grip sleeve is a visible rigid box on the same rigid body. The fixed hood uses authored meshes with panel box collision envelopes and external mounting lugs. Supports, tray and target are primitive rigid geometry.

The part represents a lightweight training component, the guard rigid plastic, the tray plastic and the sleeve rubber. Materials, friction, mass distribution and compliance are approximate and have not been calibrated to physical hardware. Loose objects use the station's 1.2 static/1.0 dynamic friction, 5 mm contact offset and zero rest offset. The sleeve does not deform. No magnetism, adhesion, suction, breakaway joints or object attachments are used.

## Validation and review state

The refined mesh-housing demo scored **1.0** in **823 steps**, terminated with
**success**, and took **92.56 seconds wall time** on an L40S. Seed 0,
zero jitter, 20 Hz control, 640 × 360 calibrated cameras and a 2,400-step limit
were used. The complete 41.20-second H.264 videos from all three cameras were
downloaded. The top sequence, setup, final tray and right-wrist closeup were
visually inspected: tool extraction, returned spatula and seated part are visible.
Contact history records tool contact, extraction, no invalid contact/drop, and
both grippers open and clear. Eleven scorer tests pass, including actual PhysX LeftArm/RightArm actor paths.

The runtime is Python 3.12.12, Isaac Sim 6.1.0.0, Isaac Lab 17.0.2, Arena 0.3.0,
YAM Arena 0.1.0 and Inspect Robots 0.60.0. Source pins: YAM
`0059d02a1a91bc6834c1d0846022891df0c78730`, Arena
`c7edf3a9d5d8276565208f72422f8f8a0806d6cf`. `validation.json` records source hashes.
The completed model trial uses this refined revision.

For a configured cluster, activate the configured container environment with
`$SIM_ROOT/cluster/run-arena.sh`, install this folder with `uv pip install -e . --no-deps`,
then run `run_demo.sh` inside the GPU allocation. Set `RETRIEVAL_LOG_ROOT` to a
writable directory outside the repository. `run_dream.sh` uses the same setup.

## Dream-1 evaluation

`dreamlabs/dream-1-beta` completed with a final score of **0** in **2,400 steps**,
terminating with `max_steps`. Wall time including initialization and inference
was **282.52 seconds**. The run used seed 0, zero jitter, 20 Hz control, 640 × 360
saved camera frames, the `calibrated` top-camera configuration and the same runtime source
hashes and physics as the expert demonstration. The model received ordinary camera images
and joint state. All three complete camera videos were downloaded and visually reviewed.

The reviewed sequence shows grasp attempts around the spatula while the blue part remains beneath the housing. No extraction or tray placement is completed. The final contact history records no tool-part contact and no extraction. The top sequence, final frame and both wrist views were inspected.

`validation.json` records the completed result, runtime revision and video hashes.
The Dream video, full-score demo and setup image are linked through the media ID in `task.json`.
