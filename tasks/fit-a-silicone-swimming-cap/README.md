# Fit a loose silicone swimming cap

Stretch the swimming cap over the headform. Cover the crown, leave the face clear, and release it without tearing the cap.

## Run and score

Use the Python 3.12 / Isaac Sim 6.1 / YAM runtime pinned in `ENVIRONMENT.json`.
An NVIDIA GPU is needed for native rendering. From the repository root:

```sh
OMNI_KIT_ACCEPT_EULA=YES python tasks/fit-a-silicone-swimming-cap/run.py --mode demo --out /tmp/fit-a-silicone-swimming-cap-demo
OMNI_KIT_ACCEPT_EULA=YES python tasks/fit-a-silicone-swimming-cap/run.py --mode dream --out /tmp/fit-a-silicone-swimming-cap-dream
python tasks/fit-a-silicone-swimming-cap/rescore_native_precision.py /tmp/fit-a-silicone-swimming-cap-demo
```

Dream receives the ordinary RGB cameras and joint state through Inspect Robots.
The SDK resolves its credential privately. Use one episode per key at a time.
The fixed layout has no random jitter or seed. The limit is 1,800
controls at 20 Hz. Demo and Dream use identical physics and hardware; the demo
commands only the robot joints and grippers.

Each run saves its source, complete state trace, camera recordings and report.
Rescoring checks source hashes and recomputes outcomes from the recorded native
states on CPU; it is not a new physics simulation. Camera strides above 1 are
for diagnostics and are excluded from submission evidence.

## Setup and reset

Use a smooth, rigid ABS adult headform, 190 × 160 × 240 mm, fixed to its 140 × 130 mm base. The silicone cap has a 190 × 150 mm unstretched opening, a 90 mm dome, a 0.8 mm wall and a 3 mm rolled rim. Match one cap model and size across trials.

The head centre is (360, 0, 872.5) mm, with its nose facing negative X, towards the robot. X points forward, Y left, and the table is 747.5 mm high. Each arm stands on a fixed 160 × 160 × 100 mm steel riser. Both grippers have 80 × 24 × 3 mm silicone-faced extensions and 40 N finger actuator limits.

The cap starts loose across the crown, with both hands holding its two side rims. The initial held rim centre is (360, 0, 977.5) mm, 15 mm below the crown, with a 3 mm pad gap. It is still unstretched and above the scored fitting bands. Reset verifies actual contact before frame zero and lets the material settle under gravity; demo and Dream share this reset. The robot must stretch, draw down and release the cap. No fixture holds it on the head and no material pose is assigned during a trial. Restore the photographed layout with the cap loose, not stretched into its final fit or damaged.

## Scoring and material model

Use the video and a ruler to check the rim around the head, the clear face,
the crown gap and three seconds of retention after release. The numerical
bands and partial credit are in `task.json`. A cap resting beside the head,
or crumpled on its crown without an encircling rim, earns no credit. Partial credit requires an encircling rim lowered below the 100 mm height band; the full-fit band remains 75 mm.

The cap is a 0.8 mm native elastic shell with self-contact and a 3 mm rolled
rim. Young's modulus is 1 MPa; this uncalibrated value approximates silicone.
The head is rigid. A compliant native pinch forms only when a rim element is
between closing finger faces. It releases on opening or 8 mm slip, and breaks
at 40 N or 0.08 Nm. Its uncalibrated translational stiffness is 40,000 N/m;
the force cap and 8 mm slip limit remain active. The finite grip connects to the native gripper and transmits reaction loads. The robot
must stretch, position and release the material; no fitted shape is assigned.

## Assets

All task geometry is original procedural work under the included MIT license.
The robot, table and runtime come from the pinned YAM / Isaac environment.
No third-party task meshes, scans or textures are used.

## Contact approximation

The shell uses a 353-vertex geodesic mesh, 0.4 mm rest offset and 1 mm contact margin. The smooth ABS head and cap have approximate friction coefficients of 0.30 and 0.35. There is no attachment to the head. Grip capacity is shared between the rim points actually inside each pad; contacts that leave detach individually. Both policies use the same geometry, contact model and material parameters.

## Recorded results

The demo scores **1.0** in 600 actions, with an intact, fitted cap and 5.1 seconds clear of both hands. Dream-1 scores **0.0** over all 1,800 actions. Wall runtimes were 18.8 minutes for the demo on the shared L40S and 61.0 minutes for Dream on the shared RTX PRO 6000 Blackwell.

Both complete traces pass source-hash checks and independent rescoring. A separate check found at most 0.44 mm of cap-node penetration into the head mesh, within the 1 mm collision margin; all three camera views were also reviewed. Six score-boundary checks cover released retention and integrity failures. The CPU precision adapter preserves the recorded float32 coordinates; it does not loosen the frozen audit's tolerances or alter its scoring.

Dream made 75 API attempts across 37 sessions, including 37 SDK-reported expiry recoveries. This is a recovered-session baseline; the full transcript retains those diagnostics.

## Submission camera views

Each submission video places the synchronized close view beside the robot camera, which shows the face and eye clearance. Both are complete native recordings at 20 fps, with no time cuts. The robot-camera image is enlarged for visibility. The setup image is the first frame of the same layout; media hashes and processing are in `evidence/media-composition.json`.
