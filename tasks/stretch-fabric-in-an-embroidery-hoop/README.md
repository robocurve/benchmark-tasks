# Stretch and secure fabric in an embroidery hoop

Stretch the fabric in the embroidery hoop and secure it, ready to stitch.

Prepare fabric for hand embroidery: remove slack in several directions, then tighten the ordinary hoop screw so the unsupported working area stays flat after release.

## Run and score

Use the pinned Python 3.12 / Isaac 6.1 / YAM environment in `ENVIRONMENT.json`.
From the repository root, with an NVIDIA GPU and the ordinary Dream SDK credential:

```sh
OMNI_KIT_ACCEPT_EULA=YES python tasks/stretch-fabric-in-an-embroidery-hoop/run.py --mode demo --out /tmp/stretch-fabric-in-an-embroidery-hoop-demo
OMNI_KIT_ACCEPT_EULA=YES python tasks/stretch-fabric-in-an-embroidery-hoop/run.py --mode dream --out /tmp/stretch-fabric-in-an-embroidery-hoop-dream
python tasks/stretch-fabric-in-an-embroidery-hoop/rescore.py /tmp/stretch-fabric-in-an-embroidery-hoop-demo
python tasks/stretch-fabric-in-an-embroidery-hoop/test_model.py
```

The fixed reset has no random seed. Both modes have the same 1,600-action limit
(80 seconds at 20 Hz), material model, robot and observations. Dream receives
standard RGB cameras and joint state. The demo issues saved ordinary bounded
joint/gripper commands with joint feedback. Material response uses measured hand
poses; it never reads the demo stage, intended target, policy mode or score.
`--cpu-preview` is a planning check, not submission evidence.

## Setup and reset

A 130 mm beechwood embroidery hoop with a brass M3 thumb screw is held horizontal by three wooden rim supports, 45 mm above the table. A 240 mm diameter piece of plain woven cotton (140-180 g/m2) is already loosely seated between the rings, with its centre 8-12 mm below the rim; its outer edge hangs freely. The screw starts one quarter-turn short of its finger-tight setting. The centre has no backing or pins. Both hands start open. README gives the low-cost support layout and repeatable reset.

Coordinates are metres in the YAM station: x points away from the robot,
y to its left, table height 0.7475 m. Hoop centre is (0.32, 0, 0.7925),
screw centre (0.399, 0, 0.7965), screw axis along y. The hoop has an approximately
8 mm high, 3.6 mm thick wooden rim and a 14 mm thumb head; use a standard wooden
hand-embroidery hoop, not an oversized handle added for the robot.

Three 18 x 18 x 37 mm wooden blocks support the inner rim at angles 90, 210 and
330 degrees, radius 65 mm. Fasten them to the table and secure the inner rim at
those three points with small removable clamps. The outer ring and cloth remain
free to slip and tighten. The illustrated 120 x 80 x 6 mm wood base is centred
at (0.32, 0.14, 0.7505); it supports the rear mounting hardware. A normal tabletop
embroidery stand gripping only the rim is an equivalent low-cost fixture. Do
not put a disk, foam or any support under the working cloth. No fixture part
performs the stretching or tightening. All parts are ordinary craft supplies.

Cut one 240 mm circle from plain, unstarched 140-180 g/m2 woven cotton. A light
10 mm fabric-marker grid may help photographs; it provides no grip tabs. Set the
inner rim, lay the cloth centrally and seat the loosened outer hoop. Use a ruler
across the rim to set 8-12 mm centre sag. Mark the screw's ordinary finger-tight
position during setup and back it off 90 degrees; it must still retain the
cloth but permit a gentle outward tug. Reproduce this reset between trials.
The recorded simulated mesh starts at 8 mm sag and settles under gravity.

Use the same cotton stock, unstarched and dry. Loosen the screw, lift out and recenter the cloth, then reseat with 8-12 mm centre sag measured with a straightedge. Set the screw a quarter-turn back from the recorded finger-tight mark. Reject damaged or permanently stretched cloth. No needle or stitching is needed.

## Judging

Score the final state. 1.0: fabric is retained around at least 95% of the rim, the screw is tightened, and the central 96 mm working area stays within 5 mm of the rim plane, with at most 5 mm peak-to-valley variation, for 3 seconds after both hands release. 0.75 for that result before release/hold completion; 0.5 if retained and tightened but still too slack; 0.25 if at least half the rim retains the cloth but less than 95%, or if fully retained and drawn within 5 mm but not tightened. Otherwise 0. Any tear or pulled-out fabric invalidates the result. Use a ruler, straightedge, photographs and the video.

Use an overhead photograph to check rim retention (no gap longer than 20 mm for
95% of a 130 mm hoop), and side photographs with a ruler/straightedge spanning
the rim to check the central 96 mm diameter area. Check along two perpendicular
diameters, without pressing on the cloth. The 5 mm allowance permits ordinary
ruler judging; there is no micrometre or tension-sensor requirement. The recorded
screw mark provides a visible tightened endpoint. The cloth must not be pressed
flat by either hand during the final hold. A tear, pull-out or later loss of
retention overrides an earlier good state.

## Fidelity

Native YAM arms and cameras. Cotton is a flexible spring mesh with gravity, in-plane stretch/shear and weak bending. Only the hoop annulus retains it; material can slide through the loose rim. Measured screw rotation increases rim friction. The centre is unsupported. Finger captures, fixture contact, screw friction and tear thresholds are reduced, uncalibrated approximations; no scripted flattening or interior pins.

All material parameters are engineering approximations. The mesh shown in each
frame is the current simulated state. They are not calibrated force or material
measurements, and do not establish real-hardware success.
## Recorded trials

- Demo: **1.0**, 872 actions / 43.6 simulated seconds; 149.1 wall seconds including startup.
- Dream: **0.0**, 1600 actions / 80 simulated seconds; 622.9 wall seconds including startup.

Dream made 33 requests across 33 server session(s). Server-session recoveries occurred during this completed rollout; it is not an uninterrupted single-session baseline. The request transcript preserves the attempts and timing.

Both runs use the same material physics, cameras, instruction and starting
layout. Independent measured-hand replay agrees with both results. The videos
were checked from multiple views. `VALIDATION.json` records source hashes and
any non-executed source differences. Native traces, summaries and Dream request
records are in `evidence/`; no server session IDs or credentials are published.

Dream used the official `BridgeServer` / `BridgeSession` over an SSH loopback
tunnel, keeping its credential on the client computer. The bridge and runtime
versions are recorded in `evidence/`. A local SDK credential also works with
the commands above.

The uploaded videos preserve every native frame at 20 fps, with the selected two
judging views side by side. The setup image shows the same two starting views.
For CPU-only replay of the supplied evidence, install NumPy and SciPy and run:

```sh
python tasks/stretch-fabric-in-an-embroidery-hoop/replay_evidence.py demo
python tasks/stretch-fabric-in-an-embroidery-hoop/replay_evidence.py dream
```
