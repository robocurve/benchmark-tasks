# Curl gift ribbon

Curl the ribbon with the curling edge, then leave it loose on the table.

Make a loose decorative curl for a wrapped gift. The robot must draw flat curling ribbon over an edge under tension, leaving a curl that holds after release.

## Run and score

Use the Python 3.12 / Isaac 6.1 / YAM environment pinned in `ENVIRONMENT.json`.
Native camera runs require an NVIDIA GPU. From the repository root:

```sh
OMNI_KIT_ACCEPT_EULA=YES python tasks/curl-gift-ribbon/run.py --mode demo --out /tmp/curl-gift-ribbon-demo
OMNI_KIT_ACCEPT_EULA=YES python tasks/curl-gift-ribbon/run.py --mode dream --out /tmp/curl-gift-ribbon-dream
python tasks/curl-gift-ribbon/rescore.py /tmp/curl-gift-ribbon-demo
python tasks/curl-gift-ribbon/audit_geometry.py /tmp/curl-gift-ribbon-demo
python tasks/curl-gift-ribbon/test_model.py
```

The fixed reset has no random seed. Limit: 1,600 controls (80 seconds at 20 Hz).
Both modes use the same reset, physical model, instruction and cameras. Dream
receives normal RGB and joint state through the official adapter. The demo is a
fixed sequence of ordinary bounded joint/gripper commands with joint feedback.
The material law reads measured hand poses, never the stage, intended action,
policy mode or score. CPU previews are planning checks, not submission evidence.

## Setup

One straight 400 x 10 mm strip of commercial polypropylene curling ribbon, about 0.1 mm thick, lies flat on the table. A deburred steel ruler (90 x 30 x 1 mm working piece) is secured vertically to a 100 x 25 x 55 mm wooden block; its blunt working edge is 65 mm above the table. The edge is fixed, so both hands can tension and draw the ribbon. Both hands start open. No sharp cutting blade, pre-curled strip or adhesive is used. README gives the fixed layout and material check.

Coordinates are metres: x away from the robot, y to its left, table height
0.7475 m. Ribbon centreline starts at x=0.235, y=-0.16 to +0.24, flat on the table.
The ruler edge runs along x, centred at (0.32, 0, 0.8125). The wooden support is
centred at (0.32, 0.010, 0.775). Secure the ruler and block with ordinary padded
bench clamps outside the ribbon path. There is no mechanism inside the support;
it only holds the curling edge. A deburred metal ruler gives the familiar
scissors-edge action without exposing a cutting blade to the robot pads.
Inspect its corners and edge for burrs before each session.

Use one make and roll of non-fabric, non-metallic polypropylene gift curling
ribbon, 10 mm wide and roughly 0.1 mm thick. Record the product/lot and keep it
fixed. Condition it flat and dry at 18-24 C. The spare-piece check below excludes
satin and decorative strips that do not plastically curl; it does not pre-work
the trial piece. All items are ordinary low-cost gift-wrapping or bench supplies.

Cut a fresh 400 mm piece from the same roll and lay it flat without creases or existing curls. Use ordinary curling ribbon, not woven satin. Check a spare piece by hand once per material batch: one loaded pass over the same edge must retain two turns after release. Use a new strip each trial; inspect the deburred edge and padded clamps.

## Judging

Judge the loose ribbon at the end. 1.0 for at least two consecutive complete turns, 8-60 mm in diameter, intact and on the table, with both hands released for 3 seconds. 0.75 for two turns before the release hold completes; after release, 0.5 for one complete turn or 0.25 for at least half a turn. Otherwise 0. Tearing or cutting the ribbon gives 0. Count visible turns from side photographs or video with a ruler. The video must show drawing the ribbon over the curling edge; folding or wrapping it around a support is not the target.

A turn is a full 360-degree circuit of the ribbon centreline in one direction;
two turns require 720 degrees, not two hairpin folds. Photograph approximately
along the curl axis with a ruler beside it; use a second view if turns obscure
one another. Count the longest consecutive curled section. Ignore uncurled
ends, ribbon colour and where it lands on the table. The final hold must be
present in the video, not assumed after the episode ends. A ruler and ordinary
video suffice; simulated tension or plastic work is not part of physical judging.

## Fidelity

Native YAM arms and cameras. Ribbon is a reduced flexible strip with stretch, bending, gravity and full-segment self-contact. Loaded sliding over the ruler creates local plastic curvature; a free 3D beam solve forms the curl, not an animation. Width contact, torsion, finger capture, edge tension, plasticity and tearing are uncalibrated approximations. Stored curvature alone earns nothing: actual released turns are scored.

The 81-node material model uses an explicit, uncalibrated load-and-slide law.
It only creates plastic bend memory at actual edge contact with two-ended
tension and material travel. Stored memory alone earns no credit: the current
released geometry must form the turns. A projective solve handles the held
strip; an inextensible implicit 3D beam solve handles its free motion, including
inertia, gravity, bending/torsion energy and full-segment self-contact. Its
10 mm width / 1 mm thickness contact envelope is deliberately coarse.
Lateral freedom lets turns separate instead of passing through one another. Grasp
capture idealizes friction between enclosing finger pads. It is not a native
finite-element model or a measurement of any commercial ribbon's properties.

The qualitative load/sliding mechanism follows the
[ribbon-curling experiment](https://pubmed.ncbi.nlm.nih.gov/26831118/) and its
[demonstration notes](https://personalpages.manchester.ac.uk/staff/Andrew.Hazel/teachers_resources/ribbon_curling/ribbon.html).
The chosen parameters support this benchmark demonstration; they have not been
fitted to a particular retail product or validated on real robot hardware.

The geometry audit rejects invalid strip crossings independently of the score.
Its numerical contact tolerance is a simulation integrity check, not an extra
measurement required of a real-world judge.
## Recorded trials

- Demo: **1.0**, 824 actions / 41.2 simulated seconds; 191.6 wall seconds including startup.
- Dream: **0.0**, 1600 actions / 80 simulated seconds; 641.3 wall seconds including startup.

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
python tasks/curl-gift-ribbon/replay_evidence.py demo
python tasks/curl-gift-ribbon/replay_evidence.py dream
```

## Recorded geometry

The compact final turns partly hide one another in the camera view. This
[diagnostic plot](evidence/final-ribbon-shape.png) shows the recorded final
centreline, coloured by material order. It is not a replacement for the native
video. The supplied geometry audit checks strip-face crossings separately from
the score; the final three seconds retain at least 2.6468 turns.
