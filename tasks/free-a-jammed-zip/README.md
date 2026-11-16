# Free lining caught in a zip and leave the zip working

Free the yellow lining caught in the washbag zip without tearing it. Check the zip moves past the snag, then let go.

Freeing a fabric snag requires easing the lining out while controlling the slider; pulling harder can damage the lining or zip.

## Run and score

Use the Python 3.12 / Isaac 6.1 / YAM versions pinned in `ENVIRONMENT.json`.
An NVIDIA GPU is required for the native cameras. From the repository root:

```sh
OMNI_KIT_ACCEPT_EULA=YES python tasks/free-a-jammed-zip/run.py --mode demo --out /tmp/free-a-jammed-zip-demo
OMNI_KIT_ACCEPT_EULA=YES python tasks/free-a-jammed-zip/run.py --mode dream --out /tmp/free-a-jammed-zip-dream
python tasks/free-a-jammed-zip/rescore.py /tmp/free-a-jammed-zip-demo
```

The layout is fixed; there is no random seed. Limit: 800 actions (40 seconds at 20 Hz).
Both modes use identical material dynamics, observations and reset. Dream gets
the standard three RGB views and robot joint state through the normal adapter.
Its credential is read privately by the SDK. Run only one trial per key at once.

The demo uses a saved sequence of ordinary bounded joint/gripper commands and
joint feedback for tracking. The material model reads measured hand poses;
it does not read the demo stage, commanded target, policy mode or score.
`run.json` records status, wall time and result. The trace records every measured
hand pose, material state and applied action. `rescore.py` verifies the frozen
source hashes and replays the measured hands. `--cpu-preview` is a planning
check and is explicitly excluded from submission evidence.

## Setup

A 300 x 280 x 36 mm fabric washbag rests flat, weighted by its contents. Its 200 mm coil zip runs front to back, with a 10 x 13 mm slider and 8 x 25 mm pull tab. A yellow polyester lining flap, 140 x 60 x 0.4 mm, is caught at the left entrance of the slider. The two remote corners remain sewn to the bag. All starting positions are fixed by the setup image and model.py.

The setup image and constants in `model.py` fix positions in metres. Restore
the photographed starting arrangement and open both hands before each trial.

## Judging

1.0: lining free and intact, slider has moved at least 25 mm below and 45 mm above the original snag after the lining is eased clear, and both hands released for 2 seconds. 0.75 for that functioning zip while still held. 0.5 for freed intact lining before the travel check. Otherwise 0; lining or zip damage overrides all credit.

Scores describe the final result; later damage invalidates earlier progress.
The demo must satisfy the release hold in the recorded episode itself.

## Fidelity

Native YAM arms/cameras; flexible lining uses mass points with stretch and gravity. The slider is constrained to its sewn track. A tapered entrance compresses trapped cloth and creates finite friction; reversing unloads it. Grips are idealized. The weighted bag/track stay fixed. Teeth, tearing and friction use reduced rules; forces are uncalibrated. No phase-triggered release.

The material mesh is drawn from the current simulated state at every step.
These are reduced models, not calibrated material or force measurements.

## Recorded trials

- Demo: **1.0**, 380 actions / 19 simulated seconds; 193.4 wall seconds including startup.
- Dream: **0.0**, 800 actions / 40 simulated seconds; 665.9 wall seconds including startup.

Both runs use the same material physics, cameras, instruction and starting
layout. Independent measured-hand replay agrees with both results. The videos
were checked from multiple views. `VALIDATION.json` records source hashes and
any non-executed source differences. Native traces, summaries and Dream request
records are in `evidence/`; no server session IDs or credentials are published.

Dream used the official `BridgeServer` / `BridgeSession` over an SSH loopback
tunnel, keeping its credential on the client computer. The bridge and runtime
versions are recorded in `evidence/`. A local SDK credential also works with
the commands above.

The uploaded videos preserve every native frame at 20 fps, with front and rear
judging views side by side. The setup image shows the same two starting views.
For CPU-only replay of the supplied evidence, install NumPy and SciPy and run:

```sh
python tasks/free-a-jammed-zip/replay_evidence.py demo
python tasks/free-a-jammed-zip/replay_evidence.py dream
```
