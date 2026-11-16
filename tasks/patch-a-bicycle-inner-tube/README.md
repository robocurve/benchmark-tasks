# Patch a punctured bicycle inner tube

Apply the orange patch over the marked puncture, press it down firmly and leave it sealed after releasing both hands.

A small emergency bicycle repair: positioning a flexible adhesive patch is only useful if its bond seals the puncture. The operator has already cleaned and lightly abraded the rubber and removed the backing.

## Run and score

Use the Python 3.12 / Isaac 6.1 / YAM versions pinned in `ENVIRONMENT.json`.
An NVIDIA GPU is required for the native cameras. From the repository root:

```sh
OMNI_KIT_ACCEPT_EULA=YES python tasks/patch-a-bicycle-inner-tube/run.py --mode demo --out /tmp/patch-a-bicycle-inner-tube-demo
OMNI_KIT_ACCEPT_EULA=YES python tasks/patch-a-bicycle-inner-tube/run.py --mode dream --out /tmp/patch-a-bicycle-inner-tube-dream
python tasks/patch-a-bicycle-inner-tube/rescore.py /tmp/patch-a-bicycle-inner-tube-demo
```

The layout is fixed; there is no random seed. Limit: 1,000 actions (50 seconds at 20 Hz).
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

A butyl bicycle inner tube lies in a 420 x 320 mm oval on the table, lightly rounded to hold its shape. Its prepared 1.2 mm puncture faces up at the front. A 24 x 24 x 0.4 mm flexible pre-glued patch lies adhesive-side down on a 35 x 35 mm silicone release card, 70 mm nearer the robot and 115 mm to its right from the puncture. Hands start open. Dimensions and the fixed coordinates are in model.py.

The setup image and constants in `model.py` fix positions in metres. Restore
the photographed starting arrangement and open both hands before each trial.

For a physical reset, use a fresh pre-glued patch and restore one prepared
puncture. The patch is 70 mm nearer the robot and 115 mm to the robot's right
of the puncture. The tube must remain on the bench; use only enough inflation
to hold its shape. A hand pump and soapy water are operator judging tools,
not objects the robot must operate. The simulation predicts this leak check;
it does not claim to simulate air pressure or a validated real seal.

## Judging

1.0: intact patch seals the puncture and remains sealed with both hands released for 3 seconds. 0.75 for a sealed patch still held or before the hold ends. 0.25 for a partly bonded patch covering the puncture but still leaking. Otherwise 0. Damage overrides all credit. Physical check: no new bubbles for 10 seconds with soapy water on the repair and the tube gently inflated just to hold its shape; use the selected tube/patch maker instructions.

The simulator requires at least12 of81 local adhesive sites for partial
attachment credit; a loose patch or a single-point tack receives0. Its leak
test checks whether an unbonded route connects the hole to any patch edge.
This grid is a disclosed approximation of the physical soap-bubble check.

Scores describe the final result; later damage invalidates earlier progress.
The demo must satisfy the release hold in the recorded episode itself.

## Fidelity

Native YAM arms/cameras. Patch is a flexible mass-point sheet; pressure and dwell create local adhesive bonds that can peel. A connected unbonded path from puncture to patch edge predicts leakage on a 3 mm grid. This is an uncalibrated seal proxy, not air/glue chemistry. Tube is a supported reduced curved surface with lateral motion, not a full pressure/deformation model. Grips are idealized.

The material mesh is drawn from the current simulated state at every step.
These are reduced models, not calibrated material or force measurements.

## Recorded trials

- Demo: **1.0**, 560 actions / 28 simulated seconds; 260.9 wall seconds including startup.
- Dream: **0.0**, 1000 actions / 50 simulated seconds; 719.0 wall seconds including startup.

Dream made 21 requests across 21 server session(s). Server-session recoveries occurred during this completed rollout; it is not an uninterrupted single-session baseline. The request transcript preserves the attempts and timing.

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
python tasks/patch-a-bicycle-inner-tube/replay_evidence.py demo
python tasks/patch-a-bicycle-inner-tube/replay_evidence.py dream
```
