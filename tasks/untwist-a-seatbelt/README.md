# Remove a seatbelt twist through its sliding tongue

Remove the twist from the seatbelt without detaching either end. Leave the webbing flat through the tongue, then release both hands.

A twisted seatbelt can be awkward to straighten because its sliding tongue traps the fold. Form a diagonal fold, work it through the slot and spread the webbing flat.

## Run and score

Use the Python 3.12 / Isaac 6.1 / YAM versions pinned in `ENVIRONMENT.json`.
An NVIDIA GPU is required for the native cameras. From the repository root:

```sh
OMNI_KIT_ACCEPT_EULA=YES python tasks/untwist-a-seatbelt/run.py --mode demo --out /tmp/untwist-a-seatbelt-demo
OMNI_KIT_ACCEPT_EULA=YES python tasks/untwist-a-seatbelt/run.py --mode dream --out /tmp/untwist-a-seatbelt-dream
python tasks/untwist-a-seatbelt/rescore.py /tmp/untwist-a-seatbelt-demo
```

The layout is fixed; there is no random seed. Limit: 1,200 actions (60 seconds at 20 Hz).
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

A 500 x 47 x 1.2 mm polyester seatbelt section is attached to two 70 x 22 x 40 mm bench anchors, 440 mm apart. A steel sliding tongue has a 49 x 3.5 mm slot and 1.5 mm thickness. It starts at the midpoint with opposing half-twists trapped on either side. Blue and yellow edge markers show the webbing orientation. Both hands start open. The deterministic settled reset, dimensions and fixed layout are in model.py.

The setup image and constants in `model.py` fix positions in metres. Restore
the photographed starting arrangement and open both hands before each trial.

For reset, clamp both end cross-sections flat with the blue edge on the same
side. Turn the midpoint over by half a turn, giving opposite half-twists on
each side; thread that reversed midpoint through the tongue. Set the slot
centre 40 mm above the table, halfway between the clamps. Leave 500 mm of
webbing between clamps 440 mm apart, then allow the slack to settle. The
60-step passive simulated settling is part of reset, before the action limit.
Use a bench sample, not a belt in an occupied vehicle.

## Judging

1.0: webbing intact, tongue retained on it, crosswise roll below 15 degrees and visible width above 40 mm throughout, and both hands released for 3 seconds. 0.75 for this flat result before release/hold. 0.5 if maximum remaining roll is below 90 degrees but it is not yet flat. Otherwise 0; damage or detached hardware overrides credit. Inspect both edge markers using overhead/side photographs, a ruler and a protractor.

Scores describe the final result; later damage invalidates earlier progress.
The demo must satisfy the release hold in the recorded episode itself.

## Fidelity

Native YAM arms/cameras. Webbing is a flexible mass-point ribbon with stretch, bending, table and coarse self-contact. Continuous edge/slot constraints preserve threading. The tongue is a reduced sliding body; ideal grasps move it. Initial stiction releases after 2 mm of sliding. Detailed weave, calibrated friction and arm reaction forces are omitted. No timed or score-triggered untwisting.

The material mesh is drawn from the current simulated state at every step.
These are reduced models, not calibrated material or force measurements.

## Recorded trials

- Demo: **1.0**, 832 actions / 41.6 simulated seconds; 401.3 wall seconds including startup.
- Dream: **0.0**, 1200 actions / 60 simulated seconds; 1023.3 wall seconds including startup.

Dream made 25 requests across 25 server session(s). Server-session recoveries occurred during this completed rollout; it is not an uninterrupted single-session baseline. The request transcript preserves the attempts and timing.

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
python tasks/untwist-a-seatbelt/replay_evidence.py demo
python tasks/untwist-a-seatbelt/replay_evidence.py dream
```
