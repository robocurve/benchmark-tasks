# Remove a seatbelt twist through its sliding tongue

Remove the twist from the seatbelt without detaching either end. Leave the webbing flat through the tongue, then release both hands.

Straightening a twisted seatbelt requires coordinating its sliding tongue with the trapped webbing while working beside the seat and pillar.

## Setup and reset

A full-size empty passenger-seat mock-up has a 450 x 450 mm cushion, a 510 mm back, outboard pillar and door-side panel. A 47 x 1.2 mm polyester belt runs from the lower seat-side anchor through its sliding tongue to a shoulder guide and locked retractor. The 750 mm working span starts with opposing half-twists trapped at the tongue. Both ends remain attached. INSTALLATION.md gives inexpensive construction, dimensions, routing and reset.

See [INSTALLATION.md](INSTALLATION.md) for the physical mechanism, all dimensions, fixed positions and repeatable reset. The setup photograph shows the actual starting scene. Both hands start open. No material is consumed unless damaged.

## Run

Use the Python 3.12 / Isaac 6.1 / YAM versions pinned in `ENVIRONMENT.json`. Native cameras require an NVIDIA GPU. From the repository root:

```sh
OMNI_KIT_ACCEPT_EULA=YES python tasks/untwist-a-seatbelt/run.py --mode demo --out /tmp/untwist-a-seatbelt-demo
OMNI_KIT_ACCEPT_EULA=YES python tasks/untwist-a-seatbelt/run.py --mode dream --out /tmp/untwist-a-seatbelt-dream
python tasks/untwist-a-seatbelt/rescore.py /tmp/untwist-a-seatbelt-demo
```

Fixed layout; no random seed. Limit: 1,200 actions (60 seconds at 20 Hz) Both modes use the same scene, physics, instruction and reset. The demo uses bounded 14-element joint/gripper commands with joint feedback. It reads no task state or score. Dream uses the standard three RGB cameras and robot joint observations. Run one trial per API key at a time.

Dream can use the official SDK credential store or the official BridgeServer/BridgeSession through an authenticated SSH loopback tunnel. The latter keeps its API key on the client computer. No secret belongs in trial artifacts.

## Judging

1.0: webbing intact, tongue retained on it, crosswise roll below 15 degrees and visible width above 40 mm throughout, and both hands released for 3 seconds. 0.75 for this flat result before release/hold. 0.5 if maximum remaining roll is below 90 degrees but it is not yet flat. Otherwise 0; damage or detached hardware overrides credit. Inspect both edge markers using overhead/side photographs, a ruler and a protractor.

Scores describe the final state, including the recorded hands-clear hold. Later damage invalidates earlier progress. Review all views before judging; an occluded clasp or remaining belt twist is not evidence of success.

## Simulation

The seat, back, pillar and door-side panel are native colliding obstacles. The working belt is a 61 × 7 particle ribbon with gravity, stretch/bending constraints and coarse self-contact. Its two ends remain fixed at the vehicle-style anchors. A finite four-sided slot constrains the webbing; its tongue can translate and rotate under a measured-hand grasp. The demo turns the tongue through the trapped twist. It does not use a timed cloth target or score-triggered untwisting. Initial reduced slot stiction holds the reset until the tongue moves; its force is not calibrated to a particular donor belt.

The tongue grasp is idealized, and the ribbon does not exert native reaction forces on the arms. Tongue inertia, weave, calibrated material forces, seat softness, active reel behaviour and a human occupant are omitted. The remote guide-to-locked-reel length is fixed. The reduced tongue retains its last orientation when released; the flexible belt continues settling during the hands-clear check. This tests access and webbing manipulation on an empty seat buck, not a calibrated vehicle-restraint model.

## Evidence

`rescore.py` verifies the frozen source and independently reruns the reduced material simulation from every measured hand pose. The trace stores belt coordinates in the local installation frame; installation.py transforms them into the world for rendering and collision. A `--cpu-preview` run is useful for planning but is excluded from submission evidence.

`run.json` records result, wall time and source hashes. The trace records each ordinary applied command and measured state. Uploaded videos preserve every recorded frame at 20 fps; two judging views are placed side by side. Published compressed traces, summaries and audits are in `evidence/`, with no private server identifiers. Visual review is required in addition to the numerical audit.

For CPU-only scoring of the published evidence, install NumPy and SciPy:

```sh
python tasks/untwist-a-seatbelt/replay_evidence.py demo
python tasks/untwist-a-seatbelt/replay_evidence.py dream
```

## Recorded revision trials

- Demo: **1.0**, 592 actions / 29.6 simulated seconds; 336.6 wall seconds including startup.
- Dream: **0.0**, 1200 actions / 60 simulated seconds; 928.2 wall seconds including startup.

Dream used 25 requests across 25 server session(s). The request transcript preserves timing and attempts. Server expiry recovery did not reset the task, but this is not an uninterrupted single-session baseline.

Both runs use identical runtime physics, policy cameras, instruction and reset. Independent scoring agrees with both results. These trials replace the earlier submission evidence.
