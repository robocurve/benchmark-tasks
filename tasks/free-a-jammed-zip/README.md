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

Use a 300 × 280 × 36 mm woven-polyester washbag with a non-locking nylon-coil
zip: 200 mm working travel, approximately 10 × 13 mm slider and 8 × 25 mm pull.
Keep the same bag and zip for all compared trials. Do not glue down the bag.

### Preparation and reset

1. Weigh two sealed flat pouches of dry rice to **500 ± 5 g each**, including
   their packaging. Each is approximately 100 × 240 × 30 mm. Put them flat,
   lengthwise, one each side of the zip with their centres 75 mm from its line.
   Smooth the bag over them; keep a 50 mm central channel clear of contents.
   Nothing is loose inside the zip. Total contents: **1,000 ± 10 g**.
2. Cut a **140 × 60 mm** flap of yellow, non-stretch woven polyester,
   **0.4 ± 0.1 mm** thick. Hem or heat-seal its edges before attachment. With
   its 140 mm direction perpendicular to the zip, sew a 5 mm-wide bar tack
   through each remote corner, 60 mm apart, to the bag lining. Use ordinary
   polyester sewing thread and five passes per tack. Leave the rest free;
   do not sew the edge beside the slider. The two attachments correspond to
   `model.py` vertices 0 and 6. Replace the flap if its weave or seams are damaged.
3. Mark the initial slider position, **125 mm from the lower travel stop**.
   Mark two travel-check positions 25 mm below and 45 mm above it. Put the flap
   flat to the left, with its free edge centred 8 mm above the slider centre,
   as in the setup image. Draw a short line 6 mm in from that edge.
4. With the zip open, introduce the free edge into the slider's **left entry**
   and advance the slider gently to its initial mark. The caught depth is
   **6 ± 1 mm**, measured from that entry lip to the fabric edge; the 6 mm line
   is visible at the lip. Do not knot, glue or puncture the fabric, or jam the
   slider with another object. Photograph the ruler and caught edge.
5. Check the snag using a **0–10 N spring balance** attached to the pull tab.
   Apply 2 N along the closing direction for 2 seconds: slider movement must
   be at most 2 mm and bag movement at most 2 mm. Reverse the slider 5 mm:
   that must be possible at no more than 2 N, without damage. A free-running
   slider, an immovable hard jam or a moving bag is an invalid reset.
6. Restore the same caught depth and initial mark after the reverse check;
   repeat the forward 2 N check, then remove the balance. Open both robot hands.
   Record bag/zip identity, mass, depth and displacement. Start only with the
   stated geometry and checks satisfied. If it fails, remake the snag; do not
   change the fabric, add hidden restraints or increase the test force mid-series.

A ruler, kitchen scale and spring balance are operator preparation tools and
are removed before the trial. The reusable material takes a few minutes to
reset; sewing is a one-time preparation. The simulator represents the loaded
bag as fixed and approximates the snag by a compressive friction rule. The
physical 2 N check standardizes preparation; it is **not** a claim that the
simulation's friction has been calibrated to this bag.

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

## Revision of 16 November 2026

Added the contents, lining attachment and repeatable snag/reset protocol requested
in PR #2355. The instruction, scene, material, controller, scorer and native
recordings are unchanged. Both published traces were independently replayed
again; the existing media therefore still shows the simulated starting condition.
The physical preparation is a specified protocol, not an already-run hardware trial.
