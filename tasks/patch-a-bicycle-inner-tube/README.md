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

### Consumables

Use a **Decathlon 20-inch 1.5/1.9 Schrader butyl tube, product 8640844,
ETRTO 38/47-406**, with no sealant, and one fresh **Park Tool GP-2 pre-glued
patch** per trial. Use the same product/lot throughout a comparison. The
manufacturer's preparation requires a clean, dry, lightly scuffed tube, followed
by firm patch pressure. Handle the patch by its non-adhesive face or edges.

- [Tube specification](https://www.decathlon.co.uk/p/20-x-1-5-to-1-9-schrader-valve-inner-tube/331538/m8640844)
- [GP-2 preparation instructions](https://www.parktool.com/assets/doc/product/GP-2_instructions.pdf)
- [Manufacturer's repair and inflation guidance](https://www.parktool.com/en-us/blog/repair-help/inner-tube-repair)

Measure and record the actual patch before the session; use the untrimmed
square GP-2 patch, with side length **24–26 mm**. Do not substitute a circular
vulcanizing patch. The simulated 24 × 24 × 0.4 mm orange sheet approximates
this consumable; its colour, thickness and adhesive parameters are not claimed
as measured GP-2 properties. Keep products at **20–22°C** for the session.

### Repeatable preparation

1. Start with a fresh tube. Before puncturing it, fit it in a matching 406 mm
   wheel and 20 × 1.75 tyre rated to include 40 psi. Verify that the intact tube,
   valve and connected pump pass the pressure check below. Deflate and remove it.
2. Choose a smooth outer-face position opposite the valve, at least 30 mm from
   a mould seam. Wipe a 40 × 40 mm area with a lint-free cloth lightly dampened
   with 70% isopropyl alcohol; let it dry completely for at least 2 minutes.
   Do this away from the robot. Do not use oily cleaners or tyre dressing.
3. Pull the two tube walls apart. Make **one** perpendicular penetration through
   the upper wall with a **1.2 mm diameter smooth round pin**, using a 3 mm depth
   stop. Do not twist, cut a slit or puncture the lower wall. Remove the pin.
   The rubber hole may close elastically: 1.2 mm specifies the puncturing tool,
   not a claim about the relaxed hole diameter. Mark four short ticks 18 mm
   from the hole, outside the patch's bonding area.
4. Lightly abrade a **35 × 35 mm** square centred on the puncture using the
   kit's sandpaper on a flat 20 × 20 mm block: five gentle strokes in each of
   two perpendicular directions, without stretching the tube. Stop once the
   surface is uniformly matt. Reject a gouged/thinned area or an area still
   glossy after this preparation; use a fresh tube, not extra force.
5. Brush away sanding dust with a clean dry lint-free cloth. Check that the
   surface is dry, free of dust and untouched by fingers. Before patching,
   confirm air escapes from this hole alone using minimal rounding inflation
   and a dry tissue; then vent the pressure. Reject another leak or an absent
   puncture. No soap is applied to the prepared bonding surface.
6. Lay the tube in the photographed **420 × 320 mm** oval, hole upward at the
   front. Smooth the repair area without tension. The simulated surface is a
   supported curved approximation of the vented, naturally rounded rubber.
   Peel a fresh patch immediately before the trial and place it adhesive-side
   down on a clean **35 × 35 mm silicone-coated release card**, 70 mm nearer
   the robot and 115 mm right of the hole. Touch only the non-adhesive face;
   open the hands and begin within 2 minutes. Do not apply extra adhesive.

The robot must place and press the patch; the operator may not improve its bond
before judging. Use a fresh prepared tube and patch for the next trial. A repaired
or torn target is not reset by peeling its patch off. Keep the release card and
adhesive off the shared gripper pads. Preparation takes a few minutes, excluding
the initial intact-tube pressure check. These dimensions and preparation limits
are the benchmark protocol, not manufacturer performance tolerances.

## Judging

1.0: intact patch seals the puncture and remains sealed with both hands released for 3 seconds. 0.75 for a sealed patch still held or before the hold ends. 0.25 for a partly bonded patch covering the puncture but still leaking. Otherwise 0. Damage overrides all credit. Physical check after release: install in the specified 20-inch tyre, inflate to 40 psi, and lose at most 2 psi over 5 minutes after a 60-second settling period, without adding air. README.md defines the pump/gauge check and partial-credit inspection. Do not inflate a GP-2 repair outside a tyre.

Park Tool specifically advises against testing its pre-glued repair by inflating
the tube outside a mounted tyre. Accordingly the old exposed-tube soap test is
retired. After the robot's release hold, the operator may **only** install the
repaired tube in the same checked 20 × 1.75 tyre and 406 mm rim, without rubbing,
repositioning or pressing the patch. Inflate to **40 psi**, leaving a hand pump
with a readable inline pressure gauge connected. Allow 60 seconds to settle,
set it to 40 psi once, then record the pressure at 0 and 5 minutes without adding
air or reconnecting the gauge. A loss of at most **2 psi** passes. Use a gauge
with divisions of at most 1 psi; first verify the intact-tube control in the same
wheel/pump setup. A leaking valve, hose or rim damage invalidates the trial rather
than blaming the patch. Record the released patch and gauge readings.

For 0.25 physical credit, the patch must cover the marked hole and visibly remain
attached after release, but fail the pressure check. A loose patch earns zero.
The 0.75 stage is provisional until the same leak check passes: a failed check
cannot retain sealed-patch credit. This operator assay is outside the robot's
action budget; it adds no robot assistance and does not claim roadworthiness.

The simulator requires at least12 of81 local adhesive sites for partial
attachment credit; a loose patch or a single-point tack receives0. Its leak
test checks whether an unbonded route connects the hole to any patch edge.
This grid is a disclosed approximation of the physical leak check; it has not been calibrated to
the specified product or pressure-test tolerance.

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

## Revision of 16 November 2026

Specified tube/patch products, the puncturing tool, marking, cleaning, abrasion
and fresh-consumable reset requested in PR #2362. The physical leak-check protocol
was clarified to respect the patch manufacturer's tyre-supported inflation rule.
The simulated interface-connectivity scorer and material are unchanged; both
published native traces were independently replayed again at 1.0 / 0.0. Existing
media is retained. No physical seal or product calibration is claimed.
