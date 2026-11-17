# Peel a garlic clove with a silicone rolling peeler

Peel the garlic clove with the silicone tube and leave the peeled clove out on the board.

A familiar kitchen preparation step: press and roll a compliant peeler enough to loosen the papery skin without crushing the clove, then empty it onto the board.

## Run and score

Use the pinned Python 3.12 / Isaac 6.1 / YAM environment in `ENVIRONMENT.json`.
From the repository root, with an NVIDIA GPU and the ordinary Dream SDK credential:

```sh
OMNI_KIT_ACCEPT_EULA=YES python tasks/peel-garlic-with-a-silicone-peeler/run.py --mode demo --out /tmp/peel-garlic-with-a-silicone-peeler-demo
OMNI_KIT_ACCEPT_EULA=YES python tasks/peel-garlic-with-a-silicone-peeler/run.py --mode dream --out /tmp/peel-garlic-with-a-silicone-peeler-dream
python tasks/peel-garlic-with-a-silicone-peeler/rescore.py /tmp/peel-garlic-with-a-silicone-peeler-demo
python tasks/peel-garlic-with-a-silicone-peeler/test_model.py
```

The fixed reset has no random seed. Both modes have the same 1,600-action limit
(80 seconds at 20 Hz), material model, robot and observations. Dream receives
standard RGB cameras and joint state. The demo issues saved ordinary bounded
joint/gripper commands with joint feedback. Material response uses measured hand
poses; it never reads the demo stage, intended target, policy mode or score.
`--cpu-preview` is a planning check, not submission evidence.

## Setup and reset

On a 550 x 500 x 3 mm dry wooden chopping board, place a green food-grade silicone tube (120 mm long, 35 mm outside diameter, 1.5 mm wall), horizontal across the robot, with one intact, unpeeled garlic clove already inside at its midpoint. Use a firm fresh white garlic clove, 28-36 mm long, 16-22 mm wide and 4-7 g, at 18-22 C. No soaking, cutting or pre-crushing. Both hands start open. README specifies positions, sourcing and reset.

Coordinates are metres in the YAM station: x points away from the robot,
y to its left, table height 0.7475 m. Board centre is (0.37, 0, 0.749).
Tube centre is (0.34, 0, 0.768), axis along y. The single clove starts centred
inside, its long axis along the tube. Loading is preparation, not a tested skill.
The robot must press, roll and recover the clove; it is not already loosened.

Use ordinary firm white garlic (*Allium sativum*) from one retail batch, with dry
unbroken papery skin and no sprout or soft spots. Select using a ruler and kitchen
scale; record the batch/variety and keep it consistent. The simulator's tapered
18 x 32 x 21 mm clove is representative, not a scanned or calibrated cultivar.
Keep the produce at 18-22 C; do not heat, refrigerate immediately before use,
soak, trim or bruise it. Use a dry reusable silicone rolling peeler within the
stated dimensions. This is the ordinary [press-and-roll method](https://www.oxo.com/garlic-peeler.html),
not a claim that the generic model reproduces a particular commercial product.
Use thin removable food-safe silicone finger covers and a washable board; stop
and clean after a crushed clove. Experimental garlic is not for consumption.

Use one fresh, intact clove from the same garlic variety and size band per trial. Photograph its intact skin before loading. Remove retained skins; wash and fully dry the tube and board. Inspect protective finger covers and replace if contaminated. Preparation takes about two minutes.

## Judging

Judge the final intact clove. 0.25 for at least 50% of its skin removed; 0.5 for at least 75%, recovered on the board and at least 5 mm clear of the peeler; 0.75 for at least 95% with the same recovery; 1.0 when that result also remains released for 3 seconds. A clove still in or touching the tube is capped at 0.25. Otherwise 0. Crushing, splitting or losing the clove off the work surface gives 0. Estimate remaining skin from photographs of all sides; an operator may tip an unrecovered clove out only for inspection, without rubbing it.

Photograph the final clove from above and below, turning it only after the trial.
Use a simple 20-cell overlay on the combined views to estimate skin-covered
area: at most one cell equivalent is the 95% endpoint, five is 75%, ten is 50%.
Count attached papery skin, not colour differences in exposed flesh. An operator
may empty the tube to inspect hidden skin, but this does not earn recovery credit
and must not scrape or peel any skin. Record recovery and the three-second
hands-off hold before that inspection. Reject a visibly split or crushed clove.
The simulator uses 128 skin patches as an approximate surface-area assay;
force and internal abrasion values are diagnostic, not real-world scoring tools.

## Fidelity

Native YAM arms and cameras. A reduced silicone ellipse compresses under measured finger pads and rolls with their tangential motion. Loaded relative sliding detaches local skin patches; gravity empties the tilted tube. Grip capture, clove shape, skin adhesion and crushing thresholds are approximate and uncalibrated. Detached skin stays in the peeler. No prescribed peeling trajectory or score-triggered object motion.

All material parameters are engineering approximations. The mesh shown in each
frame is the current simulated state. They are not calibrated force or material
measurements, and do not establish real-hardware success.
## Recorded trials

- Demo: **1.0**, 870 actions / 43.5 simulated seconds; 140.7 wall seconds including startup.
- Dream: **0.0**, 1600 actions / 80 simulated seconds; 635.0 wall seconds including startup.

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
python tasks/peel-garlic-with-a-silicone-peeler/replay_evidence.py demo
python tasks/peel-garlic-with-a-silicone-peeler/replay_evidence.py dream
```
