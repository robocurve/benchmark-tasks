# Prepare an electric griddle: brace handle to seat heat control probe, turn dial on, and place bread

Files for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene).

Instruction:
`Brace the electric griddle by its handle to seat the heat control probe, turn the dial on, and place both slices of bread on its cooking surface.`

## Task summary

Short-order cooks and food service workers (SOC 35-2015, 35-2014) prepare countertop griddle surfaces for food preparation. The workstation features a Presto 07211 Liddle Griddle (1.19 kg cast aluminium, 21.6 x 26.7 cm cooking plate) and a detachable Control Master heat control probe with rotary thermostat dial. Because the lightweight griddle rests on phenolic feet, pushing the 3-pin detent probe into the shroud slides the unbraced pan across the counter, requiring bimanual coordination to brace the griddle handle while inserting the probe, rotating the dial 90 degrees into its active cooking range, and placing both bread slices flat on the cooking surface.

## Run a trial

Scripted demonstration (scores 1.0):

    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=tasks/indoor-griddle-preparation/indoor_griddle_preparation.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
      --instruction "Brace the electric griddle by its handle to seat the heat control probe, turn the dial on, and place both slices of bread on its cooking surface." \
      --scorer task_score --max-steps 1600 --store-frames

Model run (Dream-1):

    inspect-robots run --embodiment yam_arena --policy dream \
      -E task=tasks/indoor-griddle-preparation/indoor_griddle_preparation.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
      --instruction "Brace the electric griddle by its handle to seat the heat control probe, turn the dial on, and place both slices of bread on its cooking surface." \
      --scorer task_score --max-steps 1600 --store-frames

Offline self-tests (no simulator required):

    python3 tasks/indoor-griddle-preparation/test_task.py

## Precedent & Novelty

This benchmark task evaluates coordinated bimanual assembly and food preparation with an electric cooking appliance:
- **Distinction from PR #2019 (`commercial-griddle-cleaning`)**: PR #2019 focuses on scraping and squeegeeing grease from a stationary flat-top restaurant range. In contrast, this task evaluates bimanual mechanical preparation of an electric griddle (bracing the free 1.19 kg pan against sliding while inserting a 3-pin spring detent heat control probe, turning the temperature dial ON, and placing breakfast bread).

## Real-World Trial & Physical Rigging

- **Appliance & Sourcing**: Household 10.5" x 8.5" electric griddle (Presto 07211 / Liddle Griddle specification, cast aluminium cooking surface, dual cool-touch handles) and Control Master heat control probe with rotary thermostat dial.
- **Physical Safety in Real Trials**: In real physical robot evaluations, the griddle power cord can remain unplugged or connected to an isolated current-limited bench supply / multimeter continuity probe that confirms terminal engagement without heating the element.
- **Food & Props**: Synthetic silicone bread slices (35 g, 80 x 60 x 27 mm) with high-friction food-grade silicone skin; ceramic serving tray.
- **Asset Licensing**: All mesh geometries are procedurally authored primitive assemblies in `mechkit.py` licensed under Apache 2.0 / MIT. No proprietary meshes or third-party assets are required.

## Scoring

Scoring evaluates physical progress and enforces physical constraints:
- **0.0 (latched for episode)**: if any key item falls off the counter, food is placed before the probe is seated, or probe is dislodged at end of trial.
- **0.25**: heat control probe seated home into the griddle shroud with the griddle upright and stable on the counter.
- **0.50**: probe seated AND thermostat dial turned on (>= 15 deg from OFF).
- **0.75**: probe seated, dial turned ON, and 1 bread slice resting flat on the cooking surface.
- **1.00**: probe seated, dial turned ON, both bread slices resting flat and non-overlapping inside the cooking surface, and both robot arms withdrawn clear (>= 20 mm).
