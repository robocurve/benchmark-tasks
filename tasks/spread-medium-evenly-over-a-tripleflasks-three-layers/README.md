# Roll a TripleFlask flat from a lean so its layers match

**Instruction:** "Spread the medium in the TripleFlask evenly over its three layers, then leave the flask lying flat in
its growth position, imprint up, with both hands off it."

A Thermo Scientific Nunc TripleFlask (cat. 132867, three 170 x 98 mm culture layers at a 15 mm pitch, T175 outside
size) holds 200 mL of dyed water in place of a cell suspension. It has been filled, capped, stood on its closed end so
the three compartments level, and rested on its long side, top (imprint) face toward the robot: the maker's steps 1-4.
The compartments share medium through a slot at the closed end, so while the flask is turned flat the upper layers keep
draining into the lower ones. The maker's technical note says to hold the flask on its side at 75 degrees to the work
surface (leaned back 15 degrees toward its top face, which gives the top layer a head start), then quickly but gently
tilt it into the growth position, imprint up. Rolled straight over, or slowly, the bottom layer ends too full; rolled
the wrong way it lies imprint down; landed hard, medium splashes into the neck.

Sources: Thermo Scientific Nunc TripleFlask technical note
(https://documents.thermofisher.com/TFS-Assets/LSG/Application-Notes/TNLSPTRIPLEFLASK.pdf, "To seed cells"); product
page cat. 132867 (https://www.thermofisher.com/order/catalog/product/132867).

## Run and score a trial

In the simulator's environment (the Robocurve Brev launchable, or yam-isaac-lab-arena's `setup.sh`), from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=spread_medium_evenly_over_a_tripleflasks_three_layers.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Spread the medium in the TripleFlask evenly over its three layers, then leave the flask lying flat in its growth position, imprint up, with both hands off it." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir logs/tripleflask-dream

# The scripted one-hand demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=spread_medium_evenly_over_a_tripleflasks_three_layers.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Spread the medium in the TripleFlask evenly over its three layers, then leave the flask lying flat in its growth position, imprint up, with both hands off it." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir logs/tripleflask-demo
```

The run prints `task_score`, read at the end (latched zeros aside). Runtime per trial: 1,200 steps at most (60 s
simulated); the demo reaches 1.0 at step 327 (16.3 s). One fixed layout (`object_jitter=0`), no seeds. Tests:
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q`.

## Scoring (the scorer is in the task file, after the medium model)

The ladder is task.json's `scoring`. In the simulator the layer volumes are the medium model's three volumes; on a real
bench the scorer opens the cap and, with the flask flat, draws each layer dry through the neck with a 10 mL serological
pipette (top, middle, bottom, its tip guided over the layer's lip through the clear wall) into a 100 mL graduated
cylinder, read to 1 mL. The 1.0 band is 60.0-73.3 mL per layer (10%), the 0.5 band 50.0-83.3 mL (25%).

| Strategy (KIT_DEMO mode) | Cap | MuJoCo (physcheck): end volumes bottom / middle / top |
|---|---|---|
| demo: lean 15 deg, hold, roll in about 0.6 s | 1.0 | 1.0 (70.2 / 66.7 / 63.2 mL, landing 0.29 m/s) |
| none: both arms stay home | 0 | 0.0 (on its side, 66.7 each) |
| no_lean: the same roll straight from its side | 0.5 | 0.5 (77.1 / 66.7 / 56.2 mL) |
| slow_roll: the maker's lean, then a careful 4 s roll | 0.25 | 0.25 (116.2 / 63.2 / 20.6 mL) |
| over_lean: leaned 40 deg (layers at 50), the demo's roll | 0.5 | 0.0 (medium over the neck-end lip into the neck) |
| slam: let go 45 deg into the roll | 0 | 0.0 (landing 0.54 m/s: a splash) |
| held: the demo, the hand stays on the cap | 0 | 0.0 (flat and even, but held) |
| wrong way: rolled onto its top face | 0 | by the scorer: imprint down is never flat in the growth position |

Calibration (real-bench fallback, when a layer cannot be drawn dry): the lean is the key number, the roll time the
second. The model's head start from the 15 degree lean is about 8.5 mL to the top layer; the demo's roll (about 0.6 s from
the lean to flat) drains about 12 mL from it on the way, a roll about 0.1 s longer about 14 mL, a 4 s roll about 55 mL.
The 1.0 band needs the roll to drain between about 2 and 15 mL from the leaned top layer. A trial video's held lean and roll
time read against these rows gives the expected split.

Robustness (one factor at a time, `TASK_SWEEP`, demo mode, MuJoCo):

| Change | Score | End volumes bottom / middle / top (mL) | Landing (m/s) |
|---|---|---|---|
| none (the demo) | 1.0 | 70.2 / 66.7 / 63.2 | 0.29 |
| friction x 0.7 | 1.0 | 69.8 / 66.7 / 63.5 | 0.19 |
| friction x 1.3 | 1.0 | 70.6 / 66.7 / 62.7 | 0.34 |
| mass x 1.2 | 1.0 | 69.6 / 66.7 / 63.7 | 0.12 |
| slot rate x 0.8 | 1.0 | 68.1 / 66.7 / 65.3 | 0.29 |
| slot rate x 1.2 | 1.0 | 72.3 / 66.7 / 61.1 | 0.29 |

## Simulation and the real case

- The flask is one free rigid body at catalogue size (205 x 147 x 56 mm body, 30 mm neck, 40 mm cap as 16 flats),
  0.45 kg with its medium; friction 0.35 with the table and the pads (estimate). Nothing is moved, latched or held by
  code.
- The medium is a declared analytic model (`Medium` in the task file), stepped every 10 ms from the flask's pose: three
  compartment volumes, each with a level surface from its volume and the flask's tilt; neighbours exchange medium
  through the closed-end slot at 2.5 mL/s per mm of level difference while the source stands above the slot's 7 mm lip;
  medium above a neck-end lip where the compartment faces the 28 mm neck bore runs into the neck (more than 0.5 mL there:
  0). A corner reaching the table faster than 0.5 m/s counts as a splash into the neck (0).
- Estimates to check on a real flask: the slot rate, the lip heights, the landing limit and the flask's empty mass
  (0.25 kg). The internal geometry is read from the maker's drawings.
- Real-trial reset: after the layers are read, pour the 200 mL back through the neck, cap it tight, wipe the neck, stand
  the flask on its closed end for 10 s, rest it on its long side on the start marks and wait 10 s.
- Real trials are safe: the load is a capped 0.45 kg plastic flask of tinted tap water, with no sharps, heat, cells
  or chemicals; a drop or splash means only wiping up water and replacing the flask.
