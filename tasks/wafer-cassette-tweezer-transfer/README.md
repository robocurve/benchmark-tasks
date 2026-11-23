# Move a 2-inch wafer with tweezers from one cassette into the empty slot of another

A wafer stands upright in a 25-slot 2-inch cassette between two neighbours. The robot takes the wafer tweezers out of
their stand, grips the wafer by its top edge, lifts it out between the neighbours (4.76 mm pitch), carries it to a
second cassette, lowers it into the one empty slot between two wafers until it rests on the slot's rails, lets go, and
stands the tweezers back in their stand.

## Why this is done by hand

Cleanrooms that run small wafers move them between cassettes with tweezers:

- UT Dallas cleanroom, spin rinse dryer procedure: "Transfer your wafers with tweezers into a clean carrier boat,
  return the Teflon boat to the Spin Rinse Dryer, and close the door."
  https://cores.research.utdallas.edu/manuals/avenger-spin-rinse-dryer/
- UTEP Nanomaterials Integration Lab, cleaning procedure: "Make an effort to handle the substrates by the edge of the
  wafers to limit scratching to the edges." https://www.utep.edu/engineering/nanomil/Processes/cleaning.html

What differs from handling a wafer lying flat: here the wafer is upright and has to come out of, and go into, a slot
between neighbours 4.76 mm apart, held only at its edge by a sprung two-leaf tool.

## Running a trial

    # one trial; TRIAL=k picks layout k mod 3 (A, B, C)
    TRIAL=0 inspect-robots run --embodiment yam_arena -E task=tasks/wafer-cassette-tweezer-transfer/task.py \
        -E object_jitter=0 --scorer task_score --max-steps RUN_LIMIT --instruction "$(jq -r .instruction task.json)"

    TRIAL=0 python demo.py OUT_DIR          # the closed-loop scripted demo (left arm)
    python test_scoring.py                  # scorer tests, no simulator
    python parts.py                         # rewrites assets/*.usda (already committed)

Runtime per trial: up to 2,400 control steps (2 minutes of simulated time); the demo finishes in about 900 to 1,300
steps. No random seeds: the layout is fixed per trial number (object_jitter=0).

## Layouts (rotate by trial number)

| Layout (trial k mod 3) | Near cassette: wafers in slots (move the middle one) | Far cassette: wafers in slots (target = the empty one between) |
|---|---|---|
| A (0) | 8, 9, 10 (move 9) | 12, 14 (target 13) |
| B (1) | 12, 13, 14 (move 13) | 16, 18 (target 17) |
| C (2) | 16, 17, 18 (move 17) | 8, 10 (target 9) |

Slot 1 is the right-hand end of each cassette (seen from the robot).

## Scoring and real-trial checks

Judged at the end of the trial (or once 1.0 has held for 1 s). No credit while the tweezers hold the wafer: a score counts
only once the tips are open and at least 5 cm from it (trial video).

| Score | Condition | Real-trial check |
|---|---|---|
| 1.0 | the wafer stands in the target slot, fully down, and the tweezers stand tips down in their stand, robot at least 3 cm away | photo from above and from the side: wafer between the two far wafers, top edges level within 2 mm (ruler across the three tops); tweezers upright in the stand |
| 0.75 | as 1.0, but the tweezers are not back in the stand (or the robot still touches them) | same photo; tweezers elsewhere |
| 0.5 | the wafer stands in the target slot but its top edge is more than 2 mm above its neighbours' | ruler across the three top edges |
| 0 | anything else: another slot, across two slots, tilted more than 5 degrees, still in the tweezers | photo |

Zero for the whole trial (latched), whatever follows:

| Event | Simulator test | Real-trial check |
|---|---|---|
| dropped: a release above its resting place | the wafer, free of the tweezers after being lifted, moves down faster than 0.2 m/s (a fall of 2 mm or more) | trial video: the wafer falls after the tips open |
| another wafer moved or lifted | any other wafer 2 mm or more from its start | ruler / photo against the start photo; video |
| the moved wafer touched the table | its lowest point within 1 mm of the table | trial video |

`test_scoring.py` checks every level and each zero, with negative controls (held wafer, wrong slot, back in its own
slot, across two slots, tilted, neighbour displaced, dropped, table touch, tips not clear).

## Parts

| Part | Maker, part number | Size | Material | Price | Link |
|---|---|---|---|---|---|
| Wafer cassette (x2), modelled | Chung King, "2" PP Cassette" PP-C050 | 142 x 67.4 x 61.5 mm, 25 slots, pitch 4.76 mm, 82 g | PP | not listed (request) | https://www.ckplas.com/nen/ppcst_2.htm |
| Wafer cassette, priced equivalent | MSE Supplies CA0108, "MSE PRO 5 pcs of 2 inch Diameter 25 Wafers Carrier Box" | 25 wafers, 2 inch | PP | $60.95 per 5 (listing) | https://www.msesupplies.com/products/mse-pro-5-pcs-of-2-inch-diameter-25-wafers-carrier-box-polypropylene-cleanroom-class-100-grade |
| Pocket data for the model | Entegris 2" carrier PA72-20M (Specialized Handling Products catalog, p. 27) | pocket spacing 4.76 mm, pocket flat 1.02 mm, D1 14.0 mm, 142 x 68 x 59 mm | PP / PFA | not listed | Entegris "Specialized Handling Products" catalog, document 5367, p. 27 (entegris.com) |
| Wafers (x5) | MicroChemicals WSM20279250B1324SNN1, CZ-Si 2 inch (100), 1-side polished | 50.8 mm, 279 +/- 25 um, 1.32 g | silicon | EUR 6.50 each (under 49) | https://www.microchemicals.com/CZ-Si-wafer-2-inch-279-um-100-SSP-B-doped/WSM20279250B1324SNN1 |
| Wafer tweezers | Ted Pella PELCO 5048-SV, style 2WF, for 2" wafers | 136.5 mm, points 6 x 0.7 mm | NM stainless, carbon-fibre PVDF tips | $35.90 | https://www.tedpella.com/tweezers-pelco_html/pelco-pro-wafer.aspx |
| Tweezer stand | our printable part (drawn in parts.py) | 30 x 54 x 12 mm block, 16 x 18 mm through slot | PLA | filament only | this folder |

Third-party assets: none. Every part above is drawn by `parts.py` from the dimensions listed; the robot, table and
cameras are the yam_arena station.

## Assumptions (no source gives these; measure before real trials)

- Rib shape inside the cassette: root thickness = pitch - pocket flat = 3.74 mm, tapering to 1.8 mm at the tip, scaled
  from our 72-series (100 mm) carrier model; side-comb teeth reach 1.9 mm inside the wafer's radius.
- Bottom rails 15 mm either side of the centre line, so a seated wafer's top edge is 1.9 mm under the cassette's top.
- Tweezer leaf section 6 x 1.5 mm, 8 mm open tip gap, hinge spring about 1 N at the tips to close, 14 g.

## Simulation notes

- Wafers are drawn 0.5 mm thick at their real mass (1.32 g). Tested in this simulator with the same demo: at the real
  0.279 mm the pinched wafer chattered across its faces (about +-0.5 mm, +-0.1 m/s each step) and was thrown out, or the
  tips closed through it; capping the depenetration speed of the wafers and tweezer leaves at 0.02 m/s removed the
  chatter, but at 0.279 mm and 0.4 mm the wafer still slipped out of the tips when lifted (layout A at 0.4 mm). At
  0.5 mm the tips rest on its faces without overlap and hold it.
- The tweezers are two rigid leaves on a hinge with an angular spring (a passive drive toward the sprung-open rest);
  the gripper's fingers squeeze the leaves 50 mm from the tips, and only contact friction holds the wafer.
- The cassettes and the stand are fixed bodies (taped down in the real trial); wafers and tweezers are free.

## Real trials

Build: two Chung King PP-C050 (or MSE CA0108) cassettes taped to the bench as in the setup, five 2-inch wafers in the
layout's slots, the PELCO 5048-SV tweezers in a printed stand (its dimensions are in `parts.py`, `write_stand`). Before
the first trial, measure and note: each slot pocket's width at its mouth and at its root, and a seated wafer's top edge
below the cassette's top (the model assumes 1.9 mm). Take a photo from above and one from the side before and after each
trial; the scoring table above says what to read from them. Wafers break: wear gloves, and replace any chipped wafer.

## Files

- `task.json`: the task's details. `task.py`: the scene (yam_arena task file) and the scorer (`ladder()` is pure).
- `parts.py`: writes the USD parts in `assets/` (cassette, wafer, stand, tweezers) from the dimensions above.
- `artkit.py`: our hooks that spawn USD parts, the tweezers' sprung hinge and the tip contact sensor in yam_arena.
- `demo.py`: the closed-loop scripted demo. `test_scoring.py`: scorer tests.
