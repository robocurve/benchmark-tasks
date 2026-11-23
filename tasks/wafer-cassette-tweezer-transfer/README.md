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
| dropped: the wafer leaves the tips (they open, or it slips out) before it rests in a slot | when the wafer has been free of both tips for 3 control steps in a row, its position at the first free step: its centre more than 3 mm above the seated height (its top edge more than 3 mm, half a tip's width, above its neighbours'), or not inside either cassette | trial video (side view): the wafer drops or slides down after it leaves the tips |
| the robot touched a wafer | any contact between the robot's fingers, gripper, link 4 or link 5 (either arm) and any wafer | trial video |
| a tweezer tip on the wafer's face outside the edge zone | while the tips grip the wafer, any point of either tip paddle (its tip end and rows 4, 8 and 12 mm up, at its centre and both edges) lying on the wafer's face (within 1 mm of it) more than 6 mm inside the rim | trial video (side view: the tips pushed down more than their own width below the wafer's top edge), and the faces inspected for tip marks under a low-angle lamp after the trial |
| another wafer moved or lifted | any other wafer 2 mm or more from its start | ruler / photo against the start photo; video |
| the moved wafer touched the table | its lowest point within 1 mm of the table | trial video |

`test_scoring.py` checks every level and each zero, with negative controls (held wafer, wrong slot, back in its own
slot, across two slots, tilted, neighbour displaced, let go 20 mm above its slot, let go outside a cassette, robot
touching a wafer, a tip pressed 8 mm into the face, table touch, tips not clear) and the cases that must not count as a drop (let go resting, let go
within 3 mm, a two-step contact flicker while carried, a grip 5 mm below the top edge).

Edge zone and the grip: the tips are 6 mm wide and their paddles 12 mm long. Wafers lean in their pockets (about 2.2 to
2.4 deg in the simulator), so the demo aims at the wafer's own top edge and turns the blades parallel to its faces; it
sets the tip ends 4 mm below that edge (closed loop on the measured wafer, within 0.3 mm) before it squeezes, and while lowering it stops as
soon as the tips start to slide down the faces, so a correct grip stays inside the 6 mm zone (the demo logs the deepest
point, `max_tip_depth_mm`). The tips can physically reach deeper: between neighbours 4.76 mm apart the 0.7 mm tips can
be pushed tens of millimetres down a face, hence the latched zero.

`seated.png` (layout A, after the trial has scored 1.0: the empty left hand looks down at a slant from the robot's
side, with its wrist camera) shows the moved wafer standing in its target slot of the far cassette between its two
neighbours, top edges level. `demo.py` takes it after the scored episode; it is not part of the scored trial or video.

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

- Rib shape inside the cassette: the 1.02 mm pocket flat is the floor of the bottom rails' grooves, where the wafer
  rests (rib root 3.74 mm = pitch - pocket flat, then an assumed 1.8 mm above). The side combs guide the wafer: their
  teeth reach 1.9 mm inside the wafer's radius and are 1.8 mm thick (2.96 mm gaps) as far out as 0.5 mm beyond the
  wafer's edge, thickening to 3.74 mm only outside that.
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

## Quaternion order (measured)

Every rotation the scorer reads goes through one function, `quat_xyzw_to_matrix` in `task.py`, which takes (x, y, z, w).
That is the order this stack hands over, measured in the simulator rather than assumed (Isaac Lab 3.0, which yam_arena
uses, gives poses as x, y, z, w; earlier Isaac Lab versions used w, x, y, z):

- A fixed part spawned at 90 deg about x, then 30 deg about z, read back through yam_arena's object states as
  (0.683, 0.183, 0.183, 0.683), its spawn rotation written x, y, z, w. Read as x, y, z, w, all three body axes match
  the intended rotation (0.0 deg error); read as w, x, y, z they are off by 60, 60 and 76 deg.
- A tweezer leaf (artkit.body_pose, Isaac Lab's body_link_pose_w), spawned at 90 deg about z and standing in its
  stand, read back as (-0.0045, 0.0045, 0.707, 0.7071): within 0.7 deg of its spawn as x, y, z, w, 90 deg off as
  w, x, y, z. Physically, the tweezers stand tips down in the stand and the gripper squeezes their leaves along y; read
  as w, x, y, z, that spawn would be a half turn about (0, 1, 1) and lay them on their side, tips along -y.
- `quat_probe_task.py` and `quat_check.py` in this folder reproduce the measurement: run
  `python quat_check.py OUT.json` and it prints both readbacks and the axis errors for each order.

`test_scoring.py` checks the order and the poses that depend on it with these measured quaternions: the wafer's
orientation (upright, lying flat, turned 30 deg), the tweezer tip positions (tips at the table in the stand, the 8 mm
sprung-open gap) and the upright-stand test (upright, leaning 20 deg, lying, moving, off the slot, lifted).

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
- `seated.png`: the end state of the demo (layout A): the wafer seated in its target slot.
