# Hoist a bucket onto a scaffold deck

Hoist the bucket up to the scaffold deck with the gin wheel and land it there.

Roofers’ helpers (O*NET 47-3016.00), construction labourers (47-2061.00) and riggers (49-9096.00) haul material pails with gin wheels, hold suspended loads and land them on scaffold boards while keeping hands out from underneath. Usually one hand hauls the rope over the wheel while the other draws the pail over the boards by its upright bail, and rope is paid out to land it.

## Run and score

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=hoist_bucket_onto_scaffold_deck.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Hoist the bucket up to the scaffold deck with the gin wheel and land it there." \
  --scorer task_score --max-steps 1000 --store-frames --log-dir demo-run
```

Use `--policy dream` for a model trial with credentials configured. Run time is 1000 steps (50 seconds at 20 Hz). One fixed reset, no seed-dependent geometry. `task_score` is the last-step score.

Highest current tier; measure with a tape measure, photo or video. Hand positions are pad-centre points from robot joint readings. Bottom means the pail's lowest outside corner. Landed: bottom no more than 1 mm above the deck top (z ≤ 888.5 mm) and no more than 5 mm below it, pail centre over the deck (x 100..260, y 50..210), tilt at most 10°. A pail held or hung more than 1 mm over the boards is not landed. Rope on the wheel: a rope link centre at least 10 mm above the axle, its whole 8 mm width inside the 12.2 mm channel, and its surface at least 25 mm from the axle.

| Score | Condition |
|---|---|
| 1.0 | Landed, all footprint corners at least 20 mm inside each deck edge, rope on the wheel, each hand at least 30 mm from the pail, its bail and any part of the rope; distances are to each piece's bounding box, including every rope link. Hold for 1 s; each pail corner moves less than 1 mm over that second. |
| 0.5 | Landed, each pail corner moving less than 1 mm over 1 s, but the 20 mm margin or the 30 mm release fails. |
| 0.25 | Bottom more than 1 mm above deck-top height (z > 888.5 mm), anywhere, hanging on the rope with the rope on the wheel; its centre moves less than 5 mm over 1 s (turning on its hook is allowed); bottom height also changes less than 5 mm. Rope attachment gap at most 10 mm; load-fall length at most 8 mm longer than its straight span. |
| 0 | Otherwise. |

Sticky 0 after bottom descent over 400 mm/s or tilt over 45°. Sticky 0 if no link is on the wheel as defined above, or if any link within 55 mm of the axle (and no lower than 12 mm below it) is less than 25 mm from the axle at its surface or 28 mm at its centre (rope sunk into the wheel). If the simulator reports an invalid position for the pail or the rope, the trial scores 0. Also sticky 0 if, while the pail is not landed and its bottom is over 30 mm above the table, a hand passes inside the pail footprint below its bottom; or, while bottom is over 10 mm above its 2 mm start and below deck top minus 5 mm, a hand is within 12 mm of pail/bail boxes or a gripper at most 50% open comes within 15 mm of the rope between the pail and the wheel. Sticky 0 if, in that same bottom-height band, the rope is not carrying the pail for more than 0.5 s in a row. Carrying means attached (the first rope link's end within 10 mm of the swivel hook) and taut (the load fall, from the hook to the first link within 55 mm of the axle and no lower than 12 mm below it, at most 8 mm longer than its straight span); a hand, wrist or forearm lifting the pail leaves the load fall slack. Success is 1.0, held a further 1 s.

## Setup and reset

An open steel pail stands on the table beside a scaffold deck, hooked to a rope that runs over a gin wheel hung from a scaffold standard. The stand and deck are fixed; the wheel turns freely.

Centres and sizes are in mm: x away from the arms, y to their left, z up; table top z=747.5. Box sizes are x/y/z. Values are rounded to 0.5 mm, except the 12.2 mm wheel channel; masses to the gram. No other task objects are present.

Scaffold standard: grey steel box, centre (320,-30,982.5), size 18 x 32 x 470.
Putlog cantilever: grey steel box, centre (260,-30,1207.5), size 138 x 25 x 18.
Wheel hanger: grey steel box, centre (200,-30,1177.5), size 8 x 12 x 60.
Axle: grey steel cylinder, centre (196,-30,1147.5), diameter 6, length 24 along x; wheel rotation axis through (180,-30,1147.5).
Wheel: blue painted steel, 65 g total. Groove cylinder centre (180,-30,1147.5), diameter 52, width 12.2 along x. Two flange discs, diameter 76, thickness 2, centres (173,-30,1147.5) and (187,-30,1147.5), also along x. Clear channel between flanges: 12.2.

Deck boards 1–4: brown timber boxes, each 160 x 40 x 12; respective centres (180,70,881.5), (180,110,881.5), (180,150,881.5), (180,190,881.5). Together their top is z=887.5, spanning x=100..260, y=50..210; no toe board.
Ledgers 1–2: grey steel boxes, each 12 x 160 x 12; centres (120,130,869.5), (240,130,869.5).
Deck legs 1–4: grey steel boxes, each 12 x 12 x 116; centres (120,70,805.5), (120,190,805.5), (240,70,805.5), (240,190,805.5).

Pail: open silver steel container, outside 60 x 60 x 70, 300 g including bail and ears. Its bottom starts 2 above the table at x=180, y=0, with 20 clearance to the deck’s near edge.
Pail bottom: silver steel box, centre (180,0,751.5), size 60 x 60 x 4.
Pail -x/+x walls: silver steel boxes, each 4 x 60 x 66; centres (152,0,786.5), (208,0,786.5).
Pail -y/+y walls: silver steel boxes, each 52 x 4 x 66; centres (180,-28,786.5), (180,28,786.5).
Bail -x/+x legs: silver steel round wire, each diameter 6, length 40 along z; centres (146,0,839.5), (214,0,839.5).
Bail top wire: silver steel cylinder, centre (180,0,859.5), diameter 6, length 68 along x. The bail stays upright, rising 40 above the rim.
Bail -x/+x ears: silver steel boxes, each 8 x 8 x 4; centres (148,0,817.5), (212,0,817.5).

Rope: 8 mm brown fibre rope, length 1.06 m, mass 64 g. It hooks to the middle of the bail’s top wire with a swivel hook at (180,2.5,859.5), runs straight up about 280 mm, over the wheel’s top in its groove, then down the hauling side at y=-62.5. It bends onto the table and ends in a straight 260 mm tail toward -y, from (180,-93,753.5) to (180,-353,753.5). The rope initially clears the groove and table by 2 mm. Surface friction is 0.8.

Restore the pail upright at its listed starting position, with the rope over the wheel and its tail straight toward -y. Put the hands back at their starting positions. Use the same layout each trial. Nothing is consumed.

## Fidelity

The rope is a chain of short rigid links with light bending springs and no stretch. The sheave is a free wheel on a lightly damped axle. The steel bail stays fixed upright. Static and dynamic surface friction are 0.8. Release is judged from robot joint-state FK. Geometry and friction approximate small-scale construction equipment.

Scores: Dream-1 run: 0.0; demo: 1.0.
