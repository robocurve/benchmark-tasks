# Get the stones out of a running shoe: tip it opening-down with the toe up, shake all 8 out, set it back down

A men's US 9 road running shoe stands on the table with eight gravel pebbles inside, most of them in the toe box under the closed upper. The right gripper pinches the shoe's heel pull tab at its root (one finger in front of it, one behind), lifts it, turns it toe-up past vertical so the opening faces down, shakes it along its length, turns it until the opening faces straight down for the last stones, shakes again, and sets it back down on its sole.

## Run

From this folder, with the harness's Python on a GPU host:

```
python demokit.py task.py OUT                      # final film, 640 x 360, GPU to yourself
python demokit.py task.py OUT --no-record --mode full,none,collar_up,heel_up,on_side,drop
python check_score.py                              # the scorer on hand-built states, no simulator
```

The harness's instruction (Dream-1 run): `--instruction "There are some small stones in this shoe. Get them all out, then put the shoe back down on the table, sole down."`

`SHOE_LAYOUT=B` turns the shoe to a 50 degree heading, moves it 7 cm and changes the stones' places.

## Scoring, and the routes that score less

A stone is in while its centre lies within the shoe's outline above the sole (over the collar opening, up to 1 cm above the edge, so a stone on the lip is still in). The count is read from the current state at the end; only the void latches.

| Score | When |
|---|---|
| 0.25 | at least 2 of the 8 stones out |
| 0.5 | at least 4 out |
| 0.75 | at least 6 out, or all 8 out with the shoe not set down |
| 1.0 | all 8 out, the shoe resting on its sole on the table within 20 degrees of level, both grippers open and every part of them (fingers, pads, palm, housing, wrist flange) at least 3 cm from the shoe, held for the last 2 s |
| 0 | the shoe ever moves faster than 0.45 m/s (a 1 cm fall) with no gripper closed on it (dropped or thrown), or ends off the table |

| Mode | What it does | Score (GPU) |
|---|---|---|
| full | pull tab pinched, toe turned up 135 degrees and shaken, then 162 degrees and shaken along and across the shoe, set down, hand clear | 1.0 (624 steps); with `SHOE_LAYOUT=B`, 1.0 (644 steps) |
| none | nothing | 0.0 |
| collar_up | lifted and shaken upright, opening up | 0.0 |
| heel_up | opening turned down with the toe kept low (heel up), shaken | 0.0 |
| on_side | emptied as in full, then rolled toward its side and let go | 0.0 (the roll is out of the wrist's reach: the shoe fell from the hand at 1.7 m/s, void) |
| drop | emptied as in full (7 out), then let go 10 cm above the table | 0.0 (fell at 0.72 m/s with no hand on it, void) |
| Dream-1 | the model, given the instruction above, 320 x 176 cameras, 1200 steps, seed 1 | 0.0 (it pushed the shoe a few centimetres across the table and never lifted it; all 8 stones stayed in) |

All 8 out with the shoe still held or not upright scores 0.75 (`check_score.py`). The task is one-handed: the second arm is not needed, as when a person tips a stone out of a shoe with one hand.

## Simulation

- The shoe is one rigid free body: 141 convex pieces built in `task.py` (sole, side walls 4.5 mm thick, a closed vamp over the toe box, a solid padded heel counter and toe cap, and a 12 mm thick, 30 mm wide rigid heel pull tab standing 35 mm above the counter, the hand's only handle), so the cavity is open only at the collar. Physics at 240 Hz on the GPU, with CCD on each body and on the scene, contact offsets of 2 mm (shoe) and 1.5 mm (stones).
- The stones are irregular convex hulls 4.1 to 8.0 mm long, mass from hull volume at 2.65 g/cm3 (0.04 to 0.28 g).
- The demo pinches the tab with about 30 N a finger (the YAM's figure is about 40 N), lifts 0.3 kg, and holds the toe torque (about 0.35 N m with the toe up, against about 1.2 N m the grip holds); it runs 624 of the 1200 steps (31 s). It reads only the arm's own joints; it never reads the stones or the shoe's pose.

## Real trial

Parts: one lace-up road running shoe with a heel pull tab, US 9 (the Nike Pegasus 41 itself, or any trainer of this size: a used or budget pair of about $30 does as well), and about 20 pea-gravel stones sieved between 4 mm and 8 mm (a bag of washed pea gravel and two kitchen sieves, a few dollars; estimate). Pick the 8 whose lengths run from 4 to 8 mm.

Reset (about 2 minutes): stand the shoe on the table, laces tied, heel tab at the mark; drop the 8 stones in through the collar; tip the shoe toe-down once and set it back on its sole so they lie in the toe box.

Score: lift the shoe and count the stones left on the table; tip the shoe into a hand to confirm none is inside. A phone video of the whole trial (the hand opening and backing off at least 3 cm, and any drop) and a photo of the table and of the shoe's resting pose (level within 20 degrees, by eye against a 20 degree wedge cut from card) is the whole record. The same rule runs in `score()`: a stone is out once it is clear of the shoe's outline.

Protection: a table mat so stones do not roll away, and a rubber pad on each fingertip so the pinch does not mark the tab.

## Sources

- Nike Pegasus 41 men's road running shoe (nike.com/t/pegasus-41-mens-road-running-shoes-LMhfRGdO): 297 g, 10 mm drop. Fleet Feet Pegasus 41 listing: 37/27 mm stack.
- Nike men's footwear size chart (nike.com/size-fit/mens-footwear): US 9 = foot length 10 5/16 in (262 mm); the 290 mm outer length adds toe room (estimate). Collar opening 100 x 67 mm, widths along the shoe, pull tab and upper heights are estimates.
- Wentworth (1922) grade scale: fine pebble 4 to 8 mm (zenodo.org/record/1431467). Quartz density 2.65 g/cm3 (webmineral.com/data/Quartz.shtml).
- World Footwear Yearbook 2025 (APICCAPS), via La Conceria, 'Some buy, some produce: the map of shoe consumption in 2024' (laconceria.it/en/footwear/some-buy-some-produce-the-map-of-shoe-consumption-in-2024): the US bought about 2.1 billion pairs in 2024, about 10% of the world total.
- J. Clarke, How to keep stones out of your shoes on the trail, Advnture, 3 Jun 2023 (advnture.com/how-to/keep-stones-out-of-shoes).
- Friction (0.5 stones, 0.6 shoe) and zero restitution are estimates.
- All meshes are built in `task.py`; no third-party meshes are used.

## Licence

MIT
