# Build a mixed pallet layer so each top carton sits level on two equal-height cartons

Four base cartons, two short blue ones (4 cm) and two tall green ones (6 cm), go onto a small pallet deck, and two
orange cartons go across them as the next layer. An orange carton laid across a short and a tall carton rests on
one edge and would slide off in transit, so each one has to lie across two cartons of the same height. Each arm
starts with one short and one tall carton, so the equal-height pairs are made of one carton from each arm: the obvious
build, each arm stacking its own two cartons in a column and bridging them, is the wrong one and scores 0. No carton
has to pass from one arm to the other: the left arm lays both orange cartons across the midline.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=mixed_pallet_level_layer_pairs.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stand the four blue and green cartons upright on the brown pallet, then lay each orange carton level across two of them that are the same height." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/mixed-pallet-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=mixed_pallet_level_layer_pairs.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stand the four blue and green cartons upright on the brown pallet, then lay each orange carton level across two of them that are the same height." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/mixed-pallet-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/mixed-pallet-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once all four base cartons stand on the deck and both
orange cartons lie level across equal pairs, everything still for 2.5 s; otherwise it scores the partial credit
below at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_mixed_pallet_level_layer_pairs.py` in an
environment with `yam_arena` installed.

## Scoring

| Credit | For |
|---|---|
| 0.03 | each base carton standing flat on the deck (a blue one on either 6 x 6 cm face, its 4 cm edges within 5 degrees of vertical; a green cube, which has no up side, on any face, one set of its edges within 5 degrees of vertical), its bottom within 3 mm of the deck's top face (on the deck, not on another carton), its footprint entirely on the deck, and still |
| 0.44 | each orange carton on either big (13 x 6 cm) face across two counted base cartons whose top faces are within 5 mm of the same height: resting on their top faces (its lowest point within 0.5 mm of them, so a carton held even 1 mm above them doesn't count; in a real trial the gripper must have let go of it), at least 40 mm of its length over each, a gap of at most 20 mm between them along its length, at most 15 mm of it past the outline of the two on any side, tilted at most 2 degrees, and still |
| 0.22 | instead of 0.44: the same, but tilted more than 2 and at most 5 degrees |
| 0 | for an orange carton resting on one base carton, on the deck, on the other orange carton or anywhere else, or still carried |
| 0 overall | an orange carton down on a base carton standing on the deck (that base carton's footprint within the deck's outline; footprints overlapping by at least 10 mm each way, seen from above; and the orange carton's lowest point at most 1 cm above that carton's top face) and tilted more than 5 degrees: a case across a short and a tall carton (about 11 degrees here) or on its edge. A carton carried higher than that, at any tilt, or tipped onto a base carton off the deck (still on the table or knocked off), doesn't trigger this |

All four base cartons (0.12) and both orange cartons (0.88): 1.0. Doing nothing scores 0. Each arm bridging its own
blue and green carton scores 0. All four base cartons on the deck, no orange carton on an equal pair and none tilted
down on a base carton: 0.12.

On a real table, a ruler checks each carton's bottom against the deck and the orange cartons' height above it, a
phone level app their tilt, and a photo from above the overlaps, the gap between the supports and the overhang.

## The scripted demo

The right arm picks up its green carton and stands it on the deck in the back row (29 cm in front, 3.5 cm right of
the midline), then its blue carton in the front row (19 cm in front, 3.5 cm right), turning each a quarter turn so
its fingers close front to back over the deck, and parks above its starting marks. The left arm waits above its blue
carton until then, stands it in the front row 3.5 cm left of the midline and its green carton in the back row, so
the two blues form one pair and the two greens the other, with 1 cm between the cartons of each pair and 4 cm between
the rows. It then picks up each orange carton across its width, turns it a quarter turn so its long side runs side to
side, and lays the first across the two blues (centred on the midline, 19 cm in front) and the second across the two
greens (29 cm in front), letting go 7 mm above them; each one has 6 cm of its length over each support and none past
them. The left arm parks above its starting marks so the camera sees the finished layer.

## Runtime and seeds

- The demo's plan takes 60.9 s of simulated time; in the offline replay it reaches success at 61.5 s (about 1,230
  steps at 20 Hz, including the 2.5 s hold), about half of the 2,400-step limit. A Dream-1 trial runs at most
  2,400 steps (120 s of simulated time).
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
