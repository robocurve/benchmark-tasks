# Park the carts in a hospital corridor so 140 mm always stays clear

A model hospital corridor, 280 mm wide, runs across the table in front of the arms. It has four grey parking bays,
one on each wall on each side of a pair of red door zones. Two deep blue carts (80 mm) wait in the room at the right
end and two shallow green carts (40 mm) in the room at the left end. The carts may stand in the corridor only while
at least 140 mm (half its width) stays clear between carts facing each other. Two blue carts face to face leave at
most 120 mm, so each half of the corridor needs one blue and one green cart. Each arm reaches only its own room and
its own half, so each arm has to pass one of its carts to the other on the yellow pad in the middle.

The scale is 1:10.9: the corridor stands for 3.05 m (10 ft), the 140 mm minimum for the 1.52 m (5 ft) that NFPA 101
requires to stay clear when in-use wheeled equipment stands in a hospital corridor, a blue cart for a 0.87 m deep
crash cart or workstation on wheels, a green cart for a 0.44 m deep isolation cart.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=park_corridor_equipment_keep_clear_width.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Park the two blue carts (80 mm deep) and the two green carts (40 mm deep) in the four grey corridor bays, one per bay, each flat against its wall with its 70 mm side along it, so that at least 140 mm stays clear between carts facing each other across the corridor. Keep the red door zones clear. You may pass carts between the arms by setting them on the yellow pad, but leave nothing on it at the end." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/corridor-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=park_corridor_equipment_keep_clear_width.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Park the two blue carts (80 mm deep) and the two green carts (40 mm deep) in the four grey corridor bays, one per bay, each flat against its wall with its 70 mm side along it, so that at least 140 mm stays clear between carts facing each other across the corridor. Keep the red door zones clear. You may pass carts between the arms by setting them on the yellow pad, but leave nothing on it at the end." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/corridor-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/corridor-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once all four carts are parked with at least 140 mm
clear in both halves for 2.5 s; otherwise it scores the partial credit below at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_park_corridor_equipment_keep_clear_width.py`
in an environment with `yam_arena` installed.

## Scoring

A cart is **parked** when all of these hold:

- it is still;
- it is tilted at most 10 degrees, either way up (the stand-in box has no top or bottom);
- its 70 mm side is within 10 degrees of the wall's direction;
- it is set down: its bottom is no more than 1 mm above the bay's 1 mm floor sheet and no more than 3 mm below it
  (a cart standing on the sheet, or on the table beside it), so a cart still held more than 1 mm up doesn't count;
- seen from above, its footprint is inside one grey bay's 100 mm length along the wall, at most 3 mm past either
  end (into the red door zone or past the black stop);
- its nearest point is within 10 mm of that bay's wall face.

The **clear width** of a half of the corridor (left or right of the centre line) is defined when both of its bays
hold a parked cart. It is the narrowest gap between the near-wall cart and the far-wall cart, measured perpendicular
to the walls at every point along the corridor where both carts are present. A blue and a green cart parked flush and
square leave 160 mm; two blue carts leave at most 120 mm. The 10 mm and 10 degree tolerances don't by themselves
guarantee 140 mm: a cart set off its wall or turned reaches farther out, and the gap is measured wherever it is
narrowest.

| Credit | For |
|---|---|
| 0.25 | each parked cart |
| 1.0 | all four parked, at least 140 mm clear in both halves |
| 0 overall | either half's clear width under 140 mm |
| 0 overall | any cart that is not parked lies even partly over the corridor floor between the walls' inner faces and within their 44 cm length: on the pad, in a door zone, more than 10 mm off its wall, turned, on its side or end, stacked, resting on a stop or wall, still moving, or still held more than 1 mm above the floor |
| 0 overall | any cart, anywhere, tilted more than 30 degrees or fallen off the table |
| 0 for that cart | a cart left outside the corridor (in a room at either end, or elsewhere on the table); it doesn't zero the run |

Egress is a series path: one pinch point or one blocked door zone stops every bed behind it, so a narrowed corridor
earns nothing rather than partial credit. A cart left on the handover pad is an obstruction, so staging earns
nothing. Three carts parked correctly with the fourth still in its room score 0.75. The naive layout, where each arm
parks its own pair, puts the two blue carts face to face (112 mm clear) and scores 0.

On a real table, a tape measure laid across the corridor between the two carts of a half, at their ends and along
their facing sides, gives the clear width (the smallest reading), and from each wall gives each cart's gap to it; a
photo from above shows each footprint against the bay, door zone
and pad markings.

## The scripted demo

Every cart is gripped straight down across its 70 mm length, the fingers closing along the walls, so no finger comes
between a cart and a wall. The fingertips pass 7 mm above the end stops. Each cart is set down 4 mm off its wall,
letting go 6 mm above the bay. The right arm sets the first blue cart on the yellow pad, then parks the second in
the right far bay. Meanwhile the left arm parks a green cart in the left near bay, takes the blue cart off the pad to
the left far bay, and sets the other green cart on the pad. The right arm then takes that cart to the right near
bay. Both arms end parked above their empty rooms. Each half ends with a green cart at the near wall (44 mm out) and
a blue cart at the far wall (84 mm out): 152 mm clear.

## Runtime and seeds

- The demo's plan takes 51.3 s of simulated time (1,026 steps at 20 Hz). With the 2.5 s success hold it uses 53.8 s
  (1,076 steps), just under 60% of the 1,800-step (90 s) limit.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
