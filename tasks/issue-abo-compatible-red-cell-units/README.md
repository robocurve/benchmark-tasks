# Issue one ABO-compatible red-cell unit to each patient tray

Three patients wait for red cells: one of group O (blue tray), one of group A (yellow tray) and one of group B (pink
tray). The issue shelf holds two group O units (blue), one group B unit (pink) and one group AB unit (white), and no
group A unit. A patient can receive their own group or group O, so the correct issue is O to the O patient, O to the A
patient, B to the B patient, and the AB unit stays on the shelf. Matching tray colour to unit colour leaves the A
patient without a unit and tempts the robot to fill that tray with the leftover AB unit, which in a real blood bank
causes an acute haemolytic reaction. Only the left arm reaches the shelf and only the right arm reaches the trays, so
every issued unit crosses the table by a handover on the pad between the arms. All units are RhD-compatible and
crossmatched: only the ABO check is tested.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
INSTRUCTION="Issue red-cell units from the shelf on the left to the patient trays on the right, handing each one across on the grey pad in the middle, and put exactly one compatible unit, standing upright, in each tray: a unit's colour is its ABO group (blue = O, pink = B, white = AB), a tray's colour is its patient's group (blue = O, yellow = A, pink = B), a patient can receive their own group or group O, and no group A units are in stock. Leave any unit that no patient can receive on the shelf."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=issue_abo_compatible_red_cell_units.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/abo-issue-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=issue_abo_compatible_red_cell_units.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/abo-issue-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/abo-issue-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once every tray holds its compatible unit, seated and
upright, the white unit on the shelf, with every unit still (moving under 1 cm/s and turning under 0.5 rad/s) for
2.5 s; otherwise it scores the partial credit below at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_issue_abo_compatible_red_cell_units.py` in an
environment with `yam_arena` installed.

## Scoring

Definitions. A unit is **over a tray** when its centre is inside that tray's inner outline (9 x 12 cm) or within 5 mm
past it, at any height: a unit held over a patient's tray counts as issued to that patient. It is **on the shelf** when
its centre is over the shelf's base plate (31.5 x 11 cm), at any height and in any pose. It is **loose** when it is
neither and either its lowest point is less than 5 cm above the table top, or it rests on another unit (its lowest
point within 5 mm above that unit's top, their outlines overlapping seen from above). The white unit is **in
storage** when its centre is over the shelf's base plate and its lowest point is at most 2 cm above the plate's top.

| Credit | For |
|---|---|
| 0 overall | the white (AB) unit not in storage, held or not (over the pad, over a tray, just off the shelf, on the table, or lifted more than 2 cm straight up out of its slot) |
| 0 overall | any unit loose: set down, dropped or knocked over on the pad, the table or the floor, or let go on top of other units outside the trays |
| 0 overall | the pink (B) unit over the blue or yellow tray |
| 1/3 | each tray with exactly one unit over it that is compatible (blue tray: blue only; yellow tray: blue only; pink tray: pink or blue), seated (its lowest point from 8 mm below to 0.5 mm above the tray floor), upright (its 9 cm axis within 15 degrees of vertical, either end up) and still (moving under 1 cm/s and turning under 0.5 rad/s) |
| 0 for that tray | empty, two or more units over it, or its unit held above the floor, resting on a wall or another unit, tilted more than 15 degrees, or still moving |

Full score, 1.0: a blue unit in the blue tray, a blue unit in the yellow tray, the pink unit in the pink tray and the
white unit on the shelf. A blue unit given to the B patient is safe but wastes O stock: the A or O patient then goes
without, so at most 2/3. A blue or pink unit outside the shelf and trays whose lowest point is 5 cm or more above the
table with no unit under it (in practice, still held in a gripper mid-handover at time-out) earns nothing but does not
zero the score; let go on top of other units, it is loose and zeroes the score. The white unit gets no such exemption:
carrying it anywhere outside the shelf's base-plate outline, at any height, or lifting it more than 2 cm straight up
over the plate zeroes the score. Knocking the white unit over inside its slot changes nothing; pushing a unit off the
shelf, the pad or the table zeroes the score.

On a real table, a top-down photo shows which unit's centre is over which tray, the shelf or neither; a ruler checks
that each tray's unit stands on the floor (its top 9.4 cm above the table) and a phone level app its tilt.

## The scripted demo

Three issues, one after the other: the blue unit nearest the arms goes to the yellow tray, the pink unit to the pink
tray, and the far blue unit to the blue tray. For each, the left arm grips the unit from above across its 4 cm side
(the tool point 5.5 cm above the unit's bottom, the fingertips clear of the dividers), lifts it to 14 cm above the
table, holds it there 2 s, and stands it on the pad, letting go 6 mm above it. Half a second after the left arm has
risen clear, the right arm picks the unit from the pad with the same grip and stands it in the centre of its tray,
letting go 6 mm above the floor with the fingertips 1.2 cm above the walls. No carry passes over a standing unit. At
the end the left arm parks beside the shelf and the right arm between the pad and the trays, so the camera sees the
trays.

## Runtime and seeds

- The demo's plan takes 47.9 s of simulated time (the right arm's plan; the left arm's ends at 36.8 s); with the
  2.5 s success hold it fits well within the 2,400-step (120 s) limit.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
