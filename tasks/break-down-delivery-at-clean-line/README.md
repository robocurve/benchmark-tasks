# Break down a supply delivery at the clean-storage line

A delivery of three supply boxes arrives in an opened brown shipping carton on the dirty side of a partition, with a
brown corrugated insert lying on top of them. Clean storage, where corrugated cardboard must never go, is on the other
side. The robot has to put the insert aside on the dirty side and pass only the supply boxes across, through the
pass-through in the partition, onto the clean shelf. The left arm reaches the carton but not the shelf, and the right
arm the shelf but not the carton, so each box goes from one arm to the other through the pass-through.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=break_down_delivery_at_clean_line.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Unpack the brown carton on the left: put the brown cardboard insert on the dark grey pad, and pass the blue, green and orange boxes through the grey pass-through in the yellow partition onto the teal shelf on the right. When you finish, no brown cardboard may be over the pass-through or past the partition." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/clean-line-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=break_down_delivery_at_clean_line.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Unpack the brown carton on the left: put the brown cardboard insert on the dark grey pad, and pass the blue, green and orange boxes through the grey pass-through in the yellow partition onto the teal shelf on the right. When you finish, no brown cardboard may be over the pass-through or past the partition." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/clean-line-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/clean-line-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once all three boxes are on the shelf and the insert
is on the pad, as in the table below (each at rest), for 2.5 s; otherwise it scores the sum of the credits below at
the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_break_down_delivery_at_clean_line.py` in an
environment with `yam_arena` installed.

## Scoring

| Credit | For |
|---|---|
| 0.3 | each supply box on the clean shelf: its whole footprint (its real outline from above, corners included however the box is turned, not a bounding box) inside the shelf top's edges, its bottom within 3 mm of the shelf top, lying on one of its faces (any face: the boxes are one colour all over) tilted at most 10 degrees, at rest |
| 0.1 | the insert on the breakdown pad: its centre inside the pad's outline, its bottom within 3 mm of the pad's top, its 18 x 6 cm face down (either side) tilted at most 15 degrees, at rest |
| 0 overall | at the end of the trial, any part of the insert past the partition's left face (1 cm left of the midline), or any part of its real outline from above overlapping the pass-through's outline, at any height |

The cardboard rule is judged on the final state only (when the trial ends, at success or at the time limit), not on
the path taken: an insert carried over the pass-through and brought back to the pad before the end is not penalised.
At rest means moving under 1 cm/s and turning under 0.5 rad/s. A box on the pass-through, on the clean-side table,
stacked on another box, in the air, still being carried, held still in the gripper above its place (the demo lets go
8 mm up, beyond the 3 mm height tolerance), tipped onto an edge or on the floor earns nothing; so does an insert
standing on its long edge or held above the pad. On a real table, a photo from above shows each footprint against the shelf and pad
edges and where the insert lies relative to the partition and the pass-through; a ruler checks the bottoms' heights,
a phone level app the tilts, and the trial video whether anything is still moving.

The scorer sees only the end state, so it doesn't check that a box went through the pass-through rather than around
an end of the partition. Reach rules that route out: the left arm can't reach past 5 cm right of the midline, so it
can't reach the shelf, and the right arm can't reach the carton. A box has to change arms near the midline, and the
partition covers the midline from 10 to 55 cm in front of the bases. Nearer than 10 cm is the arms' own bases, and
beyond 55 cm is past the 40 cm the arms reach on the midline.

## The scripted demo

The left arm lifts the insert off the boxes, gripping it across its width, and lays it flat in the middle of the
breakdown pad, which lies between the carton and the partition. It then takes the boxes in order, blue, green, orange, gripping each across its sides, and sets each
one in the middle of the pass-through, letting go 8 mm above it; before the green and the orange it holds the box
2.5 s above the carton so that the right arm has left the pass-through. The right arm starts toward the pass-through
1.6 s after the left arm rises from it, picks the box up the same way, and stands it on the shelf, the first 6.5 cm
beyond the shelf's centre, the second at the centre and the third 6.5 cm nearer the arms. The left arm ends parked
12 cm in front of its base and 11.5 cm out to its left, beyond the carton's left wall and clear of the pad; the right
arm ends parked 15 cm in front of its own base; so the camera sees the insert on the pad and the boxes on the shelf.

## Runtime and seeds

- The demo's plan takes 56.1 s of simulated time (the right arm's; the left arm's takes 44.9 s), plus the 2.5 s hold,
  within the 2,400-step (120 s) limit; replayed offline, it reaches success at 56.7 s.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
