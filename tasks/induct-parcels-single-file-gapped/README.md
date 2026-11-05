# Induct the parcels onto the sorter lane single file, with a gap between them

Four cartons lie jumbled in a tote: one stacked on another, one touching its neighbour, one turned 45 degrees. The
robot has to lay them on an induction lane the way a worker at a manual induction station does: one behind another,
flat, long side along the lane, at least a hand's width (10 cm) apart, because a sorter's scanner treats two parcels
that touch or lie side by side as one (a double feed). The lane is a stopped belt, so the gaps are the robot's work,
not the belt's. Only the left arm reaches the tote and only the right arm reaches the lane's right half, so two
cartons have to pass through a staging pad between the arms.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=induct_parcels_single_file_gapped.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Feed the four parcels from the blue tote onto the grey induction lane in single file: each lying flat between the rails with its long side along the lane, at least 10 cm of clear lane from the next parcel, and never side by side with or on top of another." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/induct-parcels-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=induct_parcels_single_file_gapped.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Feed the four parcels from the blue tote onto the grey induction lane in single file: each lying flat between the rails with its long side along the lane, at least 10 cm of clear lane from the next parcel, and never side by side with or on top of another." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/induct-parcels-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/induct-parcels-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once all four cartons are inducted in single file with
gaps of at least 10 cm and have stayed so for 2.5 s; otherwise it scores the partial credit below at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_induct_parcels_single_file_gapped.py` in an
environment with `yam_arena` installed.

## Scoring

A parcel is **on the lane** when any part of it is over the lane slab and its bottom is between 5 mm below and 2 cm
above the lane's top, or at most 2 cm above the top of a parcel on the lane whose footprint it overlaps (a stack). A
carton still carried higher than that is skipped, as if it were still in the tote. It is **inducted** when, at rest
(moving under 1 cm/s and turning under 0.5 rad/s): its footprint is between the rails (at most 3 mm past a rail's
inner face) and within the lane's ends; its bottom is within 5 mm of the lane's top; it is tilted at most 10 degrees
from lying flat on one of its two largest faces (either way up: the cartons are unmarked); and its long side is
within 15 degrees of the lane's length.

The parcels on the lane are taken in order along it, and each neighbouring pair is scored on the clear lane between
them, measured along the lane between their nearest points:

| Credit | For |
|---|---|
| 0.25 | each neighbouring pair, both inducted, at least 10 cm apart |
| 0.1 | instead of 0.25: the pair 5 to 10 cm apart |
| 0 | the pair less than 5 cm apart, or either parcel in it not inducted |
| 0.25 | all four inducted and all three gaps at least 10 cm (1.0 in all) |
| 0 overall | any two parcels on the lane less than 1 cm apart along it, or overlapping along it: side by side, stacked, or touching end to end (a double feed) |

A parcel on the lane that is not inducted (turned more than 15 degrees, on its side, on a rail, hanging past an end)
still sits in the line and breaks both pairs it is in. A parcel left in the tote, on the pad, on the table or on the
floor, or still carried more than 2 cm up, is skipped, so the pairs it would have formed are missing. Which carton goes where along the lane does not
matter, and there is no upper limit on a gap other than the lane's ends (90 cm hold the four cartons with 10 cm gaps
and 18 cm to spare). The untouched start scores 0.

On a real table, a tape measure along the rail gives each gap and a photo from above shows the line, the cartons'
angle to the rails and that none share any stretch of the lane.

## The scripted demo

The left arm lifts the orange carton off the brown one, turns it 90 degrees and lays it on the lane 7 cm left of the
midline. It picks the purple carton across its short side, squares it to the lane and sets it on the yellow pad,
then lays the brown carton at 30 cm left of the midline, sets the red carton on the pad long side front to back,
and parks above the empty tote. The right arm waits until the left arm has left the pad, lays the purple carton at
16 cm right of the midline, waits again for the red carton, turns it 90 degrees and lays it at 39 cm right of the
midline, then parks in front of the lane's right half. Every carton is let go 8 mm above the lane or pad. The clear
gaps along the lane are 12, 13.5 and 13 cm. The left arm turns its grip toward its own side (-90 degrees) and the
right arm toward its own (+90 degrees), which keeps both wrists inside their joint limits.

## Runtime and seeds

- The demo's plan takes 51.6 s of simulated time (the right arm's); the last carton is let go at about 48 s. Run
  with the second command above on the official simulator, the scripted demo scores 1.0 (the demo video ends with all
  four cartons single file on the lane). In the kinematic replay of the same plan the trial reaches success at 51.2 s
  (about 1,025 steps at 20 Hz) including the 2.5 s hold, well within the 2,400-step limit.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
