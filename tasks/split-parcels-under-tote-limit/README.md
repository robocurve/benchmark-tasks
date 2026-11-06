# Split five parcels between two totes without going over the tote weight limit

Five parcels with stated weights of 4, 3, 3, 2 and 2 kg (14 kg in all) go into two totes that may hold at most 7 kg
each, so each tote must hold exactly 7 kg: the red 4 kg parcel with one 3 kg parcel, and the other 3 kg parcel with
both 2 kg parcels. The red, yellow and orange parcels start on the left half of the mat, so loading each tote with
the parcels nearest it puts 8 kg in the left tote and scores 0. Each tote is in front of one arm and only that arm
reaches it, so both arms pack, and the red and one 3 kg parcel have to be taken by the arm on the far side of the mat.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=split_parcels_under_tote_limit.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pack all five parcels into the two grey totes, each set flat on a tote's floor, side by side and not stacked, so that neither tote holds more than 7 kg. The red parcel weighs 4 kg, the blue and green parcels 3 kg each, and the yellow and orange parcels 2 kg each." \
  --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/split-parcels-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=split_parcels_under_tote_limit.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pack all five parcels into the two grey totes, each set flat on a tote's floor, side by side and not stacked, so that neither tote holds more than 7 kg. The red parcel weighs 4 kg, the blue and green parcels 3 kg each, and the yellow and orange parcels 2 kg each." \
  --scorer task_score --max-steps 1400 --store-frames --log-dir ~/yam-eval/split-parcels-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/split-parcels-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once all five parcels are packed, neither tote over
7 kg, and every parcel has been still for 2.5 s; otherwise it scores the partial credit below at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_split_parcels_under_tote_limit.py` in an
environment with `yam_arena` installed.

## Scoring

| Credit | For |
|---|---|
| 0.2 | each parcel packed, while the split can still be completed: the parcels over neither tote can each be added to one tote without either tote's load going over 7 kg |
| 0.1 | each parcel packed, once the split can no longer be completed (four parcels packed two and two, leaving a parcel that fits in neither tote, score 0.4) |
| 0 overall | either tote's load over 7 kg: the stated weights (red 4 kg, blue and green 3 kg, yellow and orange 2 kg) of every parcel whose centre is over the tote's outer outline, walls included, at any height or tilt |
| 0 overall | any parcel on the floor (its bottom more than 5 cm below the table top) |

A parcel is packed when it has been set down in a tote: its outline seen from above inside the tote's inner outline
(or within 5 mm past it), its bottom within 5 mm of the tote's floor (a parcel held above the floor isn't packed), its
top no higher than the rim (8 cm above the table), and resting flat on any one of its faces, tilted at most 10 degrees
(the parcels are unmarked, so no side is "up"). All five packed scores 1.0.

A parcel left on the mat or elsewhere on the table earns nothing itself, but it decides whether the split can still be
completed. Stacking can't pack a parcel: every parcel is at least 5 cm tall, so one on top of another sticks out above
the 8 cm rim, but its weight still counts toward the tote's load.

On a real table, a photo from above shows which parcels are over which tote and whether each is inside the inner
outline; a ruler checks that no parcel's top is above the rim or its bottom above the floor, and a phone level app
its tilt.

## The scripted demo

Every grip is turned 90 degrees, the fingers closing front to back, so the open fingers stay clear of the
neighbouring parcels on the mat and of the tote walls. Each parcel is set down at the tote's centre line, in a row
from side to side, and let go 8 mm above the floor. The left arm packs the yellow and then the orange parcel into
its tote while the right arm packs the blue parcel into its own and then waits above it. The right arm then takes the
red parcel from the left half of the mat to its tote; once it has carried it away, the left arm takes the green
parcel from the right half of the mat to its tote. Each arm then parks in front of its own base so the camera sees
both totes: the left tote holds green, yellow and orange (7 kg), the right one red and blue (7 kg).

## Runtime and seeds

- The demo's plan takes about 37 s of simulated time; with the 2.5 s hold, success comes after about 760 steps at
  20 Hz, well within the 1,400-step limit. On an L40S, once Isaac Sim has started, a scripted trial takes about
  1 minute of wall time and a full Dream-1 trial 2 to 3 minutes (the first start on a new machine takes about 4
  minutes more).
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
