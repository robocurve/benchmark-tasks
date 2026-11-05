# Sandbag each light stand on the leg under its boom

Two light stands, each with three low legs and a short yellow boom near the top of its post, stand one in front of
each arm (the left post 40 cm in front of the left arm's base, the right post 30 cm in front of the right arm's base
and 10 cm further out). Set convention for a stand carrying a boom is to put the sandbag on the leg under the boom,
the side the load hangs over; a bag on another leg is the rookie mistake a key grip corrects, and here it scores 0.
The stands are fixed, so the score checks that convention rather than computing a tipping risk. The booms point different ways (the left one back toward
the arms, the right one toward the centre line), so the right leg has to be read from each stand, and the right bag
must be turned a quarter turn to lie along its leg. Each stand is in reach of one arm only, so a full score takes
both arms.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=sandbag_the_leg_under_the_boom.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Weight both light stands with the red sandbags, one bag per stand, following set convention: lay each bag flat and lengthwise along the leg directly under that stand's yellow boom, its centre 7 to 13 cm out from the post." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/sandbag-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=sandbag_the_leg_under_the_boom.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Weight both light stands with the red sandbags, one bag per stand, following set convention: lay each bag flat and lengthwise along the leg directly under that stand's yellow boom, its centre 7 to 13 cm out from the post." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/sandbag-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/sandbag-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The score is taken at the last step; the trial ends early at 1.0 once both
stands earn full credit and both bags are still (moving under 1 cm/s and turning under 0.5 radians a second) for
2.5 s.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_sandbag_the_leg_under_the_boom.py` in an
environment with `yam_arena` installed.

## Scoring

Each stand is scored on its own; either bag can count for either stand.

| Credit | For |
|---|---|
| 0.35 | per stand, a bag lying on the leg under the boom: resting on one of its 70 x 50 mm faces (either one, so upside down counts), not on a side or end face, within 10 degrees of level; its lowest point touching the leg's top (1.5 mm above to 3 mm below it; the leg's top is 20 mm above the table); its centre within 20 mm of the leg's centre line and 70 to 130 mm out from the post's centre along the leg |
| 0.15 | per stand, on top of that: the bag's long side within 20 degrees of the leg's direction |
| 0 for the stand | no bag meets the 0.35 conditions: on the table, held or hanging more than 1.5 mm above the leg, stacked on the other bag, wedged between two legs, resting on a 70 x 35 or 50 x 35 mm face, or its centre outside 70 to 130 mm |
| 0 overall | any bag lies on a leg that is not under a boom: its lowest point 1.5 mm above to 3 mm below that leg's top and its centre within 20 mm of that leg's centre line, anywhere along it |

A bag still in the gripper counts if its lowest point is within the 1.5 mm above to 3 mm below band; the score does
not check release.

On a real table, a tape measure along the leg gives the bag centre's distance from the post's centre and its offset
from the leg's centre line (the leg is 40 mm wide, so a centre over the leg is within 20 mm); a photo from above
shows the bag's long side against the leg's direction, and a side photo shows the bag's bottom touching the leg's
top (no gap: a 1.5 mm feeler gauge does not slide under it), not on the table.

## The scripted demo

Both arms work at once. The left arm picks up the left bag from above, gripping across its 50 mm width, carries it
to the left stand and lays it lengthwise along the leg that points back toward the arms, its centre 100 mm from the
post, letting go 8 mm above the leg; it rises straight up and parks over the bag's starting spot. The right arm
picks up the right bag the same way, turns it a quarter turn while carrying it, lays it lengthwise along the right
stand's leg that points toward the centre line, 100 mm from the post, and parks over its starting spot, so the
camera sees both stands.

## Runtime and seeds

- The demo's plan takes 11.6 s of simulated time per arm; in the offline replay both bags are down at 8.6 s and the
  2.5 s hold completes by 11.7 s (about 235 steps at 20 Hz), well within the 1,200-step limit. In the GPU run the
  scripted demo reported `task_score` 1.0 and the trial ended early, on success, at step 222 (11.1 s of simulated
  time); it took 47 s of wall time, simulator start-up included. The Dream-1 trial ran all 1,200 steps (score 0.0)
  and took 178 s of wall time.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
