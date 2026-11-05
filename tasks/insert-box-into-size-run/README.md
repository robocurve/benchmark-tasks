# Put a returned box back into its place in a size run

A shelf channel holds one style's size run: a red (small) box against the left end stop and a green (large) box next
to it. The blue (medium) box, a return, has to go back between them, not at the end of the row. That means moving the
green box along to open a slot first, then closing the row up against the left end stop and the back stop. The blue
box starts on a mat only the left arm can reach, and the channel is only in the right arm's reach, so the box has to
cross the midline from one arm to the other.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=insert_box_into_size_run.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the blue box back into the shelf channel in front of the right arm, between the red and green boxes, so the row reads red, blue, green from its left end, closed up with every box standing against the back." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/size-run-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=insert_box_into_size_run.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the blue box back into the shelf channel in front of the right arm, between the red and green boxes, so the row reads red, blue, green from its left end, closed up with every box standing against the back." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/size-run-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/size-run-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the row reads red, blue, green, closed up and
against the back stop, with all three boxes still (moving under 1 cm/s and turning under 0.5 rad/s) for 2.5 s; otherwise it scores the partial credit below at the
last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_insert_box_into_size_run.py` in an
environment with `yam_arena` installed.

## Scoring

A box *stands in the channel* when its centre is over the floor strip (or within 5 mm past its outline), its bottom is
within 5 mm of the strip's top, and it is tilted at most 10 degrees.

| Credit | For |
|---|---|
| 0.1 | a slot: the gap between the red and green boxes at least 6 cm (the blue box's width), or the blue box standing between them |
| 0.5 | in order: the blue box standing in the channel between the red and green boxes, so the row reads red, blue, green from the left |
| 0.2 | with the order: closed up, the red box within 5 mm of the left end stop and both gaps between neighbouring boxes 5 mm or less |
| 0.2 | with the order: against the back, for every box both bottom corners of its back face within 5 mm of the back stop's face, measured at the bottom of the box. The back face is the box's 6 cm x 9 cm face towards the stop, so each box must stand with its 6 cm side along the row: a box turned a quarter turn (a 4 cm end towards the stop) gets no back credit, and a box turned more than 4.8 degrees from square can't get it either (its corners are 6 cm apart; 6 cm x sin 4.8 degrees = 5 mm) |
| 0 overall | any box tilted more than 10 degrees, or off the table (its bottom more than 1 cm below the table top) |
| 0 overall | the red or green box no longer standing in the channel, or the green box left of the red one |
| 0 overall | the blue box over the channel anywhere but standing between the red and green boxes: at the end of the row, or on top of another box |

On a real table, a ruler or feeler gauge checks the gaps along the row and between each box and the back stop, a phone
level app the tilt, and a photo from above the order and how square each box stands.

## The scripted demo

The right arm grips the green box from above across its 4 cm depth, its fingers in front of and behind the box and
above the back stop. It lifts the box, sets it down 6.4 cm further right and lets go 8 mm above the strip, which
leaves a 6.5 cm slot. At the same time the left arm picks the blue box off the mat, gripping across its 6 cm width,
and sets it down on the table on the midline, 20 cm in front of the arm bases, then parks to its left. The right arm
picks the blue box up there, again across its depth, carries it over the red box's top with 1.3 cm to spare, and
lowers it into the slot, letting go 8 mm above the strip. That leaves 2.5 mm on each side, 1 mm between the red box
and the left end stop, and 1 mm behind every box. The right arm then parks to its right so the camera sees the row.
The demo lifts the boxes rather than pushing them, because the offline replay can check lifted boxes but not pushed
ones. Sliding the green box along the channel with closed fingers is just as valid; the scorer only looks at the end
state.

## Runtime and seeds

- In the demo run the blue box was let go at about 21 s, and with the 2.5 s hold success came by about 24.5 s
  (about 490 steps at 20 Hz), well within the 1,200-step limit. The demo scored 1.0.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
