# Pull a file from the shelf and stand an out-card in its place

A file is pulled from a row on a records shelf for a reader, and a red out-card goes into its exact place so the gap
stays open and anyone can see the file is out. The blue file has to reach the reader stand, which only the right arm
can reach; the out-card starts in a holder only the left arm can reach. Without the card in the gap the score is 0:
a file out with nothing marking its place is how paper records get lost. The yellow, green and orange files must stay
where they are, in order.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pull_file_leave_out_card.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the blue file out of the row on the grey shelf and stand it in the purple stand on the right, then stand the red out-card from the holder on the left in the blue file's place, between the green and orange files, without moving the other files." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/out-card-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pull_file_leave_out_card.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the blue file out of the row on the grey shelf and stand it in the purple stand on the right, then stand the red out-card from the holder on the left in the blue file's place, between the green and orange files, without moving the other files." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/out-card-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/out-card-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the out-card stands in the gap, the blue file
stands in the reader stand and the other three files are in place, all still for 2.5 s; otherwise it scores the
partial credit below at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_pull_file_leave_out_card.py` in an
environment with `yam_arena` installed.

## Scoring

A file or the card is **standing** when its thickness axis (the 3 cm or 2.5 cm direction) is within 15 degrees of
horizontal, whether it stands on its base or on an end; it is **upright** when, as well, its 15 cm or 16 cm height
edge is within 15 degrees of vertical, whichever end is up (a uniform block looks and stands the same upside down); it
is **knocked over** when its thickness axis is more than 45 degrees from horizontal (lying on a broad face).

| Credit | For |
|---|---|
| 0.4 | the red out-card in the gap: its centre within 20 mm of where the blue file's centre started, standing (on any edge or end), its bottom within 5 mm of the shelf board |
| 0.4 | with the card in the gap and the blue file off the board: the blue file upright in the reader stand, its centre inside the slot's outline (or at most 5 mm past it), its bottom within 5 mm of the stand's base |
| 0.2 | with the card in the gap: the yellow, green and orange files all in place, each centre within 15 mm of its start, upright, its bottom within 5 mm of the board |
| 0 overall | the out-card not in the gap |
| 0 overall | the blue file's centre still over the shelf board (inside the board's outline or at most 5 mm past it): slid along the row, not pulled |
| 0 overall | any file or the card knocked over, or fallen off the table (its centre below the table top) |
| 0 overall | any file's bottom less than 5 mm above the table top (standing or lying on the bare table) |
| 0 overall | the yellow, green or orange file's centre outside the shelf board's outline (5 mm tolerance), or its bottom more than 5 mm below the board's top |

The board's outline runs from 19.5 to 33 cm in front of the arm bases and from 9.5 cm right of the midline to 15.5 cm
left of it. The green and orange files leave a 9 cm space around the blue file's old centre; a 2.5 cm card whose
centre is within 20 mm of it stays at least 12.5 mm clear of both, so any card standing alone in that space counts.

On a real table, a ruler from tape marks on the board checks each centre, a phone level app the tilts, and a photo
from above shows where every file and the card stand.

## The scripted demo

Both arms grip straight down with the fingers closing across a file's or the card's thickness. A file is too tall to
lift over its neighbours, so each arm works the row from its open front. The right arm narrows its fingers to 40 mm
in front of the shelf, slides them into the 3 cm gaps on either side of the blue file, closes, lifts the file 5 mm,
slides it forward out of the row, carries it past the shelf's right end, lowers it into the reader stand and lets go
7 mm above the base, then backs out along the slot and parks in front of the stand. Meanwhile the left arm slides its
open fingers around the out-card from the front, lifts it out of the holder and holds it in front of the holder until
the blue file has left the midline (11.5 s). It then carries the card in front of the gap, lowers it to 7 mm above
the board, slides it into the gap, opens to 40 mm, backs out, and parks in front of the holder.

## Runtime and seeds

- The demo's plan takes about 21 s of simulated time; with the 2.5 s hold, success is due by about 20 s (400 steps),
  well within the 1,200-step limit. On an L40S, once Isaac Sim has started, a scripted trial takes about 1 minute of
  wall time and a full Dream-1 trial 2 to 3 minutes.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
