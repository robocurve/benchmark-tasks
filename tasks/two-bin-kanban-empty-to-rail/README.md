# Two-bin kanban: put the empty bin on the reorder shelf and bring the full bin to the front

A shelf lane holds two bins of the same supply item: an empty one at the front and a full one behind it. As in a
hospital supply room's two-bin system, the empty bin has to come out and go on the reorder shelf, where it signals
the buyer, and the full bin then moves up to the front stop so it is the one in reach. The full bin can't reach the
stop until the empty one is out, so the order is set by the lane itself. The reorder shelf is within the left arm's
reach only, and the lane is on the midline, so the left arm clears the empty bin and the right arm then moves the
full one.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=two_bin_kanban_empty_to_rail.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lift the empty red bin out of the front of the grey lane and set it on the orange reorder shelf on your left, then move the full blue bin to the front of the lane, against the stop." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/kanban-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=two_bin_kanban_empty_to_rail.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lift the empty red bin out of the front of the grey lane and set it on the orange reorder shelf on your left, then move the full blue bin to the front of the lane, against the stop." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/kanban-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/kanban-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the red bin stands on the shelf and the blue bin
stands at the stop, both still, for 2.5 s; otherwise it scores the partial credit below at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_two_bin_kanban_empty_to_rail.py` in an
environment with `yam_arena` installed.

## Scoring

| Credit | For |
|---|---|
| 0.5 | the red bin on the shelf: its whole outline seen from above inside the shelf's 15 x 15 cm top, its bottom within 2 mm of the shelf top (set down, not held above it), tilted at most 10 degrees |
| 0.5 | the blue bin at the front: its outline inside the lane's inside (give or take 5 mm), its bottom within 2 mm of the lane floor (set down, not held above it), tilted at most 10 degrees, its front face at most 10 mm behind the stop lip's inner face and no part of it more than 1 mm in front of that face (not up on the lip) |
| 0 overall | any part of the red bin's outline seen from above over the lane's outline, walls and stop lip included, at any height (left in the lane, perched on a side or back wall, put back behind the blue bin, or stacked on it) |
| 0 overall | the blue bin's centre over the shelf's top (the full bin sent for reorder) |
| 0 overall | either bin's bottom more than 1 cm below the table top (fallen off the table) |

On a real table, a ruler checks the gap between the blue bin's front face and the stop lip and that each bin stands
on its surface (released, no gap under it), and a phone level app each bin's tilt; a photo from above shows that the
red bin's outline is inside the shelf top, that no part of it is over the lane or its walls, and that the blue bin's
centre is not over the shelf.

## The scripted demo

The left arm grips the red bin across its 7 cm width, the fingertips 9 mm above the lane walls, lifts it out of the
lane, carries it to the shelf with its bottom 2.8 cm above the shelf top, sets it down in the middle of the shelf,
letting go 6 mm above it, lifts clear and parks to the left of the shelf. The right arm waits off to the right of
the lane for 12 s, until the left arm has parked, then grips the blue bin the same way, lifts it 2.5 cm off the lane
floor (clear of the 1.5 cm stop lip), carries it 10.8 cm along the lane and sets it down 3 mm behind the stop,
letting go 6 mm above the floor, then lifts clear and parks in front of its own base. The demo lifts the bin a little
rather than sliding it: with the simulator's rubber-like friction (1.0 dynamic) a 500 g bin takes about 5 N to drag
along the floor. The scorer looks only at where the bins end up, so a policy may slide the bin instead.

## Runtime and seeds

- The demo's plan takes 22.9 s of simulated time. The last bin is let go at 20.4 s, so with the 2.5 s hold the demo
  ends in success after about 470 steps at 20 Hz, well within the 1,200-step limit. On the official stack, the demo
  reached success at step 458 and scored 1.0. That score also shows the 2 mm seated check holds in
  physics: success requires both bins released and resting within 2 mm of their surfaces.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
