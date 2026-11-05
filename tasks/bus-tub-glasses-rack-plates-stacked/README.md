# Clear a bussing tray: glasses into the glass rack, plates stacked largest first in the bus tub

A serving tray comes back from the dining room with two glasses and three plates of different sizes on it. The
dish-pit rule: the glasses go into the glass rack, one per cell and upright, never into the bus tub; the plates go
into the bus tub in one stack, largest at the bottom, so small plates aren't buried and chipped under large ones. The
rack is in front of the left arm and the tub in front of the right, while the glasses start on the tray's right half
and the plates on its left half, out of reach of the arm that has to put them away. Every item has to be passed
across the midline, so a full score takes both arms.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=bus_tub_glasses_rack_plates_stacked.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Clear the black tray: stand each blue glass upright in its own cell of the green rack on the left, and stack the three orange plates in the grey tub on the right, the largest at the bottom and the smallest on top." \
  --scorer task_score --max-steps 3000 --store-frames --log-dir ~/yam-eval/bus-tub-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=bus_tub_glasses_rack_plates_stacked.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Clear the black tray: stand each blue glass upright in its own cell of the green rack on the left, and stack the three orange plates in the grey tub on the right, the largest at the bottom and the smallest on top." \
  --scorer task_score --max-steps 3000 --store-frames --log-dir ~/yam-eval/bus-tub-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/bus-tub-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once both glasses are racked and the three plates are
stacked in order in the tub, all still, for 2.5 s; otherwise it scores the partial credit below at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_bus_tub_glasses_rack_plates_stacked.py` in
an environment with `yam_arena` installed.

## Scoring

| Credit | For |
|---|---|
| 0.2 | each glass racked: its centre inside one of the rack's four 8 cm cells, tilted at most 10 degrees, its bottom within 1 cm of the rack's base, and no other glass with its centre in that cell |
| 0.2 | the large plate at the bottom of the stack: its centre inside the tub's inner outline (or within 5 mm past it), tilted at most 10 degrees, its bottom within 1 cm of the tub floor |
| 0.2 | then the medium plate on it: in the tub and tilted at most 10 degrees the same way, its bottom within 1 cm of the large plate's top, its centre within 2 cm sideways of the large plate's centre |
| 0.2 | then the small plate on the medium plate, the same way |
| 0 overall | a glass with its centre inside the tub's outer outline; a plate with its centre inside the rack's outer outline; a larger plate resting on a smaller one, flat on it or tilted onto it with an edge lower down (seen from above, the two outlines overlap: centres closer than the sum of their radii; the larger plate's top is more than 1 cm above the smaller plate's top; and the larger plate's lowest point is at most 1 cm above the smaller plate's top; a plate held higher in the gripper doesn't count); a glass or plate tilted more than 45 degrees anywhere, or fallen off the table (its top below the table surface) |

The plates are counted from the bottom up and the count stops at the first plate that fails, so a medium plate on the
tub floor earns nothing for the plates. On a real table, a ruler checks each plate's height above the tub floor (0,
3 and 6 cm with these plates) and its offset from the plate under it, a phone level app checks tilt, and a photo
from above shows which cell each glass is in.

## The scripted demo

Each arm only works on its own side and at a pass mark on the tray's midline, 22 cm in front of the arm bases, and
only one arm is at the pass mark at a time. The right arm picks each glass from above, gripping it 5.7 to 9 cm up its
side, and stands it on the pass mark; the left arm picks it up there and lowers it into a rack cell (the far inner
cell first, then the near inner cell), letting go 8 mm above the rack's base with its fingertips above the cell walls.
The left arm then brings the plates to the pass mark one at a time, large, medium, small, gripping each across its
width; the right arm, waiting over the tub, takes each one and sets it down at the tub's centre, the large plate on
the floor and each next plate on the one before, letting go 8 mm above where it lands. Each arm lifts straight up to
the 15 cm travel height after letting go, and both end parked in front of their bases so the camera sees the rack and
the tub.

## Runtime and seeds

- The demo's plan takes 70.1 s of simulated time; its last release is at 67.6 s, so with the 2.5 s hold it should
  end in success after about 1,420 steps at 20 Hz, within the 3,000-step limit. On an L40S, once Isaac Sim has
  started, a scripted trial should take about 2 minutes of wall time and a full Dream-1 trial 5 to 7 minutes.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
