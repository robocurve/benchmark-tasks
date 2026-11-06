# Lift the two marked floor tiles and stand a cone in each open hole

A small raised-floor frame holds four tiles; two are orange, marked for lifting. The robot has to lift exactly those
two, store them flat on a staging pad outside the aisle, and stand a cone in each hole it opens, without disturbing
the green tiles. An open hole with no cone in it, a green tile pulled or shoved, or a lifted tile left in the aisle
scores 0. The pad is within reach of the left arm only and the cones of the right arm only, and the marked tiles sit
on opposite sides of the midline, so each hole needs both arms and two objects cross the midline.

Technicians lift floor panels with suction lifters; here the robot grips a tile by its edges through the 2.5 cm
finger slots around it, a stand-in for the lifter. The yellow cylinder stands in for the floor cone or barrier post
set in an open panel: like those it guards the hole by standing in it, and whether it still stands is a tilt a level
reads.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=raised_floor_open_tiles_coned.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lift the two orange floor tiles out of the frame and lay them flat on the blue pad on the left, then stand a yellow cone upright in each of the two holes they leave. Leave the green tiles in place." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/floor-tiles-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=raised_floor_open_tiles_coned.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lift the two orange floor tiles out of the frame and lay them flat on the blue pad on the left, then stand a yellow cone upright in each of the two holes they leave. Leave the green tiles in place." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/floor-tiles-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/floor-tiles-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once both holes are coned, both orange tiles are on the
pad and both green tiles are seated, with every tile and cone still (under 1 cm/s) for 2.5 s; otherwise it scores the
partial credit below at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_raised_floor_open_tiles_coned.py` in an
environment with `yam_arena` installed.

## Scoring

Six points, each worth 1/6. Only the end state is scored; a hole may stand open while the work is in progress.
Definitions used below:

- A **tilt** is what a level on the object's top face reads. The tiles and cones are symmetric, so a cone stood on
  either end or a tile lying either face up counts the same.
- A **height** is that of the object's centre: its top's height minus half its height (1.5 cm for a tile, 4 cm for a
  cone).
- A tile **lies in** a pocket when its centre is over the pocket's 12 cm square, its centre is 1.5 cm above the pocket
  floor (the table top) to within 3 mm, and it is tilted at most 3 degrees. It is **seated** when it also has its
  centre within 8 mm of the pocket's centre.
- A cone **guards** a pocket when it is tilted at most 10 degrees, its centre is 4 cm above the pocket floor (the table
  top) to within 3 mm, and its centre is over the pocket's 12 cm square.
- The **aisle** is the frame's outline grown 5 cm on every side: 11 to 50 cm in front of the arm bases and up to
  19.5 cm either side of the midline. The pad's inner edge is 5.5 cm outside it.

| Credit | For |
|---|---|
| 1 point | each orange tile's pocket with no tile's centre over its square (at any height) and a cone guarding it |
| 1 point | each orange tile flat on the pad, counted only if its own pocket earned the point above: tilted at most 5 degrees, its centre over the pad's 18 x 11 cm top, its centre 1.5 cm above the pad's top to within 3 mm (or 3 cm above the other orange tile's centre to within 3 mm while that tile lies flat on the pad) |
| 1 point | each green tile seated in its own pocket, counted only if at least one orange tile's pocket earned its cone point |
| 0 overall | a pocket with no tile lying in it and no cone guarding it (an open, unguarded hole) |
| 0 overall | a green tile that doesn't lie in its own pocket (the wrong panel pulled, or shoved up onto a web) |
| 0 overall | an orange tile that doesn't lie in its own pocket with its centre in the aisle (a loose panel on the walkway) |
| 0 overall | any tile or cone with its centre below the table top (knocked off the table) |

The untouched start scores 0: no pocket is coned, so the green tiles earn nothing. Lifting one orange tile onto the
pad and coning its hole, with the other orange tile left in place, scores 4/6. A ruler checks that a cone's centre is
over its pocket's square and a tile's offset in its pocket, a phone level app the tilts, and a photo from above
shows which pockets hold what and that the staged tiles' centres are over the pad.

## The scripted demo

Every grip is straight down with the fingers closing across the robot (world y); for a tile the fingers open to 84 mm
and reach into the finger slots, 7 mm clear of the tile and 8 mm clear of the pocket wall on each side. The left arm
lifts the near-left orange tile onto the pad, 4 cm nearer the arms than the pad's centre. Meanwhile the right arm
lifts the far-right orange tile and sets it on the bare table at the handover spot on the midline, 11.5 cm in front of
the arm bases (its centre 4.5 cm behind the frame's near edge), then fetches the farther cone to the same spot. The left arm takes the tile
from the handover spot to the pad, 4 cm beyond the pad's centre, then takes the cone from the handover spot and stands
it in the near-left hole, while the right arm stands the nearer cone in the far-right hole. Each object is let go 6 mm
above where it lands; both arms then park beside the work area so the camera sees the result.

## Runtime and seeds

- The demo's plan takes about 31 s of simulated time (the last cone is let go at 27.8 s), well within the 1,800-step
  limit with the 2.5 s hold.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
