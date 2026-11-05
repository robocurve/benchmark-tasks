# Put each parcel in the smallest compartment it fits

Four parcels (two small, one medium, one large) go into a tray with exactly one compartment of each matching size:
two small, one medium, one large. Every parcel fits the large compartment and the small ones fit the medium one too,
so dropping a parcel into the nearest or biggest compartment that will take it leaves a later parcel with nowhere
to go. The parcels all look alike apart from their size, so the robot has to compare each parcel's size with the
compartments'. The parcels start on the left arm's side and the tray is on the right arm's side, so every parcel
is handed over on a mat in the middle.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=smallest_cubby_that_fits.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Move the four brown parcels from the blue mat on the left into the grey tray on the right, each into the smallest compartment it fits in, one parcel per compartment." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/smallest-cubby-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=smallest_cubby_that_fits.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Move the four brown parcels from the blue mat on the left into the grey tray on the right, each into the smallest compartment it fits in, one parcel per compartment." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/smallest-cubby-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/smallest-cubby-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once all four parcels are seated in their
compartments and stay still for 2.5 s; otherwise it scores the partial credit below at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_smallest_cubby_that_fits.py` in an
environment with `yam_arena` installed.

## Scoring

A parcel is *in* a compartment when its centre, seen from above, is inside that compartment's inner outline (8 cm
front to back, and 12.5, 7.5, 5.5 or 5.5 cm side to side) and either its bottom is less than 3 cm above the
compartment floor (below the rim) or it rests on top of another parcel whose centre is inside the same outline (its
bottom within 3 mm of that parcel's top, and moving under 1 cm/s). A parcel carried higher over the tray, on its way
to another compartment, and not resting on another parcel is in no compartment. It is *seated* there when, in
addition, its bottom is within 5 mm of the compartment floor and one of its faces (any face: bottom, top or side) is
tilted at most 10 degrees from horizontal.

| Credit | For |
|---|---|
| 0.25 | each parcel seated alone in a compartment of the smallest size it fits: a small parcel in either small compartment, the medium parcel in the medium compartment, the large parcel in the large compartment |
| 0 for that parcel | a parcel in a compartment that holds another parcel (two parcels in one compartment as defined above, stacked or side by side), not seated (no face within 10 degrees of horizontal, or not on the floor), or outside the tray |
| 0 overall | any parcel in a compartment larger than the smallest it fits: a small parcel in the medium or large compartment, or the medium parcel in the large one |
| 0 overall | any parcel fallen off the table (its bottom more than 5 cm below the table top) |

All four seated: 1.0. Which parcel fits which compartment follows from the sizes, on whichever face a parcel stands:
the small parcels (4.5 x 4.5 x 6 cm) fit every compartment; the medium parcel (6 x 6 x 6.5 cm) is at least 6 cm
across on any face, too wide for the 5.5 cm small compartments, so it fits only the medium and large ones; the large
parcel (7 x 11 x 8.5 cm) is at least 7 x 8.5 cm on any face, too big for the 7.5 x 8 cm medium compartment, so it
fits only the large one.

On a real table, a photo from above shows which outline each parcel's centre is in; a ruler checks that a parcel's
bottom is on the compartment floor (its top as high above the floor as the parcel measures on that face), and a
phone level app laid on the parcel's top face reads the tilt. A parcel resting on another parcel touches it; one
held in the gripper above it is in no compartment until let go.

## The scripted demo

Both arms grip straight down, with the fingers closing front to back (the left arm's grip turned -90 degrees, the
right arm's +90 degrees), gripping each parcel 1.5 cm below its top.
The left arm takes the parcels in the order large, medium, small, small from the blue mat to the centre of the
green mat, letting go 8 mm above it. After each handover the right arm takes the parcel from the green mat to the
centre of its compartment (the small parcels to the inner small compartment first, then the outer one), lets go 8
mm above the floor and lifts straight up. While the right arm places one parcel, the left arm picks up the next and
holds it above its starting mark for 2.5 s until the green mat is clear. At the end the left arm parks above the
empty blue mat and the right arm 15 cm in front of the arm bases and 36 cm right of centre, nearer the arms than
the tray, so the camera sees the sorted tray.

## Runtime and seeds

- The demo's plan takes about 58 s of simulated time (the right arm finishes last), so with the 2.5 s hold the
  scripted trial should end in success after about 1,200 steps at 20 Hz, well within the 2,400-step limit.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
