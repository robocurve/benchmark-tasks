# Load glazed pots into a kiln with no piece touching another, a post or the wall

Five glazed pots of different sizes go into a small open-top kiln with three fixed shelf posts, one in the middle of
the back and one in each front corner. The kiln's floor is a tan shelf, a 5 mm tan-painted wood board; its walls are
grey-painted wood; the posts are solid wooden dowels painted dark grey, 2 cm across and 10 cm tall. Melted glaze welds whatever it touches, so every pot needs 2 cm of clear air
to every other pot and post and 1.5 cm to every wall. In one row the pots would need 29 cm of pot, four 2 cm gaps and
two 1.5 cm wall bands, 40 cm in all, but the kiln is 33 cm wide inside: the robot has to plan two rows around the
posts. The left arm reaches the green and blue pots and the right arm the yellow, red and purple ones, and both load
the same small interior, so a full score takes both arms.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=kiln_load_glazed_pots_no_contact.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Load all five glazed pots standing upright into the kiln. Glaze fuses on contact, so leave at least 2 cm between any two pots and between a pot and a dark grey post, and at least 1.5 cm between a pot and the kiln wall." \
  --scorer task_score --max-steps 2000 --store-frames --log-dir ~/yam-eval/kiln-load-dream

# The scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=kiln_load_glazed_pots_no_contact.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Load all five glazed pots standing upright into the kiln. Glaze fuses on contact, so leave at least 2 cm between any two pots and between a pot and a dark grey post, and at least 1.5 cm between a pot and the kiln wall." \
  --scorer task_score --max-steps 2000 --store-frames --log-dir ~/yam-eval/kiln-load-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/kiln-load-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once all five pots are loaded cleanly and still for
2.5 s; otherwise it scores the partial credit below at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_kiln_load_glazed_pots_no_contact.py` in an
environment with `yam_arena` installed.

## Scoring

Gaps from an upright pot to a post, and between two upright pots, are measured surface to surface from above: the
distance between centres minus both radii (the posts are 10 mm in radius). When the other pot is tilted more than 5
degrees or lying, the gap is the shortest distance between the two pots' actual surfaces, so a lying pot's end counts
as well as its side. Gaps to walls run from the pot's side to the wall's inner face, square to the wall. "At its
level" means the two pots' height ranges overlap or are within 2 mm: a pot resting on another counts, one held more
than 2 mm above it doesn't. "Upright" means the pot's axis is within 5 degrees of vertical, either end up: the pots
are closed cylinders, the same at both ends. A pot's bottom, top and outermost points are those of the actual
cylinder: a pot of radius R and height H tilted t degrees has its bottom H/2 x cos t + R x sin t below its centre. (The
scorer works them out from the pot's centre, tilt and size, not from the simulator's bounding box, which is padded by
about 5 mm.)

| Credit | For |
|---|---|
| 0.2 | each pot loaded cleanly: its bottom within 3 mm of the shelf top, upright (axis within 5 degrees of vertical, either end up), at least 15 mm from every wall, at least 20 mm from every post, and at least 20 mm from every other pot at its level |
| 0 for both | two pots at the same level less than 20 mm apart: neither earns its 0.2 |
| 0 overall | any pot with its centre inside the kiln's inner outline touching (closer than 2 mm, surface to surface) another pot, a post (its side or its flat top) or a wall (its inner face or its top edge) |

Contact is measured between the actual surfaces. For a tilted or toppled pot it is taken from its actual outline
(its side, ends and rims), not its upright radius: a pot lying with its end against a post, or leaning with its rim
on a post or another pot, is touching. For two upright pots side by side it is the centre distance minus both radii.
Each post is a solid cylinder with a flat top, so a pot held 5 mm above a post's top is not touching it. To a wall,
contact runs from the pot's outermost point to the wall's inner face, or, for a pot above the walls, to their top
edge: a pot resting on a wall's top, or held less than 2 mm above it, is touching.

Full score is all five pots loaded cleanly: 1.0. A pot left outside the kiln, lying on its side, held above the shelf
or standing on another pot earns nothing; one standing on another inside the kiln also zeroes the score. On a real
table, a feeler gauge (or a 2 mm card) shows any contact, and a top-down
photo with a ruler gives every centre distance, a ruler on the shelf the pots' bottoms, and a phone level app their
tilt.

## The scripted demo

The left arm picks up the green pot from above, gripping its sides with the fingers closing across the kiln, and
stands it in the front left of the kiln (centre 20 cm in front of the arm bases and 10 cm left of the midline: 25 mm
from the front wall, 29 mm from the front-left post), letting go 8 mm above the shelf. It does the same with the
blue pot at the back left (29.5 cm in front, 9.5 cm left: 25 mm from the back wall, 30 mm from green), then parks
over green's empty mark. The right arm waits until then above the yellow pot, and loads yellow at the front right
(20.5 cm in front, 10 cm right), red at the back right (29.5 cm in front, 9.5 cm right) and purple at the front centre
(20.5 cm in front, on the midline, 40 mm from green and 45 mm from yellow), turning its grip on the way so the
fingers close front to back: purple's side neighbours stand taller than its fingers reach. After every release the
gripper rises straight up before moving on, and the right arm parks over yellow's empty mark so the camera sees the
loaded kiln. Every gap in the demo's layout is at least 9 mm above its threshold (the tightest is green to the
front-left post, 29.3 mm against 20 mm).

## Runtime and seeds

- The demo's plan takes 51.3 s of simulated time (the right arm's plan; the left arm's takes 20.7 s). The last pot is
  let go at 48.8 s; in the recorded run the pots were loaded cleanly and still for the 2.5 s hold by 51.3 s, so the
  trial ended on success at 51.3 s (1,026 steps at 20 Hz), within the 2,000-step limit. Without that early success
  stop, the scripted policy would stop at 54.3 s (about 1,090 steps), 3 s after its plan ends. The score the demo
  reaches in the simulator is recorded in `task.json`'s `scores`.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
