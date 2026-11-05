# Shelve the artifacts so no object stands behind a taller one

Five artifacts of different heights go from a cart onto a storage shelf that is too narrow for one row, so they have
to be set out in depth. Collections-storage practice puts small objects in front of larger ones, so every object can
be seen from the aisle and none has to be lifted over another, and leaves space between objects and the shelf edge.
The cart is within reach of the left arm only and the shelf of the right arm only, so every artifact is handed across
on a pad between the arms.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
INSTRUCTION="Move the blue vase, green figurine, yellow bowl, orange pot and purple bead box from the grey cart on the left onto the wooden shelf on the right, with taller objects behind shorter ones so none is hidden. Leave at least 2 cm between objects and keep each at least 2 cm back from the shelf's front edge, the edge nearest you."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=shelve_artifacts_short_in_front.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/shelve-artifacts-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=shelve_artifacts_short_in_front.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTRUCTION" \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/shelve-artifacts-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/shelve-artifacts-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once all five artifacts are shelved, set back, spaced
and none hidden, and stay still (under 1 cm/s and 0.5 rad/s) for 2.5 s; otherwise it scores the partial credit below at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_shelve_artifacts_short_in_front.py` in an
environment with `yam_arena` installed.

## Scoring

An artifact is **shelved** when it stands upright (axis tilted at most 15 degrees), has been set down with its bottom
within 3 mm of the shelf top (not on another artifact, and not held above it), and its whole footprint is on the
usable top: between the front edge, the back lip's face and the two side ends.

| Credit | For |
|---|---|
| 0.2 | each artifact shelved with its footprint at least 2 cm behind the front edge |
| 0.1 | instead of 0.2: shelved, but less than 2 cm behind the front edge |
| -0.05 | each pair of shelved artifacts less than 2 cm apart (the gap between their outlines, each taken as the rectangle around it squared to the shelf edges); the total is floored at 0 |
| 0 overall | any artifact on the shelf hidden behind another on the shelf (below) |

An artifact A is **hidden** behind B when all three hold:

- B is taller standing, by 5 mm or more (vase 12 cm, figurine 10, bowl 6, pot 5, bead box 4; any two differ by at
  least 1 cm, so any taller artifact counts);
- B's back face (the side of its footprint farthest from the front edge) is nearer the front edge than A's centre,
  so more than half of A stands behind B;
- along the edge, their centres are less than (width of A + width of B) / 2 + 2 cm apart (widths: vase 6 cm,
  figurine 5, bowl 7, pot 5, bead box 4).

Worked example: the pot centred 2 cm along the edge from the bead box (less than 5/2 + 4/2 + 2 = 6.5 cm) and 7 cm
nearer the front has its back face 4.5 cm in front of the bead box's centre, so it hides the bead box and the trial
scores 0. Placed side by side 8 cm apart along the edge, they are in different lanes and neither hides the other.

Side-by-side neighbours in a row that are staggered a little in depth don't hide each other: with the bowl 1.3 cm
further forward than the pot and 1.5 cm from it along the edge, the bowl's back face is 2.2 cm behind the pot's
centre, so the pot is not hidden; the pair only costs 0.05 for being less than 2 cm apart (score 0.95).

"On the shelf" for hiding means the artifact's centre is over the usable top and it has been set down: its bottom is
within 3 mm of the shelf top (standing or lying), or within 3 mm of the top of another artifact on the shelf with its
centre over that artifact's footprint (stacked). An artifact still held higher above the shelf hides nothing until it
is set down. A toppled artifact earns no credit, and still hides others at its standing height, so knocking the vase
flat does not clear its lane. Artifacts left in the cart, on the pad, on the table or on the floor earn nothing.

On a real table, a tape measure gives each centre's distance from the front edge and along it, and each footprint's
distance from the front edge and from its neighbours; a phone level app checks the tilt.

Why two rows are forced: the five widths add up to 6 + 5 + 7 + 5 + 4 = 27 cm, more than the shelf's 26 cm, and in
separate lanes they would need 27 + 4 x 2 = 35 cm. So at least two artifacts always share a lane, and since no two
are the same height, the taller of them must not stand in front. The height rule always binds.

## The scripted demo

The left arm empties the cart one artifact at a time, figurine, vase, bowl, pot, bead box, gripping each from above
across its width and setting it down on the transfer pad; it holds the next artifact above the cart until the right
arm has lifted the last one off the pad, then parks. The right arm picks each artifact up from the pad and stands it
on the shelf, back row first: the figurine and the vase with their centres 17.5 cm behind the front edge, 33 and 23 cm
right of centre; then the bowl, the pot and the bead box with their centres 6.5 cm behind the front edge, 22, 31 and
39 cm right of centre. It lets go of each 8 mm above the shelf, lifts clear, and parks between the arms so the camera
sees the shelf. For the front row the right arm closes its fingers front to back, so they fit between neighbours.
The smallest gaps are 3 cm (bowl to pot) and 3.5 cm (pot to bead box); the front row sits 3 cm or more behind the
front edge.

## Runtime and seeds

- The demo's plan takes 64.1 s of simulated time. In the recorded run (demo.mp4) the trial ended with success,
  including the 2.5 s hold, at step 1,249 (62.45 s at 20 Hz), well within the 2,400-step limit. The scripted policy
  would stop 3 s after the plan ends.
- The Dream-1 run used all 2,400 steps and scored 0.0: the green figurine reached the shelf but ended lying on its
  side against the lip, which earns nothing.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
